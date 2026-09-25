#include "dap_trace.h"

#include <inttypes.h>
#include <string.h>

#include "dap_probe.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "DAP_TRACE";

/* miniMCDS trace FIFO registers. */
#define TRACE_MCDS_BASE     0xFB718000u
#define TRACE_FIFONOW       (TRACE_MCDS_BASE + 0x0200u)
#define TRACE_FIFOBOT       (TRACE_MCDS_BASE + 0x0204u)
#define TRACE_FIFOTOP       (TRACE_MCDS_BASE + 0x020Cu)
#define TRACE_FIFOCTL       (TRACE_MCDS_BASE + 0x0210u)
#define TRACE_FIFOWARN0     (TRACE_MCDS_BASE + 0x0214u)
#define TRACE_FIFOWARN1     (TRACE_MCDS_BASE + 0x0218u)
#define TRACE_FIFOOVRCNT    (TRACE_MCDS_BASE + 0x021Cu)

#define TRACE_TRAM_BASE     0xB8000000u
#define TRACE_PARAGRAPH     0x400u          /* 1 kB, the framing unit */
#define TRACE_WORDS_PER_PAR (TRACE_PARAGRAPH / 4u)

/* Pointer registers hold a TRAM offset in bits [12:5]. */
#define TRACE_PTR_MASK      0x1FE0u

/* Output ring: 1 MB in PSRAM is ~0.5 s of slack at 2 MB/s against WiFi stalls;
 * internal RAM fallback is small. */
#define TRACE_RING_PSRAM    (1024u * 1024u)
#define TRACE_RING_INTERNAL (64u * 1024u)
#define TRACE_RING_BYTES    s_ring_bytes

/* Paragraphs kept between the writer and the oldest one read (see drain_to). */
#define TRACE_MARGIN        3u

/* FIFOOVRCNT only feeds a statistic; read it every n-th poll. */
#define TRACE_OVRCNT_EVERY  16u

static uint8_t          *s_ring;
static size_t            s_ring_bytes;
static size_t            s_ring_head;      /* next write */
static size_t            s_ring_tail;      /* next read */
static SemaphoreHandle_t s_lock;

static bool     s_running;
static uint32_t s_bot, s_top;              /* FIFO bounds, byte offsets */
static uint32_t s_span;                    /* top - bot + 1 */
static uint32_t s_next_par;                /* paragraph index we expect next */
static uint32_t s_last_ovrcnt;
static uint32_t s_pending_lost;            /* lost paragraphs not yet flagged */
static uint32_t s_seq;
static dap_trace_stats_t s_stats;
static TaskHandle_t s_task;

static void trace_task(void *arg);

static size_t ring_used(void)
{
    return (s_ring_head >= s_ring_tail)
         ? (s_ring_head - s_ring_tail)
         : (TRACE_RING_BYTES - s_ring_tail + s_ring_head);
}

static size_t ring_free(void)
{
    /* One byte held back so full and empty stay distinguishable. */
    return TRACE_RING_BYTES - 1u - ring_used();
}

static void ring_put(const uint8_t *src, size_t len)
{
    const size_t first = (len < TRACE_RING_BYTES - s_ring_head)
                       ? len : TRACE_RING_BYTES - s_ring_head;
    memcpy(s_ring + s_ring_head, src, first);
    memcpy(s_ring, src + first, len - first);
    s_ring_head = (s_ring_head + len) % TRACE_RING_BYTES;
}

/* Publish one paragraph, or drop it whole (and count it) if the ring is full. */
static void publish(uint32_t par_index, const uint32_t *words, uint32_t lost)
{
    const dap_trace_record_t hdr = {
        .magic       = DAP_TRACE_MAGIC,
        .seq         = s_seq++,
        .tram_offset = s_bot + par_index * TRACE_PARAGRAPH,
        .length      = (uint16_t)TRACE_PARAGRAPH,
        .flags       = (uint16_t)(lost ? DAP_TRACE_FLAG_GAP : 0u),
        .lost        = lost,
    };

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (ring_free() >= sizeof(hdr) + TRACE_PARAGRAPH) {
        ring_put((const uint8_t *)&hdr, sizeof(hdr));
        ring_put((const uint8_t *)words, TRACE_PARAGRAPH);
        s_stats.paragraphs++;
        s_stats.bytes += TRACE_PARAGRAPH;
    } else {
        s_stats.queue_dropped += TRACE_PARAGRAPH;
    }
    s_stats.queue_free = (uint32_t)ring_free();
    xSemaphoreGive(s_lock);
}

/* dap_trace_start() without the drain task, so the self-test can poll from its
 * own thread (DAP access is not locked). */
static esp_err_t trace_begin(void);

esp_err_t dap_trace_start(void)
{
    const esp_err_t err = trace_begin();
    if (err != ESP_OK) {
        return err;
    }
    /* Core 0: WiFi and lwIP are pinned to core 1.  The 8 kB TRAM fills in
     * ~5 ms at 1.8 MB/s, far below one 10 ms tick, so the drain busy-polls
     * above the other core-0 tasks; IDLE0 starves meanwhile. */
    esp_task_wdt_delete(xTaskGetIdleTaskHandleForCore(0));
    if (s_task == NULL &&
        xTaskCreatePinnedToCore(trace_task, "dap_trace", 4096, NULL, 12, &s_task, 0) != pdPASS) {
        s_running = false;
        ESP_LOGE(TAG, "could not start the drain task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t trace_begin(void)
{
    if (s_ring == NULL) {
        s_ring_bytes = TRACE_RING_PSRAM;
        s_ring = heap_caps_malloc(s_ring_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_ring == NULL) {
            s_ring_bytes = TRACE_RING_INTERNAL;
            s_ring = heap_caps_malloc(s_ring_bytes, MALLOC_CAP_8BIT);
        }
        if (s_ring == NULL) {
            ESP_LOGE(TAG, "no room for a %u byte ring", (unsigned)s_ring_bytes);
            return ESP_ERR_NO_MEM;
        }
    }
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    uint32_t bot = 0, top = 0, now = 0, ovr = 0;
    if (dap_probe_read32(TRACE_FIFOBOT, &bot) != ESP_OK ||
        dap_probe_read32(TRACE_FIFOTOP, &top) != ESP_OK ||
        dap_probe_read32(TRACE_FIFONOW, &now) != ESP_OK ||
        dap_probe_read32(TRACE_FIFOOVRCNT, &ovr) != ESP_OK) {
        ESP_LOGE(TAG, "trace FIFO is not readable - is OCDS enabled?");
        return ESP_ERR_INVALID_STATE;
    }
    if (top <= bot || ((top - bot + 1u) % TRACE_PARAGRAPH) != 0u) {
        ESP_LOGE(TAG, "FIFOBOT 0x%08" PRIX32 " / FIFOTOP 0x%08" PRIX32
                      " are not a whole number of paragraphs", bot, top);
        return ESP_ERR_INVALID_STATE;
    }

    /* Zero the warning comparators so the FIFO never halts at a threshold. */
    dap_probe_write32(TRACE_FIFOWARN0, 0);
    dap_probe_write32(TRACE_FIFOWARN1, 0);

    s_bot  = bot;
    s_top  = top;
    s_span = top - bot + 1u;

    /* Start behind the pointer's own paragraph: that one is still being written. */
    const uint32_t now_par = ((now - bot) % s_span) / TRACE_PARAGRAPH;
    s_next_par     = now_par;
    s_last_ovrcnt  = ovr;
    s_pending_lost = 0;
    s_seq          = 0;
    s_ring_head    = 0;
    s_ring_tail    = 0;

    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.queue_free = TRACE_RING_BYTES - 1u;
    s_stats.fifonow    = now;
    s_running          = true;

    ESP_LOGI(TAG, "draining %" PRIu32 " paragraphs of %u bytes from 0x%08" PRIX32
                  ", FIFONOW 0x%08" PRIX32 " (paragraph %" PRIu32 ")",
             s_span / TRACE_PARAGRAPH, (unsigned)TRACE_PARAGRAPH,
             TRACE_TRAM_BASE + bot, now, now_par);
    return ESP_OK;
}

void dap_trace_stop(void)
{
    s_running = false;
    s_stats.running = false;
    /* The task leaves after its current pass; do not hold the DAP lock here. */
    for (int i = 0; i < 100 && s_task != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static esp_err_t drain_to(uint32_t now, uint32_t ovr);

esp_err_t dap_trace_finish(void)
{
    static uint32_t words[TRACE_WORDS_PER_PAR];
    uint32_t now = 0;

    if (s_ring == NULL || s_span == 0u) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dap_probe_read32(TRACE_FIFONOW, &now) != ESP_OK) {
        return ESP_ERR_TIMEOUT;
    }
    now &= TRACE_PTR_MASK;
    s_running = true;
    drain_to(now, s_last_ovrcnt);              /* complete paragraphs */
    s_running = false;

    /* The paragraph NOW points into ends with <endoftrace>. */
    const uint32_t par = ((now - s_bot) % s_span) / TRACE_PARAGRAPH;
    if (dap_probe_blockread(TRACE_TRAM_BASE + s_bot + par * TRACE_PARAGRAPH, words,
                            TRACE_WORDS_PER_PAR) != ESP_OK) {
        return ESP_ERR_TIMEOUT;
    }
    publish(par, words, 0);
    s_stats.running = false;
    return ESP_OK;
}

/* A drain pass for a given write pointer; FIFONOW is not DAP-writable, so the
 * self-test passes its own. */
static esp_err_t drain_to(uint32_t now, uint32_t ovr)
{
    static uint32_t words[TRACE_WORDS_PER_PAR];

    const int64_t t0 = esp_timer_get_time();

    if (ovr != s_last_ovrcnt) {
        /* Observation-unit overflow: the stream has an ERR message, so no gap record. */
        s_stats.overruns += (ovr - s_last_ovrcnt);
        s_last_ovrcnt = ovr;
    }

    const uint32_t total_par = s_span / TRACE_PARAGRAPH;
    const uint32_t now_par   = ((now - s_bot) % s_span) / TRACE_PARAGRAPH;

    /* Complete paragraphs waiting, excluding the one still being written. */
    uint32_t available = (now_par + total_par - s_next_par) % total_par;

    /* Lapped (no ERR for this): everything unread is lost; resume at the
     * pointer's paragraph and flag a gap on the next record published. */
    uint32_t lost = s_pending_lost;
    if (available >= total_par - 1u && available != 0u) {
        const uint32_t n = available;           /* all skipped: the oldest is torn */
        lost += n;
        s_stats.laps++;
        s_stats.lost += n;
        s_next_par = now_par;
        available  = 0;
        ESP_LOGW(TAG, "TRAM lapped: %" PRIu32 " paragraphs lost, resuming at %" PRIu32,
                 lost, now_par);
    }

    /* Behind the writer: drop the oldest paragraphs (the ones it overwrites
     * next) so at least TRACE_MARGIN separate it from the first one read. */
    if (available + TRACE_MARGIN > total_par) {
        const uint32_t n = available + TRACE_MARGIN - total_par;
        lost += n;
        s_stats.lost += n;
        s_stats.laps++;
        s_next_par = (s_next_par + n) % total_par;
        available -= n;
    }

    /* The writer must cross `margin` paragraphs before it reaches the oldest
     * unread one; each read paragraph shifts that by one. */
    const uint32_t to_read = available;
    const uint32_t margin  = total_par - available;
    for (uint32_t i = 0; i < to_read; i++) {
        const uint32_t addr = TRACE_TRAM_BASE + s_bot + s_next_par * TRACE_PARAGRAPH;

        if (dap_probe_blockread(addr, words, TRACE_WORDS_PER_PAR) != ESP_OK) {
            s_stats.read_errors++;
            dap_probe_clear_error_state();
            s_pending_lost = lost;
            return ESP_ERR_TIMEOUT;
        }

        /* Torn if the writer reached this paragraph while it was read. */
        uint32_t w = now;
        if (dap_probe_read32(TRACE_FIFONOW, &w) == ESP_OK) {
            const uint32_t w_par = (((w & TRACE_PTR_MASK) - s_bot) % s_span) / TRACE_PARAGRAPH;
            const uint32_t moved = (w_par + total_par - now_par) % total_par;
            if (moved >= margin + i) {
                const uint32_t n = to_read - i;
                lost += n;
                s_stats.laps++;
                s_stats.lost += n;
                s_next_par = w_par;
                break;
            }
        }
        publish(s_next_par, words, lost);
        lost = 0;                       /* the marker belongs to one record only */
        s_next_par = (s_next_par + 1u) % total_par;
    }
    s_pending_lost = lost;

    const uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
    if (us > s_stats.poll_us_max) {
        s_stats.poll_us_max = us;
    }
    s_stats.fifonow = now;
    s_stats.running = true;
    return to_read ? ESP_OK : ESP_ERR_NOT_FOUND;    /* NOT_FOUND: nothing waiting */
}

esp_err_t dap_trace_poll(void)
{
    if (!s_running) {
        return ESP_OK;
    }

    static uint32_t polls;
    uint32_t now = 0, ovr = s_last_ovrcnt;
    if (dap_probe_read32(TRACE_FIFONOW, &now) != ESP_OK) {
        s_stats.read_errors++;
        dap_probe_clear_error_state();
        return ESP_ERR_TIMEOUT;
    }
    if ((polls++ % TRACE_OVRCNT_EVERY) == 0u &&
        dap_probe_read32(TRACE_FIFOOVRCNT, &ovr) != ESP_OK) {
        ovr = s_last_ovrcnt;
    }
    return drain_to(now & TRACE_PTR_MASK, ovr);
}

/* Drain task: never sleeps while data waits (read time is close to the arrival
 * rate); 2 ms tick when idle. */
static void trace_task(void *arg)
{
    (void)arg;
    while (s_running) {
        const esp_err_t err = dap_trace_poll();
        if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
            vTaskDelay(1);                   /* a read failed; back off one tick */
        } else {
            taskYIELD();                     /* one tick (10 ms) would lap the TRAM */
        }
    }
    esp_task_wdt_add(xTaskGetIdleTaskHandleForCore(0));
    s_task = NULL;
    vTaskDelete(NULL);
}

size_t dap_trace_read(uint8_t *out, size_t max)
{
    size_t n = 0;

    if (s_ring == NULL || s_lock == NULL) {
        return 0;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const size_t used = ring_used();
    n = (used < max) ? used : max;
    const size_t first = (n < TRACE_RING_BYTES - s_ring_tail)
                       ? n : TRACE_RING_BYTES - s_ring_tail;
    memcpy(out, s_ring + s_ring_tail, first);
    memcpy(out + first, s_ring, n - first);
    s_ring_tail = (s_ring_tail + n) % TRACE_RING_BYTES;
    s_stats.queue_free = (uint32_t)ring_free();
    xSemaphoreGive(s_lock);
    return n;
}

void dap_trace_get_stats(dap_trace_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    *out = s_stats;
    out->running = s_running;
}

/* ------------------------------------------------------------------------- */
/* Self-test                                                                  */
/* ------------------------------------------------------------------------- */

/* Write a pattern into two paragraphs (so the index is seen to advance) and run
 * the real drain, ring and read path over them. */
#define SELFTEST_PARAGRAPHS  2u

esp_err_t dap_trace_selftest(void)
{
    static uint8_t got[SELFTEST_PARAGRAPHS *
                       (sizeof(dap_trace_record_t) + TRACE_PARAGRAPH)];

    dap_trace_stop();

    uint32_t bot = 0, top = 0;
    if (dap_probe_read32(TRACE_FIFOBOT, &bot) != ESP_OK ||
        dap_probe_read32(TRACE_FIFOTOP, &top) != ESP_OK) {
        ESP_LOGE(TAG, "selftest: the trace FIFO registers are not readable");
        return ESP_ERR_INVALID_STATE;
    }
    if (top <= bot || ((top - bot + 1u) % TRACE_PARAGRAPH) != 0u) {
        ESP_LOGE(TAG, "selftest: FIFOBOT/FIFOTOP are not whole paragraphs");
        return ESP_ERR_INVALID_STATE;
    }

    /* Paragraph index in the top byte, word index below it. */
    for (uint32_t par = 0; par < SELFTEST_PARAGRAPHS; par++) {
        const uint32_t addr = TRACE_TRAM_BASE + bot + par * TRACE_PARAGRAPH;

        for (uint32_t i = 0; i < TRACE_WORDS_PER_PAR; i++) {
            if (dap_probe_write32(addr + i * 4u,
                                  (par << 24) | (i & 0x00FFFFFFu)) != ESP_OK) {
                ESP_LOGE(TAG, "selftest: TRAM is not writable at 0x%08" PRIX32,
                         addr + i * 4u);
                return ESP_ERR_NOT_SUPPORTED;
            }
        }

        /* An acknowledged write is not necessarily applied; read one back. */
        uint32_t check = 0;
        const uint32_t want = (par << 24) | 1u;
        if (dap_probe_read32(addr + 4u, &check) != ESP_OK || check != want) {
            ESP_LOGE(TAG, "selftest: TRAM at 0x%08" PRIX32 " read back 0x%08"
                          PRIX32 ", wanted 0x%08" PRIX32, addr + 4u, check, want);
            return ESP_ERR_NOT_SUPPORTED;
        }
    }

    /* No background task: this thread drives the poll. */
    if (trace_begin() != ESP_OK) {
        return ESP_FAIL;
    }

    /* Report the pointer one paragraph past the pattern, so both count as
     * complete. */
    s_next_par = 0;

    size_t n = 0;
    for (int pass = 0; pass < 8 && n < sizeof(got); pass++) {
        const esp_err_t perr =
            drain_to(bot + SELFTEST_PARAGRAPHS * TRACE_PARAGRAPH, s_last_ovrcnt);
        if (perr != ESP_OK && perr != ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "selftest: a drain pass failed: %s",
                     esp_err_to_name(perr));
            break;
        }
        n += dap_trace_read(got + n, sizeof(got) - n);
    }
    s_running = false;

    if (n != sizeof(got)) {
        ESP_LOGE(TAG, "selftest: drained %u bytes of %u", (unsigned)n,
                 (unsigned)sizeof(got));
        return ESP_FAIL;
    }

    int            failures = 0;
    const uint8_t *p = got;

    for (uint32_t par = 0; par < SELFTEST_PARAGRAPHS; par++) {
        dap_trace_record_t hdr;

        memcpy(&hdr, p, sizeof(hdr));
        p += sizeof(hdr);

        if (hdr.magic != DAP_TRACE_MAGIC) {
            ESP_LOGE(TAG, "  record %" PRIu32 ": magic 0x%08" PRIX32, par, hdr.magic);
            failures++;
        }
        if (hdr.seq != par) {
            ESP_LOGE(TAG, "  record %" PRIu32 ": seq %" PRIu32, par, hdr.seq);
            failures++;
        }
        if (hdr.tram_offset != bot + par * TRACE_PARAGRAPH) {
            ESP_LOGE(TAG, "  record %" PRIu32 ": offset 0x%08" PRIX32, par,
                     hdr.tram_offset);
            failures++;
        }
        if (hdr.length != TRACE_PARAGRAPH) {
            ESP_LOGE(TAG, "  record %" PRIu32 ": length %u", par, hdr.length);
            failures++;
        }
        if (hdr.flags != 0 || hdr.lost != 0) {
            ESP_LOGE(TAG, "  record %" PRIu32 ": a gap was reported where none "
                          "was made, flags 0x%04X lost %" PRIu32,
                     par, hdr.flags, hdr.lost);
            failures++;
        }

        int bad = 0;
        for (uint32_t i = 0; i < TRACE_WORDS_PER_PAR; i++) {
            uint32_t w;

            memcpy(&w, p + i * 4u, sizeof(w));
            if (w != ((par << 24) | (i & 0x00FFFFFFu))) {
                if (bad == 0) {
                    ESP_LOGE(TAG, "  record %" PRIu32 ": word %" PRIu32
                                  " is 0x%08" PRIX32 ", wanted 0x%08" PRIX32,
                             par, i, w, (par << 24) | i);
                }
                bad++;
            }
        }
        if (bad) {
            ESP_LOGE(TAG, "  record %" PRIu32 ": %d of %u words wrong", par, bad,
                     (unsigned)TRACE_WORDS_PER_PAR);
            failures++;
        } else {
            ESP_LOGW(TAG, "  record %" PRIu32 ": seq %" PRIu32 ", offset 0x%08"
                          PRIX32 ", %u bytes, all correct", par, hdr.seq,
                     hdr.tram_offset, (unsigned)TRACE_PARAGRAPH);
        }
        p += TRACE_PARAGRAPH;
    }

    if (failures) {
        ESP_LOGE(TAG, "selftest FAILED (%d)", failures);
        return ESP_FAIL;
    }
    ESP_LOGW(TAG, "selftest passed: %u paragraphs drained and verified byte for "
                  "byte", (unsigned)SELFTEST_PARAGRAPHS);
    return ESP_OK;
}
