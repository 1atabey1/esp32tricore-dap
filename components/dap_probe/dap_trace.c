#include "dap_trace.h"

#include <inttypes.h>
#include <string.h>

#include "dap_probe.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "DAP_TRACE";

/*
 * The miniMCDS trace FIFO, repeated here rather than shared with dap_probe.c,
 * because these are the only addresses this file needs and a header of
 * addresses shared between two files is a header that grows.
 */
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

/*
 * Ring size.  The drain publishes at most one TRAM's worth per pass, and the
 * host reads over WiFi, whose latency spikes are what this absorbs; 64 kB is
 * eight whole buffers of slack and comes out of PSRAM, where there is room.
 */
#define TRACE_RING_BYTES    (64u * 1024u)

static uint8_t          *s_ring;
static size_t            s_ring_head;      /* next write */
static size_t            s_ring_tail;      /* next read */
static SemaphoreHandle_t s_lock;

static bool     s_running;
static uint32_t s_bot, s_top;              /* FIFO bounds, byte offsets */
static uint32_t s_span;                    /* top - bot + 1 */
static uint32_t s_next_par;                /* paragraph index we expect next */
static uint32_t s_last_ovrcnt;
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
    for (size_t i = 0; i < len; i++) {
        s_ring[s_ring_head] = src[i];
        s_ring_head = (s_ring_head + 1u) % TRACE_RING_BYTES;
    }
}

/*
 * Publish one paragraph.  Dropped whole rather than in part when the ring is
 * full: half a paragraph in the stream is worse than a missing one, because the
 * missing one is reported and the half is not.
 */
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

esp_err_t dap_trace_start(void)
{
    if (s_ring == NULL) {
        s_ring = heap_caps_malloc(TRACE_RING_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_ring == NULL) {
            s_ring = heap_caps_malloc(TRACE_RING_BYTES, MALLOC_CAP_8BIT);
        }
        if (s_ring == NULL) {
            ESP_LOGE(TAG, "no room for a %u byte ring", (unsigned)TRACE_RING_BYTES);
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

    /*
     * Zero the warning comparators.  Their purpose is to halt the buffer at a
     * threshold so a host can catch up, which is exactly the behaviour this
     * replaces - the drain follows the pointer instead, and a halted buffer
     * would stall the application under test rather than lose a paragraph.
     */
    dap_probe_write32(TRACE_FIFOWARN0, 0);
    dap_probe_write32(TRACE_FIFOWARN1, 0);

    s_bot  = bot;
    s_top  = top;
    s_span = top - bot + 1u;

    /* Start behind the pointer's own paragraph: that one is still being written. */
    const uint32_t now_par = ((now - bot) % s_span) / TRACE_PARAGRAPH;
    s_next_par     = now_par;
    s_last_ovrcnt  = ovr;
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

    if (s_task == NULL &&
        xTaskCreate(trace_task, "dap_trace", 4096, NULL, 6, &s_task) != pdPASS) {
        s_running = false;
        ESP_LOGE(TAG, "could not start the drain task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void dap_trace_stop(void)
{
    s_running = false;
    s_stats.running = false;
}

esp_err_t dap_trace_poll(void)
{
    static uint32_t words[TRACE_WORDS_PER_PAR];

    if (!s_running) {
        return ESP_OK;
    }

    const int64_t t0 = esp_timer_get_time();

    uint32_t now = 0, ovr = 0;
    if (dap_probe_read32(TRACE_FIFONOW, &now) != ESP_OK) {
        s_stats.read_errors++;
        dap_probe_clear_error_state();
        return ESP_ERR_TIMEOUT;
    }
    if (dap_probe_read32(TRACE_FIFOOVRCNT, &ovr) != ESP_OK) {
        ovr = s_last_ovrcnt;
    }
    if (ovr != s_last_ovrcnt) {
        /*
         * Observation-unit overflow, which is a different loss from a TRAM lap:
         * messages never reached the buffer.  The stream carries an ERR message
         * for it, so it is counted here and not turned into a gap record.
         */
        s_stats.overruns += (ovr - s_last_ovrcnt);
        s_last_ovrcnt = ovr;
    }

    const uint32_t total_par = s_span / TRACE_PARAGRAPH;
    const uint32_t now_par   = ((now - s_bot) % s_span) / TRACE_PARAGRAPH;

    /*
     * How many complete paragraphs are waiting.  The pointer's own paragraph is
     * excluded because it is still being written, so a drain that is fully
     * caught up has nothing to do.
     */
    uint32_t available = (now_par + total_par - s_next_par) % total_par;

    /*
     * A lap: the pointer has come all the way round and is writing into
     * paragraphs we had not read yet.  There is no ERR for this, so it is
     * detected by arithmetic and reported explicitly - which is the whole
     * difference between a stream a decoder can trust and one it cannot.
     *
     * Everything still unread is gone; resume at the pointer's paragraph.
     */
    uint32_t lost = 0;
    if (available >= total_par - 1u && available != 0u) {
        lost = available - (total_par - 1u) + 1u;
        s_stats.laps++;
        s_stats.lost += lost;
        s_next_par = now_par;
        available  = 0;
        ESP_LOGW(TAG, "TRAM lapped: %" PRIu32 " paragraphs lost, resuming at %" PRIu32,
                 lost, now_par);
    }

    const uint32_t to_read = available;
    while (available--) {
        const uint32_t addr = TRACE_TRAM_BASE + s_bot + s_next_par * TRACE_PARAGRAPH;

        if (dap_probe_blockread(addr, words, TRACE_WORDS_PER_PAR) != ESP_OK) {
            s_stats.read_errors++;
            dap_probe_clear_error_state();
            return ESP_ERR_TIMEOUT;
        }
        publish(s_next_par, words, lost);
        lost = 0;                       /* the marker belongs to one record only */
        s_next_par = (s_next_par + 1u) % total_par;
    }

    const uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
    if (us > s_stats.poll_us_max) {
        s_stats.poll_us_max = us;
    }
    s_stats.fifonow = now;
    s_stats.running = true;
    return to_read ? ESP_OK : ESP_ERR_NOT_FOUND;    /* NOT_FOUND: nothing waiting */
}

/*
 * The drain task.
 *
 * It does not sleep while there is anything to read, and that is not laziness
 * about pacing: three signals from a 20 kHz task come to roughly 400 kB/s of
 * messages, against 453 kB/s of block-read throughput, so a paragraph arrives
 * about every 2.5 ms and takes about 2.2 ms to read.  There is no idle time to
 * spend in a delay when a capture is actually running, and the 8 kB buffer is
 * only eight paragraphs of slack.  When the buffer is empty - no tracing
 * configured, or a slow application - it drops to a 2 ms tick so the rest of
 * the system gets the core back.
 */
static void trace_task(void *arg)
{
    (void)arg;
    while (s_running) {
        const esp_err_t err = dap_trace_poll();
        if (err == ESP_ERR_NOT_FOUND) {
            vTaskDelay(pdMS_TO_TICKS(2));
        } else if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));   /* a read failed; back off a little */
        } else {
            taskYIELD();
        }
    }
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
    while (n < max && s_ring_tail != s_ring_head) {
        out[n++] = s_ring[s_ring_tail];
        s_ring_tail = (s_ring_tail + 1u) % TRACE_RING_BYTES;
    }
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
