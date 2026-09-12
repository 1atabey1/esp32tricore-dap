/*
 * Upload an Intel HEX and program it into the TriCore, over HTTP.
 *
 * The image arrives as a HEX rather than an ELF because the ELF for this
 * project is 43 MB and nearly all of it is debug information that flashing
 * never touches; the HEX beside it describes the same bytes in 2 MB.  That is
 * the difference between an image this probe can hold and one it cannot.
 *
 * The records are parsed as they stream in rather than buffered and parsed
 * afterwards, so the 2 MB of text never exists anywhere at once - only the
 * 700 kB of program content it describes.
 *
 * The programming itself runs in a task of its own.  esp_http_server handles
 * every request from one task, so doing it in the handler would block every
 * other endpoint for the length of a flash - including the one that reports
 * progress, which is the only thing anyone wants to look at while it runs.
 */

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tricore_flash.h"

static const char *TAG = "FLASH_WEB";

/*
 * The window of program flash this accepts, as physical (uncached) addresses.
 *
 * Anything outside it is counted and skipped rather than programmed: a HEX
 * carries the UCBs, the boot mode headers and data-flash content too, and a bad
 * write to any of those is not recoverable with this tool.  The same rule the
 * reference flasher applies, for the same reason.
 */
#define PFLASH_BASE  0xA0000000u
#define PFLASH_END   0xA0800000u
/* The cached alias the linker usually emits; the flash is the same. */
#define PFLASH_CACHED_BASE 0x80000000u

/* How much image the probe will hold.  TC38x program flash is larger than this,
 * but an application image that needs more than 2 MB is not what this is for. */
#define IMAGE_MAX_BYTES (2u * 1024u * 1024u)

typedef struct {
    uint8_t *data;          /* PSRAM, IMAGE_MAX_BYTES, 0xFF-filled */
    uint32_t lowest;        /* physical address of the first byte seen */
    uint32_t highest;       /* physical address after the last byte seen */
    uint32_t bytes;         /* program-flash bytes accepted */
    uint32_t skipped;       /* bytes outside program flash */
    uint32_t records;
    uint32_t upper;         /* from the last type 04/02 record */
    bool     bad;
    char     error[96];
    /* A record split across two chunks of the upload. */
    char     line[128];
    size_t   line_len;
} hex_image_t;

static hex_image_t s_image;
static TaskHandle_t s_flash_task;

/* -- the parser ----------------------------------------------------------- */

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void fail(const char *why)
{
    if (!s_image.bad) {
        s_image.bad = true;
        strncpy(s_image.error, why, sizeof(s_image.error) - 1);
        s_image.error[sizeof(s_image.error) - 1] = '\0';
        ESP_LOGE(TAG, "%s", why);
    }
}

/*
 * One complete record, without its leading colon.
 *
 * Checksums are verified rather than assumed.  The whole reason flashing from a
 * HEX is safe is that it is the same bytes as the ELF, and a reader that
 * quietly repaired one would not be checking that any more.
 */
static void parse_record(const char *text, size_t len)
{
    uint8_t raw[72];
    size_t n = 0;

    if (len < 10 || (len & 1u)) {
        fail("a record has an odd or impossible length");
        return;
    }
    if (len / 2 > sizeof(raw)) {
        fail("a record is longer than this reader accepts");
        return;
    }
    for (size_t i = 0; i < len; i += 2) {
        const int hi = hex_nibble(text[i]);
        const int lo = hex_nibble(text[i + 1]);
        if (hi < 0 || lo < 0) {
            fail("a record contains something that is not hexadecimal");
            return;
        }
        raw[n++] = (uint8_t)((hi << 4) | lo);
    }

    const uint8_t count = raw[0];
    const uint16_t offset = (uint16_t)((raw[1] << 8) | raw[2]);
    const uint8_t kind = raw[3];

    if (n != (size_t)count + 5u) {
        fail("a record's length byte does not match what it carries");
        return;
    }
    uint8_t sum = 0;
    for (size_t i = 0; i < n; i++) {
        sum = (uint8_t)(sum + raw[i]);
    }
    if (sum != 0) {
        fail("a record's checksum does not add up");
        return;
    }

    s_image.records++;

    switch (kind) {
    case 0x00: {
        uint32_t address = s_image.upper + offset;

        /* The linker emits the cached alias; the flash is the same. */
        if (address >= PFLASH_CACHED_BASE && address < PFLASH_CACHED_BASE + 0x800000u) {
            address = address - PFLASH_CACHED_BASE + PFLASH_BASE;
        }
        if (address < PFLASH_BASE || address + count > PFLASH_END) {
            s_image.skipped += count;
            return;
        }
        const uint32_t at = address - PFLASH_BASE;
        if (at + count > IMAGE_MAX_BYTES) {
            fail("the image reaches past what this probe will hold");
            return;
        }
        memcpy(s_image.data + at, &raw[4], count);
        if (address < s_image.lowest) {
            s_image.lowest = address;
        }
        if (address + count > s_image.highest) {
            s_image.highest = address + count;
        }
        s_image.bytes += count;
        break;
    }
    case 0x04:
        if (count != 2) {
            fail("a type 04 record does not carry two bytes");
            return;
        }
        s_image.upper = (uint32_t)((raw[4] << 8) | raw[5]) << 16;
        break;
    case 0x02:
        if (count != 2) {
            fail("a type 02 record does not carry two bytes");
            return;
        }
        s_image.upper = (uint32_t)((raw[4] << 8) | raw[5]) << 4;
        break;
    case 0x01:      /* end of file */
    case 0x03:      /* start segment address: says nothing about content */
    case 0x05:      /* start linear address: likewise */
        break;
    default:
        fail("a record has a type this reader does not know");
        break;
    }
}

static void feed(const char *chunk, size_t len)
{
    for (size_t i = 0; i < len && !s_image.bad; i++) {
        const char c = chunk[i];

        if (c == '\n' || c == '\r') {
            if (s_image.line_len) {
                parse_record(s_image.line, s_image.line_len);
                s_image.line_len = 0;
            }
            continue;
        }
        if (c == ':') {
            s_image.line_len = 0;       /* a record starts here */
            continue;
        }
        if (s_image.line_len < sizeof(s_image.line)) {
            s_image.line[s_image.line_len++] = c;
        } else {
            fail("a record is longer than this reader accepts");
        }
    }
}

static esp_err_t image_reset(void)
{
    if (s_image.data == NULL) {
        s_image.data = heap_caps_malloc(IMAGE_MAX_BYTES,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_image.data == NULL) {
            ESP_LOGE(TAG, "no PSRAM for the image");
            return ESP_ERR_NO_MEM;
        }
    }
    uint8_t *keep = s_image.data;
    memset(&s_image, 0, sizeof(s_image));
    s_image.data = keep;
    s_image.lowest = 0xFFFFFFFFu;
    /* Erased, so a gap inside the image programs as erased flash. */
    memset(s_image.data, TRICORE_FLASH_ERASED, IMAGE_MAX_BYTES);
    return ESP_OK;
}

/* -- the endpoints -------------------------------------------------------- */

extern esp_err_t check_auth(httpd_req_t *req);

static esp_err_t flash_upload_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    if (s_flash_task) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "a flash is already running\n");
        return ESP_OK;
    }
    if (image_reset() != ESP_OK) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "no memory for the image\n");
        return ESP_OK;
    }

    char *buf = malloc(4096);
    if (buf == NULL) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "no buffer for the upload\n");
        return ESP_OK;
    }

    int left = req->content_len;
    while (left > 0) {
        const int want = (left > 4096) ? 4096 : left;
        const int got = httpd_req_recv(req, buf, want);
        if (got <= 0) {
            free(buf);
            ESP_LOGE(TAG, "the upload stopped early");
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_sendstr(req, "the upload stopped early\n");
            return ESP_OK;
        }
        feed(buf, (size_t)got);
        left -= got;
    }
    free(buf);

    /* A file that ends without a newline still has a record in hand. */
    if (!s_image.bad && s_image.line_len) {
        parse_record(s_image.line, s_image.line_len);
        s_image.line_len = 0;
    }

    char reply[256];
    if (s_image.bad) {
        snprintf(reply, sizeof(reply), "rejected: %s\n", s_image.error);
        httpd_resp_set_status(req, "400 Bad Request");
    } else if (s_image.bytes == 0) {
        snprintf(reply, sizeof(reply),
                 "rejected: nothing in this file lands in program flash\n");
        httpd_resp_set_status(req, "400 Bad Request");
    } else {
        snprintf(reply, sizeof(reply),
                 "ok records=%" PRIu32 " bytes=%" PRIu32 " skipped=%" PRIu32
                 " span=0x%08" PRIX32 "..0x%08" PRIX32 "\n",
                 s_image.records, s_image.bytes, s_image.skipped,
                 s_image.lowest, s_image.highest);
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, reply);
    return ESP_OK;
}

static void flash_task(void *arg)
{
    /*
     * Rounded out to whole pages.
     *
     * A page is the smallest thing that can be written and it is written whole,
     * so a region that starts or ends inside one has to be widened rather than
     * truncated.  It costs nothing: the buffer is erased-filled, so the bytes
     * this pulls in are the value erased flash already holds.
     */
    const uint32_t from = s_image.lowest & ~(TRICORE_FLASH_PAGE - 1u);
    const uint32_t to   = (s_image.highest + TRICORE_FLASH_PAGE - 1u) &
                          ~(TRICORE_FLASH_PAGE - 1u);

    const tricore_flash_region_t region = {
        .address = from,
        .length  = to - from,
        .data    = s_image.data + (from - PFLASH_BASE),
    };

    ESP_LOGI(TAG, "programming 0x%08" PRIX32 ", %" PRIu32 " bytes",
             region.address, region.length);
    tricore_flash_write(&region, 1);

    s_flash_task = NULL;
    vTaskDelete(NULL);
}

static esp_err_t flash_start_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    if (s_flash_task) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "a flash is already running\n");
        return ESP_OK;
    }
    if (s_image.data == NULL || s_image.bytes == 0 || s_image.bad) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "upload an image first\n");
        return ESP_OK;
    }

    /* ?slow=1 forces the word-at-a-time path, to tell a fabric problem from
     * everything else without a rebuild. */
    char query[32], val[8];
    bool fast = true;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "slow", val, sizeof(val)) == ESP_OK) {
        fast = (atoi(val) == 0);
    }
    tricore_flash_set_blockwrite(fast);

    /*
     * Its own task, at a low priority.  The flash takes seconds and the web
     * server runs every request from one task, so doing it here would block the
     * progress endpoint - the one thing worth looking at while it runs.
     */
    if (xTaskCreate(flash_task, "tricore_flash", 8192, NULL, 4,
                    &s_flash_task) != pdPASS) {
        s_flash_task = NULL;
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "could not start the flash task\n");
        return ESP_OK;
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "started\n");
    return ESP_OK;
}

static const char *phase_name(tricore_flash_phase_t phase)
{
    switch (phase) {
    case TRICORE_FLASH_IDLE:        return "idle";
    case TRICORE_FLASH_PREPARING:   return "preparing";
    case TRICORE_FLASH_ERASING:     return "erasing";
    case TRICORE_FLASH_PROGRAMMING: return "programming";
    case TRICORE_FLASH_VERIFYING:   return "verifying";
    case TRICORE_FLASH_DONE:        return "done";
    default:                        return "failed";
    }
}

static esp_err_t flash_status_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    tricore_flash_status_t st;
    char line[320];

    tricore_flash_get_status(&st);
    const int n = snprintf(line, sizeof(line),
        "phase=%s running=%d total=%" PRIu32 " done=%" PRIu32
        " sectors=%" PRIu32 " sectors_done=%" PRIu32 " ms=%" PRIu32
        " verified=%d errsr=0x%08" PRIX32 " image_bytes=%" PRIu32
        " message=%s\n",
        phase_name(st.phase), s_flash_task ? 1 : 0, st.total_bytes,
        st.done_bytes, st.sectors, st.sectors_done, st.elapsed_ms,
        st.verified ? 1 : 0, st.errsr, s_image.bytes, st.message);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, line, (n > 0) ? (size_t)n : 0);
    return ESP_OK;
}

esp_err_t flash_page_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;
    extern const unsigned char flash_start_[] asm("_binary_flash_html_start");
    extern const unsigned char flash_end_[]   asm("_binary_flash_html_end");

    return httpd_resp_send(req, (const char *)flash_start_,
                           (size_t)(flash_end_ - flash_start_));
}

static httpd_uri_t uri_flash_page = {
    .uri = "/flash.html", .method = HTTP_GET,
    .handler = flash_page_handler, .user_ctx = NULL
};
static httpd_uri_t uri_flash_upload = {
    .uri = "/api/flash/upload", .method = HTTP_POST,
    .handler = flash_upload_handler, .user_ctx = NULL
};
static httpd_uri_t uri_flash_start = {
    .uri = "/api/flash/start", .method = HTTP_POST,
    .handler = flash_start_handler, .user_ctx = NULL
};
static httpd_uri_t uri_flash_status = {
    .uri = "/api/flash/status", .method = HTTP_GET,
    .handler = flash_status_handler, .user_ctx = NULL
};

void flash_web_register(httpd_handle_t server)
{
    httpd_register_uri_handler(server, &uri_flash_page);
    httpd_register_uri_handler(server, &uri_flash_upload);
    httpd_register_uri_handler(server, &uri_flash_start);
    httpd_register_uri_handler(server, &uri_flash_status);
}
