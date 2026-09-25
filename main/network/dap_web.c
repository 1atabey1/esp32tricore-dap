/*
 * HTTP endpoints for the TriCore DAP probe: bring-up, trace drain, FPGA image
 * selection and the Black Magic GDB target.
 *
 * Diagnostic endpoints return the log they produced as a text/plain body,
 * followed by a one-line verdict.
 */

#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "storage.h"
#include "../esp32jtag_common.h"
#include "../ice40up5k/ice.h"
#include "dap_lock.h"
#include "dap_probe.h"
#include "dap_trace.h"
#include "dap_phy_fpga.h"
#include "tricore.h"
#include "tricore_bmp.h"

static const char *TAG = "DAP_WEB";

extern esp_err_t check_auth(httpd_req_t *req);

/* Log capture buffer for one diagnostic run. */
#define DAP_CAPTURE_BYTES (96 * 1024)

/* One socket write per pass of the trace stream loop. */
#define DAP_TRACE_CHUNK_BYTES 4096

/* Empty 5 ms passes to wait after the drain stops, so a paragraph still in
 * flight is not cut off. */
#define DAP_TRACE_DRAIN_TAIL_PASSES 20

/* Empty 5 ms passes to wait while the drain runs before returning anyway. */
#define DAP_TRACE_IDLE_PASSES 100

/* Longest a single stream response may run: esp_http_server serves all
 * requests from one task, so every other endpoint waits behind it. */
#define DAP_TRACE_STREAM_MAX_MS 1000

/* ------------------------------------------------------------------------ */
/* Log capture                                                                */
/* ------------------------------------------------------------------------ */

/*
 * Tee the log into a PSRAM buffer between dap_capture_begin() and
 * dap_capture_end(), which sends it as the response body.  Lines logged by
 * other tasks meanwhile are captured too.
 */
static char           *s_dap_cap;
static size_t          s_dap_cap_len;
static vprintf_like_t  s_dap_cap_prev;

static int dap_capture_vprintf(const char *format, va_list args)
{
    if (s_dap_cap) {
        va_list copy;
        va_copy(copy, args);
        const size_t room = DAP_CAPTURE_BYTES - 1 - s_dap_cap_len;
        if (room > 1) {
            const int n = vsnprintf(s_dap_cap + s_dap_cap_len, room, format, copy);
            if (n > 0) {
                s_dap_cap_len += ((size_t)n < room) ? (size_t)n : room - 1;
            }
        }
        va_end(copy);
    }
    return s_dap_cap_prev ? s_dap_cap_prev(format, args) : 0;
}

/* Also holds the DAP lock until dap_capture_end(): a diagnostic run
 * reconfigures the link and must not interleave with GDB or the flasher. */
static void dap_capture_begin(void)
{
    dap_lock();
    s_dap_cap_len = 0;
    s_dap_cap = heap_caps_malloc(DAP_CAPTURE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_dap_cap) {
        return;
    }
    s_dap_cap[0] = '\0';
    s_dap_cap_prev = esp_log_set_vprintf(dap_capture_vprintf);
}

/* Restore logging and send whatever was captured, with `verdict` on the end. */
static void dap_capture_end(httpd_req_t *req, const char *verdict)
{
    if (s_dap_cap) {
        esp_log_set_vprintf(s_dap_cap_prev);
        s_dap_cap_prev = NULL;
    }
    dap_unlock();
    httpd_resp_set_type(req, "text/plain");
    if (s_dap_cap) {
        s_dap_cap[s_dap_cap_len] = '\0';
        /* A zero-length chunk terminates chunked encoding, so an empty capture
         * is not sent; otherwise the verdict would never go out. */
        if (s_dap_cap_len > 0) {
            httpd_resp_send_chunk(req, s_dap_cap, s_dap_cap_len);
        }
        free(s_dap_cap);
        s_dap_cap = NULL;
    }
    httpd_resp_send_chunk(req, verdict, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, NULL, 0);
}

/* ------------------------------------------------------------------------ */
/* Bring-up                                                                   */
/* ------------------------------------------------------------------------ */

/*
 * GET /api/dap_bringup - run the DAP bring-up checkpoints; returns the log and
 * a verdict, or 503 if the DAP PHY is unavailable.  Port C must be in SWD/JTAG
 * mode.
 */
static esp_err_t dap_bringup_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    esp_err_t err = dap_probe_init(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "DAP PHY unavailable on this board");
        return ESP_OK;
    }

    dap_capture_begin();
    err = dap_probe_bringup_report();
    dap_capture_end(req, err == ESP_OK
        ? "\n=== DAP bring-up PASSED ===\n"
        : "\n=== DAP bring-up did not complete ===\n");
    return ESP_OK;
}

/*
 * GET /api/dap_spi - run the bring-up checkpoints with GP-SPI clocking and
 * measure both backends; returns the log and a verdict.  A failed SPI run
 * reverts to bit-bang.
 */
static esp_err_t dap_spi_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    esp_err_t err = dap_probe_init(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "DAP PHY unavailable on this board");
        return ESP_OK;
    }

    dap_capture_begin();
    err = dap_probe_spi_bringup();
    dap_capture_end(req, err == ESP_OK
        ? "\n=== GP-SPI backend PASSED ===\n"
        : "\n=== GP-SPI backend did not pass ===\n");
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Trace drain                                                                */
/* ------------------------------------------------------------------------ */

/*
 * GET /api/dap_trace/start - attach, enable OCDS and start the drain; returns
 * the log and a verdict.  MCDS itself is configured by the host, not here.
 */
static esp_err_t dap_trace_start_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    esp_err_t err = dap_probe_init(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "DAP PHY unavailable on this board\n");
        return ESP_OK;
    }

    /* OCDS must be on, or the miniMCDS registers bus-error. */
    dap_capture_begin();

    dap_exchange_t x;
    err = dap_probe_attach(&x, 3);
    if (err == ESP_OK) {
        dap_probe_client_set(1, &x);
        dap_probe_clear_error_state();
        dap_probe_set_rw_mode(true);
        dap_probe_enable_ocds();
        err = dap_trace_start();
    }
    dap_capture_end(req, err == ESP_OK
        ? "\n=== draining ===\n"
        : "\n=== could not start the drain ===\n");
    return ESP_OK;
}

/*
 * GET /api/dap_trace/selftest - attach and run dap_trace_selftest(), which
 * checks the drain on a target that is not tracing; returns the log and a
 * verdict.
 */
static esp_err_t dap_trace_selftest_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    esp_err_t err = dap_probe_init(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "DAP PHY unavailable on this board\n");
        return ESP_OK;
    }

    dap_capture_begin();

    dap_exchange_t x;
    err = dap_probe_attach(&x, 3);
    if (err == ESP_OK) {
        dap_probe_client_set(1, &x);
        dap_probe_clear_error_state();
        dap_probe_set_rw_mode(true);
        dap_probe_enable_ocds();
        err = dap_trace_selftest();
    }
    dap_capture_end(req, err == ESP_OK
        ? "\n=== trace drain verified ===\n"
        : "\n=== trace drain NOT verified ===\n");
    return ESP_OK;
}

/* GET /api/dap_trace/stop - stop the drain; returns "stopped". */
static esp_err_t dap_trace_stop_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    dap_trace_stop();
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "stopped\n");
    return ESP_OK;
}

/* GET /api/dap_trace/stats - one text line of drain counters. */
static esp_err_t dap_trace_stats_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    dap_trace_stats_t st;
    char line[384];

    dap_trace_get_stats(&st);
    const int n = snprintf(line, sizeof(line),
        "running=%d paragraphs=%" PRIu32 " bytes=%" PRIu32 " lost=%" PRIu32
        " laps=%" PRIu32 " overruns=%" PRIu32 " read_errors=%" PRIu32
        " fifonow=0x%08" PRIX32 " queue_free=%" PRIu32 " queue_dropped=%" PRIu32
        " poll_us_max=%" PRIu32 "\n",
        st.running ? 1 : 0, st.paragraphs, st.bytes, st.lost, st.laps,
        st.overruns, st.read_errors, st.fifonow, st.queue_free,
        st.queue_dropped, st.poll_us_max);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, line, (n > 0) ? (size_t)n : 0);
    return ESP_OK;
}

/*
 * GET /api/dap_trace/stream - the drained stream as chunked
 * application/octet-stream: dap_trace_record_t-framed paragraphs, with gaps
 * flagged in the header.
 *
 * esp_http_server serves all requests from one task, so the stream returns
 * after 1 s (or sooner once idle) and the client reconnects; the 64 kB ring
 * covers the gap.  /ws/trace is the transport for sustained capture.
 */
static esp_err_t dap_trace_stream_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    const int64_t deadline = esp_timer_get_time() + DAP_TRACE_STREAM_MAX_MS * 1000;

    uint8_t *buf = heap_caps_malloc(DAP_TRACE_CHUNK_BYTES,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == NULL) {
        buf = heap_caps_malloc(DAP_TRACE_CHUNK_BYTES, MALLOC_CAP_8BIT);
    }
    if (buf == NULL) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "no buffer for the stream\n");
        return ESP_OK;
    }
    httpd_resp_set_type(req, "application/octet-stream");

    int idle = 0;
    for (;;) {
        const size_t n = dap_trace_read(buf, DAP_TRACE_CHUNK_BYTES);

        if (n) {
            idle = 0;
            if (httpd_resp_send_chunk(req, (const char *)buf, n) != ESP_OK) {
                break;                  /* the host went away */
            }
            if (esp_timer_get_time() >= deadline) {
                break;                  /* hand the server back; the host returns */
            }
            continue;                   /* there may be more waiting already */
        }

        dap_trace_stats_t st;
        dap_trace_get_stats(&st);
        if (!st.running && idle > DAP_TRACE_DRAIN_TAIL_PASSES) {
            break;                      /* stopped, and the ring has run dry */
        }
        if (idle > DAP_TRACE_IDLE_PASSES || esp_timer_get_time() >= deadline) {
            break;                      /* nothing arriving; do not hold the server */
        }
        idle++;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    free(buf);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Trace over a WebSocket                                                     */
/* ------------------------------------------------------------------------ */

/*
 * The upgrade handler returns at once and a sender task pushes the drain with
 * httpd_ws_send_frame_async, so the server task stays free.  One client at a
 * time: dap_trace_read() is destructive, so two readers would each get part of
 * the capture.
 */

#define TRACE_WS_CHUNK   4096
#define TRACE_WS_IDLE_MS 5

static httpd_handle_t s_trace_ws_hd;
static int            s_trace_ws_fd = -1;
static TaskHandle_t   s_trace_ws_task;
static volatile bool  s_trace_ws_run;

static void trace_ws_close(void)
{
    s_trace_ws_fd = -1;
    s_trace_ws_hd = NULL;
    s_trace_ws_run = false;
}

/* Sender task: forward the drain as binary frames until the socket closes or
 * a send fails. */
static void trace_ws_task(void *arg)
{
    uint8_t *buf = heap_caps_malloc(TRACE_WS_CHUNK,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == NULL) {
        buf = heap_caps_malloc(TRACE_WS_CHUNK, MALLOC_CAP_8BIT);
    }
    if (buf == NULL) {
        ESP_LOGE(TAG, "trace ws: no buffer");
        trace_ws_close();
        s_trace_ws_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "trace ws: streaming to fd %d", s_trace_ws_fd);

    while (s_trace_ws_run && s_trace_ws_fd >= 0) {
        const size_t n = dap_trace_read(buf, TRACE_WS_CHUNK);

        if (n == 0) {
            vTaskDelay(pdMS_TO_TICKS(TRACE_WS_IDLE_MS));
            continue;
        }

        httpd_ws_frame_t pkt = {
            .final   = true,
            .type    = HTTPD_WS_TYPE_BINARY,
            .payload = buf,
            .len     = n,
        };

        const esp_err_t err = httpd_ws_send_frame_async(s_trace_ws_hd,
                                                        s_trace_ws_fd, &pkt);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "trace ws: send failed (%s); closing",
                     esp_err_to_name(err));
            break;
        }
    }

    ESP_LOGI(TAG, "trace ws: stopped");
    free(buf);
    trace_ws_close();
    s_trace_ws_task = NULL;
    vTaskDelete(NULL);
}

/*
 * GET /ws/trace - websocket carrying the drained stream as binary frames.
 * The handshake starts the sender task (a second client is refused); frames
 * from the client are read and discarded, a close stops the sender.
 */
static esp_err_t trace_ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        if (s_trace_ws_fd >= 0) {
            ESP_LOGW(TAG, "trace ws: already streaming to fd %d",
                     s_trace_ws_fd);
            return ESP_FAIL;      /* refuse rather than split the stream */
        }
        s_trace_ws_hd  = req->handle;
        s_trace_ws_fd  = httpd_req_to_sockfd(req);
        s_trace_ws_run = true;

        if (xTaskCreate(trace_ws_task, "trace_ws", 4096, NULL, 5,
                        &s_trace_ws_task) != pdPASS) {
            ESP_LOGE(TAG, "trace ws: could not start the sender");
            trace_ws_close();
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    httpd_ws_frame_t pkt = {0};
    uint8_t scratch[64];

    pkt.payload = scratch;
    if (httpd_ws_recv_frame(req, &pkt, sizeof(scratch)) != ESP_OK) {
        s_trace_ws_run = false;
        return ESP_OK;
    }
    if (pkt.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "trace ws: client closed");
        s_trace_ws_run = false;
    }
    return ESP_OK;
}

/* GET /trace.html - the trace capture page. */
static esp_err_t trace_page_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;
    extern const unsigned char trace_start[] asm("_binary_trace_html_start");
    extern const unsigned char trace_end[]   asm("_binary_trace_html_end");

    return httpd_resp_send(req, (const char *)trace_start,
                           (size_t)(trace_end - trace_start));
}

/* ------------------------------------------------------------------------ */
/* FPGA image                                                                 */
/* ------------------------------------------------------------------------ */

/*
 * GET /api/fpga_image[?sel=dap|stock] - with `sel`, store the boot bitstream
 * choice and load that image now; without, report the current choice only.
 * Returns the log and "boot bitstream: ..."; 400 on a bad `sel`.
 */
static esp_err_t fpga_image_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    char query[64] = {0};
    char sel[16]   = {0};
    bool have_sel = false;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "sel", sel, sizeof(sel)) == ESP_OK) {
        have_sel = true;
    }

    if (have_sel) {
        if (strcmp(sel, "dap") != 0 && strcmp(sel, "stock") != 0) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_sendstr(req, "sel must be dap or stock\n");
            return ESP_OK;
        }
        if (storage_write(FPGA_IMAGE_KEY, sel, strlen(sel) + 1) != ESP_OK) {
            httpd_resp_set_status(req, "500 Internal Server Error");
            httpd_resp_sendstr(req, "could not save the setting\n");
            return ESP_OK;
        }
    }

    char *current = NULL;
    const bool stock = (storage_alloc_and_read(FPGA_IMAGE_KEY, &current) == ESP_OK &&
                        current && strcmp(current, "stock") == 0);
    free(current);

    dap_capture_begin();

    if (have_sel) {
        extern const unsigned char bitstream_bin_start[]  asm("_binary_bitstream_bin_start");
        extern const unsigned char bitstream_bin_end[]    asm("_binary_bitstream_bin_end");
        extern const unsigned char dap_master_bin_start[] asm("_binary_dap_master_bin_start");
        extern const unsigned char dap_master_bin_end[]   asm("_binary_dap_master_bin_end");

        /* The fabric is about to be replaced; drop the cached link state. */
        dap_phy_fpga_invalidate();

        /* Hand the config pins back to GPIO for ICE_FPGA_Config's bit-bang. */
        const gpio_config_t cfg_out = {
            .mode         = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << AEL_PIN_NUM_CLK) | (1ULL << AEL_PIN_NUM_MOSI),
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        const gpio_config_t cfg_in = {
            .mode         = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << AEL_PIN_NUM_MISO),
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg_out);
        gpio_config(&cfg_in);

        const uint8_t status = stock
            ? ICE_FPGA_Config(bitstream_bin_start,
                              (uint32_t)(bitstream_bin_end - bitstream_bin_start))
            : ICE_FPGA_Config(dap_master_bin_start,
                              (uint32_t)(dap_master_bin_end - dap_master_bin_start));
        ESP_LOGW(TAG, "loaded the %s bitstream: CDONE %s", stock ? "stock" : "DAP",
                 status == 0 ? "up" : "DID NOT COME UP");
    }

    char verdict[96];
    snprintf(verdict, sizeof(verdict), "\n=== boot bitstream: %s ===\n",
             stock ? "stock" : "DAP master");
    dap_capture_end(req, verdict);
    return ESP_OK;
}

/*
 * POST /api/fpga_load - configure the FPGA from the uploaded bitstream (body,
 * up to 512 kB) until the next reboot; the boot choice is unchanged.  Returns
 * the log and whether CDONE came up; 400 on a bad size or short upload.
 *
 *     curl -s -u admin:admin --data-binary @dap_master.bin \
 *          http://<board>/api/fpga_load
 */
static esp_err_t fpga_load_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    const int total = req->content_len;
    if (total <= 0 || total > 512 * 1024) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "implausible bitstream size\n");
        return ESP_OK;
    }

    /* PSRAM is fine: the bitstream is bit-banged by the CPU, not DMAed. */
    uint8_t *image = heap_caps_malloc(total, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (image == NULL) {
        image = heap_caps_malloc(total, MALLOC_CAP_8BIT);
    }
    if (image == NULL) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "no room for the bitstream\n");
        return ESP_OK;
    }

    int got = 0;
    while (got < total) {
        const int n = httpd_req_recv(req, (char *)image + got, total - got);
        if (n <= 0) {
            free(image);
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_sendstr(req, "upload did not complete\n");
            return ESP_OK;
        }
        got += n;
    }

    dap_capture_begin();
    ESP_LOGW(TAG, "configuring the FPGA from %d uploaded bytes", got);

    /* The fabric is about to be replaced; without this the next route check
     * trusts a stale "link up" and skips probing the new image. */
    dap_phy_fpga_invalidate();

    /*
     * ICE_FPGA_Config bit-bangs with gpio_set_level, which a pad still routed
     * to SPI2 ignores.  gpio_config() resets the IO_MUX function to GPIO.
     */
    const gpio_config_t cfg_out = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << AEL_PIN_NUM_CLK) | (1ULL << AEL_PIN_NUM_MOSI),
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    const gpio_config_t cfg_in = {
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << AEL_PIN_NUM_MISO),
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_out);
    gpio_config(&cfg_in);

    const uint8_t status = ICE_FPGA_Config(image, (uint32_t)got);
    free(image);

    /* Non-zero status: CDONE never came up, the FPGA rejected the image. */
    char verdict[96];
    snprintf(verdict, sizeof(verdict),
             "\n=== FPGA config %s (status %u) ===\n",
             status == 0 ? "accepted, CDONE up" : "REFUSED", status);
    dap_capture_end(req, verdict);
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Fabric DAP route and GDB target                                            */
/* ------------------------------------------------------------------------ */

/* GET /api/dap_fpga[?wide=1] - fabric DAP route check, optionally wide mode. */
static esp_err_t dap_fpga_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    bool wide = false;
    char query[32];
    char val[8];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "wide", val, sizeof(val)) == ESP_OK) {
        wide = atoi(val) != 0;
    }

    dap_capture_begin();
    const esp_err_t err = dap_probe_fpga_route_check(wide);
    dap_capture_end(req, err == ESP_OK
        ? "\n=== the fabric route works ===\n"
        : "\n=== the fabric route did not come up ===\n");
    return ESP_OK;
}

/*
 * GET /api/dap_gdb/attach - register the TC3xx cores as targets of the Black
 * Magic GDB server on port 4242; returns the log and a verdict.  On request
 * only, because attaching drives Port C.
 *
 *     (gdb) target extended-remote <board>:4242
 *     (gdb) attach 1
 */
static esp_err_t dap_gdb_attach_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    dap_capture_begin();
    const esp_err_t err = tricore_bmp_probe();
    dap_capture_end(req, err == ESP_OK
        ? "\n=== registered; target extended-remote <board>:4242, then attach 1 ===\n"
        : "\n=== could not attach to the target ===\n");
    return ESP_OK;
}

/* GET /api/dap_gdb/status - one text line: core count, each core's run state
 * (or "busy" while a flash or diagnostic holds the DAP), and gdb_thread's
 * minimum free stack. */
static esp_err_t dap_gdb_status_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    char   line[256];
    size_t n = 0;

    n += (size_t)snprintf(line + n, sizeof(line) - n, "cores=%d",
                          tricore_core_count());
    if (dap_trylock(300)) {
        for (int i = 0; i < tricore_core_count(); i++) {
            const int core = tricore_core_index(i);
            n += (size_t)snprintf(line + n, sizeof(line) - n, " cpu%d=%s", core,
                                  !tricore_core_started(core) ? "not-started"
                                  : tricore_is_halted(core) ? "halted" : "running");
        }
        dap_unlock();
    } else {
        n += (size_t)snprintf(line + n, sizeof(line) - n, " busy");
    }
    TaskHandle_t gdb_task = xTaskGetHandle("gdb_thread");
    if (gdb_task) {
        n += (size_t)snprintf(line + n, sizeof(line) - n, " gdb_stack_free=%u",
                              (unsigned)uxTaskGetStackHighWaterMark(gdb_task));
    }
    n += (size_t)snprintf(line + n, sizeof(line) - n, "\n");

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, line, n);
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Registration                                                               */
/* ------------------------------------------------------------------------ */

static httpd_uri_t uri_trace_page = {
    .uri = "/trace.html", .method = HTTP_GET,
    .handler = trace_page_handler, .user_ctx = NULL
};
static httpd_uri_t uri_trace_ws = {
    .uri = "/ws/trace", .method = HTTP_GET,
    .handler = trace_ws_handler, .user_ctx = NULL,
    .is_websocket = true,
};
static httpd_uri_t uri_dap_bringup = {
    .uri = "/api/dap_bringup", .method = HTTP_GET,
    .handler = dap_bringup_handler, .user_ctx = NULL
};
static httpd_uri_t uri_dap_spi = {
    .uri = "/api/dap_spi", .method = HTTP_GET,
    .handler = dap_spi_handler, .user_ctx = NULL
};
static httpd_uri_t uri_dap_trace_start = {
    .uri = "/api/dap_trace/start", .method = HTTP_GET,
    .handler = dap_trace_start_handler, .user_ctx = NULL
};
static httpd_uri_t uri_dap_trace_selftest = {
    .uri = "/api/dap_trace/selftest", .method = HTTP_GET,
    .handler = dap_trace_selftest_handler, .user_ctx = NULL
};
static httpd_uri_t uri_dap_trace_stop = {
    .uri = "/api/dap_trace/stop", .method = HTTP_GET,
    .handler = dap_trace_stop_handler, .user_ctx = NULL
};
static httpd_uri_t uri_dap_trace_stats = {
    .uri = "/api/dap_trace/stats", .method = HTTP_GET,
    .handler = dap_trace_stats_handler, .user_ctx = NULL
};
static httpd_uri_t uri_dap_trace_stream = {
    .uri = "/api/dap_trace/stream", .method = HTTP_GET,
    .handler = dap_trace_stream_handler, .user_ctx = NULL
};
static httpd_uri_t uri_fpga_image = {
    .uri = "/api/fpga_image", .method = HTTP_GET,
    .handler = fpga_image_handler, .user_ctx = NULL
};
static httpd_uri_t uri_fpga_load = {
    .uri = "/api/fpga_load", .method = HTTP_POST,
    .handler = fpga_load_handler, .user_ctx = NULL
};
static httpd_uri_t uri_dap_fpga = {
    .uri = "/api/dap_fpga", .method = HTTP_GET,
    .handler = dap_fpga_handler, .user_ctx = NULL
};
static httpd_uri_t uri_dap_gdb_attach = {
    .uri = "/api/dap_gdb/attach", .method = HTTP_GET,
    .handler = dap_gdb_attach_handler, .user_ctx = NULL
};
static httpd_uri_t uri_dap_gdb_status = {
    .uri = "/api/dap_gdb/status", .method = HTTP_GET,
    .handler = dap_gdb_status_handler, .user_ctx = NULL
};

void dap_web_register(httpd_handle_t server)
{
    httpd_register_uri_handler(server, &uri_trace_page);
    httpd_register_uri_handler(server, &uri_trace_ws);
    httpd_register_uri_handler(server, &uri_dap_bringup);
    httpd_register_uri_handler(server, &uri_dap_spi);
    httpd_register_uri_handler(server, &uri_dap_trace_start);
    httpd_register_uri_handler(server, &uri_dap_trace_selftest);
    httpd_register_uri_handler(server, &uri_dap_trace_stop);
    httpd_register_uri_handler(server, &uri_dap_trace_stats);
    httpd_register_uri_handler(server, &uri_dap_trace_stream);
    httpd_register_uri_handler(server, &uri_fpga_image);
    httpd_register_uri_handler(server, &uri_fpga_load);
    httpd_register_uri_handler(server, &uri_dap_fpga);
    httpd_register_uri_handler(server, &uri_dap_gdb_attach);
    httpd_register_uri_handler(server, &uri_dap_gdb_status);
}
