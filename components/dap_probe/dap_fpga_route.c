/*
 * Fabric DAP master: attach, and a self-checking route test.
 *
 * The DAP bitstream replaces the one that routes Port C to the CPU's DAP pins,
 * so there is no CPU-path reference to compare against.  Constants the target
 * must produce stand in for it:
 *
 *   sync          0xAAAAAAAA, valid CRC
 *   CLIENT_ID     0x0260
 *   miniMCDS ID   0x00D6C007 (after OCDS enable)
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dap_fpga_priv.h"
#include "dap_lock.h"
#include "dap_wide.h"
#include "dap_frame.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DAP_FPGA_RT";

#define MCDS_ID     0xFB718008u
#define MCDS_ID_VAL 0x00D6C007u

/* Trace RAM, first kB: idle outside a trace session. */
#define VERIFY_ADDR 0xB8000000u

static bool sync_ok(void)
{
    dap_exchange_t x;
    return dap_probe_attach(&x, 3) == ESP_OK && x.reply == 0xAAAAAAAAu;
}

esp_err_t dap_phy_fpga_attach(void)
{
    esp_err_t err = dap_phy_fpga_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "the fabric register file did not answer");
        return err;
    }

    dap_phy_fpga_set_div(5);            /* 48 MHz / (2*6) = 4 MHz */
    dap_phy_fpga_set_maxwait(1024);
    dap_phy_fpga_set_trail(1);
    dap_phy_fpga_use(true);

    /* A silent target is usually still in wide mode from an interrupted
     * session; only a target reset clears that. */
    if (!sync_ok()) {
        ESP_LOGW(TAG, "no answer to sync; resetting the target and retrying");
        dap_phy_fpga_set_wide(false);
        dap_phy_fpga_set_trst(true);
        vTaskDelay(pdMS_TO_TICKS(2));
        dap_phy_fpga_set_trst(false);
        vTaskDelay(pdMS_TO_TICKS(20));
        dap_probe_clear_error_state();

        if (!sync_ok()) {
            ESP_LOGE(TAG, "the target did not answer sync through the fabric");
            dap_phy_fpga_use(false);
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGW(TAG, "the reset brought it back");
    }

    /* Required once: without it the device answers sync and nothing else. */
    dap_dapisc_send();

    dap_exchange_t x;
    dap_exchange_t id = {0};
    esp_err_t      id_err = ESP_FAIL;

    for (int attempt = 0; attempt < 4; attempt++) {
        /* The IOINFO read inside clear_error_state is required before
         * client_read answers. */
        dap_probe_clear_error_state();
        if (dap_probe_attach(&x, 3) != ESP_OK ||
            dap_probe_client_set(1, &x) != ESP_OK) {
            continue;
        }
        id_err = dap_probe_client_read(DAP_IO_CLIENT_ID, 4, 16, &id);
        if (id_err == ESP_OK && id.reply == DAP_CLIENT_ID_EXPECT) {
            ESP_LOGI(TAG, "attached through the fabric: CLIENT_ID 0x%04X on "
                          "attempt %d", (unsigned)id.reply, attempt + 1);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "attach attempt %d: CLIENT_ID 0x%04X (%s)", attempt + 1,
                 (unsigned)id.reply, esp_err_to_name(id_err));
    }

    ESP_LOGE(TAG, "CLIENT_ID came back 0x%04X, not 0x%04X",
             (unsigned)id.reply, DAP_CLIENT_ID_EXPECT);
    return ESP_FAIL;
}

void dap_fpga_block_read_sweep(const char *label)
{
    static uint32_t buf[256];
    static const uint8_t divs[] = { 11, 5, 3, 2, 1, 0 };
    const int iterations = 8;

    ESP_LOGW(TAG, "--- %s block read throughput ---", label);
    for (size_t d = 0; d < sizeof(divs); d++) {
        dap_phy_fpga_set_div(divs[d]);
        int ok = 0;

        const int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < iterations; i++) {
            if (dap_probe_blockread(CHECK_ADDR, buf, 256) == ESP_OK) {
                ok++;
            } else {
                dap_probe_clear_error_state();
            }
        }
        const int64_t us = esp_timer_get_time() - t0;
        const unsigned khz = 48000u / (2u * (divs[d] + 1u));

        if (ok && us > 0) {
            ESP_LOGW(TAG, "  div %2u (%5u kHz): %d/%d x 1 kB in %6lld us -> %4d kB/s",
                     divs[d], khz, ok, iterations, (long long)us,
                     (int)((int64_t)ok * 1000000 / us));
        } else {
            ESP_LOGW(TAG, "  div %2u (%5u kHz): no full blocks", divs[d], khz);
        }
    }
    dap_phy_fpga_set_div(1);
}

esp_err_t dap_fpga_bench(uint32_t addr, int n, size_t words, uint8_t div, bool wide,
                         int chain, int trail, char *out, size_t outlen)
{
    dap_fpga_stats_t st;
    dap_block_req_t  reqs[16];

    if (words == 0 || words > 256 || n <= 0 || chain < 1 || chain > 16) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t *buf = malloc(words * sizeof(uint32_t) * (size_t)chain);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < chain; i++) {
        reqs[i].addr  = addr;
        reqs[i].count = (uint16_t)words;
    }
    n = (n + chain - 1) / chain * chain;         /* whole chains */
    dap_lock();
    esp_err_t err = ESP_OK;
    if (!dap_phy_fpga_in_use()) {
        err = dap_phy_fpga_attach();
        if (err == ESP_OK) {
            dap_probe_set_rw_mode(true);
            err = dap_probe_enable_ocds();
        }
    }
    /* Data check: a pattern written into the trace RAM at the attach clock. */
    static uint32_t ref[256];
    dap_phy_fpga_set_div(5);
    for (uint32_t i = 0, x = 0x2545F491u; i < 256; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        ref[i] = x;
    }
    if (err == ESP_OK) {
        err = dap_probe_enable_ocds();
    }
    for (uint32_t i = 0; i < 256 && err == ESP_OK; i++) {
        err = dap_probe_write32(VERIFY_ADDR + 4u * i, ref[i]);
    }
    if (err == ESP_OK && wide) {
        err = dap_wide_enter(div);
    }
    if (err != ESP_OK) {
        dap_unlock();
        free(buf);
        snprintf(out, outlen, "setup failed: %s\n", esp_err_to_name(err));
        return err;
    }
    dap_phy_fpga_set_div(div);

    /* The same flash block read back at this clock, as chains. */
    int mismatch = 0;
    {
        dap_block_req_t vreq[16];
        for (int i = 0; i < chain; i++) {
            vreq[i].addr  = VERIFY_ADDR;
            vreq[i].count = 256;
        }
        uint32_t *vbuf = malloc(256 * sizeof(uint32_t) * (size_t)chain);
        for (int rep = 0; vbuf != NULL && rep < 8; rep++) {
            if (dap_probe_blockread_many(vreq, (size_t)chain, vbuf) != ESP_OK) {
                mismatch += chain;
                dap_probe_clear_error_state();
                continue;
            }
            for (int i = 0; i < chain; i++) {
                mismatch += memcmp(vbuf + 256 * i, ref, sizeof(ref)) != 0;
            }
        }
        free(vbuf);
    }

    /* Block write at this clock: 128 words, first word repaired as the flash
     * loader does, read back. */
    int bw_bad = -1;
    {
        static uint32_t bw[128], bwback[128];
        for (int i = 0; i < 128; i++) {
            bw[i] = ref[i] ^ 0xFFFFFFFFu;
        }
        if (dap_phy_fpga_block_write(VERIFY_ADDR + 0x800u, bw, 128) == ESP_OK) {
            dap_probe_clear_error_state();
            dap_probe_write32(VERIFY_ADDR + 0x800u, bw[0]);
            if (dap_probe_blockread(VERIFY_ADDR + 0x800u, bwback, 128) == ESP_OK) {
                bw_bad = 0;
                for (int i = 0; i < 128; i++) {
                    if (bwback[i] != bw[i]) {
                        if (bw_bad == 0) {
                            ESP_LOGW(TAG, "block write word %d: 0x%08" PRIX32 " read 0x%08" PRIX32,
                                     i, bw[i], bwback[i]);
                        }
                        bw_bad++;
                    }
                }
            }
        }
        dap_probe_clear_error_state();
    }
    dap_phy_fpga_set_chain_lead((uint8_t)(trail > 0 ? trail : 2));
    dap_phy_fpga_stats(&st, true);

    int ok = 0, bad = 0;
    const int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < n; i += chain) {
        if (dap_probe_blockread_many(reqs, (size_t)chain, buf) == ESP_OK) {
            ok += chain;
        } else {
            bad += chain;
            dap_probe_clear_error_state();
        }
    }
    const int64_t us = esp_timer_get_time() - t0;
    dap_phy_fpga_stats(&st, true);

    /* Single-word reads, for comparison: read32 and the fast polling read. */
    uint32_t w = 0;
    int64_t r0 = esp_timer_get_time();
    for (int i = 0; i < 16; i++) {
        dap_probe_read32(addr, &w);
    }
    const int64_t read_us = (esp_timer_get_time() - r0) / 16;
    r0 = esp_timer_get_time();
    for (int i = 0; i < 16; i++) {
        dap_probe_read32_fast(addr, &w);
    }
    const int64_t fast_us = (esp_timer_get_time() - r0) / 16;

    /* Writes at this clock: word writes into the trace RAM (idle outside a
     * trace session), read back. */
    int wr_bad = 0;
    for (uint32_t i = 0; i < 16; i++) {
        const uint32_t pat = 0xA5000000u ^ (i * 0x01010101u);
        uint32_t back = 0;
        const esp_err_t we = dap_probe_write32(0xB8000000u + 4u * i, pat);
        const esp_err_t re = (we == ESP_OK) ? dap_probe_read32(0xB8000000u + 4u * i, &back)
                                            : ESP_FAIL;
        if (we != ESP_OK || re != ESP_OK || back != pat) {
            if (wr_bad == 0) {
                ESP_LOGW(TAG, "write check %u: write %s, read %s, 0x%08" PRIX32 " -> 0x%08" PRIX32,
                         (unsigned)i, esp_err_to_name(we), esp_err_to_name(re), pat, back);
            }
            wr_bad++;
            dap_probe_clear_error_state();
        }
    }

    uint32_t ns_short = 0, ns_long = 0;
    int      sck_khz  = 0;
    dap_phy_fpga_link_timing(&ns_short, &ns_long, &sck_khz);

    if (wide) {
        dap_wide_exit();
    }
    dap_phy_fpga_set_div(5);
    dap_unlock();
    free(buf);

    const double kbps = us > 0 ? (double)ok * words * 4 * 1e6 / 1024.0 / (double)us : 0;
    snprintf(out, outlen,
             "addr 0x%08" PRIX32 " div %u (%u kHz) %s: %d/%d blocks of %u words, "
             "chains of %d\n"
             "  %.0f kB/s, %.1f us/block\n"
             "  per block: %.1f SPI xfers, %.0f bytes, %.1f us inside SPI (%.0f%%), %.1f polls\n"
             "  verify: %d of %d trace-RAM blocks differ from the pattern written; "
             "block write: %d of 128 words wrong (-1: failed)\n"
             "  read32: %lld us, read32_fast: %lld us, write32+read32 failures %d/16\n"
             "  link: SCK %d kHz, 7-byte read %.2f us, 1 KB read %.1f us (%.0f kB/s)\n",
             addr, div, (unsigned)(48000u / (2u * (div + 1u))), wide ? "wide" : "narrow",
             ok, n, (unsigned)words, chain, kbps, (double)us / n,
             (double)st.xfers / n, (double)st.xfer_bytes / n, (double)st.xfer_us / n,
             us ? 100.0 * st.xfer_us / (double)us : 0.0, (double)st.polls / n,
             mismatch, 8 * chain, bw_bad,
             (long long)read_us, (long long)fast_us, wr_bad,
             sck_khz, ns_short / 1000.0, ns_long / 1000.0,
             ns_long ? 1024.0 * 1e6 / 1024.0 / ns_long * 1000.0 : 0.0);
    return bad ? ESP_FAIL : ESP_OK;
}

esp_err_t dap_probe_fpga_route_check(bool wide)
{
    ESP_LOGW(TAG, "=== fabric DAP route ===");

    if (dap_phy_fpga_attach() != ESP_OK) {
        dap_phy_fpga_log_status();
        dap_phy_fpga_use(false);
        return ESP_FAIL;
    }

    dap_probe_clear_error_state();
    dap_probe_set_rw_mode(true);

    uint32_t mcds = 0;
    if (dap_probe_enable_ocds() == ESP_OK &&
        dap_probe_read32(MCDS_ID, &mcds) == ESP_OK) {
        ESP_LOGW(TAG, "  miniMCDS ID = 0x%08" PRIX32 "%s", mcds,
                 mcds == MCDS_ID_VAL ? "  (expected)" : "  UNEXPECTED");
    } else {
        ESP_LOGW(TAG, "  miniMCDS ID unreadable");
    }

    uint32_t oifm = 0;
    if (dap_probe_read32(OIFM_ADDR, &oifm) == ESP_OK) {
        const unsigned mode = oifm & 7u;
        ESP_LOGW(TAG, "  OIFM = 0x%08" PRIX32 ", DAPMODE %u (%s)", oifm, mode,
                 (mode == 0 || mode == 3 || mode == 4) ? "wide allowed" : "reserved");
    }

    uint32_t head[8];
    if (dap_probe_blockread(CHECK_ADDR, head, 8) != ESP_OK) {
        ESP_LOGE(TAG, "  a short block read failed");
        dap_phy_fpga_log_status();
        dap_phy_fpga_use(false);
        return ESP_FAIL;
    }
    ESP_LOGW(TAG, "  block of 8: %08" PRIX32 " %08" PRIX32 " %08" PRIX32,
             head[0], head[1], head[2]);

    dap_fpga_block_read_sweep("narrow");

    esp_err_t err = ESP_OK;
    if (wide) {
        err = dap_wide_check();
    }

    ESP_LOGW(TAG, "=== fabric route %s ===", err == ESP_OK ? "verified" : "FAILED");
    return err;
}
