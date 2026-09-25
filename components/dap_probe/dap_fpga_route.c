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

#include "dap_fpga_priv.h"
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
