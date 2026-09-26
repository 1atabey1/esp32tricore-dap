/*
 * Sweeps and probes for a link that is not answering.
 */

#include <inttypes.h>
#include <string.h>

#include "board_profile.h"
#include "dap_frame.h"
#include "dap_phy.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "dap_probe_priv.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DAP";

esp_err_t dap_probe_dump_sync_reply(size_t window_bits)
{
    uint8_t     bits[160];
    dap_frame_t f;

    if (window_bits > sizeof(bits)) {
        window_bits = sizeof(bits);
    }
    if (!dap_phy_ready() || !dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 0)) {
        return ESP_FAIL;
    }

    dap_phy_idle_clocks(11, 0);
    dap_phy_write_frame(&f);
    dap_phy_turnaround_to_read();
    dap_phy_read_bits(bits, window_bits);
    dap_phy_turnaround_to_write();

    char text[168];
    size_t n = 0;
    for (size_t i = 0; i < window_bits && n < sizeof(text) - 1; i++) {
        text[n++] = bits[i] ? '1' : '0';
    }
    text[n] = '\0';
    ESP_LOGW(TAG, "sync reply window, %u bits raw:", (unsigned)window_bits);
    ESP_LOGW(TAG, "  %s", text);
    return ESP_OK;
}

esp_err_t dap_probe_clock_pin_search(void)
{
    /*
     * DAP1 is fixed on the bidirectional pin.  PC01 is register-driven and
     * PC04 (GPIO40) is the target's reset, so neither is tried as the clock.
     */
    static const struct { int pin; const char *label; } candidates[] = {
        { AEL_DAP0_PIN,     "GPIO47 -> PC02, J3 pin 23" },
    };

    ESP_LOGI(TAG, "--- clock pin search, data fixed on GPIO%d (PC03) ---", AEL_DAP1_PIN);

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        const dap_phy_cfg_t cfg = {
            .clk_pin  = candidates[i].pin,
            .dat_pin  = AEL_DAP1_PIN,
            .dir_pin  = AEL_DAP1_DIR_PIN,
            /* Do not touch the other candidate while it might be the clock. */
            .trst_pin = -1,
            .clock_hz = CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ,
        };
        if (dap_phy_init(&cfg) != ESP_OK) {
            continue;
        }

        dap_exchange_t x;
        const esp_err_t err = dap_probe_sync(&x);
        if (err == ESP_OK) {
            ESP_LOGW(TAG, "clock on %s ANSWERED: wait %d, reply 0x%08" PRIX64,
                     candidates[i].label, x.wait_cycles, x.reply);
            return ESP_OK;
        }
        ESP_LOGI(TAG, "clock on %s: silent", candidates[i].label);
    }

    /* Leave the PHY on the documented assignment. */
    dap_probe_init(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    return ESP_FAIL;
}

esp_err_t dap_probe_attach_sweep(void)
{
    static const uint32_t rates[]  = { 200000u, 500000u, 1000000u };
    static const size_t   idles[]  = { 8u, 64u, 256u };
    static const uint8_t  lens[]   = { 63u, 0u };

    int attempts = 0;
    int answered = 0;

    if (!dap_phy_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "--- attach sweep: rate x idle clocks x sync LEN ---");

    for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
        dap_phy_set_clock(rates[r]);
        for (size_t i = 0; i < sizeof(idles) / sizeof(idles[0]); i++) {
            for (size_t l = 0; l < sizeof(lens) / sizeof(lens[0]); l++) {
                dap_frame_t    f;
                dap_exchange_t x;

                dap_phy_idle_clocks(idles[i], 0);

                if (!dap_frame_build(&f, DAP_CMD_SYNC, lens[l], 0, 0)) {
                    continue;
                }
                attempts++;
                if (dap_probe_exchange(&f, 32, &x) == ESP_OK) {
                    answered++;
                    ESP_LOGW(TAG, "ANSWERED: %" PRIu32 " Hz, %zu idle clocks, "
                                  "LEN %u -> wait %d, reply 0x%08" PRIX64,
                             rates[r], idles[i], lens[l], x.wait_cycles, x.reply);
                }
            }
        }
    }

    ESP_LOGI(TAG, "--- sweep done: %d/%d combinations answered ---", answered, attempts);
    /* Leave the PHY where the caller expects it. */
    dap_phy_set_clock(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    return answered ? ESP_OK : ESP_FAIL;
}

void dap_probe_log_exchange(const char *step, const dap_exchange_t *x)
{
    if (x->timed_out) {
        ESP_LOGE(TAG, "%-22s sent 0x%05" PRIX64 " (%zu bits) -> %s within %d clocks",
                 step, x->sent_word, x->sent_bits,
                 x->idle_high ? "line stayed idle high, nothing driving it"
                              : "line held low, no start bit",
                 dap_probe_max_wait);
        return;
    }
    ESP_LOGI(TAG, "%-22s sent 0x%05" PRIX64 " (%zu bits) -> wait %d, reply 0x%08" PRIX64
                  " (%zu bits), crc 0x%02X %s",
             step, x->sent_word, x->sent_bits, x->wait_cycles, x->reply,
             x->reply_bits, x->reply_crc, x->crc_ok ? "residue-ok" : "residue-BAD");
}

esp_err_t dap_probe_attach_now(dap_exchange_t out[6])
{
    const uint64_t dapisc = ((uint64_t)DAP_DAPISC_SIGNATURE << 16) | DAP_DAPISC_VALUE;
    dap_frame_t f;

    if (!dap_phy_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    dap_phy_idle_clocks(11, 0);
    if (dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 0)) {
        dap_probe_exchange(&f, 32, &out[0]);
    }
    if (dap_frame_build(&f, DAP_CMD_DAPISC, 48, dapisc, 48)) {
        dap_probe_exchange(&f, 0, &out[1]);
    }
    if (dap_frame_build(&f, DAP_CMD_CLIENT_SET, 3, 1, 3)) {
        dap_probe_exchange(&f, 0, &out[2]);
    }
    if (dap_frame_build(&f, DAP_CMD_CLIENT_READ, 7,
                        dap_client_read_payload(DAP_IO_CLIENT_ID, 4), 7)) {
        dap_probe_exchange(&f, 16, &out[3]);
    }
    return ESP_OK;
}

esp_err_t dap_probe_block_throughput(void)
{
    /* 1 kB (256 words, the maximum) per telegram; baseline is a miniWiggler's 38 kB/s. */
    static const uint32_t rates[] = { 400000u, 1000000u, 2000000u, 4000000u,
                                      6000000u, 8000000u, 12000000u, 16000000u };
    static uint32_t buf[256];
    const int iterations = 8;

    ESP_LOGW(TAG, "--- block read throughput, 1 kB per telegram ---");
    for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
        dap_phy_set_clock(rates[r]);

        int ok = 0;
        const int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < iterations; i++) {
            if (dap_probe_blockread(0x70000000u, buf, 256) == ESP_OK) {
                ok++;
            } else {
                dap_probe_clear_error_state();
            }
        }
        const int64_t us = esp_timer_get_time() - t0;
        if (us > 0 && ok) {
            const int kbps = (int)((int64_t)ok * 1024 * 1000000 / us / 1024);
            ESP_LOGW(TAG, "  %7" PRIu32 " Hz: %d/%d blocks, %lld us -> %d kB/s",
                     rates[r], ok, iterations, (long long)us, kbps);
        } else {
            ESP_LOGW(TAG, "  %7" PRIu32 " Hz: no blocks completed", rates[r]);
        }
    }
    dap_phy_set_clock(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    return ESP_OK;
}

esp_err_t dap_probe_rate_test(void)
{
    /* CLIENT_ID is hard-wired to 0x0260, so every reply is self-checking. */
    static const uint32_t rates[] = { 400000u, 1000000u, 2000000u, 4000000u,
                                      6000000u, 8000000u, 12000000u, 16000000u };
    const int reads = 200;

    ESP_LOGW(TAG, "--- rate test: %d CLIENT_ID reads per rate ---", reads);

    for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
        dap_exchange_t x;
        int      good = 0;
        uint32_t crc_ok = 0;

        dap_phy_set_clock(rates[r]);

        const int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < reads; i++) {
            if (dap_probe_client_read(DAP_IO_CLIENT_ID, 4, 16, &x) == ESP_OK) {
                if (x.reply == DAP_CLIENT_ID_EXPECT) {
                    good++;
                }
                if (x.crc_ok) {
                    crc_ok++;
                }
            }
        }
        const int64_t us = esp_timer_get_time() - t0;

        /* 26-bit frame + 23-bit reply, plus lead-in and wait. */
        const int per_read_us = (int)(us / reads);
        ESP_LOGW(TAG, "  %7" PRIu32 " Hz: %3d/%d correct, %" PRIu32 " CRC ok, "
                      "%d us/read -> %d reads/s",
                 rates[r], good, reads, crc_ok, per_read_us,
                 per_read_us ? (int)(1000000 / per_read_us) : 0);
    }

    dap_phy_set_clock(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    return ESP_OK;
}

/* Send the real attach frames through the normal path and log each raw reply window. */
void dap_probe_dump_raw_attach(const char *what)
{
    dap_exchange_t x;

    ESP_LOGW(TAG, "--- raw windows, %s ---", what);
    dap_phy_idle_clocks(dap_probe_max_wait, 0);

    dap_probe_set_raw_window(48);
    dap_probe_sync(&x);
    dap_probe_client_set(1, &x);
    dap_probe_client_read(DAP_IO_CLIENT_ID, 4, 16, &x);
    dap_probe_set_raw_window(0);
}
