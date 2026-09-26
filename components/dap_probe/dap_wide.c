/*
 * DAP wide mode (DAPISC.MODE = 01B): DAP1 carries a frame's even bits, DAP2
 * (the target's P21.7) the odd ones.  dap_wide_enter/exit run it as a session;
 * dap_wide_check measures it for /api/dap_fpga?wide=1.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "dap_fpga_priv.h"
#include "dap_frame.h"
#include "dap_lock.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "dap_wide.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DAP_WIDE";

/* Calibration reference: a pattern written narrow into the second half of the
 * trace RAM (a trace session rewrites all of it), read back wide. */
#define REF_ADDR    0xB8001000u
#define REF_WORDS   64u

static bool s_active;

bool dap_wide_active(void)
{
    return s_active;
}

/* P21.7 must be an input (IOCR bit 4 clear): an output fights DAP2 through
 * the 22 ohm series resistor. */
static bool p21_7_is_input(void)
{
    uint32_t iocr4 = 0;

    if (dap_probe_read32(P21_IOCR4, &iocr4) != ESP_OK) {
        ESP_LOGE(TAG, "P21_IOCR4 unreadable");
        return false;
    }
    const uint32_t pc = (iocr4 >> IOCR4_SHIFT(DAP2_PIN)) & 0x1Fu;
    if (pc & IOCR_PP_OUT) {
        ESP_LOGE(TAG, "P21.7 is an output (PC 0x%02" PRIX32 "); wide mode refused", pc);
        return false;
    }
    ESP_LOGI(TAG, "P21.7 is an input (PC 0x%02" PRIX32 ")", pc);
    return true;
}

/* Reference data written narrow before the switch, compared wide. */
static uint32_t s_ref_block[REF_WORDS];

/* Every bit position toggling, both lines busy: an xorshift sequence. */
static esp_err_t write_reference(void)
{
    uint32_t x = 0x9E3779B9u;
    static uint32_t back[REF_WORDS];

    for (size_t i = 0; i < REF_WORDS; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        s_ref_block[i] = x;
    }
    /* Word writes: a block write drops its first parcel (see the flash loader). */
    for (size_t i = 0; i < REF_WORDS; i++) {
        if (dap_probe_write32(REF_ADDR + 4u * i, s_ref_block[i]) != ESP_OK) {
            return ESP_FAIL;
        }
    }
    if (dap_probe_blockread(REF_ADDR, back, REF_WORDS) != ESP_OK ||
        memcmp(back, s_ref_block, sizeof(back)) != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Does this tap pair carry real traffic: a word read with its CRC and a block
 * read, both matching what narrow mode read? */
static bool taps_carry_data(void)
{
    static uint32_t blk[REF_WORDS];
    uint32_t w = 0;

    const bool ok = dap_probe_set_rw_mode(true) == ESP_OK &&
                    dap_probe_read32(REF_ADDR + 4u, &w) == ESP_OK && w == s_ref_block[1] &&
                    dap_probe_blockread(REF_ADDR, blk, REF_WORDS) == ESP_OK &&
                    memcmp(blk, s_ref_block, sizeof(blk)) == 0;
    if (!ok) {
        dap_probe_clear_error_state();
    }
    return ok;
}

/*
 * Pick the capture taps at the session's clock (at the fast dividers each tap
 * is a large part of a bit).  First the wide sync (0xAAAAAAAA on each line,
 * reassembled WIDE_SYNC_EXPECT; its CRC cannot validate) over every pair; it is
 * periodic, so a pair one sample off can pass it too.  Then real reads over
 * the passing pairs, most passing neighbours first, until one returns the
 * reference data.
 */
static bool calibrate(uint8_t *tap1, uint8_t *tap2)
{
    bool ok[4][4] = { { false } };
    int  score[4][4] = { { 0 } };
    int  found = 0;

    for (uint8_t t1 = 0; t1 < 4; t1++) {
        char line[40];
        int n = snprintf(line, sizeof(line), "  sync taps DAP1 %u:", t1);
        for (uint8_t t2 = 0; t2 < 4; t2++) {
            dap_exchange_t x = {0};
            dap_phy_fpga_set_skew(t1, t2);
            (void)dap_probe_sync(&x);
            ok[t1][t2] = (x.reply == WIDE_SYNC_EXPECT);
            found += ok[t1][t2];
            n += snprintf(line + n, sizeof(line) - (size_t)n, " %c", ok[t1][t2] ? 'D' : '.');
        }
        ESP_LOGI(TAG, "%s", line);
    }
    if (!found) {
        return false;
    }
    for (int t1 = 0; t1 < 4; t1++) {
        for (int t2 = 0; t2 < 4; t2++) {
            for (int d1 = -1; d1 <= 1 && ok[t1][t2]; d1++) {
                for (int d2 = -1; d2 <= 1; d2++) {
                    const int a = t1 + d1, b = t2 + d2;
                    score[t1][t2] += (a >= 0 && a < 4 && b >= 0 && b < 4 && ok[a][b]);
                }
            }
        }
    }

    /* Failing reads are expected while searching; keep them out of the log. */
    const esp_log_level_t was = esp_log_level_get("DAP");
    esp_log_level_set("DAP", ESP_LOG_ERROR);
    bool done = false;
    for (int want = 9; want >= 1 && !done; want--) {
        for (int t1 = 0; t1 < 4 && !done; t1++) {
            for (int t2 = 0; t2 < 4 && !done; t2++) {
                if (score[t1][t2] != want) {
                    continue;
                }
                dap_phy_fpga_set_skew((uint8_t)t1, (uint8_t)t2);
                if (taps_carry_data()) {
                    *tap1 = (uint8_t)t1;
                    *tap2 = (uint8_t)t2;
                    done = true;
                } else {
                    ESP_LOGI(TAG, "  taps DAP1 %d DAP2 %d pass sync but not data", t1, t2);
                }
            }
        }
    }
    esp_log_level_set("DAP", was);
    return done;
}

static esp_err_t enter_locked(uint8_t div)
{
    uint16_t now = 0;
    uint8_t tap1 = 0, tap2 = 0;

    if (s_active) {
        dap_wide_exit();                 /* re-enter to calibrate at this clock */
    }
    if (!dap_phy_fpga_in_use()) {
        return ESP_ERR_INVALID_STATE;
    }
    dap_phy_fpga_set_div(div);
    dap_probe_clear_error_state();
    if (!p21_7_is_input()) {
        return ESP_ERR_INVALID_STATE;
    }
    /* What calibration must read back once wide; the trace RAM needs OCDS. */
    if (dap_probe_enable_ocds() != ESP_OK || write_reference() != ESP_OK) {
        ESP_LOGE(TAG, "the calibration pattern did not read back narrow at div %u",
                 (unsigned)div);
        return ESP_FAIL;
    }

    /* Past the dapisc telegrams the device swallows after an attach. */
    dap_dapisc_prime();

    /* The mode change goes out narrow; its reply already comes back wide. */
    const uint16_t widev = DAPISC_VALUE | (DAPISC_MODE_WIDE << DAPISC_MODE_SHIFT);
    const uint64_t sig = ((uint64_t)DAPISC_SIGNATURE << 16) | widev;
    dap_phy_fpga_set_rx_wide(true);
    dap_dapisc_rx_wide = true;
    const bool switched = dap_dapisc_write_read(48, sig, 48, &now) &&
                          ((now >> DAPISC_MODE_SHIFT) & 3u) == DAPISC_MODE_WIDE;
    dap_dapisc_rx_wide = false;
    dap_phy_fpga_set_rx_wide(false);
    if (!switched || dap_phy_fpga_set_wide(true) != ESP_OK) {
        ESP_LOGE(TAG, "the device did not switch to wide mode (DAPISC 0x%04X)", now);
        dap_dapisc_narrow();
        return ESP_FAIL;
    }
    if (!calibrate(&tap1, &tap2)) {
        ESP_LOGE(TAG, "no capture tap pair carries data at div %u", (unsigned)div);
        dap_dapisc_narrow();
        dap_phy_fpga_set_skew(0, 0);
        return ESP_FAIL;
    }
    dap_phy_fpga_set_skew(tap1, tap2);
    s_active = true;
    ESP_LOGI(TAG, "wide mode on at div %u, taps DAP1 %u DAP2 %u", (unsigned)div, tap1, tap2);
    return ESP_OK;
}

esp_err_t dap_wide_enter(uint8_t div)
{
    dap_lock();
    const esp_err_t err = enter_locked(div);
    dap_unlock();
    return err;
}

void dap_wide_exit(void)
{
    dap_lock();
    if (s_active) {
        dap_dapisc_narrow();
        dap_phy_fpga_set_skew(0, 0);
        s_active = false;
        ESP_LOGI(TAG, "back to narrow mode");
    }
    dap_unlock();
}

esp_err_t dap_wide_check(void)
{
    ESP_LOGW(TAG, "--- DAP wide mode ---");
    const esp_err_t err = dap_wide_enter(5);
    if (err != ESP_OK) {
        return err;
    }
    dap_fpga_block_read_sweep("wide");
    dap_wide_exit();
    dap_phy_fpga_set_div(1);
    return ESP_OK;
}
