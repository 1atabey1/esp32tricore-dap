/*
 * DAP wide mode (DAPISC.MODE = 01B): DAP1 carries a frame's even bits, DAP2
 * (the target's P21.7) the odd ones.  Brings the link up wide, proves it with
 * real bus reads, measures block-read throughput, and returns to narrow -
 * everything else in the firmware runs narrow.
 */

#include <inttypes.h>
#include <stdio.h>

#include "dap_fpga_priv.h"
#include "dap_frame.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tricore.h"

static const char *TAG = "DAP_FPGA_RT";

#define HALT_LINE   1               /* line 0 does not halt */
#define MCDS_ID     0xFB718008u
#define MCDS_ID_VAL 0x00D6C007u

static uint8_t  s_halted;           /* cores this file halted */
static uint32_t s_iocr4_saved;
static bool     s_iocr4_dirty;

/* -- P21.7 ------------------------------------------------------------------ */

/* The application owns P21.7 and would reconfigure it behind us; halt it. */
static void halt_application(void)
{
    s_halted = 0;
    for (int core = 0; core < TRICORE_MAX_CORES; core++) {
        if (tricore_core_present(core) && tricore_core_started(core) &&
            !tricore_is_halted(core) &&
            tricore_halt(core, HALT_LINE, 200) == ESP_OK) {
            s_halted |= (uint8_t)(1u << core);
        }
    }
}

static void resume_application(void)
{
    for (int core = 0; core < TRICORE_MAX_CORES; core++) {
        if (s_halted & (1u << core)) {
            tricore_resume(core, 200);
        }
    }
    s_halted = 0;
}

/*
 * Make P21.7 an input (with pull-up) so the interface can drive it.  A
 * push-pull output there fights DAP2 through a 22 ohm resistor.  Returns
 * whether it is safe to drive DAP2.
 */
static bool release_p21_7(void)
{
    const uint32_t mask = 0x1Fu << IOCR4_SHIFT(DAP2_PIN);
    uint32_t now = 0;

    if (dap_probe_read32(P21_IOCR4, &s_iocr4_saved) != ESP_OK) {
        return false;
    }
    if (!((s_iocr4_saved >> IOCR4_SHIFT(DAP2_PIN)) & IOCR_PP_OUT)) {
        return true;                            /* already an input */
    }
    const uint32_t as_in = (s_iocr4_saved & ~mask) |
                           (IOCR_IN_PULLUP << IOCR4_SHIFT(DAP2_PIN));
    if (dap_probe_write32(P21_IOCR4, as_in) != ESP_OK ||
        dap_probe_read32(P21_IOCR4, &now) != ESP_OK ||
        ((now >> IOCR4_SHIFT(DAP2_PIN)) & IOCR_PP_OUT)) {
        return false;
    }
    s_iocr4_dirty = true;
    ESP_LOGW(TAG, "  P21.7 made an input for DAP2 (IOCR4 0x%08" PRIX32 ")", now);
    return true;
}

/* Narrow again, the application running, P21.7 as the application had it. */
static void revert(void)
{
    dap_dapisc_narrow();
    resume_application();
    if (s_iocr4_dirty) {
        s_iocr4_dirty = false;
        dap_probe_write32(P21_IOCR4, s_iocr4_saved);
        dap_probe_clear_error_state();
    }
    dap_phy_fpga_set_div(1);
}

/* -- capture taps ----------------------------------------------------------- */

/*
 * Find a pair of capture taps (where each line is sampled) that reads a wide
 * sync correctly.  The reply is the 0xAAAAAAAA training pattern on each line,
 * so it reassembles to WIDE_SYNC_EXPECT; its CRC cannot validate and is not
 * part of the test.
 */
static bool calibrate(uint8_t *tap1, uint8_t *tap2)
{
    bool found = false;

    ESP_LOGW(TAG, "  capture taps (D = sync reads 0x%08X):", WIDE_SYNC_EXPECT);
    for (uint8_t t1 = 0; t1 < 4; t1++) {
        char line[48];
        int n = snprintf(line, sizeof(line), "    DAP1 tap %u:", t1);

        for (uint8_t t2 = 0; t2 < 4; t2++) {
            dap_exchange_t x = {0};

            dap_phy_fpga_set_skew(t1, t2);
            vTaskDelay(pdMS_TO_TICKS(5));
            (void)dap_probe_sync(&x);
            const bool ok = (x.reply == WIDE_SYNC_EXPECT);
            n += snprintf(line + n, sizeof(line) - (size_t)n, " %c", ok ? 'D' : '.');
            if (ok && !found) {
                found = true;
                *tap1 = t1;
                *tap2 = t2;
            }
        }
        ESP_LOGW(TAG, "%s", line);
    }
    return found;
}

/* -- bring-up --------------------------------------------------------------- */

esp_err_t dap_wide_check(void)
{
    uint16_t now = 0;
    uint8_t tap1 = 0, tap2 = 0;

    ESP_LOGW(TAG, "--- DAP wide mode ---");

    /* Baseline in narrow, past the dapisc telegrams the device swallows. */
    dap_dapisc_prime();
    const uint16_t narrowv = DAPISC_VALUE | (DAPISC_MODE_NARROW << DAPISC_MODE_SHIFT);
    if (!dap_dapisc_write_read(16, narrowv, 16, &now)) {
        ESP_LOGE(TAG, "  DAPISC does not read back narrow");
        return ESP_FAIL;
    }
    dap_probe_clear_error_state();

    halt_application();
    if (!release_p21_7()) {
        ESP_LOGE(TAG, "  P21.7 is still driven by the target; DAP2 unusable");
        revert();
        return ESP_FAIL;
    }

    /* The mode change goes out narrow; its reply already comes back wide. */
    const uint16_t widev = DAPISC_VALUE | (DAPISC_MODE_WIDE << DAPISC_MODE_SHIFT);
    const uint64_t sig = ((uint64_t)DAPISC_SIGNATURE << 16) | widev;
    dap_phy_fpga_set_rx_wide(true);
    dap_dapisc_rx_wide = true;
    const bool switched = dap_dapisc_write_read(48, sig, 48, &now) &&
                          ((now >> DAPISC_MODE_SHIFT) & 3u) == DAPISC_MODE_WIDE;
    dap_dapisc_rx_wide = false;
    dap_phy_fpga_set_rx_wide(false);
    ESP_LOGW(TAG, "  handshake: DAPISC 0x%04X, MODE %u", (unsigned)now,
             (unsigned)((now >> DAPISC_MODE_SHIFT) & 3u));
    if (!switched || dap_phy_fpga_set_wide(true) != ESP_OK) {
        ESP_LOGE(TAG, "  the device did not switch to wide mode");
        revert();
        return ESP_FAIL;
    }

    if (!calibrate(&tap1, &tap2)) {
        ESP_LOGE(TAG, "  no capture tap pair reads the wide sync");
        revert();
        return ESP_FAIL;
    }
    dap_phy_fpga_set_skew(tap1, tap2);
    ESP_LOGW(TAG, "  taps DAP1 %u, DAP2 %u", tap1, tap2);

    /* Real transactions: DAPISC over both lines, a bus read, a block read.
     * The client selected in narrow mode carries over; client_set, with its
     * odd 3-bit payload, does not answer wide and is not needed. */
    uint32_t mcds = 0;
    const bool reg = dap_dapisc_write_read(16, widev, 16, &now);
    dap_probe_set_rw_mode(true);
    const bool bus = dap_probe_read32(MCDS_ID, &mcds) == ESP_OK && mcds == MCDS_ID_VAL;
    ESP_LOGW(TAG, "  wide: DAPISC %s 0x%04X, miniMCDS ID 0x%08" PRIX32 " %s",
             reg ? "reads" : "does not read", (unsigned)now, mcds,
             bus ? "(correct)" : "(WRONG)");
    if (!bus) {
        revert();
        return ESP_FAIL;
    }

    dap_fpga_block_read_sweep("wide");
    revert();
    ESP_LOGW(TAG, "  back to narrow mode");
    return ESP_OK;
}
