/*
 * The dapisc telegram (CMD 0x11): configure the DAP interface and read DAPISC
 * back.  Replies are taken from a raw window and decoded here.
 */

#include <inttypes.h>
#include <string.h>

#include "dap_fpga_priv.h"
#include "dap_frame.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DAP_FPGA_RT";

bool dap_dapisc_rx_wide;

/* Long form (value + 0x4ABBAF53), fire and forget, for the cold attach.  Raw
 * window, so an idle-high line is not taken as a start bit. */
void dap_dapisc_send(void)
{
    const uint64_t data = ((uint64_t)DAPISC_SIGNATURE << 16) | DAPISC_VALUE;
    uint32_t reply = 0;
    uint16_t waited = 0;

    dap_phy_fpga_set_raw_window(true);
    dap_phy_fpga_exchange(DAPISC_CMD, 48, data, 48, 2, &reply, &waited);
    dap_phy_fpga_set_raw_window(false);
}

/*
 * Reply: start bit, 16 bits LSB first, CRC6.  In a wide window the start bit
 * arrives on both lines at once and so fills two positions.
 */
static bool decode(uint32_t raw, uint16_t *now)
{
    uint8_t bits[32];
    int start = -1;

    for (int i = 0; i < 32; i++) {
        bits[i] = (uint8_t)((raw >> i) & 1u);
        if (start < 0 && bits[i]) {
            start = i;
        }
    }
    const int skip = (dap_dapisc_rx_wide || dap_phy_fpga_is_wide()) ? 2 : 1;
    if (start < 0 || start + skip + 16 + 6 > 32) {
        return false;
    }

    uint16_t value = 0;
    for (int i = 0; i < 16; i++) {
        value |= (uint16_t)bits[start + skip + i] << i;
    }
    *now = value;
    return dap_crc6_residue_ok(&bits[start + skip], 16 + 6);
}

/*
 * Write DAPISC and read back what it now holds.  The short form is retried:
 * the device swallows the first dapisc telegrams after an attach.  The long
 * form changes the framing, so it goes out once.
 */
bool dap_dapisc_write_read(uint8_t len_field, uint64_t data, size_t dbits,
                           uint16_t *now)
{
    const int sends = (len_field == 16) ? 4 : 1;
    bool ok = false;

    *now = 0;
    for (int attempt = 0; attempt < sends && !ok; attempt++) {
        uint32_t raw = 0;
        uint16_t waited = 0;

        dap_phy_fpga_set_raw_window(true);
        const esp_err_t err = dap_phy_fpga_exchange(DAPISC_CMD, len_field, data,
                                                    dbits, 32, &raw, &waited);
        dap_phy_fpga_set_raw_window(false);
        ok = (err == ESP_OK) && decode(raw, now);
    }
    return ok;
}

/* Two short writes of the resting value, so the next telegram - the long-form
 * mode change, which cannot be retried - is past the swallowed pair. */
void dap_dapisc_prime(void)
{
    uint16_t ignored = 0;

    for (int i = 0; i < 2; i++) {
        dap_dapisc_write_read(16, DAPISC_VALUE, 16, &ignored);
    }
}

/*
 * Back to two pins.  The telegram goes out in the current framing first and
 * the fabric follows: the device only changes mode on a telegram it can parse.
 * Resets the target if narrow does not come back.
 */
bool dap_dapisc_narrow(void)
{
    const uint64_t narrow = DAPISC_VALUE | (DAPISC_MODE_NARROW << DAPISC_MODE_SHIFT);
    uint32_t reply = 0;
    uint16_t waited = 0;
    dap_exchange_t x;

    dap_phy_fpga_set_raw_window(true);
    dap_phy_fpga_exchange(DAPISC_CMD, 16, narrow, 16, 2, &reply, &waited);
    dap_phy_fpga_set_raw_window(false);
    dap_phy_fpga_set_wide(false);
    dap_probe_clear_error_state();

    if (dap_probe_attach(&x, 3) == ESP_OK && x.reply == 0xAAAAAAAAu) {
        return true;
    }
    ESP_LOGW(TAG, "narrow mode did not come back; resetting the target");
    dap_phy_fpga_set_trst(true);
    vTaskDelay(pdMS_TO_TICKS(2));
    dap_phy_fpga_set_trst(false);
    vTaskDelay(pdMS_TO_TICKS(10));
    dap_probe_clear_error_state();
    return dap_probe_attach(&x, 3) == ESP_OK;
}
