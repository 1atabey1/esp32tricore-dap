/*
 * The FPGA DAP master, driven from the ESP32 over SPI.
 *
 * This is not a third dap_phy backend in the sense the other two are.  The
 * bit-bang and GP-SPI paths implement the bit-level primitives - write a frame,
 * clock in some bits - and the transaction layer above assembles exchanges out
 * of them.  The FPGA does a whole exchange by itself, so what it replaces is
 * exchange(), not the PHY.
 *
 * That is also the entire point.  A 1 kB block read is 256 parcels, each a
 * start-bit hunt and 32 clocks; done from here that measured 453 kB/s and the
 * limit was the software loop rather than the wire.  With the fabric issuing
 * the parcels, the host writes a command, says how many parcels, and collects
 * the answer as one DMA burst - it is not in the per-parcel loop at all.
 *
 * Requires the DAP-only bitstream (fpga/dap_master).  Under the stock image
 * these registers do not exist and every read comes back as whatever the
 * logic-analyser fabric happens to put on the wire, so dap_phy_fpga_probe()
 * checks for something only the DAP image can answer before anything trusts it.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bring the link up: add a device on the bus the FPGA already sits on, and
 * confirm the DAP image is loaded rather than the stock one.
 *
 * The clock is capped at 6 MHz because the fabric runs at 24 MHz and its SPI
 * slave synchronises an asynchronous SCK, so it needs sysclk >= 4x SCK.  Going
 * faster does not degrade gracefully - it drops bits silently.
 */
esp_err_t dap_phy_fpga_init(void);

/* True once init has succeeded and the DAP bitstream answered. */
bool dap_phy_fpga_ready(void);

/* Route exchanges through the fabric (true) or leave them to the CPU (false). */
void dap_phy_fpga_use(bool enable);
bool dap_phy_fpga_in_use(void);

/*
 * Bit rate, as the fabric divider.  The DAP clock is 24 MHz / (2 * (div + 1)),
 * so 5 gives 2 MHz and 0 would give 12 - but the receiver samples the target
 * through a two-flop synchroniser, so a half period shorter than a few fabric
 * clocks puts the sample in the neighbouring bit.  Values below 2 are refused.
 */
esp_err_t dap_phy_fpga_set_div(uint8_t div);

/*
 * The two numbers the device turned out to be fussy about, kept settable for
 * the same reason they are registers in the fabric: both were established by
 * sweeping them on hardware and a different target may want different ones.
 */
esp_err_t dap_phy_fpga_set_trail(uint8_t clocks);
esp_err_t dap_phy_fpga_set_maxwait(uint16_t clocks);

/* Assert (low) or release the target reset line. */
esp_err_t dap_phy_fpga_set_trst(bool asserted);

/*
 * One command and its reply.  `reply_bits` of zero means a bare acknowledge.
 * Returns ESP_OK only when the fabric reported a reply with a good CRC.
 */
esp_err_t dap_phy_fpga_exchange(uint8_t cmd, uint8_t len_field,
                                uint64_t data, size_t data_bits,
                                size_t reply_bits,
                                uint32_t *reply, uint16_t *wait_cycles);

/*
 * A block read: one command frame, then `count` parcels the fabric clocks out
 * by itself, collected in a single burst.  `count` is 1..256.
 */
esp_err_t dap_phy_fpga_blockread(uint64_t cmd_payload, size_t payload_bits,
                                 uint32_t *words, size_t count);

/* Whatever the fabric last reported, for diagnosis. */
void dap_phy_fpga_log_status(void);

#ifdef __cplusplus
}
#endif
