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

/*
 * Forget the link, so the next dap_phy_fpga_init() probes again.  Call this
 * whenever the FPGA is reconfigured: the fabric's registers go back to their
 * reset values and the host's picture of them is stale, which shows up as a
 * route that worked a moment ago returning zeros.
 */
void dap_phy_fpga_invalidate(void);

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

/*
 * Idle clocks issued before each frame.  Two by default, which is what the
 * CPU path settles on - but the notes there record eleven as the value that
 * was actually measured on this target, so it is swept rather than assumed.
 */
esp_err_t dap_phy_fpga_set_lead(uint8_t clocks);

/* Assert (low) or release the target reset line. */
esp_err_t dap_phy_fpga_set_trst(bool asserted);

/*
 * Keep the whole reply window instead of hunting for a start bit in it.
 *
 * A frame that draws no reply reports one fact - the hunt ran out - and that
 * cannot separate a line the target is holding low from one nothing is
 * driving, nor show a start bit that arrived at an unexpected moment.  With
 * this set, `reply` is the first `reply_bits` clocks of the window verbatim,
 * first clock in bit 0, and no CRC is read or checked.
 *
 * This is the fabric's version of dap_probe_set_raw_window(), which is what
 * settled the same question on the CPU path.
 */
esp_err_t dap_phy_fpga_set_raw_window(bool enable);

/*
 * Wide mode: two bits per DAP0 clock, the even bits of a frame on DAP1 and the
 * odd ones on DAP2.
 *
 * This is the probe's half only - it changes how the fabric frames and
 * samples and says nothing to the device, which is told separately by a DAPISC
 * telegram with MODE = 01B.  The ordering is forced: that telegram has to go
 * out in narrow mode, because narrow is the only framing the device
 * understands until it has read the telegram.  So enable this after the
 * telegram is acknowledged, and disable it before talking to a device that
 * might have been reset since.
 */
esp_err_t dap_phy_fpga_set_wide(bool enable);
bool      dap_phy_fpga_is_wide(void);

/*
 * Where in the window each line is sampled, in fabric clocks, 0 to 3.
 *
 * Tap 0 on both is exactly what narrow mode has always done.  The taps exist
 * because the silicon does not promise DAP1 and DAP2 leave the pads together:
 * at the fastest divider a bit period is two fabric clocks, so one clock of
 * skew between the lines is half a bit.  Sweep them against a reply whose
 * value is known - sync answers 0xAAAAAAAA - rather than guessing from the
 * board.
 */
esp_err_t dap_phy_fpga_set_skew(uint8_t dap1_tap, uint8_t dap2_tap);

/*
 * Did the last reply's start bit arrive on DAP2 at the same sample as on DAP1?
 *
 * The start bit is the one bit the device drives on both lines together, so it
 * is the only direct evidence that the two taps agree.  A reply that fails
 * this is still delivered - the point of the bit is to let a sweep see which
 * settings work, and refusing the frame would hide that.
 */
bool      dap_phy_fpga_last_aligned(void);

/*
 * What levels each data line was seen at during the last exchange, split by
 * who was driving.
 *
 * Bit 0 is the start-bit alignment above; 1 and 2 are DAP2 seen low and high
 * while the probe was transmitting; 3 and 4 the same while the target had the
 * lines; 5 and 6 DAP1 while the target had them.  The point is to separate
 * three failures a reply value cannot tell apart: a DAP2 our own driver never
 * reaches, a DAP2 the target never drives, and a live line read at the wrong
 * phase.
 */
uint8_t   dap_phy_fpga_line_witness(void);

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

/*
 * Bring the fabric up and attach the target through it, leaving exchanges
 * routed to the fabric on success.
 *
 * The sequence is not obvious and was expensive to find, so it lives here
 * rather than in whichever caller needed it first: sync, then the LEN-48
 * DAPISC clocked without a start-bit hunt, then an error-state clear, a
 * resync, the client select and CLIENT_ID - retried, because the first attempt
 * after the FPGA is configured fails about as often as it succeeds.
 *
 * Sync alone is not an attach: it is the resynchronisation command and the
 * device answers it from any state, so it proves much less than it appears to.
 * Without the DAPISC every later command is ignored; without the error-state
 * clear, client_set works and client_read times out every time.
 *
 * Returns ESP_OK only when the target's hard-wired CLIENT_ID came back.
 */
esp_err_t dap_phy_fpga_attach(void);

#ifdef __cplusplus
}
#endif
