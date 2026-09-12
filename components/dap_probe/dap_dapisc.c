/*
 * The dapisc telegram: encode, send, and decode the reply.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "dap_fpga_priv.h"
#include "dap_frame.h"
#include "dap_phy.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tricore.h"

static const char *TAG = "DAP_FPGA_RT";

/* ------------------------------------------------------------------------ */
/* Wide mode                                                                 */
/* ------------------------------------------------------------------------ */

/*
 * The target's DAP2 pin, as an ordinary port pin.
 *
 * DAP2 is P21.7 on this part, and while the interface is in two-pin mode the
 * port module owns it - which is the whole reason wide mode did not work: the
 * application configures it as a push-pull output and holds it low, so
 * Cerberus can put its protocol engine in wide mode, and does, but cannot
 * drive the pad.
 */

/* IOCR4 holds four five-bit fields for pins 4..7, pin n at bit 3 + 8*(n-4).
 * 0x10 is push-pull general-purpose output; 0x00 is a plain input. */


/* Defined below; dap_wide_revert needs them and comes first. */
void dap_wide_halt_application(void);
void dap_wide_resume_application(void);


/*
 * DAPISC field positions, from the register layout: MODE is bits 5:4, and 01B
 * there is wide mode - DAP1 carrying the even bits of a frame and DAP2 the odd
 * ones, two bits per DAP0 clock.
 *
 * Worth being explicit about which bits are *not* being touched, because this
 * register is shared with the wait-state timeout: MAXWAIT8 is bits 12:8 and
 * writing it as zero disables the timeout, which turns an internal bus lockup
 * into a probe that hangs forever.  DAPISC_VALUE's 0x0F00 is MAXWAIT8 = 15
 * with MW8E clear - 120 DAP0 clocks - and the mode bits are ORed into that
 * rather than replacing it.
 */


/*
 * What a wide sync reply reassembles to.
 *
 * The device sends the 0xAAAAAAAA training pattern on *each* line, not one
 * pattern split across the two, so weaving them back together doubles every
 * bit: 0,1,0,1... becomes 0,0,1,1,0,0,1,1..., which is 0xCCCCCCCC.
 *
 * This was read the other way round for a long time - the note here used to say
 * alternating bits put all the zeros on one line and all the ones on the other,
 * which would have made 0xAAAAAAAA the wide answer too.  That is what made a
 * correct wide link look like a broken one.
 *
 * Its CRC is not checked with it.  Six CRC bits sent per line reassemble into
 * twelve, which is not a CRC6 over the doubled payload and does not validate;
 * a real telegram's payload is genuinely interleaved and its CRC does.  So the
 * pattern is the whole of the evidence here, and a block read is what proves
 * the link.
 */

/*
 * Both DAPISC forms.  Long carries the 32-bit signature and is what the cold
 * attach uses; short is for reconfiguring a link that is already up.
 *
 * Both go out with the reply window raw: with a hunt enabled the undriven line
 * reads idle high, the fabric latches a phantom start bit and issues trailing
 * clocks on top of it, and the *next* frame is the one that gets lost.
 */
/* dapisc is command 0x11.  The register spec gives 0x09, which is
 * client_blockwrite: sent that way the telegram desynchronises the link.
 */

static void send_dapisc_long(uint16_t value)
{
    const uint64_t data = ((uint64_t)DAPISC_SIGNATURE << 16) | value;
    uint32_t reply = 0;
    uint16_t waited = 0;

    dap_phy_fpga_set_raw_window(true);
    dap_phy_fpga_exchange(DAPISC_CMD, 48, data, 48, 2, &reply, &waited);
    dap_phy_fpga_set_raw_window(false);
}

void dap_dapisc_send(void)
{
    send_dapisc_long(DAPISC_VALUE);
}

static void send_dapisc_short(uint16_t value)
{
    uint32_t reply = 0;
    uint16_t waited = 0;

    dap_phy_fpga_set_raw_window(true);
    dap_phy_fpga_exchange(DAPISC_CMD, 16, value, 16, 2, &reply, &waited);
    dap_phy_fpga_set_raw_window(false);
}

/*
 * The same write, keeping the reply.
 *
 * Both forms answer with the register as it now stands - start bit, the
 * updated sixteen bits, then a CRC6 - which makes the telegram self-verifying
 * in the one direction that matters.  Only useful when both ends agree on the
 * framing; the telegram that *changes* the framing cannot be read back this
 * way, which cost a debugging round and is written up at the call site.
 *
 * The window is taken raw and decoded here rather than handed to the fabric's
 * hunt.  The device answers this telegram correctly - the reply carries the
 * register value and a CRC6, and a raw dump shows it plainly - but the fabric
 * rejected it, and every dapisc read came back "no valid reply" for as long as
 * this existed.  Nothing noticed, because attach does not need the reply: the
 * write lands either way, and the register's reset value is usable.  It only
 * surfaced when wide mode needed to *confirm* that MODE had changed.
 */
/*
 * Whether the window being decoded was sampled wide.
 *
 * Set around the one telegram whose reply comes back interleaved while the
 * fabric is otherwise narrow; see the note at the handshake.  A flag rather
 * than a parameter because it belongs to the sampling, not to the caller's
 * telegram - the command still goes out narrow.
 *
 * Once the fabric is wide outright, every reply is interleaved, so the decode
 * has to ask it too.  Missing that read a perfectly good 0x0F10 back as 0x1E21
 * for a second time, in the register read that exists to confirm the mode.
 */
bool dap_dapisc_rx_wide;

static bool dapisc_decode(uint32_t raw, uint16_t *now)
{
    uint8_t bits[32];
    int start = -1;

    for (int i = 0; i < 32; i++) {
        bits[i] = (uint8_t)((raw >> i) & 1u);
        if (start < 0 && bits[i]) {
            start = i;            /* the reply's start bit */
        }
    }
    /*
     * The start bit is two bits wide in an interleaved window.
     *
     * Wide framing sends the start bit on both lines in the same DAP0 clock,
     * so once the two are woven back into one stream it occupies two
     * positions, not one.  Skipping only one shifts the whole value up by a
     * bit and leaves the start bit in bit 0 - which is how a perfectly good
     * 0x0F10 reply read back as 0x1E21 and failed its CRC.
     */
    const int skip = (dap_dapisc_rx_wide || dap_phy_fpga_is_wide()) ? 2 : 1;

    /* The value and its CRC6 both have to fit in what was sampled. */
    if (start < 0 || start + skip + 16 + 6 > 32) {
        return false;
    }

    uint16_t value = 0;
    for (int i = 0; i < 16; i++) {          /* LSB first, like every field */
        value |= (uint16_t)bits[start + skip + i] << i;
    }
    *now = value;

    /* The CRC covers the sixteen value bits; the manual's CalcCrc() leaves a
     * fixed residue when it is run over the value and the CRC together. */
    return dap_crc6_residue_ok(&bits[start + skip], 16 + 6);
}

/*
 * Wake the dapisc reply path up after an attach.
 *
 * Measured: the first two dapisc telegrams following an attach draw an empty
 * reply window, and every one after that is answered immediately - a second
 * call gets its answer on its first send.  So it is a one-time run-up, not a
 * per-telegram delay.
 *
 * That is the whole reason the wide-mode handshake never read back.  The long
 * form cannot simply be repeated - it is the telegram that changes the framing,
 * so a second copy would arrive in a mode the device has just left - which put
 * it permanently in the swallowed pair.  Two short writes of the value the
 * register is going to hold anyway cost nothing and move it out.
 */
bool dap_dapisc_write_read(uint8_t len_field, uint64_t data, size_t dbits,
                              uint16_t *now);

void dap_dapisc_prime(void)
{
    uint16_t ignored = 0;

    for (int i = 0; i < 2; i++) {
        dap_dapisc_write_read(16, DAPISC_VALUE, 16, &ignored);
    }
}

bool dap_dapisc_write_read(uint8_t len_field, uint64_t data, size_t dbits,
                              uint16_t *now)
{
    /*
     * Up to two sends, and only for the short form.
     *
     * The first dapisc telegram after an attach draws an empty window; a second
     * one, identical, comes back with the register and a good CRC.  Measured
     * both ways round - it was the raw probe's *second* send that first showed
     * a reply at all, and moving the raw window into the first send moved the
     * silence with it.
     *
     * The long form is deliberately left single-shot.  It is the telegram that
     * changes the framing, so a second copy would arrive in a mode the device
     * has just left, and the reply it draws could not be read anyway.
     */
    const int sends = (len_field == 16) ? 4 : 1;
    uint32_t raw = 0;
    uint16_t waited = 0;
    bool ok = false;

    *now = 0;
    for (int attempt = 0; attempt < sends && !ok; attempt++) {
        raw = 0;
        dap_phy_fpga_set_raw_window(true);
        const esp_err_t err = dap_phy_fpga_exchange(DAPISC_CMD, len_field,
                                                    data, dbits, 32, &raw,
                                                    &waited);
        dap_phy_fpga_set_raw_window(false);
        if (err == ESP_OK) {
            ok = dapisc_decode(raw, now);
        }
        if (!ok && attempt + 1 == sends) {
            ESP_LOGW(TAG, "  dapisc unanswered after %d sends, last raw "
                          "0x%08X (err %d)", sends, (unsigned)raw, (int)err);
        }
    }
    return ok;
}

/*
 * What do the two data lines idle at?
 *
 * The fabric can be put in wide mode while the *device* is still on two pins.
 * The frame that goes out is then one the device does not understand, so it
 * neither replies nor changes mode - but the reply window is still clocked and
 * still sampled on both lines, and in wide mode the raw window interleaves
 * them into the payload exactly as a real reply would be.  So one raw read
 * separates into the two lines and reports what each sits at when nothing is
 * driving it, at no cost: no telegram, no mode change, nothing to recover.
 *
 *   DAP2 all zero -> the net is held low.  In two-pin mode that pin is an
 *                    ordinary port pin and the target owns it.
 *   DAP2 all ones -> the net floats, so nothing drives it from either end and
 *                    wide mode has somewhere to put the odd bits.
 *
 * An earlier version answered this by adding a register to the fabric that
 * drove DAP2 from the host and read the pad back.  It worked, and it is what
 * established that package pin 34 is the right pad in both directions - but
 * overriding the pad's output enable cost about four megahertz of fabric clock
 * wherever the readback was decoded, and at 48.5 MHz the block read started
 * losing bytes at the slow dividers.  Not worth the clock when the window
 * gives the part that matters for free.
 */
/*
 * The line witnesses from the last exchange, decoded.
 *
 * Printed as two characters per phase - the levels that were actually seen -
 * because "did not toggle" and "was never sampled" look the same in a single
 * bit and mean entirely different things here.
 */
void dap_wide_report_lines(const char *what)
{
    const uint8_t w = dap_phy_fpga_line_witness();

    ESP_LOGW(TAG, "  %-24s DAP2 while we drove %s%s | DAP2 while target drove "
                  "%s%s | DAP1 while target drove %s%s%s", what,
             (w & (1u << 1)) ? "0" : "-", (w & (1u << 2)) ? "1" : "-",
             (w & (1u << 3)) ? "0" : "-", (w & (1u << 4)) ? "1" : "-",
             (w & (1u << 5)) ? "0" : "-", (w & (1u << 6)) ? "1" : "-",
             (w & (1u << 0)) ? "  [start bit on DAP2]" : "");
}

bool dap_wide_dap2_line_free(void)
{
    uint32_t reply = 0;
    uint16_t waited = 0;
    char     l1[17], l2[17];
    unsigned ones = 0;

    /* Receive wide, transmit narrow: DAP2 is read and never driven, which
     * matters because the target still owns that pin here. */
    dap_phy_fpga_set_rx_wide(true);
    dap_phy_fpga_set_raw_window(true);
    dap_phy_fpga_exchange(0x10u, 63, 0, 0, 32, &reply, &waited);
    dap_phy_fpga_set_raw_window(false);
    dap_phy_fpga_set_rx_wide(false);

    for (int i = 0; i < 16; i++) {
        l1[i] = (reply & (1u << (2 * i)))     ? '1' : '0';
        l2[i] = (reply & (1u << (2 * i + 1))) ? '1' : '0';
        if (reply & (1u << (2 * i + 1))) {
            ones++;
        }
    }
    l1[16] = l2[16] = '\0';

    ESP_LOGW(TAG, "  idle levels, fabric wide and device not: DAP1 %s "
                  "DAP2 %s", l1, l2);
    dap_wide_report_lines("wide frame, narrow device:");
    return ones == 16;
}

/*
 * Put the device back on two pins and get the link working again.
 *
 * The telegram goes out FIRST, in whatever mode the fabric is currently in,
 * and the fabric follows.  Getting that backwards is a trap worth naming:
 * switching the fabric to narrow first means the telegram asking the device to
 * go narrow is itself sent in a framing the device - still wide - cannot
 * parse, so it is never received and the link stays broken.  The device only
 * ever changes mode on a telegram it understood, so the probe has to speak the
 * old mode to ask for the new one.  That is true in both directions.
 */

void dap_wide_revert(void)
{
    dap_exchange_t x;

    send_dapisc_short(DAPISC_VALUE |
                      (DAPISC_MODE_NARROW << DAPISC_MODE_SHIFT));
    dap_phy_fpga_set_wide(false);
    dap_probe_clear_error_state();

    if (dap_probe_attach(&x, 3) != ESP_OK || x.reply != 0xAAAAAAAAu) {
        ESP_LOGW(TAG, "  narrow mode did not come back from a telegram; "
                      "resetting the target");
        dap_phy_fpga_set_trst(true);
        vTaskDelay(pdMS_TO_TICKS(2));
        dap_phy_fpga_set_trst(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        dap_probe_clear_error_state();
        (void)dap_probe_attach(&x, 3);
    }

    dap_wide_resume_application();

    if (dap_wide_iocr4_dirty) {
        dap_wide_iocr4_dirty = false;
        if (dap_probe_write32(P21_IOCR4, dap_wide_iocr4_saved) == ESP_OK) {
            ESP_LOGW(TAG, "  P21.7 given back to the application");
        } else {
            ESP_LOGE(TAG, "  P21.7 could not be given back; IOCR4 should be "
                          "0x%08" PRIX32, dap_wide_iocr4_saved);
        }
        dap_probe_clear_error_state();
    }
}

