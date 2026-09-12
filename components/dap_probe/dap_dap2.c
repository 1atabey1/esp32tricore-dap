/*
 * The target's DAP2 pin (P21.7), and whether it is wired through.
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

/*
 * The target application's own setting for P21.7, put back on every exit.
 *
 * It configured that pin deliberately and is entitled to it again once the
 * probe is done borrowing it.
 */
uint32_t dap_wide_iocr4_saved;
bool     dap_wide_iocr4_dirty;

/* ------------------------------------------------------------------------ */
/* Is DAP2 wired to the target at all?                                        */
/* ------------------------------------------------------------------------ */

/*
 * DAP2 is P21.7 on this part.  While the interface is in two-pin mode that pin
 * is an ordinary port pin, which makes the wiring answerable from this end
 * without a scope: the probe still has full bus access, so it can drive P21.7
 * from the target side and look at what its own DAP2 pad reads.
 *
 * Three outcomes, and only one of them is a protocol problem:
 *
 *   the target pad follows and this board sees it  -> the net is good
 *   the target pad follows and this board sees nothing -> the two are not
 *       connected, and no amount of framing work will help
 *   the target pad does not follow -> the pad is not under port control and
 *       the test proved nothing either way
 *
 * The control - reading the target's own input register back at each level -
 * is what separates the second from the third, and without it the first
 * version of this test pointed confidently at the wrong conclusion on the
 * wrong port.
 *
 * P21.7 is restored exactly as it was found.
 */

/*
 * What DAP2 reads right now, as sixteen samples.
 *
 * Reuses the wide raw window: with the fabric wide the reply window is sampled
 * on both lines and interleaved into the payload, so the odd bit positions are
 * DAP2.  Returns how many of them read high.  Costs one frame the device does
 * not understand and does not answer, which is harmless and changes nothing.
 */
static unsigned dap2_level_samples(void)
{
    uint32_t reply = 0;
    uint16_t waited = 0;
    unsigned ones = 0;

    /* Receive wide, transmit narrow: DAP2 is read and never driven, which
     * matters because the target still owns that pin here. */
    dap_phy_fpga_set_rx_wide(true);
    dap_phy_fpga_set_raw_window(true);
    dap_phy_fpga_exchange(0x10u, 63, 0, 0, 32, &reply, &waited);
    dap_phy_fpga_set_raw_window(false);
    dap_phy_fpga_set_rx_wide(false);

    for (int i = 0; i < 16; i++) {
        if (reply & (1u << (2 * i + 1))) {
            ones++;
        }
    }
    return ones;
}

void dap_wide_dap2_wiring_probe(void)
{
    uint32_t iocr4 = 0, pdisc = 0, out = 0, in = 0;

    if (dap_probe_read32(P21_IOCR4, &iocr4) != ESP_OK ||
        dap_probe_read32(P21_PDISC, &pdisc) != ESP_OK ||
        dap_probe_read32(P21_OUT,   &out)   != ESP_OK ||
        dap_probe_read32(P21_IN,    &in)    != ESP_OK) {
        ESP_LOGW(TAG, "  port 21 unreadable, so the DAP2 wiring is untested");
        return;
    }

    ESP_LOGW(TAG, "  P21 IOCR4 0x%08" PRIX32 " PDISC 0x%08" PRIX32
                  " OUT 0x%08" PRIX32 " IN 0x%08" PRIX32, iocr4, pdisc, out, in);
    ESP_LOGW(TAG, "    P21.7 (DAP2): PC 0x%02X, %s",
             (unsigned)((iocr4 >> IOCR4_SHIFT(DAP2_PIN)) & 0x1Fu),
             (pdisc & (1u << DAP2_PIN)) ? "pad disabled" : "pad enabled");

    const uint32_t mask   = 0x1Fu << IOCR4_SHIFT(DAP2_PIN);
    const uint32_t as_out = (iocr4 & ~mask) |
                            (IOCR_PP_OUT << IOCR4_SHIFT(DAP2_PIN));

    if (dap_probe_write32(P21_IOCR4, as_out) != ESP_OK) {
        ESP_LOGW(TAG, "    P21.7: IOCR4 would not take the write");
        return;
    }

    uint32_t in_lo = 0, in_hi = 0;

    /*
     * The scan across every other BANK0 pad that used to run here is gone: it
     * answered its question - package pin 34 is the right pad and nothing else
     * sees the target's DAP2 - and the sixteen input pads it needed were worth
     * more to the block-write FIFO.  It is in the history if the wiring ever
     * changes.
     */
    dap_probe_write32(P21_OUT, out & ~(1u << DAP2_PIN));
    dap_probe_read32(P21_IN, &in_lo);
    const unsigned lo = dap2_level_samples();

    dap_probe_write32(P21_OUT, out | (1u << DAP2_PIN));
    dap_probe_read32(P21_IN, &in_hi);
    const unsigned hi = dap2_level_samples();

    /* Put the pin back before judging anything. */
    dap_probe_write32(P21_OUT, out);
    dap_probe_write32(P21_IOCR4, iocr4);
    dap_probe_clear_error_state();

    const bool moved = ((in_lo >> DAP2_PIN) & 1u) == 0u &&
                       ((in_hi >> DAP2_PIN) & 1u) == 1u;

    ESP_LOGW(TAG, "    P21.7: target pad %s (IN %u then %u) -> our DAP2 high "
                  "in %u/16 then %u/16 samples%s",
             moved ? "followed" : "did NOT follow",
             (unsigned)((in_lo >> DAP2_PIN) & 1u),
             (unsigned)((in_hi >> DAP2_PIN) & 1u), lo, hi,
             (moved && lo == 0 && hi == 16)
                 ? "   <== the net is good"
                 : moved ? "   <== the pin moved and this board saw nothing"
                         : "   <== inconclusive: the pad is not port-controlled");
}

/*
 * Stop the application before borrowing its pin.
 *
 * Releasing P21.7 by writing IOCR4 is not enough on a running target: the
 * application owns that pin and puts it back, so the release survives for
 * however long it takes the next configuration pass to undo it - which is why
 * the wiring probe kept reporting "the target pad did not follow" on some runs
 * and not others, and why a wide frame sent afterwards found the pin driven
 * again and put two outputs across a 22 ohm resistor.
 *
 * Halting the cores stops that.  Cerberus stays up - it is what does the
 * halting - so the DAP link is unaffected, and a pin the application is no
 * longer executing cannot be reconfigured behind us.
 *
 * The cores are resumed again on the way out.  Halting a target to test a
 * probe feature and leaving it stopped would be its own kind of rude.
 */
static uint8_t s_halted_mask;

void dap_wide_halt_application(void)
{
    s_halted_mask = 0;

    for (int core = 0; core < TRICORE_MAX_CORES; core++) {
        if (!tricore_core_present(core)) {
            continue;
        }
        if (tricore_halt(core, core, 200) == ESP_OK) {
            s_halted_mask |= (uint8_t)(1u << core);
        } else {
            ESP_LOGW(TAG, "  CPU%d would not halt", core);
        }
    }
    ESP_LOGW(TAG, "  halted cores 0x%02X so the application stops "
                  "reconfiguring P21.7", s_halted_mask);
}

void dap_wide_resume_application(void)
{
    for (int core = 0; core < TRICORE_MAX_CORES; core++) {
        if (s_halted_mask & (1u << core)) {
            tricore_resume(core, 200);
        }
    }
    if (s_halted_mask) {
        ESP_LOGW(TAG, "  cores resumed");
    }
    s_halted_mask = 0;
}

/*
 * Hand P21.7 back so the debug interface can drive it.
 *
 * This is the thing that was actually wrong, and the port registers said so
 * from the first time they were read: IOCR4 reads 0x80808080, which is PC7 =
 * 0x10 - push-pull general-purpose output - so the target's own application
 * owns the DAP2 pin and holds it low.  Cerberus can put its protocol engine in
 * wide mode, and does, but it cannot drive a pad the port module is driving, so
 * the odd bits go nowhere and every wide frame after that is unanswerable.
 *
 * Every symptom this chased for a long time follows from that one fact: the net
 * idling low with nothing of ours driving it, DAP2 never going high while the
 * target had the lines, and a reply that reads 0x0334 - exactly the even bits
 * of a wide 0x0F10 - because DAP1 carried half of a frame whose other half was
 * being held down.
 *
 * Setting PC7 to an input mode stops the port driving and leaves the pin to the
 * interface.  The previous value is returned so it can be put back: the target's
 * application configured that pin deliberately and is entitled to it again.
 */
/*
 * What P21.7 has to become, and what is still a guess.
 *
 * The requirement is exact: PC7's bit 4 is the direction, so any 0x1x value
 * leaves the port's push-pull driver on and that is the contention which browns
 * the board out.  Turning that driver off is what hands the pad to the OCDS,
 * and every input encoding does it.
 *
 * Which input encoding is the open part.  0x00 is the bare minimum - stop
 * driving, no pull device - and leaves the line floating through every
 * half-duplex turnaround.  0x02 adds a pull-up, which is the better guess on
 * two counts: DAP lines idle high, which is what the receiver's idle_high
 * detection keys off, and the pin read as DAP0 in the Port 22 dump is itself
 * 0x02.  Neither has been confirmed against a working wide link, so it is a
 * parameter rather than a constant - /api/dap_fpga?pc=N picks it, and the
 * default is the one with the better argument behind it.
 */

static uint8_t s_dap2_pc = IOCR_IN_PULLUP;

void dap_probe_fpga_dap2_mode(int pc)
{
    /* Refuse an output mode outright: it is the one setting that can damage
     * something, and a typo in a query string should not reach the pad. */
    if (pc >= 0 && (pc & 0x10) == 0) {
        s_dap2_pc = (uint8_t)(pc & 0x1F);
    } else if (pc >= 0) {
        ESP_LOGE(TAG, "  refusing PC 0x%02X for P21.7: that is an output mode",
                 (unsigned)pc);
    }
}

/*
 * Returns whether it is safe for this end to drive DAP2.
 *
 * That is a different question from whether anything was changed, and
 * conflating the two is dangerous here: "already an input" needs no change and
 * is safe, while "could not read or write IOCR4" also needs no change and is
 * emphatically not.  The restore flag is set here rather than by the caller so
 * the two cannot drift apart.
 */
bool dap_wide_p21_7_release(uint32_t *saved_iocr4)
{
    if (dap_probe_read32(P21_IOCR4, saved_iocr4) != ESP_OK) {
        ESP_LOGW(TAG, "  P21 IOCR4 unreadable; the pin's owner is unknown, so "
                      "DAP2 will not be driven from here");
        return false;
    }

    const unsigned pc = (unsigned)((*saved_iocr4 >> IOCR4_SHIFT(DAP2_PIN)) & 0x1Fu);
    if ((pc & 0x10u) == 0u) {
        ESP_LOGW(TAG, "  P21.7 is already an input (PC 0x%02X); nothing to "
                      "release", pc);
        return true;
    }

    const uint32_t mask = 0x1Fu << IOCR4_SHIFT(DAP2_PIN);
    const uint32_t as_in = (*saved_iocr4 & ~mask) |
                           ((uint32_t)s_dap2_pc << IOCR4_SHIFT(DAP2_PIN));

    if (dap_probe_write32(P21_IOCR4, as_in) != ESP_OK) {
        ESP_LOGW(TAG, "  P21 IOCR4 would not take the write");
        return false;
    }
    dap_probe_clear_error_state();

    uint32_t now = 0;
    if (dap_probe_read32(P21_IOCR4, &now) != ESP_OK) {
        ESP_LOGW(TAG, "  P21.7 write not confirmed; not driving DAP2");
        return false;
    }

    const unsigned pc_now = (unsigned)((now >> IOCR4_SHIFT(DAP2_PIN)) & 0x1Fu);
    ESP_LOGW(TAG, "  P21.7 set to PC 0x%02X (input, %s)", (unsigned)s_dap2_pc,
             s_dap2_pc == IOCR_IN_PULLUP ? "pull-up" :
             s_dap2_pc == IOCR_IN_NOPULL ? "no pull device" : "other");
    ESP_LOGW(TAG, "  P21.7 released for the interface: IOCR4 0x%08" PRIX32
                  " -> 0x%08" PRIX32 " (PC 0x%02X -> 0x%02X)",
             *saved_iocr4, now, pc, pc_now);

    if ((pc_now & 0x10u) != 0u) {
        ESP_LOGE(TAG, "  P21.7 is still an output; not driving DAP2 into it");
        return false;
    }
    dap_wide_iocr4_dirty = true;
    return true;
}

