/*
 * Wide-mode bring-up, stage by stage.
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
 * Wide mode end to end: get the part to route the line, tell the device,
 * switch the fabric, find the taps, prove a real transaction, measure it.
 */
void dap_wide_route_check(void)
{
    /*
     * Opt-in, and off by default.
     *
     * Everything below drives DAP2, and on this bench that reliably takes the
     * board down after one or two frames - see the note on stage 6.  The
     * narrow path is solid and fast, the BMP target and the trace drain run on
     * it, and a route check that leaves the board needing a reset every time
     * it is run is worse than one that does not test an unfinished feature.
     * So this only runs when /api/dap_fpga is asked for a stage.
     */
    if (dap_wide_stage == 0) {
        ESP_LOGW(TAG, "--- DAP wide mode: skipped (add ?stage=N to run it) ---");
        return;
    }

    ESP_LOGW(TAG, "--- DAP wide mode ---");

    /*
     * Trailing clocks, if the caller asked for a different number.
     *
     * A candidate explanation for the contention worth one frame to test: the
     * trailing clocks after a reply are what give the device time to let go of
     * the line, and the default of one was swept for DAP1 alone.  If DAP2 takes
     * longer to release, every wide frame this end sends starts while the
     * device is still driving it.
     */
    if (dap_wide_trail >= 0) {
        dap_phy_fpga_set_trail((uint8_t)dap_wide_trail);
        ESP_LOGW(TAG, "  trailing clocks set to %d for this run", dap_wide_trail);
    }

    /*
     * OIFM first, for the record rather than as a gate.
     *
     * DAPMODE 000B is the reset value and it already permits both two-pin
     * standard and three-pin wide mode - it is not a two-pin strapping, which
     * an earlier reading of it here assumed and spent a while acting on.  So
     * there is nothing to configure and nothing to restore; the value is
     * logged because a part that did have it set otherwise would change what
     * everything below means.
     */
    {
        uint32_t oifm = 0;
        if (dap_probe_read32(OIFM_ADDR, &oifm) == ESP_OK) {
            ESP_LOGW(TAG, "  OIFM 0x%08" PRIX32 ", DAPMODE %u, PADCTL %u",
                     oifm, (unsigned)(oifm & 7u), (unsigned)((oifm >> 12) & 3u));
        }
    }

    /*
     * Before any protocol theory: is the line even connected?
     *
     * This needs the device narrow and the bus reachable, which is true here
     * and stops being true the moment the mode changes, so it goes first.
     */
    dap_wide_dap2_wiring_probe();
    WIDE_STAGE(1, false);

    /*
     * What the lines do with the fabric wide and the device still narrow.
     *
     * Informational, not a gate.  The transmitter drives real data onto DAP2
     * during this frame, so the witness bits it leaves behind answer the one
     * question that has to be settled before any protocol theory is worth
     * testing: whether our own driver reaches the pad and comes back through
     * the input path at all.
     */
    (void)dap_wide_dap2_line_free();
    WIDE_STAGE(2, false);

    /*
     * Baseline: write the mode the device is already in and read it back.  If
     * that does not come back cleanly the readback is not trustworthy here and
     * nothing below it means much, so it is established before the interesting
     * write rather than after.
     */
    dap_dapisc_prime();

    uint16_t now = 0;
    const uint16_t narrow = DAPISC_VALUE |
                            (DAPISC_MODE_NARROW << DAPISC_MODE_SHIFT);
    const bool base_ok = dap_dapisc_write_read(16, narrow, 16, &now);
    ESP_LOGW(TAG, "  dapisc narrow baseline: wrote 0x%04X -> read 0x%04X%s, "
                  "MODE %u", (unsigned)narrow, (unsigned)now,
             base_ok ? "" : " NO VALID REPLY",
             (unsigned)((now >> DAPISC_MODE_SHIFT) & 3u));
    dap_probe_clear_error_state();

    /*
     * Take the pin off the application before asking for wide mode.
     *
     * Order matters: while the port module drives P21.7 the interface cannot,
     * so a mode change made first produces a device that is wide and mute.
     */
    dap_wide_halt_application();

    if (!dap_wide_p21_7_release(&dap_wide_iocr4_saved)) {
        ESP_LOGE(TAG, "  the target still owns P21.7, so DAP2 cannot be driven "
                      "from here - two push-pull drivers on one net through a "
                      "22 ohm resistor is about 150 mA, and enough frames of it "
                      "brown this board out");
        dap_wide_revert();
        return;
    }

    WIDE_STAGE(3, false);

    /*
     * The mode change, as a handshake rather than a fire-and-forget write.
     *
     * This is the step that was wrong for a long time.  The short dapisc form
     * does change the register - the device starts interleaving its replies
     * the moment it has read one, which is how 0x0334 came back for a register
     * that should read 0x0F10 - but changing the register is not the same as
     * the physical multiplexer being reconfigured.  The long form carries the
     * 32-bit signature, which the spec describes as the protection against
     * accidental *configuration changes*, and it answers: three DAP0 clocks
     * after the host's CRC6 the device sends a start bit, the sixteen updated
     * bits, and a CRC6.
     *
     * Sending it and not clocking that reply leaves the handshake half done.
     * So this one goes out with an ordinary hunting window and its answer is
     * read and checked, which is the whole difference from send_dapisc_long().
     *
     * It goes out narrow, because narrow is all the device understands until
     * it has read it.
     */
    const uint16_t widev = DAPISC_VALUE |
                           (DAPISC_MODE_WIDE << DAPISC_MODE_SHIFT);
    const uint64_t sig = ((uint64_t)DAPISC_SIGNATURE << 16) | widev;

    /*
     * Receive wide, transmit narrow, for this one telegram.
     *
     * The note above had it right: the device interleaves its replies the
     * moment it has read the telegram, so the answer comes back on both lines
     * even though the command had to go out narrow.  Sampling DAP1 alone is
     * what turned 0x0F10 into 0x0334 - half the bits, read as if they were all
     * of them, which is also why the CRC never checked out.
     */
    dap_phy_fpga_set_rx_wide(true);
    dap_dapisc_rx_wide = true;
    const bool hs = dap_dapisc_write_read(48, sig, 48, &now);
    dap_dapisc_rx_wide = false;
    dap_phy_fpga_set_rx_wide(false);
    ESP_LOGW(TAG, "  dapisc wide (long form, handshake): wrote 0x%04X -> "
                  "read 0x%04X%s, MODE %u", (unsigned)widev, (unsigned)now,
             hs ? "" : " NO VALID REPLY",
             (unsigned)((now >> DAPISC_MODE_SHIFT) & 3u));
    dap_wide_report_lines("after the handshake:");

    /*
     * Does the device still answer narrow?
     *
     * If it does, the mux has not switched and the handshake did not take,
     * which is worth knowing before sixteen tap combinations are tried against
     * a device that is still on one line.  If it does not, something changed -
     * and the sweep below is the right next question.
     */
    dap_exchange_t probe = {0};
    const bool still_narrow = dap_probe_attach(&probe, 3) == ESP_OK &&
                              probe.reply == 0xAAAAAAAAu;
    ESP_LOGW(TAG, "  narrow sync after the handshake: %s (0x%08" PRIX64 ")",
             still_narrow ? "still answers - the mux did NOT switch"
                          : "no longer answers", probe.reply);

    WIDE_STAGE(4, true);

    if (dap_phy_fpga_set_wide(true) != ESP_OK) {
        ESP_LOGE(TAG, "  the fabric would not switch to wide mode");
        dap_wide_revert();
        return;
    }

    /*
     * The fabric's own framing first, across all sixteen tap pairs.
     *
     * Worth trying before the candidate rules now that the start bit arrives on
     * both lines: that says the device is transmitting wide and the remaining
     * question is where in the bit the two lines are sampled, which is exactly
     * what the taps are.  The candidates only matter if no tap pair works,
     * because then the frames being sent are not being understood at all.
     */
    WIDE_STAGE(5, true);

    uint8_t tap1 = 0, tap2 = 0;

    /*
     * Stage 6: exactly one wide frame.
     *
     * Everything up to and including switching the fabric to wide survives;
     * sixteen wide frames do not.  Whether one does is the difference between
     * something cumulative - two drivers on the DAP2 net, heating up over a
     * sweep - and something that is wrong on the very first frame, and those
     * need opposite fixes.
     */
    /*
     * Stage 6: exactly one wide frame, and nothing else in the request.
     *
     * The count matters and is the whole reason this is its own stage.  One
     * frame is survivable; a request that sent two - a sync and then a raw
     * window to look at the lines - took the board down as reliably as a sweep
     * of sixteen did.  A threshold that low is not software: driving DAP2 while
     * the device is also driving it puts two push-pull outputs across a 22 ohm
     * series resistor, and the rail does not survive much of that.
     *
     * So this sends one frame and reports what came back, and the sweep across
     * tap pairs is done one HTTP request at a time from the host, which leaves
     * the device back in narrow mode between attempts.
     */
    if (dap_wide_stage == 6) {
        /*
         * dap_exchange_t::reply is 64 bits, and every log of it has to say so.
         *
         * Printed with PRIX32 it hands vsnprintf four bytes of an eight-byte
         * argument and every argument after it comes from the wrong slot: the
         * trailing %s here picked up the value's high half, which is zero, and
         * strlen(NULL) panicked the probe.  That is the whole of the "one wide
         * frame is survivable, sixteen are not" folklore in the notes above -
         * the tap sweep logs the same way, so it died on whichever frame first
         * reached the log, and it looked like something cumulative on the DAP2
         * net.  Nothing was wrong with the hardware.
         */
        dap_exchange_t one = {0};

        dap_phy_fpga_set_skew((uint8_t)dap_wide_tap1, (uint8_t)dap_wide_tap2);
        const esp_err_t e = dap_probe_sync(&one);
        ESP_LOGW(TAG, "  taps %d,%d: sync %s, reply 0x%08" PRIX64 "%s",
                 dap_wide_tap1, dap_wide_tap2, esp_err_to_name(e), one.reply,
                 one.reply == WIDE_SYNC_EXPECT ? "   <== CORRECT" : "");
        dap_wide_revert();
        return;
    }

    /*
     * Stage 9: the device wide, this end never driving DAP2.
     *
     * This is the configuration worth having even if the other never works.
     * Everything the throughput is for travels device to probe - a block read
     * is a short command and a kilobyte of reply - so a link that receives on
     * two lines and transmits on one would collect the whole benefit while
     * leaving DAP2 an input at this end, and no amount of turnaround
     * disagreement can then put two drivers on that net.
     *
     * Whether the device will answer a narrow command while its own protocol
     * engine is wide is the open question, and it is one frame to find out.
     */
    if (dap_wide_stage == 9) {
        uint32_t raw = 0;
        uint16_t waited = 0;
        char l1[17], l2[17];

        dap_phy_fpga_set_wide(false);      /* transmit narrow */
        dap_phy_fpga_set_rx_wide(true);    /* sample both lines */
        dap_phy_fpga_set_skew((uint8_t)dap_wide_tap1, (uint8_t)dap_wide_tap2);

        dap_phy_fpga_set_raw_window(true);
        dap_phy_fpga_exchange(0x10u, 63, 0, 0, 32, &raw, &waited);
        dap_phy_fpga_set_raw_window(false);
        dap_phy_fpga_set_rx_wide(false);

        for (int i = 0; i < 16; i++) {
            l1[i] = (raw & (1u << (2 * i)))     ? '1' : '0';
            l2[i] = (raw & (1u << (2 * i + 1))) ? '1' : '0';
        }
        l1[16] = l2[16] = '\0';
        ESP_LOGW(TAG, "  narrow command, wide device, taps %d,%d:",
                 dap_wide_tap1, dap_wide_tap2);
        ESP_LOGW(TAG, "    DAP1 %s", l1);
        ESP_LOGW(TAG, "    DAP2 %s", l2);
        dap_wide_report_lines("after it:");

        dap_wide_revert();
        return;
    }

    const bool tap_ok = dap_wide_calibrate(&tap1, &tap2);
    WIDE_STAGE(7, true);

    if (!tap_ok) {
        ESP_LOGW(TAG, "  no tap pair worked with the assembled framing; "
                      "trying the candidate rules");
        dap_wide_report_lines("after the tap sweep:");

        wide_framing_t rule = WF_PAD_CRC;
        if (!dap_wide_framing_sweep(&rule, &tap1, &tap2)) {
            ESP_LOGE(TAG, "  no framing rule and tap pair got sync answered");
            dap_wide_report_lines("after the framing sweep:");
            dap_wide_revert();
            return;
        }
        ESP_LOGW(TAG, "  framing \"%s\" is the one the device wants",
                 dap_wide_framing_name(rule));
    }

    WIDE_STAGE(8, true);

    dap_phy_fpga_set_skew(tap1, tap2);
    ESP_LOGW(TAG, "  taps DAP1 %u, DAP2 %u", tap1, tap2);

    /* Now the register can be read for real, on both lines. */
    const bool wide_ok = dap_dapisc_write_read(16, widev, 16, &now);
    ESP_LOGW(TAG, "  dapisc read wide: 0x%04X%s, MODE %u", (unsigned)now,
             wide_ok ? "" : " (no valid reply)",
             (unsigned)((now >> DAPISC_MODE_SHIFT) & 3u));

    /*
     * sync answering is not the same as the link working: sync is answered from
     * any state and carries no address, so it says nothing about whether a
     * command with a payload gets through.  That was exactly the failure this
     * project spent a session on in narrow mode, so wide mode gets the same
     * check - attach properly, then read a constant off the bus.
     */
    dap_probe_clear_error_state();
    dap_exchange_t x = {0}, id = {0};

    /* One step per line: three commands collapsed into one condition reported
     * the same "will not attach" whichever of them failed, and they fail for
     * different reasons - sync carries no payload, client_set carries three
     * bits, client_read carries an address and draws a real reply. */
    const esp_err_t a_err = dap_probe_attach(&x, 3);
    ESP_LOGW(TAG, "  wide attach: sync %s (0x%08" PRIX64 ")",
             esp_err_to_name(a_err), x.reply);

    const esp_err_t s_err = (a_err == ESP_OK) ? dap_probe_client_set(1, &x)
                                              : ESP_FAIL;
    ESP_LOGW(TAG, "  wide attach: client_set %s", esp_err_to_name(s_err));

    const esp_err_t r_err = (s_err == ESP_OK)
        ? dap_probe_client_read(DAP_IO_CLIENT_ID, 4, 16, &id) : ESP_FAIL;
    ESP_LOGW(TAG, "  wide attach: client_read %s -> 0x%04X (want 0x%04X)",
             esp_err_to_name(r_err), (unsigned)id.reply,
             (unsigned)DAP_CLIENT_ID_EXPECT);

    /*
     * A failed re-attach is not the end of the run.
     *
     * The client selected during the narrow attach survives the mode change -
     * the telegram changes the framing, not the device's state - so a block
     * read can go out without one, and whether *that* works is the question
     * the whole exercise is here to answer.  Aborting on client_set meant the
     * throughput sweep below had never once been reached.
     */
    const bool attached = (r_err == ESP_OK && id.reply == DAP_CLIENT_ID_EXPECT);

    if (!attached) {
        ESP_LOGW(TAG, "  wide re-attach did not take; going on to the bus "
                      "reads with the client the narrow attach selected");
    }

    dap_probe_set_rw_mode(true);
    uint32_t mcds_id = 0;
    const esp_err_t rd = dap_probe_read32(0xFB718008u, &mcds_id);
    ESP_LOGW(TAG, "  wide bus read: %s -> 0x%08" PRIX32 " (want 0x00D6C007)",
             esp_err_to_name(rd), mcds_id);

    /* Try the block read regardless: it is a different command with an even
     * payload, and it is the one that carries the throughput. */
    static uint32_t probe_buf[8];
    dap_probe_clear_error_state();
    const esp_err_t br = dap_probe_blockread(CHECK_ADDR, probe_buf, 8);
    ESP_LOGW(TAG, "  wide block read of 8 words: %s -> "
                  "%08" PRIX32 " %08" PRIX32 " %08" PRIX32,
             esp_err_to_name(br), probe_buf[0], probe_buf[1], probe_buf[2]);

    if (rd != ESP_OK || mcds_id != 0x00D6C007u) {
        if (br != ESP_OK) {
            ESP_LOGE(TAG, "  neither a single read nor a block read works wide");
            dap_wide_revert();
            return;
        }
    }
    ESP_LOGW(TAG, "  attached wide: CLIENT_ID 0x%04X, miniMCDS ID "
                  "0x%08" PRIX32, (unsigned)id.reply, mcds_id);

    /*
     * The same block-read sweep as narrow mode, at the same dividers, so the
     * two numbers are comparable.  Wide mode halves the clocks a block takes,
     * so the wire term should halve; the drain and the host overhead do not
     * move, so the gain is less than twice and the size of that gap is the
     * interesting part.
     */
    static uint32_t buf[256];
    static const uint8_t divs[] = { 11, 5, 3, 2, 1, 0 };
    const int iterations = 8;

    ESP_LOGW(TAG, "  block read throughput, wide:");
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

        if (ok && us > 0) {
            const int kbps = (int)((int64_t)ok * 1024 * 1000000 / us / 1024);
            ESP_LOGW(TAG, "    div %2u (%4u kHz): %d/%d blocks in %6lld us "
                          "-> %3d kB/s", divs[d],
                     (unsigned)(48000u / (2u * (divs[d] + 1u))),
                     ok, iterations, (long long)us, kbps);
        } else {
            ESP_LOGW(TAG, "    div %2u: no full blocks completed", divs[d]);
        }
    }

    /*
     * Left in narrow mode with OIFM as it was found.
     *
     * Everything above this in the firmware - the BMP target, the trace drain -
     * runs narrow, and leaving the device in a mode only this function knows
     * how to speak would break all of it.  Wide mode becomes the default when
     * the calibration happens at attach rather than in a check.
     */
    dap_wide_revert();
    dap_phy_fpga_set_div(1);
    ESP_LOGW(TAG, "  back to narrow mode");
}

