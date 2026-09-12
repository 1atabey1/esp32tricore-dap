/*
 * Does the fabric DAP master actually work?
 *
 * Checked in an order where a failure names its own cause.  There is no
 * cross-check against the CPU path here, and that is not an omission: the DAP
 * bitstream replaces the stock one, and the stock one is what carries Port C
 * through to the CPU's own DAP pins.  With the fabric loaded the CPU path reads
 * nothing but 0xFFFFFFFF, so a comparison between the two is not available in
 * one session - an earlier version tried it and compared against garbage.
 *
 * Self-checking values stand in for it, and are better evidence anyway:
 *
 *   sync            -> 0xAAAAAAAA with a valid CRC
 *   CLIENT_ID       -> 0x0260, hard-wired in the silicon
 *   miniMCDS ID     -> 0x00D6C007 after the OCDS enable
 *
 * Each is a constant the target must produce, so agreeing with it by accident
 * is much harder than producing plausible-looking bytes.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "dap_phy.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DAP_FPGA_RT";

#define CHECK_ADDR   0x70000000u    /* DSPR: readable without OCDS */

/*
 * IOClient instruction 0xF reads CLIENT_ID, which is hard-wired to 0x0260 - the
 * value that first proved this project was talking to Cerberus at all.  Both
 * live in dap_probe.c rather than a header; repeated here rather than widening
 * that file's interface for one diagnostic.
 */
#define IO_CLIENT_ID      0xFu
#define CLIENT_ID_EXPECT  0x0260u

/*
 * Clocks issued after a reply, with the target still driving.
 *
 * The device is fussy about this and the right value was never settled
 * analytically - too few and the reply is cut short, too many and the *next*
 * command is ignored, which presents as an intermittent failure of an unrelated
 * command.  The bit-banged path gets it right by accident; the fabric has it as
 * a register so it can be found the same way everything else here was, by
 * trying values and seeing which one the silicon accepts.
 */
static const uint8_t k_trail_candidates[] = { 1, 2, 0, 3, 4, 6, 8 };

/* sync, then select the client and read its hard-wired ID. */
static bool try_trail(uint8_t trail, uint16_t *id_out)
{
    dap_exchange_t x;

    dap_phy_fpga_set_trail(trail);
    dap_probe_clear_error_state();

    if (dap_probe_attach(&x, 3) != ESP_OK || x.reply != 0xAAAAAAAAu) {
        return false;
    }
    /*
     * Report both frames, not just the second.
     *
     * This discarded client_set's result, so a failure here has only ever
     * been able to say "CLIENT_ID did not come back" - when the device may
     * well have stopped answering one frame earlier, at the select.  Those
     * are different faults and they were indistinguishable from the log.
     */
    const esp_err_t set_err = dap_probe_client_set(1, &x);
    const int       set_wait = x.wait_cycles;
    const esp_err_t err = dap_probe_client_read(IO_CLIENT_ID, 4, 16, &x);

    ESP_LOGW(TAG, "    client_set %s (wait %d), client_read %s (wait %d)",
             esp_err_to_name(set_err), set_wait, esp_err_to_name(err),
             x.wait_cycles);

    *id_out = (uint16_t)x.reply;
    return (err == ESP_OK) && (x.reply == CLIENT_ID_EXPECT);
}

/*
 * Did the device answer at all, and did the fabric send what it was told to?
 *
 * A frame that draws no reply has two quite different causes with one symptom.
 * Either the payload never reached the fabric - in which case a well formed
 * frame went out saying the wrong thing - or it did and the device ignored it.
 * The register file now reads DATA back, so the first is answerable directly
 * rather than by inference.
 *
 * The second half separates "no start bit at all" from "a start bit and then
 * nothing", by asking for a 32-bit reply where only an acknowledge is due: a
 * timeout means the device never spoke, while an all-ones payload means it
 * spoke and the reply was shorter than the window.
 */
/*
 * The initialisation telegram, clocked without a start-bit hunt.
 *
 * This one is required - without it the device answers sync and ignores every
 * command after it, measured both ways round - but it draws no reply at all.
 * The LEN-48 form leaves the line undriven, reading idle high, so the fabric's
 * hunt latches the first high sample as a phantom start bit and then issues its
 * trailing clocks on top of it.  Those extra clocks put the device a bit out of
 * step and the next frame is lost; one more frame and it recovers, which is
 * exactly the "client_set times out once, then everything works" in the log.
 *
 * Raw-window mode is the fix rather than a diagnostic here: no hunt, no CRC, no
 * trailing clocks - just a fixed, known number of clocks for a command that has
 * nothing to say back.
 *
 * Constants repeated from dap_probe.c, which keeps them file-local.
 */
#define DAPISC_SIGNATURE  0x4ABBAF53u
#define DAPISC_VALUE      0x0F00u

static void send_dapisc_long(uint16_t value);

static void send_dapisc(void)
{
    send_dapisc_long(DAPISC_VALUE);
}

/*
 * What is actually on the wire during a reply window.
 *
 * Printed first clock leftmost, which is the order the target drives them in.
 * All zeros means something is holding the line down - the target stuffing
 * busy, or the probe never letting go.  All ones means nobody is driving it.
 * Anything else shows where the start bit landed.
 */
static void show_window(const char *what, uint8_t cmd, uint8_t len,
                        uint64_t data, size_t dbits)
{
    uint32_t reply = 0;
    uint16_t waited = 0;
    char     text[33];

    dap_phy_fpga_set_raw_window(true);
    const esp_err_t err = dap_phy_fpga_exchange(cmd, len, data, dbits, 32,
                                                &reply, &waited);
    dap_phy_fpga_set_raw_window(false);

    for (int i = 0; i < 32; i++) {
        text[i] = (reply & (1u << i)) ? '1' : '0';
    }
    text[32] = '\0';
    ESP_LOGW(TAG, "  %-22s window %s  (%s)", what, text, esp_err_to_name(err));
}

/*
 * Which field of the frame is the device objecting to?
 *
 * sync works and every other frame does not, and they differ in three ways at
 * once: the command, the LEN field, and whether a DATA field follows.  Varying
 * one at a time says which of the three matters - and a sync that still works
 * with data bolted onto it, or a LEN 3 sync that stops working, is a far
 * sharper statement than "payload frames fail".
 */
static void probe_payload_frame(void)
{
    uint32_t reply = 0;
    uint16_t waited = 0;

    ESP_LOGW(TAG, "--- one field at a time, from sync towards client_set ---");

    struct { const char *what; uint8_t cmd; uint8_t len; uint64_t data;
             uint8_t dbits; uint8_t rbits; } k[] = {
        { "sync (the control)",   0x10, 63, 0, 0, 32 },
        { "sync + 3 data bits",   0x10, 63, 1, 3, 32 },
        { "sync with LEN 3",      0x10,  3, 0, 0, 32 },
        { "client_set cmd, LEN63",0x1C, 63, 0, 0,  0 },
        { "client_set as sent",   0x1C,  3, 1, 3,  0 },
    };

    for (size_t i = 0; i < sizeof(k) / sizeof(k[0]); i++) {
        /*
         * A sync before each one, and nothing else.  Going through
         * dap_probe_clear_error_state() would put its own frames on the wire
         * between the test cases - and those are exactly the frames under
         * suspicion, so they would be part of what is being measured.
         */
        dap_phy_fpga_exchange(0x10u, 63, 0, 0, 32, &reply, &waited);

        const esp_err_t err = dap_phy_fpga_exchange(k[i].cmd, k[i].len, k[i].data,
                                                    k[i].dbits, k[i].rbits,
                                                    &reply, &waited);
        ESP_LOGW(TAG, "  %-22s %-18s reply 0x%08" PRIX32 " wait %u",
                 k[i].what, esp_err_to_name(err), reply, waited);
    }

    /*
     * The two framing parameters the fabric fixes and the CPU path does not:
     * how many idle clocks precede a frame, and how fast the clock runs.  Both
     * are swept against the frame that fails, because the bits are now known
     * to be identical to the ones the CPU path sends and gets answered.
     */
    ESP_LOGW(TAG, "--- lead-in clocks before the frame ---");
    static const uint8_t leads[] = { 2, 3, 4, 6, 8, 11, 16, 24, 32, 1, 0 };

    for (size_t i = 0; i < sizeof(leads); i++) {
        dap_phy_fpga_set_lead(leads[i]);
        dap_phy_fpga_exchange(0x10u, 63, 0, 0, 32, &reply, &waited);
        const esp_err_t err = dap_phy_fpga_exchange(0x1Cu, 3, 1, 3, 0,
                                                    &reply, &waited);
        ESP_LOGW(TAG, "  lead %2u: client_set %-18s wait %u%s", leads[i],
                 esp_err_to_name(err), waited, (err == ESP_OK) ? "  <- works" : "");
        if (err == ESP_OK) {
            break;
        }
    }
    dap_phy_fpga_set_lead(2);

    ESP_LOGW(TAG, "--- bit rate ---");
    static const uint8_t divs[] = { 2, 5, 11, 23, 59, 119 };

    for (size_t i = 0; i < sizeof(divs); i++) {
        dap_phy_fpga_set_div(divs[i]);
        dap_phy_fpga_exchange(0x10u, 63, 0, 0, 32, &reply, &waited);
        const esp_err_t sync_err = (reply == 0xAAAAAAAAu) ? ESP_OK : ESP_FAIL;
        const esp_err_t err = dap_phy_fpga_exchange(0x1Cu, 3, 1, 3, 0,
                                                    &reply, &waited);
        ESP_LOGW(TAG, "  div %3u (%u kHz): sync %s, client_set %-18s wait %u",
                 divs[i], (unsigned)(48000u / (2u * (divs[i] + 1u))),
                 (sync_err == ESP_OK) ? "ok" : "FAILED",
                 esp_err_to_name(err), waited);
    }
    dap_phy_fpga_set_div(5);

    ESP_LOGW(TAG, "--- the reply window, raw ---");
    dap_phy_fpga_exchange(0x10u, 63, 0, 0, 32, &reply, &waited);
    show_window("after sync",       0x10, 63, 0, 0);
    dap_phy_fpga_exchange(0x10u, 63, 0, 0, 32, &reply, &waited);
    show_window("after client_set", 0x1C,  3, 1, 3);
}

/* Sweep the one parameter the device turned out to be fussy about, if the
 * handshake failed and the cause is still open. */
static void trail_sweep(void)
{
    ESP_LOGW(TAG, "--- trailing clocks: which value does the device accept? ---");
    for (size_t i = 0; i < sizeof(k_trail_candidates); i++) {
        uint16_t id = 0;
        const bool ok = try_trail(k_trail_candidates[i], &id);

        ESP_LOGW(TAG, "  trail %u: CLIENT_ID 0x%04X %s",
                 k_trail_candidates[i], id, ok ? "<- works" : "");
        if (ok) {
            ESP_LOGW(TAG, "  using %u trailing clocks", k_trail_candidates[i]);
            dap_phy_fpga_set_trail(k_trail_candidates[i]);
            return;
        }
    }
    dap_phy_fpga_set_trail(1);
}

/*
 * The attach, as everything else uses it.  Declared in dap_phy_fpga.h; it
 * lives here because this is where it was worked out and where the
 * diagnostics that explain each step still are.
 */
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

    dap_exchange_t x;
    if (dap_probe_attach(&x, 3) != ESP_OK || x.reply != 0xAAAAAAAAu) {
        ESP_LOGE(TAG, "the target did not answer sync through the fabric");
        dap_phy_fpga_use(false);
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * The DAPISC goes out once, before the retry loop rather than inside it.
     * It configures the link; sending it again per attempt only repeats the
     * frame it costs, so every attempt failed at the same place for the same
     * reason - which read as "the handshake never works" rather than "it is
     * being restarted each time".
     */
    send_dapisc();

    dap_exchange_t id = {0};
    esp_err_t      id_err = ESP_FAIL;

    for (int attempt = 0; attempt < 4; attempt++) {
        /*
         * Clear the error state, then resync.
         *
         * The IOINFO read this sends is not housekeeping here - dropping it
         * left client_set working and client_read timing out every time, and
         * putting it back fixed that.  The reference FSM for this device has
         * the same read as its second step, before any address or data
         * telegram, for the same stated reason.
         *
         * Sync after it: whatever the telegram before left behind, sync clears
         * it, and it is the one command the device answers from any state.
         */
        dap_probe_clear_error_state();
        if (dap_probe_attach(&x, 3) != ESP_OK) {
            continue;
        }
        if (dap_probe_client_set(1, &x) != ESP_OK) {
            continue;
        }
        id_err = dap_probe_client_read(IO_CLIENT_ID, 4, 16, &id);
        if (id_err == ESP_OK && id.reply == CLIENT_ID_EXPECT) {
            ESP_LOGI(TAG, "attached through the fabric: CLIENT_ID 0x%04X on "
                          "attempt %d", (unsigned)id.reply, attempt + 1);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "attach attempt %d: CLIENT_ID 0x%04X (%s)", attempt + 1,
                 (unsigned)id.reply, esp_err_to_name(id_err));
    }

    ESP_LOGE(TAG, "CLIENT_ID came back 0x%04X, not 0x%04X",
             (unsigned)id.reply, CLIENT_ID_EXPECT);
    return ESP_FAIL;
}

/* ------------------------------------------------------------------------ */
/* Wide mode                                                                 */
/* ------------------------------------------------------------------------ */

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
#define DAPISC_MODE_SHIFT   4
#define DAPISC_MODE_NARROW  0u   /* 2-pin, DAP1 bidirectional */
#define DAPISC_MODE_WIDE    1u   /* 3-pin, DAP1 even bits and DAP2 odd */

#define OIFM_ADDR           0xF000040Cu

/*
 * Both DAPISC forms.  Long carries the 32-bit signature and is what the cold
 * attach uses; short is for reconfiguring a link that is already up.
 *
 * Both go out with the reply window raw: with a hunt enabled the undriven line
 * reads idle high, the fabric latches a phantom start bit and issues trailing
 * clocks on top of it, and the *next* frame is the one that gets lost.
 */
static void send_dapisc_long(uint16_t value)
{
    const uint64_t data = ((uint64_t)DAPISC_SIGNATURE << 16) | value;
    uint32_t reply = 0;
    uint16_t waited = 0;

    dap_phy_fpga_set_raw_window(true);
    dap_phy_fpga_exchange(0x11u, 48, data, 48, 2, &reply, &waited);
    dap_phy_fpga_set_raw_window(false);
}

static void send_dapisc_short(uint16_t value)
{
    uint32_t reply = 0;
    uint16_t waited = 0;

    dap_phy_fpga_set_raw_window(true);
    dap_phy_fpga_exchange(0x11u, 16, value, 16, 2, &reply, &waited);
    dap_phy_fpga_set_raw_window(false);
}

/*
 * The same write, keeping the reply.
 *
 * Both forms answer with the register as it now stands - start bit, the updated
 * sixteen bits, then a CRC6 - which makes the telegram self-verifying in the
 * one direction that matters.  Only useful when both ends agree on the framing;
 * the telegram that *changes* the framing cannot be read back this way, which
 * cost a debugging round and is written up at the call site.
 */
static bool dapisc_write_read(uint8_t len_field, uint64_t data, size_t dbits,
                              uint16_t *now)
{
    uint32_t reply = 0;
    uint16_t waited = 0;

    const esp_err_t err = dap_phy_fpga_exchange(0x11u, len_field, data, dbits,
                                                16, &reply, &waited);
    *now = (uint16_t)reply;
    return err == ESP_OK;
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
static void report_lines(const char *what)
{
    const uint8_t w = dap_phy_fpga_line_witness();

    ESP_LOGW(TAG, "  %-24s DAP2 while we drove %s%s | DAP2 while target drove "
                  "%s%s | DAP1 while target drove %s%s%s", what,
             (w & (1u << 1)) ? "0" : "-", (w & (1u << 2)) ? "1" : "-",
             (w & (1u << 3)) ? "0" : "-", (w & (1u << 4)) ? "1" : "-",
             (w & (1u << 5)) ? "0" : "-", (w & (1u << 6)) ? "1" : "-",
             (w & (1u << 0)) ? "  [start bit on DAP2]" : "");
}

static bool dap2_line_free(void)
{
    uint32_t reply = 0;
    uint16_t waited = 0;
    char     l1[17], l2[17];
    unsigned ones = 0;

    dap_phy_fpga_set_wide(true);
    dap_phy_fpga_set_raw_window(true);
    dap_phy_fpga_exchange(0x10u, 63, 0, 0, 32, &reply, &waited);
    dap_phy_fpga_set_raw_window(false);
    dap_phy_fpga_set_wide(false);

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
    report_lines("wide frame, narrow device:");
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
static void wide_revert(void)
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

}

/*
 * Which pair of capture taps reads the wire correctly?
 *
 * sync is the only command the device answers from any state and its reply is a
 * known constant, 0xAAAAAAAA - which in wide mode is exactly the useful
 * pattern, because alternating bits put all the zeros on one line and all the
 * ones on the other.  A tap pair with the two lines swapped reads 0x55555555;
 * one sampling a line outside its bit reads something with a dead CRC.  So the
 * sweep is a measurement and not a smoke test.
 *
 * The start-bit alignment bit is recorded alongside but not used to choose: it
 * is one sample of one bit and can agree by luck, where the payload and its CRC
 * are thirty-eight bits of evidence.  Where the two disagree is worth seeing,
 * which is why both are printed.
 */
static bool wide_calibrate(uint8_t *tap1_out, uint8_t *tap2_out)
{
    bool found = false;

    ESP_LOGW(TAG, "  capture tap sweep (sync must answer 0xAAAAAAAA):");

    for (uint8_t t1 = 0; t1 < 4; t1++) {
        char line[80];
        int  n = snprintf(line, sizeof(line), "    DAP1 tap %u:", t1);

        for (uint8_t t2 = 0; t2 < 4; t2++) {
            dap_exchange_t x = {0};

            if (dap_phy_fpga_set_skew(t1, t2) != ESP_OK) {
                continue;
            }

            /*
             * sync and nothing before it.
             *
             * An earlier version cleared the error state first, which sends a
             * real IOINFO read - and a payload command emitted in a framing
             * the device is not using is not a neutral act: it can leave the
             * device far enough out of step that the sync after it fails too.
             * Then every tap pair reads as "no answer" and the sweep says
             * nothing about the taps at all.  sync is answered from any state,
             * so it needs no preparation.
             */
            const bool ok = dap_probe_attach(&x, 3) == ESP_OK &&
                            x.reply == 0xAAAAAAAAu;
            const bool aligned = dap_phy_fpga_last_aligned();

            n += snprintf(line + n, sizeof(line) - (size_t)n, "  %c%c",
                          ok ? 'D' : (x.reply == 0x55555555u ? 'x' : '.'),
                          aligned ? 'a' : '-');

            if (ok && !found) {
                found     = true;
                *tap1_out = t1;
                *tap2_out = t2;
            }
        }
        ESP_LOGW(TAG, "%s", line);
    }

    ESP_LOGW(TAG, "    (D data good, x lines swapped, . no answer; "
                  "a start bit seen on DAP2)");
    return found;
}

/*
 * Wide mode end to end: get the part to route the line, tell the device,
 * switch the fabric, find the taps, prove a real transaction, measure it.
 */
static void wide_route_check(void)
{
    ESP_LOGW(TAG, "--- DAP wide mode ---");

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
     * What the lines do with the fabric wide and the device still narrow.
     *
     * Informational, not a gate.  The transmitter drives real data onto DAP2
     * during this frame, so the witness bits it leaves behind answer the one
     * question that has to be settled before any protocol theory is worth
     * testing: whether our own driver reaches the pad and comes back through
     * the input path at all.
     */
    (void)dap2_line_free();

    /*
     * Baseline: write the mode the device is already in and read it back.  If
     * that does not come back cleanly the readback is not trustworthy here and
     * nothing below it means much, so it is established before the interesting
     * write rather than after.
     */
    uint16_t now = 0;
    const uint16_t narrow = DAPISC_VALUE |
                            (DAPISC_MODE_NARROW << DAPISC_MODE_SHIFT);
    const bool base_ok = dapisc_write_read(16, narrow, 16, &now);
    ESP_LOGW(TAG, "  dapisc narrow baseline: wrote 0x%04X -> read 0x%04X%s, "
                  "MODE %u", (unsigned)narrow, (unsigned)now,
             base_ok ? "" : " NO VALID REPLY",
             (unsigned)((now >> DAPISC_MODE_SHIFT) & 3u));
    dap_probe_clear_error_state();

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

    const bool hs = dapisc_write_read(48, sig, 48, &now);
    ESP_LOGW(TAG, "  dapisc wide (long form, handshake): wrote 0x%04X -> "
                  "read 0x%04X%s, MODE %u", (unsigned)widev, (unsigned)now,
             hs ? "" : " NO VALID REPLY",
             (unsigned)((now >> DAPISC_MODE_SHIFT) & 3u));
    report_lines("after the handshake:");

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
    ESP_LOGW(TAG, "  narrow sync after the handshake: %s (0x%08" PRIX32 ")",
             still_narrow ? "still answers - the mux did NOT switch"
                          : "no longer answers", probe.reply);

    if (dap_phy_fpga_set_wide(true) != ESP_OK) {
        ESP_LOGE(TAG, "  the fabric would not switch to wide mode");
        wide_revert();
        return;
    }

    uint8_t tap1 = 0, tap2 = 0;
    if (!wide_calibrate(&tap1, &tap2)) {
        ESP_LOGE(TAG, "  no capture tap pair read sync correctly in wide mode");
        report_lines("after the sweep:");
        wide_revert();
        return;
    }

    dap_phy_fpga_set_skew(tap1, tap2);
    ESP_LOGW(TAG, "  taps DAP1 %u, DAP2 %u", tap1, tap2);

    /* Now the register can be read for real, on both lines. */
    const bool wide_ok = dapisc_write_read(16, widev, 16, &now);
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
    dap_exchange_t x, id = {0};
    if (dap_probe_attach(&x, 3) != ESP_OK ||
        dap_probe_client_set(1, &x) != ESP_OK ||
        dap_probe_client_read(IO_CLIENT_ID, 4, 16, &id) != ESP_OK ||
        id.reply != CLIENT_ID_EXPECT) {
        ESP_LOGE(TAG, "  wide mode answers sync but will not attach: "
                      "CLIENT_ID 0x%04X", (unsigned)id.reply);
        wide_revert();
        return;
    }

    dap_probe_set_rw_mode(true);
    uint32_t mcds_id = 0;
    if (dap_probe_enable_ocds() != ESP_OK ||
        dap_probe_read32(0xFB718008u, &mcds_id) != ESP_OK ||
        mcds_id != 0x00D6C007u) {
        ESP_LOGE(TAG, "  wide mode attaches but a bus read came back "
                      "0x%08" PRIX32, mcds_id);
        wide_revert();
        return;
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
    wide_revert();
    dap_phy_fpga_set_div(1);
    ESP_LOGW(TAG, "  back to narrow mode");
}

esp_err_t dap_probe_fpga_route_check(void)
{
    ESP_LOGW(TAG, "=== fabric DAP route ===");

    /* 1 and 2: the fabric answers, and the target attaches through it. */
    if (dap_phy_fpga_attach() != ESP_OK) {
        probe_payload_frame();
        trail_sweep();
        dap_phy_fpga_log_status();
        dap_phy_fpga_use(false);
        return ESP_FAIL;
    }

    /* 3: a real bus read, self-checked against a constant. */
    dap_probe_clear_error_state();
    dap_probe_set_rw_mode(true);

    uint32_t mcds_id = 0;
    if (dap_probe_enable_ocds() == ESP_OK &&
        dap_probe_read32(0xFB718008u, &mcds_id) == ESP_OK) {
        ESP_LOGW(TAG, "  miniMCDS ID = 0x%08" PRIX32 "%s", mcds_id,
                 mcds_id == 0x00D6C007u ? "  (expected)" : "  UNEXPECTED");
    } else {
        ESP_LOGW(TAG, "  miniMCDS ID unreadable");
    }

    /*
     * 3b: does this part allow wide mode at all?
     *
     * OIFM.DAPMODE selects the DAP interface's line count, and wide mode is
     * only reachable if it is set to one of the values that route DAP2 out.
     * Worth knowing before any of it is built: the whole feature is untestable
     * on a part whose interface is strapped to two pins.
     */
    {
        uint32_t oifm = 0;
        if (dap_probe_read32(0xF000040Cu, &oifm) == ESP_OK) {
            static const char *k_mode[8] = {
                "2-pin, wide allowed", "reserved", "reserved",
                "wide allowed", "wide allowed", "reserved",
                "reserved", "reserved",
            };
            const unsigned mode = oifm & 7u;
            ESP_LOGW(TAG, "  OIFM = 0x%08" PRIX32 ", DAPMODE %u (%s), PADCTL %u",
                     oifm, mode, k_mode[mode], (unsigned)((oifm >> 12) & 3u));
        } else {
            ESP_LOGW(TAG, "  OIFM unreadable");
        }
    }

    /* 4: a block read, and what it costs. */
    {
        static uint32_t buf[256];
        const int iterations = 8;
        int ok = 0;

        if (dap_probe_blockread(CHECK_ADDR, buf, 8) == ESP_OK) {
            ESP_LOGW(TAG, "  block of 8: %08" PRIX32 " %08" PRIX32 " %08" PRIX32,
                     buf[0], buf[1], buf[2]);
        } else {
            ESP_LOGE(TAG, "  a short block read failed");
            dap_phy_fpga_log_status();
            dap_phy_fpga_use(false);
            return ESP_FAIL;
        }

        /*
         * Swept rather than measured at one divider.  The fabric clock is
         * 48 MHz, so the DAP clock is 48/(2*(div+1)) and the fastest setting
         * the clock generator allows is div 1, for 12 MHz.  Which divider the
         * silicon still answers at is a separate question from what the
         * fabric can emit, and the answer is the rate, so both go in the log.
         */
        static const uint8_t divs[] = { 11, 5, 3, 2, 1, 0 };

        ESP_LOGW(TAG, "--- block read throughput ---");
        for (size_t d = 0; d < sizeof(divs); d++) {
            dap_phy_fpga_set_div(divs[d]);
            ok = 0;

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
                ESP_LOGW(TAG, "  div %2u (%4u kHz): %d/%d blocks of 1 kB in "
                              "%6lld us -> %3d kB/s", divs[d],
                         (unsigned)(48000u / (2u * (divs[d] + 1u))),
                         ok, iterations, (long long)us, kbps);
            } else {
                ESP_LOGW(TAG, "  div %2u: no full blocks completed", divs[d]);
            }
        }
        ESP_LOGW(TAG, "  (the CPU path does 453 kB/s at 12 MHz)");
        dap_phy_fpga_set_div(1);
    }

    /* 5: the same thing again, on two data lines. */
    wide_route_check();

    /*
     * Left switched on.  Unlike the CPU path, this one only exists while the
     * DAP bitstream is loaded, and a reboot puts the stock image back - so the
     * state is not sticky in a way that could confuse a later session.
     */
    ESP_LOGW(TAG, "=== fabric route verified ===");
    return ESP_OK;
}
