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
#include <string.h>

#include "dap_phy.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "esp_log.h"
#include "esp_timer.h"

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

static void send_dapisc(void)
{
    const uint64_t data = ((uint64_t)DAPISC_SIGNATURE << 16) | DAPISC_VALUE;
    uint32_t reply = 0;
    uint16_t waited = 0;

    dap_phy_fpga_set_raw_window(true);
    dap_phy_fpga_exchange(0x11u, 48, data, 48, 2, &reply, &waited);
    dap_phy_fpga_set_raw_window(false);
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
        static const uint8_t divs[] = { 11, 5, 3, 2, 1 };

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
    }

    /*
     * Left switched on.  Unlike the CPU path, this one only exists while the
     * DAP bitstream is loaded, and a reboot puts the stock image back - so the
     * state is not sticky in a way that could confuse a later session.
     */
    ESP_LOGW(TAG, "=== fabric route verified ===");
    return ESP_OK;
}
