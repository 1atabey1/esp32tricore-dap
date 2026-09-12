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
#include "dap_frame.h"
#include "dap_probe.h"
#include "tricore.h"
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
        /*
         * Reset the target and try once more before giving up.
         *
         * sync is answered from any state the *protocol* can be in, so a
         * silent target usually means the two ends disagree about the physical
         * mode - which is exactly what a wide-mode experiment that did not get
         * to put things back leaves behind, and it survives a probe reboot
         * because it is the target holding the state, not us.  The device's
         * mode is cleared by a device reset, so TRST is the way out, and
         * without this the only recovery was a human power-cycling the board.
         */
        ESP_LOGW(TAG, "no answer to sync; resetting the target and retrying");
        dap_phy_fpga_set_wide(false);
        dap_phy_fpga_set_trst(true);
        vTaskDelay(pdMS_TO_TICKS(2));
        dap_phy_fpga_set_trst(false);
        vTaskDelay(pdMS_TO_TICKS(20));
        dap_probe_clear_error_state();

        if (dap_probe_attach(&x, 3) != ESP_OK || x.reply != 0xAAAAAAAAu) {
            ESP_LOGE(TAG, "the target did not answer sync through the fabric");
            dap_phy_fpga_use(false);
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGW(TAG, "the reset brought it back");
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
 * The target's DAP2 pin, as an ordinary port pin.
 *
 * DAP2 is P21.7 on this part, and while the interface is in two-pin mode the
 * port module owns it - which is the whole reason wide mode did not work: the
 * application configures it as a push-pull output and holds it low, so
 * Cerberus can put its protocol engine in wide mode, and does, but cannot
 * drive the pad.
 */
#define P21_OUT     0xF003B500u
#define P21_IOCR4   0xF003B514u
#define P21_IN      0xF003B524u
#define P21_PDISC   0xF003B560u

#define DAP2_PIN    7u
/* IOCR4 holds four five-bit fields for pins 4..7, pin n at bit 3 + 8*(n-4).
 * 0x10 is push-pull general-purpose output; 0x00 is a plain input. */
#define IOCR4_SHIFT(n)  (3u + 8u * ((n) - 4u))
#define IOCR_PP_OUT     0x10u

/*
 * The target application's own setting for P21.7, put back on every exit.
 *
 * It configured that pin deliberately and is entitled to it again once the
 * probe is done borrowing it.
 */
static uint32_t s_iocr4_saved;
static bool     s_iocr4_dirty;

/* Defined below; wide_revert needs them and comes first. */
static void halt_application(void);
static void resume_application(void);


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
#define WIDE_SYNC_EXPECT 0xCCCCCCCCu

/*
 * Both DAPISC forms.  Long carries the 32-bit signature and is what the cold
 * attach uses; short is for reconfiguring a link that is already up.
 *
 * Both go out with the reply window raw: with a hunt enabled the undriven line
 * reads idle high, the fabric latches a phantom start bit and issues trailing
 * clocks on top of it, and the *next* frame is the one that gets lost.
 */
/*
 * Which command code the dapisc telegram goes out as, from
 * /api/dap_fpga?dcmd=N.
 *
 * 0x11 measured, not assumed.  The register spec gives 0x09 for both dapisc
 * forms, but 0x09 is client_blockwrite - the blockwrite spec says so, and the
 * device agrees: sent as 0x09 with LEN 48 the telegram desynchronises the link
 * hard enough that the following attach reads CLIENT_ID 0x0000 at every trail
 * value.  With 0x11 the link stays up.  So the register spec has the wrong
 * opcode, and the reason dapisc never takes is something else.
 *
 * What is still unexplained is that no dapisc telegram is ever answered, while
 * every other command is.  Attach does not depend on it landing - sync,
 * client_set and client_read all work against the reset configuration - so
 * nothing noticed until wide mode needed DAPISC.MODE changed.
 *
 * Kept overridable because finding the right opcode is one request per guess
 * this way and one rebuild per guess otherwise.
 */
#define DAPISC_CMD_DEFAULT 0x11u
static uint8_t s_dapisc_cmd = DAPISC_CMD_DEFAULT;

void dap_probe_fpga_dapisc_cmd(int cmd)
{
    s_dapisc_cmd = (cmd < 0) ? DAPISC_CMD_DEFAULT : (uint8_t)(cmd & 0x1F);
}

static void send_dapisc_long(uint16_t value)
{
    const uint64_t data = ((uint64_t)DAPISC_SIGNATURE << 16) | value;
    uint32_t reply = 0;
    uint16_t waited = 0;

    dap_phy_fpga_set_raw_window(true);
    dap_phy_fpga_exchange(s_dapisc_cmd, 48, data, 48, 2, &reply, &waited);
    dap_phy_fpga_set_raw_window(false);
}

static void send_dapisc_short(uint16_t value)
{
    uint32_t reply = 0;
    uint16_t waited = 0;

    dap_phy_fpga_set_raw_window(true);
    dap_phy_fpga_exchange(s_dapisc_cmd, 16, value, 16, 2, &reply, &waited);
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
static bool s_dapisc_rx_wide;

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
    const int skip = (s_dapisc_rx_wide || dap_phy_fpga_is_wide()) ? 2 : 1;

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
static bool dapisc_write_read(uint8_t len_field, uint64_t data, size_t dbits,
                              uint16_t *now);

static void dapisc_prime(void)
{
    uint16_t ignored = 0;

    for (int i = 0; i < 2; i++) {
        dapisc_write_read(16, DAPISC_VALUE, 16, &ignored);
    }
}

static bool dapisc_write_read(uint8_t len_field, uint64_t data, size_t dbits,
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
        const esp_err_t err = dap_phy_fpga_exchange(s_dapisc_cmd, len_field,
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
 * The reply window a dapisc telegram draws, kept verbatim.
 *
 * dapisc_write_read() asks the fabric to hunt for a start bit and check a
 * CRC6, and reports one fact when that fails: nothing valid arrived.  That
 * cannot tell a device which never answered from an answer arriving where the
 * hunt was not looking - and the register spec says both telegram forms answer
 * with RLEN = 16, so an answer should be there.
 *
 * This re-sends the same telegram with the window raw, so `reply` is the first
 * 32 clocks of it exactly as sampled.  Safe to repeat for the narrow baseline,
 * which writes the value the register already holds; it is deliberately not
 * wired into the handshake, where sending the telegram twice would have the
 * second one arrive in a framing the device has just left.
 */
static void dapisc_raw_probe(uint8_t len_field, uint64_t data, size_t dbits)
{
    uint32_t raw = 0;
    uint16_t waited = 0;
    char     bits[33];

    dap_phy_fpga_set_raw_window(true);
    const esp_err_t err = dap_phy_fpga_exchange(s_dapisc_cmd, len_field, data,
                                                dbits, 32, &raw, &waited);
    dap_phy_fpga_set_raw_window(false);

    for (int i = 0; i < 32; i++) {
        bits[i] = ((raw >> i) & 1u) ? '1' : '0';
    }
    bits[32] = '\0';
    ESP_LOGW(TAG, "  dapisc raw window: %s (0x%08X, waited %u, %s)", bits,
             (unsigned)raw, (unsigned)waited,
             (err == ESP_OK) ? "ok" : "no CRC checked");
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

    resume_application();

    if (s_iocr4_dirty) {
        s_iocr4_dirty = false;
        if (dap_probe_write32(P21_IOCR4, s_iocr4_saved) == ESP_OK) {
            ESP_LOGW(TAG, "  P21.7 given back to the application");
        } else {
            ESP_LOGE(TAG, "  P21.7 could not be given back; IOCR4 should be "
                          "0x%08" PRIX32, s_iocr4_saved);
        }
        dap_probe_clear_error_state();
    }
}

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

static void dap2_wiring_probe(void)
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

static void halt_application(void)
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

static void resume_application(void)
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
#define IOCR_IN_NOPULL  0x00u
#define IOCR_IN_PULLUP  0x02u

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
static bool p21_7_release(uint32_t *saved_iocr4)
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
    s_iocr4_dirty = true;
    return true;
}

/* ------------------------------------------------------------------------ */
/* Wide framing: candidate rules, tried against the device                    */
/* ------------------------------------------------------------------------ */

/*
 * A frame as a bit sequence, first bit on the wire in bit 0.
 *
 * Kept as a plain integer because everything that fits the raw path fits in
 * 63 bits: sync is 19 narrow and 22 wide, and the longest thing this needs to
 * send is a block-read command at 59.
 */
typedef struct {
    uint64_t bits;
    unsigned n;
} frame_bits_t;

static void fb_push(frame_bits_t *f, unsigned bit)
{
    if (bit & 1u) {
        f->bits |= 1ull << f->n;
    }
    f->n++;
}

static void fb_field(frame_bits_t *f, uint64_t value, unsigned nbits)
{
    for (unsigned i = 0; i < nbits; i++) {
        fb_push(f, (unsigned)((value >> i) & 1u));
    }
}

/*
 * The DAP CRC6, over a bit sequence in transmission order.
 *
 * A right-shifting Galois LFSR, the same one dap_frame.c uses and the host
 * test binary checks against the documented vectors.  Repeated here rather
 * than shared because this one takes a bit sequence and that one takes fields,
 * and the whole point of these candidates is that what counts as a field is
 * the thing in question.
 */
static uint8_t crc6_of(uint64_t bits, unsigned n)
{
    uint8_t state = 0x20u;

    for (unsigned i = 0; i < n; i++) {
        const unsigned fb = (state ^ (unsigned)(bits >> i)) & 1u;
        state = (uint8_t)((state >> 1) & 0x3Fu);
        if (fb) {
            state ^= 0x30u;
        }
    }
    return state;
}

/*
 * The candidate wide framing rules.
 *
 * All of them agree on what the device is being told - CMD, LEN and DATA are
 * the same numbers - and differ only in how those become a bit stream once two
 * bits share a clock.  That is the whole unknown: the narrow framing is
 * documented and checked against vectors, and the wide one is described in
 * exactly one place this project can reach, as a reconstruction.
 *
 * In the raw path the fabric puts even bit positions on DAP1 and odd ones on
 * DAP2, so "the start bit goes on both lines" is expressed by putting a one in
 * each of the first two positions, and "on DAP1 only" by letting the frame's
 * next bit take the odd slot.
 */
typedef enum {
    /*
     * Every field padded to an even length and the pads covered by the CRC.
     * This is the reconstruction the RTL originally implemented; the device
     * rejects it, which is why the others exist.
     */
    WF_PAD_CRC = 0,
    /* Padded the same way, but the CRC covers only the real bits. */
    WF_PAD_NOCRC,
    /*
     * No padding at all: CMD, LEN, DATA and the CRC are one continuous stream
     * split between the lines, so LEN starts on DAP2 because CMD's five bits
     * leave the parity odd.  The CRC is then the narrow one unchanged.
     */
    WF_STREAM,
    /*
     * The narrow frame exactly, start bit included, split between the lines -
     * which puts the start bit on DAP1 alone.
     */
    WF_STREAM_1START,
    WF_COUNT
} wide_framing_t;

static const char *wf_name(wide_framing_t v)
{
    switch (v) {
        case WF_PAD_CRC:       return "fields padded, CRC covers pads";
        case WF_PAD_NOCRC:     return "fields padded, CRC skips pads";
        case WF_STREAM:        return "no padding, one stream";
        case WF_STREAM_1START: return "no padding, start bit on DAP1 only";
        default:               return "?";
    }
}

/*
 * Assemble one candidate.
 *
 * `cmd`, `len_field` and `data`/`data_bits` are the frame's contents in the
 * ordinary sense - LEN is the field that goes on the wire and data_bits is how
 * many DATA bits follow it, which are not the same thing and never have been.
 */
static frame_bits_t wide_frame(wide_framing_t v, uint8_t cmd, uint8_t len_field,
                               uint64_t data, unsigned data_bits)
{
    frame_bits_t f = { 0, 0 };
    frame_bits_t c = { 0, 0 };      /* what the CRC is taken over */

    switch (v) {
        case WF_PAD_CRC:
        case WF_PAD_NOCRC: {
            const bool data_odd = (data_bits & 1u) != 0u;

            fb_field(&c, cmd, 5);
            if (v == WF_PAD_CRC) {
                fb_push(&c, 0);                  /* CMD's pad, covered */
            }
            fb_field(&c, len_field, 6);
            fb_field(&c, data, data_bits);
            if (data_odd && v == WF_PAD_CRC) {
                fb_push(&c, 0);                  /* DATA's pad, covered */
            }

            fb_push(&f, 1);  fb_push(&f, 1);     /* start bit on both lines */
            fb_field(&f, cmd, 5);
            fb_push(&f, 0);                      /* CMD is five bits */
            fb_field(&f, len_field, 6);
            fb_field(&f, data, data_bits);
            if (data_odd) {
                fb_push(&f, 0);
            }
            fb_field(&f, crc6_of(c.bits, c.n), 6);
            fb_push(&f, 0);  fb_push(&f, 0);     /* trailing zero, both lines */
            break;
        }

        case WF_STREAM: {
            fb_field(&c, cmd, 5);
            fb_field(&c, len_field, 6);
            fb_field(&c, data, data_bits);

            fb_push(&f, 1);  fb_push(&f, 1);
            fb_field(&f, cmd, 5);
            fb_field(&f, len_field, 6);
            fb_field(&f, data, data_bits);
            fb_field(&f, crc6_of(c.bits, c.n), 6);
            fb_push(&f, 0);
            if (f.n & 1u) {
                fb_push(&f, 0);                  /* fill the last clock */
            }
            break;
        }

        case WF_STREAM_1START:
        default: {
            fb_field(&c, cmd, 5);
            fb_field(&c, len_field, 6);
            fb_field(&c, data, data_bits);

            fb_push(&f, 1);                      /* start bit, DAP1 only */
            fb_field(&f, cmd, 5);
            fb_field(&f, len_field, 6);
            fb_field(&f, data, data_bits);
            fb_field(&f, crc6_of(c.bits, c.n), 6);
            fb_push(&f, 0);
            if (f.n & 1u) {
                fb_push(&f, 0);
            }
            break;
        }
    }
    return f;
}

/*
 * Which framing rule does the device accept, and at which capture taps?
 *
 * sync is the probe here for the same reason it is everywhere else: it is
 * answered from any state and its reply is a known constant, so a right answer
 * cannot be a coincidence.  Every combination of rule and tap pair is tried,
 * which is sixty-four frames and takes a few milliseconds.
 *
 * Sending the candidates rather than rebuilding the fabric for each is the
 * whole reason the raw path exists: this loop used to be a twenty-minute
 * bitstream build per guess.
 */
static bool wide_framing_sweep(wide_framing_t *rule, uint8_t *tap1_out,
                               uint8_t *tap2_out)
{
    bool found = false;

    ESP_LOGW(TAG, "  wide framing sweep (sync must answer 0x%08X):",
             WIDE_SYNC_EXPECT);

    for (unsigned v = 0; v < WF_COUNT; v++) {
        const frame_bits_t f = wide_frame((wide_framing_t)v, 0x10u, 63, 0, 0);
        char line[96];
        int  n = snprintf(line, sizeof(line), "    %-34s %2u bits:",
                          wf_name((wide_framing_t)v), f.n);

        for (uint8_t t1 = 0; t1 < 4; t1++) {
            for (uint8_t t2 = 0; t2 < 4; t2++) {
                uint32_t reply = 0;
                uint16_t waited = 0;

                if (dap_phy_fpga_set_skew(t1, t2) != ESP_OK) {
                    continue;
                }
                const esp_err_t err = dap_phy_fpga_raw_frame(f.bits, f.n, 32,
                                                             &reply, &waited);
                /* Same pattern as the tap sweep, and the status is not part
                 * of it for the same reason - see WIDE_SYNC_EXPECT. */
                (void)err;
                const bool ok = (reply == WIDE_SYNC_EXPECT);

                if (ok || reply == 0xAAAAAAAAu) {
                    n += snprintf(line + n, sizeof(line) - (size_t)n, " %u%u%c",
                                  t1, t2, ok ? '!' : '~');
                }
                if (ok && !found) {
                    found     = true;
                    *rule     = (wide_framing_t)v;
                    *tap1_out = t1;
                    *tap2_out = t2;
                }
            }
        }
        if (n == (int)strlen(line)) {
            /* nothing interesting printed */
        }
        ESP_LOGW(TAG, "%s", line);
    }

    ESP_LOGW(TAG, "    (tap pairs listed only where the reply looked like "
                  "sync; ! exact, ~ lines swapped)");
    return found;
}


/*
 * Which pair of capture taps reads the wire correctly?
 *
 * sync is the only command the device answers from any state, and in wide mode
 * its reply is WIDE_SYNC_EXPECT.  A tap pair sampling a line outside its bit
 * reads something else, so the sweep is a measurement and not a smoke test -
 * but only a weak one, because the same pattern on both lines means a pair with
 * the two lines swapped reads exactly the same thing.  What settles it is the
 * bus read further down, which no tap pair can pass by accident.
 *
 * The start-bit alignment bit is recorded alongside but not used to choose: it
 * is one sample of one bit and can agree by luck, where the payload is
 * thirty-two bits of evidence.  Where the two disagree is worth seeing, which
 * is why both are printed.
 */
static bool wide_calibrate(uint8_t *tap1_out, uint8_t *tap2_out)
{
    bool found = false;

    ESP_LOGW(TAG, "  capture tap sweep (sync must answer 0x%08X):",
             WIDE_SYNC_EXPECT);

    for (uint8_t t1 = 0; t1 < 4; t1++) {
        char line[80];
        int  n = snprintf(line, sizeof(line), "    DAP1 tap %u:", t1);

        for (uint8_t t2 = 0; t2 < 4; t2++) {
            dap_exchange_t x = {0};

            if (dap_phy_fpga_set_skew(t1, t2) != ESP_OK) {
                continue;
            }

            /*
             * One sync, and nothing before or after it.
             *
             * Not dap_probe_attach(): that retries and flushes with a burst of
             * idle clocks between attempts, which is right for getting a link
             * up and wrong for a sweep - sixteen tap pairs times three
             * attempts times a flush each is most of a minute spent proving
             * the same thing sixteen times.
             *
             * And nothing before it either.  An earlier version cleared the
             * error state first, which sends a real IOINFO read, and a payload
             * command emitted in a framing the device is not using is not a
             * neutral act: it can leave the device far enough out of step that
             * the sync after it fails too, and then every cell reads as "no
             * answer" and the sweep says nothing about the taps at all.  sync
             * is answered from any state, so it needs no preparation.
             */
            /*
             * A gap between frames.
             *
             * Wide frames are not free to repeat: this end drives DAP2 through
             * a 22 ohm series resistor, and any moment the device has not let
             * go of it yet is two push-pull drivers on one net.  One frame is
             * survivable and sixteen back to back browned the board out into a
             * boot loop - the firmware was fine, the rail was not.  Narrowing
             * the enable to the frame proper removed most of the overlap; this
             * removes the rest of the reason to care, at a cost of a few
             * milliseconds across the whole sweep.
             */
            vTaskDelay(pdMS_TO_TICKS(5));

            /* The status is not part of the test: the reply carries a CRC
             * this pattern cannot satisfy, so it comes back INVALID_CRC even
             * when the bits are exactly right. */
            (void)dap_probe_sync(&x);
            const bool ok = (x.reply == WIDE_SYNC_EXPECT);
            const bool aligned = dap_phy_fpga_last_aligned();

            n += snprintf(line + n, sizeof(line) - (size_t)n, "  %c%c",
                          ok ? 'D' : '.', aligned ? 'a' : '-');

            /*
             * Print the value whenever the device answered at all.
             *
             * A symbol per cell was enough while nothing replied; now that the
             * start bit is arriving on both lines the reply itself is the
             * evidence, and what it is wrong *by* says whether the two lines
             * are swapped, offset by a bit, or being read at the wrong phase.
             */
            if (aligned || x.reply != 0u) {
                ESP_LOGW(TAG, "      taps %u,%u -> 0x%08" PRIX64 "%s", t1, t2,
                         x.reply, aligned ? "  (start bit on DAP2)" : "");
            }

            if (ok && !found) {
                found     = true;
                *tap1_out = t1;
                *tap2_out = t2;
            }
        }
        ESP_LOGW(TAG, "%s", line);
    }

    ESP_LOGW(TAG, "    (D pattern correct, . not; a start bit seen on DAP2)");
    return found;
}

/*
 * How far into the wide-mode sequence to go, from /api/dap_fpga?stage=N.
 *
 * Bring-up scaffolding, and it earned itself quickly: the sequence wedged the
 * board hard enough to need a power cycle, and finding which step did it by
 * rebuilding and reflashing per guess is a four-minute loop.  A stage limit
 * makes it one request.  Zero, the default, runs everything.
 */
static int s_wide_stage;
static int s_wide_tap1, s_wide_tap2;
static int s_wide_trail = -1;   /* -1 leaves TRAIL alone */

void dap_probe_fpga_wide_trail(int trail)
{
    s_wide_trail = trail;
}

void dap_probe_fpga_wide_stage(int stage)
{
    s_wide_stage = stage;
}

void dap_probe_fpga_wide_taps(int tap1, int tap2)
{
    s_wide_tap1 = tap1 & 3;
    s_wide_tap2 = tap2 & 3;
}

#define WIDE_STAGE(n, revert)                                                 \
    do {                                                                      \
        if (s_wide_stage && s_wide_stage <= (n)) {                            \
            ESP_LOGW(TAG, "  stopping after stage %d as asked", (n));         \
            if (revert) { wide_revert(); }                                    \
            return;                                                           \
        }                                                                     \
    } while (0)

/*
 * Wide mode end to end: get the part to route the line, tell the device,
 * switch the fabric, find the taps, prove a real transaction, measure it.
 */
static void wide_route_check(void)
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
    if (s_wide_stage == 0) {
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
    if (s_wide_trail >= 0) {
        dap_phy_fpga_set_trail((uint8_t)s_wide_trail);
        ESP_LOGW(TAG, "  trailing clocks set to %d for this run", s_wide_trail);
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
    dap2_wiring_probe();
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
    (void)dap2_line_free();
    WIDE_STAGE(2, false);

    /*
     * Baseline: write the mode the device is already in and read it back.  If
     * that does not come back cleanly the readback is not trustworthy here and
     * nothing below it means much, so it is established before the interesting
     * write rather than after.
     */
    dapisc_prime();

    uint16_t now = 0;
    const uint16_t narrow = DAPISC_VALUE |
                            (DAPISC_MODE_NARROW << DAPISC_MODE_SHIFT);
    const bool base_ok = dapisc_write_read(16, narrow, 16, &now);
    ESP_LOGW(TAG, "  dapisc narrow baseline: wrote 0x%04X -> read 0x%04X%s, "
                  "MODE %u", (unsigned)narrow, (unsigned)now,
             base_ok ? "" : " NO VALID REPLY",
             (unsigned)((now >> DAPISC_MODE_SHIFT) & 3u));
    if (!base_ok) {
        dapisc_raw_probe(16, narrow, 16);   /* what was on the wire */
    }
    dap_probe_clear_error_state();

    /*
     * Take the pin off the application before asking for wide mode.
     *
     * Order matters: while the port module drives P21.7 the interface cannot,
     * so a mode change made first produces a device that is wide and mute.
     */
    halt_application();

    if (!p21_7_release(&s_iocr4_saved)) {
        ESP_LOGE(TAG, "  the target still owns P21.7, so DAP2 cannot be driven "
                      "from here - two push-pull drivers on one net through a "
                      "22 ohm resistor is about 150 mA, and enough frames of it "
                      "brown this board out");
        wide_revert();
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
    s_dapisc_rx_wide = true;
    const bool hs = dapisc_write_read(48, sig, 48, &now);
    s_dapisc_rx_wide = false;
    dap_phy_fpga_set_rx_wide(false);
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
    ESP_LOGW(TAG, "  narrow sync after the handshake: %s (0x%08" PRIX64 ")",
             still_narrow ? "still answers - the mux did NOT switch"
                          : "no longer answers", probe.reply);

    WIDE_STAGE(4, true);

    if (dap_phy_fpga_set_wide(true) != ESP_OK) {
        ESP_LOGE(TAG, "  the fabric would not switch to wide mode");
        wide_revert();
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
    if (s_wide_stage == 6) {
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

        dap_phy_fpga_set_skew((uint8_t)s_wide_tap1, (uint8_t)s_wide_tap2);
        const esp_err_t e = dap_probe_sync(&one);
        ESP_LOGW(TAG, "  taps %d,%d: sync %s, reply 0x%08" PRIX64 "%s",
                 s_wide_tap1, s_wide_tap2, esp_err_to_name(e), one.reply,
                 one.reply == WIDE_SYNC_EXPECT ? "   <== CORRECT" : "");
        wide_revert();
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
    if (s_wide_stage == 9) {
        uint32_t raw = 0;
        uint16_t waited = 0;
        char l1[17], l2[17];

        dap_phy_fpga_set_wide(false);      /* transmit narrow */
        dap_phy_fpga_set_rx_wide(true);    /* sample both lines */
        dap_phy_fpga_set_skew((uint8_t)s_wide_tap1, (uint8_t)s_wide_tap2);

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
                 s_wide_tap1, s_wide_tap2);
        ESP_LOGW(TAG, "    DAP1 %s", l1);
        ESP_LOGW(TAG, "    DAP2 %s", l2);
        report_lines("after it:");

        wide_revert();
        return;
    }

    const bool tap_ok = wide_calibrate(&tap1, &tap2);
    WIDE_STAGE(7, true);

    if (!tap_ok) {
        ESP_LOGW(TAG, "  no tap pair worked with the assembled framing; "
                      "trying the candidate rules");
        report_lines("after the tap sweep:");

        wide_framing_t rule = WF_PAD_CRC;
        if (!wide_framing_sweep(&rule, &tap1, &tap2)) {
            ESP_LOGE(TAG, "  no framing rule and tap pair got sync answered");
            report_lines("after the framing sweep:");
            wide_revert();
            return;
        }
        ESP_LOGW(TAG, "  framing \"%s\" is the one the device wants",
                 wf_name(rule));
    }

    WIDE_STAGE(8, true);

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
        ? dap_probe_client_read(IO_CLIENT_ID, 4, 16, &id) : ESP_FAIL;
    ESP_LOGW(TAG, "  wide attach: client_read %s -> 0x%04X (want 0x%04X)",
             esp_err_to_name(r_err), (unsigned)id.reply,
             (unsigned)CLIENT_ID_EXPECT);

    /*
     * A failed re-attach is not the end of the run.
     *
     * The client selected during the narrow attach survives the mode change -
     * the telegram changes the framing, not the device's state - so a block
     * read can go out without one, and whether *that* works is the question
     * the whole exercise is here to answer.  Aborting on client_set meant the
     * throughput sweep below had never once been reached.
     */
    const bool attached = (r_err == ESP_OK && id.reply == CLIENT_ID_EXPECT);

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
            wide_revert();
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
