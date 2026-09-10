#include "dap_probe.h"

#include <inttypes.h>
#include <string.h>

#include "board_profile.h"
#include "dap_frame.h"
#include "dap_phy.h"
#include "esp_log.h"

static const char *TAG = "DAP";

/*
 * Bits clocked after a reply's CRC, with the target still driving.
 *
 * Replies alternate - every second exchange answers - which is what leaving a
 * fixed amount of state behind per exchange looks like.  Making this adjustable
 * turns that into a measurement instead of a guess.
 */
static size_t s_trailer_bits;   /* zero: measured to be what the device wants */

/*
 * IOClient registers, by IO instruction nibble.
 *
 * CLIENT_ID is instruction 0xF at offset 0x0F, 16 bits wide, hard-wired to
 * 0x0260: TYPE 0x02 (Cerberus_FPI 32-bit client), VERSION 6, REVISION 0.  A
 * 16-bit read uses payload 0x4F; a 32-bit read uses 0x5F and the hardware
 * replicates the halfword, giving 0x02600260.
 *
 * This was briefly changed to 0xB on the strength of a USB capture, in which
 * every client_read the reference probe issued carried payload 0x4B.  That was
 * a misreading: 0xB is IOINFO at offset 0x0B, so the reference was reading
 * client info and error status, not the ID.  The target then returned exactly
 * what was asked for - 0x0020, IOINFO's PWR_DWN bit - which is why the reply
 * was CRC-valid but not 0x0260.  A capture shows what a tool happened to do,
 * not what a register requires.
 */
#define DAP_IO_CLIENT_ID       0xFu
#define DAP_IO_INFO            0xBu
#define DAP_CLIENT_ID_EXPECT   0x0260u

/*
 * The training handshake the reference performs and this project did not.
 *
 * After sync and dapisc the device sends an alternating pattern, and the
 * reference answers with CMD 0x02 carrying 0xAAAAAA83 - itself alternating -
 * twice, and only then gets real data back.  Neither the command nor the
 * constant is in the catalog this project started from; both come straight out
 * of a USB capture of a working attach.  Without it the device answers sync
 * once and ignores everything afterwards.
 */
#define DAP_CMD_CLIENT_RESET   0x1Du

/*
 * Low clocks before every frame.
 *
 * The protocol needs no multi-cycle preamble: the device latches any DAP1 = 1
 * on a rising edge as a start bit.  What it does need is the line settled low
 * first - the device drives DAP1 = 0 for one cycle after its reply before
 * releasing the pad, and DAPISC.SISP can hold off start-bit detection for a
 * further 0 to 3 cycles.  So the floor is 1 + SISP, and 2 covers the default
 * SISP of 0 with margin.
 *
 * Eleven was what the reference probe uses and what this code copied.  Per the
 * specification that figure is a conservative host-side buffer covering FPGA
 * clock-domain-crossing latency, not a device requirement - so most of it is
 * pure overhead for a probe that is already slow.  Nine clocks per frame back
 * is worth having in the throughput budget.
 */
#define DAP_FRAME_LEAD_CLOCKS  2

/*
 * Recovery: after a CRC error, a lost reply or an aborted block transfer, the
 * host must clock at least MAXWAIT8 cycles with DAP1 low so the device can
 * finish any pending reply and fall back to Active::RECEIVE.  A new command
 * sent before that is not guaranteed to be seen.
 */
#define DAP_RESYNC_CLOCKS      DAP_MAXWAIT_RESET_CYCLES
#define DAP_TRAINING_DATA      0xAAAAAA83u

/* Cold-attach dapisc: signature plus the register value the reference writes. */
#define DAP_DAPISC_SIGNATURE   0x4ABBAF53u
#define DAP_DAPISC_VALUE       0x0F00u

#define DAP_SYNC_EXPECT        0xAAAAAAAAu
#define DAP_SYNC_WIRE_WORD     0x09FE1u

/* Widest reply this layer reads in one go: start bit is consumed separately. */
#define DAP_REPLY_MAX_BITS     64

esp_err_t dap_probe_init(uint32_t clock_hz)
{
#if !AEL_BOARD_HAS_DAP_PROBE
    ESP_LOGW(TAG, "board has no DAP probe wiring");
    return ESP_ERR_NOT_SUPPORTED;
#else
    const dap_phy_cfg_t cfg = {
        .clk_pin  = AEL_DAP0_PIN,
        .dat_pin  = AEL_DAP1_PIN,
        .dir_pin  = AEL_DAP1_DIR_PIN,
        .trst_pin = AEL_DAP_TRST_PIN,
        .clock_hz = clock_hz ? clock_hz : 1000000u,
    };
    return dap_phy_init(&cfg);
#endif
}

esp_err_t dap_probe_park_idle(void)
{
#if !AEL_BOARD_HAS_DAP_PROBE
    return ESP_ERR_NOT_SUPPORTED;
#else
    const esp_err_t err = dap_probe_init(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    if (err != ESP_OK) {
        return err;
    }
    /* dap_phy_init() already parks clock low, data high and TRST released;
     * say so in the log, because a target that was held in reset coming back
     * to life at this exact point is the symptom this explains. */
    ESP_LOGI(TAG, "DAP pins parked: TRST released (GPIO%d high), clock low",
             AEL_DAP_TRST_PIN);
    return ESP_OK;
#endif
}

/*
 * Clock a frame out, turn the line around, wait out the busy stuffing and read
 * `reply_bits` of payload plus six CRC bits.
 *
 * The upstream CRC convention is the one part of this not pinned by
 * documentation, so the residue check is recorded rather than enforced: a
 * mismatch here is interesting, not fatal, and the raw payload is reported
 * either way.
 */
static esp_err_t exchange(const dap_frame_t *frame, size_t reply_bits,
                          dap_exchange_t *out)
{
    uint8_t bits[DAP_REPLY_MAX_BITS + 6];

    if (!dap_phy_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (reply_bits > DAP_REPLY_MAX_BITS) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    out->sent_word = dap_frame_word(frame);
    out->sent_bits = frame->len;

    /*
     * Idle clocks before every frame, not just the first.
     *
     * Measured: a sync preceded by eleven low clocks is answered every time,
     * while the identical frame sent straight after a previous exchange is
     * ignored - and that holds for sync itself, so it was never about which
     * command.  The device needs the idle run to recognise a frame start.
     * This is the same eleven clocks the reference probe sends before its own
     * sync, which it turns out are not a one-off attach ritual.
     */
    dap_phy_idle_clocks(DAP_FRAME_LEAD_CLOCKS, 0);
    dap_phy_write_frame(frame);
    dap_phy_turnaround_to_read();

    out->wait_cycles = dap_phy_await_start_bit(DAP_MAXWAIT_RESET_CYCLES);
    if (out->wait_cycles == DAP_AWAIT_IDLE_HIGH) {
        out->idle_high = true;
    }
    if (out->wait_cycles < 0) {
        out->timed_out = true;
        dap_phy_turnaround_to_write();
        /* Flush the device back to Active::RECEIVE before anything else is
         * attempted; see DAP_RESYNC_CLOCKS. */
        dap_phy_idle_clocks(DAP_RESYNC_CLOCKS, 0);
        return ESP_ERR_TIMEOUT;
    }

    /* The timeout counter is deactivated by the start bit, so from here the
     * clock may stop anywhere with no consequence. */
    dap_phy_read_bits(bits, reply_bits + 6);
    /*
     * Then a short trailer, still with the target driving.  The measured sync
     * window ends around bit 42 while payload plus CRC accounts for 38, so
     * something follows the CRC; leaving it unclocked risks parking the device
     * mid-frame, which would explain why sync answers and every later frame
     * finds the device back in receive and ignoring us.
     */
    uint8_t trailer[24];
    if (s_trailer_bits > sizeof(trailer)) {
        s_trailer_bits = sizeof(trailer);
    }
    if (s_trailer_bits) {
        dap_phy_read_bits(trailer, s_trailer_bits);
    }
    dap_phy_turnaround_to_write();

    for (size_t i = 0; i < reply_bits; i++) {
        out->reply |= (uint64_t)(bits[i] & 1u) << i;
    }
    for (size_t i = 0; i < 6; i++) {
        out->reply_crc |= (uint8_t)((bits[reply_bits + i] & 1u) << i);
    }
    out->reply_bits = reply_bits;
    out->crc_ok = dap_crc6_residue_ok(bits, reply_bits + 6);

    return ESP_OK;
}

esp_err_t dap_probe_sync(dap_exchange_t *out)
{
    dap_frame_t f;

    if (!dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 0)) {
        return ESP_FAIL;
    }
    /*
     * `sync` is the one command sent with LEN all ones, as a JTAG-TAP safety
     * measure while the pins may still be shared: the run of ones parks the TAP
     * instead of risking a drift into instruction execution during a hot plug.
     *
     * It is preceded by eleven clocks with the line *low*, which is what the
     * reference probe does - a 3-bit write of zeros then an 8-bit one, at
     * 400 kHz.  An earlier draft used eight clocks held high, which was a
     * guess and is now known to be wrong.
     */
    return exchange(&f, 32, out);
}

esp_err_t dap_probe_dapisc_read(dap_exchange_t *out)
{
    dap_frame_t f;

    /*
     * The cold-attach form: LEN 48, carrying the 16-bit register value in the
     * bits sent first and the 32-bit signature 0x4ABBAF53 above it.
     *
     * A USB capture of a miniWiggler attaching to this target shows exactly
     * this frame - LEN 48, data 0x4ABBAF530F00 - and the register half 0x0F00
     * sets MAXWAIT8 to 15 with MW8E clear.  The 16-bit form this code sent
     * first drew no reply at all, which is consistent with the handshake not
     * completing without the signature.
     */
    const uint64_t data = ((uint64_t)DAP_DAPISC_SIGNATURE << 16) | DAP_DAPISC_VALUE;

    if (!dap_frame_build(&f, DAP_CMD_DAPISC, 48, data, 48)) {
        return ESP_FAIL;
    }
    /*
     * No reply is read back.  Measured on this target, the window after a
     * LEN-48 dapisc is 200 bits of solid idle high - nothing drives the line -
     * and the reference capture agrees once read correctly: the alternating
     * pattern that turns up after its dapisc is sync's late reply, not
     * dapisc's own.  The plan's "the register echoed back" does not hold for
     * the cold-attach form.
     */
    return exchange(&f, 0, out);
}

esp_err_t dap_probe_client_set(uint8_t client, dap_exchange_t *out)
{
    dap_frame_t f;

    if (!dap_frame_build(&f, DAP_CMD_CLIENT_SET, 3, client, 3)) {
        return ESP_FAIL;
    }
    /* The acknowledge is a bare start bit: no payload to read. */
    return exchange(&f, 0, out);
}

esp_err_t dap_probe_client_read(uint8_t io_instruction, uint8_t size_exponent,
                                size_t reply_bits, dap_exchange_t *out)
{
    dap_frame_t f;
    const uint8_t payload = dap_client_read_payload(io_instruction, size_exponent);

    if (!dap_frame_build(&f, DAP_CMD_CLIENT_READ, 7, payload, 7)) {
        return ESP_FAIL;
    }
    return exchange(&f, reply_bits, out);
}

esp_err_t dap_probe_dump_sync_reply(size_t window_bits)
{
    uint8_t     bits[160];
    dap_frame_t f;

    if (window_bits > sizeof(bits)) {
        window_bits = sizeof(bits);
    }
    if (!dap_phy_ready() || !dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 0)) {
        return ESP_FAIL;
    }

    dap_phy_idle_clocks(11, 0);
    dap_phy_write_frame(&f);
    dap_phy_turnaround_to_read();
    dap_phy_read_bits(bits, window_bits);
    dap_phy_turnaround_to_write();

    /* Print as a bit string: the shape of the window is what matters here,
     * not any particular field, so do not impose a frame layout on it. */
    char text[168];
    size_t n = 0;
    for (size_t i = 0; i < window_bits && n < sizeof(text) - 1; i++) {
        text[n++] = bits[i] ? '1' : '0';
    }
    text[n] = ' ';
    ESP_LOGW(TAG, "sync reply window, %u bits raw:", (unsigned)window_bits);
    ESP_LOGW(TAG, "  %s", text);
    return ESP_OK;
}

esp_err_t dap_probe_clock_pin_search(void)
{
    /*
     * DAP1 has to be on the bidirectional pin, so the data line is fixed - but
     * the clock only needs to be an output, and two Port C pins can be one.
     * If the cables are not where we think, trying both is cheaper and more
     * reliable than asking someone to trace a wire.
     *
     * PC01 is not a candidate: it is register-driven and cannot be clocked.
     */
    static const struct { int pin; const char *label; } candidates[] = {
        { AEL_DAP0_PIN,     "GPIO47 -> PC02, J3 pin 23" },
        /*
         * GPIO40 -> PC04 was a candidate until it turned out to be the
         * target's reset on this bench - measured as the only Port C wire the
         * target pulls up, and confirmed by the target refusing to run while
         * it was held low.  Clocking it would reset the target hundreds of
         * times per attempt, so it is deliberately not tried.
         */
    };

    ESP_LOGI(TAG, "--- clock pin search, data fixed on GPIO%d (PC03) ---", AEL_DAP1_PIN);

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        const dap_phy_cfg_t cfg = {
            .clk_pin  = candidates[i].pin,
            .dat_pin  = AEL_DAP1_PIN,
            .dir_pin  = AEL_DAP1_DIR_PIN,
            /* Do not touch the other candidate while it might be the clock. */
            .trst_pin = -1,
            .clock_hz = CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ,
        };
        if (dap_phy_init(&cfg) != ESP_OK) {
            continue;
        }

        dap_exchange_t x;
        const esp_err_t err = dap_probe_sync(&x);
        if (err == ESP_OK) {
            ESP_LOGW(TAG, "clock on %s ANSWERED: wait %d, reply 0x%08" PRIX64,
                     candidates[i].label, x.wait_cycles, x.reply);
            return ESP_OK;
        }
        ESP_LOGI(TAG, "clock on %s: silent", candidates[i].label);
    }

    /* Leave the PHY on the documented assignment. */
    dap_probe_init(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    return ESP_FAIL;
}

esp_err_t dap_probe_attach_sweep(void)
{
    static const uint32_t rates[]  = { 200000u, 500000u, 1000000u };
    static const size_t   idles[]  = { 8u, 64u, 256u };
    static const uint8_t  lens[]   = { 63u, 0u };

    int attempts = 0;
    int answered = 0;

    if (!dap_phy_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "--- attach sweep: rate x idle clocks x TRST x sync LEN ---");

    for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
        dap_phy_set_clock(rates[r]);
        for (size_t i = 0; i < sizeof(idles) / sizeof(idles[0]); i++) {
            for (int trst = 0; trst < 2; trst++) {
                for (size_t l = 0; l < sizeof(lens) / sizeof(lens[0]); l++) {
                    dap_frame_t    f;
                    dap_exchange_t x;

                    /*
                     * The TRST pulse this loop used to do is gone: that pin is
                     * the target's reset here, so "try it with a TAP reset"
                     * meant rebooting the application under test.  If a TAP
                     * reset is ever needed it belongs behind an explicit
                     * opt-in, not inside a sweep.
                     */
                    (void)trst;
                    dap_phy_idle_clocks(idles[i], 0);

                    if (!dap_frame_build(&f, DAP_CMD_SYNC, lens[l], 0, 0)) {
                        continue;
                    }
                    attempts++;
                    if (exchange(&f, 32, &x) == ESP_OK) {
                        answered++;
                        ESP_LOGW(TAG, "ANSWERED: %" PRIu32 " Hz, %zu idle clocks, TRST %s, "
                                      "LEN %u -> wait %d, reply 0x%08" PRIX64,
                                 rates[r], idles[i], trst ? "pulsed" : "held",
                                 lens[l], x.wait_cycles, x.reply);
                    }
                }
            }
        }
    }

    ESP_LOGI(TAG, "--- sweep done: %d/%d combinations answered ---", answered, attempts);
    /* Leave the PHY where the caller expects it. */
    dap_phy_set_clock(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    return answered ? ESP_OK : ESP_FAIL;
}

static void log_exchange(const char *step, const dap_exchange_t *x)
{
    if (x->timed_out) {
        ESP_LOGE(TAG, "%-22s sent 0x%05" PRIX64 " (%zu bits) -> %s within %d clocks",
                 step, x->sent_word, x->sent_bits,
                 x->idle_high ? "line stayed idle high, nothing driving it"
                              : "line held low, no start bit",
                 DAP_MAXWAIT_RESET_CYCLES);
        return;
    }
    ESP_LOGI(TAG, "%-22s sent 0x%05" PRIX64 " (%zu bits) -> wait %d, reply 0x%08" PRIX64
                  " (%zu bits), crc 0x%02X %s",
             step, x->sent_word, x->sent_bits, x->wait_cycles, x->reply,
             x->reply_bits, x->reply_crc, x->crc_ok ? "residue-ok" : "residue-BAD");
}

esp_err_t dap_probe_replay_preamble(void)
{
    /* Verbatim from the capture: 43 bytes, then 7 bytes, clocked LSB first. */
    static const uint8_t pattern[43] = {
        0xF0, 0x03, 0x3F, 0xF0, 0x03, 0x3F, 0xF0, 0x03, 0x3F, 0xF0, 0x03,
        0x3F, 0xF0, 0x03, 0x3F, 0xF0, 0x03, 0x3F, 0xF0, 0x03, 0x3F, 0xF0,
        0x03, 0x3F, 0xF0, 0x03, 0x3F, 0xF0, 0x03, 0x3F, 0xF0, 0x03, 0x3F,
        0xF0, 0x03, 0x3F, 0xF0, 0x03, 0x3F, 0xF0, 0x03, 0x3F, 0x70,
    };
    static const uint8_t tail[7] = { 0x80, 0x1F, 0x38, 0x70, 0xFC, 0xF8, 0x71 };

    if (!dap_phy_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    for (size_t i = 0; i < sizeof(pattern); i++) {
        for (int b = 0; b < 8; b++) {
            dap_phy_idle_clocks(1, (pattern[i] >> b) & 1);
        }
    }
    for (size_t i = 0; i < sizeof(tail); i++) {
        for (int b = 0; b < 8; b++) {
            dap_phy_idle_clocks(1, (tail[i] >> b) & 1);
        }
    }

    /* The reference then reads 30 bytes with the target driving. */
    uint8_t reply[240];
    dap_phy_turnaround_to_read();
    dap_phy_read_bits(reply, sizeof(reply));
    dap_phy_turnaround_to_write();

    int ones = 0;
    for (size_t i = 0; i < sizeof(reply); i++) {
        ones += reply[i];
    }
    ESP_LOGW(TAG, "preamble replayed; read window %u bits, %d high",
             (unsigned)sizeof(reply), ones);
    return ESP_OK;
}

esp_err_t dap_probe_attach_now(dap_exchange_t out[6])
{
    /*
     * The four frames back to back, with nothing between them.
     *
     * The bring-up report logs after every step, which puts milliseconds of
     * host work between frames.  Gaps are supposed to be free - the timeout
     * counts probe clocks - but "supposed to" is what this whole exercise
     * keeps correcting, and the device has answered sync and then ignored
     * everything after it.  This removes the gaps as a variable.
     */
    const uint64_t dapisc = ((uint64_t)DAP_DAPISC_SIGNATURE << 16) | DAP_DAPISC_VALUE;
    dap_frame_t f;

    if (!dap_phy_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    dap_phy_idle_clocks(11, 0);
    if (dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 0)) {
        exchange(&f, 32, &out[0]);
    }
    if (dap_frame_build(&f, DAP_CMD_DAPISC, 48, dapisc, 48)) {
        exchange(&f, 0, &out[1]);
    }
    if (dap_frame_build(&f, DAP_CMD_CLIENT_SET, 3, 1, 3)) {
        exchange(&f, 0, &out[2]);
    }
    if (dap_frame_build(&f, DAP_CMD_CLIENT_READ, 7,
                        dap_client_read_payload(DAP_IO_CLIENT_ID, 4), 7)) {
        exchange(&f, 16, &out[3]);
    }
    return ESP_OK;
}

esp_err_t dap_probe_trailer_sweep(void)
{
    static const size_t trailers[] = { 0, 1, 2, 4, 8, 18 };

    ESP_LOGW(TAG, "--- trailer sweep: 8 syncs per trailer length ---");
    for (size_t t = 0; t < sizeof(trailers) / sizeof(trailers[0]); t++) {
        char   line[16];
        size_t n = 0;

        s_trailer_bits = trailers[t];
        for (int i = 0; i < 8; i++) {
            dap_frame_t    f;
            dap_exchange_t x;

            dap_phy_idle_clocks(11, 0);
            if (!dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 0)) {
                break;
            }
            exchange(&f, 32, &x);
            line[n++] = (x.reply == DAP_SYNC_EXPECT) ? 'o' : '.';
        }
        line[n] = ' ';
        ESP_LOGW(TAG, "  trailer %2u bits: %s", (unsigned)trailers[t], line);
    }
    s_trailer_bits = 0;      /* restore the value that works, not the one tested */
    return ESP_OK;
}

esp_err_t dap_probe_sync_health(int attempts)
{
    /*
     * How many syncs in a row does the device answer, and does anything
     * recover it once it stops?
     *
     * The second-frame matrix showed sync answering four times and then
     * failing, which means the device wedges rather than simply refusing
     * non-sync commands.  Knowing whether a re-sync, a client_reset or the
     * idle clocks bring it back narrows down what state it is falling into.
     */
    char line[80];
    size_t n = 0;

    for (int i = 0; i < attempts && n < sizeof(line) - 4; i++) {
        dap_frame_t    f;
        dap_exchange_t x;

            if (!dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 0)) {
            break;
        }
        exchange(&f, 32, &x);
        line[n++] = (x.reply == DAP_SYNC_EXPECT) ? 'o' : '.';

        /* Every fourth attempt, try a client_reset first and mark it. */
        if ((i % 4) == 3) {
            if (dap_frame_build(&f, DAP_CMD_CLIENT_RESET, 0, 0, 0)) {
                dap_exchange_t r;
                exchange(&f, 0, &r);
            }
            line[n++] = '|';
        }
    }
    line[n] = ' ';
    ESP_LOGW(TAG, "sync health (o=0xAAAAAAAA, .=no reply, |=client_reset): %s", line);
    return ESP_OK;
}

esp_err_t dap_probe_second_frame_matrix(void)
{
    /*
     * One question: can any command other than sync draw a reply?
     *
     * Each row does a fresh sync - which is known to answer - and then exactly
     * one candidate as the second frame.  If every candidate is ignored while
     * every sync answers, the device is accepting only sync, and the reason
     * lies in what enables the rest rather than in any frame we build.  The
     * reference's own order is sync, dapisc, client_set, client_set,
     * client_read, so these are the frames that should work.
     */
    struct { const char *name; uint8_t cmd; uint8_t len; uint64_t data; size_t db; size_t reply; } rows[] = {
        { "dapisc LEN16",   DAP_CMD_DAPISC,      16, 0, 16, 16 },
        { "dapisc LEN48",   DAP_CMD_DAPISC,      48,
          ((uint64_t)DAP_DAPISC_SIGNATURE << 16) | DAP_DAPISC_VALUE, 48, 0 },
        { "client_set(1)",  DAP_CMD_CLIENT_SET,   3, 1, 3, 0 },
        { "client_read ID", DAP_CMD_CLIENT_READ,  7, 0x4B, 7, 16 },
        { "poll",           0x12,                 0, 0, 0, 8 },
        { "sync again",     DAP_CMD_SYNC,        63, 0, 0, 32 },
    };

    ESP_LOGW(TAG, "--- second-frame matrix: fresh sync, then one candidate ---");
    for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        dap_frame_t    f;
        dap_exchange_t s1, s2;

        dap_phy_idle_clocks(11, 0);
        if (!dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 0)) {
            continue;
        }
        exchange(&f, 32, &s1);

        if (!dap_frame_build(&f, rows[i].cmd, rows[i].len, rows[i].data, rows[i].db)) {
            continue;
        }
        exchange(&f, rows[i].reply, &s2);

        ESP_LOGW(TAG, "  sync %s -> %-15s %s reply 0x%08" PRIX64,
                 s1.reply == DAP_SYNC_EXPECT ? "ok  " : "FAIL",
                 rows[i].name,
                 s2.idle_high ? "idle-high" : (s2.timed_out ? "held-low " : "ANSWERED "),
                 s2.reply);
    }
    return ESP_OK;
}

esp_err_t dap_probe_bringup_report(void)
{
    dap_exchange_t x;
    esp_err_t      err;
    int            failures = 0;

    ESP_LOGI(TAG, "=== DAP bring-up: DAP0=GPIO%d DAP1=GPIO%d dir=GPIO%d trst=GPIO%d ===",
             AEL_DAP0_PIN, AEL_DAP1_PIN, AEL_DAP1_DIR_PIN, AEL_DAP_TRST_PIN);

    /*
     * Wire check first.  A silent target has several causes, and these two
     * measurements separate "our side is broken" from "nothing is listening"
     * before any protocol theory gets involved.
     */
    const bool self_ok = dap_phy_self_drive_check();
    const int  idle_high = dap_phy_sample_target_idle(32);
    ESP_LOGI(TAG, "0 wire check          self-drive %s, DAP1 idle %d/32 high",
             self_ok ? "ok" : "FAILED", idle_high);
    if (!self_ok) {
        ESP_LOGE(TAG, "   the S3 cannot read back its own drive: check Port C mode "
                      "(cfgpc) and that swd_gpio selects the GPIO path");
    }
    /*
     * Measured on this board with the analyser: the DAP1 net has no pull-up and
     * no pull-down.  Released from low it stays low for milliseconds, released
     * from high it stays high.  So this reading reports the charge left by
     * whatever we last drove, and says nothing at all about the target.  It is
     * kept because a *change* in it across a run is still worth seeing.
     */
    ESP_LOGI(TAG, "   (idle level is residual charge on an unpulled net, not "
                  "evidence of a target)");
    (void)idle_high;

    /* Checkpoint 1: sync must draw the 0xAAAAAAAA training pattern. */
    err = dap_probe_sync(&x);
    log_exchange("1 sync", &x);
    if (x.sent_word != DAP_SYNC_WIRE_WORD) {
        ESP_LOGE(TAG, "   frame assembly is wrong: expected wire word 0x%05X", DAP_SYNC_WIRE_WORD);
        failures++;
    }
    if (err != ESP_OK || x.reply != DAP_SYNC_EXPECT) {
        ESP_LOGE(TAG, "   expected 0x%08" PRIX32 " - target silent or framing off",
                 (uint32_t)DAP_SYNC_EXPECT);
        failures++;
        /* Everything downstream assumes sync worked, so rather than emit a
         * cascade of failures with one cause, try the cheap attach variants. */
        if (dap_probe_clock_pin_search() == ESP_OK ||
            dap_probe_attach_sweep() == ESP_OK) {
            ESP_LOGW(TAG, "=== a sweep combination answered: adopt it and re-run ===");
        } else {
            ESP_LOGE(TAG, "=== target silent on every attach variant ===");
        }
        return ESP_FAIL;
    }

    /* Measure the real shape of a reply window before trusting any length. */
    dap_probe_dump_sync_reply(120);

    dap_probe_trailer_sweep();
    dap_probe_sync_health(12);
    dap_probe_second_frame_matrix();

    /* And the whole handshake with no host work between frames. */
    {
        static const char *names[6] = { "sync", "dapisc", "client_set(1)",
                                        "client_read ID", "spare", "spare" };
        dap_exchange_t seq[6];
        memset(seq, 0, sizeof(seq));
        dap_probe_attach_now(seq);
        ESP_LOGW(TAG, "--- gapless handshake ---");
        for (int i = 0; i < 6; i++) {
            ESP_LOGW(TAG, "  %-15s wait %-5d reply 0x%08" PRIX64 " %s",
                     names[i], seq[i].wait_cycles, seq[i].reply,
                     seq[i].idle_high ? "(idle high)"
                                      : (seq[i].timed_out ? "(held low)" : ""));
        }
        if (seq[3].reply == DAP_CLIENT_ID_EXPECT) {
            ESP_LOGW(TAG, "  CLIENT_ID 0x0260 - the gapless sequence works");
        }
    }

    /* Checkpoint 2: DAPISC.  Dump its window first, for the same reason. */
    {
        dap_frame_t df;
        const uint64_t d = ((uint64_t)DAP_DAPISC_SIGNATURE << 16) | DAP_DAPISC_VALUE;
        if (dap_frame_build(&df, DAP_CMD_DAPISC, 48, d, 48)) {
            uint8_t raw[200];
            char    text[208];
            dap_phy_write_frame(&df);
            dap_phy_turnaround_to_read();
            dap_phy_read_bits(raw, sizeof(raw));
            dap_phy_turnaround_to_write();
            size_t n = 0;
            for (size_t i = 0; i < sizeof(raw) && n < sizeof(text) - 1; i++) {
                text[n++] = raw[i] ? '1' : '0';
            }
            text[n] = ' ';
            ESP_LOGW(TAG, "dapisc reply window, %u bits raw:", (unsigned)sizeof(raw));
            ESP_LOGW(TAG, "  %s", text);
        }
    }
    err = dap_probe_dapisc_read(&x);
    log_exchange("2 dapisc write", &x);
    if (err != ESP_OK) {
        /* Expected: this form takes no reply, so a timeout here is not a
         * failure.  It is logged so a change in behaviour is still visible. */
        ESP_LOGI(TAG, "   no reply, as expected for the LEN-48 write form");
    } else {
        ESP_LOGI(TAG, "   MAXWAIT8=%u MW8E=%u -> %u wait clocks allowed",
                 (unsigned)((x.reply >> 8) & 0x1Fu), (unsigned)((x.reply >> 13) & 1u),
                 (unsigned)(((x.reply >> 8) & 0x1Fu) * 8u *
                            (1u + ((x.reply >> 13) & 1u) * 15u)));
    }

    /* Checkpoint 3: select the Cerberus IOClient. */
    err = dap_probe_client_set(1, &x);
    log_exchange("3 client_set(1)", &x);
    if (err != ESP_OK) {
        failures++;
    }

    /* Checkpoint 4: CLIENT_ID.  This is the one that says the probe is real. */
    err = dap_probe_client_read(DAP_IO_CLIENT_ID, 4, 16, &x);
    log_exchange("4 client_read ID", &x);
    {
        /* 32-bit form: the hardware replicates the halfword, so a correct
         * transport returns 0x02600260 and confirms the width handling too. */
        dap_exchange_t w;
        if (dap_probe_client_read(DAP_IO_CLIENT_ID, 5, 32, &w) == ESP_OK) {
            ESP_LOGI(TAG, "   32-bit CLIENT_ID reads 0x%08" PRIX64 "%s",
                     w.reply, w.reply == 0x02600260u ? " (expected)" : "");
        }
        dap_exchange_t info;
        if (dap_probe_client_read(DAP_IO_INFO, 4, 16, &info) == ESP_OK) {
            ESP_LOGI(TAG, "   IOINFO reads 0x%04" PRIX64, info.reply);
        }
    }
    if (err != ESP_OK || x.reply != DAP_CLIENT_ID_EXPECT) {
        ESP_LOGE(TAG, "   expected CLIENT_ID 0x%04X", DAP_CLIENT_ID_EXPECT);
        failures++;
    } else {
        ESP_LOGI(TAG, "   CLIENT_ID 0x0260: the probe is talking to Cerberus");
    }

    ESP_LOGI(TAG, "=== bring-up %s (%d failure%s) ===",
             failures ? "INCOMPLETE" : "PASSED", failures, failures == 1 ? "" : "s");
    return failures ? ESP_FAIL : ESP_OK;
}
