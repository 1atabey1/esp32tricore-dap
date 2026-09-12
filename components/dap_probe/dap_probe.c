#include "dap_probe.h"
#include "dap_probe_priv.h"

#include <inttypes.h>
#include <string.h>

#include "board_profile.h"
#include "dap_frame.h"
#include "dap_phy.h"
#include "dap_phy_fpga.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "DAP";

/*
 * Bits clocked after a reply's CRC, with the target still driving.
 *
 * Replies alternate - every second exchange answers - which is what leaving a
 * fixed amount of state behind per exchange looks like.  Making this adjustable
 * turns that into a measurement instead of a guess.
 */
size_t dap_probe_trailer_bits;   /* zero: measured to be what the device wants */
static size_t s_raw_window;     /* non-zero: dump this many raw reply bits */

/*
 * How long to wait for a reply's start bit, in probe clocks.
 *
 * Starts at the DAPISC reset allowance and is re-derived whenever dapisc
 * reports a new value, because the window is a property of the device's
 * current configuration rather than a constant.  Getting this wrong is not
 * harmless: the reference sets MAXWAIT8 to 15, which is 120 clocks, and a
 * probe still waiting 248 then flushing 248 more on the timeout path turns the
 * first read after a dapisc into a failure that the *next* read silently
 * recovers from.
 */
uint32_t dap_probe_max_wait = DAP_MAXWAIT_RESET_CYCLES;



/* Widest reply this layer reads in one go: start bit is consumed separately. */
#define DAP_REPLY_MAX_BITS     64

/*
 * How many clocks to issue after a reply, with the target still driving.
 *
 * Zero is right for the bit-banged backend, which issues one implicitly: it
 * samples before raising the clock, so it is always one clock ahead of the
 * cells it has looked at.  The SPI backend samples on the edge and has no such
 * offset, so what the device needs after a reply becomes an explicit number
 * here rather than an accident of how the sampling loop is written.
 */
/*
 * Dump `bits` raw reply bits instead of decoding, for every exchange until it
 * is set back to zero.  The frames sent are the real ones - that is the point.
 */
void dap_probe_set_raw_window(size_t bits)
{
    s_raw_window = bits;
}

void dap_probe_set_trailer_bits(size_t n)
{
    dap_probe_trailer_bits = n;
}

size_t dap_probe_get_trailer_bits(void)
{
    return dap_probe_trailer_bits;
}

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
esp_err_t dap_probe_exchange(const dap_frame_t *frame, size_t reply_bits,
                          dap_exchange_t *out)
{
    uint8_t bits[DAP_REPLY_MAX_BITS + 6];

    /*
     * The fabric does a whole exchange by itself, so when it is driving there
     * is nothing here to assemble: hand it the frame's fields and take the
     * answer.  Everything above this - read32, OCDS enable, the GDB target, the
     * trace drain - is unchanged either way, which is the point of putting the
     * switch at this level rather than in the PHY.
     */
    if (dap_phy_fpga_in_use()) {
        uint32_t reply = 0;
        uint16_t waited = 0;

        memset(out, 0, sizeof(*out));
        out->sent_word = dap_frame_word(frame);
        out->sent_bits = frame->len;

        const esp_err_t err = dap_phy_fpga_exchange(
            frame->cmd, frame->len_field, frame->data, frame->data_bits,
            reply_bits, &reply, &waited);

        out->reply       = reply;
        out->reply_bits  = reply_bits;
        out->wait_cycles = (int)waited;
        out->crc_ok      = (err == ESP_OK);
        out->timed_out   = (err == ESP_ERR_TIMEOUT);
        out->idle_high   = (err == ESP_ERR_NOT_FOUND);
        return err;
    }

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
    dap_phy_write_frame_with_lead(frame, DAP_FRAME_LEAD_CLOCKS);
    dap_phy_turnaround_to_read();

    if (s_raw_window) {
        uint8_t raw[96];
        char    text[104];
        size_t  n = s_raw_window > sizeof(raw) ? sizeof(raw) : s_raw_window;

        dap_phy_read_bits(raw, n);
        dap_phy_turnaround_to_write();
        size_t k = 0;
        for (size_t i = 0; i < n && k < sizeof(text) - 1; i++) {
            text[k++] = raw[i] ? '1' : '0';
        }
        text[k] = '\0';
        ESP_LOGW(TAG, "  raw CMD 0x%02X: %s", frame->bit[1] | (frame->bit[2] << 1) |
                 (frame->bit[3] << 2) | (frame->bit[4] << 3) | (frame->bit[5] << 4), text);
        return ESP_OK;
    }

    out->wait_cycles = dap_phy_read_reply(bits, reply_bits, dap_probe_max_wait + DAP_WAIT_MARGIN);
    if (out->wait_cycles < 0) {
        out->timed_out = true;
        dap_phy_turnaround_to_write();
        /* Flush the device back to Active::RECEIVE before anything else is
         * attempted; see DAP_RESYNC_CLOCKS. */
        dap_phy_idle_clocks(dap_probe_max_wait, 0);
        return ESP_ERR_TIMEOUT;
    }

    /*
     * The timeout counter is deactivated by the start bit, so from here the
     * clock may stop anywhere with no consequence.
     *
     * A reply that carries data is [start][RDATA][CRC6]; a bare acknowledge -
     * client_set's, for one - is the start bit and nothing else.  Clocking six
     * phantom CRC bits after an acknowledge runs the clock into the next
     * telegram's space and gets the *following* command discarded, which then
     * looks like an intermittent failure of that command rather than of this
     * one.
     */
    const size_t to_read = reply_bits ? reply_bits + 6 : 0;
    if (to_read) {
        /*
         * All ones, CRC included, is not a reply - it is an undriven wire read
         * as a reply.  This is where that is visible: the start-bit search
         * cannot tell a floating high line from a legitimate start bit, and
         * should not try, because requiring a leading zero broke the SPI
         * backend's alignment.
         */
        size_t ones = 0;
        for (size_t i = 0; i < to_read; i++) {
            ones += bits[i] ? 1u : 0u;
        }
        out->idle_high = (ones == to_read);
    }
    /*
     * Then a short trailer, still with the target driving.  The measured sync
     * window ends around bit 42 while payload plus CRC accounts for 38, so
     * something follows the CRC; leaving it unclocked risks parking the device
     * mid-frame, which would explain why sync answers and every later frame
     * finds the device back in receive and ignoring us.
     */
    uint8_t trailer[24];
    if (dap_probe_trailer_bits > sizeof(trailer)) {
        dap_probe_trailer_bits = sizeof(trailer);
    }
    if (dap_probe_trailer_bits) {
        dap_phy_read_bits(trailer, dap_probe_trailer_bits);
    }
    dap_phy_turnaround_to_write();

    for (size_t i = 0; i < reply_bits; i++) {
        out->reply |= (uint64_t)(bits[i] & 1u) << i;
    }
    out->reply_bits = reply_bits;
    if (reply_bits) {
        for (size_t i = 0; i < 6; i++) {
            out->reply_crc |= (uint8_t)((bits[reply_bits + i] & 1u) << i);
        }
        out->crc_ok = dap_crc6_residue_ok(bits, reply_bits + 6);
    } else {
        /* Nothing to check: the acknowledge is the start bit itself. */
        out->crc_ok = true;
    }

    /*
     * A reply whose residue fails is not a reply.
     *
     * The start-bit search can still be fooled: late in a long wait window a
     * glitch reads as low-then-high, the caller then clocks in idle-high bits,
     * and the result is a confident 0xFFFF or 0xFFFFFFFF with a wait count
     * near the window limit.  That is how four addresses - including a
     * free-running timer read twice - all came back as 0xFFFFFFFF and were
     * reported as successful reads.  The CRC is the discriminator and it was
     * being recorded rather than enforced.
     */
    if (!out->crc_ok) {
        dap_phy_idle_clocks(dap_probe_max_wait, 0);      /* flush to Active::RECEIVE */
        return ESP_ERR_INVALID_CRC;
    }

    /*
     * An acknowledge carries no CRC, so it is the one reply that cannot be
     * checked - and a phantom start bit late in the wait window is
     * indistinguishable from a real one by content.  It is distinguishable by
     * *when* it arrives: the device answers within a few cycles of the
     * command, while a glitch turns up near the limit.  Anything past
     * DAP_ACK_MAX_WAIT is treated as no acknowledge, which stops a write from
     * being reported as landed when nothing received it.
     */
    if (reply_bits == 0 && out->wait_cycles > DAP_ACK_MAX_WAIT) {
        out->timed_out = true;
        dap_phy_idle_clocks(dap_probe_max_wait, 0);
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t dap_probe_attach(dap_exchange_t *out, int attempts)
{
    /*
     * Sync, retrying after a flush.  The first attach following a board reset
     * regularly fails where the second succeeds, and MAXWAIT8 low clocks is
     * the documented way to put the device back in Active::RECEIVE, so a
     * retry is cheaper and more honest than reporting a dead target.
     */
    esp_err_t err = ESP_FAIL;

    for (int i = 0; i < attempts; i++) {
        err = dap_probe_sync(out);
        const bool wide = dap_phy_fpga_is_wide();
        if ((wide || err == ESP_OK) &&
            out->reply == (wide ? DAP_SYNC_EXPECT_WIDE : DAP_SYNC_EXPECT)) {
            if (i) {
                ESP_LOGI(TAG, "sync succeeded on attempt %d", i + 1);
            }
            return ESP_OK;
        }
        dap_phy_idle_clocks(dap_probe_max_wait, 0);
    }
    return err == ESP_OK ? ESP_FAIL : err;
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
    return dap_probe_exchange(&f, 32, out);
}

esp_err_t dap_probe_dapisc(uint16_t value, bool cold, dap_exchange_t *out)
{
    dap_frame_t f;

    /*
     * Two telegram variants, and picking the wrong one gets silence.
     *
     * The long form - LEN 48, 66 bits, carrying the 16-bit value plus the
     * 32-bit signature 0x4ABBAF53 - is the *initialisation* telegram, for use
     * straight after PORST release or on the Enabled-to-Active transition.
     * Once the device is Active, which a sync reply proves, the short form
     * (LEN 16, no signature) is what reconfigures DAP options.
     *
     * Either way the device replies with the newly updated 16-bit DAPISC
     * value, and it drives the start bit exactly 3 DAP0 cycles after the
     * command's last CRC bit - a fixed delay, not wait stuffing.  A discarded
     * command produces no reply at all: a CRC failure, a signature shifted the
     * wrong way round, or an unsynchronised attach all look identical from
     * here, which is why this code sends sync first and checks it answered.
     */
    if (cold) {
        const uint64_t data = ((uint64_t)DAP_DAPISC_SIGNATURE << 16) | value;
        if (!dap_frame_build(&f, DAP_CMD_DAPISC, 48, data, 48)) {
            return ESP_FAIL;
        }
    } else if (!dap_frame_build(&f, DAP_CMD_DAPISC, 16, value, 16)) {
        return ESP_FAIL;
    }
    const esp_err_t err = dap_probe_exchange(&f, 16, out);
    if (err == ESP_OK) {
        dap_probe_note_dapisc((uint16_t)out->reply);
    }
    return err;
}

/*
 * Adopt the wait window the device is now configured for.
 *
 *   T_timeout = MAXWAIT8 * 8 * (1 + MW8E * 15)   DAP0 clocks
 *
 * MAXWAIT8 = 0 disables the device's timeout entirely, which means an internal
 * bus lockup would hang the probe; cap it rather than wait forever.
 */
void dap_probe_note_dapisc(uint16_t dapisc)
{
    const uint32_t maxwait8 = (dapisc >> 8) & 0x1Fu;
    const uint32_t mw8e     = (dapisc >> 13) & 1u;

    dap_probe_max_wait = maxwait8 ? maxwait8 * 8u * (1u + mw8e * 15u)
                          : DAP_MAXWAIT_GENEROUS_CYCLES;
    ESP_LOGI(TAG, "wait window now %" PRIu32 " clocks (MAXWAIT8=%" PRIu32
                  " MW8E=%" PRIu32 ")", dap_probe_max_wait, maxwait8, mw8e);
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
    return dap_probe_exchange(&f, 0, out);
}

esp_err_t dap_probe_client_set(uint8_t client, dap_exchange_t *out)
{
    dap_frame_t f;

    if (!dap_frame_build(&f, DAP_CMD_CLIENT_SET, 3, client, 3)) {
        return ESP_FAIL;
    }
    /* The acknowledge is a bare start bit: no payload to read. */
    return dap_probe_exchange(&f, 0, out);
}

esp_err_t dap_probe_client_read(uint8_t io_instruction, uint8_t size_exponent,
                                size_t reply_bits, dap_exchange_t *out)
{
    dap_frame_t f;
    const uint8_t payload = dap_client_read_payload(io_instruction, size_exponent);

    if (!dap_frame_build(&f, DAP_CMD_CLIENT_READ, 7, payload, 7)) {
        return ESP_FAIL;
    }
    return dap_probe_exchange(&f, reply_bits, out);
}

esp_err_t dap_probe_client_write(uint8_t io_instruction, uint8_t size_exponent,
                                 uint64_t data, size_t data_bits,
                                 dap_exchange_t *out)
{
    dap_frame_t f;

    /*
     * A write telegram carries no size exponent - unlike a read, where the
     * 7-bit selector holds one.  The width comes from LEN alone:
     *
     *   LEN     = 4 + n          (4-bit IO instruction plus n data bits)
     *   payload = [instruction][data], both LSB first
     *
     * The reply is a bare start bit with no data and no CRC6, unless
     * DAPISC.RC6 is enabled.
     */
    (void)size_exponent;
    if (data_bits > 32) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dap_frame_build(&f, DAP_CMD_CLIENT_WRITE, (uint8_t)(4u + data_bits),
                         ((uint64_t)data << 4) | (io_instruction & 0x0Fu),
                         4u + data_bits)) {
        return ESP_FAIL;
    }
    /* Writes acknowledge with a bare start bit, like client_set. */
    return dap_probe_exchange(&f, 0, out);
}

esp_err_t dap_probe_set_rw_mode(bool supervisor)
{
    dap_exchange_t x;
    const uint16_t conf = DAP_IOCONF_MODE_RW |
                          (uint16_t)(supervisor ? DAP_IOCONF_SVM : 0u);

    const esp_err_t err = dap_probe_client_write(DAP_IO_CONF, 4, conf,
                                                DAP_IOCONF_BITS, &x);
    ESP_LOGI(TAG, "IOCONF <- 0x%04X: %s after %d cycles", conf,
             err == ESP_OK ? "acknowledged" : "NOT acknowledged", x.wait_cycles);
    return err;
}

void dap_probe_log_ioinfo(uint16_t v)
{
    ESP_LOGI(TAG, "   IOINFO 0x%04X:%s%s%s%s%s%s%s%s", v,
             (v & DAP_IOINFO_IDLE)        ? " IDLE"        : "",
             (v & DAP_IOINFO_PWR_DWN)     ? " PWR_DWN"     : "",
             (v & DAP_IOINFO_BUS_RD_ERR)  ? " BUS_RD_ERR"  : "",
             (v & DAP_IOINFO_BUS_WR_ERR)  ? " BUS_WR_ERR"  : "",
             (v & DAP_IOINFO_PWR_DWN_ERR) ? " PWR_DWN_ERR" : "",
             (v & DAP_IOINFO_ENDINIT)     ? " ENDINIT"     : "",
             (v & DAP_IOINFO_BUS_RST)     ? " BUS_RST"     : "",
             (v & DAP_IOINFO_IF_LCK)      ? " IF_LCK"      : "");
}

esp_err_t dap_probe_clear_error_state(void)
{
    dap_exchange_t x;

    /*
     * A bus error or protection fault puts Cerberus in Error State, where all
     * IO_READ_* and IO_WRITE_* instructions are silently dropped.  Executing
     * IO_SUPERVISOR - instruction 0xB, the same one that reads IOINFO - clears
     * it.  Worth doing before a sequence, because the state survives whatever
     * caused it, including an earlier session's mistake.
     */
    const esp_err_t err = dap_probe_client_read(DAP_IO_INFO, 4, 16, &x);
    if (err == ESP_OK && (x.reply & (DAP_IOINFO_BUS_RST | DAP_IOINFO_IF_LCK |
                                     DAP_IOINFO_BUS_RD_ERR | DAP_IOINFO_BUS_WR_ERR))) {
        dap_probe_log_ioinfo((uint16_t)x.reply);
    }
    return err;
}

esp_err_t dap_probe_read32(uint32_t addr, uint32_t *value)
{
    dap_exchange_t x;

    /*
     * IO_SET_ADDRESS (instruction 0x1) loads IOADDR - 32 bits for a full
     * address, or 16 bits to change only the low half, which saves shift
     * cycles for random access inside a 64 kB window.  Then IO_READ_WORD
     * (0x5) at size exponent 5 returns the 32-bit word.
     */
    esp_err_t err = dap_probe_client_write(DAP_IO_SET_ADDRESS, 5, addr, 32, &x);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "IOADDR write for 0x%08" PRIX32 " not acknowledged (%s)",
                 addr, x.idle_high ? "idle high" : "no start bit in time");
        return err;
    }
    ESP_LOGD(TAG, "IOADDR write acknowledged after %d cycles", x.wait_cycles);

    err = dap_probe_client_read(DAP_IO_READ_WORD, 5, 32, &x);
    if (err != ESP_OK) {
        return err;
    }
    if (!x.crc_ok) {
        ESP_LOGW(TAG, "read of 0x%08" PRIX32 " has a bad CRC residue", addr);
    }
    *value = (uint32_t)x.reply;
    return ESP_OK;
}

esp_err_t dap_probe_write32(uint32_t addr, uint32_t value)
{
    dap_exchange_t x;
    esp_err_t err = dap_probe_client_write(DAP_IO_SET_ADDRESS, 5, addr, 32, &x);

    if (err != ESP_OK) {
        return err;
    }
    return dap_probe_client_write(DAP_IO_WRITE_WORD, 5, value, 32, &x);
}

esp_err_t dap_probe_enable_ocds(void)
{
    /*
     * Turn OCDS on, which the miniMCDS register space and the TRAM behind it
     * need: with OSTATE.OEN clear, anything in 0xFB718000..0xFB71FFFF raises a
     * bus error on the SRI slave interface.
     *
     * The four pattern writes to OEC.PAT must be contiguous.  Any other write
     * to OEC in between, or any deviation in the values, resets the hardware
     * matcher to its first step - which is why this is one function and not a
     * sequence a caller can interleave with anything.
     */
    static const uint32_t pattern[4] = { 0xA1u, 0x5Eu, 0xA1u, 0x5Eu };
    uint32_t ostate = 0;

    for (size_t i = 0; i < 4; i++) {
        const esp_err_t err = dap_probe_write32(DAP_ADDR_OEC_PAT, pattern[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OEC.PAT write %u of 4 failed; matcher is now reset",
                     (unsigned)(i + 1));
            return err;
        }
    }

    if (dap_probe_read32(DAP_ADDR_OSTATE, &ostate) != ESP_OK) {
        ESP_LOGE(TAG, "OSTATE unreadable after the enable pattern");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OSTATE = 0x%08" PRIX32 " (OEN=%" PRIu32 ")", ostate, ostate & 1u);
    if (!(ostate & 1u)) {
        return ESP_FAIL;
    }

    /* OC4 with its protection bit: sets OSTATE.EECTRC and routes the core
     * trace lines to the miniMCDS. */
    if (dap_probe_write32(DAP_ADDR_OCNTRL, 0x0300u) != ESP_OK) {
        ESP_LOGW(TAG, "OCNTRL write failed");
        return ESP_FAIL;
    }

    /*
     * Then the module's own clock.  OEN alone leaves the miniMCDS unreadable:
     * enabling OCDS makes the register space *reachable*, but the block is
     * still clock-gated, so CLC has to be written to zero to run it.  A CLC
     * register stays accessible while its module is disabled - that is how a
     * module gets enabled at all.
     */
    if (dap_probe_write32(DAP_ADDR_MCDS_CLC, 0x00000000u) != ESP_OK) {
        ESP_LOGW(TAG, "miniMCDS CLC write failed");
    }

    /* CT.SETE unlocks writes to the rest of the miniMCDS space. */
    return dap_probe_write32(DAP_ADDR_MCDS_CT, 0x8000u);
}

esp_err_t dap_probe_blockread(uint32_t addr, uint32_t *words, size_t count)
{
    /*
     * client_blockread: one telegram, many words, and the reason this project
     * exists.  A single-word read costs a whole frame plus a reply per 4 bytes;
     * a block read amortises the frame over up to 256 words.
     *
     * Telegram 0x0A, with the payload
     *   bit 0      request the 32-bit block CRC (CRCup) as a final parcel
     *   bit 1      CRC6 after every parcel, rather than only the last
     *   bits 9:2   word count, 1..255, with 0 meaning 256 words (1 kB)
     *   bits 39:10 word-aligned address, shifted right by two
     * and LEN selecting the address form: 10 for none, 24 for a 14-bit
     * address, 40 for a 30-bit one.  With an address present the device loads
     * IOADDR itself, so no separate IO_SET_ADDRESS is needed, and IOADDR
     * post-increments by four per word.
     */
    dap_frame_t f;
    uint8_t     bits[32];

    if (count == 0 || count > 256 || words == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * With the fabric driving, this is the call that stops being a loop.  Here
     * every parcel costs a start-bit hunt and 32 software-clocked bits; there
     * the whole block is one command and one burst, and the host never enters
     * the per-parcel path at all.
     */
    if (dap_phy_fpga_in_use()) {
        const uint64_t payload = ((uint64_t)(count & 0xFFu) << 2) |
                                 ((uint64_t)(addr >> 2) << 10);
        return dap_phy_fpga_blockread(payload, 40, words, count);
    }
    if (!dap_phy_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint64_t payload = ((uint64_t)(count & 0xFFu) << 2) |
                             ((uint64_t)(addr >> 2) << 10);
    if (!dap_frame_build(&f, DAP_CMD_BLOCKREAD, 40, payload, 40)) {
        return ESP_FAIL;
    }

    dap_phy_idle_clocks(DAP_FRAME_LEAD_CLOCKS, 0);
    dap_phy_write_frame(&f);
    dap_phy_turnaround_to_read();

    esp_err_t err = ESP_OK;
    for (size_t w = 0; w < count; w++) {
        /* The timeout re-arms per parcel, so each one gets its own window. */
        if (dap_phy_await_start_bit(dap_probe_max_wait + DAP_WAIT_MARGIN) < 0) {
            ESP_LOGW(TAG, "blockread: no parcel %u of %u",
                     (unsigned)(w + 1), (unsigned)count);
            err = ESP_ERR_TIMEOUT;
            break;
        }
        dap_phy_read_bits(bits, 32);
        uint32_t v = 0;
        for (size_t i = 0; i < 32; i++) {
            v |= (uint32_t)(bits[i] & 1u) << i;
        }
        words[w] = v;
    }

    if (err == ESP_OK) {
        dap_phy_read_bits(bits, 6);      /* CRC6 on the final parcel only */
    }
    dap_phy_turnaround_to_write();
    if (err != ESP_OK) {
        dap_phy_idle_clocks(dap_probe_max_wait, 0);
    }
    return err;
}

