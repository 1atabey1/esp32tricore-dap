#include "dap_probe.h"
#include "dap_lock.h"
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

size_t dap_probe_trailer_bits;   /* clocks after a reply's CRC; zero by default */
static size_t s_raw_window;     /* non-zero: dump this many raw reply bits */

/* Reply start-bit wait in probe clocks; re-derived from each DAPISC value. */
uint32_t dap_probe_max_wait = DAP_MAXWAIT_RESET_CYCLES;

/* Widest reply this layer reads in one go: start bit is consumed separately. */
#define DAP_REPLY_MAX_BITS     64

void dap_probe_set_raw_window(size_t bits)
{
    s_raw_window = bits;
}

/* Bit-bang needs zero (it samples one clock ahead); SPI needs an explicit count. */
void dap_probe_set_trailer_bits(size_t n)
{
    dap_probe_trailer_bits = n;
}

static esp_err_t dap_probe_init_locked(uint32_t clock_hz)
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
    /* dap_phy_init() parks clock low, data high and TRST released. */
    ESP_LOGI(TAG, "DAP pins parked: TRST released (GPIO%d high), clock low",
             AEL_DAP_TRST_PIN);
    return ESP_OK;
#endif
}

/*
 * Clock a frame out, turn the line around, wait out the busy stuffing and read
 * `reply_bits` of payload plus six CRC bits (none for a bare acknowledge).
 * A bad CRC residue returns ESP_ERR_INVALID_CRC after a resync flush.
 */
static esp_err_t dap_probe_exchange_locked(const dap_frame_t *frame, size_t reply_bits,
                          dap_exchange_t *out)
{
    uint8_t bits[DAP_REPLY_MAX_BITS + 6];

    /* The FPGA fabric performs the whole exchange from the frame's fields. */
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

    /* Low clocks before every frame, so the device sees a clean start bit. */
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
        /* Flush the device back to Active::RECEIVE; see DAP_RESYNC_CLOCKS. */
        dap_phy_idle_clocks(dap_probe_max_wait, 0);
        return ESP_ERR_TIMEOUT;
    }

    /*
     * A data reply is [start][RDATA][CRC6]; an acknowledge is the start bit
     * alone.  Clocking phantom CRC bits after an ack discards the next command.
     */
    const size_t to_read = reply_bits ? reply_bits + 6 : 0;
    if (to_read) {
        /* All ones, CRC included, is an undriven wire, not a reply. */
        size_t ones = 0;
        for (size_t i = 0; i < to_read; i++) {
            ones += bits[i] ? 1u : 0u;
        }
        out->idle_high = (ones == to_read);
    }
    /* Optional trailer clocks, still with the target driving. */
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

    /* A failed residue means no reply (e.g. a glitch taken for a start bit). */
    if (!out->crc_ok) {
        dap_phy_idle_clocks(dap_probe_max_wait, 0);      /* flush to Active::RECEIVE */
        return ESP_ERR_INVALID_CRC;
    }

    /* An ack has no CRC; one arriving later than DAP_ACK_MAX_WAIT is a glitch. */
    if (reply_bits == 0 && out->wait_cycles > DAP_ACK_MAX_WAIT) {
        out->timed_out = true;
        dap_phy_idle_clocks(dap_probe_max_wait, 0);
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t dap_probe_attach(dap_exchange_t *out, int attempts)
{
    /* Retry sync after a MAXWAIT8 flush back to Active::RECEIVE. */
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
    /* LEN all ones parks a JTAG TAP that may share the pins during hot plug. */
    return dap_probe_exchange(&f, 32, out);
}

esp_err_t dap_probe_dapisc(uint16_t value, bool cold, dap_exchange_t *out)
{
    dap_frame_t f;

    /*
     * Cold: LEN 48 with the signature, for after PORST or Enabled-to-Active.
     * Otherwise LEN 16 for an Active device.  The reply is the updated DAPISC,
     * 3 DAP0 cycles after the last CRC bit; a rejected command gets no reply.
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
 * T_timeout = MAXWAIT8 * 8 * (1 + MW8E * 15) DAP0 clocks.  MAXWAIT8 = 0
 * disables the device timeout, so a fixed cap is used instead.
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
     * Writes carry no size exponent: LEN = 4 + n, payload = [instruction][data]
     * LSB first.  Reply is a bare start bit (no CRC6 unless DAPISC.RC6).
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
    return dap_probe_exchange(&f, 0, out);
}

static bool s_fpi_prio;

void dap_probe_set_bus_priority(bool high)
{
    s_fpi_prio = high;
}

static esp_err_t dap_probe_set_rw_mode_locked(bool supervisor)
{
    dap_exchange_t x;
    const uint16_t conf = DAP_IOCONF_MODE_RW |
                          (uint16_t)(supervisor ? DAP_IOCONF_SVM : 0u) |
                          (uint16_t)(s_fpi_prio ? DAP_IOCONF_FPI_PRIO : 0u);

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

static esp_err_t dap_probe_clear_error_state_locked(void)
{
    dap_exchange_t x;

    /* IO_SUPERVISOR (0xB, which also reads IOINFO) clears Error State. */
    const esp_err_t err = dap_probe_client_read(DAP_IO_INFO, 4, 16, &x);
    if (err == ESP_OK && (x.reply & (DAP_IOINFO_BUS_RST | DAP_IOINFO_IF_LCK |
                                     DAP_IOINFO_BUS_RD_ERR | DAP_IOINFO_BUS_WR_ERR))) {
        dap_probe_log_ioinfo((uint16_t)x.reply);
    }
    return err;
}

static esp_err_t dap_probe_read32_locked(uint32_t addr, uint32_t *value)
{
    dap_exchange_t x;

    /* IO_SET_ADDRESS loads IOADDR (32 bits), then IO_READ_WORD fetches the word. */
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

/* One-word block read: one frame instead of two exchanges.  For polling
 * known-good addresses (FIFONOW); bus errors are not distinguished. */
esp_err_t dap_probe_read32_fast(uint32_t addr, uint32_t *value)
{
    if (!dap_phy_fpga_in_use()) {
        return dap_probe_read32(addr, value);
    }
    return dap_probe_blockread(addr, value, 1);
}

static esp_err_t dap_probe_write32_locked(uint32_t addr, uint32_t value)
{
    dap_exchange_t x;
    esp_err_t err = dap_probe_client_write(DAP_IO_SET_ADDRESS, 5, addr, 32, &x);

    if (err != ESP_OK) {
        return err;
    }
    return dap_probe_client_write(DAP_IO_WRITE_WORD, 5, value, 32, &x);
}

static esp_err_t dap_probe_enable_ocds_locked(void)
{
    /*
     * With OSTATE.OEN clear, 0xFB718000..0xFB71FFFF bus-errors.  The four
     * OEC.PAT writes must be contiguous or the matcher resets.
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

    /* The miniMCDS is still clock-gated; CLC = 0 enables it. */
    if (dap_probe_write32(DAP_ADDR_MCDS_CLC, 0x00000000u) != ESP_OK) {
        ESP_LOGW(TAG, "miniMCDS CLC write failed");
    }

    /* CT.SETE unlocks writes to the rest of the miniMCDS space. */
    return dap_probe_write32(DAP_ADDR_MCDS_CT, 0x8000u);
}

static esp_err_t dap_probe_blockread_locked(uint32_t addr, uint32_t *words, size_t count)
{
    /*
     * client_blockread (0x0A) payload:
     *   bit 0      request the 32-bit block CRC (CRCup) as a final parcel
     *   bit 1      CRC6 after every parcel, rather than only the last
     *   bits 9:2   word count, 1..255, with 0 meaning 256 words (1 kB)
     *   bits 39:10 word-aligned address, shifted right by two
     * LEN 10/24/40 = no/14-bit/30-bit address.  With an address the device
     * loads IOADDR itself and post-increments by four per word.
     */
    dap_frame_t f;
    uint8_t     bits[32];

    if (count == 0 || count > 256 || words == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* The fabric does the whole block as one command and burst. */
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

/* -- locked entry points (dap_lock.h) -------------------------------------- */

esp_err_t dap_probe_exchange(const dap_frame_t *frame, size_t reply_bits, dap_exchange_t *out)
{
    dap_lock();
    const esp_err_t err = dap_probe_exchange_locked(frame, reply_bits, out);
    dap_unlock();
    return err;
}

esp_err_t dap_probe_set_rw_mode(bool supervisor)
{
    dap_lock();
    const esp_err_t err = dap_probe_set_rw_mode_locked(supervisor);
    dap_unlock();
    return err;
}

esp_err_t dap_probe_clear_error_state(void)
{
    dap_lock();
    const esp_err_t err = dap_probe_clear_error_state_locked();
    dap_unlock();
    return err;
}

esp_err_t dap_probe_read32(uint32_t addr, uint32_t *value)
{
    dap_lock();
    const esp_err_t err = dap_probe_read32_locked(addr, value);
    dap_unlock();
    return err;
}

esp_err_t dap_probe_write32(uint32_t addr, uint32_t value)
{
    dap_lock();
    const esp_err_t err = dap_probe_write32_locked(addr, value);
    dap_unlock();
    return err;
}

esp_err_t dap_probe_enable_ocds(void)
{
    dap_lock();
    const esp_err_t err = dap_probe_enable_ocds_locked();
    dap_unlock();
    return err;
}

esp_err_t dap_probe_blockread(uint32_t addr, uint32_t *words, size_t count)
{
    dap_lock();
    const esp_err_t err = dap_probe_blockread_locked(addr, words, count);
    dap_unlock();
    return err;
}

/* Blocks per fabric chain; the chain descriptors live on the stack. */
#define BLOCKREAD_CHAIN_MAX 16

esp_err_t dap_probe_blockread_many(const dap_block_req_t *reqs, size_t n, uint32_t *words)
{
    if (reqs == NULL || words == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ESP_OK;

    dap_lock();
    while (n > 0 && err == ESP_OK) {
        const size_t batch = (n > BLOCKREAD_CHAIN_MAX) ? BLOCKREAD_CHAIN_MAX : n;
        size_t words_in_batch = 0;

        if (dap_phy_fpga_in_use()) {
            dap_fpga_block_t chain[BLOCKREAD_CHAIN_MAX];
            for (size_t i = 0; i < batch; i++) {
                if (reqs[i].count == 0 || reqs[i].count > 256) {
                    err = ESP_ERR_INVALID_ARG;
                    break;
                }
                /* Payload layout as in dap_probe_blockread_locked. */
                chain[i].payload = ((uint64_t)(reqs[i].count & 0xFFu) << 2) |
                                   ((uint64_t)(reqs[i].addr >> 2) << 10);
                chain[i].payload_bits = 40;
                chain[i].count = reqs[i].count;
                words_in_batch += reqs[i].count;
            }
            if (err == ESP_OK) {
                err = dap_phy_fpga_blockread_chain(chain, batch, words);
            }
        } else {
            for (size_t i = 0; i < batch && err == ESP_OK; i++) {
                err = dap_probe_blockread_locked(reqs[i].addr, words + words_in_batch,
                                                 reqs[i].count);
                words_in_batch += reqs[i].count;
            }
        }
        reqs  += batch;
        words += words_in_batch;
        n     -= batch;
    }
    dap_unlock();
    return err;
}

esp_err_t dap_probe_init(uint32_t clock_hz)
{
    dap_lock();
    const esp_err_t err = dap_probe_init_locked(clock_hz);
    dap_unlock();
    return err;
}
