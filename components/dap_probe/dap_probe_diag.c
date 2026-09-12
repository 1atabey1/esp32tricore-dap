/*
 * Sweeps and probes for a link that is not answering.
 */

#include <inttypes.h>
#include <string.h>

#include "board_profile.h"
#include "dap_frame.h"
#include "dap_phy.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "dap_probe_priv.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DAP";

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
    text[n] = '\0';
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
                    if (dap_probe_exchange(&f, 32, &x) == ESP_OK) {
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

void dap_probe_log_exchange(const char *step, const dap_exchange_t *x)
{
    if (x->timed_out) {
        ESP_LOGE(TAG, "%-22s sent 0x%05" PRIX64 " (%zu bits) -> %s within %d clocks",
                 step, x->sent_word, x->sent_bits,
                 x->idle_high ? "line stayed idle high, nothing driving it"
                              : "line held low, no start bit",
                 dap_probe_max_wait);
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
        dap_probe_exchange(&f, 32, &out[0]);
    }
    if (dap_frame_build(&f, DAP_CMD_DAPISC, 48, dapisc, 48)) {
        dap_probe_exchange(&f, 0, &out[1]);
    }
    if (dap_frame_build(&f, DAP_CMD_CLIENT_SET, 3, 1, 3)) {
        dap_probe_exchange(&f, 0, &out[2]);
    }
    if (dap_frame_build(&f, DAP_CMD_CLIENT_READ, 7,
                        dap_client_read_payload(DAP_IO_CLIENT_ID, 4), 7)) {
        dap_probe_exchange(&f, 16, &out[3]);
    }
    return ESP_OK;
}

esp_err_t dap_probe_block_throughput(void)
{
    /*
     * The number this project is measured against.
     *
     * A miniWiggler through DAS/TAS reads target memory at about 38 kB/s, and
     * that ceiling is what forced a 12x cut in the trace publish rate.  This
     * reads 1 kB per telegram - 256 words, the maximum - so the frame cost is
     * amortised and what is left is wire time plus per-parcel overhead.
     */
    /*
     * Up to 16 MHz now that the pins are driven through the GPIO registers.
     * The old ceiling was the driver call overhead rather than the wire or the
     * device, so the rates above 4 MHz are new ground and the point where the
     * target stops agreeing is a number this project has never had.
     */
    static const uint32_t rates[] = { 400000u, 1000000u, 2000000u, 4000000u,
                                      6000000u, 8000000u, 12000000u, 16000000u };
    static uint32_t buf[256];
    const int iterations = 8;

    ESP_LOGW(TAG, "--- block read throughput, 1 kB per telegram ---");
    for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
        dap_phy_set_clock(rates[r]);

        int ok = 0;
        const int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < iterations; i++) {
            if (dap_probe_blockread(0x70000000u, buf, 256) == ESP_OK) {
                ok++;
            } else {
                dap_probe_clear_error_state();
            }
        }
        const int64_t us = esp_timer_get_time() - t0;
        if (us > 0 && ok) {
            const int kbps = (int)((int64_t)ok * 1024 * 1000000 / us / 1024);
            ESP_LOGW(TAG, "  %7" PRIu32 " Hz: %d/%d blocks, %lld us -> %d kB/s",
                     rates[r], ok, iterations, (long long)us, kbps);
        } else {
            ESP_LOGW(TAG, "  %7" PRIu32 " Hz: no blocks completed", rates[r]);
        }
    }
    dap_phy_set_clock(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    return ESP_OK;
}

esp_err_t dap_probe_rate_test(void)
{
    /*
     * How fast does a single-word register read actually go, and how far does
     * the bit-banged PHY carry before the wire stops agreeing?
     *
     * CLIENT_ID is the ideal probe for this: hard-wired to 0x0260, so every
     * reply is self-checking.  A rate where the value still reads 0x0260 with
     * a valid CRC is a rate that works; one where it does not is where the
     * PHY's edge placement or the target's setup and hold give out.  This
     * plan's Phase 1 numbers - "150-250 kB/s at ~4 MHz" - were an estimate and
     * have never been measured.
     */
    /*
     * Up to 16 MHz now that the pins are driven through the GPIO registers.
     * The old ceiling was the driver call overhead rather than the wire or the
     * device, so the rates above 4 MHz are new ground and the point where the
     * target stops agreeing is a number this project has never had.
     */
    static const uint32_t rates[] = { 400000u, 1000000u, 2000000u, 4000000u,
                                      6000000u, 8000000u, 12000000u, 16000000u };
    const int reads = 200;

    ESP_LOGW(TAG, "--- rate test: %d CLIENT_ID reads per rate ---", reads);

    for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
        dap_exchange_t x;
        int      good = 0;
        uint32_t crc_ok = 0;

        dap_phy_set_clock(rates[r]);

        const int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < reads; i++) {
            if (dap_probe_client_read(DAP_IO_CLIENT_ID, 4, 16, &x) == ESP_OK) {
                if (x.reply == DAP_CLIENT_ID_EXPECT) {
                    good++;
                }
                if (x.crc_ok) {
                    crc_ok++;
                }
            }
        }
        const int64_t us = esp_timer_get_time() - t0;

        /* A 16-bit read is a 26-bit frame plus a 23-bit reply, so call it
         * 49 bits of wire time plus the lead-in and the wait. */
        const int per_read_us = (int)(us / reads);
        ESP_LOGW(TAG, "  %7" PRIu32 " Hz: %3d/%d correct, %" PRIu32 " CRC ok, "
                      "%d us/read -> %d reads/s",
                 rates[r], good, reads, crc_ok, per_read_us,
                 per_read_us ? (int)(1000000 / per_read_us) : 0);
    }

    dap_phy_set_clock(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    return ESP_OK;
}

esp_err_t dap_probe_trailer_sweep(void)
{
    static const size_t trailers[] = { 0, 1, 2, 4, 8, 18 };

    ESP_LOGW(TAG, "--- trailer sweep: 8 syncs per trailer length ---");
    for (size_t t = 0; t < sizeof(trailers) / sizeof(trailers[0]); t++) {
        char   line[16];
        size_t n = 0;

        dap_probe_trailer_bits = trailers[t];
        for (int i = 0; i < 8; i++) {
            dap_frame_t    f;
            dap_exchange_t x;

            dap_phy_idle_clocks(11, 0);
            if (!dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 0)) {
                break;
            }
            dap_probe_exchange(&f, 32, &x);
            line[n++] = (x.reply == DAP_SYNC_EXPECT) ? 'o' : '.';
        }
        line[n] = '\0';
        ESP_LOGW(TAG, "  trailer %2u bits: %s", (unsigned)trailers[t], line);
    }
    dap_probe_trailer_bits = 0;      /* restore the value that works, not the one tested */
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
        dap_probe_exchange(&f, 32, &x);
        line[n++] = (x.reply == DAP_SYNC_EXPECT) ? 'o' : '.';

        /* Every fourth attempt, try a client_reset first and mark it. */
        if ((i % 4) == 3) {
            if (dap_frame_build(&f, DAP_CMD_CLIENT_RESET, 0, 0, 0)) {
                dap_exchange_t r;
                dap_probe_exchange(&f, 0, &r);
            }
            line[n++] = '|';
        }
    }
    line[n] = '\0';
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
        dap_probe_exchange(&f, 32, &s1);

        if (!dap_frame_build(&f, rows[i].cmd, rows[i].len, rows[i].data, rows[i].db)) {
            continue;
        }
        dap_probe_exchange(&f, rows[i].reply, &s2);

        ESP_LOGW(TAG, "  sync %s -> %-15s %s reply 0x%08" PRIX64,
                 s1.reply == DAP_SYNC_EXPECT ? "ok  " : "FAIL",
                 rows[i].name,
                 s2.idle_high ? "idle-high" : (s2.timed_out ? "held-low " : "ANSWERED "),
                 s2.reply);
    }
    return ESP_OK;
}

/*
 * Phase 1c: bring the GP-SPI backend up and prove it against the bit-banged
 * one on the same wire.
 *
 * The bit-bang path is correct and slow for a measured reason: 33.5 us per
 * 33-bit word is about 1 MHz effective against a 4 MHz setting, so nearly all
 * of that time is gpio_set_level() and cycle-counter spinning rather than wire
 * time.  The frame layer is untouched by this; only the clocking changes.
 *
 * Correctness comes before speed, in an order where each failure names its own
 * cause:
 *
 *   sync           - edge placement and bit order, against a known constant
 *   CLIENT_ID      - a reply read back through the peripheral input path
 *   STM0_TIM0 x2   - a live bus read, which a stuck wire cannot fake
 *   blockread      - multi-parcel framing, cross-checked word for word
 *
 * If any of those fail the backend is switched off and the bit-bang path is
 * re-verified, so a bad SPI experiment never leaves the probe broken.
 */
/*
 * Send the real attach frames and print the raw reply window for each.
 *
 * Uses the ordinary transaction path with decoding switched off, so the frames,
 * the lead clocks and the turnaround are exactly what the working bit-bang
 * attach sends.  Run on both backends it shows whether the device answers
 * client_set and client_read at all, and where in the window the answer sits.
 */
void dap_probe_dump_raw_attach(const char *what)
{
    dap_exchange_t x;

    ESP_LOGW(TAG, "--- raw windows, %s ---", what);
    dap_phy_idle_clocks(dap_probe_max_wait, 0);

    dap_probe_set_raw_window(48);
    dap_probe_sync(&x);
    dap_probe_client_set(1, &x);
    dap_probe_client_read(DAP_IO_CLIENT_ID, 4, 16, &x);
    dap_probe_set_raw_window(0);
}
