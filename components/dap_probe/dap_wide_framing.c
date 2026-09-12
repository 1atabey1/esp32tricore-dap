/*
 * Wide-mode framing candidates and capture-tap calibration.
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

const char *dap_wide_framing_name(wide_framing_t v)
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
bool dap_wide_framing_sweep(wide_framing_t *rule, uint8_t *tap1_out,
                               uint8_t *tap2_out)
{
    bool found = false;

    ESP_LOGW(TAG, "  wide framing sweep (sync must answer 0x%08X):",
             WIDE_SYNC_EXPECT);

    for (unsigned v = 0; v < WF_COUNT; v++) {
        const frame_bits_t f = wide_frame((wide_framing_t)v, 0x10u, 63, 0, 0);
        char line[96];
        int  n = snprintf(line, sizeof(line), "    %-34s %2u bits:",
                          dap_wide_framing_name((wide_framing_t)v), f.n);

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
bool dap_wide_calibrate(uint8_t *tap1_out, uint8_t *tap2_out)
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
int dap_wide_stage;
int dap_wide_tap1, dap_wide_tap2;
int dap_wide_trail = -1;   /* -1 leaves TRAIL alone */

void dap_probe_fpga_wide_trail(int trail)
{
    dap_wide_trail = trail;
}

void dap_probe_fpga_wide_stage(int stage)
{
    dap_wide_stage = stage;
}

void dap_probe_fpga_wide_taps(int tap1, int tap2)
{
    dap_wide_tap1 = tap1 & 3;
    dap_wide_tap2 = tap2 & 3;
}

