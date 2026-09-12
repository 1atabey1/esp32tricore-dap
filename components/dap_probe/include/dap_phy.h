/*
 * Bit-banged physical layer for the Infineon DAP interface.
 *
 * DAP is entirely probe-clocked and synchronous: the target advances only on
 * DAP0 edges, and the MAXWAIT8 timeout counts *probe-supplied clocks* rather
 * than device time.  Stopping the clock therefore costs nothing - no critical
 * sections, no pinned core, no interrupt juggling.  An interrupt landing in the
 * middle of a frame simply pauses the target's view of the world.
 *
 * Downstream bits are latched by the target on the rising edge of DAP0.
 * Upstream bits become valid a delay after the falling edge (t19, at most 10 ns
 * at 40 MHz), so the probe samples them shortly before the next rising edge.
 *
 * This layer is deliberately slow and obvious.  It exists to prove the framing
 * where it can be single-stepped; the GP-SPI path that follows reuses the frame
 * layer and replaces only this file's clocking.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dap_frame.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * At the DAPISC reset value MAXWAIT8 = 31 with MW8E = 0, so the target may
 * stuff up to 248 wait cycles before its reply's start bit.  Leave DAPISC
 * alone during bring-up: that generosity is what makes a cold attach survive
 * the target's untrimmed backup oscillator.
 */
/*
 * Roughly what one clock edge costs in CPU cycles once the pins are driven
 * through the GPIO registers: the store, the loop overhead and the cycle-count
 * read.  Subtracted from each half period so a requested rate comes out close
 * to the request instead of well under it.
 */
#define DAP_EDGE_OVERHEAD_TICKS 12

#define DAP_MAXWAIT_RESET_CYCLES 248

/*
 * A generous window for bring-up.  MW8E=1 raises the device's own limit to
 * 3968 clocks, and a frame whose reply is still stuffing at 248 looks
 * identical to a frame that was ignored - the line simply stays low.  Waiting
 * longer costs microseconds and removes the ambiguity.
 */
#define DAP_MAXWAIT_GENEROUS_CYCLES 4096

typedef struct {
    int      clk_pin;      /* DAP0 */
    int      dat_pin;      /* DAP1 */
    int      dir_pin;      /* FPGA output enable: 0 = probe drives, 1 = target drives */
    int      trst_pin;     /* -1 to leave TRST alone */
    uint32_t clock_hz;     /* target bit rate; 1-2 MHz for bring-up */
} dap_phy_cfg_t;

/* Configure the pins and park the bus idle with the probe driving. */
esp_err_t dap_phy_init(const dap_phy_cfg_t *cfg);

/* Change the bit rate without reconfiguring pins. */
void dap_phy_set_clock(uint32_t clock_hz);

/* True once dap_phy_init() has succeeded. */
bool dap_phy_ready(void);

/*
 * Issue `count` clock pulses with DAP1 held at `level` while the probe drives.
 * The hot-attach sequence starts with eight of these before the first frame.
 */
void dap_phy_idle_clocks(size_t count, int level);

/* Clock a whole assembled frame out, probe driving throughout. */
void dap_phy_write_frame(const dap_frame_t *frame);

/*
 * The same, preceded by `lead` clocks with the line held low, as one unit.  On
 * the SPI backend that is a single transfer, so no gap opens between the
 * lead-in and the frame.
 */
void dap_phy_write_frame_with_lead(const dap_frame_t *frame, size_t lead);

/* Hand the line to the target (dir = 1) or take it back (dir = 0). */
void dap_phy_turnaround_to_read(void);
void dap_phy_turnaround_to_write(void);

/*
 * Clock `nbits` in with the target driving, storing one bit per byte in
 * transmission order.  Does not look for a start bit: callers that need one use
 * dap_phy_await_start_bit() first, so a failed attach can still dump what the
 * wire actually did.
 */
void dap_phy_read_bits(uint8_t *bits, size_t nbits);

/*
 * Clock until DAP1 reads high - the reply's start bit - or until `max_cycles`
 * have gone by.  Busy stuffing is DAP1 held low, so this is "skip zeros, find
 * the first one".  Returns the number of wait cycles consumed, or -1 on
 * timeout.  The MAXWAIT8 counter is deactivated once the start bit arrives, so
 * everything after this call is unconstrained in time.
 */
int dap_phy_await_start_bit(uint32_t max_cycles);

/*
 * Window for dap_phy_read_reply(): the reply itself plus a few clocks of
 * slack, which cover the busy cycles before the start bit and run the telegram
 * out after the CRC.  Waits measured on this bench are 0 to 2 cycles.
 */
#define DAP_REPLY_WINDOW_SLACK 8
#define DAP_REPLY_WINDOW_MAX   80

/*
 * Clock one whole reply - busy cycles, start bit, payload and CRC - and return
 * the number of busy cycles, or -1.  `bits` receives reply_bits + 6 bits, one
 * per byte, or nothing when reply_bits is zero (a bare acknowledge).
 *
 * On the SPI backend this is a single transfer, so the clock runs continuously
 * for the whole reply.  That matters: splitting a reply across transfers stops
 * and restarts the clock inside it, and replies read back wrong when it does.
 */
int dap_phy_read_reply(uint8_t *bits, size_t reply_bits, uint32_t max_wait);

/*
 * How many busy cycles the SPI reply window allows for before the start bit.
 *
 * The window has to be exactly the right length - over-clocking after a reply
 * leaves the next command ignored - and its length depends on how many busy
 * cycles the device inserts, which differs per command.  So it is learned: a
 * window that finds the start bit somewhere else adopts that position and
 * reports a miss, and the caller's retry is clocked correctly.
 */
void   dap_phy_set_expected_wait(size_t cycles);
size_t dap_phy_get_expected_wait(void);

/*
 * dap_phy_await_start_bit() returns this instead of -1 when the line never
 * went low during the whole window.  That is not a timeout: it means nothing
 * was driving the wire, so there was no reply to wait for.
 */
#define DAP_AWAIT_IDLE_HIGH (-2)

/* Assert (low) or release TRST, if the board wired it. */
void dap_phy_set_trst(bool asserted);

/*
 * Diagnostics, for the case where the target says nothing at all.  A silent
 * target has several possible causes and these separate them before any
 * protocol theory gets involved.
 */

/*
 * Hand the line to the target and sample it `samples` times, returning how
 * many reads came back high.  A powered target idles DAP1 high, so all-low is
 * a dead wire, an unpowered target, or an inverted direction pin; all-high
 * says the wire and the FPGA's inbound path are intact.
 */
int dap_phy_sample_target_idle(int samples);

/*
 * Drive the line both ways with the probe in control and read the pad back
 * through the S3's own input path.  This proves the S3 side and the FPGA's
 * outbound enable independently of whether anything is attached.  Returns true
 * when both levels read back correctly.
 */
bool dap_phy_self_drive_check(void);

/*
 * Clock a recognisable pattern out for a scope or the on-board logic analyser:
 * `reps` repetitions of 0xA on DAP1 with DAP0 clocking normally.
 */
void dap_phy_training_pattern(int reps);

/*
 * Force the direction pin to a chosen level with the S3 driving the data line,
 * for polarity testing.  dap_phy_self_drive_check() cannot settle this: in
 * INPUT_OUTPUT mode the S3 reads back its own pad and the FPGA's buffer is
 * never in the loop, so it passes either way.  Watching the connector with the
 * logic analyser while this drives is what actually decides it.
 */
void dap_phy_force_dir(int level);

/*
 * Drive each DAP pad from plain GPIO and read its own level back.  Answers the
 * one question the self-drive check cannot: whether the pads are still under
 * GPIO control at all, which is what separates "the pads are gone" from "the
 * target is desynchronised" after the SPI backend has used them.
 */
bool dap_phy_pad_toggle_check(void);

/*
 * GP-SPI backend.  Same wire, same frames, hardware clocking.
 *
 * dap_phy_use_spi() switches the frame and reply paths over once the backend is
 * initialised; the bit-bang path stays available and is what every result so
 * far was measured on, so the two can be compared directly.
 */
esp_err_t dap_phy_spi_init(int clk_pin, int dat_pin, uint32_t clock_hz);
bool      dap_phy_spi_ready(void);
void      dap_phy_spi_set_clock(uint32_t clock_hz);
esp_err_t dap_phy_spi_write_bits(const uint8_t *bits, size_t nbits);
esp_err_t dap_phy_spi_read_bits(uint8_t *bits, size_t nbits);

/*
 * Point the clock and data pads at the SPI peripheral or back at plain GPIO.
 * Called by dap_phy_use_spi(); exposed because the two backends share pins and
 * a pad left pointing at SPI ignores gpio_set_level() entirely.
 */
void dap_phy_spi_route(bool to_spi);

/*
 * Write a pattern and read it straight back on the same pad, with the probe
 * driving.  Proves the clock is generated and both directions of the pad
 * routing work, independently of anything attached.
 */
esp_err_t dap_phy_spi_loopback(uint8_t pattern);

/*
 * Send `nbits` full-duplex with the probe driving and return what the pad
 * carried, one bit per byte.  Checks the serialisation of a length that is not
 * a multiple of eight, which a byte-sized loopback cannot.
 */
esp_err_t dap_phy_spi_echo_bits(const uint8_t *bits, size_t nbits, uint8_t *out);

/*
 * Watch the clock pad through the GPIO input while a long SPI transfer runs.
 * The clock is the only DAP signal with no other readback path, and a target
 * with no clock looks exactly like a dead target.
 */
esp_err_t dap_phy_spi_clock_pad_check(void);

/* Log which signal each DAP pad is currently driven by. */
void dap_phy_spi_log_routing(void);

/*
 * Point the clock and data pads at the SPI peripheral or back at plain GPIO.
 * dap_phy_use_spi() calls this; it is declared here because the two backends
 * share pins and a pad left pointing at SPI ignores gpio_set_level() entirely.
 */
void dap_phy_spi_route(bool to_spi);

/* Route the frame and reply paths through GP-SPI (true) or bit-bang (false). */
void dap_phy_use_spi(bool enable);
bool dap_phy_is_spi(void);

/* Drive the data line to a level, with the probe in control. */
void dap_phy_drive_data(int level);

#ifdef __cplusplus
}
#endif
