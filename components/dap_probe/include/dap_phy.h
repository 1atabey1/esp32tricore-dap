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
#define DAP_MAXWAIT_RESET_CYCLES 248

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

#ifdef __cplusplus
}
#endif
