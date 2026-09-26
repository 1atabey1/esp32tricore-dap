/*
 * Physical layer for the Infineon DAP interface: bit-bang and GP-SPI backends.
 *
 * DAP is probe-clocked: the target advances only on DAP0 edges and MAXWAIT8
 * counts probe clocks, so pausing the clock (e.g. for an interrupt) is harmless.
 * The target latches DAP1 on the rising edge; upstream bits are valid t19
 * (<= 10 ns) after the falling edge and are sampled just before the next rise.
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

/* CPU cycles one bit-bang edge costs; subtracted from each half period. */
#define DAP_EDGE_OVERHEAD_TICKS 12

/* Wait cycles allowed at DAPISC reset (MAXWAIT8 = 31, MW8E = 0). */
#define DAP_MAXWAIT_RESET_CYCLES 248

/* Host-side wait limit; covers MW8E = 1 (3968 clocks) on the device. */
#define DAP_MAXWAIT_GENEROUS_CYCLES 4096

typedef struct {
    int      clk_pin;      /* DAP0 */
    int      dat_pin;      /* DAP1 */
    int      dir_pin;      /* FPGA output enable: 0 = probe drives, 1 = target drives */
    int      trst_pin;     /* -1 to leave TRST alone */
    uint32_t clock_hz;     /* target bit rate */
} dap_phy_cfg_t;

/* Configure the pins and park the bus idle with the probe driving. */
esp_err_t dap_phy_init(const dap_phy_cfg_t *cfg);

/* Change the bit rate without reconfiguring pins. */
void dap_phy_set_clock(uint32_t clock_hz);

/* True once dap_phy_init() has succeeded. */
bool dap_phy_ready(void);

/* Issue `count` clocks with DAP1 held at `level`, probe driving. */
void dap_phy_idle_clocks(size_t count, int level);

/* Clock a whole assembled frame out, probe driving throughout. */
void dap_phy_write_frame(const dap_frame_t *frame);

/* As above, preceded by `lead` low clocks with no gap (one SPI transfer). */
void dap_phy_write_frame_with_lead(const dap_frame_t *frame, size_t lead);

/* Hand the line to the target (dir = 1) or take it back (dir = 0). */
void dap_phy_turnaround_to_read(void);
void dap_phy_turnaround_to_write(void);

/* Clock `nbits` in with the target driving, one bit per byte; no start-bit hunt. */
void dap_phy_read_bits(uint8_t *bits, size_t nbits);

/*
 * Clock until DAP1 reads high (the start bit) or `max_cycles` pass.  Returns
 * the wait cycles consumed, -1 on timeout, or DAP_AWAIT_IDLE_HIGH.
 */
int dap_phy_await_start_bit(uint32_t max_cycles);

/* Reply window for dap_phy_read_reply(): reply bits plus slack, capped. */
#define DAP_REPLY_WINDOW_SLACK 8
#define DAP_REPLY_WINDOW_MAX   80

/*
 * Clock one whole reply (busy cycles, start bit, payload, CRC) and return the
 * busy cycles, or -1.  `bits` receives reply_bits + 6 bits, one per byte, or
 * nothing when reply_bits is zero.  On SPI this is one transfer: the clock must
 * not stop inside a reply.
 */
int dap_phy_read_reply(uint8_t *bits, size_t reply_bits, uint32_t max_wait);

/* dap_phy_await_start_bit(): the line never went low, so nothing drove it. */
#define DAP_AWAIT_IDLE_HIGH (-2)

/* Assert (low) or release TRST, if the board wired it. */
void dap_phy_set_trst(bool asserted);

/* Diagnostics. */

/*
 * Release the line and sample it `samples` times; returns how many read high.
 * A powered target idles DAP1 high.
 */
int dap_phy_sample_target_idle(int samples);

/* Drive DAP1 both levels and read the pad back; true when both match. */
bool dap_phy_self_drive_check(void);

/* Toggle each DAP pad from plain GPIO and read it back; true if under GPIO control. */
bool dap_phy_pad_toggle_check(void);

/* GP-SPI backend: same frames, hardware clocking.  Enable with dap_phy_use_spi(). */
esp_err_t dap_phy_spi_init(int clk_pin, int dat_pin, uint32_t clock_hz);
bool      dap_phy_spi_ready(void);
void      dap_phy_spi_set_clock(uint32_t clock_hz);
esp_err_t dap_phy_spi_write_bits(const uint8_t *bits, size_t nbits);
esp_err_t dap_phy_spi_read_bits(uint8_t *bits, size_t nbits);

/* Route the clock and data pads to SPI or back to GPIO (SPI ignores gpio_set_level). */
void dap_phy_spi_route(bool to_spi);

/* Write `pattern` and read it back on the same pad, probe driving. */
esp_err_t dap_phy_spi_loopback(uint8_t pattern);

/* Send `nbits` full-duplex, probe driving; `out` gets the pad, one bit per byte. */
esp_err_t dap_phy_spi_echo_bits(const uint8_t *bits, size_t nbits, uint8_t *out);

/* Check that the clock pad toggles during a long SPI transfer. */
esp_err_t dap_phy_spi_clock_pad_check(void);

/* Log which signal each DAP pad is currently driven by. */
void dap_phy_spi_log_routing(void);

/* Route the frame and reply paths through GP-SPI (true) or bit-bang (false). */
void dap_phy_use_spi(bool enable);
bool dap_phy_is_spi(void);

#ifdef __cplusplus
}
#endif
