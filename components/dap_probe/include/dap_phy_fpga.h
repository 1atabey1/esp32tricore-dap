/*
 * FPGA DAP master, driven from the ESP32 over SPI.
 *
 * The fabric performs whole exchanges (and block transfers) itself, so it
 * replaces exchange() rather than the bit-level PHY.  Requires the DAP-only
 * bitstream (fpga/dap_master); init verifies it is loaded.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Add the FPGA SPI device and confirm the DAP image is loaded. */
esp_err_t dap_phy_fpga_init(void);

/* True once init has succeeded and the DAP bitstream answered. */
bool dap_phy_fpga_ready(void);

/* Forget the link so the next init probes again; call after reconfiguring the FPGA. */
void dap_phy_fpga_invalidate(void);

/* Route exchanges through the fabric (true) or leave them to the CPU (false). */
void dap_phy_fpga_use(bool enable);
bool dap_phy_fpga_in_use(void);

/* DAP clock = 48 MHz / (2 * (div + 1)); 5 gives 4 MHz, 0 gives 24 MHz. */
esp_err_t dap_phy_fpga_set_div(uint8_t div);

/* Trailing clocks after each frame, and the start-bit wait limit, in DAP clocks. */
esp_err_t dap_phy_fpga_set_trail(uint8_t clocks);
esp_err_t dap_phy_fpga_set_maxwait(uint16_t clocks);

/* Assert (low) or release the target reset line. */
esp_err_t dap_phy_fpga_set_trst(bool asserted);

/*
 * Return the first `reply_bits` clocks of the reply window verbatim (first
 * clock in bit 0) instead of hunting for a start bit; no CRC check.
 */
esp_err_t dap_phy_fpga_set_raw_window(bool enable);

/*
 * Wide mode on the probe side: even frame bits on DAP1, odd on DAP2.  The
 * device is switched by a narrow-mode DAPISC with MODE = 01B, so enable this
 * only after that is acknowledged and disable it after a possible target reset.
 */
esp_err_t dap_phy_fpga_set_wide(bool enable);
bool      dap_phy_fpga_is_wide(void);

/* Receive wide, transmit narrow: DAP2 is sampled but never driven. */
esp_err_t dap_phy_fpga_set_rx_wide(bool enable);

/* Per-line sample tap in the bit window, 0..3 fabric clocks (0/0 = narrow default). */
esp_err_t dap_phy_fpga_set_skew(uint8_t dap1_tap, uint8_t dap2_tap);

/*
 * Fast mode: a bit every fabric clock, a 48 MHz DAP clock, DIV ignored.  The
 * frames go out raw, built here; one too long for that (over 44 data bits
 * narrow, 40 wide) is sent at DIV instead, so keep DIV fast too.
 */
esp_err_t dap_phy_fpga_set_fast(bool enable);
bool      dap_phy_fpga_is_fast(void);

/*
 * Dual-line reply FIFO reads (block-read drain over MOSI and MISO, the level
 * in each read's prefix).  Turned on at init when the input-timing
 * calibration finds a window; set_dual(true) re-runs it, (false) turns the
 * drain back to one line.
 */
bool dap_phy_fpga_dual_ok(void);
void dap_phy_fpga_set_dual(bool enable);

/*
 * Fast-mode receive timing.  LAG (0..3): a DAP0 clock's bit is sampled LAG +
 * DAP1 tap + 1 fabric clocks later; wrong, it shifts block-read parcels by a
 * bit.  edge1/edge2: sample that line on the falling edge, half a clock later.
 */
esp_err_t dap_phy_fpga_set_fast_timing(uint8_t lag, bool edge1, bool edge2);

/* Words per client_blockwrite (write FIFO depth), and per SPI fill burst. */
#define DAP_FPGA_BLOCK_WORDS  128
#define DAP_FPGA_BURST_WORDS  128

/* One client_blockwrite: `count` words streamed to `address` by the fabric.
 * ESP_ERR_NOT_SUPPORTED in wide mode: use word writes there. */
esp_err_t dap_phy_fpga_block_write(uint32_t address, const uint32_t *words,
                                   size_t count);

/*
 * One command and its reply.  `reply_bits` of zero means a bare acknowledge.
 * Returns ESP_OK only when the fabric reported a reply with a good CRC.
 */
esp_err_t dap_phy_fpga_exchange(uint8_t cmd, uint8_t len_field,
                                uint64_t data, size_t data_bits,
                                size_t reply_bits,
                                uint32_t *reply, uint16_t *wait_cycles);

/* Block read: one command frame, then `count` (1..256) parcels in one burst. */
esp_err_t dap_phy_fpga_blockread(uint64_t cmd_payload, size_t payload_bits,
                                 uint32_t *words, size_t count);

/* One client_blockread of a chain: the command payload and its word count. */
typedef struct {
    uint64_t payload;
    uint8_t  payload_bits;
    uint16_t count;          /* 1..256 */
} dap_fpga_block_t;

/*
 * Several block reads back to back: the fabric starts each one as the previous
 * ends while the host drains.  The words of all blocks land in `words` in
 * order.  Fails as a whole; nothing after a failed block is read.
 */
esp_err_t dap_phy_fpga_blockread_chain(const dap_fpga_block_t *blocks, size_t n,
                                       uint32_t *words);

/* Idle clocks before each chained block after the first (0..63). */
void dap_phy_fpga_set_chain_lead(uint8_t clocks);

/* Log the fabric's last status, for diagnosis. */
void dap_phy_fpga_log_status(void);

/* SPI link counters since the last reset. */
typedef struct {
    uint32_t xfers;          /* SPI transactions */
    uint32_t xfer_bytes;     /* bytes clocked, headers included */
    uint32_t xfer_us;        /* time inside those transactions */
    uint32_t polls;          /* STATUS/LEVEL polls during block reads */
} dap_fpga_stats_t;

void dap_phy_fpga_stats(dap_fpga_stats_t *out, bool reset);

/* Time a 7-byte and a 1 KB register read (ns each), and report the SPI clock. */
void dap_phy_fpga_link_timing(uint32_t *ns_short, uint32_t *ns_long, int *clock_khz);

/* WAIT: busy clocks before the last reply's start bit (a block's last parcel). */
uint16_t dap_phy_fpga_last_wait(void);

/*
 * Attach the target through the fabric (sync, LEN-48 DAPISC, error clear,
 * resync, client select, CLIENT_ID; retried) and route exchanges to it.
 * Returns ESP_OK only when the expected CLIENT_ID came back.
 */
esp_err_t dap_phy_fpga_attach(void);

#ifdef __cplusplus
}
#endif
