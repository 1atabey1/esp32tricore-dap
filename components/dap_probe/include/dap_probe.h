/*
 * DAP transactions and the bring-up checkpoint sequence.  Expected answers:
 *   1. eight slow clocks, then `sync`  -> 0xAAAAAAAA
 *   2. `dapisc`                        -> the register echoed back
 *   3. `client_set(1)`                 -> start-bit acknowledge
 *   4. `client_read(IO_CLIENT_ID)`     -> 0x0260
 *   5. `client_write` address, then a 32-bit `client_read` -> target memory
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Raw result of one command-plus-reply exchange. */
typedef struct {
    uint64_t sent_word;      /* the frame as clocked out, first bit in bit 0 */
    size_t   sent_bits;
    int      wait_cycles;    /* clocks of busy stuffing, or -1 if no start bit */
    uint64_t reply;          /* reply payload, LSB first */
    size_t   reply_bits;
    uint8_t  reply_crc;      /* the six CRC bits that followed, if read */
    bool     crc_ok;         /* residue check over payload+CRC */
    bool     timed_out;
    bool     idle_high;   /* line never went low: nothing was driving it */
} dap_exchange_t;

/*
 * Configure the DAP pins and park them safely for an attached target, sending
 * nothing.  TRST drives the target's reset and must not be left low, so call
 * this after the background tasks start, whether or not DAP bring-up is enabled.
 */
esp_err_t dap_probe_park_idle(void);

/* Bring the PHY up on this board's pins at `clock_hz` (0 for the 1 MHz default). */
esp_err_t dap_probe_init(uint32_t clock_hz);

/* Hot-attach opener: eight idle-high clocks, `sync`, reply window.  `out` is filled even on failure. */
esp_err_t dap_probe_sync(dap_exchange_t *out);

/* Sync with retries and a flush between them; the first attach often fails. */
esp_err_t dap_probe_attach(dap_exchange_t *out, int attempts);

/*
 * Write DAPISC and read back the updated value.  `cold` sends the 66-bit
 * initialisation telegram (signature 0x4ABBAF53) for Enabled-to-Active;
 * otherwise the short LEN-16 form an Active device accepts.
 */
esp_err_t dap_probe_dapisc(uint16_t value, bool cold, dap_exchange_t *out);

/* Adopt the reply-wait window implied by a DAPISC value. */
void dap_probe_note_dapisc(uint16_t dapisc);

/* Select an IOClient (1 = Cerberus). */
esp_err_t dap_probe_client_set(uint8_t client, dap_exchange_t *out);

/* Issue a client_read of one IO instruction at the given size exponent. */
esp_err_t dap_probe_client_read(uint8_t io_instruction, uint8_t size_exponent,
                                size_t reply_bits, dap_exchange_t *out);

/* The four bring-up frames back to back, with no host work between them. */
esp_err_t dap_probe_attach_now(dap_exchange_t out[6]);

/* Write an IOClient register or operand: instruction, size exponent, data. */
esp_err_t dap_probe_client_write(uint8_t io_instruction, uint8_t size_exponent,
                                 uint64_t data, size_t data_bits,
                                 dap_exchange_t *out);

/* Put the IOClient in read/write mode (bus access, not COMDATA); required before memory access. */
esp_err_t dap_probe_set_rw_mode(bool supervisor);

/* Dump raw reply bits instead of decoding, for diagnosis.  0 turns it off. */
void   dap_probe_set_raw_window(size_t bits);

/* Clocks to issue after a reply with the target still driving (0 for bit-bang). */
void   dap_probe_set_trailer_bits(size_t n);

/* Log an IOINFO value with its bits named. */
void dap_probe_log_ioinfo(uint16_t v);

/* Clear Cerberus Error State, in which reads and writes are silently dropped. */
esp_err_t dap_probe_clear_error_state(void);

/* Set IOADDR and read the 32-bit word there. */
esp_err_t dap_probe_read32(uint32_t addr, uint32_t *value);

/* Set IOADDR and write a 32-bit word there. */
esp_err_t dap_probe_write32(uint32_t addr, uint32_t value);

/*
 * Enable OCDS: OEC.PAT pattern writes, check OSTATE.OEN, then OCNTRL and
 * CT.SETE.  Without it the miniMCDS register space bus-errors.
 */
esp_err_t dap_probe_enable_ocds(void);

/* Read `count` words (1..256) in one client_blockread; the device post-increments IOADDR. */
esp_err_t dap_probe_blockread(uint32_t addr, uint32_t *words, size_t count);

/* Block read throughput in kB/s, against the 38 kB/s baseline. */
esp_err_t dap_probe_block_throughput(void);

/* Single-word read rate, and how far the bit-banged PHY carries. */
esp_err_t dap_probe_rate_test(void);

/* Run the checkpoint sequence, logging each step; ESP_OK only if all matched. */
esp_err_t dap_probe_bringup_report(void);

/* Switch clocking to GP-SPI, verify and measure it; reverts to bit-bang on failure. */
esp_err_t dap_probe_spi_bringup(void);

/*
 * Attach through the fabric DAP master, check CLIENT_ID and miniMCDS ID, and
 * measure block-read throughput.  `wide` adds wide mode, then returns to narrow.
 */
esp_err_t dap_probe_fpga_route_check(bool wide);

/*
 * Sweep bit rate, idle clocks before the first frame, TRST pulse and sync LEN,
 * reporting every combination that draws a start bit.  ESP_OK if any answered.
 */
esp_err_t dap_probe_attach_sweep(void);

/* Send `sync` and log a raw reply window of `window_bits`. */
esp_err_t dap_probe_dump_sync_reply(size_t window_bits);

/* Try each output-capable Port C pin as the clock and report which draws a reply. */
esp_err_t dap_probe_clock_pin_search(void);

#ifdef __cplusplus
}
#endif
