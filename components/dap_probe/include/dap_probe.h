/*
 * DAP transactions and the bring-up checkpoint sequence.
 *
 * Every step here has a documented expected answer, which is the point: a
 * failure names itself instead of needing a bisect.  In order,
 *
 *   1. eight slow clocks, then `sync`  -> 0xAAAAAAAA
 *   2. `dapisc`                        -> the register echoed back
 *   3. `client_set(1)`                 -> start-bit acknowledge
 *   4. `client_read(IO_CLIENT_ID)`     -> 0x0260
 *   5. `client_write` address, then a 32-bit `client_read` -> target memory
 *
 * Checkpoint 4 reading 0x0260 is the moment the probe is real.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Raw result of one command-plus-reply exchange, kept verbose on purpose:
 * first contact is debugged from what the wire did, not from a status code. */
typedef struct {
    uint64_t sent_word;      /* the frame as clocked out, first bit in bit 0 */
    size_t   sent_bits;
    int      wait_cycles;    /* clocks of busy stuffing, or -1 if no start bit */
    uint64_t reply;          /* reply payload, LSB first */
    size_t   reply_bits;
    uint8_t  reply_crc;      /* the six CRC bits that followed, if read */
    bool     crc_ok;         /* residue check over payload+CRC */
    bool     timed_out;
} dap_exchange_t;

/* Bring the PHY up on this board's pins at `clock_hz` (0 for the 1 MHz default). */
esp_err_t dap_probe_init(uint32_t clock_hz);

/*
 * The hot-attach opener: eight clocks with the line idle high, then the `sync`
 * frame, then the reply window.  `out` is filled in even on failure.
 */
esp_err_t dap_probe_sync(dap_exchange_t *out);

/* Read the DAPISC register.  Does not write it: see the plan on cold attach. */
esp_err_t dap_probe_dapisc_read(dap_exchange_t *out);

/* Select the Cerberus IOClient (client 1). */
esp_err_t dap_probe_client_set(uint8_t client, dap_exchange_t *out);

/* Issue a client_read of one IO instruction at the given size exponent. */
esp_err_t dap_probe_client_read(uint8_t io_instruction, uint8_t size_exponent,
                                size_t reply_bits, dap_exchange_t *out);

/*
 * Run the whole checkpoint sequence and log each step with its expected value.
 * Returns ESP_OK only if every checkpoint matched.
 */
esp_err_t dap_probe_bringup_report(void);

/*
 * Sweep the attach variables that are cheap to vary in software - bit rate,
 * how many idle clocks precede the first frame, whether TRST is pulsed, and
 * which LEN the sync frame carries - and report every combination that draws a
 * start bit out of the target.
 *
 * This exists because a silent target on the first attempt leaves a handful of
 * plausible causes, and trying them by hand is slower and less complete than
 * letting the probe try all of them in a few milliseconds.  Returns ESP_OK if
 * any combination answered.
 */
esp_err_t dap_probe_attach_sweep(void);

#ifdef __cplusplus
}
#endif
