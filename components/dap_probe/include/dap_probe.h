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
    bool     idle_high;   /* line never went low: nothing was driving it */
} dap_exchange_t;

/*
 * Configure the DAP pins and park them in a state that is safe for an attached
 * target, without sending anything.
 *
 * The pin that matters is TRST: on this bench it goes to the target's reset,
 * and the target holds it high through a pull-up.  Left as an output at its
 * power-on level it sits *low*, which holds the target in reset - a TC38x then
 * stays powered but never runs its application.  Black Magic Probe's
 * platform_init() configures the same pin as an output and leaves it low, so
 * this has to run after the background tasks start, and it has to run whether
 * or not any DAP bring-up is enabled.
 */
esp_err_t dap_probe_park_idle(void);

/* Bring the PHY up on this board's pins at `clock_hz` (0 for the 1 MHz default). */
esp_err_t dap_probe_init(uint32_t clock_hz);

/*
 * The hot-attach opener: eight clocks with the line idle high, then the `sync`
 * frame, then the reply window.  `out` is filled in even on failure.
 */
esp_err_t dap_probe_sync(dap_exchange_t *out);

/* Sync with retries and a flush between them; the first attach often fails. */
esp_err_t dap_probe_attach(dap_exchange_t *out, int attempts);

/*
 * Write DAPISC and read back the updated value.  `cold` selects the 66-bit
 * initialisation telegram with the 0x4ABBAF53 signature, for the
 * Enabled-to-Active transition; otherwise the short LEN-16 form is used, which
 * is what an already-Active device accepts.
 */
esp_err_t dap_probe_dapisc(uint16_t value, bool cold, dap_exchange_t *out);

/* Adopt the reply-wait window implied by a DAPISC value. */
void dap_probe_note_dapisc(uint16_t dapisc);

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
/*
 * Replay the reference probe's pre-sync preamble, byte for byte.
 *
 * Taken from a USB capture of a miniWiggler attach: a 43-byte pattern that is
 * a pure period-12 repeat of 000011111100, then a 56-bit sequence, then a read
 * window.  The reference sends this at five clock rates before it sends sync,
 * and this project never did - which is the last structural difference between
 * a probe that completes an attach and one that gets a single answer to sync
 * and is then ignored.  0xAAAAAAAA is itself a training pattern, so answering
 * sync may only mean the device is in a training state rather than attached.
 */
esp_err_t dap_probe_replay_preamble(void);

/* The four bring-up frames back to back, with no host work between them. */
esp_err_t dap_probe_attach_now(dap_exchange_t out[6]);

/* Write an IOClient register or operand: instruction, size exponent, data. */
esp_err_t dap_probe_client_write(uint8_t io_instruction, uint8_t size_exponent,
                                 uint64_t data, size_t data_bits,
                                 dap_exchange_t *out);

/*
 * Put the IOClient in read/write mode, so read instructions address the system
 * bus rather than COMDATA.  Required before any memory access.
 */
esp_err_t dap_probe_set_rw_mode(bool supervisor);

/* Log an IOINFO value with its bits named. */
void dap_probe_log_ioinfo(uint16_t v);

/* Clear Cerberus Error State, in which reads and writes are silently dropped. */
esp_err_t dap_probe_clear_error_state(void);

/* Set IOADDR and read the 32-bit word there. */
esp_err_t dap_probe_read32(uint32_t addr, uint32_t *value);

/* Set IOADDR and write a 32-bit word there. */
esp_err_t dap_probe_write32(uint32_t addr, uint32_t value);

/*
 * Enable OCDS: the four contiguous OEC.PAT pattern writes, a check that
 * OSTATE.OEN came up, then OCNTRL and CT.SETE.  Without this the whole
 * miniMCDS register space bus-errors.
 */
esp_err_t dap_probe_enable_ocds(void);

/*
 * Read `count` words (1..256) in one client_blockread telegram.  The device
 * loads IOADDR itself from the telegram and post-increments per word.
 */
esp_err_t dap_probe_blockread(uint32_t addr, uint32_t *words, size_t count);

/* Block read throughput in kB/s, against the 38 kB/s baseline. */
esp_err_t dap_probe_block_throughput(void);

/* Single-word read rate, and how far the bit-banged PHY carries. */
esp_err_t dap_probe_rate_test(void);

/* Does the reply trailer length explain the alternation? */
esp_err_t dap_probe_trailer_sweep(void);

/* How many consecutive syncs the device answers, and what recovers it. */
esp_err_t dap_probe_sync_health(int attempts);

/* Fresh sync then one candidate second frame, for each candidate. */
esp_err_t dap_probe_second_frame_matrix(void);

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

/*
 * Send `sync` and clock a long raw window in, logging every bit.
 *
 * The upstream frame length is the one thing neither the documentation nor the
 * captures pinned down: the reference probe clocks 56 bits after sync while
 * this code clocks 38, and leaving a reply half-clocked would desynchronise
 * the device - which is exactly the symptom, sync answering and everything
 * after it going quiet.  Measuring beats guessing.
 */
esp_err_t dap_probe_dump_sync_reply(size_t window_bits);

/*
 * Try each Port C pin that can be an output as the clock, with the data line
 * fixed on the one bidirectional pin, and report which one draws a reply.
 * Cheaper than asking someone to trace a cable, and it covers the case where
 * the wires are not where the documentation says.
 */
esp_err_t dap_probe_clock_pin_search(void);

#ifdef __cplusplus
}
#endif
