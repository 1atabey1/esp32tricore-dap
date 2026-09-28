/*
 * TC3xx user configuration blocks: the boot mode headers, written from an
 * image inside a flash session.  A port of tas-debug's ucb.py and
 * ucb_program.py - the same catalogue, checks, plan and command sequences.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define TRICORE_UCB_BASE    0xAF400000u
#define TRICORE_UCB_SIZE    0x200u          /* one block, 512 bytes */
#define TRICORE_UCB_WINDOW  0x6000u         /* every block, BMHD0_ORIG .. OTP7_COPY */
#define TRICORE_UCB_SLOTS   (TRICORE_UCB_WINDOW / TRICORE_UCB_SIZE)

/* What an image carries for the UCB area: TRICORE_UCB_WINDOW bytes from
 * TRICORE_UCB_BASE, unspecified bytes 0x00 (erased data flash), and bit n of
 * `present` set when the image has records in slot n. */
typedef struct {
    const uint8_t *data;
    uint64_t       present;
} tricore_ucb_image_t;

/* Name of the block at slot n, or NULL for a slot no block occupies. */
const char *tricore_ucb_slot_name(uint32_t slot);

/*
 * Plan and write the boot mode headers the image carries; every other block is
 * listed and skipped.  Call inside a flash session with CPU0 halted, after the
 * program flash is written and verified (the reference's order).  ESP_OK when
 * nothing was refused and nothing failed; tricore_ucb_report() says what
 * happened, one line per block.
 */
esp_err_t tricore_ucb_program(const tricore_ucb_image_t *image);

/* The last run's report, empty before the first. */
const char *tricore_ucb_report(void);
