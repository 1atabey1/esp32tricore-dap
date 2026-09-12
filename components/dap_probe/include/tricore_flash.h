/*
 * Program the TriCore's flash, from an image the probe holds.
 *
 * Ported from tas-debug's flasher, which is the reference this project follows
 * for anything TriCore-specific.  Two things carry over unchanged because they
 * are the whole design:
 *
 *   - Program flash command cycles are only accepted from the core, so the
 *     debug link cannot issue them.  A small stub runs from CPU0's scratchpad
 *     and walks the pages itself; the host fills a buffer and hands it over.
 *     One round trip per chunk instead of one per 32-byte page.
 *   - Verification is a CRC32 the stub computes by reading the flash through
 *     the core.  Reading it back over the link instead is a kilobyte per round
 *     trip, and there is 700 kB of it.
 *
 * What is new here is how the buffer gets there: client_blockwrite streams it
 * from the fabric's FIFO, so the transfer is close to wire speed rather than
 * two register-file round trips per word.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Program flash page: the smallest unit that can be written, and it is written
 * whole. */
#define TRICORE_FLASH_PAGE   32u
/* Erase sector on TC38x. */
#define TRICORE_FLASH_SECTOR 0x4000u
/* What erased flash reads as. */
#define TRICORE_FLASH_ERASED 0xFFu

typedef struct {
    uint32_t address;      /* physical (uncached) start */
    uint32_t length;
    const uint8_t *data;
} tricore_flash_region_t;

typedef enum {
    TRICORE_FLASH_IDLE = 0,
    TRICORE_FLASH_PREPARING,
    TRICORE_FLASH_ERASING,
    TRICORE_FLASH_PROGRAMMING,
    TRICORE_FLASH_VERIFYING,
    TRICORE_FLASH_DONE,
    TRICORE_FLASH_FAILED,
} tricore_flash_phase_t;

typedef struct {
    tricore_flash_phase_t phase;
    uint32_t total_bytes;
    uint32_t done_bytes;
    uint32_t sectors;
    uint32_t sectors_done;
    uint32_t elapsed_ms;
    bool     verified;
    char     message[128];
} tricore_flash_status_t;

/*
 * Program `regions` into flash and verify them.
 *
 * Erases every sector the regions touch, programs whole pages - bytes no region
 * covers are left erased - and checks each region with the stub's CRC32.  The
 * target is halted throughout and left halted; the caller resets and starts it.
 *
 * Long-running: call it from a task of its own, not from the web server's.
 */
esp_err_t tricore_flash_write(const tricore_flash_region_t *regions,
                              size_t count);

/* A snapshot of what the flash task is doing, for progress reporting. */
void tricore_flash_get_status(tricore_flash_status_t *out);

/* The same CRC32 the stub computes, over an image held here. */
uint32_t tricore_flash_crc32(const uint8_t *data, uint32_t length);

/* CRC32 of a target range, computed by the stub. */
esp_err_t tricore_flash_checksum(uint32_t address, uint32_t length,
                                 uint32_t *out);

#ifdef __cplusplus
}
#endif
