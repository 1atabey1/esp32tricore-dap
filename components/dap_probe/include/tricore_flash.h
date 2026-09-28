/*
 * TC3xx program flash, following tas-debug's flasher.
 *
 * The DMU only accepts command sequences from the core, so a stub in CPU0's
 * scratchpad programs each buffer and computes CRC32s for verification.  Two
 * ways in: tricore_flash_write() for a whole image (the web flasher), or the
 * stages below for a caller that streams one (GDB `load`).
 *
 * Addresses may be cached (0x8...) or uncached (0xA...).  Long-running; call
 * from a task of its own.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "tricore_ucb.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRICORE_FLASH_PAGE   32u        /* written whole */
#define TRICORE_FLASH_SECTOR 0x4000u    /* erased whole */
#define TRICORE_FLASH_ERASED 0xFFu

typedef struct {
    uint32_t address;
    uint32_t length;
    const uint8_t *data;
} tricore_flash_region_t;

/* A populated program-flash range, uncached, end exclusive. */
typedef struct {
    uint32_t start;
    uint32_t end;
} tricore_flash_range_t;

typedef enum {
    TRICORE_FLASH_IDLE = 0,
    TRICORE_FLASH_PREPARING,
    TRICORE_FLASH_ERASING,
    TRICORE_FLASH_PROGRAMMING,
    TRICORE_FLASH_VERIFYING,
    TRICORE_FLASH_DONE,
    TRICORE_FLASH_FAILED,
} tricore_flash_phase_t;

enum {
    TRICORE_FLASH_UCB_OFF = 0,  /* not asked for */
    TRICORE_FLASH_UCB_OK,       /* nothing refused, nothing failed */
    TRICORE_FLASH_UCB_FAILED,   /* see tricore_ucb_report() */
};

typedef struct {
    tricore_flash_phase_t phase;
    uint32_t total_bytes;
    uint32_t done_bytes;
    uint32_t sectors;           /* to erase and program */
    uint32_t sectors_done;
    uint32_t sectors_skipped;   /* differential: already holding the image */
    uint32_t elapsed_ms;
    uint32_t compare_ms;        /* differential: comparing with the target */
    uint32_t erase_ms;          /* sector erases */
    uint32_t write_ms;          /* data into the loader buffer, over DAP */
    uint32_t loader_ms;         /* the loader programming pages */
    uint32_t errsr;             /* DMU_HF_ERRSR at the last failure */
    bool     verified;
    uint8_t  ucb;               /* TRICORE_FLASH_UCB_* */
    char     message[160];
} tricore_flash_status_t;

/* Erase the sectors the regions touch, program, verify by CRC32, then reset
 * and start the target.  A failed image is left halted.  Sectors that already
 * hold the image are skipped (see tricore_flash_set_differential), and
 * consecutive sectors are erased in one command. */
esp_err_t tricore_flash_write(const tricore_flash_region_t *regions,
                              size_t count);

/* The same inside an open session (begin() ... end()), without the reset:
 * compare, erase, program, verify. */
esp_err_t tricore_flash_apply(const tricore_flash_region_t *regions, size_t count);

/* Stages.  begin() resets and halts the target and installs the loader; end()
 * restores the flash configuration and leaves the target at its reset vector,
 * running it if asked. */
esp_err_t tricore_flash_begin(void);
esp_err_t tricore_flash_erase(uint32_t address, uint32_t length);
esp_err_t tricore_flash_program(uint32_t address, const uint8_t *data,
                                uint32_t length);
esp_err_t tricore_flash_verify(uint32_t address, const uint8_t *data,
                               uint32_t length);
esp_err_t tricore_flash_end(bool start_target);

/* Populated program flash, from SCU_CHIPID.FSIZE; 0 if the size is unknown. */
size_t tricore_flash_layout(const tricore_flash_range_t **ranges);

/* Cached alias to uncached; anything else unchanged. */
uint32_t tricore_flash_to_physical(uint32_t address);

/* The UCBs and configuration store, which the program flash path never
 * erases or writes (UCBs only through tricore_flash_set_ucb). */
bool tricore_flash_is_never(uint32_t address);

/* Off forces word-at-a-time transfers instead of client_blockwrite. */
void tricore_flash_set_blockwrite(bool enable);

/* On (the default): tricore_flash_write erases and programs only the sectors
 * whose bytes differ from the image, as tas-debug's differential mode. */
void tricore_flash_set_differential(bool enable);

/* Also program the boot mode headers from `image` in the next
 * tricore_flash_write, after the program flash verifies and before the
 * target starts (tas-debug's --ucb).  NULL (the default) leaves every UCB
 * alone.  The image must stay valid until the write returns. */
void tricore_flash_set_ucb(const tricore_ucb_image_t *image);

void tricore_flash_get_status(tricore_flash_status_t *out);

/* The CRC32 the stub computes, for comparing against an image held here. */
uint32_t tricore_flash_crc32(const uint8_t *data, uint32_t length);

/* CRC32 of a target range, computed by the stub. */
esp_err_t tricore_flash_checksum(uint32_t address, uint32_t length,
                                 uint32_t *out);

#ifdef __cplusplus
}
#endif
