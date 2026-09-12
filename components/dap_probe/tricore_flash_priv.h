#pragma once

/*
 * Shared between tricore_flash.c and tricore_flash_loader.c.  Not part of the
 * interface.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "tricore_flash.h"

/* -- the command sequence interpreter, in the data flash window ----------- */

#define CSI_BASE        0xAF000000u
#define CYCLE_5554      0x5554u
#define CYCLE_AA50      0xAA50u
#define CYCLE_AA58      0xAA58u
#define CYCLE_AAA8      0xAAA8u

#define CYCLE_55F0      0x55F0u

#define CMD_CLEAR_STATUS 0xFAu
#define CMD_ERASE_SETUP  0x80u
#define CMD_ERASE        0x50u
/* Program flash page mode.  0x5D would select data flash instead, and then the
 * page loads go to DFPAGE and the program commits nothing. */
#define CMD_ENTER_PAGE_PFLASH 0x50u

/* PFPAGE: the interpreter is in program flash page mode. */
#define STATUS_PFPAGE    (1u << 21)

#define DMU_HF_STATUS    0xF8040010u
#define DMU_HF_ERRSR     0xF8040034u
#define DMU_HF_PCONTROL  0xF8040064u
#define DMU_HF_ACCEN0    0xF80400FCu
#define CPU0_FLASHCON0   0xF8801100u
#define SCU_WDTS_CON0    0xF00362A8u
/* CPU0's own watchdog, and the reset status that names what tripped. */
#define WDTCPU0_CON0     0xF0036100u
#define WDTCPU0_CON1     0xF0036104u
#define SCU_RSTSTAT      0xF0036050u
#define SCU_PLLSTAT      0xF0036010u
#define SCU_CCUCON0      0xF0036030u
/* WDTxCON1.DR: stop the watchdog counting.  Writable only while that
 * watchdog's own ENDINIT is clear. */
#define WDTCON1_DR       (1u << 3)

/* Reserved master tag 0x3F in every TAGx field disables that prefetch buffer,
 * so the fetch pipeline cannot speculatively read a bank being written. */
#define FLASHCON0_NO_PREFETCH 0x3F3F3F3Fu
/* DEMAND = 0b11 at bits [11:10]. */
#define PCONTROL_DEMAND  (3u << 10)
/* D0BUSY, D1BUSY, P0BUSY..P3BUSY */
#define STATUS_BUSY      0x0000003Fu
/* OPER, SQER, PROER, PVER, EVER */
#define ERRSR_FAILED     0x0000001Fu

/* -- program flash geometry ---------------------------------------------- */

/* Uncached program flash.  Programming goes through this alias so nothing is
 * served from a cache line. */
#define PFLASH_BASE      0xA0000000u
#define PFLASH_END       0xA1000000u
#define PFLASH_CACHED    0x80000000u

/*
 * The program flash address swap, as the device profile declares it.
 *
 * The two 6 MB bank groups exchange places, which is the state a UDS update
 * leaves a device in: it runs from the group that was the spare.  Reads follow
 * the swap, the command interpreter does not, so erase and program have to be
 * addressed through the exchanged address while verify reads the image's own.
 */
#define SWAP_GROUP_LOW   0xA0000000u
#define SWAP_GROUP_HIGH  0xA0600000u
#define SWAP_STRIDE      (SWAP_GROUP_HIGH - SWAP_GROUP_LOW)
/* SCU_SWAPCTRL.ADDRCONFIG; 0b10 means the swap is active. */
#define SWAP_ACTIVE_REG  0xF003614Cu
#define SWAP_ACTIVE_MASK 0x3u
#define SWAP_ACTIVE_VAL  0x2u
/* DMU_HF_PROCONPF, the two SWAPEN bits. */
#define SWAP_EN_REG      0xF8040084u
#define SWAP_EN_MASK     0x30000u

/*
 * Ranges that are never erased or written, whatever an image asks for.
 *
 * The UCBs carry the boot configuration and a bad write there is not
 * recoverable with this tool; the configuration sector store is the same kind
 * of thing.  These are the ranges the device profile lists as never_program,
 * collapsed to the two contiguous blocks they form.
 */
static const struct { uint32_t start, end; } NEVER_PROGRAM[] = {
    { 0xAF400000u, 0xAF406000u },   /* UCB00..UCB47 */
    { 0xAF800000u, 0xAF810000u },   /* CFS */
};

/* The most sectors an image can touch: 12 MB of program flash. */
#define MAX_SECTORS 768

/* Where the stub, its parameters, its buffer and its stack sit in CPU0's
 * scratchpad. */
#define LOADER_CODE   0x70100000u
#define LOADER_PARAMS 0x70000000u
#define LOADER_BUFFER 0x70004000u
#define LOADER_STACK  0x70038000u

#define LOADER_BUFFER_BYTES (32u * 1024u)

#define LOADER_CMD_PROGRAM  1u
#define LOADER_CMD_CHECKSUM 3u
#define LOADER_ST_RUNNING   0u
#define LOADER_ST_OK        1u

/* Put the stub in scratchpad and arm its self-halt; safe to call repeatedly. */
esp_err_t tricore_flash_install_loader(void);

/* Run one stub command and wait for it to halt itself. */
esp_err_t tricore_flash_run_loader(uint32_t cmd, uint32_t address,
                                   uint32_t count, uint32_t timeout_ms,
                                   uint32_t *checksum);

/* Bulk write to target RAM, by whatever route is fastest. */
esp_err_t tricore_flash_write_block(uint32_t address, const uint8_t *data,
                                    size_t len);

/* Wait for the bank to go idle, then return the interpreter to idle. */
esp_err_t tricore_flash_wait_idle(uint32_t address, uint32_t timeout_ms);

/* Put the flash back into a state the target can run from. */
void tricore_flash_safe_shutdown(void);

void tricore_flash_set_phase(tricore_flash_phase_t phase, const char *message);

/* Progress, shared so both halves can report into it. */
extern tricore_flash_status_t tricore_flash_status;

/* Cleared by /api/flash/start?slow=1 to force the word-at-a-time path. */
extern bool tricore_flash_use_blockwrite;

/* Whether the address swap is remapping the bank groups, found in preflight. */
extern bool tricore_flash_swap_active;

/* What preflight turned off, to put back afterwards. */
extern bool     tricore_flash_saved_valid;
extern uint32_t tricore_flash_saved_flashcon0;
extern uint32_t tricore_flash_saved_pcontrol;

/* Whether the stub is already in scratchpad. */
extern bool tricore_flash_installed;

/* Read SCU_SWAPCTRL and DMU_HF_PROCONPF to see which bank answers where. */
void tricore_flash_check_swap(void);

/* Drop the safety ENDINIT guarding the flash command interface. */
esp_err_t tricore_flash_clear_safety_endinit(void);

/* Reset the application and stop the cores before any of it runs. */
bool tricore_flash_reset_and_halt(void);
