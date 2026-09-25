/* Shared by tricore_flash.c and tricore_flash_loader.c.  Not part of the interface. */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "tricore_flash.h"

/* -- DMU command sequence interpreter (in the data flash window) ---------- */

#define CSI_BASE              0xAF000000u
#define CYCLE_5554            0x5554u
#define CYCLE_55F0            0x55F0u
#define CYCLE_AA50            0xAA50u
#define CYCLE_AA58            0xAA58u
#define CYCLE_AAA8            0xAAA8u

#define CMD_CLEAR_STATUS      0xFAu
#define CMD_ERASE_SETUP       0x80u
#define CMD_ERASE             0x50u
#define CMD_ENTER_PAGE_PFLASH 0x50u     /* 0x5D would be data flash */

#define DMU_HF_STATUS         0xF8040010u
#define DMU_HF_ERRSR          0xF8040034u
#define DMU_HF_PCONTROL       0xF8040064u
#define STATUS_BUSY           0x0000003Fu   /* D0/D1BUSY, P0..P3BUSY */
#define STATUS_PFPAGE         (1u << 21)
#define ERRSR_FAILED          0x0000001Fu   /* OPER SQER PROER PVER EVER */

#define CPU0_FLASHCON0        0xF8801100u
#define FLASHCON0_NO_PREFETCH 0x3F3F3F3Fu   /* reserved tag 0x3F disables each buffer */
#define PCONTROL_DEMAND       (3u << 10)

#define SCU_WDTS_CON0         0xF00362A8u
#define SCU_CHIPID            0xF0036140u   /* FSIZE in bits 27:24 */

/* -- program flash --------------------------------------------------------- */

#define PFLASH_BASE           0xA0000000u   /* uncached; programming goes here */
#define PFLASH_CACHED         0x80000000u
#define PFLASH_WINDOW         0x01000000u

/* Address swap: the two 6 MB bank groups exchange places (tc38x profile). */
#define SWAP_GROUP_LOW        0xA0000000u
#define SWAP_GROUP_HIGH       0xA0600000u
#define SWAP_STRIDE           (SWAP_GROUP_HIGH - SWAP_GROUP_LOW)
#define SWAP_ACTIVE_REG       0xF003614Cu   /* SCU_SWAPCTRL.ADDRCONFIG */
#define SWAP_ACTIVE_MASK      0x3u
#define SWAP_ACTIVE_VAL       0x2u

/* Never erased or written: the UCBs and the configuration sector store. */
static const struct { uint32_t start, end; } NEVER_PROGRAM[] = {
    { 0xAF400000u, 0xAF406000u },
    { 0xAF800000u, 0xAF810000u },
};

#define MAX_SECTORS           768           /* 12 MB of 16 KB sectors */

/* -- the RAM loader, in CPU0 scratchpad ----------------------------------- */

#define LOADER_CODE           0x70100000u
#define LOADER_PARAMS         0x70000000u
#define LOADER_BUFFER         0x70004000u
#define LOADER_STACK          0x70038000u
#define LOADER_BUFFER_BYTES   (32u * 1024u)

#define LOADER_CMD_PROGRAM    1u
#define LOADER_CMD_CHECKSUM   3u
#define LOADER_ST_RUNNING     0u
#define LOADER_ST_OK          1u

esp_err_t tricore_flash_install_loader(void);
esp_err_t tricore_flash_run_loader(uint32_t cmd, uint32_t address,
                                   uint32_t count, uint32_t timeout_ms,
                                   uint32_t *checksum);
esp_err_t tricore_flash_write_block(uint32_t address, const uint8_t *data,
                                    size_t len);
bool      tricore_flash_reset_and_halt(void);

esp_err_t tricore_flash_wait_idle(uint32_t address, uint32_t timeout_ms);
esp_err_t tricore_flash_clear_safety_endinit(void);
void      tricore_flash_check_swap(void);
void      tricore_flash_safe_shutdown(void);
void      tricore_flash_set_phase(tricore_flash_phase_t phase, const char *message);

extern tricore_flash_status_t tricore_flash_status;
extern bool     tricore_flash_use_blockwrite;
extern bool     tricore_flash_installed;
extern bool     tricore_flash_swap_active;
extern bool     tricore_flash_saved_valid;
extern uint32_t tricore_flash_saved_flashcon0;
extern uint32_t tricore_flash_saved_pcontrol;
