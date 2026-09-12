/*
 * Program the TriCore's flash.  See tricore_flash.h for the shape of it.
 */

#include "tricore_flash.h"
#include "tricore_flash_priv.h"

#include <stdio.h>
#include <string.h>

#include "dap_phy_fpga.h"
#include "dap_frame.h"
#include "dap_probe.h"
#include "esp_log.h"
#include <inttypes.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tricore.h"

static const char *TAG = "TRICORE_FLASH";

/*
 * Whether to use client_blockwrite for bulk transfers.
 *
 * Cleared by /api/flash/start?slow=1, which forces the word-at-a-time path.
 * That is the difference between "the fabric's block write is wrong" and
 * "something else is wrong", and it is one request rather than a rebuild.
 */
bool tricore_flash_use_blockwrite = true;

void tricore_flash_set_blockwrite(bool enable)
{
    tricore_flash_use_blockwrite = enable;
}

tricore_flash_status_t tricore_flash_status;

void tricore_flash_set_phase(tricore_flash_phase_t phase, const char *message)
{
    tricore_flash_status.phase = phase;
    if (message) {
        strncpy(tricore_flash_status.message, message, sizeof(tricore_flash_status.message) - 1);
        tricore_flash_status.message[sizeof(tricore_flash_status.message) - 1] = '\0';
    }
}

void tricore_flash_get_status(tricore_flash_status_t *out)
{
    if (out) {
        *out = tricore_flash_status;
    }
}

/* -- geometry ------------------------------------------------------------- */

static uint32_t to_physical(uint32_t address)
{
    if (address >= PFLASH_CACHED &&
        address < PFLASH_CACHED + (PFLASH_END - PFLASH_BASE)) {
        return address - PFLASH_CACHED + PFLASH_BASE;
    }
    return address;
}

static bool is_never_program(uint32_t address)
{
    for (size_t i = 0; i < sizeof(NEVER_PROGRAM) / sizeof(NEVER_PROGRAM[0]); i++) {
        if (address >= NEVER_PROGRAM[i].start && address < NEVER_PROGRAM[i].end) {
            return true;
        }
    }
    return false;
}

static bool is_program_flash(uint32_t address)
{
    const uint32_t physical = to_physical(address);

    if (is_never_program(physical)) {
        return false;
    }
    return physical >= PFLASH_BASE && physical < PFLASH_END;
}

/*
 * The address to hand the command interpreter for a byte of the image.
 *
 * Normally the image's own.  While the swap is active it is the address in the
 * other bank group: the command interface is not remapped by the swap even
 * though reads are, so programming 0xA0020000 with the swap on writes the bank
 * that answers at 0xA0620000.  To reach the bank the core boots from, the image
 * has to be addressed there.  The stride is a whole number of sectors
 * (0x600000 / 0x4000 = 384), so alignment survives the translation.
 */
static uint32_t command_address(uint32_t address)
{
    if (!tricore_flash_swap_active) {
        return address;
    }
    if (address >= SWAP_GROUP_LOW && address < SWAP_GROUP_HIGH) {
        return address + SWAP_STRIDE;
    }
    if (address >= SWAP_GROUP_HIGH && address < SWAP_GROUP_HIGH + SWAP_STRIDE) {
        return address - SWAP_STRIDE;
    }
    return address;
}

/* -- the command interface ------------------------------------------------ */

static esp_err_t cycle(uint32_t offset, uint32_t value)
{
    return dap_probe_write32(CSI_BASE + offset, value);
}

static esp_err_t clear_status(void)
{
    return cycle(CYCLE_5554, CMD_CLEAR_STATUS);
}

/*
 * Wait for the bank to go idle, then put the interpreter back to idle.
 *
 * The stub cannot do either: issued too early a clear aborts the operation, and
 * it has no way to tell when the operation finished.  Left undone, the
 * interpreter is still mid-sequence and the next command is refused with SQER.
 */
esp_err_t tricore_flash_wait_idle(uint32_t address, uint32_t timeout_ms)
{
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    for (;;) {
        uint32_t status = 0;
        if (dap_probe_read32(DMU_HF_STATUS, &status) == ESP_OK &&
            !(status & STATUS_BUSY)) {
            break;
        }
        if (esp_timer_get_time() >= deadline) {
            ESP_LOGE(TAG, "flash still busy after the command at 0x%08" PRIX32,
                     address);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }

    /*
     * Read the error flags *before* clearing the interpreter, not after.
     *
     * CMD_CLEAR_STATUS drops the latched error along with the sequence state,
     * so an ERRSR read that follows it always comes back zero - which is how a
     * whole run of erases that the device refused with SQER could report
     * success.  The reference reads the flags first for exactly this reason.
     */
    uint32_t errsr = 0;
    const bool read_ok = dap_probe_read32(DMU_HF_ERRSR, &errsr) == ESP_OK;

    clear_status();

    if (read_ok && (errsr & ERRSR_FAILED)) {
        ESP_LOGE(TAG, "flash error after the command at 0x%08" PRIX32
                      ": ERRSR=0x%08" PRIX32 "%s%s%s%s%s", address, errsr,
                 (errsr & 1u) ? " OPER" : "", (errsr & 2u) ? " SQER" : "",
                 (errsr & 4u) ? " PROER" : "", (errsr & 8u) ? " PVER" : "",
                 (errsr & 16u) ? " EVER" : "");
        tricore_flash_status.errsr = errsr;
        return ESP_FAIL;
    }
    return ESP_OK;
}

/*
 * Drop the safety ENDINIT guarding the flash command interface.
 *
 * Without this the command sequences are accepted and report no error while
 * committing nothing - the page stays erased.
 *
 * The password is the *current* PW field inverted, which is why this reads the
 * register first and works from what it finds.  An earlier version here forced
 * a fixed password instead, which only happens to be right when PW already
 * holds its complement.  Read-backs stand in for the fences a core would use.
 */
esp_err_t tricore_flash_clear_safety_endinit(void)
{
    uint32_t con0 = 0, ignored = 0;

    if (dap_probe_read32(SCU_WDTS_CON0, &con0) != ESP_OK) {
        return ESP_FAIL;
    }
    const uint32_t password = con0 & 0xFFFFFFFCu;

    if (dap_probe_write32(SCU_WDTS_CON0, password ^ 0xFDu) != ESP_OK ||
        dap_probe_read32(SCU_WDTS_CON0, &ignored) != ESP_OK ||
        dap_probe_write32(SCU_WDTS_CON0, password | 0x02u) != ESP_OK) {
        return ESP_FAIL;
    }

    uint32_t back = 0;
    if (dap_probe_read32(SCU_WDTS_CON0, &back) != ESP_OK || (back & 1u)) {
        ESP_LOGE(TAG, "the safety ENDINIT would not clear (0x%08" PRIX32 ")",
                 back);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "safety ENDINIT cleared: SCU_WDTS_CON0 = 0x%08" PRIX32, back);
    return ESP_OK;
}

/*
 * Work out whether the address swap is remapping the bank groups.
 *
 * The two registers are the ones the vendor script's ReadSwapConfig reads.  An
 * active swap on a part whose mapping is not known would mean erasing an
 * unknown bank, but the mapping here is the profile's own, so finding it active
 * only changes which address the commands carry.
 */
void tricore_flash_check_swap(void)
{
    uint32_t active = 0, enabled = 0;

    tricore_flash_swap_active = false;
    if (dap_probe_read32(SWAP_ACTIVE_REG, &active) != ESP_OK) {
        ESP_LOGW(TAG, "could not read SCU_SWAPCTRL; assuming no swap");
        return;
    }
    dap_probe_read32(SWAP_EN_REG, &enabled);

    if ((active & SWAP_ACTIVE_MASK) == SWAP_ACTIVE_VAL) {
        tricore_flash_swap_active = true;
        ESP_LOGW(TAG, "the address swap is ACTIVE (SWAPCTRL 0x%08" PRIX32
                      "): erase and program go through the exchanged address "
                      "(+/-0x%X), verify stays where the core reads",
                 active, SWAP_STRIDE);
    } else {
        ESP_LOGI(TAG, "address swap not active (SWAPCTRL 0x%08" PRIX32
                      ", PROCONPF 0x%08" PRIX32 ")", active, enabled);
    }
}

/*
 * Put the flash back into a state the target can run from.
 *
 * Restoring the prefetch configuration matters for more than tidiness: while
 * the buffers are disabled, reads through the cached flash alias fault, so a
 * run that failed part way would otherwise leave the part looking dead.  Never
 * fails - there is nothing useful to do about it here.
 */
void tricore_flash_safe_shutdown(void)
{
    clear_status();
    if (tricore_flash_saved_valid) {
        dap_probe_write32(CPU0_FLASHCON0, tricore_flash_saved_flashcon0);
        dap_probe_write32(DMU_HF_PCONTROL, tricore_flash_saved_pcontrol);
    }
}

/* -- erase ---------------------------------------------------------------- */

/*
 * Erase one sector.
 *
 * One command per sector, deliberately.  The AA58 cycle does carry a sector
 * count and a contiguous image would be one command rather than forty-four,
 * but this device refuses any count above one with a sequence error - and
 * because CMD_CLEAR_STATUS drops the latched flag along with the sequence
 * state, a run that was refused this way used to report success and leave the
 * flash full of the old image.  The reference erases sector by sector for the
 * same reason.
 */
static esp_err_t erase_sector(uint32_t address)
{
    const uint32_t command = command_address(address);

    if (!is_program_flash(command)) {
        ESP_LOGE(TAG, "refusing to erase outside program flash: 0x%08" PRIX32,
                 command);
        return ESP_ERR_INVALID_ARG;
    }
    if (command % TRICORE_FLASH_SECTOR) {
        ESP_LOGE(TAG, "erase address 0x%08" PRIX32 " is not sector aligned",
                 command);
        return ESP_ERR_INVALID_ARG;
    }

    if (clear_status() != ESP_OK ||
        cycle(CYCLE_AA50, command) != ESP_OK ||
        cycle(CYCLE_AA58, 1) != ESP_OK ||
        cycle(CYCLE_AAA8, CMD_ERASE_SETUP) != ESP_OK ||
        cycle(CYCLE_AAA8, CMD_ERASE) != ESP_OK) {
        return ESP_FAIL;
    }
    if (tricore_flash_wait_idle(command, 5000) != ESP_OK) {
        return ESP_FAIL;
    }
    return clear_status();
}

/*
 * Confirm the interpreter really does enter program flash page mode.
 *
 * Worth doing once per run: if this is wrong every later page fails the same
 * way, and the failure is otherwise silent - the pages simply stay erased.
 * 0x50 selects program flash here; 0x5D would select data flash and set DFPAGE
 * instead, so the stub's loads would go to the wrong page buffer.
 */
static esp_err_t check_page_mode(void)
{
    uint32_t status = 0, errsr = 0;

    clear_status();
    if (cycle(CYCLE_5554, CMD_ENTER_PAGE_PFLASH) != ESP_OK ||
        dap_probe_read32(DMU_HF_STATUS, &status) != ESP_OK) {
        return ESP_FAIL;
    }
    dap_probe_read32(DMU_HF_ERRSR, &errsr);
    clear_status();

    if (!(status & STATUS_PFPAGE)) {
        ESP_LOGE(TAG, "program flash did not enter page mode (STATUS 0x%08"
                      PRIX32 ", ERRSR 0x%08" PRIX32 ")", status, errsr);
        tricore_flash_status.errsr = errsr;
        return ESP_FAIL;
    }
    return ESP_OK;
}

/*
 * The same CRC32 the stub computes, so the two can be compared.
 *
 * Reflected polynomial, no table - it matches loader/tricore_flash_loader.c
 * byte for byte, which is the only property that matters here.  Verifying this
 * way costs one round trip per region; reading the flash back over the link
 * instead is a kilobyte per round trip and there is 700 kB of it.
 */
uint32_t tricore_flash_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return ~crc;
}

/* -- the public entry point ----------------------------------------------- */

esp_err_t tricore_flash_checksum(uint32_t address, uint32_t length,
                                 uint32_t *out)
{
    /*
     * Scaled to the range rather than fixed.  What exhausts a budget here is
     * not a slow checksum but a range containing erased flash, whose reads
     * fault and trap the stub - which is a normal thing for a compare to meet,
     * so it has to cost a second rather than two minutes.
     */
    const uint32_t budget = 2000u + length / 500u;
    return tricore_flash_run_loader(LOADER_CMD_CHECKSUM, address, length, budget, out);
}

esp_err_t tricore_flash_write(const tricore_flash_region_t *regions,
                              size_t count)
{
    const int64_t started = esp_timer_get_time();

    if (regions == NULL || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&tricore_flash_status, 0, sizeof(tricore_flash_status));
    tricore_flash_installed = false;
    tricore_flash_set_phase(TRICORE_FLASH_PREPARING, "halting the target");

    for (size_t i = 0; i < count; i++) {
        tricore_flash_status.total_bytes += regions[i].length;
    }

    if (tricore_flash_install_loader() != ESP_OK) {
        /* tricore_flash_install_loader has already said which step it was. */
        return ESP_FAIL;
    }

    /*
     * -- the sectors the regions actually touch ---------------------------
     *
     * Every sector a region covers, not the span from the lowest to the
     * highest: a gap between sections is flash this run has no business
     * erasing, and on a differential image the gap is most of it.
     */
    static uint32_t sector_list[MAX_SECTORS];
    uint32_t sectors = 0;

    for (size_t i = 0; i < count; i++) {
        const uint32_t first = regions[i].address -
                               (regions[i].address % TRICORE_FLASH_SECTOR);
        const uint32_t end = regions[i].address + regions[i].length;

        for (uint32_t at = first; at < end; at += TRICORE_FLASH_SECTOR) {
            bool seen = false;
            for (uint32_t k = 0; k < sectors; k++) {
                if (sector_list[k] == at) {
                    seen = true;
                    break;
                }
            }
            if (seen) {
                continue;
            }
            if (sectors == MAX_SECTORS) {
                tricore_flash_set_phase(TRICORE_FLASH_FAILED, "too many sectors for one run");
                tricore_flash_safe_shutdown();
                return ESP_FAIL;
            }
            sector_list[sectors++] = at;
        }
    }
    tricore_flash_status.sectors = sectors;

    /* Refuse the whole run before erasing any of it, rather than finding the
     * one bad address half way through with the image already destroyed. */
    for (uint32_t k = 0; k < sectors; k++) {
        if (!is_program_flash(command_address(sector_list[k]))) {
            char why[96];
            snprintf(why, sizeof(why),
                     "0x%08" PRIX32 " is not erasable program flash",
                     sector_list[k]);
            tricore_flash_set_phase(TRICORE_FLASH_FAILED, why);
            tricore_flash_safe_shutdown();
            return ESP_FAIL;
        }
    }

    tricore_flash_set_phase(TRICORE_FLASH_ERASING, "erasing");
    ESP_LOGI(TAG, "erasing %" PRIu32 " sectors from 0x%08" PRIX32,
             sectors, sector_list[0]);
    for (uint32_t k = 0; k < sectors; k++) {
        if (erase_sector(sector_list[k]) != ESP_OK) {
            char why[96];
            snprintf(why, sizeof(why),
                     "erase of 0x%08" PRIX32 " failed, ERRSR 0x%08" PRIX32,
                     sector_list[k], tricore_flash_status.errsr);
            tricore_flash_set_phase(TRICORE_FLASH_FAILED, why);
            tricore_flash_safe_shutdown();
            return ESP_FAIL;
        }
        tricore_flash_status.sectors_done = k + 1u;
        tricore_flash_status.elapsed_ms =
            (uint32_t)((esp_timer_get_time() - started) / 1000);
    }

    /* -- program, a buffer at a time ------------------------------------- */
    tricore_flash_set_phase(TRICORE_FLASH_PROGRAMMING, "programming");

    if (check_page_mode() != ESP_OK) {
        char why[96];
        snprintf(why, sizeof(why),
                 "program flash would not enter page mode, ERRSR 0x%08" PRIX32,
                 tricore_flash_status.errsr);
        tricore_flash_set_phase(TRICORE_FLASH_FAILED, why);
        tricore_flash_safe_shutdown();
        return ESP_FAIL;
    }

    static uint8_t page[LOADER_BUFFER_BYTES];

    for (size_t i = 0; i < count; i++) {
        const tricore_flash_region_t *r = &regions[i];
        uint32_t offset = 0;

        while (offset < r->length) {
            /*
             * Whole pages only, and a page that a region ends inside is filled
             * out with the erased value rather than left to chance.
             */
            uint32_t chunk = r->length - offset;
            if (chunk > LOADER_BUFFER_BYTES) {
                chunk = LOADER_BUFFER_BYTES;
            }
            const uint32_t padded =
                (chunk + TRICORE_FLASH_PAGE - 1u) & ~(TRICORE_FLASH_PAGE - 1u);

            memset(page, TRICORE_FLASH_ERASED, padded);
            memcpy(page, r->data + offset, chunk);

            if (tricore_flash_write_block(LOADER_BUFFER, page, padded) != ESP_OK) {
                tricore_flash_set_phase(TRICORE_FLASH_FAILED, "could not fill the buffer");
                tricore_flash_safe_shutdown();
                return ESP_FAIL;
            }
            /* The stub issues the same command sequences the host would, so
             * what it gets is a command address and it goes through the swap
             * like every other one. */
            if (tricore_flash_run_loader(LOADER_CMD_PROGRAM,
                           command_address(r->address + offset),
                           padded / TRICORE_FLASH_PAGE, 20000, NULL) != ESP_OK) {
                /* tricore_flash_run_loader has already said what the stub reported; saying
                 * "programming failed" over the top of it threw away the only
                 * useful part. */
                tricore_flash_status.phase = TRICORE_FLASH_FAILED;
                tricore_flash_safe_shutdown();
                return ESP_FAIL;
            }

            offset += chunk;
            tricore_flash_status.done_bytes += chunk;
            tricore_flash_status.elapsed_ms =
                (uint32_t)((esp_timer_get_time() - started) / 1000);
        }
    }

    /*
     * -- verify, with the stub reading the flash --------------------------
     *
     * Put the prefetch configuration back first: while it is off, reads through
     * the cached alias fault, and the verify is a read of every byte just
     * written.  Verify uses the image's own addresses, not the command ones -
     * reads do follow the swap.
     */
    tricore_flash_safe_shutdown();
    tricore_flash_set_phase(TRICORE_FLASH_VERIFYING, "verifying");
    bool ok = true;

    for (size_t i = 0; i < count; i++) {
        uint32_t on_target = 0;
        if (tricore_flash_checksum(regions[i].address, regions[i].length,
                                   &on_target) != ESP_OK) {
            ok = false;
            break;
        }
        const uint32_t expect = tricore_flash_crc32(regions[i].data,
                                                    regions[i].length);
        if (on_target != expect) {
            ESP_LOGE(TAG, "region at 0x%08" PRIX32 ": target CRC 0x%08" PRIX32
                          ", image 0x%08" PRIX32,
                     regions[i].address, on_target, expect);
            ok = false;
            break;
        }
    }

    tricore_flash_status.elapsed_ms = (uint32_t)((esp_timer_get_time() - started) / 1000);
    tricore_flash_status.verified = ok;

    /*
     * Leave a board that runs, not one halted at its reset vector.
     *
     * The reference finishes the same way: reset, disarm the halt-after-reset
     * trigger, then resume every core.  The reset matters as much as the
     * resume - the cores have been halted with the flash mid-operation and the
     * prefetch configuration changed under them, and starting a fresh image
     * from the vector is the only sensible entry point for it.
     *
     * Only after a verified image.  A run that failed leaves the flash part
     * written, and starting that is worse than leaving the target stopped
     * where somebody can look at it.
     */
    if (ok) {
        tricore_flash_set_phase(TRICORE_FLASH_DONE, "starting the target");
        if (tricore_flash_reset_and_halt()) {
            int running = 0, present = 0;
            for (int core = 0; core < TRICORE_MAX_CORES; core++) {
                if (!tricore_core_present(core)) {
                    continue;
                }
                present++;
                tricore_halt_release(core);
                tricore_clear_debug_events(core);
                if (tricore_resume(core, 500) == ESP_OK) {
                    running++;
                } else {
                    ESP_LOGW(TAG, "CPU%d would not start", core);
                }
            }
            /* Only CPU0 not running means the board is not running: the others
             * are started by software on TC3xx, so a sibling still halted is
             * the image's business rather than this code's. */
            ESP_LOGI(TAG, "started %d of %d cores", running, present);
            tricore_flash_installed = false;
        } else {
            ESP_LOGW(TAG, "programmed, but the target would not reset to start");
        }
    }

    tricore_flash_set_phase(ok ? TRICORE_FLASH_DONE : TRICORE_FLASH_FAILED,
              ok ? "programmed, verified and started"
                 : "the image does not match");
    return ok ? ESP_OK : ESP_FAIL;
}
