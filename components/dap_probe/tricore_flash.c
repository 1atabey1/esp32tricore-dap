/*
 * TC3xx program flash: geometry, the DMU command interface, and the staged
 * erase / program / verify sequence shared by the web flasher and GDB `load`.
 * Follows tas-debug's flasher.py; the RAM loader lives in tricore_flash_loader.c.
 */

#include "tricore_flash.h"
#include "dap_lock.h"
#include "tricore_flash_priv.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tricore.h"

static const char *TAG = "TRICORE_FLASH";

bool tricore_flash_use_blockwrite = true;
tricore_flash_status_t tricore_flash_status;

static int64_t s_started_us;

void tricore_flash_set_blockwrite(bool enable)
{
    tricore_flash_use_blockwrite = enable;
}

void tricore_flash_set_phase(tricore_flash_phase_t phase, const char *message)
{
    tricore_flash_status.phase = phase;
    if (message) {
        strncpy(tricore_flash_status.message, message,
                sizeof(tricore_flash_status.message) - 1);
        tricore_flash_status.message[sizeof(tricore_flash_status.message) - 1] = '\0';
    }
    tricore_flash_status.elapsed_ms =
        (uint32_t)((esp_timer_get_time() - s_started_us) / 1000);
}

void tricore_flash_get_status(tricore_flash_status_t *out)
{
    if (out) {
        *out = tricore_flash_status;
    }
}

static esp_err_t fail(const char *fmt, uint32_t a, uint32_t b)
{
    char why[sizeof(tricore_flash_status.message)];

    snprintf(why, sizeof(why), fmt, a, b);
    ESP_LOGE(TAG, "%s", why);
    tricore_flash_set_phase(TRICORE_FLASH_FAILED, why);
    return ESP_FAIL;
}

/* -- geometry ------------------------------------------------------------- */

/* Populated program flash, by SCU_CHIPID.FSIZE, as tas-debug's tc38x profile
 * lists it.  An unlisted size is refused rather than guessed. */
static tricore_flash_range_t s_layout[2];
static size_t s_layout_count;

size_t tricore_flash_layout(const tricore_flash_range_t **ranges)
{
    uint32_t chipid = 0;

    s_layout_count = 0;
    if (dap_probe_read32(SCU_CHIPID, &chipid) == ESP_OK) {
        switch ((chipid >> 24) & 0xFu) {
        case 0xCu:                                  /* 10 MB, contiguous */
            s_layout[0] = (tricore_flash_range_t){ 0xA0000000u, 0xA0A00000u };
            s_layout_count = 1;
            break;
        case 0xBu:                                  /* 8 MB, two groups */
            s_layout[0] = (tricore_flash_range_t){ 0xA0000000u, 0xA0400000u };
            s_layout[1] = (tricore_flash_range_t){ 0xA0600000u, 0xA0A00000u };
            s_layout_count = 2;
            break;
        default:
            ESP_LOGE(TAG, "SCU_CHIPID 0x%08" PRIX32 ": FSIZE %u is not a known "
                          "TC38x layout", chipid, (unsigned)((chipid >> 24) & 0xFu));
            break;
        }
    }
    if (ranges) {
        *ranges = s_layout;
    }
    return s_layout_count;
}

uint32_t tricore_flash_to_physical(uint32_t address)
{
    if (address >= PFLASH_CACHED && address < PFLASH_CACHED + PFLASH_WINDOW) {
        return address - PFLASH_CACHED + PFLASH_BASE;
    }
    return address;
}

bool tricore_flash_is_never(uint32_t address)
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
    const uint32_t physical = tricore_flash_to_physical(address);

    if (tricore_flash_is_never(physical)) {
        return false;
    }
    for (size_t i = 0; i < s_layout_count; i++) {
        if (physical >= s_layout[i].start && physical < s_layout[i].end) {
            return true;
        }
    }
    return false;
}

/* The command interface is not remapped by the swap even though reads are, so
 * while it is active erase and program target the other bank group. */
static uint32_t command_address(uint32_t address)
{
    const uint32_t a = tricore_flash_to_physical(address);

    if (!tricore_flash_swap_active) {
        return a;
    }
    if (a >= SWAP_GROUP_LOW && a < SWAP_GROUP_HIGH) {
        return a + SWAP_STRIDE;
    }
    if (a >= SWAP_GROUP_HIGH && a < SWAP_GROUP_HIGH + SWAP_STRIDE) {
        return a - SWAP_STRIDE;
    }
    return a;
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

/* Wait for idle, then return the interpreter to idle.  ERRSR is read before
 * the clear, because CMD_CLEAR_STATUS also clears the latched error. */
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

    uint32_t errsr = 0;
    const bool read_ok = dap_probe_read32(DMU_HF_ERRSR, &errsr) == ESP_OK;

    clear_status();

    if (read_ok && (errsr & ERRSR_FAILED)) {
        ESP_LOGE(TAG, "flash error at 0x%08" PRIX32 ": ERRSR 0x%08" PRIX32
                      "%s%s%s%s%s", address, errsr,
                 (errsr & 1u) ? " OPER" : "", (errsr & 2u) ? " SQER" : "",
                 (errsr & 4u) ? " PROER" : "", (errsr & 8u) ? " PVER" : "",
                 (errsr & 16u) ? " EVER" : "");
        tricore_flash_status.errsr = errsr;
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Safety ENDINIT off, with the reference's password sequence: the current PW
 * field inverted, then written back with LCK set.  Read-backs act as fences. */
esp_err_t tricore_flash_clear_safety_endinit(void)
{
    uint32_t con0 = 0, ignored = 0, back = 0;

    if (dap_probe_read32(SCU_WDTS_CON0, &con0) != ESP_OK) {
        return ESP_FAIL;
    }
    const uint32_t password = con0 & 0xFFFFFFFCu;

    if (dap_probe_write32(SCU_WDTS_CON0, password ^ 0xFDu) != ESP_OK ||
        dap_probe_read32(SCU_WDTS_CON0, &ignored) != ESP_OK ||
        dap_probe_write32(SCU_WDTS_CON0, password | 0x02u) != ESP_OK ||
        dap_probe_read32(SCU_WDTS_CON0, &back) != ESP_OK || (back & 1u)) {
        ESP_LOGE(TAG, "safety ENDINIT would not clear (0x%08" PRIX32 ")", back);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* The two registers the vendor script's ReadSwapConfig reads. */
void tricore_flash_check_swap(void)
{
    uint32_t active = 0;

    tricore_flash_swap_active =
        dap_probe_read32(SWAP_ACTIVE_REG, &active) == ESP_OK &&
        (active & SWAP_ACTIVE_MASK) == SWAP_ACTIVE_VAL;
    if (tricore_flash_swap_active) {
        ESP_LOGW(TAG, "address swap active: erase/program use the exchanged bank");
    }
}

/* Restore what preflight turned off; with prefetch disabled, cached-alias
 * reads fault. */
void tricore_flash_safe_shutdown(void)
{
    clear_status();
    if (tricore_flash_saved_valid) {
        dap_probe_write32(CPU0_FLASHCON0, tricore_flash_saved_flashcon0);
        dap_probe_write32(DMU_HF_PCONTROL, tricore_flash_saved_pcontrol);
    }
}

/* Erase Logical Sector Range: `count` sectors from `address`, all inside one
 * physical sector and at most ERASE_RUN_MAX of them (TC38x data sheet). */
static esp_err_t erase_range(uint32_t address, uint32_t count)
{
    const uint32_t command = command_address(address);

    if (clear_status() != ESP_OK ||
        cycle(CYCLE_AA50, command) != ESP_OK ||
        cycle(CYCLE_AA58, count) != ESP_OK ||
        cycle(CYCLE_AAA8, CMD_ERASE_SETUP) != ESP_OK ||
        cycle(CYCLE_AAA8, CMD_ERASE) != ESP_OK ||
        tricore_flash_wait_idle(command, 5000) != ESP_OK) {
        return ESP_FAIL;
    }
    return clear_status();
}

static esp_err_t erase_sector(uint32_t address)
{
    return erase_range(address, 1);
}

/*
 * The data sheet allows up to 32 logical sectors (512 KB) per command inside
 * one 1 MB physical sector, erased in about the time of one (tMERP = tERP =
 * 0.5 s max).  The reference found counts above one refused with a sequence
 * error on its device; a refusal is detected (SQER) and the rest of the run,
 * and every run after it until reboot, goes one sector per command.
 */
#define ERASE_RUN_MAX     32u
#define PHYSICAL_SECTOR   0x00100000u
#define ERRSR_SQER        (1u << 1)

static bool s_multi_erase = true;

/* Erase sectors[0..n) (sorted, sector aligned), merging consecutive ones. */
static esp_err_t erase_sectors(const uint32_t *sectors, uint32_t n)
{
    for (uint32_t i = 0; i < n; ) {
        const int64_t t0 = esp_timer_get_time();
        const uint32_t first = command_address(sectors[i]);
        uint32_t run = 1;

        while (s_multi_erase && i + run < n && run < ERASE_RUN_MAX &&
               command_address(sectors[i + run]) == first + run * TRICORE_FLASH_SECTOR &&
               (first & ~(PHYSICAL_SECTOR - 1u)) ==
                   ((first + run * TRICORE_FLASH_SECTOR) & ~(PHYSICAL_SECTOR - 1u))) {
            run++;
        }
        esp_err_t err = erase_range(sectors[i], run);
        if (err != ESP_OK && run > 1 && (tricore_flash_status.errsr & ERRSR_SQER)) {
            ESP_LOGW(TAG, "a %u-sector erase was refused (SQER); one sector per command "
                          "from here on", (unsigned)run);
            s_multi_erase = false;
            tricore_flash_status.errsr = 0;
            run = 1;
            err = erase_sector(sectors[i]);
        }
        tricore_flash_status.erase_ms += (uint32_t)((esp_timer_get_time() - t0) / 1000);
        if (err != ESP_OK) {
            return fail("erase of 0x%08" PRIX32 " failed, ERRSR 0x%08" PRIX32,
                        sectors[i], tricore_flash_status.errsr);
        }
        tricore_flash_status.sectors_done += run;
        i += run;
    }
    return ESP_OK;
}

/* 0x50 is program flash page mode; 0x5D would select data flash instead. */
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
        tricore_flash_status.errsr = errsr;
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* The CRC32 the stub computes: reflected 0xEDB88320, no table.  _update takes
 * and returns the running (not inverted) value, starting at 0xFFFFFFFF. */
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return crc;
}

uint32_t tricore_flash_crc32(const uint8_t *data, uint32_t length)
{
    return ~crc32_update(0xFFFFFFFFu, data, length);
}

/* -- stages --------------------------------------------------------------- */

/*
 * The DAP clock while flashing: 24 MHz instead of the 4 MHz attach default.
 * Filling the loader buffer is the only link-bound part (block writes, which
 * wide mode cannot carry); at 4 MHz it took 2.75 s of a 700 kB image.  The
 * on-target CRC catches a bad transfer.  Every exit restores the default.
 */
#define FLASH_DAP_DIV    0u
#define ATTACH_DAP_DIV   5u

static void flash_clock(bool fast)
{
    if (dap_phy_fpga_ready() && !dap_phy_fpga_is_wide()) {
        dap_phy_fpga_set_div(fast ? FLASH_DAP_DIV : ATTACH_DAP_DIV);
    }
}

/* A session holds the DAP lock from begin to end, across GDB packets. */
static bool s_session_locked;

static void session_unlock(void)
{
    if (s_session_locked) {
        s_session_locked = false;
        flash_clock(false);
        dap_unlock();
    }
}

static esp_err_t begin_locked(void)
{
    s_started_us = esp_timer_get_time();
    memset(&tricore_flash_status, 0, sizeof(tricore_flash_status));
    tricore_flash_installed = false;
    tricore_flash_set_phase(TRICORE_FLASH_PREPARING, "halting the target");

    if (tricore_flash_install_loader() != ESP_OK) {
        return ESP_FAIL;            /* the loader has set the reason */
    }
    flash_clock(true);              /* after the resets, which attach at 4 MHz */
    if (tricore_flash_layout(NULL) == 0) {
        tricore_flash_safe_shutdown();
        return fail("unknown flash layout (SCU_CHIPID)", 0, 0);
    }
    if (check_page_mode() != ESP_OK) {
        tricore_flash_safe_shutdown();
        return fail("no program flash page mode, ERRSR 0x%08" PRIX32,
                    tricore_flash_status.errsr, 0);
    }
    return ESP_OK;
}

esp_err_t tricore_flash_begin(void)
{
    dap_lock();
    if (s_session_locked) {
        dap_unlock();               /* already inside a session */
    }
    s_session_locked = true;
    const esp_err_t err = begin_locked();
    if (err != ESP_OK) {
        session_unlock();
    }
    return err;
}

esp_err_t tricore_flash_erase(uint32_t address, uint32_t length)
{
    const uint32_t first = address & ~(TRICORE_FLASH_SECTOR - 1u);

    tricore_flash_set_phase(TRICORE_FLASH_ERASING, "erasing");
    /* Check the whole range before touching any of it. */
    for (uint32_t at = first; at < address + length; at += TRICORE_FLASH_SECTOR) {
        if (!is_program_flash(command_address(at))) {
            return fail("0x%08" PRIX32 " is not erasable program flash", at, 0);
        }
    }
    for (uint32_t at = first; at < address + length; at += TRICORE_FLASH_SECTOR) {
        const int64_t t0 = esp_timer_get_time();
        const esp_err_t err = erase_sector(at);
        tricore_flash_status.erase_ms += (uint32_t)((esp_timer_get_time() - t0) / 1000);
        if (err != ESP_OK) {
            return fail("erase of 0x%08" PRIX32 " failed, ERRSR 0x%08" PRIX32,
                        at, tricore_flash_status.errsr);
        }
        tricore_flash_status.sectors_done++;
    }
    return ESP_OK;
}

esp_err_t tricore_flash_program(uint32_t address, const uint8_t *data,
                                uint32_t length)
{
    static uint8_t buf[LOADER_BUFFER_BYTES];

    if (address % TRICORE_FLASH_PAGE) {
        return fail("program address 0x%08" PRIX32 " is not page aligned",
                    address, 0);
    }
    tricore_flash_set_phase(TRICORE_FLASH_PROGRAMMING, "programming");

    for (uint32_t off = 0; off < length; ) {
        const uint32_t chunk = (length - off > LOADER_BUFFER_BYTES)
                                   ? LOADER_BUFFER_BYTES : length - off;
        const uint32_t padded = (chunk + TRICORE_FLASH_PAGE - 1u) &
                                ~(TRICORE_FLASH_PAGE - 1u);

        if (!is_program_flash(command_address(address + off))) {
            return fail("0x%08" PRIX32 " is not programmable", address + off, 0);
        }
        memset(buf, TRICORE_FLASH_ERASED, padded);   /* partial pages stay erased */
        memcpy(buf, data + off, chunk);

        const int64_t t0 = esp_timer_get_time();
        if (tricore_flash_write_block(LOADER_BUFFER, buf, padded) != ESP_OK) {
            return fail("could not fill the loader buffer", 0, 0);
        }
        const int64_t t1 = esp_timer_get_time();
        const esp_err_t err = tricore_flash_run_loader(LOADER_CMD_PROGRAM,
                                                       command_address(address + off),
                                                       padded / TRICORE_FLASH_PAGE, 20000,
                                                       NULL);
        tricore_flash_status.write_ms += (uint32_t)((t1 - t0) / 1000);
        tricore_flash_status.loader_ms += (uint32_t)((esp_timer_get_time() - t1) / 1000);
        if (err != ESP_OK) {
            tricore_flash_status.phase = TRICORE_FLASH_FAILED;
            return ESP_FAIL;        /* the loader has set the reason */
        }
        off += chunk;
        tricore_flash_status.done_bytes += chunk;
    }
    return ESP_OK;
}

esp_err_t tricore_flash_checksum(uint32_t address, uint32_t length,
                                 uint32_t *out)
{
    /* Scaled to the range: erased flash traps the stub, and that has to cost a
     * second, not minutes. */
    return tricore_flash_run_loader(LOADER_CMD_CHECKSUM, address, length,
                                    2000u + length / 500u, out);
}

esp_err_t tricore_flash_verify(uint32_t address, const uint8_t *data,
                               uint32_t length)
{
    uint32_t on_target = 0;

    tricore_flash_set_phase(TRICORE_FLASH_VERIFYING, "verifying");
    if (tricore_flash_checksum(address, length, &on_target) != ESP_OK) {
        return ESP_FAIL;
    }
    const uint32_t expect = tricore_flash_crc32(data, length);
    if (on_target != expect) {
        return fail("CRC 0x%08" PRIX32 " on target, 0x%08" PRIX32 " in image",
                    on_target, expect);
    }
    return ESP_OK;
}

static esp_err_t end_locked(bool start_target)
{
    tricore_flash_safe_shutdown();

    /* Leave the part at its reset vector either way: the loader has been
     * running on CPU0 and its registers belong to the stub. */
    if (!tricore_flash_reset_and_halt()) {
        return fail("the target would not reset after flashing", 0, 0);
    }
    tricore_flash_installed = false;
    if (!start_target) {
        return ESP_OK;
    }

    int running = 0;
    for (int core = 0; core < TRICORE_MAX_CORES; core++) {
        if (!tricore_core_present(core) || !tricore_core_started(core)) {
            continue;
        }
        tricore_halt_release(core);
        tricore_clear_debug_events(core);
        if (tricore_resume(core, 500) == ESP_OK) {
            running++;
        }
    }
    ESP_LOGI(TAG, "started %d core(s)", running);
    return ESP_OK;
}

esp_err_t tricore_flash_end(bool start_target)
{
    const esp_err_t err = end_locked(start_target);
    session_unlock();
    return err;
}

/* -- differential ----------------------------------------------------------- */

bool tricore_flash_differential = true;

void tricore_flash_set_differential(bool enable)
{
    tricore_flash_differential = enable;
}

/*
 * The target's CRC of [address, +length), without marking the session failed
 * when the stub cannot read it: erased flash has no valid ECC and traps the
 * stub, which is the case that needs programming anyway.  The timeout is
 * scaled to what the stub needs (~0.4 us per byte), not to verify's.
 */
static bool target_crc_quiet(uint32_t address, uint32_t length, uint32_t *crc)
{
    const tricore_flash_phase_t phase = tricore_flash_status.phase;
    char message[sizeof(tricore_flash_status.message)];
    const uint32_t errsr = tricore_flash_status.errsr;

    memcpy(message, tricore_flash_status.message, sizeof(message));
    const esp_err_t err = tricore_flash_run_loader(LOADER_CMD_CHECKSUM, address, length,
                                                   60u + length / 2000u, crc);
    if (err != ESP_OK) {
        tricore_flash_status.phase = phase;
        memcpy(tricore_flash_status.message, message, sizeof(message));
        tricore_flash_status.errsr = errsr;
    }
    return err == ESP_OK;
}

/*
 * As tas-debug's select_changed: true if any byte this image programs into
 * the sector differs from the target - only those bytes (with the page
 * padding program adds at a region's end), never the rest of the sector,
 * which is not ours to predict.  Skipping a sector leaves data outside the
 * image alone; erasing it would destroy it, so a skip is never less safe.
 */
static bool sector_differs(uint32_t sector, const tricore_flash_region_t *regions,
                           size_t count)
{
    static const uint8_t erased[TRICORE_FLASH_PAGE] = { 0 };   /* TRICORE_FLASH_ERASED */

    for (size_t i = 0; i < count; i++) {
        const uint32_t start = regions[i].address, end = start + regions[i].length;
        const uint32_t lo = (start > sector) ? start : sector;
        const uint32_t hi = (end < sector + TRICORE_FLASH_SECTOR) ? end
                                                                  : sector + TRICORE_FLASH_SECTOR;
        if (lo >= hi) {
            continue;
        }
        /* program pads the last page of a region with the erased value */
        const uint32_t pad = (hi == end) ? ((TRICORE_FLASH_PAGE - (hi % TRICORE_FLASH_PAGE)) %
                                            TRICORE_FLASH_PAGE) : 0u;
        uint32_t expect = crc32_update(0xFFFFFFFFu, regions[i].data + (lo - start), hi - lo);
        expect = ~crc32_update(expect, erased, pad);

        uint32_t actual = 0;
        if (!target_crc_quiet(tricore_flash_to_physical(lo), hi - lo + pad, &actual) ||
            actual != expect) {
            return true;
        }
    }
    return false;
}

/* Program the parts of a region that fall into the kept sectors. */
static esp_err_t program_kept(const tricore_flash_region_t *r, const uint32_t *sectors,
                              const bool *keep, uint32_t n)
{
    const uint32_t end = r->address + r->length;

    for (uint32_t k = 0; k < n; ) {
        if (!keep[k] || sectors[k] + TRICORE_FLASH_SECTOR <= r->address || sectors[k] >= end) {
            k++;
            continue;
        }
        /* a run of consecutive kept sectors */
        uint32_t last = k;
        while (last + 1 < n && keep[last + 1] &&
               sectors[last + 1] == sectors[last] + TRICORE_FLASH_SECTOR &&
               sectors[last + 1] < end) {
            last++;
        }
        const uint32_t lo = (sectors[k] > r->address) ? sectors[k] : r->address;
        const uint32_t top = sectors[last] + TRICORE_FLASH_SECTOR;
        const uint32_t hi = (top < end) ? top : end;
        if (tricore_flash_program(lo, r->data + (lo - r->address), hi - lo) != ESP_OK) {
            return ESP_FAIL;
        }
        k = last + 1;
    }
    return ESP_OK;
}

static int cmp_u32(const void *a, const void *b)
{
    const uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return (x > y) - (x < y);
}

/* -- the whole-image path, for the web flasher and GDB `load` ---------------- */

/* Inside a session: erase and program what differs, then verify everything.
 * Leaves the target halted; the caller ends the session. */
esp_err_t tricore_flash_apply(const tricore_flash_region_t *regions, size_t count)
{
    static uint32_t sectors[MAX_SECTORS];
    static bool keep[MAX_SECTORS];
    uint32_t n = 0;

    if (regions == NULL || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* The sectors the regions touch, not the span between them. */
    for (size_t i = 0; i < count; i++) {
        const uint32_t end = regions[i].address + regions[i].length;
        for (uint32_t at = regions[i].address & ~(TRICORE_FLASH_SECTOR - 1u);
             at < end; at += TRICORE_FLASH_SECTOR) {
            bool seen = false;
            for (uint32_t k = 0; k < n && !seen; k++) {
                seen = (sectors[k] == at);
            }
            if (!seen) {
                if (n == MAX_SECTORS) {
                    tricore_flash_safe_shutdown();
                    return fail("more than %" PRIu32 " sectors", MAX_SECTORS, 0);
                }
                sectors[n++] = at;
            }
        }
    }
    qsort(sectors, n, sizeof(sectors[0]), cmp_u32);
    for (uint32_t k = 0; k < n; k++) {
        if (!is_program_flash(command_address(sectors[k]))) {
            tricore_flash_safe_shutdown();
            return fail("0x%08" PRIX32 " is not erasable program flash", sectors[k], 0);
        }
    }

    /* Differential: only sectors whose bytes differ are erased and written. */
    uint32_t changed = 0;
    if (tricore_flash_differential) {
        tricore_flash_set_phase(TRICORE_FLASH_PREPARING, "comparing with the target");
    }
    const int64_t t_cmp = esp_timer_get_time();
    for (uint32_t k = 0; k < n; k++) {
        keep[k] = !tricore_flash_differential || sector_differs(sectors[k], regions, count);
        changed += keep[k];
    }
    tricore_flash_status.compare_ms = (uint32_t)((esp_timer_get_time() - t_cmp) / 1000);
    tricore_flash_status.sectors = changed;
    tricore_flash_status.sectors_skipped = n - changed;
    for (size_t i = 0; i < count; i++) {
        const uint32_t start = regions[i].address, end = start + regions[i].length;
        for (uint32_t k = 0; k < n; k++) {
            const uint32_t lo = (start > sectors[k]) ? start : sectors[k];
            const uint32_t top = sectors[k] + TRICORE_FLASH_SECTOR;
            const uint32_t hi = (end < top) ? end : top;
            if (keep[k] && lo < hi) {
                tricore_flash_status.total_bytes += hi - lo;
            }
        }
    }
    if (n - changed) {
        ESP_LOGI(TAG, "differential: %" PRIu32 " of %" PRIu32 " sectors already match",
                 n - changed, n);
    }

    /* Erase the changed ones (compacted, still sorted), then program them. */
    static uint32_t erase_list[MAX_SECTORS];
    uint32_t m = 0;
    for (uint32_t k = 0; k < n; k++) {
        if (keep[k]) {
            erase_list[m++] = sectors[k];
        }
    }
    esp_err_t err = ESP_OK;
    if (m) {
        tricore_flash_set_phase(TRICORE_FLASH_ERASING, "erasing");
        err = erase_sectors(erase_list, m);
    }
    for (size_t i = 0; i < count && err == ESP_OK && m; i++) {
        err = program_kept(&regions[i], sectors, keep, n);
    }
    if (err != ESP_OK) {
        tricore_flash_safe_shutdown();
        return err;
    }

    /* Verify with prefetch restored, at the image's own addresses. */
    tricore_flash_safe_shutdown();
    for (size_t i = 0; i < count && err == ESP_OK; i++) {
        err = tricore_flash_verify(regions[i].address, regions[i].data,
                                   regions[i].length);
    }
    tricore_flash_status.verified = (err == ESP_OK);
    if (err == ESP_OK) {
        tricore_flash_set_phase(TRICORE_FLASH_DONE, "programmed and verified");
    }
    return err;
}

static const tricore_ucb_image_t *s_ucb;

void tricore_flash_set_ucb(const tricore_ucb_image_t *image)
{
    s_ucb = image;
}

static esp_err_t write_locked(const tricore_flash_region_t *regions,
                              size_t count)
{
    if (regions == NULL || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (tricore_flash_begin() != ESP_OK) {
        return ESP_FAIL;
    }
    if (tricore_flash_apply(regions, count) != ESP_OK) {
        return ESP_FAIL;            /* left halted, for inspection */
    }

    /* The reference's order: program flash verified, then the UCBs, then the
     * reset that starts the target - which happens whatever the UCBs did. */
    if (s_ucb != NULL) {
        tricore_flash_set_phase(TRICORE_FLASH_PROGRAMMING, "programming the boot mode headers");
        tricore_flash_status.ucb = (tricore_ucb_program(s_ucb) == ESP_OK)
                                       ? TRICORE_FLASH_UCB_OK : TRICORE_FLASH_UCB_FAILED;
    }

    tricore_flash_set_phase(TRICORE_FLASH_DONE, "starting the target");
    if (tricore_flash_end(true) != ESP_OK) {
        return ESP_FAIL;
    }
    if (tricore_flash_status.ucb == TRICORE_FLASH_UCB_FAILED) {
        tricore_flash_set_phase(TRICORE_FLASH_FAILED, "programmed, verified and started, but "
                                "a boot mode header was refused or failed (UCB report)");
        return ESP_FAIL;
    }
    tricore_flash_set_phase(TRICORE_FLASH_DONE, "programmed, verified and started");
    return ESP_OK;
}

esp_err_t tricore_flash_write(const tricore_flash_region_t *regions,
                              size_t count)
{
    dap_lock();
    const esp_err_t err = write_locked(regions, count);
    session_unlock();
    dap_unlock();
    return err;
}
