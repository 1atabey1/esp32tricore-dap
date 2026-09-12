/*
 * Program the TriCore's flash.  See tricore_flash.h for the shape of it.
 */

#include "tricore_flash.h"

#include <string.h>

#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "esp_log.h"
#include <inttypes.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tricore.h"

static const char *TAG = "TRICORE_FLASH";

/* -- the command sequence interpreter, in the data flash window ----------- */

#define CSI_BASE        0xAF000000u
#define CYCLE_5554      0x5554u
#define CYCLE_AA50      0xAA50u
#define CYCLE_AA58      0xAA58u
#define CYCLE_AAA8      0xAAA8u

#define CMD_CLEAR_STATUS 0xFAu
#define CMD_ERASE_SETUP  0x80u
#define CMD_ERASE        0x50u

#define DMU_HF_STATUS    0xF8040010u
#define DMU_HF_ERRSR     0xF8040034u
#define DMU_HF_PCONTROL  0xF8040064u
#define DMU_HF_ACCEN0    0xF80400FCu
#define CPU0_FLASHCON0   0xF8801100u
#define SCU_WDTS_CON0    0xF00362A8u

/* Reserved master tag 0x3F in every TAGx field disables that prefetch buffer,
 * so the fetch pipeline cannot speculatively read a bank being written. */
#define FLASHCON0_NO_PREFETCH 0x3F3F3F3Fu
/* DEMAND = 0b11 at bits [11:10]. */
#define PCONTROL_DEMAND  (3u << 10)
/* D0BUSY, D1BUSY, P0BUSY..P3BUSY */
#define STATUS_BUSY      0x0000003Fu
/* OPER, SQER, PROER, PVER, EVER */
#define ERRSR_FAILED     0x0000001Fu

/* -- the RAM loader ------------------------------------------------------- */

/*
 * Built from tas-debug's loader/tricore_flash_loader.c and carried as bytes so
 * flashing needs no TriCore toolchain here.  Rebuild it there if the source
 * changes; it is 310 bytes and has no relocations.
 */
static const uint8_t LOADER_BLOB[] = {
    0x82,0x0F,0xA5,0x7F,0x10,0x00,0xA5,0x7F,0x18,0x00,0xA5,0x7F,0x14,0x00,0x85,0x7F,
    0x00,0x00,0x20,0x08,0xDF,0x1F,0x58,0x80,0x85,0x72,0x0C,0x08,0x85,0x79,0x04,0x00,
    0x91,0x00,0xF0,0x3A,0x80,0x22,0x91,0x00,0xF0,0x6A,0x91,0x10,0xF0,0x7A,0x91,0x10,
    0xF0,0xCA,0x91,0x10,0xF0,0x4A,0x82,0x0F,0xA2,0x29,0xD9,0x33,0x54,0x55,0x3B,0xA0,
    0x0F,0x60,0x3B,0x00,0x05,0x70,0x82,0x00,0xD9,0x66,0x70,0x75,0xD9,0x77,0x90,0x9A,
    0xD9,0xCC,0x98,0x9A,0xD9,0x44,0xA8,0xAA,0x3B,0x00,0x0A,0x10,0x3B,0xA0,0x0A,0x80,
    0x3B,0x00,0xFA,0x40,0x80,0x25,0x85,0x72,0x08,0x00,0x42,0x95,0x7F,0x2F,0x53,0x80,
    0x74,0x36,0x0D,0x00,0x80,0x04,0x74,0x37,0x0D,0x00,0x80,0x04,0xA0,0x05,0xA0,0x3F,
    0x01,0x52,0x10,0xD0,0x09,0xD2,0x40,0x09,0x89,0x62,0x40,0x09,0x0D,0x00,0x80,0x04,
    0xD9,0x55,0x08,0x00,0xFC,0xF6,0x74,0x75,0x74,0xC0,0x74,0x41,0x74,0x48,0x0D,0x00,
    0x80,0x04,0x59,0xA4,0x04,0x00,0x19,0xA2,0x04,0x00,0x76,0x27,0x19,0xA2,0x04,0x00,
    0xC2,0xF2,0x59,0xA2,0x04,0x00,0x3C,0xF8,0xC2,0x1F,0xD9,0x22,0x20,0x00,0xA5,0x7F,
    0x18,0x00,0x3C,0xD1,0x85,0x72,0x00,0x00,0x82,0x4F,0xDF,0x32,0x25,0x80,0x85,0x74,
    0x04,0x08,0x85,0x7F,0x08,0x08,0x7B,0x90,0xDB,0x3E,0xA0,0x03,0x82,0xFF,0x1B,0x03,
    0x32,0x38,0xFD,0xF0,0x0A,0x00,0x46,0x0F,0xA5,0x7F,0x1C,0x00,0x85,0x7F,0x08,0x00,
    0xA5,0x7F,0x18,0x00,0x3C,0x0F,0x01,0x43,0x10,0x20,0x14,0x22,0xA0,0x72,0xC6,0x2F,
    0x8F,0xFF,0x1F,0x20,0x37,0x0F,0x41,0xF0,0x26,0x3F,0xC6,0x2F,0xFC,0x2A,0xB0,0x13,
    0x3C,0xE9,0x82,0x1F,0x91,0x40,0x80,0xFF,0xD9,0xFF,0x34,0x00,0x48,0x02,0xA5,0x72,
    0x14,0x00,0xA5,0x7F,0x10,0x00,0x0D,0x00,0x80,0x04,0x00,0xA0,0x3C,0x00,
};

/* CPU0's scratchpad on TC38x: 0x70000000..0x7003C000 data, 0x70100000 program. */
#define LOADER_CODE   0x70100000u
#define LOADER_PARAMS 0x70000000u
#define LOADER_BUFFER 0x70004000u
#define LOADER_STACK  0x70038000u
/* Bigger chunks only save hand-offs to the stub; the bytes travel either way. */
#define LOADER_BUFFER_BYTES (32u * 1024u)

#define LOADER_CMD_PROGRAM  1u
#define LOADER_CMD_CHECKSUM 3u
#define LOADER_ST_RUNNING   0u
#define LOADER_ST_OK        1u

/* EVTA = halt.  Without it the stub's `debug` instruction is a no-op, and a
 * halt request through DBGSR is ignored while it runs from scratchpad - the
 * core would run on with no way to stop it short of a reset. */
#define SWEVT_HALT 0x2u
#define OFF_SWEVT  0xFD08u
#define CPU0_BASE  0xF8810000u

static bool s_installed;

static tricore_flash_status_t s_status;

static void set_phase(tricore_flash_phase_t phase, const char *message)
{
    s_status.phase = phase;
    if (message) {
        strncpy(s_status.message, message, sizeof(s_status.message) - 1);
        s_status.message[sizeof(s_status.message) - 1] = '\0';
    }
}

void tricore_flash_get_status(tricore_flash_status_t *out)
{
    if (out) {
        *out = s_status;
    }
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
static esp_err_t wait_idle(uint32_t address, uint32_t timeout_ms)
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

    clear_status();

    uint32_t errsr = 0;
    if (dap_probe_read32(DMU_HF_ERRSR, &errsr) == ESP_OK &&
        (errsr & ERRSR_FAILED)) {
        ESP_LOGE(TAG, "flash error after the command at 0x%08" PRIX32
                      ": ERRSR=0x%08" PRIX32, address, errsr);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/*
 * Drop the safety ENDINIT guarding the flash command interface.
 *
 * Without this the command sequences are accepted and report no error, and
 * nothing is committed - the page stays erased.  The sequence is the one the
 * toolchain's own crt0 uses: read it, write it back with the password field
 * forced and LCK clear, then write the value wanted with LCK set.
 */
static esp_err_t clear_safety_endinit(void)
{
    uint32_t v = 0;

    if (dap_probe_read32(SCU_WDTS_CON0, &v) != ESP_OK) {
        return ESP_FAIL;
    }
    const uint32_t unlock = (v & ~14u) | 241u;
    const uint32_t apply  = ((v & ~1u) & ~12u) | 242u;

    if (dap_probe_write32(SCU_WDTS_CON0, unlock) != ESP_OK ||
        dap_probe_write32(SCU_WDTS_CON0, apply) != ESP_OK) {
        return ESP_FAIL;
    }

    uint32_t back = 0;
    if (dap_probe_read32(SCU_WDTS_CON0, &back) != ESP_OK || (back & 1u)) {
        ESP_LOGE(TAG, "the safety ENDINIT would not clear (0x%08" PRIX32 ")",
                 back);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* -- the stub ------------------------------------------------------------- */

/*
 * Write a block of bytes to target RAM, by whatever route is fastest.
 *
 * The fabric streams it with client_blockwrite - one command and a FIFO's worth
 * of parcels, with the host out of the loop - and falls back to word writes if
 * that is not available, which costs two frames a word and is the difference
 * between seconds and minutes over a 700 kB image.
 */
static esp_err_t write_block(uint32_t address, const uint8_t *data, size_t len)
{
    static uint32_t words[DAP_FPGA_BLOCK_WORDS];

    if (len % 4u) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t offset = 0; offset < len; ) {
        const size_t chunk = (len - offset > sizeof(words))
                                 ? sizeof(words) : (len - offset);
        const size_t n = chunk / 4u;

        for (size_t i = 0; i < n; i++) {
            const size_t at = offset + 4u * i;
            words[i] = (uint32_t)data[at] |
                       ((uint32_t)data[at + 1] << 8) |
                       ((uint32_t)data[at + 2] << 16) |
                       ((uint32_t)data[at + 3] << 24);
        }

        esp_err_t err = dap_phy_fpga_ready()
            ? dap_phy_fpga_block_write(address + offset, words, n)
            : ESP_ERR_NOT_SUPPORTED;

        if (err == ESP_ERR_NOT_SUPPORTED) {
            for (size_t i = 0; i < n; i++) {
                err = dap_probe_write32(address + offset + 4u * i, words[i]);
                if (err != ESP_OK) {
                    break;
                }
            }
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "writing 0x%08" PRIX32 " failed", address + offset);
            return err;
        }
        offset += chunk;
    }
    return ESP_OK;
}

static esp_err_t install_loader(void)
{
    if (s_installed) {
        return ESP_OK;
    }
    if (tricore_halt(0, 0, 500) != ESP_OK) {
        ESP_LOGE(TAG, "CPU0 would not halt");
        return ESP_FAIL;
    }

    /* The blob is not a multiple of four, so it is padded to a word; the stub
     * never reads past its own code. */
    static uint8_t padded[(sizeof(LOADER_BLOB) + 3u) & ~3u];
    memset(padded, 0, sizeof(padded));
    memcpy(padded, LOADER_BLOB, sizeof(LOADER_BLOB));

    if (write_block(LOADER_CODE, padded, sizeof(padded)) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Read it back: scratchpad that is not there accepts writes and keeps
     * nothing, and the stub would then program whatever was in the buffer. */
    uint32_t first = 0;
    if (dap_probe_read32(LOADER_CODE, &first) != ESP_OK ||
        first != ((uint32_t)padded[0] | ((uint32_t)padded[1] << 8) |
                  ((uint32_t)padded[2] << 16) | ((uint32_t)padded[3] << 24))) {
        ESP_LOGE(TAG, "the loader did not stay in scratchpad at 0x%08X",
                 LOADER_CODE);
        return ESP_FAIL;
    }

    if (dap_probe_write32(CPU0_BASE + OFF_SWEVT, SWEVT_HALT) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Polling the DMU from the core is only safe if nothing pulls data out of
     * the bank being written, and the prefetch buffers will do exactly that
     * speculatively.  A reset restores both. */
    uint32_t accen0 = 0;
    if (dap_probe_read32(DMU_HF_ACCEN0, &accen0) == ESP_OK &&
        accen0 != 0xFFFFFFFFu) {
        ESP_LOGW(TAG, "DMU_HF_ACCEN0 = 0x%08" PRIX32 " (not fully enabled)",
                 accen0);
    }
    dap_probe_write32(CPU0_FLASHCON0, FLASHCON0_NO_PREFETCH);
    dap_probe_write32(DMU_HF_PCONTROL, PCONTROL_DEMAND);

    if (clear_safety_endinit() != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "loader in place at 0x%08X (%u bytes)", LOADER_CODE,
             (unsigned)sizeof(LOADER_BLOB));
    s_installed = true;
    return ESP_OK;
}

/*
 * Run one command on the stub and wait for it to halt itself.
 *
 * The stub writes its status and stops with a `debug` instruction, so waiting
 * for the halt is the completion signal - ordinary memory reads work while the
 * core runs, register reads do not.
 */
static esp_err_t run_loader(uint32_t cmd, uint32_t address, uint32_t count,
                            uint32_t timeout_ms, uint32_t *checksum)
{
    if (install_loader() != ESP_OK) {
        return ESP_FAIL;
    }

    uint32_t params[8] = { cmd, address, count, LOADER_BUFFER, 0, 0, 0, 0 };
    if (write_block(LOADER_PARAMS, (const uint8_t *)params,
                    sizeof(params)) != ESP_OK) {
        return ESP_FAIL;
    }

    /* A leaf routine still needs a stack pointer, and A11 has to be something
     * valid even though the stub never returns. */
    if (tricore_write_reg(0, 16 + 10, LOADER_STACK) != ESP_OK ||
        tricore_write_reg(0, 16 + 11, LOADER_CODE) != ESP_OK ||
        tricore_write_pc(0, LOADER_CODE) != ESP_OK ||
        tricore_request_resume(0) != ESP_OK) {
        ESP_LOGE(TAG, "could not start the loader");
        return ESP_FAIL;
    }

    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    uint32_t dbgsr = 0;

    while (esp_timer_get_time() < deadline) {
        if (tricore_dbgsr(0, &dbgsr) == ESP_OK && (dbgsr & 0x2u)) {
            break;      /* halted itself */
        }
        vTaskDelay(1);
    }
    tricore_halt(0, 0, 200);

    uint32_t out[8] = {0};
    for (int i = 0; i < 8; i++) {
        if (dap_probe_read32(LOADER_PARAMS + 4u * i, &out[i]) != ESP_OK) {
            ESP_LOGE(TAG, "could not read the loader's result");
            return ESP_FAIL;
        }
    }

    if (out[4] == LOADER_ST_RUNNING) {
        ESP_LOGE(TAG, "the loader did not finish (%" PRIu32 " of %" PRIu32 ")",
                 out[6], count);
        return ESP_ERR_TIMEOUT;
    }
    if (out[4] != LOADER_ST_OK) {
        ESP_LOGE(TAG, "the loader failed at 0x%08" PRIX32 " after %" PRIu32
                      " of %" PRIu32 " (ERRSR=0x%08" PRIX32 ")",
                 address, out[6], count, out[5]);
        return ESP_FAIL;
    }
    if (checksum) {
        *checksum = out[7];
    }
    return wait_idle(address, 2000);
}

/* -- erase ---------------------------------------------------------------- */

/*
 * Erase a run of sectors with one command.
 *
 * The interface takes a sector count, so a contiguous image is one erase rather
 * than forty-four - and erase dominates the time a full program takes.
 */
static esp_err_t erase_sectors(uint32_t address, uint32_t sectors)
{
    if (address % TRICORE_FLASH_SECTOR) {
        ESP_LOGE(TAG, "erase address 0x%08" PRIX32 " is not sector aligned",
                 address);
        return ESP_ERR_INVALID_ARG;
    }

    if (clear_status() != ESP_OK ||
        cycle(CYCLE_AA50, address) != ESP_OK ||
        cycle(CYCLE_AA58, sectors) != ESP_OK ||
        cycle(CYCLE_AAA8, CMD_ERASE_SETUP) != ESP_OK ||
        cycle(CYCLE_AAA8, CMD_ERASE) != ESP_OK) {
        return ESP_FAIL;
    }
    /* Generous: a sector takes a few milliseconds and this may be dozens. */
    return wait_idle(address, 1000u + 200u * sectors);
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
    return run_loader(LOADER_CMD_CHECKSUM, address, length, budget, out);
}

esp_err_t tricore_flash_write(const tricore_flash_region_t *regions,
                              size_t count)
{
    const int64_t started = esp_timer_get_time();

    if (regions == NULL || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_status, 0, sizeof(s_status));
    s_installed = false;
    set_phase(TRICORE_FLASH_PREPARING, "halting the target");

    for (size_t i = 0; i < count; i++) {
        s_status.total_bytes += regions[i].length;
    }

    if (install_loader() != ESP_OK) {
        set_phase(TRICORE_FLASH_FAILED, "could not install the RAM loader");
        return ESP_FAIL;
    }

    /* -- erase every sector the regions touch, in runs ------------------- */
    uint32_t lowest = 0xFFFFFFFFu, highest = 0;
    for (size_t i = 0; i < count; i++) {
        const uint32_t first = regions[i].address -
                               (regions[i].address % TRICORE_FLASH_SECTOR);
        const uint32_t last = regions[i].address + regions[i].length - 1u;
        if (first < lowest) {
            lowest = first;
        }
        if (last > highest) {
            highest = last;
        }
    }
    const uint32_t sectors =
        (highest - lowest) / TRICORE_FLASH_SECTOR + 1u;
    s_status.sectors = sectors;

    set_phase(TRICORE_FLASH_ERASING, "erasing");
    ESP_LOGI(TAG, "erasing %" PRIu32 " sectors from 0x%08" PRIX32,
             sectors, lowest);
    if (erase_sectors(lowest, sectors) != ESP_OK) {
        set_phase(TRICORE_FLASH_FAILED, "erase failed");
        return ESP_FAIL;
    }
    s_status.sectors_done = sectors;

    /* -- program, a buffer at a time ------------------------------------- */
    set_phase(TRICORE_FLASH_PROGRAMMING, "programming");

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

            if (write_block(LOADER_BUFFER, page, padded) != ESP_OK) {
                set_phase(TRICORE_FLASH_FAILED, "could not fill the buffer");
                return ESP_FAIL;
            }
            if (run_loader(LOADER_CMD_PROGRAM, r->address + offset,
                           padded / TRICORE_FLASH_PAGE, 20000, NULL) != ESP_OK) {
                set_phase(TRICORE_FLASH_FAILED, "programming failed");
                return ESP_FAIL;
            }

            offset += chunk;
            s_status.done_bytes += chunk;
            s_status.elapsed_ms =
                (uint32_t)((esp_timer_get_time() - started) / 1000);
        }
    }

    /* -- verify, with the stub reading the flash ------------------------- */
    set_phase(TRICORE_FLASH_VERIFYING, "verifying");
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

    s_status.elapsed_ms = (uint32_t)((esp_timer_get_time() - started) / 1000);
    s_status.verified = ok;
    set_phase(ok ? TRICORE_FLASH_DONE : TRICORE_FLASH_FAILED,
              ok ? "programmed and verified" : "the image does not match");
    return ok ? ESP_OK : ESP_FAIL;
}
