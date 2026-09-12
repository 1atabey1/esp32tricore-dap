/*
 * The RAM loader: a stub in CPU0 scratchpad that issues the flash command
 * sequences, because the DMU only accepts them from the core.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "dap_frame.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tricore.h"
#include "tricore_flash.h"
#include "tricore_flash_priv.h"

static const char *TAG = "TRICORE_FLASH";

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
/* Bigger chunks only save hand-offs to the stub; the bytes travel either way. */

/*
 * How long the stub waits for a page to program, in loop iterations.
 *
 * The stub reads no DMU status - on this device a status read issued by the
 * core hangs it whenever a flash operation is in flight - so it paces itself
 * with a spin loop and the host checks the error flags afterwards.  The blob
 * is built with 4000, which is about 200 us at the 100 MHz backup clock a DAS
 * DCO_RESET_AND_HALT leaves the part on, against the ~100 us a page needs.
 *
 * This probe gets there by a different route: an OCDS application reset
 * restarts the cores but leaves the SCU alone, so the PLL stays where the
 * application put it and the same loop is a third as long.  It was measured
 * short - page 1 programmed, then page 2's first command cycle landed on a
 * DMU still busy with page 1 and took a store bus error, DEADD 0xAF005554.
 *
 * 16000 is about 267 us at 300 MHz and 1.07 ms at the backup clock, so it is
 * over the page time either way.  Over a 700 KB image it costs about six
 * seconds.
 */
#define PROGRAM_SETTLE 16000u

/*
 * Where that constant sits in the blob, and what it looks like.
 *
 * The blob is the reference's, byte for byte, so the value cannot come from a
 * rebuild - the TriCore compiler needs a licence server this probe has no
 * business talking to.  Patching the immediate keeps the two provably the same
 * file with one documented difference, rather than a second copy that drifts.
 *
 * MOV (RLC): const16 at bits 27:12, destination register at 31:28, opcode 0x3B.
 * The guard below refuses to run if that word is not the MOV of 4000 into D4
 * this expects, so a regenerated blob that moved the instruction fails loudly
 * instead of having four bytes of its code overwritten.
 *
 * Checked against the toolchain rather than reasoned about: building the
 * reference source with SETTLE=16000 gives a blob that differs from the
 * committed one in exactly two bytes, at offsets 98 and 99, FA 40 -> E8 43 -
 * which is what this writes.
 */
#define SETTLE_MOV_OFFSET 96u
#define SETTLE_MOV_WORD(c) ((4u << 28) | (((c) & 0xFFFFu) << 12) | 0x3Bu)


/* EVTA = halt.  Without it the stub's `debug` instruction is a no-op, and a
 * halt request through DBGSR is ignored while it runs from scratchpad - the
 * core would run on with no way to stop it short of a reset. */
#define SWEVT_HALT 0x2u
/*
 * SWEVT is at 0xFD10, the same offset tricore.c uses.
 *
 * This said 0xFD08 for a while, which is a different register entirely: the
 * stub's `debug` instruction was never armed to halt, and a 2 was being written
 * into whatever lives at 0xFD08.  Copied from the wrong place rather than taken
 * from the one file in this component that already knows.
 */
#define OFF_SWEVT  0xFD10u
#define CPU0_BASE  0xF8810000u

/*
 * Which trigger line to halt with.
 *
 * The same one tricore_bmp uses.  Line 0 is not interchangeable - halting with
 * it simply does not stop the core here, which presents as "CPU0 would not
 * halt" with nothing else wrong, and cost a round of debugging to notice.
 */
#define HALT_LINE 1


bool tricore_flash_installed;

/*
 * Whether the address swap is remapping the bank groups, found in preflight.
 *
 * False means the addresses mean what they say, which is every case except a
 * device running from the other bank group after a UDS update.
 */
bool tricore_flash_swap_active;

/* What preflight found in the registers it turns off, to put back afterwards.
 * While the prefetch buffers are disabled, reads through the cached flash alias
 * fault, so leaving them off makes the part look broken. */
bool tricore_flash_saved_valid;
uint32_t tricore_flash_saved_flashcon0;
uint32_t tricore_flash_saved_pcontrol;

/* -- the stub ------------------------------------------------------------- */

/*
 * Write a block of bytes to target RAM, by whatever route is fastest.
 *
 * The fabric streams it with client_blockwrite - one command and a FIFO's worth
 * of parcels, with the host out of the loop - and falls back to word writes if
 * that is not available, which costs two frames a word and is the difference
 * between seconds and minutes over a 700 kB image.
 */
esp_err_t tricore_flash_write_block(uint32_t address, const uint8_t *data, size_t len)
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

        esp_err_t err = (tricore_flash_use_blockwrite && dap_phy_fpga_ready())
            ? dap_phy_fpga_block_write(address + offset, words, n)
            : ESP_ERR_NOT_SUPPORTED;

        /*
         * Put the first word back.
         *
         * Every block write loses its first parcel: it is written, as zero, at
         * the command address, and every word after it lands correctly.  That
         * is independent of the clock from 24 MHz down to 2 MHz, of whether the
         * address arrives inline or through a separate IO_SET_ADDRESS, and of
         * anything this end does - the parcel is assembled correctly, exactly
         * one word leaves the FIFO for it, and the device acknowledges it.
         *
         * Prepending a dummy word absorbs it, which is how it was confirmed,
         * but that shifts the whole block and the sacrificial write then lands
         * on the word before the destination - which for a contiguous image is
         * the last word of the previous chunk.  Repairing the one word
         * afterwards costs a single frame per block and touches nothing else.
         */
        if (err == ESP_OK && n > 0) {
            /*
             * Drain first.  The spec ends a bulk write with a read against a
             * known location to force the buffered bus transactions out, and
             * IO_SUPERVISOR is that read as well as the one that clears any
             * latched error - without it the repair write is refused.
             */
            dap_probe_clear_error_state();
            err = dap_probe_write32(address + offset, words[0]);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "repairing the first word at 0x%08" PRIX32
                              " failed: %s", address + offset,
                         esp_err_to_name(err));
            }
        }

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

/*
 * Bring the link up the way the BMP target does.
 *
 * The flash task runs on its own and cannot assume anything about what the
 * debug session left behind: the fabric attach needs its DAPISC, read/write
 * mode has to be selected, and OCDS has to be on before run control works.
 * Skipping this was the first failure on hardware - the loader install came
 * back with nothing more specific than "could not install", because the very
 * first memory write had nowhere to go.
 */
static esp_err_t link_up(void)
{
    dap_exchange_t x;

    if (dap_phy_fpga_attach() != ESP_OK) {
        dap_phy_fpga_use(false);
        if (dap_probe_attach(&x, 3) != ESP_OK || x.reply != 0xAAAAAAAAu) {
            ESP_LOGE(TAG, "the target did not answer sync");
            return ESP_ERR_INVALID_STATE;
        }
        dap_probe_client_set(1, &x);
    }

    dap_probe_clear_error_state();
    dap_probe_set_rw_mode(true);

    if (dap_probe_enable_ocds() != ESP_OK) {
        ESP_LOGE(TAG, "OCDS did not come up; run control needs it");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

/*
 * Reset the application and stop the cores before any of it runs.
 *
 * For the reason the sibling halt below already gives: nothing may be fetching
 * out of the bank being written, and a reset is a surer way to get there than
 * halting four cores one at a time.  The reference connects with
 * DCO_RESET_AND_HALT for the same effect.
 *
 * Not for the core's context state.  Halt-after-reset stops the cores at the
 * reset vector, before any startup code runs, so FCX and LCX are both zero
 * afterwards - and that is the state the reference runs the stub in too: it
 * resets, halts, and never resumes before handing over.  The stub is a leaf
 * with no call in it, so it never asks for a context save area.
 *
 * Which is why a class 3 trap out of the stub is a mask rather than a cause:
 * with no free list, whatever actually faulted could not be saved, and the FCU
 * trap is what gets reported instead of the real one.  The syndrome registers
 * read in tricore_flash_run_loader() are what say what it was.
 */
bool tricore_flash_reset_and_halt(void)
{
    const bool armed = (tricore_set_halt_after_reset(true) == ESP_OK);

    if (!armed) {
        ESP_LOGW(TAG, "could not arm halt-after-reset; the cores will run on");
    }
    tricore_request_application_reset();

    /* The OCDS reset leaves the link up, but clearing the error state and
     * re-attaching costs little and covers a heavier one. */
    bool back = false;
    for (int attempt = 0; attempt < 20 && !back; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(25));
        uint32_t dbgsr = 0;
        if (tricore_dbgsr(0, &dbgsr) == ESP_OK) {
            back = true;
            break;
        }
        dap_probe_clear_error_state();
        link_up();
    }
    if (!back) {
        ESP_LOGE(TAG, "the target did not come back after the OCDS reset");
        if (armed) {
            tricore_set_halt_after_reset(false);
        }
        return false;
    }

    /* Serve the request once and clear it, or the next application reset -
     * including one the application asks for itself - stops the cores with no
     * debugger expecting it. */
    if (armed) {
        tricore_set_halt_after_reset(false);
    }
    /* A halt-after-reset trigger left on TR0 re-halts the core on every
     * resume, which would stop the stub on its first instruction. */
    tricore_disarm_reset_trigger();

    /*
     * What the clock is doing, because the stub's pacing depends on it.
     *
     * program_page() reads no DMU status - on this device a status read from
     * the core hangs it while an operation is in flight - and paces itself with
     * a spin loop sized for about 200 us at the 100 MHz backup clock.  An OCDS
     * application reset restarts the cores but leaves the SCU alone, so the PLL
     * stays where the application put it; at 300 MHz the same loop is a third
     * as long and lands the next page's first command on a flash that is still
     * busy.  DAS's reset takes the whole device down to the backup clock, which
     * is the difference this logs.
     */
    uint32_t pllstat = 0, ccucon0 = 0;
    dap_probe_read32(SCU_PLLSTAT, &pllstat);
    dap_probe_read32(SCU_CCUCON0, &ccucon0);
    ESP_LOGI(TAG, "after the reset: PLLSTAT 0x%08" PRIX32 " CCUCON0 0x%08"
                  PRIX32, pllstat, ccucon0);

    tricore_discover();
    return true;
}

esp_err_t tricore_flash_install_loader(void)
{
    if (tricore_flash_installed) {
        return ESP_OK;
    }
    if (link_up() != ESP_OK) {
        tricore_flash_set_phase(TRICORE_FLASH_FAILED, "the debug link would not come up");
        return ESP_FAIL;
    }

    /* Start from the reset vector, so the stub inherits a context save area
     * list the startup software built rather than one a task was using. */
    if (!tricore_flash_reset_and_halt()) {
        tricore_flash_set_phase(TRICORE_FLASH_FAILED,
                  "the target would not reset for programming");
        return ESP_FAIL;
    }

    /*
     * Every core, not just the one the stub runs on.
     *
     * The stub executes from scratchpad precisely so the code being run is not
     * in the bank being written - but that only helps if nothing else is
     * fetching from it.  A sibling core still executing the application reads
     * program flash continuously, which is a read-while-write conflict against
     * the bank being programmed, and after the erase its code is not there at
     * all.  CPU0 halted and CPU2 running is enough to make every page program
     * fail, which is exactly what it did.
     */
    for (int core = 1; core < TRICORE_MAX_CORES; core++) {
        if (!tricore_core_present(core)) {
            continue;
        }
        tricore_halt_release(core);
        if (tricore_halt(core, HALT_LINE, 500) != ESP_OK) {
            ESP_LOGW(TAG, "CPU%d would not halt; programming may fail", core);
        }
    }

    /*
     * Release the trigger line before asserting it.
     *
     * Halting drives a trigger line force-active and releases it when the core
     * stops; a halt that failed, or a session that ended mid-halt, leaves it
     * asserted - and then the next assert is not an edge and nothing happens.
     * This is the same leak tricore_halt_release() exists for, and it is why a
     * flash attempt could find CPU0 unhaltable when GDB had just been using it.
     */
    tricore_halt_release(0);
    tricore_clear_debug_events(0);

    if (tricore_halt(0, HALT_LINE, 500) != ESP_OK) {
        tricore_halt_diag(0, "flash loader install");
        tricore_flash_set_phase(TRICORE_FLASH_FAILED, "CPU0 would not halt");
        return ESP_FAIL;
    }

    /* The blob is not a multiple of four, so it is padded to a word; the stub
     * never reads past its own code. */
    static uint8_t padded[(sizeof(LOADER_BLOB) + 3u) & ~3u];
    memset(padded, 0, sizeof(padded));
    memcpy(padded, LOADER_BLOB, sizeof(LOADER_BLOB));

    /* Lengthen the settle window; see PROGRAM_SETTLE. */
    const uint32_t found = (uint32_t)padded[SETTLE_MOV_OFFSET] |
                           ((uint32_t)padded[SETTLE_MOV_OFFSET + 1] << 8) |
                           ((uint32_t)padded[SETTLE_MOV_OFFSET + 2] << 16) |
                           ((uint32_t)padded[SETTLE_MOV_OFFSET + 3] << 24);
    if (found != SETTLE_MOV_WORD(4000u)) {
        ESP_LOGE(TAG, "the loader blob does not hold the settle constant where "
                      "expected (+0x%X reads 0x%08" PRIX32 ")",
                 (unsigned)SETTLE_MOV_OFFSET, found);
        tricore_flash_set_phase(TRICORE_FLASH_FAILED, "the loader blob is not the one this "
                                        "code knows how to patch");
        return ESP_FAIL;
    }
    const uint32_t settle = SETTLE_MOV_WORD(PROGRAM_SETTLE);
    padded[SETTLE_MOV_OFFSET]     = (uint8_t)(settle);
    padded[SETTLE_MOV_OFFSET + 1] = (uint8_t)(settle >> 8);
    padded[SETTLE_MOV_OFFSET + 2] = (uint8_t)(settle >> 16);
    padded[SETTLE_MOV_OFFSET + 3] = (uint8_t)(settle >> 24);

    if (tricore_flash_write_block(LOADER_CODE, padded, sizeof(padded)) != ESP_OK) {
        tricore_flash_set_phase(TRICORE_FLASH_FAILED, "could not write the loader to PSPR");
        return ESP_FAIL;
    }

    /* Read it back: scratchpad that is not there accepts writes and keeps
     * nothing, and the stub would then program whatever was in the buffer. */
    /*
     * The whole blob, word for word, the way the reference does it.
     *
     * Checking only the first word says the write reached somewhere, not that
     * it reached everywhere: scratchpad that is not there accepts writes and
     * keeps nothing, and a stub with a hole in it traps on the instruction in
     * the hole rather than failing to install.
     */
    uint32_t first = 0, want = 0;
    bool intact = true;

    for (size_t i = 0; i < sizeof(padded); i += 4) {
        const uint32_t expect = (uint32_t)padded[i] |
                                ((uint32_t)padded[i + 1] << 8) |
                                ((uint32_t)padded[i + 2] << 16) |
                                ((uint32_t)padded[i + 3] << 24);
        uint32_t got = 0;
        if (dap_probe_read32(LOADER_CODE + i, &got) != ESP_OK || got != expect) {
            first = got;
            want = expect;
            intact = false;
            ESP_LOGE(TAG, "loader differs at +0x%02X: 0x%08" PRIX32 " not 0x%08"
                          PRIX32, (unsigned)i, got, expect);
            break;
        }
    }
    if (!intact) {
        ESP_LOGE(TAG, "the loader did not stay in scratchpad at 0x%08X "
                      "(read 0x%08" PRIX32 ")", LOADER_CODE, first);
        char why[96];
        snprintf(why, sizeof(why),
                 "loader readback 0x%08" PRIX32 ", expected 0x%08" PRIX32
                 " (%s)", first, want,
                 tricore_flash_use_blockwrite ? "block write" : "word writes");
        tricore_flash_set_phase(TRICORE_FLASH_FAILED, why);
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
    /* Keep what is about to be turned off, so it can be put back. */
    if (dap_probe_read32(CPU0_FLASHCON0, &tricore_flash_saved_flashcon0) == ESP_OK &&
        dap_probe_read32(DMU_HF_PCONTROL, &tricore_flash_saved_pcontrol) == ESP_OK) {
        tricore_flash_saved_valid = true;
    }
    dap_probe_write32(CPU0_FLASHCON0, FLASHCON0_NO_PREFETCH);
    dap_probe_write32(DMU_HF_PCONTROL, PCONTROL_DEMAND);

    tricore_flash_check_swap();

    if (tricore_flash_clear_safety_endinit() != ESP_OK) {
        tricore_flash_set_phase(TRICORE_FLASH_FAILED, "the safety ENDINIT would not clear");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "loader in place at 0x%08X (%u bytes)", LOADER_CODE,
             (unsigned)sizeof(LOADER_BLOB));
    tricore_flash_installed = true;
    return ESP_OK;
}

/*
 * Run one command on the stub and wait for it to halt itself.
 *
 * The stub writes its status and stops with a `debug` instruction, so waiting
 * for the halt is the completion signal - ordinary memory reads work while the
 * core runs, register reads do not.
 */
esp_err_t tricore_flash_run_loader(uint32_t cmd, uint32_t address, uint32_t count,
                            uint32_t timeout_ms, uint32_t *checksum)
{
    if (tricore_flash_install_loader() != ESP_OK) {
        return ESP_FAIL;
    }

    uint32_t params[8] = { cmd, address, count, LOADER_BUFFER, 0, 0, 0, 0 };
    if (tricore_flash_write_block(LOADER_PARAMS, (const uint8_t *)params,
                    sizeof(params)) != ESP_OK) {
        return ESP_FAIL;
    }

    /* A leaf routine still needs a stack pointer, and A11 has to be something
     * valid even though the stub never returns. */
    if (tricore_write_reg(0, 16 + 10, LOADER_STACK) != ESP_OK ||
        tricore_write_reg(0, 16 + 11, LOADER_CODE) != ESP_OK ||
        tricore_write_pc(0, LOADER_CODE) != ESP_OK) {
        ESP_LOGE(TAG, "could not set up the loader's registers");
        tricore_flash_set_phase(TRICORE_FLASH_FAILED, "could not set the loader's PC");
        return ESP_FAIL;
    }

    /*
     * Confirm the PC took before resuming.
     *
     * A resume that starts at the application's PC instead of the stub's runs
     * the application with its flash half erased, and reports back as a stub
     * that "never finished" with a PC somewhere in flash - which is a much
     * harder thing to read than this check failing.
     */
    uint32_t pc_set = 0;
    if (tricore_read_pc(0, &pc_set) != ESP_OK || pc_set != LOADER_CODE) {
        char why[96];
        snprintf(why, sizeof(why),
                 "PC would not take: reads 0x%08" PRIX32 ", wanted 0x%08X",
                 pc_set, LOADER_CODE);
        ESP_LOGE(TAG, "%s", why);
        tricore_flash_set_phase(TRICORE_FLASH_FAILED, why);
        return ESP_FAIL;
    }

    if (tricore_request_resume(0) != ESP_OK) {
        ESP_LOGE(TAG, "could not resume into the loader");
        tricore_flash_set_phase(TRICORE_FLASH_FAILED, "could not resume into the loader");
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
    tricore_halt(0, HALT_LINE, 200);

    /* Where it stopped says whether it is still in its own code or took a trap
     * - a trap at a flash address means the range it was reading is erased. */
    uint32_t pc_after = 0;
    tricore_read_pc(0, &pc_after);

    uint32_t out[8] = {0};
    for (int i = 0; i < 8; i++) {
        if (dap_probe_read32(LOADER_PARAMS + 4u * i, &out[i]) != ESP_OK) {
            ESP_LOGE(TAG, "could not read the loader's result");
            return ESP_FAIL;
        }
    }

    /*
     * Report what the stub said, not just that it said something wrong.
     *
     * status, ERRSR and how far it got are three different failures - a trap in
     * the stub, a flash that refused the page, and a stub that never ran - and
     * they need different answers.  The console is not always reachable here,
     * so this goes into the status the web page shows.
     */
    if (out[4] != LOADER_ST_OK) {
        /*
         * Where it stopped, and with what context state.
         *
         * A PC inside the stub means it ran out of time; a PC in flash means it
         * took a trap and landed on the application's vector table, and then
         * PCXI/FCX say whether the trap was a context-management one - which is
         * what a resume into a stub hits when the core's free CSA list is not
         * in a state the trap handler can use.
         */
        uint32_t pcxi = 0, psw = 0, fcx = 0, lcx = 0, btv = 0, icr = 0;
        dap_probe_read32(CPU0_BASE + 0xFE00u, &pcxi);
        dap_probe_read32(CPU0_BASE + 0xFE04u, &psw);
        dap_probe_read32(CPU0_BASE + 0xFE38u, &fcx);
        dap_probe_read32(CPU0_BASE + 0xFE3Cu, &lcx);
        dap_probe_read32(CPU0_BASE + 0xFE24u, &btv);
        dap_probe_read32(CPU0_BASE + 0xFE2Cu, &icr);
        ESP_LOGE(TAG, "stub stopped at PC 0x%08" PRIX32 ": PCXI 0x%08" PRIX32
                      " PSW 0x%08" PRIX32 " FCX 0x%08" PRIX32 " LCX 0x%08"
                      PRIX32 " BTV 0x%08" PRIX32 " ICR 0x%08" PRIX32,
                 pc_after, pcxi, psw, fcx, lcx, btv, icr);

        /*
         * Whether the part reset under us, and if so what asked for it.
         *
         * A trap and a watchdog reset both end with the PC on a vector in
         * flash, and they need opposite fixes - one is the core's context
         * state, the other is a timer nobody is serving while the stub runs.
         * RSTSTAT names the source, and a safety ENDINIT that is set again is
         * the tell that a reset happened at all: this code cleared it, and only
         * a reset puts it back.
         */
        /*
         * The trap syndrome, which needs no context to survive.
         *
         * DEADD carries the address that faulted, which is the one thing that
         * ends the guessing: a data-side fault on the command interface says
         * the stub wrote the CSI while the flash was still busy with the page
         * before it, and an instruction-side one says something else entirely.
         */
        uint32_t dstr = 0, datr = 0, deadd = 0, pstr = 0;
        dap_probe_read32(CPU0_BASE + 0x9010u, &dstr);
        dap_probe_read32(CPU0_BASE + 0x9018u, &datr);
        dap_probe_read32(CPU0_BASE + 0x901Cu, &deadd);
        dap_probe_read32(CPU0_BASE + 0x9200u, &pstr);
        ESP_LOGE(TAG, "DSTR 0x%08" PRIX32 " DATR 0x%08" PRIX32 " DEADD 0x%08"
                      PRIX32 " PSTR 0x%08" PRIX32, dstr, datr, deadd, pstr);

        uint32_t rststat = 0, wdts = 0, wdtcpu0 = 0;
        dap_probe_read32(SCU_RSTSTAT, &rststat);
        dap_probe_read32(SCU_WDTS_CON0, &wdts);
        dap_probe_read32(WDTCPU0_CON0, &wdtcpu0);
        ESP_LOGE(TAG, "RSTSTAT 0x%08" PRIX32 " WDTS_CON0 0x%08" PRIX32
                      " (ENDINIT %s) WDTCPU0_CON0 0x%08" PRIX32,
                 rststat, wdts,
                 (wdts & 1u) ? "BACK - the part reset" : "still clear",
                 wdtcpu0);

        char why[110];
        snprintf(why, sizeof(why),
                 "loader status %" PRIu32 " at 0x%08" PRIX32 ", %" PRIu32 "/%"
                 PRIu32 " done, ERRSR 0x%08" PRIX32 ", PC 0x%08" PRIX32,
                 out[4], address, out[6], count, out[5], pc_after);
        tricore_flash_status.errsr = out[5];
        ESP_LOGE(TAG, "%s", why);
        tricore_flash_set_phase(TRICORE_FLASH_FAILED, why);
        return (out[4] == LOADER_ST_RUNNING) ? ESP_ERR_TIMEOUT : ESP_FAIL;
    }
    if (checksum) {
        *checksum = out[7];
    }
    return tricore_flash_wait_idle(address, 2000);
}
