/*
 * The RAM loader: a stub in CPU0's scratchpad that issues the flash command
 * sequences, because the DMU only accepts them from the core.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "dap_frame.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tricore.h"
#include "tricore_flash.h"
#include "tricore_flash_priv.h"

static const char *TAG = "TRICORE_FLASH";

/* tas-debug's loader/tricore_flash_loader.c, byte for byte (loader_blob.py). */
static const uint8_t LOADER_BLOB[] = {
    0x82,0x0F,0xA5,0x7F,0x10,0x00,0xA5,0x7F,0x18,0x00,0xA5,0x7F,0x14,0x00,0xA5,0x7F,
    0x30,0x00,0xA5,0x7F,0x34,0x00,0x85,0x7F,0x00,0x00,0x20,0x08,0xDF,0x1F,0xAC,0x80,
    0x85,0x73,0x0C,0x08,0x91,0x00,0xF0,0x2A,0x85,0x77,0x04,0x00,0x91,0x00,0xF0,0x6A,
    0x91,0x10,0xF0,0x7A,0x91,0x10,0xF0,0xCA,0x91,0x10,0xF0,0x4A,0x91,0x40,0x80,0x5F,
    0x91,0x40,0x80,0xDF,0x85,0x76,0x08,0x00,0x3B,0x80,0x00,0x90,0xD9,0x22,0x54,0x55,
    0x3B,0xA0,0x0F,0x00,0x3B,0x00,0x05,0xA0,0xD9,0x66,0x70,0x75,0xD9,0x77,0x90,0x9A,
    0xD9,0xCC,0x98,0x9A,0x82,0x0B,0xD9,0x44,0xA8,0xAA,0x3B,0x00,0x0A,0xC0,0x3B,0x60,
    0x0A,0xD0,0xD9,0x55,0x10,0x00,0xD9,0xDD,0x34,0x00,0x3B,0x00,0xFA,0x10,0xDF,0x06,
    0xA2,0x00,0x85,0x7F,0x20,0x00,0x82,0x14,0x2E,0x09,0x02,0x7F,0x16,0xFF,0x8B,0x86,
    0xA0,0x42,0x8B,0x0F,0x00,0x44,0xAB,0x19,0x80,0x44,0x74,0x20,0x0D,0x00,0x80,0x04,
    0x74,0x2A,0x0D,0x00,0x80,0x04,0x8F,0x54,0x00,0x80,0x40,0x3F,0x01,0x38,0x00,0x36,
    0x01,0xF3,0x20,0xE0,0x80,0xE5,0x06,0xD5,0x9F,0x05,0x03,0x80,0x3C,0x08,0x09,0xF2,
    0x48,0x01,0x89,0x62,0x40,0x09,0x0D,0x00,0x80,0x04,0x3C,0xF7,0x74,0x77,0x74,0xCB,
    0x8B,0x84,0x00,0x52,0x74,0x4C,0xAB,0xAD,0x8A,0xF5,0x6C,0x40,0x0D,0x00,0x80,0x04,
    0x8B,0x84,0x20,0xF2,0xEE,0x04,0x85,0x73,0x28,0x00,0x3C,0x03,0x85,0x73,0x24,0x00,
    0x85,0xF2,0x10,0x01,0x85,0x7F,0x20,0x00,0x6F,0x1F,0x18,0x80,0xF6,0x39,0x59,0xA1,
    0x04,0x00,0x58,0x01,0x6E,0x70,0x58,0x01,0xC2,0xFF,0x78,0x01,0x3C,0xFB,0x85,0xFF,
    0x10,0x01,0xA2,0x2F,0x3F,0x3F,0xFD,0xFF,0x3C,0x66,0x85,0xFF,0x10,0x01,0x85,0x7E,
    0x2C,0x00,0xA2,0x2F,0x3F,0xFE,0x1B,0x80,0x4C,0x50,0x16,0x3F,0x6E,0xF7,0x4C,0x50,
    0x16,0x3F,0x6E,0x08,0x85,0xFF,0x10,0x01,0xA2,0x2F,0x7F,0xF3,0xFA,0xFF,0x82,0x3F,
    0x3C,0x42,0x85,0xFF,0x10,0x01,0x52,0x22,0x85,0x7F,0x34,0x00,0x7F,0x2F,0x04,0x80,
    0xA5,0x72,0x34,0x00,0x4C,0xD0,0x16,0x1F,0x6E,0x0A,0x91,0x40,0x80,0xFF,0xD9,0xFF,
    0x34,0x00,0x4C,0xF0,0xA5,0x7F,0x14,0x00,0x82,0x2F,0x3C,0x2D,0x74,0x20,0x0D,0x00,
    0x80,0x04,0x3C,0x39,0x85,0x72,0x00,0x00,0x82,0x4F,0xDF,0x32,0x25,0x80,0x85,0x74,
    0x04,0x08,0x85,0x7F,0x08,0x08,0x7B,0x90,0xDB,0x3E,0xA0,0x03,0x82,0xFF,0x1B,0x03,
    0x32,0x38,0xFD,0xF0,0x0A,0x00,0x46,0x0F,0xA5,0x7F,0x1C,0x00,0x85,0x7F,0x08,0x00,
    0xA5,0x7F,0x18,0x00,0x3C,0x0F,0x01,0x43,0x10,0x20,0x14,0x22,0xA0,0x72,0xC6,0x2F,
    0x8F,0xFF,0x1F,0x20,0x37,0x0F,0x41,0xF0,0x26,0x3F,0xC6,0x2F,0xFC,0x2A,0xB0,0x13,
    0x3C,0xE9,0x82,0x1F,0x85,0x72,0x14,0x00,0xF6,0x28,0x91,0x40,0x80,0xFF,0xD9,0xFF,
    0x34,0x00,0x48,0x02,0xA5,0x72,0x14,0x00,0xA5,0x7F,0x10,0x00,0x0D,0x00,0x80,0x04,
    0x00,0xA0,0x3C,0x00,0x85,0x7F,0x08,0x00,0xA2,0x46,0xA2,0x6F,0xA5,0x7F,0x18,0x00,
    0x42,0x87,0xDF,0x05,0x46,0x7F,0x85,0x7F,0x30,0x00,0xC2,0x1F,0xA5,0x7F,0x30,0x00,
    0x1D,0xFF,0x3F,0xFF,
};

/*
 * The stub programs 256-byte bursts where the address allows (pages for the
 * rest) and, with LOADER_FLAG_POLL, polls DMU_HF_STATUS until the bank is idle,
 * timed out in System Timer ticks - so its pace follows the flash, not a loop
 * count.  The STM's frequency depends on the clock tree the reset left (our
 * OCDS reset keeps the application's PLL), so it is measured at install.
 *
 * Only if that fails does the stub fall back to its spin loop of
 * PROGRAM_SETTLE iterations per page.  The blob is built with 4000, sized for
 * the 100 MHz backup clock a DAS reset leaves; at our 300 MHz that is too short
 * (page 2 meets a busy interpreter), so the constant is patched to 12000 - the
 * reference's ~200 us window - wherever the `mov dN, #4000` sits in the blob.
 */
#define PROGRAM_SETTLE     12000u
#define SETTLE_MOV_MASK    0x0FFFFFFFu                  /* all but the register */
#define SETTLE_MOV_WORD(c) ((((c) & 0xFFFFu) << 12) | 0x3Bu)

/* System Timer 0, lower 32 bits, and its OCDS control (SUS: suspend while the
 * core is halted; SUS_P: write-enable for SUS). */
#define STM0_TIM0          0xF0001010u
#define STM0_OCS           0xF00010E8u
#define STM_OCS_SUS        (0xFu << 24)
#define STM_OCS_SUS_P      (1u << 28)
/* The TC38x data sheet's longest program times at 3.3 V (tPRP3, tPRPB3);
 * polling uses POLL_FACTOR times them as the time-out. */
#define PAGE_MAX_US        115u
#define BURST_MAX_US       530u
#define POLL_FACTOR        8u
/* How long BUSY may take to come up after the last command cycle. */
#define POLL_START_US      20u

static uint32_t s_stm_hz;

/* SWEVT = halt, so the stub's `debug` instruction stops the core. */
#define CPU0_BASE   0xF8810000u
#define OFF_SWEVT   0xFD10u
#define SWEVT_HALT  0x2u

/* The trigger line tricore_bmp halts with; line 0 does not stop the core. */
#define HALT_LINE   1

bool tricore_flash_installed;
bool tricore_flash_swap_active;
bool tricore_flash_saved_valid;
uint32_t tricore_flash_saved_flashcon0;
uint32_t tricore_flash_saved_pcontrol;

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static esp_err_t fail(const char *why)
{
    ESP_LOGE(TAG, "%s", why);
    tricore_flash_set_phase(TRICORE_FLASH_FAILED, why);
    return ESP_FAIL;
}

/*
 * Bulk write to target RAM through client_blockwrite, falling back to word
 * writes when the fabric is not loaded.  Every block write loses its first
 * parcel (written as zero, the rest land correctly), so that word is written
 * again - after an IO_SUPERVISOR read, which drains the bus and without which
 * the repair is refused.
 */
esp_err_t tricore_flash_write_block(uint32_t address, const uint8_t *data,
                                    size_t len)
{
    static uint32_t words[DAP_FPGA_BLOCK_WORDS];

    if (len % 4u) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t off = 0; off < len; ) {
        const size_t chunk = (len - off > sizeof(words)) ? sizeof(words) : len - off;
        const size_t n = chunk / 4u;

        for (size_t i = 0; i < n; i++) {
            words[i] = le32(data + off + 4u * i);
        }

        esp_err_t err = (tricore_flash_use_blockwrite && dap_phy_fpga_ready())
            ? dap_phy_fpga_block_write(address + off, words, n)
            : ESP_ERR_NOT_SUPPORTED;

        if (err == ESP_OK) {
            dap_probe_clear_error_state();
            err = dap_probe_write32(address + off, words[0]);
        } else if (err == ESP_ERR_NOT_SUPPORTED) {
            err = ESP_OK;
            for (size_t i = 0; i < n && err == ESP_OK; i++) {
                err = dap_probe_write32(address + off + 4u * i, words[i]);
            }
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "writing 0x%08" PRIX32 " failed: %s",
                     (uint32_t)(address + off), esp_err_to_name(err));
            return err;
        }
        off += chunk;
    }
    return ESP_OK;
}

/* Attach, read/write mode, OCDS - the flash task cannot assume a live session. */
static esp_err_t link_up(void)
{
    dap_exchange_t x;

    if (dap_phy_fpga_attach() != ESP_OK) {
        dap_phy_fpga_use(false);
        if (dap_probe_attach(&x, 3) != ESP_OK || x.reply != 0xAAAAAAAAu) {
            return ESP_ERR_INVALID_STATE;
        }
        dap_probe_client_set(1, &x);
    }
    dap_probe_clear_error_state();
    dap_probe_set_rw_mode(true);
    return dap_probe_enable_ocds();
}

/*
 * Application reset with halt-after-reset armed, so no core runs application
 * code - the equivalent of DAS's DCO_RESET_AND_HALT.  The request is cleared
 * straight after, so a later reset the application asks for runs normally.
 */
bool tricore_flash_reset_and_halt(void)
{
    const bool armed = (tricore_set_halt_after_reset(true) == ESP_OK);

    tricore_request_application_reset();

    bool back = false;
    for (int attempt = 0; attempt < 20 && !back; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(25));
        uint32_t dbgsr = 0;
        back = (tricore_dbgsr(0, &dbgsr) == ESP_OK);
        if (!back) {
            dap_probe_clear_error_state();
            link_up();
        }
    }
    if (armed) {
        tricore_set_halt_after_reset(false);
    }
    if (!back) {
        ESP_LOGE(TAG, "the target did not come back after the reset");
        return false;
    }
    tricore_disarm_reset_trigger();
    tricore_discover();
    return true;
}

/* Halt every running core: a sibling fetching from flash while it is written
 * is a read-while-write conflict.  Never-started cores are already stopped. */
static esp_err_t halt_all(void)
{
    for (int core = 1; core < TRICORE_MAX_CORES; core++) {
        if (!tricore_core_present(core) || !tricore_core_started(core)) {
            continue;
        }
        tricore_halt_release(core);
        if (tricore_halt(core, HALT_LINE, 500) != ESP_OK) {
            ESP_LOGW(TAG, "CPU%d would not halt; programming may fail", core);
        }
    }
    tricore_halt_release(0);
    tricore_clear_debug_events(0);
    if (tricore_halt(0, HALT_LINE, 500) != ESP_OK) {
        tricore_halt_diag(0, "flash loader install");
        return fail("CPU0 would not halt");
    }
    return ESP_OK;
}

/* Copy the blob and patch the fallback settle constant; refuse a blob in
 * which it is not found exactly once. */
static esp_err_t load_blob(void)
{
    static uint8_t blob[(sizeof(LOADER_BLOB) + 3u) & ~3u];
    int at = -1;

    memset(blob, 0, sizeof(blob));
    memcpy(blob, LOADER_BLOB, sizeof(LOADER_BLOB));
    for (size_t off = 0; off + 4u <= sizeof(LOADER_BLOB); off += 2u) {   /* 16-bit aligned code */
        if ((le32(blob + off) & SETTLE_MOV_MASK) == SETTLE_MOV_WORD(4000u)) {
            if (at >= 0) {
                return fail("loader blob: settle constant found twice");
            }
            at = (int)off;
        }
    }
    if (at < 0) {
        return fail("loader blob: settle constant not found");
    }
    const uint32_t settle = (le32(blob + at) & ~SETTLE_MOV_MASK) | SETTLE_MOV_WORD(PROGRAM_SETTLE);
    memcpy(blob + at, &settle, 4);   /* little-endian host */

    if (tricore_flash_write_block(LOADER_CODE, blob, sizeof(blob)) != ESP_OK) {
        return fail("could not write the loader to PSPR");
    }
    /* Word-for-word read-back: missing scratchpad accepts writes silently. */
    for (size_t i = 0; i < sizeof(blob); i += 4) {
        uint32_t got = 0;
        if (dap_probe_read32(LOADER_CODE + i, &got) != ESP_OK ||
            got != le32(blob + i)) {
            char why[96];
            snprintf(why, sizeof(why), "loader readback +0x%02X: 0x%08" PRIX32
                     " not 0x%08" PRIX32, (unsigned)i, got, le32(blob + i));
            return fail(why);
        }
    }
    return ESP_OK;
}

/* The System Timer's frequency, from two reads 20 ms apart timed here; 0 if
 * it does not count (suspended) or reads nonsense. */
static uint32_t measure_stm_hz(void)
{
    uint32_t a = 0, b = 0, ocs = 0;
    bool ok;

    /* A GDB session suspends STM0 while CPU0 is halted (so compare-based
     * timers survive a halt), and a session that was killed leaves it so -
     * across our application reset too.  The core is halted now, so lift that
     * for the measurement; the stub itself runs, so it never sees it frozen. */
    const bool suspended = dap_probe_read32(STM0_OCS, &ocs) == ESP_OK && (ocs & STM_OCS_SUS);
    if (suspended) {
        dap_probe_write32(STM0_OCS, STM_OCS_SUS_P);
    }
    int64_t t0 = esp_timer_get_time();
    ok = dap_probe_read32(STM0_TIM0, &a) == ESP_OK;
    t0 = (t0 + esp_timer_get_time()) / 2;
    vTaskDelay(pdMS_TO_TICKS(20));
    int64_t t1 = esp_timer_get_time();
    ok = ok && dap_probe_read32(STM0_TIM0, &b) == ESP_OK;
    t1 = (t1 + esp_timer_get_time()) / 2;
    if (suspended) {
        dap_probe_write32(STM0_OCS, STM_OCS_SUS_P | (ocs & STM_OCS_SUS));
    }
    if (!ok) {
        return 0;
    }
    const uint64_t hz = (uint64_t)(uint32_t)(b - a) * 1000000u / (uint64_t)(t1 - t0);
    return (hz >= 1000000u && hz <= 500000000u) ? (uint32_t)hz : 0u;
}

static uint32_t ticks_of(uint32_t us)
{
    const uint64_t t = (uint64_t)s_stm_hz * us / 1000000u;
    return s_stm_hz ? (uint32_t)(t ? t : 1u) : 0u;
}

esp_err_t tricore_flash_install_loader(void)
{
    if (tricore_flash_installed) {
        return ESP_OK;
    }
    if (link_up() != ESP_OK) {
        return fail("the debug link would not come up");
    }
    if (!tricore_flash_reset_and_halt()) {
        return fail("the target would not reset for programming");
    }
    if (halt_all() != ESP_OK || load_blob() != ESP_OK) {
        return ESP_FAIL;
    }
    if (dap_probe_write32(CPU0_BASE + OFF_SWEVT, SWEVT_HALT) != ESP_OK) {
        return fail("could not arm the stub's self-halt");
    }

    /* Prefetch off and demand mode, so nothing reads the bank being written. */
    tricore_flash_saved_valid =
        dap_probe_read32(CPU0_FLASHCON0, &tricore_flash_saved_flashcon0) == ESP_OK &&
        dap_probe_read32(DMU_HF_PCONTROL, &tricore_flash_saved_pcontrol) == ESP_OK;
    dap_probe_write32(CPU0_FLASHCON0, FLASHCON0_NO_PREFETCH);
    dap_probe_write32(DMU_HF_PCONTROL, PCONTROL_DEMAND);

    tricore_flash_check_swap();
    if (tricore_flash_clear_safety_endinit() != ESP_OK) {
        return fail("the safety ENDINIT would not clear");
    }
    s_stm_hz = measure_stm_hz();
    if (s_stm_hz) {
        ESP_LOGI(TAG, "loader in place at 0x%08X; STM at %" PRIu32 ".%02" PRIu32
                      " MHz: bursts, polling the flash status", LOADER_CODE,
                 s_stm_hz / 1000000u, (s_stm_hz / 10000u) % 100u);
    } else {
        ESP_LOGW(TAG, "loader in place at 0x%08X; the STM does not count, so "
                      "pages with a fixed spin-loop wait", LOADER_CODE);
    }
    tricore_flash_installed = true;
    return ESP_OK;
}

/* On failure: where it stopped, and the trap syndrome, which survives even
 * with no free CSA list (DEADD names the faulting address). */
static void dump_stop(uint32_t pc)
{
    uint32_t fcx = 0, btv = 0, datr = 0, deadd = 0, pstr = 0;

    dap_probe_read32(CPU0_BASE + 0xFE38u, &fcx);
    dap_probe_read32(CPU0_BASE + 0xFE24u, &btv);
    dap_probe_read32(CPU0_BASE + 0x9018u, &datr);
    dap_probe_read32(CPU0_BASE + 0x901Cu, &deadd);
    dap_probe_read32(CPU0_BASE + 0x9200u, &pstr);
    ESP_LOGE(TAG, "stub stopped at PC 0x%08" PRIX32 " (BTV 0x%08" PRIX32
                  ", FCX 0x%08" PRIX32 "): DATR 0x%08" PRIX32 " DEADD 0x%08"
                  PRIX32 " PSTR 0x%08" PRIX32, pc, btv, fcx, datr, deadd, pstr);
}

/* Run one stub command; the stub halts itself with `debug` when done. */
esp_err_t tricore_flash_run_loader(uint32_t cmd, uint32_t address,
                                   uint32_t count, uint32_t timeout_ms,
                                   uint32_t *checksum)
{
    if (tricore_flash_install_loader() != ESP_OK) {
        return ESP_FAIL;
    }

    /* struct params in tas-debug's loader source: cmd, address, count, buffer,
     * status, errsr, done, checksum, flags, wait/burst/start ticks, bursts, max. */
    const bool poll = s_stm_hz != 0u;
    const uint32_t factor = poll ? POLL_FACTOR : 1u;
    const uint32_t params[LOADER_PARAM_WORDS] = {
        cmd, address, count, LOADER_BUFFER, 0, 0, 0, 0,
        poll ? (LOADER_FLAG_BURST | LOADER_FLAG_POLL) : 0u,
        ticks_of(PAGE_MAX_US * factor), ticks_of(BURST_MAX_US * factor),
        ticks_of(POLL_START_US), 0, 0,
    };
    if (tricore_flash_write_block(LOADER_PARAMS, (const uint8_t *)params,
                                  sizeof(params)) != ESP_OK) {
        return fail("could not write the loader parameters");
    }

    /* A10 = stack, A11 must be valid even though the stub never returns. */
    uint32_t pc = 0;
    if (tricore_write_reg(0, 16 + 10, LOADER_STACK) != ESP_OK ||
        tricore_write_reg(0, 16 + 11, LOADER_CODE) != ESP_OK ||
        tricore_write_pc(0, LOADER_CODE) != ESP_OK ||
        tricore_read_pc(0, &pc) != ESP_OK || pc != LOADER_CODE) {
        return fail("could not point CPU0 at the loader");
    }
    if (tricore_request_resume(0) != ESP_OK) {
        return fail("could not resume into the loader");
    }

    const int64_t started = esp_timer_get_time();
    const int64_t deadline = started + (int64_t)timeout_ms * 1000;
    uint32_t dbgsr = 0;
    while (esp_timer_get_time() < deadline) {
        if (tricore_dbgsr(0, &dbgsr) == ESP_OK && (dbgsr & 0x2u)) {
            break;
        }
        /* Short runs (a sector's checksum takes ~7 ms) are polled closely;
         * a tick is 10 ms, which would double them. */
        if (esp_timer_get_time() - started < 20000) {
            esp_rom_delay_us(250);
        } else {
            vTaskDelay(1);
        }
    }
    tricore_halt(0, HALT_LINE, 200);
    tricore_read_pc(0, &pc);

    uint32_t out[LOADER_PARAM_WORDS] = {0};
    for (int i = 0; i < LOADER_PARAM_WORDS; i++) {
        if (i >= 8 && i < 12) {
            continue;                   /* our own inputs */
        }
        if (dap_probe_read32(LOADER_PARAMS + 4u * i, &out[i]) != ESP_OK) {
            return fail("could not read the loader's result");
        }
    }
    /* out[4] status, out[5] ERRSR, out[6] pages done, out[7] checksum,
     * out[12] bursts, out[13] the longest polled operation in STM ticks */
    if (cmd == LOADER_CMD_PROGRAM) {
        tricore_flash_status.bursts += out[12];
        const uint32_t us = s_stm_hz ? (uint32_t)((uint64_t)out[13] * 1000000u / s_stm_hz) : 0u;
        if (us > tricore_flash_status.op_us_max) {
            tricore_flash_status.op_us_max = us;
        }
    }
    if (out[4] != LOADER_ST_OK) {
        char why[sizeof(tricore_flash_status.message)];

        dump_stop(pc);
        snprintf(why, sizeof(why),
                 "loader status %" PRIu32 " at 0x%08" PRIX32 ", %" PRIu32 "/%"
                 PRIu32 " done, ERRSR 0x%08" PRIX32 ", PC 0x%08" PRIX32,
                 out[4], address, out[6], count, out[5], pc);
        tricore_flash_status.errsr = out[5];
        fail(why);
        return (out[4] == LOADER_ST_RUNNING) ? ESP_ERR_TIMEOUT : ESP_FAIL;
    }
    if (checksum) {
        *checksum = out[7];
    }
    return tricore_flash_wait_idle(address, 2000);
}
