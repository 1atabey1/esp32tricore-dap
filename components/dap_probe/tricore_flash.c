/*
 * Program the TriCore's flash.  See tricore_flash.h for the shape of it.
 */

#include "tricore_flash.h"

#include <stdio.h>
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

static bool s_installed;

/*
 * Whether the address swap is remapping the bank groups, found in preflight.
 *
 * False means the addresses mean what they say, which is every case except a
 * device running from the other bank group after a UDS update.
 */
static bool s_swap_active;

/* What preflight found in the registers it turns off, to put back afterwards.
 * While the prefetch buffers are disabled, reads through the cached flash alias
 * fault, so leaving them off makes the part look broken. */
static bool s_saved_valid;
static uint32_t s_saved_flashcon0;
static uint32_t s_saved_pcontrol;

/*
 * Whether to use client_blockwrite for bulk transfers.
 *
 * Cleared by /api/flash/start?slow=1, which forces the word-at-a-time path.
 * That is the difference between "the fabric's block write is wrong" and
 * "something else is wrong", and it is one request rather than a rebuild.
 */
static bool s_use_blockwrite = true;

void tricore_flash_set_blockwrite(bool enable)
{
    s_use_blockwrite = enable;
}

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
    if (!s_swap_active) {
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
        s_status.errsr = errsr;
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
static esp_err_t clear_safety_endinit(void)
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
static void check_swap(void)
{
    uint32_t active = 0, enabled = 0;

    s_swap_active = false;
    if (dap_probe_read32(SWAP_ACTIVE_REG, &active) != ESP_OK) {
        ESP_LOGW(TAG, "could not read SCU_SWAPCTRL; assuming no swap");
        return;
    }
    dap_probe_read32(SWAP_EN_REG, &enabled);

    if ((active & SWAP_ACTIVE_MASK) == SWAP_ACTIVE_VAL) {
        s_swap_active = true;
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
static void safe_shutdown(void)
{
    clear_status();
    if (s_saved_valid) {
        dap_probe_write32(CPU0_FLASHCON0, s_saved_flashcon0);
        dap_probe_write32(DMU_HF_PCONTROL, s_saved_pcontrol);
    }
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

        esp_err_t err = (s_use_blockwrite && dap_phy_fpga_ready())
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
 * This is not tidiness, it is what makes the stub runnable at all.  The stub is
 * ordinary compiled code: it calls a subroutine, and a TriCore call saves the
 * upper context into a CSA taken from the free list at FCX.  Halting a running
 * application and redirecting its PC inherits whatever that application had
 * left in FCX - and on a target running PXROS that list belongs to the task
 * that happened to be executing, so the first call in the stub takes a context
 * management trap, lands on the application's trap vector at BTV, and the
 * handler traps again until the list is empty.  What comes back is a stub that
 * "never finished" with FCX 0 and a PC in flash.
 *
 * After a reset with halt-after-reset armed, the startup software has built the
 * free list and nothing has consumed it.  The reference does the same thing by
 * connecting with DCO_RESET_AND_HALT before it touches the flash.
 */
static bool reset_and_halt(void)
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

    tricore_discover();
    return true;
}

static esp_err_t install_loader(void)
{
    if (s_installed) {
        return ESP_OK;
    }
    if (link_up() != ESP_OK) {
        set_phase(TRICORE_FLASH_FAILED, "the debug link would not come up");
        return ESP_FAIL;
    }

    /* Start from the reset vector, so the stub inherits a context save area
     * list the startup software built rather than one a task was using. */
    if (!reset_and_halt()) {
        set_phase(TRICORE_FLASH_FAILED,
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
        set_phase(TRICORE_FLASH_FAILED, "CPU0 would not halt");
        return ESP_FAIL;
    }

    /* The blob is not a multiple of four, so it is padded to a word; the stub
     * never reads past its own code. */
    static uint8_t padded[(sizeof(LOADER_BLOB) + 3u) & ~3u];
    memset(padded, 0, sizeof(padded));
    memcpy(padded, LOADER_BLOB, sizeof(LOADER_BLOB));

    if (write_block(LOADER_CODE, padded, sizeof(padded)) != ESP_OK) {
        set_phase(TRICORE_FLASH_FAILED, "could not write the loader to PSPR");
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
                 s_use_blockwrite ? "block write" : "word writes");
        set_phase(TRICORE_FLASH_FAILED, why);
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
    if (dap_probe_read32(CPU0_FLASHCON0, &s_saved_flashcon0) == ESP_OK &&
        dap_probe_read32(DMU_HF_PCONTROL, &s_saved_pcontrol) == ESP_OK) {
        s_saved_valid = true;
    }
    dap_probe_write32(CPU0_FLASHCON0, FLASHCON0_NO_PREFETCH);
    dap_probe_write32(DMU_HF_PCONTROL, PCONTROL_DEMAND);

    check_swap();

    if (clear_safety_endinit() != ESP_OK) {
        set_phase(TRICORE_FLASH_FAILED, "the safety ENDINIT would not clear");
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
        tricore_write_pc(0, LOADER_CODE) != ESP_OK) {
        ESP_LOGE(TAG, "could not set up the loader's registers");
        set_phase(TRICORE_FLASH_FAILED, "could not set the loader's PC");
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
        set_phase(TRICORE_FLASH_FAILED, why);
        return ESP_FAIL;
    }

    if (tricore_request_resume(0) != ESP_OK) {
        ESP_LOGE(TAG, "could not resume into the loader");
        set_phase(TRICORE_FLASH_FAILED, "could not resume into the loader");
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
        s_status.errsr = out[5];
        ESP_LOGE(TAG, "%s", why);
        set_phase(TRICORE_FLASH_FAILED, why);
        return (out[4] == LOADER_ST_RUNNING) ? ESP_ERR_TIMEOUT : ESP_FAIL;
    }
    if (checksum) {
        *checksum = out[7];
    }
    return wait_idle(address, 2000);
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
    if (wait_idle(command, 5000) != ESP_OK) {
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
        s_status.errsr = errsr;
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
        /* install_loader has already said which step it was. */
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
                set_phase(TRICORE_FLASH_FAILED, "too many sectors for one run");
                safe_shutdown();
                return ESP_FAIL;
            }
            sector_list[sectors++] = at;
        }
    }
    s_status.sectors = sectors;

    /* Refuse the whole run before erasing any of it, rather than finding the
     * one bad address half way through with the image already destroyed. */
    for (uint32_t k = 0; k < sectors; k++) {
        if (!is_program_flash(command_address(sector_list[k]))) {
            char why[96];
            snprintf(why, sizeof(why),
                     "0x%08" PRIX32 " is not erasable program flash",
                     sector_list[k]);
            set_phase(TRICORE_FLASH_FAILED, why);
            safe_shutdown();
            return ESP_FAIL;
        }
    }

    set_phase(TRICORE_FLASH_ERASING, "erasing");
    ESP_LOGI(TAG, "erasing %" PRIu32 " sectors from 0x%08" PRIX32,
             sectors, sector_list[0]);
    for (uint32_t k = 0; k < sectors; k++) {
        if (erase_sector(sector_list[k]) != ESP_OK) {
            char why[96];
            snprintf(why, sizeof(why),
                     "erase of 0x%08" PRIX32 " failed, ERRSR 0x%08" PRIX32,
                     sector_list[k], s_status.errsr);
            set_phase(TRICORE_FLASH_FAILED, why);
            safe_shutdown();
            return ESP_FAIL;
        }
        s_status.sectors_done = k + 1u;
        s_status.elapsed_ms =
            (uint32_t)((esp_timer_get_time() - started) / 1000);
    }

    /* -- program, a buffer at a time ------------------------------------- */
    set_phase(TRICORE_FLASH_PROGRAMMING, "programming");

    if (check_page_mode() != ESP_OK) {
        char why[96];
        snprintf(why, sizeof(why),
                 "program flash would not enter page mode, ERRSR 0x%08" PRIX32,
                 s_status.errsr);
        set_phase(TRICORE_FLASH_FAILED, why);
        safe_shutdown();
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

            if (write_block(LOADER_BUFFER, page, padded) != ESP_OK) {
                set_phase(TRICORE_FLASH_FAILED, "could not fill the buffer");
                safe_shutdown();
                return ESP_FAIL;
            }
            /* The stub issues the same command sequences the host would, so
             * what it gets is a command address and it goes through the swap
             * like every other one. */
            if (run_loader(LOADER_CMD_PROGRAM,
                           command_address(r->address + offset),
                           padded / TRICORE_FLASH_PAGE, 20000, NULL) != ESP_OK) {
                /* run_loader has already said what the stub reported; saying
                 * "programming failed" over the top of it threw away the only
                 * useful part. */
                s_status.phase = TRICORE_FLASH_FAILED;
                safe_shutdown();
                return ESP_FAIL;
            }

            offset += chunk;
            s_status.done_bytes += chunk;
            s_status.elapsed_ms =
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
    safe_shutdown();
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
