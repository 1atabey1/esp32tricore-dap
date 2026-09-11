/*
 * Can this probe write 64 bits to the flash page assembly buffer?
 *
 * That one question decides whether flashing can move off DAS, and it is worth
 * answering before building a flash driver rather than after.  The rest of TC3xx
 * program flashing is settled: tas-debug's flasher drives the DMU command
 * sequence interpreter purely by memory writes, nothing executes on the target,
 * and it works.  The sequences, the 16 kB sectors, the 32-byte pages, the
 * prerequisites and the error flags are all known.
 *
 * What is not settled is the page load itself:
 *
 *     "The page assembly buffer takes 64-bit writes.  32-bit loads are refused
 *      with a sequence error (SQER) and drop the interpreter out of page mode."
 *
 * The documented Cerberus IO instruction catalog has 8, 16 and 32 bit accesses
 * and a word-wide block transfer.  No dword instruction.  So either the width
 * can be changed some other way, or program flashing does not come across and
 * the plan should say so plainly instead of implying it might.
 *
 * The one untried lever is IOCONF's EX_BUS_HW width select.  Its position in the
 * register is not documented anywhere this project has found - IOCONF is
 * write-only, twelve bits, with MODE at 0 and SVM_MODE at 7 known, and TRIG_EN,
 * EX_BUS_TRC, EX_BUS_HW and FPI_PRIO somewhere in between.  So this sweeps them.
 *
 * What this does NOT do: no erase, no program, no ENDINIT change.  It enters
 * page mode, tries a load, and reads the error flags.  A page load with no
 * program command behind it commits nothing, and a sequence error simply drops
 * the interpreter back out of page mode - which is the result being measured.
 */

#include "tricore_flash_probe.h"

#include <inttypes.h>

#include "dap_frame.h"
#include "dap_probe.h"
#include "esp_log.h"
#include "tricore.h"

static const char *TAG = "FLASH_PROBE";

/* The DMU command interface, driven through the data flash window. */
#define COMMAND_BASE        0xAF000000u
#define CYCLE_5554          0x5554u
#define CYCLE_55F0          0x55F0u      /* the page assembly buffer */

#define CMD_CLEAR_STATUS        0xFAu
#define CMD_ENTER_PAGE_PFLASH   0x50u    /* 0x5D is the *data* flash page, and
                                          * selecting it loads the wrong buffer
                                          * and silently never commits */

#define DMU_HF_STATUS       0xF8040010u
#define DMU_HF_ERRSR        0xF8040034u
#define STATUS_PFPAGE       (1u << 21)
#define ERRSR_SQER          (1u << 1)

/*
 * IOCONF is IOClient instruction 0, write-only, twelve bits.  Only two field
 * positions are known: MODE at 0 and SVM_MODE at 7.  The instruction number and
 * the width live in dap_probe.c rather than a header, so they are repeated here
 * rather than widening that file's interface for one diagnostic.
 */
#define IO_CONF             0x0u
#define IOCONF_BITS         12u
#define IOCONF_MODE_RW      (1u << 0)
#define IOCONF_SVM          (1u << 7)

static esp_err_t cycle(uint32_t offset, uint32_t value)
{
    return dap_probe_write32(COMMAND_BASE + offset, value);
}

/* Enter program flash page mode, and say whether the interpreter agrees. */
static bool enter_page_mode(void)
{
    uint32_t status = 0;

    cycle(CYCLE_5554, CMD_CLEAR_STATUS);
    cycle(CYCLE_5554, CMD_ENTER_PAGE_PFLASH);

    if (dap_probe_read32(DMU_HF_STATUS, &status) != ESP_OK) {
        dap_probe_clear_error_state();
        return false;
    }
    return (status & STATUS_PFPAGE) != 0;
}

/*
 * Load eight bytes as two 32-bit writes and report whether page mode survived.
 *
 * Surviving is the whole result.  If the interpreter stays in page mode and
 * raises no sequence error, then whatever IOCONF setting is in force produced a
 * bus transaction the flash accepted, and program flashing comes across.
 */
static bool try_page_load(uint32_t *errsr_out)
{
    uint32_t status = 0;

    *errsr_out = 0;

    if (dap_probe_write32(COMMAND_BASE + CYCLE_55F0, 0xCAFEBABEu) != ESP_OK ||
        dap_probe_write32(COMMAND_BASE + CYCLE_55F0 + 4u, 0xDEADBEEFu) != ESP_OK) {
        dap_probe_clear_error_state();
        return false;
    }
    if (dap_probe_read32(DMU_HF_ERRSR, errsr_out) != ESP_OK) {
        dap_probe_clear_error_state();
    }
    if (dap_probe_read32(DMU_HF_STATUS, &status) != ESP_OK) {
        dap_probe_clear_error_state();
        return false;
    }
    return (status & STATUS_PFPAGE) != 0 && !(*errsr_out & ERRSR_SQER);
}

/* Leave the interpreter in a known state whatever happened. */
static void abandon_page_mode(void)
{
    cycle(CYCLE_5554, CMD_CLEAR_STATUS);
    dap_probe_clear_error_state();
}

esp_err_t tricore_flash_probe_width(void)
{
    dap_exchange_t x;
    uint32_t       errsr = 0;

    ESP_LOGW(TAG, "=== can the probe load a flash page? ===");

    /*
     * A halted core, because nothing should be fetching out of the bank whose
     * command interface is being driven.  Not fatal if it will not halt - the
     * measurement is about bus width, not about what the core is doing - but
     * worth saying, since it changes how much the result is worth.
     */
    const int core = tricore_core_index(0);
    if (core < 0) {
        ESP_LOGE(TAG, "no cores; run the GDB attach first");
        return ESP_ERR_INVALID_STATE;
    }
    if (!tricore_is_halted(core) && tricore_halt(core, 1, 1000) != ESP_OK) {
        ESP_LOGW(TAG, "CPU%d would not halt; continuing anyway", core);
    }

    /* 1: is the command interface reachable at all? */
    if (!enter_page_mode()) {
        ESP_LOGE(TAG, "program flash would not enter page mode - the command "
                      "interface is not reachable, so nothing below means anything");
        abandon_page_mode();
        return ESP_FAIL;
    }
    ESP_LOGW(TAG, "page mode entered (STATUS.PFPAGE set)");

    /* 2: confirm the premise on *this* link, rather than assuming it. */
    const bool survived_plain = try_page_load(&errsr);
    ESP_LOGW(TAG, "  32-bit page load, IOCONF as usual: %s (ERRSR 0x%08" PRIX32 "%s)",
             survived_plain ? "ACCEPTED" : "refused", errsr,
             (errsr & ERRSR_SQER) ? ", SQER" : "");
    abandon_page_mode();

    if (survived_plain) {
        /*
         * Then the premise does not hold here and flashing is simply available.
         * Worth being suspicious of: check that PFPAGE was really set first,
         * because a page load that "works" because nothing was listening looks
         * exactly like this.
         */
        ESP_LOGW(TAG, "=== 32-bit page loads are accepted; flashing can come across ===");
        return ESP_OK;
    }

    /*
     * 3: sweep the IOCONF bits that are not accounted for.  MODE at 0 and
     * SVM_MODE at 7 are known; TRIG_EN, EX_BUS_TRC, EX_BUS_HW and FPI_PRIO are
     * somewhere in 1..6, and only EX_BUS_HW should change the transaction
     * width.  Sweeping is cheaper than finding a document that pins it down.
     */
    ESP_LOGW(TAG, "--- sweeping the undocumented IOCONF bits for a width select ---");
    int found = -1;
    for (unsigned bit = 1; bit <= 6; bit++) {
        const uint16_t conf = IOCONF_MODE_RW | IOCONF_SVM | (uint16_t)(1u << bit);

        if (dap_probe_client_write(IO_CONF, 4, conf, IOCONF_BITS, &x) != ESP_OK) {
            ESP_LOGW(TAG, "  IOCONF 0x%03X: not acknowledged", conf);
            continue;
        }
        if (!enter_page_mode()) {
            ESP_LOGW(TAG, "  IOCONF 0x%03X: page mode would not enter", conf);
            abandon_page_mode();
            continue;
        }
        const bool ok = try_page_load(&errsr);
        ESP_LOGW(TAG, "  IOCONF 0x%03X (bit %u): %s%s", conf, bit,
                 ok ? "ACCEPTED" : "refused",
                 (errsr & ERRSR_SQER) ? ", SQER" : "");
        abandon_page_mode();
        if (ok && found < 0) {
            found = (int)bit;
        }
    }

    /* Put IOCONF back however the sweep left it. */
    dap_probe_set_rw_mode(true);
    dap_probe_clear_error_state();

    if (found >= 0) {
        ESP_LOGW(TAG, "=== IOCONF bit %d widens the access; flashing can come across ===",
                 found);
        return ESP_OK;
    }

    /*
     * Nothing worked.  That is a real answer, not a failure: it says the
     * remaining avenue is a two-parcel block write that the bridge merges into
     * one SRI transaction, and that program flashing stays on DAS until
     * somebody shows Cerberus can issue a 64-bit access.
     */
    ESP_LOGE(TAG, "=== no IOCONF setting made a 32-bit page load acceptable ===");
    ESP_LOGE(TAG, "    the remaining avenue is a merged two-parcel block write;");
    ESP_LOGE(TAG, "    until that is shown to work, flashing stays on DAS");
    return ESP_ERR_NOT_SUPPORTED;
}
