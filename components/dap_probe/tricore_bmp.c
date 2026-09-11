/*
 * TriCore as a Black Magic Probe target.
 *
 * BMP already has everything a GDB session needs above the target: the RSP
 * parser, qXfer, extended-remote, vCont, monitor commands, watchpoint
 * bookkeeping, and a server already listening on 4242.  What it does not have
 * is a TriCore, because its targets all sit behind ADIv5 over SWD or JTAG and
 * this one speaks Infineon DAP on two wires.
 *
 * The interesting part is that none of that requires touching BMP.  target_new()
 * appends to BMP's global target list and target_attach_n() walks it, and both
 * are public - so a target can be registered from outside the component and
 * BMP's own server serves it.  No fork, no second port, no second server
 * fighting for the same pins.
 *
 * Everything below is an adapter.  The actual run control is in tricore.c.
 */

#include <stdlib.h>
#include <string.h>

#include "dap_phy.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "tricore.h"
#include "tricore_bmp.h"

static const char *TAG = "TRICORE_BMP";

/*
 * The Cerberus trigger line used to drive break-in.  Any of 1..7 works; this
 * one is simply not otherwise spoken for on this bench.
 */
#define HALT_LINE 1

/* Which core a target_s stands for.  BMP's model is one target per core, the
 * same as it does for a multi-core Cortex, so GDB attaches to one at a time. */
#define CORE_OF(target) ((int)(intptr_t)(target)->target_storage)

/*
 * The register layout GDB is told about.
 *
 * Serving our own description is what makes the `g` packet unambiguous: GDB
 * numbers registers in the order they appear here, so it cannot disagree with
 * us about where PC is - which it otherwise would, since different tricore-gdb
 * builds number them differently.  The order matches tricore.h.
 *
 * A10 and A11 are the stack pointer and return address in the TriCore ABI, so
 * typing A11 as a code pointer is what lets GDB unwind at all.
 */
static const char k_target_xml[] =
    "<?xml version=\"1.0\"?>"
    "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">"
    "<target version=\"1.0\">"
    "<architecture>tricore</architecture>"
    "<feature name=\"org.gnu.gdb.tricore.core\">"
    "<reg name=\"d0\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d1\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d2\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d3\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d4\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d5\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d6\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d7\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d8\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d9\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d10\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d11\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d12\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d13\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d14\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d15\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"a0\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a1\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a2\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a3\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a4\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a5\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a6\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a7\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a8\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a9\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a10\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a11\" bitsize=\"32\" type=\"code_ptr\"/>"
    "<reg name=\"a12\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a13\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a14\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a15\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"pcxi\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"psw\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>"
    "</feature>"
    "</target>";

/* Sticky: set by any failed transaction, reported and cleared by check_error. */
static bool s_error;

/* ------------------------------------------------------------------------ */
/* Memory                                                                    */
/* ------------------------------------------------------------------------ */

static void tricore_mem_read(target_s *target, void *dest, target_addr64_t src, size_t len)
{
    (void)target;
    if (tricore_read_mem((uint32_t)src, (uint8_t *)dest, len) != ESP_OK) {
        /*
         * BMP's memory calls return void; a failure is reported through
         * check_error, which GDB turns into an E packet.  Zeroing rather than
         * leaving the buffer untouched keeps a partly-filled read from looking
         * like real data if the error is ever missed.
         */
        memset(dest, 0, len);
        s_error = true;
    }
}

static void tricore_mem_write(target_s *target, target_addr64_t dest, const void *src, size_t len)
{
    (void)target;
    if (tricore_write_mem((uint32_t)dest, (const uint8_t *)src, len) != ESP_OK) {
        s_error = true;
    }
}

static bool tricore_check_error(target_s *target)
{
    (void)target;
    const bool had = s_error;
    s_error = false;
    return had;
}

/* ------------------------------------------------------------------------ */
/* Registers                                                                 */
/* ------------------------------------------------------------------------ */

static const char *tricore_regs_description(target_s *target)
{
    (void)target;
    /* gdb_main.c free()s what this returns, so it has to be heap allocated -
     * handing back the static string above would free rodata. */
    return strdup(k_target_xml);
}

static void tricore_regs_read(target_s *target, void *data)
{
    uint32_t regs[TRICORE_NUM_REGS];

    if (tricore_read_all_regs(CORE_OF(target), regs) != ESP_OK) {
        memset(data, 0, sizeof(regs));
        s_error = true;
        return;
    }
    memcpy(data, regs, sizeof(regs));
}

static void tricore_regs_write(target_s *target, const void *data)
{
    const uint32_t *regs = (const uint32_t *)data;

    for (int i = 0; i < TRICORE_NUM_REGS; i++) {
        if (tricore_write_reg(CORE_OF(target), i, regs[i]) != ESP_OK) {
            s_error = true;
        }
    }
}

static size_t tricore_reg_read(target_s *target, uint32_t reg, void *data, size_t max)
{
    uint32_t value = 0;

    if (max < sizeof(value) || reg >= TRICORE_NUM_REGS) {
        return 0;
    }
    if (tricore_read_reg(CORE_OF(target), (int)reg, &value) != ESP_OK) {
        s_error = true;
        return 0;
    }
    memcpy(data, &value, sizeof(value));
    return sizeof(value);
}

static size_t tricore_reg_write(target_s *target, uint32_t reg, const void *data, size_t size)
{
    uint32_t value;

    if (size < sizeof(value) || reg >= TRICORE_NUM_REGS) {
        return 0;
    }
    memcpy(&value, data, sizeof(value));
    if (tricore_write_reg(CORE_OF(target), (int)reg, value) != ESP_OK) {
        s_error = true;
        return 0;
    }
    return sizeof(value);
}

/* ------------------------------------------------------------------------ */
/* Run control                                                               */
/* ------------------------------------------------------------------------ */

static void tricore_halt_request_cb(target_s *target)
{
    if (tricore_halt_request(CORE_OF(target), HALT_LINE) != ESP_OK) {
        s_error = true;
    }
}

/*
 * Why the core stopped, in BMP's vocabulary.
 *
 * The trigger accumulator says which comparator tripped, and what occupies that
 * slot says whether it was a breakpoint or a watchpoint - which is the whole
 * difference between GDB saying "hit breakpoint 2" and saying nothing useful.
 * It clears on read, so it is read exactly once per stop.
 */
static target_halt_reason_e tricore_halt_poll_cb(target_s *target, target_addr64_t *watch)
{
    const int core = CORE_OF(target);

    if (!tricore_halt_poll(core)) {
        return TARGET_HALT_RUNNING;
    }

    uint32_t acc = 0;
    if (tricore_trigger_acc(core, &acc) == ESP_OK && acc != 0) {
        for (int slot = 0; slot < TRICORE_NUM_TRIGGERS; slot++) {
            if (!(acc & (1u << slot))) {
                continue;
            }
            const tricore_bp_t *bp = tricore_bp_of_slot(core, slot);
            if (bp == NULL) {
                continue;
            }
            if (bp->kind == TRICORE_BP_WATCH) {
                if (watch != NULL) {
                    *watch = bp->addr;
                }
                return TARGET_HALT_WATCHPOINT;
            }
            if (bp->kind == TRICORE_BP_USER) {
                return TARGET_HALT_BREAKPOINT;
            }
        }
    }
    return TARGET_HALT_REQUEST;
}

static void tricore_halt_resume(target_s *target, bool step)
{
    const int core = CORE_OF(target);

    if (step) {
        tricore_step_t result;
        if (tricore_step(core, 500, &result) != ESP_OK) {
            s_error = true;
        } else if (result.note[0]) {
            ESP_LOGI(TAG, "CPU%d step: %s", core, result.note);
        }
        return;
    }

    /*
     * Step off a breakpoint under the PC before resuming.  A trigger armed at
     * the current address fires again the instant the core runs, so a plain
     * resume would leave it exactly where it is - and GDB, seeing an immediate
     * stop at the same PC, would resume again, forever.
     */
    uint32_t pc = 0;
    if (tricore_is_halted(core) && tricore_read_pc(core, &pc) == ESP_OK) {
        const tricore_bp_t *bp = tricore_bp_at(core, pc);
        if (bp != NULL && bp->kind == TRICORE_BP_USER) {
            tricore_step_t stepped;
            tricore_step(core, 500, &stepped);
        }
    }
    tricore_request_resume(core);
}

/* ------------------------------------------------------------------------ */
/* Breakpoints and watchpoints                                               */
/* ------------------------------------------------------------------------ */

/*
 * Both breakpoint types are served from address triggers.  TriCore software
 * breakpoints mean patching a DEBUG instruction over the code, and for
 * flash-resident code that is the wrong trade: a programmed page cannot be
 * rewritten, so each toggle means erasing and reprogramming the enclosing
 * 16 kB sector - about a second, a program/erase cycle of a code sector every
 * time, and a debugger that dies between the erase and the rewrite leaves that
 * sector blank.
 */
static int tricore_breakwatch_set(target_s *target, breakwatch_s *bw)
{
    const int core = CORE_OF(target);
    int slot;

    switch (bw->type) {
    case TARGET_BREAK_SOFT:
    case TARGET_BREAK_HARD:
        slot = tricore_bp_add(core, (uint32_t)bw->addr, TRICORE_BP_USER);
        break;
    case TARGET_WATCH_WRITE:
        slot = tricore_bp_add_watch(core, (uint32_t)bw->addr, bw->size, false, true);
        break;
    case TARGET_WATCH_READ:
        slot = tricore_bp_add_watch(core, (uint32_t)bw->addr, bw->size, true, false);
        break;
    case TARGET_WATCH_ACCESS:
        slot = tricore_bp_add_watch(core, (uint32_t)bw->addr, bw->size, true, true);
        break;
    default:
        return 1;                   /* not something this target does */
    }

    if (slot < 0) {
        /*
         * Out of triggers.  Reported rather than swallowed: a breakpoint GDB
         * believes it set and the hardware never took is worse than one that
         * visibly failed.
         */
        ESP_LOGW(TAG, "CPU%d: no free trigger for 0x%08lX; all %d are in use",
                 core, (unsigned long)bw->addr, TRICORE_NUM_TRIGGERS);
        return -1;
    }
    return 0;
}

static int tricore_breakwatch_clear(target_s *target, breakwatch_s *bw)
{
    return tricore_bp_remove(CORE_OF(target), (uint32_t)bw->addr) ? 0 : 0;
}

/* ------------------------------------------------------------------------ */
/* Attach and detach                                                         */
/* ------------------------------------------------------------------------ */

static bool tricore_attach(target_s *target)
{
    const int core = CORE_OF(target);

    /*
     * A previous debugger may have left the halt-after-reset trigger armed on
     * TR0, and with it every resume immediately re-halts at the reset vector.
     */
    tricore_disarm_reset_trigger();

    /* Stop this core's timer while it is halted, so a breakpoint does not
     * silently kill a periodic task - see tricore.h. */
    tricore_freeze_timer(core, true);

    s_error = false;
    return true;
}

static void tricore_detach(target_s *target)
{
    const int core = CORE_OF(target);

    /* Leave the target as we found it: running, with none of our triggers
     * armed to halt it again later. */
    tricore_bp_clear_kind(core, TRICORE_BP_USER);
    tricore_bp_clear_kind(core, TRICORE_BP_STEP);
    tricore_bp_clear_kind(core, TRICORE_BP_WATCH);
    tricore_bp_clear_kind(core, TRICORE_BP_WATCH_HI);
    tricore_clear_debug_events(core);
    tricore_freeze_timer(core, false);
    tricore_request_resume(core);
}

/* ------------------------------------------------------------------------ */
/* Probe                                                                     */
/* ------------------------------------------------------------------------ */

/*
 * The DAP opening sequence, used both to probe and to recover after a reset.
 *
 * OCDS is enabled through the OEC unlock pattern rather than by resetting the
 * device, which is measured to work on this part: a hot attach reaches OEN=1,
 * the miniMCDS ID reads back 0x00D6C007, and a free-running STM reads
 * differently twice.  A reset is therefore not needed to get *access* - it is
 * needed for the other reasons in tricore_reset().
 */
static esp_err_t attach_dap(void)
{
    dap_exchange_t x;

    esp_err_t err = dap_probe_init(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DAP PHY unavailable on this board");
        return err;
    }

    /*
     * Through the fabric when it is there, which by default it is.
     *
     * The DAP bitstream replaces the Port C passthrough that the CPU-driven
     * path runs over, so with it loaded that path reads nothing but 0xFFFFFFFF
     * - this is not a preference between two working routes, it is the only
     * one available.  The fabric attach also needs a DAPISC the CPU one does
     * not, which is why it is a separate call rather than a flag.
     *
     * The CPU path stays as the fallback for a board running the stock image.
     */
    if (dap_phy_fpga_attach() == ESP_OK) {
        ESP_LOGI(TAG, "using the fabric DAP master");
    } else {
        dap_phy_fpga_use(false);
        ESP_LOGI(TAG, "no fabric DAP master; using the CPU-driven path");

        err = dap_probe_attach(&x, 3);
        if (err != ESP_OK || x.reply != 0xAAAAAAAAu) {
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
 * Reset the target and take control of it again.
 *
 * On this bench the probe's TRST line is wired to the target's reset, so this
 * restarts the application under test - which is the point.  GDB's `run` asks
 * for it, and it is the only way to debug anything before main(): a hot attach
 * lands wherever the application happens to be.
 *
 * Everything in the core is gone afterwards, including our triggers, so the
 * breakpoints BMP believes are set have to be re-armed.  GDB re-inserts them on
 * the next resume, which is why this does not try to restore them itself - but
 * the bookkeeping is cleared so nothing claims a trigger that no longer exists.
 *
 * The cores come back *halted at the entry point*, because OSTATE.HARR asks the
 * startup software to stop them before any application code runs.  That is the
 * part a reset pin cannot give on its own: pulsing reset starts the application
 * immediately, and re-attaching over DAP takes long enough that it is well into
 * startup before there is anything to halt.
 */
/* Wait for the device to answer over DAP again, re-establishing the link if it
 * went down with the reset.  Returns false if it never comes back. */
static bool wait_for_target(int attempts)
{
    for (int i = 0; i < attempts; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (tricore_discover() == ESP_OK) {
            return true;
        }
        dap_probe_clear_error_state();
        /* An application reset should leave the link up, but a heavier reset
         * takes it down; rebuilding from sync costs little and covers both. */
        attach_dap();
    }
    return false;
}

static void tricore_reset(target_s *target)
{
    (void)target;

    /*
     * Ask the startup software to stop the cores before any application code
     * runs, then restart the application through OCDS.  Order matters: HARR is
     * read by the boot ROM on its way up, so it has to be set before the reset,
     * not after it.
     */
    const bool halt_armed = (tricore_set_halt_after_reset(true) == ESP_OK);
    if (!halt_armed) {
        ESP_LOGW(TAG, "could not arm halt-after-reset; the cores will run on");
    }

    tricore_request_application_reset();

    if (!wait_for_target(20)) {
        /*
         * The OCDS reset did not bring it back.  Fall back to the reset pin,
         * which is heavier - it takes the debug domain down too, so OCDS and
         * the link have to be rebuilt - but it is the one that always works.
         */
        ESP_LOGW(TAG, "no answer after the OCDS reset; falling back to the reset pin");
        dap_phy_set_trst(true);
        vTaskDelay(pdMS_TO_TICKS(20));
        dap_phy_set_trst(false);
        vTaskDelay(pdMS_TO_TICKS(50));

        if (attach_dap() != ESP_OK || !wait_for_target(10)) {
            ESP_LOGE(TAG, "the target did not come back after the reset");
            s_error = true;
            return;
        }
    }

    /*
     * Clear the request now that it has been served, or the next application
     * reset - including one the application asks for itself - stops the cores
     * with no debugger expecting it.
     */
    if (halt_armed) {
        tricore_set_halt_after_reset(false);
    }

    for (int i = 0; i < tricore_core_count(); i++) {
        const int core = tricore_core_index(i);
        ESP_LOGI(TAG, "CPU%d is %s after the reset", core,
                 tricore_is_halted(core) ? "halted" : "running");
    }

    /*
     * A halt-after-reset trigger left on TR0 by any debugger - including one
     * the device armed for itself - re-halts the core on every resume.
     */
    tricore_disarm_reset_trigger();

    for (int i = 0; i < tricore_core_count(); i++) {
        const int core = tricore_core_index(i);
        /* The core's triggers went with the reset; drop our record of them so
         * nothing claims a slot that is no longer armed. */
        tricore_bp_clear_kind(core, TRICORE_BP_USER);
        tricore_bp_clear_kind(core, TRICORE_BP_STEP);
        tricore_bp_clear_kind(core, TRICORE_BP_WATCH);
        tricore_bp_clear_kind(core, TRICORE_BP_WATCH_HI);
        tricore_freeze_timer(core, true);
    }
    ESP_LOGI(TAG, "target reset; %d core(s) back", tricore_core_count());
}

esp_err_t tricore_bmp_probe(void)
{
    esp_err_t err = attach_dap();
    if (err != ESP_OK) {
        return err;
    }
    err = tricore_discover();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no TriCore cores answered");
        return err;
    }

    for (int i = 0; i < tricore_core_count(); i++) {
        const int core = tricore_core_index(i);

        target_s *target = target_new();
        if (target == NULL) {
            return ESP_ERR_NO_MEM;
        }
        target->target_storage = (void *)(intptr_t)core;
        target->driver = "TriCore TC3xx";
        target->core = "TriCore";
        target->designer_code = 0;
        target->part_id = 0;

        target->attach = tricore_attach;
        target->detach = tricore_detach;
        target->check_error = tricore_check_error;

        target->mem_read = tricore_mem_read;
        target->mem_write = tricore_mem_write;

        target->regs_size = TRICORE_NUM_REGS * sizeof(uint32_t);
        target->regs_description = tricore_regs_description;
        target->regs_read = tricore_regs_read;
        target->regs_write = tricore_regs_write;
        target->reg_read = tricore_reg_read;
        target->reg_write = tricore_reg_write;

        target->halt_request = tricore_halt_request_cb;
        target->halt_poll = tricore_halt_poll_cb;
        target->halt_resume = tricore_halt_resume;

        target->breakwatch_set = tricore_breakwatch_set;
        target->breakwatch_clear = tricore_breakwatch_clear;

        /*
         * Reset restarts the application under test, because on this bench the
         * probe's TRST is the target's reset line.  That is what GDB's `run`
         * means, and what debugging anything before main() requires, so it is
         * wired up - but it is worth knowing it is not a debug-domain-only
         * reset: the application really does start again.
         *
         * Flash is left unset; it needs 64-bit writes through Cerberus, which
         * is untested.
         */
        target->reset = tricore_reset;
        target->extended_reset = tricore_reset;

        ESP_LOGI(TAG, "registered CPU%d as a GDB target", core);
    }
    ESP_LOGI(TAG, "%d core(s) available - attach with GDB on port 4242",
             tricore_core_count());
    return ESP_OK;
}
