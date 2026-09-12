#include "tricore.h"
#include "tricore_priv.h"

#include <inttypes.h>
#include <string.h>

#include "dap_probe.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TRICORE";

/* ------------------------------------------------------------------------ */

bool tricore_present[TRICORE_MAX_CORES];
static int          s_count;

bool tricore_core_ok(int core)
{
    return core >= 0 && core < TRICORE_MAX_CORES && tricore_present[core];
}

uint32_t tricore_core_reg(int core, uint32_t offset)
{
    return k_core_base[core] + offset;
}

esp_err_t tricore_rd(int core, uint32_t offset, uint32_t *out)
{
    return dap_probe_read32(tricore_core_reg(core, offset), out);
}

esp_err_t tricore_wr(int core, uint32_t offset, uint32_t value)
{
    return dap_probe_write32(tricore_core_reg(core, offset), value);
}

/* ------------------------------------------------------------------------ */
/* Discovery                                                                 */
/* ------------------------------------------------------------------------ */

esp_err_t tricore_discover(void)
{
    uint32_t ostate = 0;

    memset(tricore_present, 0, sizeof(tricore_present));
    tricore_forget_triggers();
    s_count = 0;

    if (dap_probe_read32(CBS_OSTATE, &ostate) != ESP_OK) {
        ESP_LOGE(TAG, "OSTATE unreadable - is the probe attached?");
        return ESP_ERR_INVALID_STATE;
    }
    if (!(ostate & 1u)) {
        /*
         * Without OEN the whole CSFR window bus-errors and every core looks
         * absent, so say what is actually wrong rather than "no cores".
         */
        ESP_LOGE(TAG, "OCDS is off (OSTATE 0x%08" PRIX32 "); enable it first", ostate);
        return ESP_ERR_INVALID_STATE;
    }

    for (int core = 0; core < TRICORE_MAX_CORES; core++) {
        uint32_t dbgsr = 0;
        if (dap_probe_read32(k_core_base[core] + OFF_DBGSR, &dbgsr) == ESP_OK) {
            tricore_present[core] = true;
            s_count++;
            ESP_LOGI(TAG, "CPU%d present, DBGSR 0x%08" PRIX32 " (%s)", core, dbgsr,
                     ((dbgsr >> DBGSR_HALT_SHIFT) & 1u) ? "halted" : "running");
        } else {
            dap_probe_clear_error_state();
        }
    }
    return s_count ? ESP_OK : ESP_ERR_NOT_FOUND;
}

int tricore_core_count(void)
{
    return s_count;
}

int tricore_core_index(int n)
{
    for (int core = 0; core < TRICORE_MAX_CORES; core++) {
        if (tricore_present[core] && n-- == 0) {
            return core;
        }
    }
    return -1;
}

bool tricore_core_present(int core)
{
    return tricore_core_ok(core);
}

/* ------------------------------------------------------------------------ */
/* State                                                                     */
/* ------------------------------------------------------------------------ */

esp_err_t tricore_dbgsr(int core, uint32_t *out)
{
    if (!tricore_core_ok(core) || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    /*
     * Retried, because a device coming out of reset briefly fails every
     * transaction and a poll loop must not treat that as fatal.
     */
    for (int attempt = 0; attempt < 3; attempt++) {
        if (tricore_rd(core, OFF_DBGSR, out) == ESP_OK) {
            return ESP_OK;
        }
        dap_probe_clear_error_state();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_ERR_TIMEOUT;
}

bool tricore_is_halted(int core)
{
    uint32_t dbgsr = 0;
    if (tricore_dbgsr(core, &dbgsr) != ESP_OK) {
        return false;
    }
    return ((dbgsr >> DBGSR_HALT_SHIFT) & 1u) != 0;
}

bool tricore_debug_enabled(int core)
{
    uint32_t dbgsr = 0;
    return tricore_dbgsr(core, &dbgsr) == ESP_OK && (dbgsr & DBGSR_DE);
}

bool tricore_wait_halted(int core, bool want, uint32_t timeout_ms)
{
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    for (;;) {
        if (tricore_is_halted(core) == want) {
            return true;
        }
        if (esp_timer_get_time() >= deadline) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/* ------------------------------------------------------------------------ */
/* Run control                                                               */
/* ------------------------------------------------------------------------ */

/* The trigger line each core is currently being halted with, and the TLC value
 * to put back when the line is released.  -1 when no halt is in flight. */
static int      s_halt_line[TRICORE_MAX_CORES];
static uint32_t s_halt_tlc[TRICORE_MAX_CORES];

esp_err_t tricore_halt_request(int core, int line)
{
    if (!tricore_core_ok(core)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (line < 1 || line > 7) {
        return ESP_ERR_INVALID_ARG;    /* line 0 does not exist */
    }

    /*
     * The long way round compared with writing DBGSR.HALT, and it buys one
     * thing: this halt is a debug *event*, so it asserts the core's suspend-out
     * signal and the peripherals watching it - the system timer among them -
     * stop with the core.  A DBGSR write is not an event, and a pause taken
     * that way leaves the timer running, which is what kills a periodic task's
     * tick when you pause inside it.
     */
    esp_err_t err = tricore_wr(core, OFF_EXEVT, EXEVT_HALT_AND_SUSPEND);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t routing = 0;
    if (dap_probe_read32(CBS_TRC(core), &routing) != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }
    routing &= ~(TRC_ROUTE_MASK << TRC_BRKIN_SHIFT);
    routing |= (uint32_t)line << TRC_BRKIN_SHIFT;
    if (dap_probe_write32(CBS_TRC(core), routing) != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t control = 0;
    if (dap_probe_read32(CBS_TLC, &control) != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }
    const int shift = 4 * line;

    /*
     * The released state of the line is the baseline, not whatever is there
     * now.
     *
     * Reading the current value and restoring it later assumes the line is
     * idle to begin with, and if a previous halt left it forced active that
     * assumption writes the fault in permanently: the "assert" below is then
     * no transition at all, the core never takes a break-in event, and every
     * halt from then on fails.  It survives a probe reboot too, because the
     * state is the target's.  So the field is cleared here and released
     * unconditionally when the wait ends - which is what the reference does
     * with a try/finally.
     */
    s_halt_line[core] = line;
    s_halt_tlc[core]  = control & ~(0xFu << shift);

    if (dap_probe_write32(CBS_TLC, s_halt_tlc[core]) != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }
    return dap_probe_write32(CBS_TLC,
                             s_halt_tlc[core] | (TLSP_FORCE_ACTIVE << shift));
}

void tricore_halt_diag(int core, const char *what)
{
    if (!tricore_core_ok(core)) {
        return;
    }

    uint32_t dbgsr = 0, exevt = 0, trc = 0, tlc = 0, ostate = 0;

    tricore_rd(core, OFF_DBGSR, &dbgsr);
    tricore_rd(core, OFF_EXEVT, &exevt);
    dap_probe_read32(CBS_TRC(core), &trc);
    dap_probe_read32(CBS_TLC, &tlc);
    dap_probe_read32(CBS_OSTATE, &ostate);

    /*
     * The four registers that separate the ways a halt can fail to arrive, and
     * what each should read once it has:
     *
     *   DBGSR  0x13   halted, DE set, SUSP set, EVTSRC 0 for EXEVT
     *   EXEVT  0x22   halt and suspend, which is what this asks for
     *   TRC    BRKIN  in bits 23:20, naming the line driving this core
     *   TLC    0      the line released again
     *   OSTATE OEN    set, or none of the rest is even listened to
     *
     * OEN clear means OCDS is off and every debug write is ignored in silence,
     * which is the one failure that looks identical to a dead probe.
     */
    ESP_LOGE(TAG, "CPU%d %s: halt did not arrive. DBGSR=0x%08" PRIX32
                  " EXEVT=0x%08" PRIX32 " TRC=0x%08" PRIX32
                  " TLC=0x%08" PRIX32 " OSTATE=0x%08" PRIX32 "%s",
             core, what, dbgsr, exevt, trc, tlc, ostate,
             (ostate & OSTATE_OEN) ? "" : "  <- OCDS is off");
}

void tricore_halt_release(int core)
{
    if (!tricore_core_ok(core) || s_halt_line[core] <= 0) {
        return;
    }
    /* Unconditional: a line left forced active stops the *next* halt from
     * being an edge, so giving up on a halt has to undo it too. */
    dap_probe_write32(CBS_TLC, s_halt_tlc[core]);
    s_halt_line[core] = 0;
}

bool tricore_halt_poll(int core)
{
    if (!tricore_core_ok(core)) {
        return false;
    }
    if (!tricore_is_halted(core)) {
        return false;
    }
    if (s_halt_line[core] > 0) {
        /* Release the line, or every core routed to it stays halted. */
        dap_probe_write32(CBS_TLC, s_halt_tlc[core]);
        s_halt_line[core] = 0;
    }
    return true;
}

esp_err_t tricore_halt(int core, int line, uint32_t timeout_ms)
{
    if (!tricore_core_ok(core)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (tricore_is_halted(core)) {
        return ESP_OK;
    }
    const esp_err_t err = tricore_halt_request(core, line);
    if (err != ESP_OK) {
        return err;
    }
    const bool stopped = tricore_wait_halted(core, true, timeout_ms);

    /* Release the line whatever happened. */
    if (s_halt_line[core] > 0) {
        dap_probe_write32(CBS_TLC, s_halt_tlc[core]);
        s_halt_line[core] = 0;
    }
    return stopped ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t tricore_request_resume(int core)
{
    if (!tricore_core_ok(core)) {
        return ESP_ERR_INVALID_ARG;
    }
    /*
     * HALT = 0b10 including the mask bit, i.e. 0x04.  DE is read-only, so this
     * cannot disturb it.
     */
    return tricore_wr(core, OFF_DBGSR, HALT_REQ_CLEAR << DBGSR_HALT_SHIFT);
}

esp_err_t tricore_resume(int core, uint32_t timeout_ms)
{
    const esp_err_t err = tricore_request_resume(core);
    if (err != ESP_OK) {
        return err;
    }
    /*
     * A caller that armed a trigger at or just after the PC should use
     * tricore_request_resume() instead: the core re-halts faster than this can
     * see it running, so the wait always times out.
     */
    return tricore_wait_halted(core, false, timeout_ms) ? ESP_OK : ESP_ERR_TIMEOUT;
}

void tricore_clear_debug_events(int core)
{
    if (!tricore_core_ok(core)) {
        return;
    }
    /*
     * A safety net for a session that died mid-pause: EXEVT left armed halts
     * the core again the moment anything drives its trigger line, and CREVT
     * would stop it on every register access.
     */
    tricore_wr(core, OFF_CREVT, 0);
    tricore_wr(core, OFF_EXEVT, 0);
    tricore_wr(core, OFF_SWEVT, 0);
}

esp_err_t tricore_set_halt_after_reset(bool enable)
{
    /*
     * The protection bit goes in whether the request is being set or cleared:
     * it is the key that makes the write land at all, not part of the value.
     */
    const uint32_t value = OCNTRL_HARR_P | (enable ? OCNTRL_HARR : 0u);

    const esp_err_t err = dap_probe_write32(CBS_OCNTRL, value);
    if (err != ESP_OK) {
        return err;
    }
    /*
     * Read it back from OSTATE rather than trusting the write.  OCNTRL is
     * write-only, so this is the only way to know the key was accepted - and a
     * dropped write here would show up much later as a reset that failed to
     * halt, with nothing to connect it to.
     */
    if (tricore_halt_after_reset_pending() != enable) {
        ESP_LOGE(TAG, "OSTATE.HARR did not follow the OCNTRL write");
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool tricore_halt_after_reset_pending(void)
{
    uint32_t ostate = 0;

    if (dap_probe_read32(CBS_OSTATE, &ostate) != ESP_OK) {
        dap_probe_clear_error_state();
        return false;
    }
    return (ostate & OSTATE_HARR) != 0;
}

esp_err_t tricore_request_application_reset(void)
{
    /*
     * An application reset restarts the application and leaves the debug
     * infrastructure running, so the DAP link and the OCDS enable survive it.
     * That is what makes this better than pulsing the reset pin, which takes
     * the whole debug domain down with it and has to be rebuilt from sync.
     *
     * No error check on the write: the device is resetting as it lands, so the
     * acknowledge may never come back.  Whether it worked is answered by what
     * the target looks like afterwards, not by this transaction.
     */
    ESP_LOGI(TAG, "requesting an OCDS application reset");
    dap_probe_write32(CBS_OCNTRL, OCNTRL_APPRESET_P | OCNTRL_APPRESET);
    dap_probe_clear_error_state();
    return ESP_OK;
}

esp_err_t tricore_freeze_timer(int core, bool enable)
{
    if (!tricore_core_ok(core)) {
        return ESP_ERR_INVALID_ARG;
    }
    /*
     * Without this a breakpoint is enough to kill a periodic task for good:
     * the STM keeps counting while the core is stopped, so a tick handler that
     * reprograms its compare as "previous + period" writes one already in the
     * past, and a 32-bit compare only matches on equality.
     */
    return dap_probe_write32(STM_BASE(core) + STM_OCS,
                             enable ? STM_OCS_SUS_HARD : STM_OCS_SUS_OFF);
}

/* ------------------------------------------------------------------------ */
/* Registers                                                                 */
/* ------------------------------------------------------------------------ */

