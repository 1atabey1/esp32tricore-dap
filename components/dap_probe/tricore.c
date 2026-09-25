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
        /* Without OEN every core would look absent. */
        ESP_LOGE(TAG, "OCDS is off (OSTATE 0x%08" PRIX32 "); enable it first", ostate);
        return ESP_ERR_INVALID_STATE;
    }

    for (int core = 0; core < TRICORE_MAX_CORES; core++) {
        uint32_t dbgsr = 0;
        if (dap_probe_read32(k_core_base[core] + OFF_DBGSR, &dbgsr) != ESP_OK) {
            dap_probe_clear_error_state();
            continue;
        }
        tricore_present[core] = true;
        s_count++;
        ESP_LOGI(TAG, "CPU%d present, %s", core,
                 !tricore_core_started(core) ? "not started (boot halt)"
                 : ((dbgsr >> DBGSR_HALT_SHIFT) & 1u) ? "halted" : "running");
    }
    return s_count ? ESP_OK : ESP_ERR_NOT_FOUND;
}

bool tricore_core_started(int core)
{
    uint32_t syscon = 0;

    /* Read live: CPU0's software releases the others after discovery ran. */
    return tricore_core_ok(core) &&
           tricore_rd(core, OFF_SYSCON, &syscon) == ESP_OK &&
           !(syscon & SYSCON_BHALT);
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
    /* Retried: a device coming out of reset briefly fails every transaction. */
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

    /* Trigger lines are shared: a running core still armed by an earlier halt
     * would stop too, and nothing would resume it. */
    for (int other = 0; other < TRICORE_MAX_CORES; other++) {
        uint32_t armed = 0;
        if (other != core && tricore_core_present(other) &&
            tricore_core_started(other) && !tricore_is_halted(other) &&
            tricore_rd(other, OFF_EXEVT, &armed) == ESP_OK && armed != 0) {
            tricore_wr(other, OFF_EXEVT, 0);
        }
    }

    /* An EXEVT halt is a debug event, so suspend-out stops the STM too;
     * a DBGSR.HALT write would not. */
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

    /* Restore to the released state, not the current value: a line left forced
     * active by an earlier halt would make the assert below no edge at all. */
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
     * Expected after a successful halt:
     *   DBGSR  0x13   halted, DE set, SUSP set, EVTSRC 0 for EXEVT
     *   EXEVT  0x22   halt and suspend
     *   TRC    BRKIN  in bits 23:20, naming the line driving this core
     *   TLC    0      the line released again
     *   OSTATE OEN    set; clear means every debug write is silently ignored
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
    /* HALT = 0b10 (mask bit set), i.e. 0x04.  DE is read-only. */
    return tricore_wr(core, OFF_DBGSR, HALT_REQ_CLEAR << DBGSR_HALT_SHIFT);
}

esp_err_t tricore_resume(int core, uint32_t timeout_ms)
{
    const esp_err_t err = tricore_request_resume(core);
    if (err != ESP_OK) {
        return err;
    }
    return tricore_wait_halted(core, false, timeout_ms) ? ESP_OK : ESP_ERR_TIMEOUT;
}

void tricore_clear_debug_events(int core)
{
    if (!tricore_core_ok(core)) {
        return;
    }
    /* A stale EXEVT re-halts on any trigger line; CREVT on every register access. */
    tricore_wr(core, OFF_CREVT, 0);
    tricore_wr(core, OFF_EXEVT, 0);
    tricore_wr(core, OFF_SWEVT, 0);
}

esp_err_t tricore_set_halt_after_reset(bool enable)
{
    /* The protection bit is the write key, needed to set or clear. */
    const uint32_t value = OCNTRL_HARR_P | (enable ? OCNTRL_HARR : 0u);

    const esp_err_t err = dap_probe_write32(CBS_OCNTRL, value);
    if (err != ESP_OK) {
        return err;
    }
    /* OCNTRL is write-only; confirm via OSTATE. */
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
    /* Keeps the DAP link and OCDS enable up.  Unchecked: the device resets as
     * the write lands, so the acknowledge may never come. */
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
    /* A running STM would put a "previous + period" compare in the past,
     * and the equality compare would not match until the timer wraps. */
    return dap_probe_write32(STM_BASE(core) + STM_OCS,
                             enable ? STM_OCS_SUS_HARD : STM_OCS_SUS_OFF);
}

