#include "tricore.h"

#include <inttypes.h>
#include <string.h>

#include "dap_probe.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TRICORE";

/* ------------------------------------------------------------------------ */
/* Cerberus, the system debug block                                          */
/* ------------------------------------------------------------------------ */
/*
 * Every offset the manual gives is relative to this base, not to 0xF0000000 -
 * a mistake that lands on real registers rather than empty space: 0xF00004B0
 * is TCCB, read-only, so writing it bus-errors, while 0xF00004C0 is TREC0 and
 * quietly accepts the write and reconfigures CPU0 event routing.
 */
#define CBS_BASE            0xF0000400u
#define CBS_OCNTRL          0xF000047Cu
#define CBS_OSTATE          0xF0000480u

/*
 * OCNTRL is write-only and paired: every control bit sits at an odd position
 * 2n+1 with its write-protection bit at 2n, and the protection bit has to be
 * set in the same write or the change is dropped.  A write missing the key
 * looks like it succeeded and does nothing, which is the failure mode worth
 * guarding against - so these are always used together.
 *
 * Cross-checked two ways before being trusted: OC4/OC4_P at 9/8 is the 0x0300
 * that dap_probe_enable_ocds() writes and that was watched enabling the
 * miniMCDS on silicon, and WDTSUS/WDTSUS_P at 13/12 matches what tas-debug
 * derived independently on the same part.
 */
#define OCNTRL_HARR         (1u << 17)   /* OJC0: halt after reset */
#define OCNTRL_HARR_P       (1u << 16)
#define OCNTRL_APPRESET     (1u << 31)   /* OJC7/RSTCL3: application reset */
#define OCNTRL_APPRESET_P   (1u << 30)

#define OSTATE_OEN          (1u << 0)
#define OSTATE_HARR         (1u << 8)

/* Trigger routing per core: which line this core's break-in input listens to. */
#define CBS_TRC(core)       (CBS_BASE + 0x20u + 4u * (core))
#define TRC_BRKIN_SHIFT     20
#define TRC_ROUTE_MASK      0xFu

/*
 * Trigger line control: seven 4-bit fields, one per line.  This is the only way
 * to drive a line from the probe; there is no separate assert register.
 */
#define CBS_TLC             (CBS_BASE + 0x90u)
#define TLSP_NONE           0x0u
#define TLSP_FORCE_ACTIVE   0x3u

/* ------------------------------------------------------------------------ */
/* Per-core CSFR window                                                      */
/* ------------------------------------------------------------------------ */

/* TC38x core bases.  A part with fewer cores simply has fewer of them answer. */
static const uint32_t k_core_base[TRICORE_MAX_CORES] = {
    0xF8810000u, 0xF8830000u, 0xF8850000u, 0xF8870000u, 0xF8890000u, 0xF88B0000u,
};

#define OFF_TR_EVT(n)       (0xF000u + 8u * (n))
#define OFF_TR_ADR(n)       (0xF004u + 8u * (n))
#define OFF_DBGSR           0xFD00u
#define OFF_EXEVT           0xFD08u
#define OFF_CREVT           0xFD0Cu
#define OFF_SWEVT           0xFD10u
#define OFF_TRIG_ACC        0xFD30u
#define OFF_PCXI            0xFE00u
#define OFF_PSW             0xFE04u
#define OFF_PC              0xFE08u
#define OFF_D0              0xFF00u     /* D0..D15, four bytes apart */
#define OFF_A0              0xFF80u     /* A0..A15, four bytes apart */

/*
 * The rest of the core registers GDB asks for, from Infineon's IfxCpu_reg.h.
 * PCXI, PSW, PC, D0 and A0 above are the five the tas-debug reference also
 * uses, and agree with it.
 */
#define OFF_SYSCON          0xFE14u
#define OFF_BIV             0xFE20u
#define OFF_BTV             0xFE24u
#define OFF_ISP             0xFE28u
#define OFF_ICR             0xFE2Cu
#define OFF_FCX             0xFE38u
#define OFF_LCX             0xFE3Cu
#define OFF_DCON0           0x9040u
#define OFF_PCON0           0x920Cu

/* DBGSR */
#define DBGSR_DE            (1u << 0)
#define DBGSR_HALT_SHIFT    1
/*
 * HALT is [2:1] and bit 2 is a write mask: HALT[0] changes only when it is set.
 * 0b10 clears the halt (resume); 0b11 sets it, which works but is not a debug
 * event and therefore cannot suspend the timers.
 */
#define HALT_REQ_CLEAR      0b10u

/* Debug event registers: EVTA in bits [2:0], SUSP at bit 5. */
#define EVTA_HALT           0b010u
#define EVT_SUSP            (1u << 5)
#define EXEVT_HALT_AND_SUSPEND (EVT_SUSP | EVTA_HALT)

/*
 * The exact TRnEVT word a working debugger uses for halt-on-instruction-address,
 * reused verbatim so these breakpoints behave like its.  Decodes as EVTA=halt,
 * BBM (break before the instruction runs), SUSP, TYP=1 (compare the PC).
 */
#define TREVT_HALT_ON_ADDR  0x0000102Au
#define TREVT_RNG           (1u << 13)
#define TREVT_AST           (1u << 27)   /* store: trigger on writes */
#define TREVT_ALD           (1u << 28)   /* load: trigger on reads */

/* Each core's own system timer, hardwired to that core's suspend-out signal. */
#define STM_BASE(core)      (0xF0001000u + 0x100u * (core))
#define STM_OCS             0xE8u
/* SUS_P is a write-only key that must accompany any change to SUS; SUS=2 is
 * hard suspend, which stops the counter while the core is halted. */
#define STM_OCS_SUS_HARD    ((1u << 28) | (0x2u << 24))
#define STM_OCS_SUS_OFF     (1u << 28)

/* Reset vectors a halt-after-reset trigger would point at. */
#define RESET_VECTOR_CACHED     0x80020000u
#define RESET_VECTOR_NONCACHED  0xA0020000u

/* ------------------------------------------------------------------------ */

static bool         s_present[TRICORE_MAX_CORES];
static int          s_count;
static tricore_bp_t s_slots[TRICORE_MAX_CORES][TRICORE_NUM_TRIGGERS];

static inline bool core_ok(int core)
{
    return core >= 0 && core < TRICORE_MAX_CORES && s_present[core];
}

static inline uint32_t core_reg(int core, uint32_t offset)
{
    return k_core_base[core] + offset;
}

static esp_err_t rd(int core, uint32_t offset, uint32_t *out)
{
    return dap_probe_read32(core_reg(core, offset), out);
}

static esp_err_t wr(int core, uint32_t offset, uint32_t value)
{
    return dap_probe_write32(core_reg(core, offset), value);
}

/* ------------------------------------------------------------------------ */
/* Discovery                                                                 */
/* ------------------------------------------------------------------------ */

esp_err_t tricore_discover(void)
{
    uint32_t ostate = 0;

    memset(s_present, 0, sizeof(s_present));
    memset(s_slots, 0, sizeof(s_slots));
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
            s_present[core] = true;
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
        if (s_present[core] && n-- == 0) {
            return core;
        }
    }
    return -1;
}

bool tricore_core_present(int core)
{
    return core_ok(core);
}

/* ------------------------------------------------------------------------ */
/* State                                                                     */
/* ------------------------------------------------------------------------ */

esp_err_t tricore_dbgsr(int core, uint32_t *out)
{
    if (!core_ok(core) || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    /*
     * Retried, because a device coming out of reset briefly fails every
     * transaction and a poll loop must not treat that as fatal.
     */
    for (int attempt = 0; attempt < 3; attempt++) {
        if (rd(core, OFF_DBGSR, out) == ESP_OK) {
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

static bool wait_halted(int core, bool want, uint32_t timeout_ms)
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
    if (!core_ok(core)) {
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
    esp_err_t err = wr(core, OFF_EXEVT, EXEVT_HALT_AND_SUSPEND);
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
    if (!core_ok(core)) {
        return;
    }

    uint32_t dbgsr = 0, exevt = 0, trc = 0, tlc = 0, ostate = 0;

    rd(core, OFF_DBGSR, &dbgsr);
    rd(core, OFF_EXEVT, &exevt);
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
    if (!core_ok(core) || s_halt_line[core] <= 0) {
        return;
    }
    /* Unconditional: a line left forced active stops the *next* halt from
     * being an edge, so giving up on a halt has to undo it too. */
    dap_probe_write32(CBS_TLC, s_halt_tlc[core]);
    s_halt_line[core] = 0;
}

bool tricore_halt_poll(int core)
{
    if (!core_ok(core)) {
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
    if (!core_ok(core)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (tricore_is_halted(core)) {
        return ESP_OK;
    }
    const esp_err_t err = tricore_halt_request(core, line);
    if (err != ESP_OK) {
        return err;
    }
    const bool stopped = wait_halted(core, true, timeout_ms);

    /* Release the line whatever happened. */
    if (s_halt_line[core] > 0) {
        dap_probe_write32(CBS_TLC, s_halt_tlc[core]);
        s_halt_line[core] = 0;
    }
    return stopped ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t tricore_request_resume(int core)
{
    if (!core_ok(core)) {
        return ESP_ERR_INVALID_ARG;
    }
    /*
     * HALT = 0b10 including the mask bit, i.e. 0x04.  DE is read-only, so this
     * cannot disturb it.
     */
    return wr(core, OFF_DBGSR, HALT_REQ_CLEAR << DBGSR_HALT_SHIFT);
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
    return wait_halted(core, false, timeout_ms) ? ESP_OK : ESP_ERR_TIMEOUT;
}

void tricore_clear_debug_events(int core)
{
    if (!core_ok(core)) {
        return;
    }
    /*
     * A safety net for a session that died mid-pause: EXEVT left armed halts
     * the core again the moment anything drives its trigger line, and CREVT
     * would stop it on every register access.
     */
    wr(core, OFF_CREVT, 0);
    wr(core, OFF_EXEVT, 0);
    wr(core, OFF_SWEVT, 0);
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
    if (!core_ok(core)) {
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

static esp_err_t require_halted(int core)
{
    if (!core_ok(core)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!tricore_is_halted(core)) {
        /*
         * Not a courtesy check.  While the core runs, these reads bus-error,
         * and a bus error mid-sequence leaves the IOClient in Error State with
         * every later access silently dropped.
         */
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static uint32_t reg_offset(int regno)
{
    if (regno >= TRICORE_REG_D0 && regno < TRICORE_REG_D0 + 16) {
        return OFF_D0 + 4u * (uint32_t)(regno - TRICORE_REG_D0);
    }
    if (regno >= TRICORE_REG_A0 && regno < TRICORE_REG_A0 + 16) {
        return OFF_A0 + 4u * (uint32_t)(regno - TRICORE_REG_A0);
    }
    switch (regno) {
    case TRICORE_REG_LCX:    return OFF_LCX;
    case TRICORE_REG_FCX:    return OFF_FCX;
    case TRICORE_REG_PCXI:   return OFF_PCXI;
    case TRICORE_REG_PSW:    return OFF_PSW;
    case TRICORE_REG_PC:     return OFF_PC;
    case TRICORE_REG_ICR:    return OFF_ICR;
    case TRICORE_REG_ISP:    return OFF_ISP;
    case TRICORE_REG_BTV:    return OFF_BTV;
    case TRICORE_REG_BIV:    return OFF_BIV;
    case TRICORE_REG_SYSCON: return OFF_SYSCON;
    case TRICORE_REG_PCON0:  return OFF_PCON0;
    case TRICORE_REG_DCON0:  return OFF_DCON0;
    default:                 return 0;
    }
}

esp_err_t tricore_read_reg(int core, int regno, uint32_t *value)
{
    const esp_err_t err = require_halted(core);
    if (err != ESP_OK) {
        return err;
    }
    if (regno < 0 || regno >= TRICORE_NUM_REGS || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return rd(core, reg_offset(regno), value);
}

esp_err_t tricore_write_reg(int core, int regno, uint32_t value)
{
    const esp_err_t err = require_halted(core);
    if (err != ESP_OK) {
        return err;
    }
    if (regno < 0 || regno >= TRICORE_NUM_REGS) {
        return ESP_ERR_INVALID_ARG;
    }
    return wr(core, reg_offset(regno), value);
}

esp_err_t tricore_read_all_regs(int core, uint32_t regs[TRICORE_NUM_REGS])
{
    const esp_err_t err = require_halted(core);
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < TRICORE_NUM_REGS; i++) {
        if (rd(core, reg_offset(i), &regs[i]) != ESP_OK) {
            /*
             * Report what was read rather than failing the whole packet: GDB
             * showing fifteen registers and one "unavailable" is far more use
             * than GDB showing an error.
             */
            regs[i] = 0xFFFFFFFFu;
            dap_probe_clear_error_state();
        }
    }
    return ESP_OK;
}

esp_err_t tricore_read_pc(int core, uint32_t *pc)
{
    return tricore_read_reg(core, TRICORE_REG_PC, pc);
}

esp_err_t tricore_write_pc(int core, uint32_t pc)
{
    return tricore_write_reg(core, TRICORE_REG_PC, pc);
}

esp_err_t tricore_read_a(int core, int n, uint32_t *value)
{
    if (n < 0 || n > 15) {
        return ESP_ERR_INVALID_ARG;
    }
    return tricore_read_reg(core, TRICORE_REG_A0 + n, value);
}

/* ------------------------------------------------------------------------ */
/* Triggers                                                                  */
/* ------------------------------------------------------------------------ */

esp_err_t tricore_set_code_trigger(int core, int slot, uint32_t addr)
{
    if (!core_ok(core) || slot < 0 || slot >= TRICORE_NUM_TRIGGERS) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Address first, event word last: arming last avoids a window where the
     * trigger is live on a stale address. */
    esp_err_t err = wr(core, OFF_TR_ADR(slot), addr);
    if (err != ESP_OK) {
        return err;
    }
    return wr(core, OFF_TR_EVT(slot), TREVT_HALT_ON_ADDR);
}

/*
 * The TRnEVT word for a data watchpoint.  Differences from the instruction
 * trigger, all deliberate:
 *
 *   - TYP stays clear, selecting the data address bus rather than the PC.
 *     Setting it by accident turns a watchpoint into a breakpoint on an address
 *     that is never executed, so it simply never fires.
 *   - BBM stays clear, i.e. break after make: the store completes before the
 *     core halts, so the new value is the one you read - which is what "break
 *     on value change" is expected to show.
 *   - SUSP is set, matching the instruction trigger, so the timers freeze too.
 */
static uint32_t data_trigger_word(bool on_read, bool on_write, bool is_range)
{
    uint32_t word = EVTA_HALT | EVT_SUSP;

    if (on_read)  { word |= TREVT_ALD; }
    if (on_write) { word |= TREVT_AST; }
    if (is_range) { word |= TREVT_RNG; }
    return word;
}

esp_err_t tricore_set_data_trigger(int core, int slot, uint32_t addr,
                                   uint32_t size, bool on_read, bool on_write)
{
    if (!core_ok(core) || slot < 0 || slot >= TRICORE_NUM_TRIGGERS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (size < 1 || (!on_read && !on_write)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (size == 1) {
        esp_err_t err = wr(core, OFF_TR_ADR(slot), addr);
        if (err != ESP_OK) {
            return err;
        }
        return wr(core, OFF_TR_EVT(slot), data_trigger_word(on_read, on_write, false));
    }

    /*
     * Anything wider than a byte needs a range, which the hardware builds from
     * an even/odd pair: the even register holds the lower bound, the odd one
     * the upper, and the odd register's own configuration is ignored.  Ranges
     * matter because the comparator matches one address, so a four-byte
     * variable written a byte at a time would only trip on the byte at its base.
     */
    if ((slot % 2) != 0 || slot + 1 >= TRICORE_NUM_TRIGGERS) {
        return ESP_ERR_INVALID_ARG;
    }
    wr(core, OFF_TR_ADR(slot), addr);
    wr(core, OFF_TR_ADR(slot + 1), addr + size);
    wr(core, OFF_TR_EVT(slot + 1), 0);
    return wr(core, OFF_TR_EVT(slot), data_trigger_word(on_read, on_write, true));
}

esp_err_t tricore_clear_trigger(int core, int slot)
{
    if (!core_ok(core) || slot < 0 || slot >= TRICORE_NUM_TRIGGERS) {
        return ESP_ERR_INVALID_ARG;
    }
    wr(core, OFF_TR_EVT(slot), 0);
    return wr(core, OFF_TR_ADR(slot), 0);
}

esp_err_t tricore_trigger_acc(int core, uint32_t *bits)
{
    if (!core_ok(core) || bits == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const esp_err_t err = rd(core, OFF_TRIG_ACC, bits);
    *bits &= 0xFFu;
    return err;
}

void tricore_disarm_reset_trigger(void)
{
    /*
     * A debugger that offered halt-after-reset implements it by arming TR0 at
     * the reset vector, and leaves it armed.  Without clearing it every resume
     * immediately re-halts there.
     */
    for (int core = 0; core < TRICORE_MAX_CORES; core++) {
        if (!s_present[core]) {
            continue;
        }
        uint32_t evt = 0, adr = 0;
        if (rd(core, OFF_TR_EVT(0), &evt) != ESP_OK ||
            rd(core, OFF_TR_ADR(0), &adr) != ESP_OK) {
            dap_probe_clear_error_state();
            continue;
        }
        if (evt != 0 && (adr == RESET_VECTOR_CACHED || adr == RESET_VECTOR_NONCACHED)) {
            ESP_LOGI(TAG, "CPU%d: clearing the halt-after-reset trigger at 0x%08" PRIX32,
                     core, adr);
            tricore_clear_trigger(core, 0);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Breakpoint allocation                                                     */
/* ------------------------------------------------------------------------ */

static int free_slot(int core)
{
    for (int n = 0; n < TRICORE_NUM_TRIGGERS; n++) {
        if (s_slots[core][n].kind == TRICORE_BP_FREE) {
            return n;
        }
    }
    return -1;
}

static int free_pair(int core)
{
    for (int n = 0; n + 1 < TRICORE_NUM_TRIGGERS; n += 2) {
        if (s_slots[core][n].kind == TRICORE_BP_FREE &&
            s_slots[core][n + 1].kind == TRICORE_BP_FREE) {
            return n;
        }
    }
    return -1;
}

static int slot_of_addr(int core, uint32_t addr)
{
    for (int n = 0; n < TRICORE_NUM_TRIGGERS; n++) {
        if (s_slots[core][n].kind != TRICORE_BP_FREE && s_slots[core][n].addr == addr) {
            return n;
        }
    }
    return -1;
}

int tricore_bp_add(int core, uint32_t addr, tricore_bp_kind_t kind)
{
    if (!core_ok(core)) {
        return -1;
    }
    const int existing = slot_of_addr(core, addr);
    if (existing >= 0) {
        return existing;
    }
    const int slot = free_slot(core);
    if (slot < 0) {
        return -1;              /* out of triggers; the caller reports it */
    }
    if (tricore_set_code_trigger(core, slot, addr) != ESP_OK) {
        dap_probe_clear_error_state();
        return -1;
    }
    s_slots[core][slot] = (tricore_bp_t){ .addr = addr, .kind = kind };
    return slot;
}

int tricore_bp_add_watch(int core, uint32_t addr, uint32_t size,
                         bool on_read, bool on_write)
{
    if (!core_ok(core)) {
        return -1;
    }
    const int existing = slot_of_addr(core, addr);
    if (existing >= 0) {
        return existing;
    }
    const bool needs_pair = size > 1;
    const int slot = needs_pair ? free_pair(core) : free_slot(core);
    if (slot < 0) {
        return -1;
    }
    if (tricore_set_data_trigger(core, slot, addr, size, on_read, on_write) != ESP_OK) {
        dap_probe_clear_error_state();
        return -1;
    }
    s_slots[core][slot] = (tricore_bp_t){
        .addr = addr, .size = size, .kind = TRICORE_BP_WATCH,
        .on_read = on_read, .on_write = on_write,
    };
    if (needs_pair) {
        /* Hold the odd trigger so nothing else takes it; the hardware ignores
         * its configuration while the even one is in range mode. */
        s_slots[core][slot + 1] = (tricore_bp_t){
            .addr = addr + size, .kind = TRICORE_BP_WATCH_HI,
        };
    }
    return slot;
}

bool tricore_bp_remove(int core, uint32_t addr)
{
    if (!core_ok(core)) {
        return false;
    }
    const int slot = slot_of_addr(core, addr);
    if (slot < 0) {
        return false;
    }
    const bool had_pair = (s_slots[core][slot].kind == TRICORE_BP_WATCH) &&
                          (s_slots[core][slot].size > 1);
    tricore_clear_trigger(core, slot);
    s_slots[core][slot] = (tricore_bp_t){ .kind = TRICORE_BP_FREE };
    if (had_pair && slot + 1 < TRICORE_NUM_TRIGGERS) {
        tricore_clear_trigger(core, slot + 1);
        s_slots[core][slot + 1] = (tricore_bp_t){ .kind = TRICORE_BP_FREE };
    }
    return true;
}

void tricore_bp_clear_kind(int core, tricore_bp_kind_t kind)
{
    if (!core_ok(core)) {
        return;
    }
    for (int n = 0; n < TRICORE_NUM_TRIGGERS; n++) {
        if (s_slots[core][n].kind == kind) {
            tricore_clear_trigger(core, n);
            s_slots[core][n] = (tricore_bp_t){ .kind = TRICORE_BP_FREE };
        }
    }
}

int tricore_bp_free_slots(int core)
{
    if (!core_ok(core)) {
        return 0;
    }
    int n = 0;
    for (int slot = 0; slot < TRICORE_NUM_TRIGGERS; slot++) {
        n += (s_slots[core][slot].kind == TRICORE_BP_FREE);
    }
    return n;
}

const tricore_bp_t *tricore_bp_of_slot(int core, int slot)
{
    if (!core_ok(core) || slot < 0 || slot >= TRICORE_NUM_TRIGGERS) {
        return NULL;
    }
    return s_slots[core][slot].kind == TRICORE_BP_FREE ? NULL : &s_slots[core][slot];
}

const tricore_bp_t *tricore_bp_at(int core, uint32_t addr)
{
    if (!core_ok(core)) {
        return NULL;
    }
    const int slot = slot_of_addr(core, addr);
    return slot < 0 ? NULL : &s_slots[core][slot];
}

/* ------------------------------------------------------------------------ */
/* Control-flow decoding, for stepping                                       */
/* ------------------------------------------------------------------------ */
/*
 * TriCore has no single-step control bit reachable this way, so a step is
 * "breakpoint on the successor instruction(s), run, stop".  The successors are
 * the fall-through, always, and the branch target when the instruction is a
 * direct branch we can decode.  For indirect flow (JI, CALLI, RET) the target
 * comes from an address register, which is readable because the core is halted.
 */

/* 32-bit B format, disp24: target = PC + sign_extend(disp24) * 2 */
static bool is_b_format(uint8_t op)
{
    return op == 0x1D || op == 0x5D || op == 0x6D || op == 0x61;  /* J JL CALL FCALL */
}

/* 32-bit BRC/BRR/BRN formats, disp15 */
static bool is_disp15(uint8_t op)
{
    switch (op) {
    case 0x1F: case 0x3F: case 0x5F: case 0x7F: case 0x9F:
    case 0xBF: case 0xDF: case 0xEF: case 0xFF: case 0x6F:
        return true;
    default:
        return false;
    }
}

/* 16-bit SB format, disp8 */
static bool is_sb(uint8_t op)
{
    switch (op) {
    case 0x3C: case 0xEE: case 0xF6: case 0x6E: case 0x76: case 0x5C:
        return true;
    default:
        return false;
    }
}

/* 16-bit SBR format, disp4 (zero extended) */
static bool is_sbr(uint8_t op)
{
    switch (op) {
    case 0x3E: case 0xBE: case 0x7E: case 0xFE: case 0xF2: case 0x7C:
        return true;
    default:
        return false;
    }
}

static int32_t sign_extend(uint32_t value, int bits)
{
    const uint32_t sign = 1u << (bits - 1);
    return (int32_t)((value ^ sign) - sign);
}

/*
 * Where execution can go from the instruction at `pc`.  `target_valid` is false
 * for straight-line code.  Reads an address register for indirect flow, which
 * is safe here because the core is halted.
 */
static void decode_successors(int core, uint32_t pc, const uint8_t raw[4],
                              uint32_t *fall_through, uint32_t *target,
                              bool *target_valid)
{
    const uint8_t op = raw[0];
    const uint32_t width = (op & 1u) ? 4u : 2u;

    *fall_through = pc + width;
    *target_valid = false;

    if (width == 4) {
        const uint32_t insn = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
                              ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
        if (is_b_format(op)) {
            /* disp24 is split: [23:16] from insn[15:8], [15:0] from insn[31:16] */
            const uint32_t disp24 = (((insn >> 8) & 0xFFu) << 16) | ((insn >> 16) & 0xFFFFu);
            *target = (uint32_t)((int32_t)pc + sign_extend(disp24, 24) * 2);
            *target_valid = true;
        } else if (is_disp15(op)) {
            const uint32_t disp15 = (insn >> 16) & 0x7FFFu;
            *target = (uint32_t)((int32_t)pc + sign_extend(disp15, 15) * 2);
            *target_valid = true;
        } else if (op == 0x2D) {                    /* JI / CALLI, indirect */
            uint32_t a = 0;
            if (tricore_read_a(core, (int)((insn >> 8) & 0xFu), &a) == ESP_OK) {
                *target = a;
                *target_valid = true;
            }
        }
        return;
    }

    const uint32_t insn16 = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8);
    if (is_sb(op)) {
        const uint32_t disp8 = (insn16 >> 8) & 0xFFu;
        *target = (uint32_t)((int32_t)pc + sign_extend(disp8, 8) * 2);
        *target_valid = true;
    } else if (is_sbr(op)) {
        const uint32_t disp4 = (insn16 >> 12) & 0xFu;
        *target = pc + disp4 * 2u;                  /* zero extended */
        *target_valid = true;
    } else if (op == 0xDC) {                        /* JI A[a] */
        uint32_t a = 0;
        if (tricore_read_a(core, (int)((insn16 >> 12) & 0xFu), &a) == ESP_OK) {
            *target = a;
            *target_valid = true;
        }
    } else if (op == 0x00 && ((insn16 >> 8) & 0xFFu) == 0x90) {   /* RET */
        uint32_t a11 = 0;
        if (tricore_read_a(core, 11, &a11) == ESP_OK) {
            *target = a11;
            *target_valid = true;
        }
    }
}

esp_err_t tricore_step(int core, uint32_t timeout_ms, tricore_step_t *out)
{
    if (!core_ok(core) || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    uint32_t pc = 0;
    esp_err_t err = tricore_read_pc(core, &pc);
    if (err != ESP_OK) {
        snprintf(out->note, sizeof(out->note), "the core is running");
        return err;
    }

    uint8_t raw[4] = {0};
    if (tricore_read_mem(pc, raw, sizeof(raw)) != ESP_OK) {
        snprintf(out->note, sizeof(out->note), "could not read the instruction");
        out->pc = pc;
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t fall_through = 0, target = 0;
    bool target_valid = false;
    decode_successors(core, pc, raw, &fall_through, &target, &target_valid);

    tricore_bp_clear_kind(core, TRICORE_BP_STEP);

    /*
     * A trigger armed at the current PC fires again the instant we resume, so
     * the core never leaves this instruction.  Step off it and put it back
     * afterwards.
     */
    uint32_t          evicted_addr = 0;
    tricore_bp_kind_t evicted_kind = TRICORE_BP_FREE;
    const tricore_bp_t *here = tricore_bp_at(core, pc);
    if (here != NULL && here->kind != TRICORE_BP_STEP) {
        evicted_addr = here->addr;
        evicted_kind = here->kind;
        tricore_bp_remove(core, pc);
    }

    const int armed_fall = tricore_bp_add(core, fall_through, TRICORE_BP_STEP);
    int armed_target = -1;
    if (target_valid && target != fall_through) {
        armed_target = tricore_bp_add(core, target, TRICORE_BP_STEP);
    }
    if (armed_fall < 0 || (target_valid && target != fall_through && armed_target < 0)) {
        snprintf(out->note, sizeof(out->note),
                 "out of triggers; the stop may be imprecise");
    }

    /*
     * request_resume rather than resume: a trigger is armed one instruction
     * ahead, so the core re-halts faster than a wait could see it running.
     */
    tricore_request_resume(core);

    const bool stopped = wait_halted(core, true, timeout_ms);
    if (!stopped) {
        /* No successor was reached - an instruction we could not decode
         * branched somewhere else.  Force a halt so the session survives. */
        if (tricore_halt(core, 1, 200) != ESP_OK) {
            snprintf(out->note, sizeof(out->note), "the core could not be stopped");
            tricore_bp_clear_kind(core, TRICORE_BP_STEP);
            return ESP_ERR_TIMEOUT;
        }
        snprintf(out->note, sizeof(out->note),
                 "no successor was reached; forced a halt");
    } else {
        out->clean = true;
    }

    tricore_bp_clear_kind(core, TRICORE_BP_STEP);
    if (evicted_kind != TRICORE_BP_FREE) {
        tricore_bp_add(core, evicted_addr, evicted_kind);
    }

    if (tricore_read_pc(core, &out->pc) != ESP_OK) {
        out->pc = pc;
    }
    /* A step can end on somebody else's breakpoint, and calling that "step"
     * hides why the core is where it is. */
    const tricore_bp_t *landed = tricore_bp_at(core, out->pc);
    out->hit_user_bp = (landed != NULL && landed->kind == TRICORE_BP_USER);
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Memory                                                                    */
/* ------------------------------------------------------------------------ */

esp_err_t tricore_read_mem(uint32_t addr, uint8_t *buf, size_t len)
{
    if (buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    while (len) {
        const uint32_t word_addr = addr & ~3u;
        const size_t   skip      = addr - word_addr;
        const size_t   take      = (4u - skip) < len ? (4u - skip) : len;

        uint32_t word = 0;
        if (dap_probe_read32(word_addr, &word) != ESP_OK) {
            dap_probe_clear_error_state();
            return ESP_FAIL;
        }
        for (size_t i = 0; i < take; i++) {
            buf[i] = (uint8_t)(word >> (8u * (skip + i)));
        }
        addr += take;
        buf  += take;
        len  -= take;
    }
    return ESP_OK;
}

esp_err_t tricore_write_mem(uint32_t addr, const uint8_t *buf, size_t len)
{
    if (buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    while (len) {
        const uint32_t word_addr = addr & ~3u;
        const size_t   skip      = addr - word_addr;
        const size_t   take      = (4u - skip) < len ? (4u - skip) : len;

        uint32_t word = 0;
        if (take != 4) {
            /*
             * Read-modify-write, because the probe exposes word writes only.
             * Right for RAM, wrong for a register with side effects on read -
             * a GDB user writing one byte into a peripheral should know that.
             */
            if (dap_probe_read32(word_addr, &word) != ESP_OK) {
                dap_probe_clear_error_state();
                return ESP_FAIL;
            }
        }
        for (size_t i = 0; i < take; i++) {
            const unsigned shift = 8u * (unsigned)(skip + i);
            word &= ~(0xFFu << shift);
            word |= (uint32_t)buf[i] << shift;
        }
        if (dap_probe_write32(word_addr, word) != ESP_OK) {
            dap_probe_clear_error_state();
            return ESP_FAIL;
        }
        addr += take;
        buf  += take;
        len  -= take;
    }
    return ESP_OK;
}
