/*
 * TriCore registers, memory and the debug triggers, all of which need the core
 * halted.
 */

#include <inttypes.h>
#include <string.h>

#include "dap_probe.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tricore.h"
#include "tricore_priv.h"

static const char *TAG = "TRICORE";

/* What each trigger slot is being used for, per core. */
static tricore_bp_t s_slots[TRICORE_MAX_CORES][TRICORE_NUM_TRIGGERS];

static esp_err_t require_halted(int core)
{
    if (!tricore_core_ok(core)) {
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
    return tricore_rd(core, reg_offset(regno), value);
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
    return tricore_wr(core, reg_offset(regno), value);
}

esp_err_t tricore_read_all_regs(int core, uint32_t regs[TRICORE_NUM_REGS])
{
    const esp_err_t err = require_halted(core);
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < TRICORE_NUM_REGS; i++) {
        if (tricore_rd(core, reg_offset(i), &regs[i]) != ESP_OK) {
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
    if (!tricore_core_ok(core) || slot < 0 || slot >= TRICORE_NUM_TRIGGERS) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Address first, event word last: arming last avoids a window where the
     * trigger is live on a stale address. */
    esp_err_t err = tricore_wr(core, OFF_TR_ADR(slot), addr);
    if (err != ESP_OK) {
        return err;
    }
    return tricore_wr(core, OFF_TR_EVT(slot), TREVT_HALT_ON_ADDR);
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
    if (!tricore_core_ok(core) || slot < 0 || slot >= TRICORE_NUM_TRIGGERS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (size < 1 || (!on_read && !on_write)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (size == 1) {
        esp_err_t err = tricore_wr(core, OFF_TR_ADR(slot), addr);
        if (err != ESP_OK) {
            return err;
        }
        return tricore_wr(core, OFF_TR_EVT(slot), data_trigger_word(on_read, on_write, false));
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
    tricore_wr(core, OFF_TR_ADR(slot), addr);
    tricore_wr(core, OFF_TR_ADR(slot + 1), addr + size);
    tricore_wr(core, OFF_TR_EVT(slot + 1), 0);
    return tricore_wr(core, OFF_TR_EVT(slot), data_trigger_word(on_read, on_write, true));
}

esp_err_t tricore_clear_trigger(int core, int slot)
{
    if (!tricore_core_ok(core) || slot < 0 || slot >= TRICORE_NUM_TRIGGERS) {
        return ESP_ERR_INVALID_ARG;
    }
    tricore_wr(core, OFF_TR_EVT(slot), 0);
    return tricore_wr(core, OFF_TR_ADR(slot), 0);
}

esp_err_t tricore_trigger_acc(int core, uint32_t *bits)
{
    if (!tricore_core_ok(core) || bits == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const esp_err_t err = tricore_rd(core, OFF_TRIG_ACC, bits);
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
        if (!tricore_present[core]) {
            continue;
        }
        uint32_t evt = 0, adr = 0;
        if (tricore_rd(core, OFF_TR_EVT(0), &evt) != ESP_OK ||
            tricore_rd(core, OFF_TR_ADR(0), &adr) != ESP_OK) {
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
    if (!tricore_core_ok(core)) {
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
    if (!tricore_core_ok(core)) {
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
    if (!tricore_core_ok(core)) {
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
    if (!tricore_core_ok(core)) {
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
    if (!tricore_core_ok(core)) {
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
    if (!tricore_core_ok(core) || slot < 0 || slot >= TRICORE_NUM_TRIGGERS) {
        return NULL;
    }
    return s_slots[core][slot].kind == TRICORE_BP_FREE ? NULL : &s_slots[core][slot];
}

const tricore_bp_t *tricore_bp_at(int core, uint32_t addr)
{
    if (!tricore_core_ok(core)) {
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
    if (!tricore_core_ok(core) || out == NULL) {
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

    const bool stopped = tricore_wait_halted(core, true, timeout_ms);
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

/* Drop the record of what is armed; discovery calls this because a reset takes
 * the triggers with it. */
void tricore_forget_triggers(void)
{
    memset(s_slots, 0, sizeof(s_slots));
}
