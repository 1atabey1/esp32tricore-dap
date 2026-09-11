/*
 * TriCore (AURIX TC3xx) run control over the DAP probe.
 *
 * Everything here is plain memory-mapped CSFR access through dap_probe_read32
 * and dap_probe_write32; the device has no run-control protocol of its own.
 * The register addresses and, more importantly, the behaviour below were
 * derived and verified against a live TC38x by the tas-debug project
 * (src/tas_debug/runcontrol/tricore_core.py), and none of it is obvious:
 *
 *   - DBGSR.HALT is two bits at [2:1], and bit 2 is a *write mask*: HALT[0]
 *     only changes when it is set.  Writing 0b01 to halt is a no-op, which is
 *     why halting a running core once looked impossible.  Resume is 0b10.
 *
 *   - A DBGSR write is not a debug *event*, so it cannot assert the core's
 *     suspend-out signal, and peripherals that watch suspend-out keep running.
 *     That matters more than it sounds: the STM keeps counting through the
 *     pause, so a tick handler that reprograms its compare as "previous +
 *     period" writes a compare already in the past, and a 32-bit compare only
 *     matches on equality - the next tick then arrives when the timer wraps,
 *     about 43 s at 100 MHz.  So pausing goes the long way round, through
 *     EXEVT and a Cerberus trigger line.
 *
 *   - OCDS must be enabled first.  Without it every DBGSR write is silently
 *     ignored and nothing reports an error.
 *
 *   - While a core is running, reads of PC, PSW, PCXI and the GPRs bus-error.
 *     Only DBGSR may be polled.
 *
 *   - TriCore has no single-step control bit reachable this way, so a step is
 *     "arm a trigger on each successor instruction, resume, wait".
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TC3xx tops out at six cores; a TC38x has four. */
#define TRICORE_MAX_CORES     6

/* Address comparators per core, shared by breakpoints, watchpoints and steps. */
#define TRICORE_NUM_TRIGGERS  8

/* GDB register numbers, which this project defines because it also serves the
 * target description: D0..D15, A0..A15, then PCXI, PSW, PC. */
#define TRICORE_REG_D0        0
#define TRICORE_REG_A0        16
#define TRICORE_REG_PCXI      32
#define TRICORE_REG_PSW       33
#define TRICORE_REG_PC        34
#define TRICORE_NUM_REGS      35

typedef enum {
    TRICORE_BP_FREE = 0,
    TRICORE_BP_USER,      /* a breakpoint GDB asked for */
    TRICORE_BP_STEP,      /* short-lived, on a step's successor */
    TRICORE_BP_WATCH,     /* a data watchpoint */
    TRICORE_BP_WATCH_HI,  /* the odd half of a range watchpoint pair */
} tricore_bp_kind_t;

typedef struct {
    uint32_t          addr;
    uint32_t          size;      /* watchpoints only */
    tricore_bp_kind_t kind;
    bool              on_read;
    bool              on_write;
} tricore_bp_t;

typedef struct {
    uint32_t pc;
    bool     clean;        /* false when no successor was reached and we forced a halt */
    bool     hit_user_bp;  /* the step ran onto a breakpoint GDB set */
    char     note[64];
} tricore_step_t;

/*
 * Find which cores answer on their CSFR window.  Needs an attached target with
 * OCDS already enabled - dap_probe_enable_ocds() - because without it the whole
 * window reads back as bus errors and every core looks absent.
 */
esp_err_t tricore_discover(void);

int  tricore_core_count(void);
/* Core index (0..5) of the n-th present core, or -1. */
int  tricore_core_index(int n);
bool tricore_core_present(int core);

/* DBGSR, the only register that is safe to read while the core is running. */
esp_err_t tricore_dbgsr(int core, uint32_t *out);
bool      tricore_is_halted(int core);
bool      tricore_debug_enabled(int core);

/*
 * Halt through this core's EXEVT and a Cerberus trigger line, so the halt is a
 * debug event and suspend-out is asserted with it.  `line` is 1..7.
 */
esp_err_t tricore_halt(int core, int line, uint32_t timeout_ms);

/*
 * The same halt, split so a caller that polls can use it.
 *
 * tricore_halt_request() drives the trigger line and returns immediately;
 * tricore_halt_poll() reports whether the core has stopped and releases the
 * line once it has.  The release matters and is why this is a pair rather than
 * one call: a line left driven keeps every core routed to it halted, so it
 * cannot simply be dropped when the request returns.
 */
esp_err_t tricore_halt_request(int core, int line);
bool      tricore_halt_poll(int core);

/* Clear the halt request and wait for the core to run again. */
esp_err_t tricore_resume(int core, uint32_t timeout_ms);

/*
 * Clear the halt request without waiting.  Needed whenever a trigger is armed
 * at or just after the PC: the core re-halts faster than we can see it run, so
 * waiting always times out.
 */
esp_err_t tricore_request_resume(int core);

/* Disarm EXEVT, CREVT and SWEVT, for a session that died mid-pause. */
void tricore_clear_debug_events(int core);

/* Stop this core's STM while the core is halted; see the file comment. */
esp_err_t tricore_freeze_timer(int core, bool enable);

/* Registers.  All of these need the core halted and say so if it is not. */
esp_err_t tricore_read_pc(int core, uint32_t *pc);
esp_err_t tricore_write_pc(int core, uint32_t pc);
esp_err_t tricore_read_a(int core, int n, uint32_t *value);
esp_err_t tricore_read_all_regs(int core, uint32_t regs[TRICORE_NUM_REGS]);
esp_err_t tricore_read_reg(int core, int regno, uint32_t *value);
esp_err_t tricore_write_reg(int core, int regno, uint32_t value);

/* Triggers, addressed by slot.  The allocator below is what callers want. */
esp_err_t tricore_set_code_trigger(int core, int slot, uint32_t addr);
esp_err_t tricore_set_data_trigger(int core, int slot, uint32_t addr,
                                   uint32_t size, bool on_read, bool on_write);
esp_err_t tricore_clear_trigger(int core, int slot);

/*
 * Which triggers have tripped since this was last read.  Reading clears it, so
 * read it once per stop and keep the answer.
 */
esp_err_t tricore_trigger_acc(int core, uint32_t *bits);

/*
 * Remove the halt-after-reset trigger a debugger may have left on TR0.  Only
 * triggers actually pointing at a reset vector are cleared, so a user
 * breakpoint that happens to live in TR0 is left alone.
 */
void tricore_disarm_reset_trigger(void);

/* Breakpoint allocation over this core's triggers. */
int  tricore_bp_add(int core, uint32_t addr, tricore_bp_kind_t kind);
int  tricore_bp_add_watch(int core, uint32_t addr, uint32_t size,
                          bool on_read, bool on_write);
bool tricore_bp_remove(int core, uint32_t addr);
void tricore_bp_clear_kind(int core, tricore_bp_kind_t kind);
int  tricore_bp_free_slots(int core);
/* The breakpoint occupying a slot, or NULL.  For turning a trigger number from
 * the accumulator into something to report. */
const tricore_bp_t *tricore_bp_of_slot(int core, int slot);
const tricore_bp_t *tricore_bp_at(int core, uint32_t addr);

/*
 * One machine instruction.  Arms a trigger on each successor - the fall-through
 * always, and the branch target when the instruction is one we can decode -
 * resumes, and waits.  Needs free triggers, and gives up user breakpoints
 * temporarily when it has to, restoring them afterwards.
 */
esp_err_t tricore_step(int core, uint32_t timeout_ms, tricore_step_t *out);

/*
 * Memory, byte-granular, over the probe's word accesses.
 *
 * Note on writes: a partial word is a read-modify-write, because the probe
 * exposes word writes only.  That is right for RAM and wrong for registers
 * with side effects on read, so GDB writing a single byte into a peripheral
 * may not do what it looks like.
 */
esp_err_t tricore_read_mem(uint32_t addr, uint8_t *buf, size_t len);
esp_err_t tricore_write_mem(uint32_t addr, const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
