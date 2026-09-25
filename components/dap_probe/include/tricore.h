/*
 * TriCore (AURIX TC3xx) run control over the DAP probe, via memory-mapped CSFRs.
 *
 *   - OCDS must be enabled first, or every DBGSR write is silently ignored.
 *   - DBGSR.HALT is [2:1] with bit 2 a write mask: resume is 0b10.
 *   - A DBGSR halt is not a debug event and does not assert suspend-out, so
 *     halting goes through EXEVT and a Cerberus trigger line instead.
 *   - While a core runs, only DBGSR may be read; PC, PSW, PCXI and GPRs bus-error.
 *   - There is no single-step bit: a step arms triggers on the successors.
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

/* GDB register numbers, in tricore-elf-gdb's order; its architecture rejects a
 * target description that lacks any of the 44. */
#define TRICORE_REG_D0        0
#define TRICORE_REG_A0        16
#define TRICORE_REG_LCX       32
#define TRICORE_REG_FCX       33
#define TRICORE_REG_PCXI      34    /* GDB calls this one `pcx` */
#define TRICORE_REG_PSW       35
#define TRICORE_REG_PC        36
#define TRICORE_REG_ICR       37
#define TRICORE_REG_ISP       38
#define TRICORE_REG_BTV       39
#define TRICORE_REG_BIV       40
#define TRICORE_REG_SYSCON    41
#define TRICORE_REG_PCON0     42
#define TRICORE_REG_DCON0     43
#define TRICORE_NUM_REGS      44

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

/* Find which cores answer on their CSFR window.  Needs OCDS enabled. */
esp_err_t tricore_discover(void);

int  tricore_core_count(void);
/* Core index (0..5) of the n-th present core, or -1. */
int  tricore_core_index(int n);
bool tricore_core_present(int core);
/* False while SYSCON.BHALT is set: the core never started. */
bool tricore_core_started(int core);

/* DBGSR, the only register safe to read while the core is running. */
esp_err_t tricore_dbgsr(int core, uint32_t *out);
bool      tricore_is_halted(int core);

/* Halt via EXEVT and Cerberus trigger `line` (1..7), asserting suspend-out. */
esp_err_t tricore_halt(int core, int line, uint32_t timeout_ms);

/* Non-blocking halt: request drives the line; poll reports the stop and
 * releases the line (a driven line keeps every core routed to it halted). */
esp_err_t tricore_halt_request(int core, int line);
bool      tricore_halt_poll(int core);

/* Release the trigger line whether or not the core halted; a line left forced
 * active makes every later halt fail. */
void      tricore_halt_release(int core);

/* Log the registers that explain a halt that did not arrive. */
void      tricore_halt_diag(int core, const char *what);

/* Clear the halt request and wait for the core to run again. */
esp_err_t tricore_resume(int core, uint32_t timeout_ms);

/* Clear the halt request without waiting, for when a trigger at the PC would
 * re-halt the core before it is seen running. */
esp_err_t tricore_request_resume(int core);

/* Disarm EXEVT, CREVT and SWEVT, for a session that died mid-pause. */
void tricore_clear_debug_events(int core);

/*
 * Reset through OCDS.  set_halt_after_reset sets OSTATE.HARR, asking the boot
 * ROM to stop the cores at the entry point; request_application_reset restarts
 * the application while keeping the debug link and OCDS enable up.
 */
esp_err_t tricore_set_halt_after_reset(bool enable);
bool      tricore_halt_after_reset_pending(void);
esp_err_t tricore_request_application_reset(void);

/* Stop this core's STM while the core is halted. */
esp_err_t tricore_freeze_timer(int core, bool enable);

/* Registers.  All need the core halted. */
esp_err_t tricore_read_pc(int core, uint32_t *pc);
esp_err_t tricore_write_pc(int core, uint32_t pc);
esp_err_t tricore_read_a(int core, int n, uint32_t *value);
esp_err_t tricore_read_all_regs(int core, uint32_t regs[TRICORE_NUM_REGS]);
esp_err_t tricore_read_reg(int core, int regno, uint32_t *value);
esp_err_t tricore_write_reg(int core, int regno, uint32_t value);

/* Triggers by slot; callers normally use the allocator below. */
esp_err_t tricore_set_code_trigger(int core, int slot, uint32_t addr);
esp_err_t tricore_set_data_trigger(int core, int slot, uint32_t addr,
                                   uint32_t size, bool on_read, bool on_write);
esp_err_t tricore_clear_trigger(int core, int slot);

/* Triggers tripped since the last read.  Reading clears it. */
esp_err_t tricore_trigger_acc(int core, uint32_t *bits);

/* Clear a halt-after-reset trigger left on TR0, only if it points at a reset
 * vector. */
void tricore_disarm_reset_trigger(void);

/* Breakpoint allocation over this core's triggers. */
int  tricore_bp_add(int core, uint32_t addr, tricore_bp_kind_t kind);
int  tricore_bp_add_watch(int core, uint32_t addr, uint32_t size,
                          bool on_read, bool on_write);
bool tricore_bp_remove(int core, uint32_t addr);
void tricore_bp_clear_kind(int core, tricore_bp_kind_t kind);
/* The breakpoint occupying a slot, or NULL. */
const tricore_bp_t *tricore_bp_of_slot(int core, int slot);
const tricore_bp_t *tricore_bp_at(int core, uint32_t addr);

/* Step one instruction: trigger the fall-through and any decoded branch target,
 * resume, wait.  User breakpoints may be lent out and are restored. */
esp_err_t tricore_step(int core, uint32_t timeout_ms, tricore_step_t *out);

/* Byte-granular memory access.  Partial-word writes are read-modify-write. */
esp_err_t tricore_read_mem(uint32_t addr, uint8_t *buf, size_t len);
esp_err_t tricore_write_mem(uint32_t addr, const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
