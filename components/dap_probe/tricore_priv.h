#pragma once

/*
 * Shared between tricore.c and tricore_regs.c.  Not part of the interface.
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "tricore.h"

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

/* Which cores answered discovery. */
extern bool tricore_present[TRICORE_MAX_CORES];

bool     tricore_core_ok(int core);
uint32_t tricore_core_reg(int core, uint32_t offset);

/* One CSFR, by offset from the core's base. */
esp_err_t tricore_rd(int core, uint32_t offset, uint32_t *out);
esp_err_t tricore_wr(int core, uint32_t offset, uint32_t value);

/* Poll DBGSR until the core is halted (or running), or the timeout expires. */
bool tricore_wait_halted(int core, bool want, uint32_t timeout_ms);

/* Forget which trigger slots are armed. */
void tricore_forget_triggers(void);
