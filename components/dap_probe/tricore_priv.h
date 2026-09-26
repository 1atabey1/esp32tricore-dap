#pragma once

/* Shared between tricore.c and tricore_regs.c.  Not part of the interface. */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "tricore.h"

/* Cerberus: manual offsets are relative to CBS_BASE, not 0xF0000000. */
#define CBS_BASE            0xF0000400u
#define CBS_OCNTRL          0xF000047Cu
#define CBS_OSTATE          0xF0000480u

/* OCNTRL is write-only; each control bit 2n+1 needs its protection bit 2n set
 * in the same write, or the change is silently dropped. */
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

/* Trigger line control: seven 4-bit fields, one per line. */
#define CBS_TLC             (CBS_BASE + 0x90u)
#define TLSP_NONE           0x0u
#define TLSP_FORCE_ACTIVE   0x3u

/* Per-core CSFR window: TC38x core bases. */
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

/* Remaining GDB registers, from IfxCpu_reg.h. */
#define OFF_SYSCON          0xFE14u
#define SYSCON_BHALT        (1u << 24)
#define OFF_BIV             0xFE20u
#define OFF_BTV             0xFE24u
#define OFF_ISP             0xFE28u
#define OFF_ICR             0xFE2Cu
#define OFF_FCX             0xFE38u
#define OFF_LCX             0xFE3Cu
#define OFF_DCON0           0x9040u
#define OFF_PCON0           0x920Cu

/* DBGSR.HALT is [2:1]; bit 2 is a write mask.  0b10 clears the halt. */
#define DBGSR_DE            (1u << 0)
#define DBGSR_HALT_SHIFT    1
#define HALT_REQ_CLEAR      0b10u

/* Debug event registers: EVTA in bits [2:0], SUSP at bit 5. */
#define EVTA_HALT           0b010u
#define EVT_SUSP            (1u << 5)
#define EXEVT_HALT_AND_SUSPEND (EVT_SUSP | EVTA_HALT)

/* TRnEVT for halt on PC: EVTA=halt, BBM, SUSP, TYP=1. */
#define TREVT_HALT_ON_ADDR  0x0000102Au
#define TREVT_RNG           (1u << 13)
#define TREVT_AST           (1u << 27)   /* store: trigger on writes */
#define TREVT_ALD           (1u << 28)   /* load: trigger on reads */

/* Each core's own STM, hardwired to that core's suspend-out. */
#define STM_BASE(core)      (0xF0001000u + 0x100u * (core))
#define STM_OCS             0xE8u
/* SUS_P (bit 28) keys any SUS change; SUS=2 is hard suspend. */
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
