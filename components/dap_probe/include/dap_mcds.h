/*
 * miniMCDS data / watch-point trace setup (TC3xx TS chapter 9).
 *
 * Two watch slots, each one DTU comparator set: address range (EA),
 * access mode (AC: read/write pulse) and an optional value filter (WD).
 * Full mode traces the matching accesses through DTU_TC; compact mode emits
 * one WPS watch-point message per hit through WTU_MCX.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DAP_MCDS_SLOTS 2

typedef enum {
    DAP_MCDS_SRC_CPU = 0,      /* CPUn pipeline: every load/store CPUn executes */
    DAP_MCDS_SRC_MEMSLAVE,     /* CPUn DSPR/PSPR/DLMU slave: any bus master */
    DAP_MCDS_SRC_LMU0,         /* LMU0 slave */
} dap_mcds_source_t;

typedef enum {
    DAP_MCDS_MODE_FULL = 0,    /* DTU messages: address and/or data */
    DAP_MCDS_MODE_COMPACT,     /* WTU WPS message per hit */
} dap_mcds_mode_t;

typedef enum {
    DAP_MCDS_PAYLOAD_ADDR_DATA = 0,   /* DTW / DTR */
    DAP_MCDS_PAYLOAD_DATA,            /* DTWD / DTRD */
    DAP_MCDS_PAYLOAD_ADDR,            /* DTWA / DTRA */
} dap_mcds_payload_t;

typedef enum {
    DAP_MCDS_TS_HIT = 0,       /* TSR (emulation clock) with every hit */
    DAP_MCDS_TS_TICKS,         /* DMC <tick>/<multick>: exact, adds idle heartbeat */
    DAP_MCDS_TS_NONE,          /* only one TSR per paragraph */
} dap_mcds_timestamps_t;

typedef struct {
    bool     enabled;
    char     name[24];
    uint32_t addr;             /* as the traced source sees it */
    uint32_t size;             /* bytes, >= 1; > 4 watches a range */
    bool     rd;
    bool     wr;
    bool     value_en;         /* filter on the value read/written */
    uint32_t value_lo;         /* inclusive, in the variable's own units */
    uint32_t value_hi;
    uint32_t value_mask;       /* bits of the variable that are compared */
    bool     value_signed;
} dap_mcds_slot_t;

typedef struct {
    dap_mcds_source_t     source;
    uint8_t               cpu;          /* for CPU / MEMSLAVE */
    dap_mcds_mode_t       mode;
    dap_mcds_payload_t    payload;      /* full mode only */
    dap_mcds_timestamps_t timestamps;
    bool                  masters;      /* keep bus master/SVM in addresses */
    uint8_t               dap_div;      /* fabric clock divider while tracing */
    dap_mcds_slot_t       slot[DAP_MCDS_SLOTS];
} dap_mcds_config_t;

/* What the probe measured while starting, for the decoder. */
typedef struct {
    uint32_t emu_hz;           /* emulation (TSU) clock */
    uint32_t tsu_start;        /* TSUEMUCNT when tracing was armed */
} dap_mcds_info_t;

void dap_mcds_default_config(dap_mcds_config_t *cfg);

/* Configure the miniMCDS for `cfg`, arm tracing and start the TRAM drain. */
esp_err_t dap_mcds_start(const dap_mcds_config_t *cfg, dap_mcds_info_t *info);

/* Flush, drain the last paragraph, stop the drain and reset the miniMCDS. */
esp_err_t dap_mcds_stop(void);

bool dap_mcds_running(void);

#ifdef __cplusplus
}
#endif
