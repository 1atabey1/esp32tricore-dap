/*
 * The /ws/trace sender's counters (dap_web.c), for status pages and clients
 * (GET /api/mcds/config's "stats.ws").
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool     streaming;     /* a client is connected */
    bool     framed;        /* it asked for ?z=1 (DTRZ frames, LZ4 where it pays) */
    uint32_t frames;        /* frames sent */
    uint32_t packed;        /* of them LZ4-compressed */
    uint64_t raw_bytes;     /* trace bytes sent */
    uint64_t wire_bytes;    /* as they went on the wire */
    uint32_t comp_kbps;     /* the compressor's measured speed, kB/s (0: not measured) */
    uint32_t comp_pct;      /* its output as a percentage of its input */
    uint32_t wire_kbps;     /* the link's measured rate while sending, kB/s */
} trace_ws_stats_t;

void trace_ws_get_stats(trace_ws_stats_t *out);
