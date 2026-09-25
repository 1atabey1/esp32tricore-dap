/*
 * Autonomous TRAM drain: the probe follows the trace FIFO write pointer and
 * publishes complete 1 kB paragraphs without host round trips.
 *
 * The paragraph FIFONOW points into is still being written and is skipped.  A
 * TRAM lap raises no ERR message, so lost paragraphs are reported as a gap
 * record; the decoder resumes at the next paragraph with its caches zeroed.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Framing of one record in the drained stream, little-endian on the wire. */
#define DAP_TRACE_MAGIC 0x50525444u   /* "DTRP" */

enum {
    DAP_TRACE_FLAG_GAP = 1u << 0,     /* paragraphs were lost before this one */
};

typedef struct {
    uint32_t magic;        /* DAP_TRACE_MAGIC, so a resynchronising reader can find it */
    uint32_t seq;          /* paragraph sequence number, gaps included in the count */
    uint32_t tram_offset;  /* byte offset of this paragraph within the TRAM */
    uint16_t length;       /* payload bytes that follow this header */
    uint16_t flags;        /* DAP_TRACE_FLAG_* */
    uint32_t lost;         /* paragraphs the target overwrote before we read them */
} dap_trace_record_t;

typedef struct {
    bool     running;
    uint32_t paragraphs;     /* published */
    uint32_t lost;           /* overwritten before they could be read */
    uint32_t laps;           /* times the pointer got ahead of the drain */
    uint32_t overruns;       /* FIFOOVRCNT movement: observation-unit overflows */
    uint32_t read_errors;    /* block reads that drew no parcels */
    uint32_t bytes;          /* payload bytes published */
    uint32_t fifonow;        /* last write pointer seen */
    uint32_t queue_free;     /* ring bytes still available */
    uint32_t queue_dropped;  /* payload bytes dropped because nobody was reading */
    uint32_t poll_us_max;    /* slowest drain pass, to size the poll interval */
} dap_trace_stats_t;

/* Start draining the trace FIFO.  Needs OCDS enabled; MCDS configuration is
 * left to the host. */
esp_err_t dap_trace_start(void);

/* Stop draining.  The FIFO is left as it is, so a capture can be resumed. */
void dap_trace_stop(void);

/* One drain pass (normally run by the drain task).  No-op when stopped;
 * ESP_ERR_NOT_FOUND when nothing was waiting. */
esp_err_t dap_trace_poll(void);

/* Copy up to `max` bytes of drained stream; returns bytes copied.  Records are
 * self-describing, so any chunking works. */
size_t dap_trace_read(uint8_t *out, size_t max);

void dap_trace_get_stats(dap_trace_stats_t *out);

/* Write a known pattern into the TRAM, run the drain over it as if the write
 * pointer had passed it, and check records and payload.  ESP_OK only if every
 * byte matched; leaves the drain stopped. */
esp_err_t dap_trace_selftest(void);

#ifdef __cplusplus
}
#endif
