/*
 * Autonomous TRAM drain.
 *
 * This is the part that fixes the original problem.  A miniWiggler through
 * DAS/TAS reads target memory at about 38 kB/s, and draining an 8 kB trace
 * buffer over that ceiling with a host round trip per read is what forced the
 * application under test to publish at a twelfth of its natural rate.  Here the
 * probe follows the write pointer itself and empties whole paragraphs without
 * asking the host for anything, at the 453 kB/s the register-driven PHY
 * sustains.
 *
 * Two design points are not negotiable, because the decoder depends on them:
 *
 *   - Only *complete* paragraphs are published.  The TRAM is divided into 1 kB
 *     paragraphs; the one FIFONOW currently points into is still being written,
 *     so it is left alone until the pointer moves past it.
 *   - A lap is reported explicitly.  A TRAM lap produces no ERR message - those
 *     are for observation-unit FIFO overflows - so a drain that falls behind
 *     silently produces a stream that parses as valid and is wrong.  A gap
 *     record in the stream is something a decoder can act on: discard the
 *     paragraph containing FIFONOW, resume at the next boundary with both
 *     compression caches zeroed, which is safe precisely because each trace
 *     unit's first message in a paragraph is uncompressed.
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

/*
 * Take over the trace FIFO and start draining.  Assumes an attached target
 * with OCDS enabled - dap_probe_enable_ocds() - and does not configure tracing
 * itself: the MCDS configuration is the host's 54-write list, and a drain that
 * invented its own would be measuring itself.
 */
esp_err_t dap_trace_start(void);

/* Stop draining.  The FIFO is left as it is, so a capture can be resumed. */
void dap_trace_stop(void);

/*
 * One drain pass: read the write pointer, publish every paragraph that has
 * been completed since the last pass, and account for anything lost.  Safe to
 * call when stopped, in which case it does nothing.
 *
 * dap_trace_start() runs this in its own task, so callers normally do not.
 * Returns ESP_ERR_NOT_FOUND when there was nothing waiting, which is how the
 * task knows whether it can afford to sleep.
 */
esp_err_t dap_trace_poll(void);

/*
 * Take up to `max` bytes of drained stream.  Returns what was copied, 0 when
 * there is nothing waiting.  Records are byte-aligned and self-describing, so a
 * reader may take them in arbitrary chunks.
 */
size_t dap_trace_read(uint8_t *out, size_t max);

void dap_trace_get_stats(dap_trace_stats_t *out);

/*
 * Prove the drain works, without waiting for the target to emit trace.
 *
 * This backend only ever follows the trace FIFO's write pointer; nothing here
 * configures the miniMCDS to produce messages, so on a target that is not
 * tracing there is nothing to drain and "it ran and published nothing" is
 * indistinguishable from "it is broken".  This writes a known pattern into the
 * trace buffer, advances the write pointer over it, and checks that exactly
 * those bytes come back out of the ring in order.
 *
 * What it proves: paragraph detection, the wrap at the end of the buffer, the
 * gap accounting, the ring, and the read path the HTTP stream uses - every
 * part of this file.  What it does not prove: that the target's trace
 * messages are what the host thinks they are.  That needs a target that is
 * actually tracing, and is a separate question from whether the drain works.
 *
 * Returns ESP_OK only if every byte matched.  Leaves the drain stopped.
 */
esp_err_t dap_trace_selftest(void);

#ifdef __cplusplus
}
#endif
