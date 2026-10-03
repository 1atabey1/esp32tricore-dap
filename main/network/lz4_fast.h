/*
 * LZ4 block compression (the block format alone, no frame), greedy, one hash
 * probe per position: fast enough to keep the trace stream's pace on the
 * WiFi core, and any LZ4 block decoder reads it (python: lz4.block).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LZ4_FAST_HASH_BITS  12
/* The scratch table lz4_fast_compress needs, in bytes. */
#define LZ4_FAST_TABLE_BYTES ((size_t)sizeof(uint16_t) << LZ4_FAST_HASH_BITS)

/*
 * Compress n bytes (n < 65536) into dst.  Returns the block's length, or 0
 * when it does not fit in cap (send the data stored instead).  table is
 * LZ4_FAST_TABLE_BYTES of scratch, internal RAM for speed.
 */
size_t lz4_fast_compress(const uint8_t *src, size_t n, uint8_t *dst, size_t cap,
                         uint16_t *table);

#ifdef __cplusplus
}
#endif
