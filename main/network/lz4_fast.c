#include "lz4_fast.h"

#include <string.h>

#include "esp_attr.h"

/* LZ4 block rules: matches are at least 4 bytes, the last 5 bytes are always
 * literals, and no match starts within the last 12. */
#define MIN_MATCH     4
#define LAST_LITERALS 5
#define MF_LIMIT      12

/*
 * Assembled from bytes: Xtensa has no unaligned loads, and memcpy from an
 * unaligned pointer became a library call per position (the compressor ran
 * at ~3 MB/s on the S3, far below the WiFi it was meant to help).
 */
static inline uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static inline uint32_t hash4(uint32_t v)
{
    return (v * 2654435761u) >> (32 - LZ4_FAST_HASH_BITS);
}

/* A length past 15 as LZ4 extends it: 255s, then the rest. */
static inline uint8_t *put_len(uint8_t *op, size_t len)
{
    while (len >= 255) {
        *op++ = 255;
        len -= 255;
    }
    *op++ = (uint8_t)len;
    return op;
}

IRAM_ATTR size_t lz4_fast_compress(const uint8_t *src, size_t n, uint8_t *dst,
                                   size_t cap, uint16_t *table)
{
    const uint8_t *ip     = src;
    const uint8_t *anchor = src;
    const uint8_t *const iend = src + n;
    uint8_t       *op     = dst;
    uint8_t *const oend   = dst + cap;

    if (n >= 65536) {
        return 0;
    }
    memset(table, 0, LZ4_FAST_TABLE_BYTES);

    if (n > MF_LIMIT) {
        const uint8_t *const mflimit    = iend - MF_LIMIT;
        const uint8_t *const matchlimit = iend - LAST_LITERALS;
        unsigned misses = 0;

        ip++;                               /* the first byte has nothing behind it */
        while (ip < mflimit) {
            const uint32_t seq = rd32(ip);
            const uint32_t h   = hash4(seq);
            const uint8_t *ref = src + table[h];
            table[h] = (uint16_t)(ip - src);

            /* Most candidates fail on the first byte: test it before the rest. */
            if (ref >= ip || ref[0] != ip[0] || rd32(ref) != seq) {
                /* Incompressible stretches are skipped faster and faster. */
                ip += 1 + (misses++ >> 6);
                continue;
            }
            misses = 0;

            /* Back over equal bytes, then forward. */
            while (ip > anchor && ref > src && ip[-1] == ref[-1]) {
                ip--;
                ref--;
            }
            const uint8_t *mp = ip + MIN_MATCH;
            const uint8_t *rp = ref + MIN_MATCH;
            while (mp < matchlimit && *mp == *rp) {
                mp++;
                rp++;
            }

            const size_t lit  = (size_t)(ip - anchor);
            const size_t mlen = (size_t)(mp - ip) - MIN_MATCH;
            /* token + literal length + literals + offset + match length */
            if (op + 1 + lit / 255 + 1 + lit + 2 + mlen / 255 + 1 > oend) {
                return 0;
            }
            uint8_t *token = op++;
            if (lit >= 15) {
                *token = 15u << 4;
                op = put_len(op, lit - 15);
            } else {
                *token = (uint8_t)(lit << 4);
            }
            memcpy(op, anchor, lit);
            op += lit;
            const uint16_t off = (uint16_t)(ip - ref);
            *op++ = (uint8_t)off;
            *op++ = (uint8_t)(off >> 8);
            if (mlen >= 15) {
                *token |= 15u;
                op = put_len(op, mlen - 15);
            } else {
                *token |= (uint8_t)mlen;
            }

            ip = mp;
            anchor = ip;
            /* One more table entry inside the match, for the next search. */
            if (ip - 2 > src) {
                table[hash4(rd32(ip - 2))] = (uint16_t)(ip - 2 - src);
            }
        }
    }

    /* The rest as literals. */
    const size_t lit = (size_t)(iend - anchor);
    if (op + 1 + lit / 255 + 1 + lit > oend) {
        return 0;
    }
    if (lit >= 15) {
        *op++ = 15u << 4;
        op = put_len(op, lit - 15);
    } else {
        *op++ = (uint8_t)(lit << 4);
    }
    memcpy(op, anchor, lit);
    op += lit;
    return (size_t)(op - dst);
}
