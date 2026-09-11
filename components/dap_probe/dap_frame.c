/*
 * DAP frame assembly and CRC6.  Pure computation, no hardware: this file is
 * compiled both into the firmware and into a host test binary, so the wire
 * format can be verified against the documented vectors before a pin moves.
 */

#include "dap_frame.h"

#define CRC6_GALOIS_POLY   0x30u
#define CRC6_GALOIS_SEED   0x20u
#define CRC6_FIBO_POLY     0x03u
#define CRC6_FIBO_SEED     0x3Fu

uint8_t dap_crc6(const uint8_t *bits, size_t nbits)
{
    uint8_t crc = CRC6_GALOIS_SEED;

    for (size_t i = 0; i < nbits; i++) {
        const uint8_t feedback = (uint8_t)((crc ^ bits[i]) & 1u);
        crc >>= 1;
        if (feedback) {
            crc ^= CRC6_GALOIS_POLY;
        }
    }
    return (uint8_t)(crc & 0x3Fu);
}

bool dap_crc6_residue_ok(const uint8_t *bits, size_t nbits)
{
    uint8_t state = CRC6_FIBO_SEED;

    /*
     * A true Fibonacci LFSR: the taps in CRC6_FIBO_POLY are XORed together
     * with the incoming bit and the result is shifted in at the top.  This is
     * deliberately a different topology from dap_crc6() above - an oracle that
     * shared the generator's structure would only prove self-consistency,
     * which is the exact mistake this protocol punishes.
     */
    for (size_t i = 0; i < nbits; i++) {
        uint8_t feedback = bits[i] & 1u;
        for (uint8_t tap = 0; tap < 6; tap++) {
            if ((CRC6_FIBO_POLY >> tap) & 1u) {
                feedback ^= (uint8_t)((state >> tap) & 1u);
            }
        }
        state = (uint8_t)(((state >> 1) | (uint8_t)(feedback << 5)) & 0x3Fu);
    }
    return (state & 0x3Fu) == 0u;
}

bool dap_frame_put(dap_frame_t *f, uint32_t value, size_t nbits)
{
    if (f->len + nbits > DAP_FRAME_MAX_BITS) {
        return false;
    }
    for (size_t i = 0; i < nbits; i++) {
        f->bit[f->len++] = (uint8_t)((value >> i) & 1u);
    }
    return true;
}

bool dap_frame_build(dap_frame_t *f, uint8_t cmd, uint8_t len_field,
                     uint64_t data, size_t data_bits)
{
    /* The payload the CRC covers: CMD, LEN and DATA, in transmission order. */
    uint8_t payload[5 + 6 + 63];
    size_t  n = 0;

    if (cmd > 0x1Fu || len_field > 63u || data_bits > 63u) {
        return false;
    }

    /* Kept for a PHY that assembles the frame in hardware and needs the fields
     * rather than the bits; see the note on dap_frame_t. */
    f->cmd       = cmd;
    f->len_field = len_field;
    f->data      = data;
    f->data_bits = data_bits;

    for (size_t i = 0; i < 5; i++) {
        payload[n++] = (uint8_t)((cmd >> i) & 1u);
    }
    for (size_t i = 0; i < 6; i++) {
        payload[n++] = (uint8_t)((len_field >> i) & 1u);
    }
    for (size_t i = 0; i < data_bits; i++) {
        payload[n++] = (uint8_t)((data >> i) & 1u);
    }

    const uint8_t crc = dap_crc6(payload, n);

    f->len = 0;
    if (!dap_frame_put(f, 1u, 1)) {           /* start bit */
        return false;
    }
    for (size_t i = 0; i < n; i++) {          /* CMD, LEN, DATA already ordered */
        if (f->len >= DAP_FRAME_MAX_BITS) {
            return false;
        }
        f->bit[f->len++] = payload[i];
    }
    if (!dap_frame_put(f, crc, 6)) {          /* CRC6, LSB first */
        return false;
    }
    return dap_frame_put(f, 0u, 1);           /* trailing zero */
}

uint64_t dap_frame_word(const dap_frame_t *f)
{
    uint64_t word = 0;

    for (size_t i = 0; i < f->len && i < 64; i++) {
        word |= (uint64_t)(f->bit[i] & 1u) << i;
    }
    return word;
}

uint8_t dap_client_read_payload(uint8_t io_instruction, uint8_t size_exponent)
{
    return (uint8_t)((io_instruction & 0x0Fu) | ((size_exponent & 0x07u) << 4));
}
