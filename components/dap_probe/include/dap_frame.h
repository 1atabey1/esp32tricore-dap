/*
 * Infineon DAP (Device Access Port) frame assembly for AURIX TC3xx targets.
 *
 * A downstream frame is, in transmission order:
 *
 *     start bit (1) | CMD (5) | LEN (6) | DATA (LEN) | CRC6 (6) | trailing 0
 *
 * Every multi-bit field goes out LSB first.  The CRC6 covers CMD, LEN and DATA
 * as one bit string in that same transmission order, and is itself appended LSB
 * first.
 *
 * The frame is carried here as a bit string whose index 0 is the *first bit on
 * the wire*.  For short frames dap_frame_word() packs that into an integer with
 * the first transmitted bit in bit 0, which is the convention the documented
 * test vectors use: the `sync` frame with LEN = 63 is 19 bits and reads
 * 0x09FE1.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Downstream command codes, from the DAP command catalog. */
#define DAP_CMD_SYNC          0x10u
#define DAP_CMD_DAPISC        0x11u
#define DAP_CMD_CLIENT_READ   0x1Au
#define DAP_CMD_CLIENT_WRITE  0x1Bu
#define DAP_CMD_CLIENT_SET    0x1Cu

/* IOClient instructions, 4 bits, paired with a 3-bit size exponent. */
#define DAP_IO_READ_BYTE      0x9u
#define DAP_IO_READ_HWORD     0x7u
#define DAP_IO_READ_WORD      0x5u

/* Longest frame this layer assembles: start + CMD + LEN + 63 data + CRC + 0. */
#define DAP_FRAME_MAX_BITS    (1 + 5 + 6 + 63 + 6 + 1)

/*
 * A frame under construction.  `bit[i]` is the i'th bit on the wire, held one
 * bit per byte so the PHY layer can pack it whichever way its hardware wants -
 * a GPIO loop consumes it directly, and the SPI path packs it into bytes.
 */
typedef struct {
    uint8_t bit[DAP_FRAME_MAX_BITS];
    size_t  len;   /* bits actually used */
} dap_frame_t;

/*
 * CRC6 generation: a right-shifting *Galois* LFSR, polynomial 0x30, seed 0x20.
 *
 * This is deliberately not the manual's GetCrc(), which does not produce a
 * checksum the silicon accepts.  The two topologies need different seeds for
 * the same initial condition: Fibonacci 0x3F and Galois 0x20 are the same
 * state expressed differently, and seeding a Galois implementation with 0x3F
 * yields a self-consistent checksum that the target rejects.
 *
 * `bits` is the payload in transmission order (LSB first within each field).
 */
uint8_t dap_crc6(const uint8_t *bits, size_t nbits);

/*
 * CRC6 verification, the manual's CalcCrc(): a right-shifting *Fibonacci*
 * LFSR, polynomial 0x03, seed 0x3F.  Shift the payload and then its six CRC
 * bits through it and a correct frame leaves the state at zero.  This is what
 * the silicon implements, and it is the oracle dap_crc6() is checked against.
 */
bool dap_crc6_residue_ok(const uint8_t *bits, size_t nbits);

/* Append `nbits` of `value`, LSB first. Returns false if the frame would overflow. */
bool dap_frame_put(dap_frame_t *f, uint32_t value, size_t nbits);

/*
 * Assemble a complete downstream frame: start bit, CMD, LEN, `len_bits` of
 * `data`, the CRC6 over CMD+LEN+DATA, and the trailing zero.
 *
 * `len_bits` is the LEN field's value as well as the width of DATA, which is
 * how the protocol encodes both - `sync` carries LEN = 63 with no data at all,
 * so pass len_bits = 63 and data = 0 for it.  That is not a special case in
 * the encoding: LEN counts payload bits the *device* expects, and for sync the
 * run of ones is a JTAG-TAP safety measure rather than a length.
 */
bool dap_frame_build(dap_frame_t *f, uint8_t cmd, uint8_t len_field,
                     uint64_t data, size_t data_bits);

/* Pack an assembled frame into an integer, first transmitted bit in bit 0. */
uint64_t dap_frame_word(const dap_frame_t *f);

/*
 * The one-byte payload of a client_read: a 4-bit IO instruction followed by a
 * 3-bit size exponent, both LSB first, giving LEN = 7.  Returns 0x39 for an
 * 8-bit read, 0x47 for 16-bit and 0x55 for 32-bit.
 */
uint8_t dap_client_read_payload(uint8_t io_instruction, uint8_t size_exponent);

#ifdef __cplusplus
}
#endif
