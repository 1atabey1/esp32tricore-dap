/*
 * Infineon DAP frame assembly for AURIX TC3xx targets.  Downstream frame:
 *     start bit (1) | CMD (5) | LEN (6) | DATA (LEN) | CRC6 (6) | trailing 0
 * All fields go out LSB first; CRC6 covers CMD+LEN+DATA in transmission order.
 * Bit index 0 is the first bit on the wire (sync, LEN 63: 19 bits, 0x09FE1).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Downstream command codes. */
#define DAP_CMD_SYNC          0x10u
#define DAP_CMD_DAPISC        0x11u
#define DAP_CMD_CLIENT_READ   0x1Au
/* client_write is 0x08; 0x1B (client_readwrite) instead desynchronises the device. */
#define DAP_CMD_CLIENT_WRITE  0x08u
#define DAP_CMD_CLIENT_RW     0x1Bu
#define DAP_CMD_CLIENT_SET    0x1Cu

/* IOClient instructions (4 bits, paired with a 3-bit size exponent). */
#define DAP_IO_READ_BYTE      0x9u
#define DAP_IO_READ_HWORD     0x7u
#define DAP_IO_READ_WORD      0x5u

/*
 * CLIENT_ID (0xF) is 16 bits, hard-wired to 0x0260; a 32-bit read returns
 * 0x02600260.  0xB is IOINFO, not the ID.
 */
#define DAP_IO_CLIENT_ID       0xFu
#define DAP_IO_INFO            0xBu
#define DAP_IO_SET_ADDRESS     0x1u   /* loads IOADDR, 16- or 32-bit write */
#define DAP_IO_CONF            0x0u   /* IOCONF, write-only, N = 12 */

/* Bus access, writes even and reads odd: 2/3 block, 4/5 word, 6/7 halfword, 8/9 byte.  RW mode only. */
#define DAP_IO_WRITE_BLOCK     0x2u
#define DAP_IO_READ_BLOCK      0x3u
#define DAP_IO_WRITE_WORD      0x4u
#define DAP_IO_WRITE_HWORD     0x6u
#define DAP_IO_WRITE_BYTE      0x8u

/*
 * IOCONF.MODE: 0 (reset) = communication mode, reads fetch COMDATA and wait
 * for target software; 1 = read/write mode, reads hit the bus at IOADDR.
 * SVM (bit 7) selects supervisor privilege.
 */
#define DAP_IOCONF_MODE_RW     0x0001u
#define DAP_IOCONF_SVM         0x0080u

/*
 * IOCONF must be written with exactly 12 bits: fewer cancels the write, more
 * keeps only the last 12, silently leaving MODE at 0.
 */
#define DAP_IOCONF_BITS        12u

/* IOINFO bits. */
#define DAP_IOINFO_IDLE        (1u << 0)
#define DAP_IOINFO_PWR_DWN     (1u << 1)
#define DAP_IOINFO_BUS_RD_ERR  (1u << 2)
#define DAP_IOINFO_BUS_WR_ERR  (1u << 3)
#define DAP_IOINFO_PWR_DWN_ERR (1u << 4)
#define DAP_IOINFO_ENDINIT     (1u << 5)
#define DAP_IOINFO_BUS_RST     (1u << 6)
#define DAP_IOINFO_IF_LCK      (1u << 7)
#define DAP_CLIENT_ID_EXPECT   0x0260u

/* Longest frame this layer assembles: start + CMD + LEN + 63 data + CRC + 0. */
#define DAP_FRAME_MAX_BITS    (1 + 5 + 6 + 63 + 6 + 1)

/* A frame under construction; `bit[i]` is the i'th bit on the wire, one per byte. */
typedef struct {
    uint8_t bit[DAP_FRAME_MAX_BITS];
    size_t  len;   /* bits actually used */

    /* The source fields, for PHYs whose hardware assembles its own frame. */
    uint8_t  cmd;
    uint8_t  len_field;
    uint64_t data;
    size_t   data_bits;
} dap_frame_t;

/*
 * CRC6 generation: right-shifting Galois LFSR, polynomial 0x30, seed 0x20
 * (not the manual's GetCrc(), which the silicon rejects).  `bits` is the
 * payload in transmission order.
 */
uint8_t dap_crc6(const uint8_t *bits, size_t nbits);

/*
 * CRC6 check, the manual's CalcCrc(): Fibonacci LFSR, polynomial 0x03, seed
 * 0x3F.  Payload plus its six CRC bits leave the state at zero if correct.
 */
bool dap_crc6_residue_ok(const uint8_t *bits, size_t nbits);

/* Append `nbits` of `value`, LSB first. Returns false if the frame would overflow. */
bool dap_frame_put(dap_frame_t *f, uint32_t value, size_t nbits);

/*
 * Assemble a complete frame: start bit, CMD, LEN, `data_bits` of `data`,
 * CRC6 and trailing zero.  For sync pass len_field = 63 with no data.
 */
bool dap_frame_build(dap_frame_t *f, uint8_t cmd, uint8_t len_field,
                     uint64_t data, size_t data_bits);

/* Pack an assembled frame into an integer, first transmitted bit in bit 0. */
uint64_t dap_frame_word(const dap_frame_t *f);

/*
 * client_read payload: 4-bit IO instruction then 3-bit size exponent (LEN 7).
 * E.g. 0x39 for an 8-bit read, 0x47 for 16-bit, 0x55 for 32-bit.
 */
uint8_t dap_client_read_payload(uint8_t io_instruction, uint8_t size_exponent);

#ifdef __cplusplus
}
#endif
