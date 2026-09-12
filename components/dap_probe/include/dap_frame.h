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
/*
 * client_write is 0x08.  0x1B is client_readwrite - a combined transaction -
 * and sending it in place of a write makes the device interpret the frame as
 * read-first and desynchronise, which presents as the write being accepted and
 * having no effect.
 */
#define DAP_CMD_CLIENT_WRITE  0x08u
#define DAP_CMD_CLIENT_RW     0x1Bu
#define DAP_CMD_CLIENT_SET    0x1Cu

/*
 * IOClient instructions, 4 bits, paired with a 3-bit size exponent, and the
 * register bit assignments that go with them.
 *
 * One copy, here, because there were four: dap_probe.c held the catalog while
 * dap_fpga_route.c, tricore_flash.c and tricore_flash_probe.c each redeclared
 * the one or two they needed under a different name.  A private second name for
 * 0xB is exactly how it came to be read as CLIENT_ID once already - see below.
 */
#define DAP_IO_READ_BYTE      0x9u
#define DAP_IO_READ_HWORD     0x7u
#define DAP_IO_READ_WORD      0x5u

/*
 * IOClient registers, by IO instruction nibble.
 *
 * CLIENT_ID is instruction 0xF at offset 0x0F, 16 bits wide, hard-wired to
 * 0x0260: TYPE 0x02 (Cerberus_FPI 32-bit client), VERSION 6, REVISION 0.  A
 * 16-bit read uses payload 0x4F; a 32-bit read uses 0x5F and the hardware
 * replicates the halfword, giving 0x02600260.
 *
 * This was briefly changed to 0xB on the strength of a USB capture, in which
 * every client_read the reference probe issued carried payload 0x4B.  That was
 * a misreading: 0xB is IOINFO at offset 0x0B, so the reference was reading
 * client info and error status, not the ID.  The target then returned exactly
 * what was asked for - 0x0020, IOINFO's PWR_DWN bit - which is why the reply
 * was CRC-valid but not 0x0260.  A capture shows what a tool happened to do,
 * not what a register requires.
 */
#define DAP_IO_CLIENT_ID       0xFu
#define DAP_IO_INFO            0xBu
#define DAP_IO_SET_ADDRESS     0x1u   /* loads IOADDR, 16- or 32-bit write */
#define DAP_IO_CONF            0x0u   /* IOCONF, write-only, N = 12 */

/*
 * The bus access instructions pair up, writes even and reads odd:
 * 2H/3H block, 4H/5H word, 6H/7H halfword, 8H/9H byte.  All RW Mode only.
 */
#define DAP_IO_WRITE_BLOCK     0x2u
#define DAP_IO_READ_BLOCK      0x3u
#define DAP_IO_WRITE_WORD      0x4u
#define DAP_IO_WRITE_HWORD     0x6u
#define DAP_IO_WRITE_BYTE      0x8u

/*
 * IOCONF.MODE selects what a read instruction means, and the reset value is
 * the wrong one for a debugger.
 *
 *   MODE = 0, communication mode: IO_READ_WORD fetches COMDATA and raises
 *             IOSR.CRSYNC to ask the target's own software for data.  With no
 *             software playing along the read simply never completes.
 *   MODE = 1, read/write mode:    IO_READ_WORD performs a bus read from the
 *             address in IOADDR, which is what memory access needs.
 *
 * SVM_MODE at bit 7 selects supervisor privilege, which register space
 * generally wants.
 */
#define DAP_IOCONF_MODE_RW     0x0001u
#define DAP_IOCONF_SVM         0x0080u

/*
 * IOCONF is a 12-bit register, and the width matters more than it looks.
 *
 * Each IO instruction has a required data length N - IO_CONFIG is 12,
 * IO_SET_ADDRESS is 16 or 32 - and the shift core is explicit about what
 * happens when a tool disagrees: fewer bits than N cancels the write
 * entirely, and *more* bits than N means only the last N are used.
 *
 * Sending IOCONF as 16 bits therefore does not write 0x0081.  The device keeps
 * the last twelve bits, which is 0x008, so MODE stays 0 and the interface
 * stays in communication mode - where every read goes to COMDATA and waits for
 * on-chip software that is not listening.  That is precisely the symptom this
 * cost a long time to find: IOClient register reads working, every bus read
 * silently dropped, and no error flagged anywhere.
 */
#define DAP_IOCONF_BITS        12u

/*
 * IOINFO bit assignments, which are not what an earlier note in this file
 * assumed.  Bit 5 is ENDINIT, not PWR_DWN, and IF_LCK is bit 7.
 */
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

/*
 * A frame under construction.  `bit[i]` is the i'th bit on the wire, held one
 * bit per byte so the PHY layer can pack it whichever way its hardware wants -
 * a GPIO loop consumes it directly, and the SPI path packs it into bytes.
 */
typedef struct {
    uint8_t bit[DAP_FRAME_MAX_BITS];
    size_t  len;   /* bits actually used */

    /*
     * The fields the frame was built from, kept alongside the serialised bits.
     *
     * A PHY that clocks the frame out itself only needs `bit`; one that hands
     * the whole exchange to hardware needs the fields back, because the
     * hardware assembles its own frame from them.  Recovering them by parsing
     * `bit` would be a second implementation of the frame format, which is the
     * one thing this file exists to avoid having two of.
     */
    uint8_t  cmd;
    uint8_t  len_field;
    uint64_t data;
    size_t   data_bits;
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
