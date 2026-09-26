#pragma once

/* Shared between dap_probe.c, dap_probe_diag.c and dap_probe_bringup.c.  Not public. */

#include <stddef.h>
#include <stdint.h>

#include "dap_frame.h"
#include "dap_probe.h"
#include "esp_err.h"

/* Commands outside the public catalog: client_reset (training handshake) and blockread. */
#define DAP_CMD_CLIENT_RESET   0x1Du
#define DAP_CMD_BLOCKREAD      0x0Au

/* Low clocks before every frame: the floor is 1 + DAPISC.SISP; 2 covers SISP 0. */
#define DAP_FRAME_LEAD_CLOCKS  2

/* Low clocks after a CRC error, lost reply or aborted block (MAXWAIT8) to resync. */
#define DAP_RESYNC_CLOCKS      DAP_MAXWAIT_RESET_CYCLES

/* Extra wait beyond MAXWAIT8, covering the 3-cycle reply delay and lead-in. */
#define DAP_WAIT_MARGIN        8

/* Latest acknowledge still believed, in clocks; later ones are treated as glitches. */
#define DAP_ACK_MAX_WAIT       32
/* Training handshake data, sent twice after sync and dapisc. */
#define DAP_TRAINING_DATA      0xAAAAAA83u

/* Cold-attach dapisc: signature plus register value. */
#define DAP_DAPISC_SIGNATURE   0x4ABBAF53u
#define DAP_DAPISC_VALUE       0x0F00u

/* SCU / OCDS control block and miniMCDS registers. */
#define DAP_ADDR_OEC_PAT       0xF0000478u
#define DAP_ADDR_OCNTRL        0xF000047Cu
#define DAP_ADDR_OSTATE        0xF0000480u
#define DAP_ADDR_MCDS_BASE     0xFB718000u
#define DAP_ADDR_MCDS_CLC      (DAP_ADDR_MCDS_BASE + 0x0000u)
#define DAP_ADDR_MCDS_ID       (DAP_ADDR_MCDS_BASE + 0x0008u)
#define DAP_ADDR_MCDS_CT       (DAP_ADDR_MCDS_BASE + 0x0010u)
#define DAP_MCDS_ID_EXPECT     0x00D6C007u

/* Trace FIFO: FIFONOW is the write pointer, FIFOBOT/FIFOTOP the buffer bounds. */
#define DAP_ADDR_FIFONOW       (DAP_ADDR_MCDS_BASE + 0x0200u)
#define DAP_ADDR_FIFOBOT       (DAP_ADDR_MCDS_BASE + 0x0204u)
#define DAP_ADDR_FIFOTOP       (DAP_ADDR_MCDS_BASE + 0x020Cu)
#define DAP_ADDR_FIFOCTL       (DAP_ADDR_MCDS_BASE + 0x0210u)
#define DAP_ADDR_FIFOWARN0     (DAP_ADDR_MCDS_BASE + 0x0214u)
#define DAP_ADDR_FIFOWARN1     (DAP_ADDR_MCDS_BASE + 0x0218u)
#define DAP_ADDR_FIFOOVRCNT    (DAP_ADDR_MCDS_BASE + 0x021Cu)

/*
 * TRAM over the non-cached SRI alias: 8 kB in eight 1 kB paragraphs.  Each
 * paragraph starts uncompressed, so a drain resumes on a paragraph boundary.
 */
#define DAP_ADDR_TRAM_BASE     0xB8000000u
#define DAP_TRAM_BYTES         0x2000u
#define DAP_TRAM_PARAGRAPH     0x400u

#define DAP_SYNC_EXPECT        0xAAAAAAAAu
/*
 * Sync reply over a wide link: the pattern is sent on each line, so bits
 * double to 0xCCCCCCCC and the CRC is not checkable.
 */
#define DAP_SYNC_EXPECT_WIDE   0xCCCCCCCCu
#define DAP_SYNC_WIRE_WORD     0x09FE1u

/* One frame out, one reply in, with the wait-state stuffing handled. */
esp_err_t dap_probe_exchange(const dap_frame_t *frame, size_t reply_bits,
                             dap_exchange_t *out);

/* MAXWAIT8 as the device currently has it, in DAP0 clocks. */
extern uint32_t dap_probe_max_wait;

/* One line per exchange, for the step-by-step reports. */
void dap_probe_log_exchange(const char *step, const dap_exchange_t *x);

/* The whole reply window of an attach, undecoded. */
void dap_probe_dump_raw_attach(const char *what);

/* Trailing clocks after a reply; zero is what the device wants. */
extern size_t dap_probe_trailer_bits;
