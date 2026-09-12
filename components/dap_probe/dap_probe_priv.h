#pragma once

/*
 * Shared between dap_probe.c and its diagnostic siblings
 * (dap_probe_diag.c, dap_probe_bringup.c).  Not part of the interface.
 */

#include <stddef.h>
#include <stdint.h>

#include "dap_frame.h"
#include "dap_probe.h"
#include "esp_err.h"

/* Constants the diagnostics share with the core. */

/*
 * The training handshake the reference performs and this project did not.
 *
 * After sync and dapisc the device sends an alternating pattern, and the
 * reference answers with CMD 0x02 carrying 0xAAAAAA83 - itself alternating -
 * twice, and only then gets real data back.  Neither the command nor the
 * constant is in the catalog this project started from; both come straight out
 * of a USB capture of a working attach.  Without it the device answers sync
 * once and ignores everything afterwards.
 */
#define DAP_CMD_CLIENT_RESET   0x1Du
#define DAP_CMD_BLOCKREAD      0x0Au

/*
 * Low clocks before every frame.
 *
 * The protocol needs no multi-cycle preamble: the device latches any DAP1 = 1
 * on a rising edge as a start bit.  What it does need is the line settled low
 * first - the device drives DAP1 = 0 for one cycle after its reply before
 * releasing the pad, and DAPISC.SISP can hold off start-bit detection for a
 * further 0 to 3 cycles.  So the floor is 1 + SISP, and 2 covers the default
 * SISP of 0 with margin.
 *
 * Eleven was what the reference probe uses and what this code copied.  Per the
 * specification that figure is a conservative host-side buffer covering FPGA
 * clock-domain-crossing latency, not a device requirement - so most of it is
 * pure overhead for a probe that is already slow.  Nine clocks per frame back
 * is worth having in the throughput budget.
 */
#define DAP_FRAME_LEAD_CLOCKS  2

/*
 * Recovery: after a CRC error, a lost reply or an aborted block transfer, the
 * host must clock at least MAXWAIT8 cycles with DAP1 low so the device can
 * finish any pending reply and fall back to Active::RECEIVE.  A new command
 * sent before that is not guaranteed to be seen.
 */
#define DAP_RESYNC_CLOCKS      DAP_MAXWAIT_RESET_CYCLES

/*
 * A few clocks past the device's own allowance, to cover the fixed 3-cycle
 * reply delay and the lead-in, so a reply that arrives at the limit is still
 * seen rather than counted as a timeout.
 */
#define DAP_WAIT_MARGIN        8

/*
 * How late an acknowledge may arrive and still be believed.  The device
 * replies within a few cycles; this only has to exclude glitches found at the
 * far end of a 120-cycle window.
 */
#define DAP_ACK_MAX_WAIT       32
#define DAP_TRAINING_DATA      0xAAAAAA83u

/* Cold-attach dapisc: signature plus the register value the reference writes. */
#define DAP_DAPISC_SIGNATURE   0x4ABBAF53u
#define DAP_DAPISC_VALUE       0x0F00u

/* SCU / OCDS control block, and the miniMCDS registers Phase 3 touches. */
#define DAP_ADDR_OEC_PAT       0xF0000478u
#define DAP_ADDR_OCNTRL        0xF000047Cu
#define DAP_ADDR_OSTATE        0xF0000480u
#define DAP_ADDR_MCDS_BASE     0xFB718000u
#define DAP_ADDR_MCDS_CLC      (DAP_ADDR_MCDS_BASE + 0x0000u)
#define DAP_ADDR_MCDS_ID       (DAP_ADDR_MCDS_BASE + 0x0008u)
#define DAP_ADDR_MCDS_CT       (DAP_ADDR_MCDS_BASE + 0x0010u)
#define DAP_MCDS_ID_EXPECT     0x00D6C007u

/*
 * The trace FIFO, offsets from the miniMCDS base.  FIFONOW is the write
 * pointer a drain follows; FIFOBOT and FIFOTOP are the buffer bounds, written
 * by whoever sets the streaming up rather than by the MCDS configuration.
 */
#define DAP_ADDR_FIFONOW       (DAP_ADDR_MCDS_BASE + 0x0200u)
#define DAP_ADDR_FIFOBOT       (DAP_ADDR_MCDS_BASE + 0x0204u)
#define DAP_ADDR_FIFOTOP       (DAP_ADDR_MCDS_BASE + 0x020Cu)
#define DAP_ADDR_FIFOCTL       (DAP_ADDR_MCDS_BASE + 0x0210u)
#define DAP_ADDR_FIFOWARN0     (DAP_ADDR_MCDS_BASE + 0x0214u)
#define DAP_ADDR_FIFOWARN1     (DAP_ADDR_MCDS_BASE + 0x0218u)
#define DAP_ADDR_FIFOOVRCNT    (DAP_ADDR_MCDS_BASE + 0x021Cu)

/*
 * The TRAM itself, over the non-cached SRI alias: 8 kB, eight 1 kB paragraphs.
 * Paragraph boundaries are where a drain resumes after a lap, because each
 * trace unit's first message in a paragraph is uncompressed.
 */
#define DAP_ADDR_TRAM_BASE     0xB8000000u
#define DAP_TRAM_BYTES         0x2000u
#define DAP_TRAM_PARAGRAPH     0x400u

#define DAP_SYNC_EXPECT        0xAAAAAAAAu
/*
 * The same reply, read back over a wide link.
 *
 * The device sends the training pattern on *each* line rather than splitting
 * one pattern across the two, so weaving them back together doubles every bit
 * and 0xAAAAAAAA arrives as 0xCCCCCCCC.  Its CRC does not survive the same
 * treatment - six bits per line reassemble into twelve - so a wide sync is
 * judged on the pattern alone.  A real telegram is genuinely interleaved and
 * its CRC checks out, which is what the attach below goes on to prove.
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
