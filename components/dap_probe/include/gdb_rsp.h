/*
 * A GDB Remote Serial Protocol server for the TC3xx, over the DAP probe.
 *
 * GDB speaks RSP to anything on the other end of `target remote`: ASCII
 * packets framed as $<payload>#<checksum>, with + / - acknowledgements.  This
 * implements the subset that a TriCore session actually needs, on top of
 * tricore.c for run control and dap_probe for memory.
 *
 * Two decisions are worth stating, because they shape everything else:
 *
 *   - The server supplies its own target description over qXfer:features:read.
 *     GDB then takes the register order from the XML rather than from whatever
 *     a particular tricore-gdb build happens to number them, so the `g` packet
 *     layout is self-describing and cannot silently disagree with the client.
 *
 *   - Breakpoints are hardware only.  TriCore software breakpoints mean
 *     patching a 16-bit DEBUG instruction over the code, and for flash-resident
 *     code that is the wrong trade: a programmed page cannot be rewritten, so
 *     each patch means erasing and reprogramming the enclosing 16 kB sector -
 *     about a second per toggle, a program/erase cycle of a code sector each
 *     time, and a debugger that dies between the erase and the rewrite leaves
 *     that sector blank.  So Z0 is served from the same eight address triggers
 *     as Z1, and running out of them is reported rather than hidden.
 *
 * One core is one GDB thread; thread IDs are the core index plus one, because
 * RSP reserves 0 for "any thread".
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 4243, one past Black Magic's 4242.  The two servers drive the same pins from
 * the same board and cannot both be attached, but they can at least both be
 * listening without a port clash.
 */
#define GDB_RSP_DEFAULT_PORT 4243

/*
 * Attach to the target and start listening.
 *
 * Does the whole opening sequence itself - probe init, DAP attach, IOClient
 * select, RW mode, OCDS enable, core discovery - because a GDB client that
 * connects to a half-attached probe gets a session where every register reads
 * as an error and nothing says why.
 */
esp_err_t gdb_rsp_start(uint16_t port);

/* Stop listening and drop any client.  Leaves the target running. */
void gdb_rsp_stop(void);

bool     gdb_rsp_running(void);
bool     gdb_rsp_connected(void);
uint16_t gdb_rsp_port(void);

#ifdef __cplusplus
}
#endif
