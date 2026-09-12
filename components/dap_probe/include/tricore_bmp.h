/*
 * Register the TC3xx as a Black Magic Probe target.
 *
 * BMP's GDB server is already running on port 4242 and already implements
 * everything above the target - RSP, qXfer, vCont, watchpoint bookkeeping,
 * monitor commands.  This adds the one thing it lacks, a TriCore, without
 * modifying the submodule: target_new() and target_attach_n() are public, so a
 * target registered from here appears in BMP's own list.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Attach the probe, enable OCDS, find the cores, and register one BMP target
 * per core.  After this, GDB on port 4242 can `attach 1`.
 *
 * Does the whole DAP opening sequence itself, because a GDB client that finds
 * a half-attached probe gets a session where every register reads as an error
 * and nothing says why.
 */
esp_err_t tricore_bmp_probe(void);

#ifdef __cplusplus
}
#endif
