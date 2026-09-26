/*
 * One recursive lock for the DAP link and the fabric behind it.  GDB, the web
 * handlers, the flasher and the trace drain all run in different tasks.
 *
 * Single operations (dap_probe_read32, a fabric frame, ...) take it
 * themselves; a sequence that must not be interleaved (a flash session, a
 * route check, target registration) holds it around the whole sequence.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void dap_lock(void);
/* False if another task still holds it after `timeout_ms`. */
bool dap_trylock(uint32_t timeout_ms);
void dap_unlock(void);

#ifdef __cplusplus
}
#endif
