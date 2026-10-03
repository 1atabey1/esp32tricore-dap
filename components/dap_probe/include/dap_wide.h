/*
 * DAP wide mode (DAPISC.MODE = 01B) as a session: DAP1 carries a frame's even
 * bits, DAP2 (target P21.7) the odd ones.  Requires P21.7 to be an input on
 * the target (the application must not use it).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Set the fabric divider `div`, switch target and fabric to wide mode and
 * calibrate the capture taps at that clock (they depend on it).  Refuses if
 * P21.7 is driven by the target.  Leaves narrow (at `div`) on failure.
 */
esp_err_t dap_wide_enter(uint8_t div);

/* Back to narrow (resets the target's DAP only if narrow does not answer). */
void dap_wide_exit(void);

bool dap_wide_active(void);

/*
 * Fast mode (a bit every fabric clock) as a session, narrow or wide: find the
 * receive timing (LAG, sampling edges, capture taps) against reference data
 * in the trace RAM, trying the settings known to work first and stopping at
 * the first that carries data three times running; a wrong LAG can desync the
 * DAP, so the search is not exhaustive.  Leaves fast mode off on failure.
 */
esp_err_t dap_fast_enter(void);
void      dap_fast_exit(void);

#ifdef __cplusplus
}
#endif
