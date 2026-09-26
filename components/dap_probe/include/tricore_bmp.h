/*
 * Register the TC3xx cores as Black Magic Probe targets, through BMP's public
 * target_new() API, so its GDB server on port 4242 can serve them.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Attach the probe, enable OCDS, find the cores and register one BMP target
 * per core, so GDB can `attach 1`. */
esp_err_t tricore_bmp_probe(void);

#ifdef __cplusplus
}
#endif
