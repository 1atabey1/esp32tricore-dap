/*
 * Settle whether program flashing can move off DAS, before building a flasher.
 *
 * TC3xx program flash is driven entirely by memory writes to the DMU command
 * sequence interpreter - nothing executes on the target - and tas-debug already
 * does it.  The one thing that does not obviously carry across to this probe is
 * the page load: the assembly buffer takes 64-bit writes, and 32-bit loads are
 * refused with a sequence error, while the documented Cerberus IO instruction
 * catalog has 8, 16 and 32 bit accesses and nothing wider.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Enter program flash page mode, try a page load, and report whether the
 * interpreter accepted it - sweeping the undocumented IOCONF bits for the
 * EX_BUS_HW width select if a plain load is refused.
 *
 * Writes nothing to flash: no erase, no program command, no ENDINIT change.  A
 * page load with nothing behind it commits nothing, and a refused one just
 * drops page mode, which is the result being measured.
 *
 * ESP_OK when a page load was accepted, ESP_ERR_NOT_SUPPORTED when none was.
 */
esp_err_t tricore_flash_probe_width(void);

#ifdef __cplusplus
}
#endif
