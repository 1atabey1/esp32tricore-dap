/*
 * Shared between the fabric DAP files: dap_fpga_route, dap_dapisc, dap_wide.
 * Not part of the component's interface.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* DSPR, readable without OCDS. */
#define CHECK_ADDR          0x70000000u

/* -- DAPISC ------------------------------------------------------------- */

/* MODE is bits 5:4 (01B wide).  MAXWAIT8 (bits 12:8) shares the register and
 * zero there disables the wait timeout, so the mode is ORed into 0x0F00. */
#define DAPISC_VALUE        0x0F00u
#define DAPISC_MODE_SHIFT   4
#define DAPISC_MODE_NARROW  0u
#define DAPISC_MODE_WIDE    1u
#define DAPISC_SIGNATURE    0x4ABBAF53u

/* The register spec says 0x09 (client_blockwrite); 0x11 is what works. */
#define DAPISC_CMD          0x11u

/* Wide sync: 0xAAAAAAAA on each line, reassembled.  Its CRC is not checkable. */
#define WIDE_SYNC_EXPECT    0xCCCCCCCCu

/* OIFM.DAPMODE (bits 2:0): which pin modes the interface permits. */
#define OIFM_ADDR           0xF000040Cu

/* -- the target's DAP2 pin, P21.7 --------------------------------------- */

#define P21_IOCR4           0xF003B514u
#define DAP2_PIN            7u

#define IOCR4_SHIFT(n)      (3u + 8u * ((n) - 4u))
#define IOCR_PP_OUT         0x10u

/* -- dap_dapisc.c ------------------------------------------------------- */

/* Set while a reply is sampled wide: the start bit then fills two positions. */
extern bool dap_dapisc_rx_wide;

/* Long-form DAPISC write, no reply expected (cold attach). */
void dap_dapisc_send(void);
/* Write DAPISC, decode the value it now holds. */
bool dap_dapisc_write_read(uint8_t len_field, uint64_t data, size_t dbits,
                           uint16_t *now);
/* Two short writes: the device swallows the first two after an attach. */
void dap_dapisc_prime(void);
/* Back to narrow mode; resets the target if sync does not answer. */
bool dap_dapisc_narrow(void);

/* -- dap_wide.c --------------------------------------------------------- */

esp_err_t dap_wide_check(void);

/* -- dap_fpga_route.c --------------------------------------------------- */

/* 1 kB block reads at each DAP clock divider, logged as kB/s. */
void dap_fpga_block_read_sweep(const char *label);
