/*
 * Shared between the fabric DAP files: dap_fpga_route, dap_dapisc, dap_dap2,
 * dap_wide_framing and dap_wide.  Not part of the component's interface.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* DSPR, readable without OCDS. */
#define CHECK_ADDR          0x70000000u

/* -- DAPISC ------------------------------------------------------------- */

/*
 * MODE is bits 5:4; 01B is wide, DAP1 carrying a frame's even bits and DAP2
 * the odd ones.  MAXWAIT8 at bits 12:8 shares the register, and zero there
 * disables the wait-state timeout, so the mode is ORed into DAPISC_VALUE
 * rather than replacing it: 0x0F00 is MAXWAIT8 = 15, 120 DAP0 clocks.
 */
#define DAPISC_VALUE        0x0F00u
#define DAPISC_MODE_SHIFT   4
#define DAPISC_MODE_NARROW  0u
#define DAPISC_MODE_WIDE    1u
#define DAPISC_SIGNATURE    0x4ABBAF53u

/* The register spec gives 0x09, which is client_blockwrite: sent that way the
 * telegram desynchronises the link. */
#define DAPISC_CMD          0x11u

/*
 * A wide sync reply, reassembled.  The device sends the 0xAAAAAAAA training
 * pattern on each line rather than splitting one across both, so weaving them
 * together doubles every bit.  Its CRC cannot be checked with it - six bits per
 * line make twelve - so the pattern is the whole of the evidence.
 */
#define WIDE_SYNC_EXPECT    0xCCCCCCCCu

/* Which pin modes the interface permits. */
#define OIFM_ADDR           0xF000040Cu

/* -- the target's DAP2 pin, P21.7 --------------------------------------- */

#define P21_OUT             0xF003B500u
#define P21_IOCR4           0xF003B514u
#define P21_IN              0xF003B524u
#define P21_PDISC           0xF003B560u
#define DAP2_PIN            7u

#define IOCR4_SHIFT(n)      (3u + 8u * ((n) - 4u))
#define IOCR_PP_OUT         0x10u
#define IOCR_IN_NOPULL      0x00u
#define IOCR_IN_PULLUP      0x02u

/* -- dap_dapisc.c ------------------------------------------------------- */

/* Set while a reply is sampled wide, so the decoder skips the doubled start
 * bit: wide framing sends it on both lines in one DAP0 clock. */
extern bool dap_dapisc_rx_wide;

void dap_dapisc_send(void);
/* Two short writes, to move the next telegram past the two the device swallows
 * after an attach. */
void dap_dapisc_prime(void);
bool dap_dapisc_write_read(uint8_t len_field, uint64_t data, size_t dbits,
                           uint16_t *now);

void dap_wide_report_lines(const char *what);
bool dap_wide_dap2_line_free(void);
void dap_wide_revert(void);

/* -- dap_dap2.c --------------------------------------------------------- */

void dap_wide_dap2_wiring_probe(void);
void dap_wide_halt_application(void);
void dap_wide_resume_application(void);
/* Take P21.7 off the application so the interface can drive it. */
bool dap_wide_p21_7_release(uint32_t *saved_iocr4);

/* -- dap_wide_framing.c ------------------------------------------------- */

typedef enum {
    /*
     * Every field padded to an even length and the pads covered by the CRC.
     * This is the reconstruction the RTL originally implemented; the device
     * rejects it, which is why the others exist.
     */
    WF_PAD_CRC = 0,
    /* Padded the same way, but the CRC covers only the real bits. */
    WF_PAD_NOCRC,
    /*
     * No padding at all: CMD, LEN, DATA and the CRC are one continuous stream
     * split between the lines, so LEN starts on DAP2 because CMD's five bits
     * leave the parity odd.  The CRC is then the narrow one unchanged.
     */
    WF_STREAM,
    /*
     * The narrow frame exactly, start bit included, split between the lines -
     * which puts the start bit on DAP1 alone.
     */
    WF_STREAM_1START,
    WF_COUNT
} wide_framing_t;

const char *dap_wide_framing_name(wide_framing_t v);
bool dap_wide_framing_sweep(wide_framing_t *rule, uint8_t *tap1_out,
                            uint8_t *tap2_out);

/* The application owns P21.7; put its setting back on every exit. */
extern uint32_t dap_wide_iocr4_saved;
extern bool     dap_wide_iocr4_dirty;

/* Stop after stage n, reverting first when the stage left the link wide. */
#define WIDE_STAGE(n, revert)                                                 \
    do {                                                                      \
        if (dap_wide_stage && dap_wide_stage <= (n)) {                            \
            ESP_LOGW(TAG, "  stopping after stage %d as asked", (n));         \
            if (revert) { dap_wide_revert(); }                                    \
            return;                                                           \
        }                                                                     \
    } while (0)


/* How far into the bring-up to go, and which capture taps to use; set from
 * /api/dap_fpga. */
extern int dap_wide_stage, dap_wide_tap1, dap_wide_tap2, dap_wide_trail;

bool dap_wide_calibrate(uint8_t *tap1_out, uint8_t *tap2_out);

/* -- dap_wide.c --------------------------------------------------------- */

void dap_wide_route_check(void);
