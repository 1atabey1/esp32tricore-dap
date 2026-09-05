#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define AEL_BOARD_FAMILY_ESP32S3 1

#if CONFIG_AEL_BOARD_ESP32JTAG_S3

#define AEL_BOARD_IDENT_SUFFIX "ESP32JTAG"
#define AEL_BOARD_IS_ESP32JTAG 1

#define AEL_BOARD_HAS_FPGA 1
#define AEL_BOARD_HAS_LCD 1
#define AEL_BOARD_HAS_TARGET_VIO_PWM 1
#define AEL_BOARD_HAS_LOGIC_ANALYZER 1
#define AEL_BOARD_HAS_XVC 1
#define AEL_BOARD_HAS_SECONDARY_BUTTON 1
#define AEL_BMP_HAS_SWDIO_RDNWR 1

#define AEL_UART_PORT_NUM UART_NUM_1
#define AEL_GPIO_UART_TXD 43
#define AEL_GPIO_UART_RXD 44

#define AEL_BMP_SWCLK_PIN 47
#define AEL_BMP_SWDIO_PIN 41
#define AEL_BMP_SWDIO_RDNWR_PIN 45
#define AEL_BMP_TDI_PIN 40
#define AEL_BMP_TDO_PIN 15

/*
 * Infineon DAP master for AURIX TC3xx targets, on Port C of header J3.
 *
 * These are not free choices.  The stock FPGA bitstream fixes which S3 pin
 * reaches which connector pin, and only PC03 is bidirectional; recovered by
 * tracing the bitstream netlist, see docs/tc38x_dap_probe_plan_2026-09-05.md
 * and tools/bitstream/trace_nets.py.
 *
 *   DAP0  clock, probe-driven    -> PC02, J3 pin 23, ICE40 pin 32
 *   DAP1  data, half duplex     <-> PC03, J3 pin 25, ICE40 pin 31
 *   DAP2  wide mode              -> PC01, J3 pin 21: register-driven in the
 *                                   stock bitstream, so there is no fast path
 *                                   and wide mode waits for Phase 2
 *   TRST                         -> PC04, J3 pin 27, ICE40 pin 28
 *
 * AEL_DAP1_DIR_PIN drives the FPGA's output enable for both ends of the
 * bidirectional pair: 0 = probe drives, 1 = target drives.
 */
#define AEL_BOARD_HAS_DAP_PROBE 1
#define AEL_DAP0_PIN 47
#define AEL_DAP1_PIN 41
#define AEL_DAP1_DIR_PIN 45
#define AEL_DAP2_PIN GPIO_NUM_NC
#define AEL_DAP_TRST_PIN 40

#define AEL_PIN_NUM_CS0 GPIO_NUM_21
#define AEL_PIN_NUM_CS1 GPIO_NUM_13
#define AEL_PIN_NUM_CS2 GPIO_NUM_11
#define AEL_PIN_NUM_CS3 GPIO_NUM_5
#define AEL_PIN_NUM_CLK GPIO_NUM_38
#define AEL_PIN_NUM_MOSI GPIO_NUM_14
#define AEL_PIN_NUM_MISO GPIO_NUM_39

#define AEL_BUTTON_BOOT_PIN 0
#define AEL_BUTTON_SECONDARY_PIN 48

#define AEL_DEFAULT_PA_CFG 0
#define AEL_DEFAULT_PB_CFG 0
#define AEL_DEFAULT_PC_CFG 1
#define AEL_DEFAULT_PD_CFG 0
#define AEL_BMP_DEFAULT_FREQUENCY_HZ 4000000U

#elif CONFIG_AEL_BOARD_ESP32S3_DEVKIT

#define AEL_BOARD_IDENT_SUFFIX "ESP32S3-DEVKIT"
#define AEL_BOARD_IS_ESP32JTAG 0

#define AEL_BOARD_HAS_FPGA 0
#define AEL_BOARD_HAS_LCD 0
#define AEL_BOARD_HAS_TARGET_VIO_PWM 0
#define AEL_BOARD_HAS_LOGIC_ANALYZER 0
#define AEL_BOARD_HAS_XVC 0
#define AEL_BOARD_HAS_SECONDARY_BUTTON 0
#define AEL_BMP_HAS_SWDIO_RDNWR 0

#define AEL_UART_PORT_NUM UART_NUM_1
#define AEL_GPIO_UART_TXD 6
#define AEL_GPIO_UART_RXD 7

#define AEL_BMP_SWCLK_PIN 4
#define AEL_BMP_SWDIO_PIN 5
#define AEL_BMP_SWDIO_RDNWR_PIN GPIO_NUM_NC
#define AEL_BMP_TDI_PIN 7
#define AEL_BMP_TDO_PIN 15

/* No DAP probe on the devkit: it has no direction control for a half-duplex line. */
#define AEL_BOARD_HAS_DAP_PROBE 0
#define AEL_DAP0_PIN GPIO_NUM_NC
#define AEL_DAP1_PIN GPIO_NUM_NC
#define AEL_DAP1_DIR_PIN GPIO_NUM_NC
#define AEL_DAP2_PIN GPIO_NUM_NC
#define AEL_DAP_TRST_PIN GPIO_NUM_NC

#define AEL_PIN_NUM_CS0 GPIO_NUM_NC
#define AEL_PIN_NUM_CS1 GPIO_NUM_NC
#define AEL_PIN_NUM_CS2 GPIO_NUM_NC
#define AEL_PIN_NUM_CS3 GPIO_NUM_NC
#define AEL_PIN_NUM_CLK GPIO_NUM_NC
#define AEL_PIN_NUM_MOSI GPIO_NUM_NC
#define AEL_PIN_NUM_MISO GPIO_NUM_NC

#define AEL_BUTTON_BOOT_PIN 0
#define AEL_BUTTON_SECONDARY_PIN -1

#define AEL_DEFAULT_PA_CFG 0
#define AEL_DEFAULT_PB_CFG 0
#define AEL_DEFAULT_PC_CFG 1
#define AEL_DEFAULT_PD_CFG 0
#define AEL_BMP_DEFAULT_FREQUENCY_HZ 100000U

#else
#error "Unsupported AEL board selection"
#endif
