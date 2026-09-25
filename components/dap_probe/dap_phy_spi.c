/*
 * GP-SPI backend for the DAP physical layer: LSB-first, bit-length transfers.
 *
 * MOSI and MISO share the data pad (half-duplex; the FPGA direction pin picks
 * the driver).  SPI mode 0 shifts out on the falling edge and samples on the
 * rising edge, matching DAP.
 */

#include "dap_phy.h"

#include <inttypes.h>
#include <string.h>
#include <sys/param.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_rom_gpio.h"
#include "hal/spi_types.h"
#include "soc/spi_periph.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_sig_map.h"

static const char *TAG = "DAP_SPI";

/* Provided by the application, which owns the SPI buses; weak default. */
__attribute__((weak)) esp_err_t spi_release_xvc_bus(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

/* SPI2 (shared only with XVC); SPI3 carries the FPGA loader and logic analyser. */
#define DAP_SPI_HOST      SPI2_HOST
#define DAP_SPI_MAX_BYTES 160

static spi_device_handle_t s_dev;
static bool                s_ready;
static int                 s_dat_pin = -1;
static int                 s_clk_pin = -1;

bool dap_phy_spi_ready(void)
{
    return s_ready;
}

esp_err_t dap_phy_spi_init(int clk_pin, int dat_pin, uint32_t clock_hz)
{
    if (s_ready) {
        return ESP_OK;
    }

    s_dat_pin = dat_pin;
    s_clk_pin = clk_pin;

    /* MISO is wired to the data pad by hand below (the bus config rejects MOSI == MISO). */
    const spi_bus_config_t bus = {
        .mosi_io_num     = dat_pin,
        .miso_io_num     = -1,
        .sclk_io_num     = clk_pin,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t err = spi_bus_initialize(DAP_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        /* Initialised by someone else, with other pins: reclaim it and retry. */
        ESP_LOGW(TAG, "SPI2 is already initialised; asking for it back");
        if (spi_release_xvc_bus() == ESP_OK) {
            err = spi_bus_initialize(DAP_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        return err;
    }

    const spi_device_interface_config_t dev = {
        .clock_speed_hz = (int)clock_hz,
        .mode           = 0,                       /* out on falling, in on rising */
        .spics_io_num   = -1,                      /* DAP has no chip select */
        .queue_size     = 4,
        .flags          = SPI_DEVICE_BIT_LSBFIRST | SPI_DEVICE_HALFDUPLEX,
        .input_delay_ns = 30,
    };
    err = spi_bus_add_device(DAP_SPI_HOST, &dev, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
        return err;
    }

    /* Feed the peripheral's input from the data pad as well as driving it. */
    esp_rom_gpio_connect_in_signal(dat_pin,
                                   spi_periph_signal[DAP_SPI_HOST].spiq_in,
                                   false);

    s_ready = true;
    ESP_LOGI(TAG, "GP-SPI backend on SPI2: clk=GPIO%d data=GPIO%d at %" PRIu32 " Hz",
             clk_pin, dat_pin, clock_hz);
    return ESP_OK;
}

void dap_phy_spi_set_clock(uint32_t clock_hz)
{
    /* Changing the clock means removing and re-adding the device. */
    if (!s_ready) {
        return;
    }
    spi_bus_remove_device(s_dev);
    const spi_device_interface_config_t dev = {
        .clock_speed_hz = (int)clock_hz,
        .mode           = 0,
        .spics_io_num   = -1,
        .queue_size     = 4,
        .flags          = SPI_DEVICE_BIT_LSBFIRST | SPI_DEVICE_HALFDUPLEX,
        .input_delay_ns = 30,
    };
    if (spi_bus_add_device(DAP_SPI_HOST, &dev, &s_dev) != ESP_OK) {
        s_ready = false;
    }
}

/* DMA buffers: must be word-aligned internal RAM.  Transfers are serialised. */
static WORD_ALIGNED_ATTR uint8_t s_txbuf[DAP_SPI_MAX_BYTES];
static WORD_ALIGNED_ATTR uint8_t s_rxbuf[DAP_SPI_MAX_BYTES];

esp_err_t dap_phy_spi_write_bits(const uint8_t *bits, size_t nbits)
{
    const size_t nbytes = (nbits + 7) / 8;

    if (nbytes > sizeof(s_txbuf) || !s_ready) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(s_txbuf, 0, nbytes);
    for (size_t i = 0; i < nbits; i++) {
        if (bits[i] & 1u) {
            s_txbuf[i / 8] |= (uint8_t)(1u << (i % 8));   /* LSB first per byte */
        }
    }

    spi_transaction_t t = {
        .length    = nbits,          /* bits, not bytes */
        .tx_buffer = s_txbuf,
    };
    const esp_err_t err = spi_device_polling_transmit(s_dev, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write %u bits: %s", (unsigned)nbits, esp_err_to_name(err));
    }
    return err;
}

esp_err_t dap_phy_spi_read_bits(uint8_t *bits, size_t nbits)
{
    const size_t nbytes = (nbits + 7) / 8;

    if (nbytes > sizeof(s_rxbuf) || !s_ready) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(s_rxbuf, 0, MIN(nbytes + 4, sizeof(s_rxbuf)));

    /* Read-only half-duplex: no write phase, rxlength bits clocked in. */
    spi_transaction_t t = {
        .length    = 0,
        .rxlength  = nbits,
        .rx_buffer = s_rxbuf,
    };
    const esp_err_t err = spi_device_polling_transmit(s_dev, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read %u bits: %s", (unsigned)nbits, esp_err_to_name(err));
        return err;
    }
    for (size_t i = 0; i < nbits; i++) {
        bits[i] = (uint8_t)((s_rxbuf[i / 8] >> (i % 8)) & 1u);
    }
    return ESP_OK;
}

void dap_phy_spi_route(bool to_spi)
{
    /* While routed to SPI the pads ignore gpio_set_level(). */
    if (!s_ready) {
        return;
    }
    if (to_spi) {
        /*
         * Direction before signal: gpio_set_direction() resets func_sel to
         * SIG_GPIO_OUT_IDX.  Data needs INPUT_OUTPUT to keep its input buffer.
         */
        gpio_set_direction((gpio_num_t)s_clk_pin, GPIO_MODE_OUTPUT);
        gpio_set_direction((gpio_num_t)s_dat_pin, GPIO_MODE_INPUT_OUTPUT);
        esp_rom_gpio_connect_out_signal(s_clk_pin,
                                        spi_periph_signal[DAP_SPI_HOST].spiclk_out,
                                        false, false);
        esp_rom_gpio_connect_out_signal(s_dat_pin,
                                        spi_periph_signal[DAP_SPI_HOST].spid_out,
                                        false, false);
    } else {
        /* Restore the IO_MUX GPIO function as well as the matrix selector. */
        esp_rom_gpio_pad_select_gpio(s_clk_pin);
        esp_rom_gpio_pad_select_gpio(s_dat_pin);
        esp_rom_gpio_connect_out_signal(s_clk_pin, SIG_GPIO_OUT_IDX, false, false);
        esp_rom_gpio_connect_out_signal(s_dat_pin, SIG_GPIO_OUT_IDX, false, false);
        gpio_set_direction((gpio_num_t)s_clk_pin, GPIO_MODE_OUTPUT);
        gpio_set_direction((gpio_num_t)s_dat_pin, GPIO_MODE_OUTPUT);
    }
}

void dap_phy_spi_log_routing(void)
{
    if (!s_ready) {
        return;
    }
    ESP_LOGI(TAG, "pad routing: clk(GPIO%d) func_sel=%u  dat(GPIO%d) func_sel=%u "
                  "(%u means plain GPIO; SPI2 clk is %u, data %u)",
             s_clk_pin, (unsigned)GPIO.func_out_sel_cfg[s_clk_pin].func_sel,
             s_dat_pin, (unsigned)GPIO.func_out_sel_cfg[s_dat_pin].func_sel,
             (unsigned)SIG_GPIO_OUT_IDX,
             (unsigned)spi_periph_signal[DAP_SPI_HOST].spiclk_out,
             (unsigned)spi_periph_signal[DAP_SPI_HOST].spid_out);
}

/* Full-duplex loopback on the data pad; uses its own (full-duplex) device handle. */
esp_err_t dap_phy_spi_loopback(uint8_t pattern)
{
    spi_device_handle_t fd = NULL;
    const spi_device_interface_config_t dev = {
        .clock_speed_hz = 1000000,
        .mode           = 0,
        .spics_io_num   = -1,
        .queue_size     = 1,
        .flags          = SPI_DEVICE_BIT_LSBFIRST,
        .input_delay_ns = 30,
    };

    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = spi_bus_add_device(DAP_SPI_HOST, &dev, &fd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "loopback add_device: %s", esp_err_to_name(err));
        return err;
    }

    s_txbuf[0] = pattern;
    s_txbuf[1] = (uint8_t)~pattern;
    s_rxbuf[0] = 0;
    s_rxbuf[1] = 0;

    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = s_txbuf,
        .rx_buffer = s_rxbuf,
    };
    err = spi_device_polling_transmit(fd, &t);
    if (err == ESP_OK) {
        const bool match = (s_rxbuf[0] == s_txbuf[0]) && (s_rxbuf[1] == s_txbuf[1]);
        ESP_LOGW(TAG, "loopback wrote %02X %02X read %02X %02X - %s",
                 s_txbuf[0], s_txbuf[1], s_rxbuf[0], s_rxbuf[1],
                 match ? "the peripheral reaches the pads"
                       : "MISMATCH: clocking or pad routing is wrong");
        if (!match) {
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "loopback transfer: %s", esp_err_to_name(err));
    }
    spi_bus_remove_device(fd);
    return err;
}

/* Send `nbits` full-duplex and return what the pad carried (checks partial bytes). */
esp_err_t dap_phy_spi_echo_bits(const uint8_t *bits, size_t nbits, uint8_t *out)
{
    spi_device_handle_t fd = NULL;
    const spi_device_interface_config_t dev = {
        .clock_speed_hz = 1000000,
        .mode           = 0,
        .spics_io_num   = -1,
        .queue_size     = 1,
        .flags          = SPI_DEVICE_BIT_LSBFIRST,
        .input_delay_ns = 30,
    };
    const size_t nbytes = (nbits + 7) / 8;

    if (!s_ready || nbytes > sizeof(s_txbuf)) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = spi_bus_add_device(DAP_SPI_HOST, &dev, &fd);
    if (err != ESP_OK) {
        return err;
    }

    memset(s_txbuf, 0, nbytes);
    memset(s_rxbuf, 0, MIN(nbytes + 4, sizeof(s_rxbuf)));
    for (size_t i = 0; i < nbits; i++) {
        if (bits[i] & 1u) {
            s_txbuf[i / 8] |= (uint8_t)(1u << (i % 8));
        }
    }

    spi_transaction_t t = {
        .length    = nbits,
        .tx_buffer = s_txbuf,
        .rx_buffer = s_rxbuf,
    };
    err = spi_device_polling_transmit(fd, &t);
    if (err == ESP_OK) {
        for (size_t i = 0; i < nbits; i++) {
            out[i] = (uint8_t)((s_rxbuf[i / 8] >> (i % 8)) & 1u);
        }
    }
    spi_bus_remove_device(fd);
    return err;
}

/* Sample the clock pad via GPIO input during a long transfer; ESP_OK if it toggles. */
esp_err_t dap_phy_spi_clock_pad_check(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Keep the SPI output routing and add the input buffer. */
    gpio_set_direction((gpio_num_t)s_clk_pin, GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal(s_clk_pin,
                                    spi_periph_signal[DAP_SPI_HOST].spiclk_out,
                                    false, false);

    memset(s_txbuf, 0x55, sizeof(s_txbuf));
    spi_transaction_t t = {
        .length    = sizeof(s_txbuf) * 8,
        .tx_buffer = s_txbuf,
    };
    esp_err_t err = spi_device_queue_trans(s_dev, &t, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "clock check queue: %s", esp_err_to_name(err));
        return err;
    }

    int high = 0, low = 0;
    for (int i = 0; i < 20000; i++) {
        if (gpio_get_level((gpio_num_t)s_clk_pin)) {
            high++;
        } else {
            low++;
        }
    }

    spi_transaction_t *done = NULL;
    err = spi_device_get_trans_result(s_dev, &done, pdMS_TO_TICKS(1000));
    ESP_LOGW(TAG, "clock pad GPIO%d during a %u-bit transfer: %d high, %d low - %s",
             s_clk_pin, (unsigned)t.length, high, low,
             (high && low) ? "it is clocking" : "IT IS NOT CLOCKING");
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "clock check result: %s", esp_err_to_name(err));
    }
    return (high && low) ? ESP_OK : ESP_FAIL;
}
