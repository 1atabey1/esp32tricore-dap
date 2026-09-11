/*
 * GP-SPI backend for the DAP physical layer.
 *
 * The bit-banged backend proved the protocol and reached 116 kB/s on block
 * reads, but measurement says the limit is the GPIO calls rather than DAP: 33.5
 * us per 33-bit word is about 1 MHz effective against a 4 MHz setting.  The S3
 * has a hardware serialiser that does exactly what a DAP frame needs -
 * LSB-first, a transfer length expressed in *bits*, and DMA - so the clocking
 * moves into it and the frame layer above is untouched.
 *
 * Three details make this work on one half-duplex wire:
 *
 *   - MOSI and MISO are both routed to the data pin.  Only one side drives at a
 *     time, decided by the FPGA's direction control, so the peripheral can read
 *     back on the same wire it writes.
 *   - The pad's output driver is switched off during a reply, exactly as the
 *     bit-bang path does, so the S3 and the FPGA never fight over the pin.
 *   - SPI mode 0 shifts MOSI out on the falling edge and samples on the rising
 *     one, which is what DAP asks for: the target latches on the rising edge
 *     and drives its reply after the falling edge.
 */

#include "dap_phy.h"

#include <inttypes.h>
#include <string.h>

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

/*
 * Provided by the application, which owns the SPI buses.  Weak here so this
 * component still links on its own - a board that never set the buses up has
 * nothing to release.
 */
__attribute__((weak)) esp_err_t spi_release_xvc_bus(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

/*
 * SPI2, not SPI3.  SPI3 looks free - it is nominally the LCD's - but the FPGA
 * bitstream loader and every logic-analyser capture go through a device on it
 * too, and without the bitstream the DAP pins never reach the connector at
 * all.  Taking it made the target go silent in a way that looked exactly like
 * a protocol fault.  SPI2 carries only the XVC devices, and XVC is the thing
 * this probe is meant to replace.
 */
#define DAP_SPI_HOST      SPI2_HOST
#define DAP_SPI_MAX_BYTES 160        /* a 1 kB block reply is bigger; see below */

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

    /*
     * MISO is left unset here and wired by hand below: giving the bus config
     * the same GPIO for MOSI and MISO is rejected, but routing the peripheral's
     * input signal to that pad through the GPIO matrix is not.
     */
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
        /*
         * Already initialised, which means its pin assignment - not ours - is
         * what the peripheral drives; adding a device to that bus would clock
         * the wrong pads and fail silently.  So ask the application for the
         * bus back and try again rather than guessing.
         */
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
    /*
     * Changing an added device's clock means removing and re-adding it, which
     * is cheap and only happens between measurements.
     */
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

/*
 * DMA needs these word-aligned and in internal RAM, which a stack array is not
 * guaranteed to be - an unaligned buffer comes back as ESP_ERR_INVALID_ARG and
 * nothing reaches the wire.  Static is fine here: every DAP transfer is
 * serialised behind one probe.
 */
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
    memset(s_rxbuf, 0, nbytes + 4);

    /*
     * Read-only half-duplex: no write phase, rxlength bits clocked in.  The
     * clock still comes from us, which is the whole point - the target only
     * advances on our edges.
     */
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
    /*
     * The two backends cannot share the pads without this.
     *
     * spi_bus_initialize() points the clock and data pads at the peripheral's
     * output signals, and from then on gpio_set_level() on those pins does
     * nothing at all - the pad ignores the GPIO output register.  A bit-bang
     * frame sent in that state clocks nothing and looks exactly like a silent
     * target.  So switching backends means switching the pad's output source
     * back and forth; SIG_GPIO_OUT_IDX is the "plain GPIO" selector.
     */
    if (!s_ready) {
        return;
    }
    if (to_spi) {
        /*
         * Direction first, signal second, and not the other way round.
         * gpio_set_direction(OUTPUT) ends with
         *
         *     func_out_sel_cfg[pin].func_sel = SIG_GPIO_OUT_IDX;
         *
         * to make sure no peripheral is still driving the pad - so calling it
         * after wiring the SPI signal quietly undoes the wiring.  That is
         * exactly what happened here: both pads read back as plain GPIO, the
         * loopback returned zeros, and it looked like a dead target.
         */
        gpio_set_direction((gpio_num_t)s_clk_pin, GPIO_MODE_OUTPUT);
        /*
         * INPUT_OUTPUT, not OUTPUT: the reply arrives on the same pad this
         * drives, so the pad input buffer has to stay enabled.  GPIO_MODE_OUTPUT
         * switches it off, and then every read comes back as zeros - which is
         * what the loopback saw.
         */
        gpio_set_direction((gpio_num_t)s_dat_pin, GPIO_MODE_INPUT_OUTPUT);
        esp_rom_gpio_connect_out_signal(s_clk_pin,
                                        spi_periph_signal[DAP_SPI_HOST].spiclk_out,
                                        false, false);
        esp_rom_gpio_connect_out_signal(s_dat_pin,
                                        spi_periph_signal[DAP_SPI_HOST].spid_out,
                                        false, false);
    } else {
        /*
         * Restoring the matrix selector is not enough on its own: the pad also
         * has an IO_MUX function select, and if that is left pointing at
         * anything but the GPIO function the matrix output is bypassed
         * entirely.  esp_rom_gpio_pad_select_gpio() puts it back, and only
         * then does the GPIO output register drive the pin again.  Without
         * this the data line came back (its readback test passes either way,
         * because it reads its own pad) while the clock silently did not, and
         * a probe with no clock looks exactly like a dead target.
         */
        esp_rom_gpio_pad_select_gpio(s_clk_pin);
        esp_rom_gpio_pad_select_gpio(s_dat_pin);
        esp_rom_gpio_connect_out_signal(s_clk_pin, SIG_GPIO_OUT_IDX, false, false);
        esp_rom_gpio_connect_out_signal(s_dat_pin, SIG_GPIO_OUT_IDX, false, false);
        gpio_set_direction((gpio_num_t)s_clk_pin, GPIO_MODE_OUTPUT);
        gpio_set_direction((gpio_num_t)s_dat_pin, GPIO_MODE_OUTPUT);
    }
}

/* What the pads are actually pointing at, for when a log line is wanted. */
void dap_phy_spi_log_routing(void)
{
    if (!s_ready) {
        return;
    }
    ESP_LOGI(TAG, "pad routing: clk(GPIO%d) func_sel=%u  dat(GPIO%d) func_sel=%u "
                  "(%u means plain GPIO; SPI3 clk is %u, data %u)",
             s_clk_pin, (unsigned)GPIO.func_out_sel_cfg[s_clk_pin].func_sel,
             s_dat_pin, (unsigned)GPIO.func_out_sel_cfg[s_dat_pin].func_sel,
             (unsigned)SIG_GPIO_OUT_IDX,
             (unsigned)spi_periph_signal[DAP_SPI_HOST].spiclk_out,
             (unsigned)spi_periph_signal[DAP_SPI_HOST].spid_out);
}

/*
 * Loopback self-test: does the peripheral actually reach these pads?
 *
 * MOSI and MISO are both on the data pin, so a full-duplex transfer with the
 * probe driving reads back exactly what it wrote - if and only if the clock is
 * being generated, the output routing works, and the input routing works.  A
 * mismatch says the wiring inside the chip is wrong and there is no point
 * blaming the target.  It needs its own device handle because the DAP device
 * is half-duplex by design.
 */
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

/*
 * Send a frame full-duplex and hand back what the pad actually carried.
 *
 * The loopback above proves the peripheral reaches the pads with a whole byte;
 * this proves the part that a byte cannot: that a length which is not a
 * multiple of eight is serialised the way the frame layer means it - 19 bits
 * for sync, LSB first, with the last partial byte taking its low bits.  If the
 * readback matches the frame then the wire carried the right pattern and any
 * silence from the target is about timing or protocol, not bit order.
 */
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
    memset(s_rxbuf, 0, nbytes + 4);
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

/*
 * Does the clock pad actually move?
 *
 * Everything else is verified: the frame serialises correctly and the data pad
 * carries it out and back.  The clock is the one signal with no readback path,
 * because SPI generates it and nothing samples it - and a DAP target with no
 * clock is indistinguishable from a dead one, since it advances only on our
 * edges.  So queue a long transfer, watch the pad through the GPIO input while
 * it runs, and report how many samples saw each level.
 *
 * The pad keeps its input buffer enabled for this, which costs nothing: it is
 * an output either way, and reading an output pad reads what is on it.
 */
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
