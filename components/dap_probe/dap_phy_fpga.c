#include "dap_phy_fpga.h"

#include <inttypes.h>
#include <string.h>

#include "board_profile.h"
#include "driver/spi_master.h"
#include "soc/spi_periph.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "esp_rom_gpio.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DAP_FPGA";

/* Provided by the application, which owns the SPI buses. */
extern esp_err_t spi_release_xvc_bus(void);

/*
 * The FPGA sits on the bus the application already set up for the fabric -
 * AEL_PIN_NUM_CLK/MOSI/MISO with CS0 - and that same bus is what loads the
 * bitstream.  So this adds a device rather than taking the bus over: nothing
 * else needs to be torn down, and configuration still works afterwards.
 */
#define FPGA_SPI_HOST   SPI2_HOST

/*
 * 11.4 MHz, which is 80/7 and the last step below the ceiling.
 *
 * The fabric runs at 48 MHz and its SPI slave oversamples an asynchronous SCK,
 * so it needs sysclk >= 4x SCK - 12 MHz.  Past that it does not degrade
 * gracefully, it drops bits silently.
 *
 * This is the system's bottleneck, and by some margin.  With the wire at
 * 24 MHz a 1 kB block spends 0.35 ms on the DAP and 0.72 ms shifting the
 * answer over this link, so every further doubling of the wire - wide mode
 * included - buys almost nothing until the slave stops oversampling SCK and
 * clocks it directly.  It started at 1 MHz, where the drain took 8 ms of a
 * 13 ms block and the whole thing managed 75 kB/s.
 */
#define FPGA_SPI_HZ     (40000 * 1000)

/*
 * Bytes to collect per drain while a block is still arriving.  Each drain is a
 * transaction of its own, so too small and the per-transaction overhead
 * outweighs the overlap; too large and the last chunk waits on the whole block
 * anyway.  Half a block is the compromise.
 */
#define DRAIN_CHUNK     512

/* Register map, mirrored from fpga/dap_master/rtl/dap_top.v. */
#define REG_STATUS      0x00
#define REG_CTRL        0x01
#define REG_DIV         0x02
#define REG_CMD         0x03
#define REG_LEN         0x04
#define REG_DBITS       0x05
#define REG_RBITS       0x06
#define REG_TRAIL       0x07
#define REG_MAXWAIT     0x08
#define REG_PARCELS     0x0A
#define REG_FLAGS       0x0B
#define REG_LEAD        0x0C
#define REG_LEVEL       0x0D    /* 16-bit, bytes waiting in the reply FIFO */
#define REG_SKEW        0x0F    /* capture tap: [1:0] DAP1, [3:2] DAP2 */

#define REG_DATA        0x10
#define REG_REPLY       0x20
#define REG_RCRC        0x24
#define REG_WAIT        0x25
#define REG_FIFO        0x40
#define REG_WLEVEL      0x43    /* 16-bit, bytes waiting in the write FIFO */
#define REG_WFIFO       0x48    /* byte port into the write FIFO */
#define REG_SCAN        0x41    /* 16-bit: every other BANK0 pad's level */

#define CTRL_START_FRAME (1u << 0)
#define CTRL_START_BLOCK (1u << 1)
#define CTRL_ABORT       (1u << 2)
#define CTRL_CLEAR_FIFO  (1u << 3)
#define CTRL_START_WRITE (1u << 4)
#define CTRL_CLEAR_WFIFO (1u << 5)

#define ST_BUSY          (1u << 0)
#define ST_DONE          (1u << 1)
#define ST_TIMED_OUT     (1u << 2)
#define ST_IDLE_HIGH     (1u << 3)
#define ST_CRC_OK        (1u << 4)
#define ST_FIFO_EMPTY    (1u << 5)
#define ST_FIFO_FULL     (1u << 6)
#define ST_OVERRUN       (1u << 7)

#define FLAG_TRST        (1u << 0)
#define FLAG_RAW_WINDOW  (1u << 1)
#define FLAG_WIDE        (1u << 2)
#define FLAG_RAW_FRAME   (1u << 3)
#define FLAG_RX_WIDE     (1u << 5)

/*
 * The wide-mode start-bit alignment, in the reply group's spare slot.
 *
 * Where it is read from is not cosmetic: it comes from the receiver, at the far
 * end of the chip from the register read mux, so the decode it joins decides
 * how much clock it costs.  In the control group the bitstream closes 46.7 MHz;
 * in LEVEL's high byte, which this file's own drain loop polls thousands of
 * times per block, 48.5 - at which it starts losing bytes at the slow
 * dividers.  Here, in a slot the reply group already spent on a constant, it
 * closes 50.4.
 */
#define REG_LINES        0x27
#define LINES_ALIGNED    (1u << 0)
#define LINES_TX2_LO     (1u << 1)
#define LINES_TX2_HI     (1u << 2)
#define LINES_RX2_LO     (1u << 3)
#define LINES_RX2_HI     (1u << 4)
#define LINES_RX1_LO     (1u << 5)
#define LINES_RX1_HI     (1u << 6)

#define WRITE_BIT        0x80

/* Longest a command should ever take: a 256-parcel block at the slowest
 * sensible bit rate is a few milliseconds, so this is generous. */
#define OP_TIMEOUT_MS    250

static spi_device_handle_t s_dev;
static bool                s_ready;
static bool                s_in_use;

/* DMA wants these word-aligned and in internal RAM. */
static WORD_ALIGNED_ATTR uint8_t s_buf[1024];

/* ------------------------------------------------------------------------ */
/* Register access                                                           */
/* ------------------------------------------------------------------------ */

/*
 * The header byte goes in the command phase rather than the data phase.  In
 * half-duplex with DMA that keeps the address out of the way of the payload,
 * which is what lets a 1 kB read be one transaction the host does not touch.
 */
/* The select, held for the whole transaction - header byte included, since the
 * fabric latches the address on the first eight clocks after it falls. */
static inline void cs_low(void)  { gpio_set_level(AEL_PIN_NUM_CS0, 0); }
static inline void cs_high(void) { gpio_set_level(AEL_PIN_NUM_CS0, 1); }

/*
 * Header and payload in one full-duplex transfer, not a command phase.
 *
 * The fabric slave counts every clock from the select falling and treats the
 * first eight as the header - which is exactly what the simulation verified,
 * with the header driven as an ordinary first byte.  Sending it as an ESP-IDF
 * command phase instead is a different shape on the wire, and the symptom was
 * a slave that saw the select and the clocks and still shifted out nothing.
 *
 * Full duplex also means one buffer carries both directions, so a read is
 * "send the header then clock zeros, take what comes back from byte one".
 */
static WORD_ALIGNED_ATTR uint8_t s_tx[1088];
static WORD_ALIGNED_ATTR uint8_t s_rx[1088];

static esp_err_t xfer(uint8_t header, const uint8_t *tx, uint8_t *rx, size_t len)
{
    /*
     * A read costs one byte more than a write: the header, then a dummy the
     * fabric sends while it fetches, then the data.  That byte is what gives
     * the register file a whole byte time to answer instead of half an SCK
     * period, which is what lets the fabric clock reach 48 MHz - and at 0.1%
     * of a 1 kB block it is the cheapest part of this.
     */
    const size_t lead = (rx != NULL) ? 2u : 1u;

    if (len + lead > sizeof(s_tx)) {
        return ESP_ERR_INVALID_SIZE;
    }

    s_tx[0] = header;
    if (tx != NULL) {
        memcpy(s_tx + lead, tx, len);
    } else {
        memset(s_tx + 1, 0, len + lead - 1);
    }

    spi_transaction_t t = {
        .length    = (len + lead) * 8,
        .tx_buffer = s_tx,
        .rx_buffer = s_rx,
    };

    cs_low();
    const esp_err_t err = spi_device_polling_transmit(s_dev, &t);
    cs_high();

    if (err == ESP_OK && rx != NULL) {
        memcpy(rx, s_rx + lead, len);
    }
    return err;
}

static esp_err_t reg_write(uint8_t addr, const uint8_t *data, size_t len)
{
    return xfer((uint8_t)(addr | WRITE_BIT), data, NULL, len);
}

static esp_err_t reg_read(uint8_t addr, uint8_t *data, size_t len)
{
    return xfer(addr, NULL, data, len);
}

static esp_err_t reg_write8(uint8_t addr, uint8_t value)
{
    return reg_write(addr, &value, 1);
}

static uint8_t reg_read8(uint8_t addr)
{
    uint8_t v = 0;
    const esp_err_t err = reg_read(addr, &v, 1);

    if (err != ESP_OK) {
        /*
         * Returning zero on a failed transaction is indistinguishable from the
         * fabric answering zero, and that ambiguity cost real time here: every
         * symptom pointed at the slave while the transfer may never have been
         * issued at all.
         */
        ESP_LOGE(TAG, "register read of 0x%02X failed: %s", addr, esp_err_to_name(err));
        return 0;
    }
    return v;
}

/* ------------------------------------------------------------------------ */
/* Bring-up                                                                  */
/* ------------------------------------------------------------------------ */

esp_err_t dap_phy_fpga_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    /*
     * Chip select driven by hand, not by the driver.
     *
     * The same pin is the bitstream loader's chip select, and that loader
     * bit-bangs it with gpio_set_level - which only works while the pad is
     * under GPIO control.  Configuration demonstrably works, so the pad is GPIO
     * controlled, which means the driver's CS signal is *not* reaching it and a
     * driver-managed select would never assert.  The existing logic-analyser
     * device on this bus drives its own select for the same reason.
     */
    const spi_device_interface_config_t dev = {
        .clock_speed_hz = FPGA_SPI_HZ,
        .mode           = 0,                      /* what the fabric expects */
        .spics_io_num   = -1,
        .queue_size     = 2,
        .flags          = 0,
    };

    /*
     * Take the bus over rather than assuming what it was set up with.
     *
     * Adding a device to an existing bus inherits that bus's pin assignment,
     * and this one is shared: the application configures it for the fabric, the
     * GP-SPI DAP backend re-initialises it on the DAP pins, and the bitstream
     * loader takes the pads back as plain GPIO in between.  Pointing the pads
     * at the peripheral by hand is not enough if the peripheral is set up for a
     * different set of pins - which showed up as a transaction that returned
     * success and produced almost no clock edges at all.
     *
     * Freeing first costs the XVC devices, which is the same trade the DAP
     * bitstream already makes: the fabric it needs is not loaded anyway.
     */
    spi_release_xvc_bus();

    const spi_bus_config_t bus = {
        .mosi_io_num     = AEL_PIN_NUM_MOSI,
        .miso_io_num     = AEL_PIN_NUM_MISO,
        .sclk_io_num     = AEL_PIN_NUM_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 2048,
    };
    esp_err_t err = spi_bus_initialize(FPGA_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        return err;
    }

    err = spi_bus_add_device(FPGA_SPI_HOST, &dev, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
        return err;
    }

    /*
     * Put the pads back under the SPI peripheral.
     *
     * Configuring the FPGA bit-bangs these same pins, and doing that requires
     * taking them back as plain GPIO - so after any bitstream load the
     * peripheral is no longer connected to them and every transaction here
     * would go nowhere.  Adding a device does not re-route them; bus
     * initialisation did that once, long ago.
     *
     * Direction first, signal second: gpio_set_direction ends by pointing the
     * pad at the GPIO output register, so doing it afterwards would quietly
     * undo the wiring.
     */
    /* The select is ours to drive, so make sure the pad is plain GPIO and
     * parked deselected before the first transaction. */
    gpio_set_direction(AEL_PIN_NUM_CS0, GPIO_MODE_OUTPUT);
    gpio_set_level(AEL_PIN_NUM_CS0, 1);

    gpio_set_direction(AEL_PIN_NUM_CLK,  GPIO_MODE_OUTPUT);
    gpio_set_direction(AEL_PIN_NUM_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(AEL_PIN_NUM_MISO, GPIO_MODE_INPUT);

    esp_rom_gpio_connect_out_signal(AEL_PIN_NUM_CLK,
                                    spi_periph_signal[FPGA_SPI_HOST].spiclk_out,
                                    false, false);
    esp_rom_gpio_connect_out_signal(AEL_PIN_NUM_MOSI,
                                    spi_periph_signal[FPGA_SPI_HOST].spid_out,
                                    false, false);
    esp_rom_gpio_connect_in_signal(AEL_PIN_NUM_MISO,
                                   spi_periph_signal[FPGA_SPI_HOST].spiq_in,
                                   false);

    /*
     * Say what the pads are actually pointing at.
     *
     * This is the check that found the equivalent fault in the GP-SPI backend:
     * a pad whose output selector still says "plain GPIO" ignores the
     * peripheral completely, and the only symptom is a slave that sees a select
     * and no clock - which is exactly what the fabric reported here.
     */
    /*
     * What the driver actually settled on, not what was asked for.  The fabric
     * synchronises SCK against a 24 MHz clock and needs at least four of its
     * clocks per SCK period; a link running far above the request would alias
     * almost every edge away, which looks exactly like a slave that never
     * counts a byte.
     */
    int actual_hz = 0;
    if (spi_device_get_actual_freq(s_dev, &actual_hz) == ESP_OK) {
        ESP_LOGI(TAG, "link clock: asked %d kHz, got %d kHz%s",
                 FPGA_SPI_HZ / 1000, actual_hz,
                 (actual_hz * 1000 > FPGA_SPI_HZ + FPGA_SPI_HZ / 4)
                     ? "  <- far above the request" : "");
    }

    ESP_LOGI(TAG, "pad routing: clk(GPIO%d)=%u mosi(GPIO%d)=%u  "
                  "(want clk %u, mosi %u; %u means plain GPIO)",
             AEL_PIN_NUM_CLK,
             (unsigned)GPIO.func_out_sel_cfg[AEL_PIN_NUM_CLK].func_sel,
             AEL_PIN_NUM_MOSI,
             (unsigned)GPIO.func_out_sel_cfg[AEL_PIN_NUM_MOSI].func_sel,
             (unsigned)spi_periph_signal[FPGA_SPI_HOST].spiclk_out,
             (unsigned)spi_periph_signal[FPGA_SPI_HOST].spid_out,
             (unsigned)SIG_GPIO_OUT_IDX);

    /*
     * Prove the DAP bitstream is loaded before anything trusts a register.
     * Under the stock image these addresses do not exist, and a read returns
     * whatever the logic-analyser fabric happens to drive - which can look
     * entirely plausible.  Writing a register and reading it back is the
     * cheapest thing only the right image can do.
     */
    const uint8_t probe_values[] = { 0x5A, 0xA5, 0x2C };
    bool answered = true;

    for (size_t i = 0; i < sizeof(probe_values); i++) {
        const esp_err_t werr = reg_write8(REG_TRAIL, probe_values[i]);
        if (werr != ESP_OK) {
            ESP_LOGE(TAG, "register write failed: %s", esp_err_to_name(werr));
            answered = false;
            break;
        }
        const uint8_t got = reg_read8(REG_TRAIL);
        /*
         * Log what came back, not just whether it matched.  The difference
         * between 0x00, 0xFF and a shifted copy of what was written is the
         * difference between nothing driving the line, a floating input and a
         * timing fault - three different bugs that a bare pass/fail hides.
         */
        ESP_LOGI(TAG, "  probe: wrote 0x%02X, read 0x%02X%s",
                 probe_values[i], got, (got == probe_values[i]) ? "" : "  <- differs");
        if (got != probe_values[i]) {
            answered = false;
            break;
        }
    }
    if (!answered) {
        ESP_LOGE(TAG, "no DAP register file answered - the stock bitstream is "
                      "probably loaded, not fpga/dap_master");
        spi_bus_remove_device(s_dev);
        s_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    s_ready = true;
    reg_write8(REG_TRAIL, 1);
    reg_write8(REG_CTRL, CTRL_CLEAR_FIFO);
    ESP_LOGI(TAG, "FPGA DAP master answered on SPI2 at %d kHz", FPGA_SPI_HZ / 1000);
    return ESP_OK;
}

void dap_phy_fpga_invalidate(void)
{
    /*
     * Configuring the FPGA resets everything behind these registers, but the
     * host's own idea of the link survives it - so init() returns early, the
     * probe that proves the DAP image is loaded never runs, and the divider,
     * trail and maxwait the host believes it set are back at their defaults.
     * That presented as sync returning zero on a route that had just worked,
     * with nothing in the log pointing at the reload.
     */
    s_in_use = false;
    s_ready  = false;
    if (s_dev != NULL) {
        spi_bus_remove_device(s_dev);
        s_dev = NULL;
    }
}

bool dap_phy_fpga_ready(void)   { return s_ready; }
bool dap_phy_fpga_in_use(void)  { return s_in_use; }

void dap_phy_fpga_use(bool enable)
{
    s_in_use = enable && s_ready;
}

esp_err_t dap_phy_fpga_set_div(uint8_t div)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    /*
     * Zero is a real setting: a one-clock half period, so DAP0 runs at half
     * the fabric clock and a bit costs two clocks.  24 MHz on a 48 MHz fabric.
     *
     * It took both ends of the bit to get there.  The transmitter used to
     * change DAP1 a clock *after* the falling edge, which leaves (half period
     * - 1) clocks of setup - fine at a divider of 1 and nothing at all at 0 -
     * so the data moved as the target latched it.  And the receiver sees DAP1
     * two clocks late through its synchroniser, so at a two-clock bit there is
     * no instant inside the bit that old; it samples the middle of the
     * previous bit instead and the whole reply simply arrives one bit later,
     * which costs nothing because the start-bit hunt shifts with it.
     */
    return reg_write8(REG_DIV, div);
}

esp_err_t dap_phy_fpga_set_trail(uint8_t clocks)
{
    return s_ready ? reg_write8(REG_TRAIL, clocks) : ESP_ERR_INVALID_STATE;
}

esp_err_t dap_phy_fpga_set_maxwait(uint16_t clocks)
{
    const uint8_t v[2] = { (uint8_t)clocks, (uint8_t)(clocks >> 8) };
    return s_ready ? reg_write(REG_MAXWAIT, v, sizeof(v)) : ESP_ERR_INVALID_STATE;
}

/* FLAGS holds both bits, so it is tracked here rather than read back before
 * every change - a read-modify-write on a register only this file touches. */
static uint8_t s_flags;

esp_err_t dap_phy_fpga_set_trst(bool asserted)
{
    s_flags = (uint8_t)((s_flags & ~FLAG_TRST) | (asserted ? FLAG_TRST : 0u));
    return s_ready ? reg_write8(REG_FLAGS, s_flags) : ESP_ERR_INVALID_STATE;
}

esp_err_t dap_phy_fpga_set_lead(uint8_t clocks)
{
    return s_ready ? reg_write8(REG_LEAD, clocks) : ESP_ERR_INVALID_STATE;
}

esp_err_t dap_phy_fpga_set_raw_window(bool enable)
{
    s_flags = (uint8_t)((s_flags & ~FLAG_RAW_WINDOW) |
                        (enable ? FLAG_RAW_WINDOW : 0u));
    return s_ready ? reg_write8(REG_FLAGS, s_flags) : ESP_ERR_INVALID_STATE;
}

/*
 * Wide mode, on the probe side only.
 *
 * This changes how the fabric frames and samples; it says nothing to the
 * device.  The device is told by a DAPISC telegram with MODE = 01B, and the
 * order matters - the telegram itself has to go out narrow, because until the
 * device has read it that is the only framing it understands.  So a caller
 * turns this on *after* the telegram is acknowledged, and back off before
 * anything that needs to talk to a device that has been reset.
 */
esp_err_t dap_phy_fpga_set_wide(bool enable)
{
    s_flags = (uint8_t)((s_flags & ~FLAG_WIDE) | (enable ? FLAG_WIDE : 0u));
    return s_ready ? reg_write8(REG_FLAGS, s_flags) : ESP_ERR_INVALID_STATE;
}

/*
 * Sample both lines without driving DAP2.
 *
 * Looking at what DAP2 is doing must not mean driving it.  In two-pin mode
 * that pin belongs to the target's application, which configures it as a
 * push-pull output - so a wide transmission from this end puts two drivers on
 * one net through a 22 ohm series resistor, about 150 mA, and enough frames of
 * that brown the board out.  This receives wide and transmits narrow, which is
 * safe against a target that has not handed the pin over yet.
 */
esp_err_t dap_phy_fpga_set_rx_wide(bool enable)
{
    s_flags = (uint8_t)((s_flags & ~FLAG_RX_WIDE) |
                        (enable ? FLAG_RX_WIDE : 0u));
    return s_ready ? reg_write8(REG_FLAGS, s_flags) : ESP_ERR_INVALID_STATE;
}

bool dap_phy_fpga_is_wide(void)
{
    return (s_flags & FLAG_WIDE) != 0u;
}

/*
 * Where in the two-clock-deep window each line is sampled.
 *
 * Tap 0 is what narrow mode has always used.  The silicon does not promise
 * DAP1 and DAP2 leave the pads together, and at the fastest divider a bit is
 * two fabric clocks, so one clock of skew between them is half a bit - which
 * is why this exists at all and why it is per line.
 */
esp_err_t dap_phy_fpga_set_skew(uint8_t dap1_tap, uint8_t dap2_tap)
{
    if (dap1_tap > 3 || dap2_tap > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint8_t v = (uint8_t)((dap1_tap & 3u) | ((dap2_tap & 3u) << 2));
    return s_ready ? reg_write8(REG_SKEW, v) : ESP_ERR_INVALID_STATE;
}

/*
 * Did the last reply's start bit arrive on DAP2 at the same sample as on DAP1?
 *
 * The start bit is the one bit the device drives on both lines together, so
 * this is the only direct evidence the two taps are set consistently.  It is
 * read from LEVEL's high byte rather than STATUS because STATUS had no bit
 * left and that byte had four.
 */
bool dap_phy_fpga_last_aligned(void)
{
    if (!s_ready) {
        return false;
    }
    return (reg_read8(REG_LINES) & LINES_ALIGNED) != 0u;
}

/*
 * Send a frame the caller assembled, bit for bit.
 *
 * `bits` is the whole thing - start bit, CMD, LEN, DATA, CRC6, trailing zero -
 * with the first bit on the wire in bit 0, and `nbits` of them.  In wide mode
 * consecutive pairs share a clock, bit 0 on DAP1 and bit 1 on DAP2, so a
 * caller that wants the start bit on both lines puts a one in each of the
 * first two positions.
 *
 * The fabric assembles nothing and checks nothing here: no CRC is generated
 * and none is expected back unless the caller asks for one.  That is the
 * point - the wide framing rule is not documented anywhere this project can
 * reach, so it has to be found by trying candidates, and a candidate that the
 * fabric would have "corrected" on the way out tests nothing.
 */
esp_err_t dap_phy_fpga_raw_frame(uint64_t bits, size_t nbits,
                                 size_t reply_bits,
                                 uint32_t *reply, uint16_t *wait_cycles)
{
    if (!s_ready || nbits == 0 || nbits > 63) {
        return ESP_ERR_INVALID_ARG;
    }

    s_flags |= FLAG_RAW_FRAME;
    const esp_err_t ferr = reg_write8(REG_FLAGS, s_flags);

    /* CMD and LEN are ignored in this mode; DBITS is the frame length. */
    /* CMD and LEN go out as zero and are ignored; DBITS carries the length. */
    const esp_err_t err = ferr == ESP_OK
        ? dap_phy_fpga_exchange(0, 0, bits, nbits, reply_bits,
                                reply, wait_cycles)
        : ferr;

    s_flags &= (uint8_t)~FLAG_RAW_FRAME;
    reg_write8(REG_FLAGS, s_flags);
    return err;
}

/*
 * The level on every BANK0 pad this design does not otherwise use.
 *
 * Bit n is the pad named scan[n] in the PCF.  Only useful for chasing where a
 * target signal actually lands: with the target toggling one of its pins, the
 * bit that follows names the pad it reaches, and no bit following says no free
 * pad reaches it.
 */
uint16_t dap_phy_fpga_pad_scan(void)
{
    uint8_t raw[2] = {0};

    if (!s_ready) {
        return 0;
    }
    reg_read(REG_SCAN, raw, sizeof(raw));
    return (uint16_t)raw[0] | (uint16_t)((uint16_t)raw[1] << 8);
}

uint8_t dap_phy_fpga_line_witness(void)
{
    return s_ready ? reg_read8(REG_LINES) : 0u;
}



/* ------------------------------------------------------------------------ */
/* Exchanges                                                                 */
/* ------------------------------------------------------------------------ */

/* Poll STATUS until the sequencer reports done, or give up. */
/*
 * How long to spin before yielding, in microseconds.
 *
 * Every frame that is going to work finishes far inside this - a 1 kB block is
 * about 2 ms at the slowest divider and a single frame is microseconds - so the
 * fast path never gets here and never pays for a context switch.
 */
#define SPIN_BEFORE_YIELD_US  3000

static esp_err_t wait_done(uint8_t *status_out)
{
    const int64_t start    = esp_timer_get_time();
    const int64_t deadline = start + OP_TIMEOUT_MS * 1000;

    for (;;) {
        const uint8_t st = reg_read8(REG_STATUS);
        if (st & ST_DONE) {
            *status_out = st;
            return ESP_OK;
        }
        const int64_t now = esp_timer_get_time();
        if (now >= deadline) {
            *status_out = st;
            ESP_LOGE(TAG, "the fabric never reported done (status 0x%02X)", st);
            return ESP_ERR_TIMEOUT;
        }

        /*
         * Yield once the frame is clearly not coming back.
         *
         * This loop used to spin flat out for the whole timeout, which is
         * invisible while frames are answered and catastrophic when they are
         * not: a sweep that sends eighty frames to a target that has stopped
         * listening holds the CPU for twenty seconds without yielding, and the
         * web server, WiFi and console all starve behind it.  That is how a
         * wide-mode experiment turned into a board that had to be power-cycled
         * - the firmware was fine, it simply never got scheduled.
         */
        if (now - start > SPIN_BEFORE_YIELD_US) {
            vTaskDelay(1);
        }
    }
}

static esp_err_t load_frame(uint8_t cmd, uint8_t len_field,
                            uint64_t data, size_t data_bits, size_t reply_bits)
{
    uint8_t payload[8];

    for (size_t i = 0; i < sizeof(payload); i++) {
        payload[i] = (uint8_t)(data >> (8 * i));
    }
    if (reg_write(REG_DATA, payload, sizeof(payload)) != ESP_OK) {
        return ESP_FAIL;
    }
    /*
     * CMD, LEN, DBITS and RBITS are consecutive, so one auto-incrementing
     * burst sets all four - which is the same property that makes the block
     * read cheap, used here to keep an ordinary exchange to three transactions.
     */
    const uint8_t regs[4] = {
        (uint8_t)(cmd & 0x1F),
        (uint8_t)(len_field & 0x3F),
        (uint8_t)(data_bits & 0x3F),
        (uint8_t)(reply_bits & 0x7F),
    };
    return reg_write(REG_CMD, regs, sizeof(regs));
}

esp_err_t dap_phy_fpga_exchange(uint8_t cmd, uint8_t len_field,
                                uint64_t data, size_t data_bits,
                                size_t reply_bits,
                                uint32_t *reply, uint16_t *wait_cycles)
{
    uint8_t status = 0;

    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (load_frame(cmd, len_field, data, data_bits, reply_bits) != ESP_OK) {
        return ESP_FAIL;
    }
    if (reg_write8(REG_CTRL, CTRL_START_FRAME) != ESP_OK) {
        return ESP_FAIL;
    }

    const esp_err_t err = wait_done(&status);
    if (err != ESP_OK) {
        return err;
    }

    if (status & (ST_TIMED_OUT | ST_IDLE_HIGH)) {
        /*
         * Read the command registers back before blaming the target.  The
         * frame the fabric sent is built from these, not from anything the
         * host keeps, so a byte that failed to land produces a perfectly
         * formed frame that says the wrong thing - and the only symptom is
         * silence, which looks identical to a target that is not listening.
         */
        uint8_t regs[4] = {0};
        uint8_t payload[8] = {0};
        reg_read(REG_CMD, regs, sizeof(regs));
        reg_read(REG_DATA, payload, sizeof(payload));
        /*
         * Debug level, not warning.  A frame drawing no reply is an ordinary
         * outcome once this path is the default - an address that bus-errors
         * answers exactly this way - and at warning level it buried the boot
         * log.  It is the instrument that found the attach fault, so it stays,
         * behind a log level rather than deleted.
         */
        ESP_LOGD(TAG, "  no reply; fabric holds cmd=0x%02X len=%u dbits=%u "
                      "rbits=%u data=%02X%02X%02X%02X%02X%02X%02X%02X",
                 regs[0], regs[1], regs[2], regs[3],
                 payload[7], payload[6], payload[5], payload[4],
                 payload[3], payload[2], payload[1], payload[0]);
        ESP_LOGD(TAG, "  host asked for  cmd=0x%02X len=%u dbits=%u rbits=%u "
                      "data=0x%08X%08X", cmd, len_field, (unsigned)data_bits,
                 (unsigned)reply_bits, (unsigned)(data >> 32), (unsigned)data);
    }

    if (reply != NULL) {
        uint8_t raw[4] = {0};
        reg_read(REG_REPLY, raw, sizeof(raw));
        *reply = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
                 ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
    }
    if (wait_cycles != NULL) {
        uint8_t raw[2] = {0};
        reg_read(REG_WAIT, raw, sizeof(raw));
        *wait_cycles = (uint16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
    }

    if (status & ST_TIMED_OUT) {
        return ESP_ERR_TIMEOUT;
    }
    if (status & ST_IDLE_HIGH) {
        /* Nobody was driving the wire.  Distinct from a timeout, and worth
         * keeping distinct: it means the target is absent or unpowered rather
         * than slow. */
        return ESP_ERR_NOT_FOUND;
    }
    if (!(status & ST_CRC_OK)) {
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
}

esp_err_t dap_phy_fpga_blockread(uint64_t cmd_payload, size_t payload_bits,
                                 uint32_t *words, size_t count)
{
    uint8_t status = 0;

    if (!s_ready || words == NULL || count == 0 || count > 256) {
        return ESP_ERR_INVALID_ARG;
    }

    reg_write8(REG_CTRL, CTRL_CLEAR_FIFO);
    reg_write8(REG_PARCELS, (uint8_t)(count - 1));

    /* DAP_CMD_CLIENT_BLOCKREAD is 0x0A with a 40-bit payload; the caller
     * supplies the payload it has already assembled. */
    if (load_frame(0x0Au, (uint8_t)payload_bits, cmd_payload, payload_bits, 0) != ESP_OK) {
        return ESP_FAIL;
    }
    if (reg_write8(REG_CTRL, CTRL_START_BLOCK) != ESP_OK) {
        return ESP_FAIL;
    }

    const size_t bytes = count * 4;
    if (bytes > sizeof(s_buf)) {
        return ESP_ERR_INVALID_SIZE;
    }

    /*
     * Drain while the block is still arriving, rather than after it.
     *
     * The two halves cost about the same - a 1 kB block is ~2.1 ms on the wire
     * at 4 MHz and ~1.6 ms to shift out over a 5 MHz link - so running them one
     * after the other spends nearly twice as long as the wire needs.  The FIFO
     * holds a whole block, so this is not required for correctness; it is
     * required for the fabric to be faster than the CPU path it replaces.
     *
     * STATUS and LEVEL are next to each other in the map, so one burst read of
     * the low registers answers both "how much is waiting" and "has it
     * finished" - which matters, because the loop has to be able to tell a
     * block that is merely slow from one that stopped early.
     */
    size_t  got = 0;
    uint8_t low[REG_LEVEL + 2];
    const int64_t deadline = esp_timer_get_time() + OP_TIMEOUT_MS * 1000;

    for (;;) {
        if (reg_read(0x00, low, sizeof(low)) != ESP_OK) {
            return ESP_FAIL;
        }
        status = low[REG_STATUS];

        size_t avail = (size_t)low[REG_LEVEL] | ((size_t)low[REG_LEVEL + 1] << 8);
        if (avail > bytes - got) {
            avail = bytes - got;
        }

        /*
         * Wait for a worthwhile chunk while the block is still running: each
         * drain costs a transaction of its own, and shifting out four bytes at
         * a time would spend more on overhead than the overlap saves.  Once
         * the sequencer is done there is nothing left to wait for.
         */
        if (avail && (avail >= DRAIN_CHUNK || (status & ST_DONE))) {
            if (reg_read(REG_FIFO, s_buf + got, avail) != ESP_OK) {
                return ESP_FAIL;
            }
            got += avail;
            if (got == bytes) {
                break;
            }
            continue;
        }

        if (status & ST_TIMED_OUT) {
            ESP_LOGW(TAG, "a parcel timed out; the block stopped after %u of "
                          "%u bytes", (unsigned)got, (unsigned)bytes);
            return ESP_ERR_TIMEOUT;
        }
        if (status & ST_OVERRUN) {
            /* The FIFO filled because nothing drained it.  Reported rather
             * than hidden: a short block the caller knows about beats a full
             * one with a hole in the middle that looks like data. */
            ESP_LOGE(TAG, "the reply FIFO overran");
            return ESP_ERR_NO_MEM;
        }
        if ((status & ST_DONE) && avail == 0) {
            ESP_LOGW(TAG, "the block ended with %u of %u bytes",
                     (unsigned)got, (unsigned)bytes);
            return ESP_ERR_INVALID_SIZE;
        }
        if (esp_timer_get_time() >= deadline) {
            ESP_LOGE(TAG, "the block never finished (status 0x%02X, %u of %u)",
                     status, (unsigned)got, (unsigned)bytes);
            return ESP_ERR_TIMEOUT;
        }
    }
    for (size_t i = 0; i < count; i++) {
        words[i] = (uint32_t)s_buf[4 * i] |
                   ((uint32_t)s_buf[4 * i + 1] << 8) |
                   ((uint32_t)s_buf[4 * i + 2] << 16) |
                   ((uint32_t)s_buf[4 * i + 3] << 24);
    }
    return ESP_OK;
}

/*
 * One client_blockwrite: the command frame, then `count` words streamed from
 * the fabric's write FIFO.
 *
 * Capped at DAP_FPGA_BLOCK_WORDS because the FIFO holds that much: the whole
 * block goes in before the transfer starts, so the host is not in the loop at
 * all once it does.  The fabric stalls rather than fails if the FIFO runs dry,
 * so a larger block would work too - it would just put the host back in the
 * loop, which is the thing worth avoiding.
 *
 * The address is word-aligned and travels shifted right by two, which is what
 * the 30-bit address form in the telegram carries.
 */


esp_err_t dap_phy_fpga_block_write(uint32_t address, const uint32_t *words,
                                   size_t count)
{
    uint8_t status = 0;

    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (count == 0 || count > DAP_FPGA_BLOCK_WORDS || words == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (address & 3u) {
        return ESP_ERR_INVALID_ARG;
    }

    if (reg_write8(REG_CTRL, CTRL_CLEAR_WFIFO) != ESP_OK) {
        return ESP_FAIL;
    }

    /*
     * The words first, as one burst.  0x48 does not auto-increment - it is a
     * port, like the reply FIFO's 0x40 - so a burst of any length lands in the
     * FIFO in order.
     */
    for (size_t i = 0; i < count; i += DAP_FPGA_BURST_WORDS) {
        const size_t n = (count - i > DAP_FPGA_BURST_WORDS)
                             ? DAP_FPGA_BURST_WORDS : (count - i);
        uint8_t *out = s_buf;

        for (size_t w = 0; w < n; w++) {
            const uint32_t value = words[i + w];
            *out++ = (uint8_t)(value);
            *out++ = (uint8_t)(value >> 8);
            *out++ = (uint8_t)(value >> 16);
            *out++ = (uint8_t)(value >> 24);
        }
        if (reg_write(REG_WFIFO, s_buf, n * 4) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    /*
     * bit 0  CRCdown, bit 1 per-parcel CRC6, bits 9:2 the word count with 0
     * meaning 256, bits 39:10 the word address.  Both checks are off: the
     * loader verifies the whole image with an on-target CRC32 afterwards,
     * which costs one round trip for the lot rather than six bits per word.
     */
    const uint64_t payload = ((uint64_t)(address >> 2) << 10) |
                             ((uint64_t)(count & 0xFFu) << 2);

    if (load_frame(0x09u, 40, payload, 40, 0) != ESP_OK) {
        return ESP_FAIL;
    }
    if (reg_write8(REG_PARCELS, (uint8_t)(count - 1)) != ESP_OK) {
        return ESP_FAIL;
    }
    if (reg_write8(REG_CTRL, CTRL_START_WRITE) != ESP_OK) {
        return ESP_FAIL;
    }

    const esp_err_t err = wait_done(&status);

    if (err != ESP_OK) {
        return err;
    }
    if (status & ST_TIMED_OUT) {
        ESP_LOGW(TAG, "block write to 0x%08" PRIX32 " was not acknowledged",
                 address);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void dap_phy_fpga_log_status(void)
{
    if (!s_ready) {
        ESP_LOGW(TAG, "not initialised");
        return;
    }
    const uint8_t st = reg_read8(REG_STATUS);

    ESP_LOGI(TAG, "status 0x%02X:%s%s%s%s%s%s%s%s", st,
             (st & ST_BUSY)       ? " busy"       : "",
             (st & ST_DONE)       ? " done"       : "",
             (st & ST_TIMED_OUT)  ? " timed_out"  : "",
             (st & ST_IDLE_HIGH)  ? " idle_high"  : "",
             (st & ST_CRC_OK)     ? " crc_ok"     : "",
             (st & ST_FIFO_EMPTY) ? " fifo_empty" : "",
             (st & ST_FIFO_FULL)  ? " fifo_full"  : "",
             (st & ST_OVERRUN)    ? " OVERRUN"    : "");
}
