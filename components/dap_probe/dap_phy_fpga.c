#include "dap_phy_fpga.h"

#include <inttypes.h>
#include <string.h>

#include "board_profile.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DAP_FPGA";

/*
 * The FPGA sits on the bus the application already set up for the fabric -
 * AEL_PIN_NUM_CLK/MOSI/MISO with CS0 - and that same bus is what loads the
 * bitstream.  So this adds a device rather than taking the bus over: nothing
 * else needs to be torn down, and configuration still works afterwards.
 */
#define FPGA_SPI_HOST   SPI2_HOST

/*
 * 6 MHz.  The fabric runs at 24 MHz and its SPI slave synchronises an
 * asynchronous SCK, so it needs sysclk >= 4x SCK.  Past that it does not
 * degrade gracefully, it drops bits, so this is a ceiling rather than a
 * starting point for tuning.
 */
#define FPGA_SPI_HZ     (6 * 1000 * 1000)

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
#define REG_DATA        0x10
#define REG_REPLY       0x20
#define REG_RCRC        0x24
#define REG_WAIT        0x25
#define REG_FIFO        0x40

#define CTRL_START_FRAME (1u << 0)
#define CTRL_START_BLOCK (1u << 1)
#define CTRL_ABORT       (1u << 2)
#define CTRL_CLEAR_FIFO  (1u << 3)

#define ST_BUSY          (1u << 0)
#define ST_DONE          (1u << 1)
#define ST_TIMED_OUT     (1u << 2)
#define ST_IDLE_HIGH     (1u << 3)
#define ST_CRC_OK        (1u << 4)
#define ST_FIFO_EMPTY    (1u << 5)
#define ST_FIFO_FULL     (1u << 6)
#define ST_OVERRUN       (1u << 7)

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
static esp_err_t reg_write(uint8_t addr, const uint8_t *data, size_t len)
{
    spi_transaction_ext_t t = {
        .base = {
            .flags     = SPI_TRANS_VARIABLE_CMD,
            .cmd       = (uint16_t)(addr | WRITE_BIT),
            .length    = len * 8,
            .tx_buffer = data,
        },
        .command_bits = 8,
    };
    return spi_device_polling_transmit(s_dev, &t.base);
}

static esp_err_t reg_read(uint8_t addr, uint8_t *data, size_t len)
{
    spi_transaction_ext_t t = {
        .base = {
            .flags     = SPI_TRANS_VARIABLE_CMD,
            .cmd       = addr,
            .rxlength  = len * 8,
            .rx_buffer = data,
        },
        .command_bits = 8,
    };
    return spi_device_polling_transmit(s_dev, &t.base);
}

static esp_err_t reg_write8(uint8_t addr, uint8_t value)
{
    return reg_write(addr, &value, 1);
}

static uint8_t reg_read8(uint8_t addr)
{
    uint8_t v = 0;
    if (reg_read(addr, &v, 1) != ESP_OK) {
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

    const spi_device_interface_config_t dev = {
        .clock_speed_hz = FPGA_SPI_HZ,
        .mode           = 0,                      /* what the fabric expects */
        .spics_io_num   = AEL_PIN_NUM_CS0,
        .queue_size     = 2,
        .command_bits   = 8,
        .flags          = SPI_DEVICE_HALFDUPLEX,
    };

    esp_err_t err = spi_bus_add_device(FPGA_SPI_HOST, &dev, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s - is the fabric bus up?",
                 esp_err_to_name(err));
        return err;
    }

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
        if (reg_write8(REG_TRAIL, probe_values[i]) != ESP_OK) {
            answered = false;
            break;
        }
        if (reg_read8(REG_TRAIL) != probe_values[i]) {
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
    if (div < 2) {
        /*
         * The fabric samples the target through a two-flop synchroniser, so a
         * half period of one or two clocks puts the sample in the next bit.
         * Refusing beats producing a probe that reads plausible rubbish.
         */
        ESP_LOGW(TAG, "divider %u is below the synchroniser's floor; using 2", div);
        div = 2;
    }
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

esp_err_t dap_phy_fpga_set_trst(bool asserted)
{
    return s_ready ? reg_write8(REG_FLAGS, asserted ? 1u : 0u)
                   : ESP_ERR_INVALID_STATE;
}

/* ------------------------------------------------------------------------ */
/* Exchanges                                                                 */
/* ------------------------------------------------------------------------ */

/* Poll STATUS until the sequencer reports done, or give up. */
static esp_err_t wait_done(uint8_t *status_out)
{
    const int64_t deadline = esp_timer_get_time() + OP_TIMEOUT_MS * 1000;

    for (;;) {
        const uint8_t st = reg_read8(REG_STATUS);
        if (st & ST_DONE) {
            *status_out = st;
            return ESP_OK;
        }
        if (esp_timer_get_time() >= deadline) {
            *status_out = st;
            ESP_LOGE(TAG, "the fabric never reported done (status 0x%02X)", st);
            return ESP_ERR_TIMEOUT;
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

    const esp_err_t err = wait_done(&status);
    if (err != ESP_OK) {
        return err;
    }
    if (status & ST_TIMED_OUT) {
        ESP_LOGW(TAG, "a parcel timed out; the block stopped there");
        return ESP_ERR_TIMEOUT;
    }
    if (status & ST_OVERRUN) {
        /* The FIFO filled because nothing drained it.  Reported rather than
         * hidden: a short block the caller knows about beats a full one with a
         * hole in the middle that looks like data. */
        ESP_LOGE(TAG, "the reply FIFO overran");
        return ESP_ERR_NO_MEM;
    }

    /*
     * One burst for the whole block.  The FIFO address does not auto-increment,
     * so this drains it rather than walking off the end of the register map -
     * and this is the transaction that replaces 256 round trips.
     */
    const size_t bytes = count * 4;
    if (bytes > sizeof(s_buf)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (reg_read(REG_FIFO, s_buf, bytes) != ESP_OK) {
        return ESP_FAIL;
    }
    for (size_t i = 0; i < count; i++) {
        words[i] = (uint32_t)s_buf[4 * i] |
                   ((uint32_t)s_buf[4 * i + 1] << 8) |
                   ((uint32_t)s_buf[4 * i + 2] << 16) |
                   ((uint32_t)s_buf[4 * i + 3] << 24);
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
