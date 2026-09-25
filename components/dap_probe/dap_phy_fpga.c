#include "dap_phy_fpga.h"
#include "dap_lock.h"

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

/* FPGA link: AEL_PIN_NUM_CLK/MOSI/MISO, CS0 driven by hand. */
#define FPGA_SPI_HOST   SPI2_HOST

/* Must be an integer division of the 80 MHz APB clock. */
#define FPGA_SPI_HZ     (40000 * 1000)

/* Minimum bytes per FIFO drain while a block is still arriving (half a block). */
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

/* Wide-mode start-bit alignment. */
#define REG_LINES       0x27
#define LINES_ALIGNED    (1u << 0)

#define WRITE_BIT        0x80

/* Upper bound for any command; a 256-parcel block takes a few ms. */
#define OP_TIMEOUT_MS    250

static spi_device_handle_t s_dev;
static bool                s_ready;
static bool                s_in_use;

/* DMA wants these word-aligned and in internal RAM. */
static WORD_ALIGNED_ATTR uint8_t s_buf[1024];

/* ------------------------------------------------------------------------ */
/* Register access                                                           */
/* ------------------------------------------------------------------------ */

/* CS is held for the whole transaction; the first byte after it falls is the header. */
static inline void cs_low(void)  { gpio_set_level(AEL_PIN_NUM_CS0, 0); }
static inline void cs_high(void) { gpio_set_level(AEL_PIN_NUM_CS0, 1); }

/* Header and payload in one full-duplex transfer. */
static WORD_ALIGNED_ATTR uint8_t s_tx[1088];
static WORD_ALIGNED_ATTR uint8_t s_rx[1088];

static esp_err_t xfer(uint8_t header, const uint8_t *tx, uint8_t *rx, size_t len)
{
    /* Write: header, data.  Read: header, dummy byte, data. */
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

    /* CS is driven by hand: the driver's CS does not reach the FPGA. */
    const spi_device_interface_config_t dev = {
        .clock_speed_hz = FPGA_SPI_HZ,
        .mode           = 0,                      /* what the fabric expects */
        .spics_io_num   = -1,
        .queue_size     = 2,
        .flags          = 0,
    };

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

    /* Reclaim the pads for SPI; FPGA configuration bit-bangs them. */
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

    /* Prove the DAP bitstream is loaded: TRAIL must read back what was written. */
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
    /* FPGA reconfiguration resets every register. */
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

/* Shadow of REG_FLAGS (write-only from here). */
static uint8_t s_flags;

esp_err_t dap_phy_fpga_set_trst(bool asserted)
{
    s_flags = (uint8_t)((s_flags & ~FLAG_TRST) | (asserted ? FLAG_TRST : 0u));
    return s_ready ? reg_write8(REG_FLAGS, s_flags) : ESP_ERR_INVALID_STATE;
}

esp_err_t dap_phy_fpga_set_raw_window(bool enable)
{
    s_flags = (uint8_t)((s_flags & ~FLAG_RAW_WINDOW) |
                        (enable ? FLAG_RAW_WINDOW : 0u));
    return s_ready ? reg_write8(REG_FLAGS, s_flags) : ESP_ERR_INVALID_STATE;
}

esp_err_t dap_phy_fpga_set_wide(bool enable)
{
    s_flags = (uint8_t)((s_flags & ~FLAG_WIDE) | (enable ? FLAG_WIDE : 0u));
    return s_ready ? reg_write8(REG_FLAGS, s_flags) : ESP_ERR_INVALID_STATE;
}

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

esp_err_t dap_phy_fpga_set_skew(uint8_t dap1_tap, uint8_t dap2_tap)
{
    if (dap1_tap > 3 || dap2_tap > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint8_t v = (uint8_t)((dap1_tap & 3u) | ((dap2_tap & 3u) << 2));
    return s_ready ? reg_write8(REG_SKEW, v) : ESP_ERR_INVALID_STATE;
}

/* ------------------------------------------------------------------------ */
/* Exchanges                                                                 */
/* ------------------------------------------------------------------------ */

/* Busy-poll this long before yielding; normal frames finish well inside it. */
#define SPIN_BEFORE_YIELD_US  3000

/* Poll STATUS until the sequencer reports done, or give up. */
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

        /* Yield once the frame is clearly slow. */
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
    /* CMD, LEN, DBITS, RBITS are consecutive: one auto-incrementing burst. */
    const uint8_t regs[4] = {
        (uint8_t)(cmd & 0x1F),
        (uint8_t)(len_field & 0x3F),
        (uint8_t)(data_bits & 0x3F),
        (uint8_t)(reply_bits & 0x7F),
    };
    return reg_write(REG_CMD, regs, sizeof(regs));
}

static esp_err_t dap_phy_fpga_exchange_locked(uint8_t cmd, uint8_t len_field,
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
        /* Log what the fabric holds versus what was asked for. */
        uint8_t regs[4] = {0};
        uint8_t payload[8] = {0};
        reg_read(REG_CMD, regs, sizeof(regs));
        reg_read(REG_DATA, payload, sizeof(payload));

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
        /* Nothing drove the wire: target absent or unpowered. */
        return ESP_ERR_NOT_FOUND;
    }
    if (!(status & ST_CRC_OK)) {
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
}

static esp_err_t dap_phy_fpga_blockread_locked(uint64_t cmd_payload, size_t payload_bits,
                                               uint32_t *words, size_t count)
{
    uint8_t status = 0;

    if (!s_ready || words == NULL || count == 0 || count > 256) {
        return ESP_ERR_INVALID_ARG;
    }

    reg_write8(REG_CTRL, CTRL_CLEAR_FIFO);
    reg_write8(REG_PARCELS, (uint8_t)(count - 1));

    /* DAP_CMD_CLIENT_BLOCKREAD (0x0A); payload assembled by the caller. */
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

    /* Drain while the block is still arriving; one read fetches STATUS..LEVEL. */
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

        /* Take a full chunk while running, or the remainder once done. */
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
            /* The FIFO filled because nothing drained it. */
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

/* client_blockwrite of up to DAP_FPGA_BLOCK_WORDS words from the write FIFO. */
static esp_err_t dap_phy_fpga_block_write_locked(uint32_t address, const uint32_t *words,
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

    /* Fill the write FIFO first. */
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
     * Payload: bit 0 CRCdown, bit 1 per-parcel CRC6 (both off), bits 9:2 word
     * count (0 = 256), bits 39:10 word address.
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

/* -- locked entry points (dap_lock.h) -------------------------------------- */

esp_err_t dap_phy_fpga_exchange(uint8_t cmd, uint8_t len_field, uint64_t data, size_t data_bits, size_t reply_bits, uint32_t *reply, uint16_t *wait_cycles)
{
    dap_lock();
    const esp_err_t err = dap_phy_fpga_exchange_locked(cmd, len_field, data, data_bits, reply_bits, reply, wait_cycles);
    dap_unlock();
    return err;
}

esp_err_t dap_phy_fpga_blockread(uint64_t cmd_payload, size_t payload_bits, uint32_t *words, size_t count)
{
    dap_lock();
    const esp_err_t err = dap_phy_fpga_blockread_locked(cmd_payload, payload_bits, words, count);
    dap_unlock();
    return err;
}

esp_err_t dap_phy_fpga_block_write(uint32_t address, const uint32_t *words, size_t count)
{
    dap_lock();
    const esp_err_t err = dap_phy_fpga_block_write_locked(address, words, count);
    dap_unlock();
    return err;
}
