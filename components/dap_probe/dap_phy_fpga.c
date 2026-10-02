#include "dap_phy_fpga.h"
#include "dap_frame.h"
#include "dap_lock.h"

#include <inttypes.h>
#include <string.h>

#include "board_profile.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_private/gdma.h"
#include "esp_private/spi_common_internal.h"
#include "hal/gdma_ll.h"
#include "hal/spi_ll.h"
#include "soc/gdma_struct.h"
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

/* FIFO drain while blocks are still arriving: at least this many bytes per read
 * (each read has a fixed cost), and at most what one transfer buffer holds. */
#define DRAIN_CHUNK     64
#define DRAIN_MAX       1024

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
#define REG_LEVEL       0x0D    /* 12-bit, bytes waiting in the reply FIFO */
#define LEVEL_HI_QUEUED  (1u << 4)  /* in LEVEL's high byte: a block start is held */
#define REG_SKEW        0x0F    /* capture tap: [1:0] DAP1, [3:2] DAP2;
                                 * [5:4] falling-edge sample, [7:6] fast LAG */
#define SKEW_TAPS        0x0Fu

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
#define FLAG_FAST        (1u << 6)

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
#if AEL_PIN_NUM_CS0 < 32
static inline void cs_low(void)  { GPIO.out_w1tc = 1u << AEL_PIN_NUM_CS0; }
static inline void cs_high(void) { GPIO.out_w1ts = 1u << AEL_PIN_NUM_CS0; }
#else
static inline void cs_low(void)  { GPIO.out1_w1tc.val = 1u << (AEL_PIN_NUM_CS0 - 32); }
static inline void cs_high(void) { GPIO.out1_w1ts.val = 1u << (AEL_PIN_NUM_CS0 - 32); }
#endif

/* Header and payload in one full-duplex transfer (padded for word copies). */
static WORD_ALIGNED_ATTR uint8_t s_tx[1088 + 64];
static WORD_ALIGNED_ATTR uint8_t s_rx[1088 + 64];

/*
 * Register-level transfers.  The driver costs ~20 us per transaction (DMA
 * descriptors, cache sync); the peripheral's 64-byte CPU buffer needs none of
 * that.  CS is a GPIO, so one logical transfer may span several hardware
 * transactions.  The driver configures clock, mode and input timing once
 * (ll_prepare runs after its first transaction); nothing else uses SPI2.
 */
#define LL_CHUNK 64
static spi_dev_t *s_hw;

static void ll_prepare(void)
{
    s_hw = SPI_LL_GET_HW(FPGA_SPI_HOST);
    spi_ll_dma_tx_enable(s_hw, false);
    spi_ll_dma_rx_enable(s_hw, false);
    spi_ll_set_half_duplex(s_hw, false);
    spi_ll_set_command_bitlen(s_hw, 0);
    spi_ll_set_addr_bitlen(s_hw, 0);
    spi_ll_set_dummy(s_hw, 0);
    spi_ll_enable_mosi(s_hw, 1);
    spi_ll_enable_miso(s_hw, 1);
}

static IRAM_ATTR void ll_chunk(const uint8_t *tx, uint8_t *rx, size_t n)
{
    const size_t bits = n * 8;

    spi_ll_write_buffer(s_hw, tx, bits);
    spi_ll_set_mosi_bitlen(s_hw, bits);
    spi_ll_set_miso_bitlen(s_hw, bits);
    spi_ll_clear_int_stat(s_hw);
    spi_ll_apply_config(s_hw);
    spi_ll_user_start(s_hw);
    while (!spi_ll_usr_is_done(s_hw)) {
    }
    if (rx != NULL) {
        spi_ll_read_buffer(s_hw, rx, bits);
    }
}

/*
 * Longer transfers go by DMA on the channels the driver allocated for the bus
 * (it is bypassed, so they are idle).  Filling and emptying the 64-byte CPU
 * buffer costs about as much as clocking it at 40 MHz and cannot overlap the
 * wire; DMA streams one whole transfer at line rate.  Buffers are internal
 * SRAM, which the S3 does not cache, so no cache maintenance is needed.
 */
#define DMA_MIN_BYTES 96
static size_t s_dma_min = DMA_MIN_BYTES;
/* The GDMA channels, driven through the LL layer: the gdma driver's calls sit
 * in flash behind spinlocks and cost microseconds per transfer. */
static bool     s_dma_ok;
static uint32_t s_dma_tx_ch, s_dma_rx_ch;
static DRAM_ATTR spi_dma_desc_t s_desc_tx, s_desc_rx;

static void dma_prepare(void)
{
    const spi_dma_ctx_t *ctx = spi_bus_get_dma_ctx(FPGA_SPI_HOST);
    int group = -1, tx_ch = -1, rx_ch = -1, rx_group = -1;

    s_dma_ok = ctx != NULL && ctx->tx_dma_chan != NULL && ctx->rx_dma_chan != NULL &&
               gdma_get_group_channel_id(ctx->tx_dma_chan, &group, &tx_ch) == ESP_OK &&
               gdma_get_group_channel_id(ctx->rx_dma_chan, &rx_group, &rx_ch) == ESP_OK &&
               group == 0 && rx_group == 0;
    s_dma_tx_ch = (uint32_t)tx_ch;
    s_dma_rx_ch = (uint32_t)rx_ch;
}

static IRAM_ATTR bool ll_dma(const uint8_t *tx, uint8_t *rx, size_t n)
{
    const size_t bits = n * 8;
    volatile spi_dma_desc_t *drx = &s_desc_rx;

    if (rx != NULL) {
        drx->dw0.size    = (n + 3) & ~3u;
        drx->dw0.length  = 0;
        drx->dw0.suc_eof = 0;
        drx->dw0.owner   = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        drx->buffer      = rx;
        drx->next        = NULL;
        gdma_ll_rx_reset_channel(&GDMA, s_dma_rx_ch);
        spi_ll_dma_rx_fifo_reset(s_hw);
        spi_ll_infifo_full_clr(s_hw);
        spi_ll_dma_rx_enable(s_hw, true);
        gdma_ll_rx_set_desc_addr(&GDMA, s_dma_rx_ch, (uint32_t)&s_desc_rx);
        gdma_ll_rx_start(&GDMA, s_dma_rx_ch);
    }
    s_desc_tx.dw0.size    = n;
    s_desc_tx.dw0.length  = n;
    s_desc_tx.dw0.suc_eof = 1;
    s_desc_tx.dw0.owner   = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
    s_desc_tx.buffer      = (void *)tx;
    s_desc_tx.next        = NULL;
    gdma_ll_tx_reset_channel(&GDMA, s_dma_tx_ch);
    spi_ll_dma_tx_fifo_reset(s_hw);
    spi_ll_outfifo_empty_clr(s_hw);
    spi_ll_dma_tx_enable(s_hw, true);
    gdma_ll_tx_set_desc_addr(&GDMA, s_dma_tx_ch, (uint32_t)&s_desc_tx);
    gdma_ll_tx_start(&GDMA, s_dma_tx_ch);

    spi_ll_set_mosi_bitlen(s_hw, bits);
    spi_ll_set_miso_bitlen(s_hw, bits);
    spi_ll_clear_int_stat(s_hw);
    spi_ll_apply_config(s_hw);
    spi_ll_user_start(s_hw);
    while (!spi_ll_usr_is_done(s_hw)) {
    }

    /* The last words can still be on their way from the DMA FIFO. */
    bool ok = true;
    if (rx != NULL) {
        int spins = 10000;
        while (!drx->dw0.suc_eof && --spins > 0) {
        }
        ok = spins > 0;
    }
    spi_ll_dma_tx_enable(s_hw, false);
    spi_ll_dma_rx_enable(s_hw, false);
    return ok;
}

/* Link statistics, for benchmarking (dap_phy_fpga_stats). */
static dap_fpga_stats_t s_stats;
static uint64_t         s_xfer_cycles;      /* CPU cycles inside xfer() */

/* Shadows of the registers the frame burst rewrites (fabric reset values). */
static uint8_t  s_flags;
static uint8_t  s_trail   = 1;
static uint8_t  s_lead    = 2;
static uint8_t  s_skew    = 0;
static uint16_t s_maxwait = 256;
/* Fast mode (FLAG_FAST per frame, added by load_frame_lead). */
static bool     s_fast;

static void shadows_reset(void)
{
    s_flags = 0;
    s_fast = false;
    s_trail = 1;
    s_lead = 2;
    s_skew = 0;
    s_maxwait = 256;
}

static esp_err_t reg_read(uint8_t addr, uint8_t *data, size_t len);

void dap_phy_fpga_link_timing(uint32_t *ns_short, uint32_t *ns_long, int *clock_khz)
{
    static uint8_t scratch[1024];
    int hz = 0;

    dap_lock();
    /* An empty reply FIFO reads as zeros, so the port is safe to time. */
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < 64; i++) {
        reg_read(REG_FIFO, scratch, 7);
    }
    *ns_short = (uint32_t)((esp_timer_get_time() - t0) * 1000 / 64);
    t0 = esp_timer_get_time();
    for (int i = 0; i < 16; i++) {
        reg_read(REG_FIFO, scratch, sizeof(scratch));
    }
    *ns_long = (uint32_t)((esp_timer_get_time() - t0) * 1000 / 16);
    dap_unlock();
    if (s_dev != NULL) {
        spi_device_get_actual_freq(s_dev, &hz);
    }
    *clock_khz = hz;
}

void dap_phy_fpga_stats(dap_fpga_stats_t *out, bool reset)
{
    *out = s_stats;
    out->xfer_us = (uint32_t)(s_xfer_cycles / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    if (reset) {
        memset(&s_stats, 0, sizeof(s_stats));
        s_xfer_cycles = 0;
    }
}

static IRAM_ATTR esp_err_t xfer(uint8_t header, const uint8_t *tx, uint8_t *rx, size_t len)
{
    /* Write: header, data.  Read: header, dummy byte, data. */
    const size_t lead = (rx != NULL) ? 2u : 1u;
    const uint32_t c0 = esp_cpu_get_cycle_count();

    if (len + lead > sizeof(s_tx)) {
        return ESP_ERR_INVALID_SIZE;
    }

    /* A read's MOSI bytes after the header are ignored by the fabric. */
    s_tx[0] = header;
    if (tx != NULL) {
        memcpy(s_tx + lead, tx, len);
    } else if (rx == NULL) {
        memset(s_tx + 1, 0, len);
    }

    esp_err_t err = ESP_OK;
    const size_t total = len + lead;

    cs_low();
    if (s_hw != NULL && s_dma_ok && total >= s_dma_min) {
        if (!ll_dma(s_tx, (rx != NULL) ? s_rx : NULL, total)) {
            err = ESP_ERR_TIMEOUT;
        }
    } else if (s_hw != NULL) {
        for (size_t off = 0; off < total; off += LL_CHUNK) {
            const size_t n = (total - off > LL_CHUNK) ? LL_CHUNK : total - off;
            ll_chunk(s_tx + off, (rx != NULL) ? s_rx + off : NULL, n);
        }
    } else {
        spi_transaction_t t = {
            .length    = total * 8,
            .tx_buffer = s_tx,
            .rx_buffer = s_rx,
        };
        err = spi_device_polling_transmit(s_dev, &t);
    }
    cs_high();

    if (err == ESP_OK && rx != NULL) {
        memcpy(rx, s_rx + lead, len);
    }
    s_stats.xfers++;
    s_stats.xfer_bytes += len + lead;
    s_xfer_cycles += esp_cpu_get_cycle_count() - c0;
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
    /* SPI2 is the FPGA's alone from here on: holding the bus skips the
     * driver's per-transaction arbitration. */
    shadows_reset();
    spi_device_acquire_bus(s_dev, portMAX_DELAY);

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
        spi_device_release_bus(s_dev);
        spi_bus_remove_device(s_dev);
        s_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    s_ready = true;
    ll_prepare();                   /* the driver has configured the peripheral */
    reg_write8(REG_TRAIL, 1);
    reg_write8(REG_CTRL, CTRL_CLEAR_FIFO);
    if (reg_read8(REG_TRAIL) != 1) {
        ESP_LOGE(TAG, "register-level SPI path does not read back; using the driver");
        s_hw = NULL;
    } else {
        /* Check the DMA path too, forced onto a write and read of DATA. */
        dma_prepare();
        static const uint8_t pattern[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
        uint8_t back[8] = {0};
        s_dma_min = 1;
        bool dma_ok = s_dma_ok &&
                      reg_write(REG_DATA, pattern, sizeof(pattern)) == ESP_OK &&
                      reg_read(REG_DATA, back, sizeof(back)) == ESP_OK &&
                      memcmp(back, pattern, sizeof(pattern)) == 0;
        s_dma_min = DMA_MIN_BYTES;
        if (!dma_ok) {
            ESP_LOGE(TAG, "the DMA path does not read back; CPU buffer only");
            s_dma_ok = false;
        }
    }
    ESP_LOGI(TAG, "FPGA DAP master answered on SPI2 at %d kHz", FPGA_SPI_HZ / 1000);
    return ESP_OK;
}

void dap_phy_fpga_invalidate(void)
{
    /* FPGA reconfiguration resets every register. */
    s_in_use = false;
    s_ready  = false;
    s_hw     = NULL;
    s_dma_ok = false;
    shadows_reset();
    if (s_dev != NULL) {
        spi_device_release_bus(s_dev);
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
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    s_trail = clocks;
    return reg_write8(REG_TRAIL, clocks);
}

esp_err_t dap_phy_fpga_set_maxwait(uint16_t clocks)
{
    const uint8_t v[2] = { (uint8_t)clocks, (uint8_t)(clocks >> 8) };
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    s_maxwait = clocks;
    return reg_write(REG_MAXWAIT, v, sizeof(v));
}

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
    const uint8_t v = (uint8_t)((s_skew & ~SKEW_TAPS) |
                                (dap1_tap & 3u) | ((dap2_tap & 3u) << 2));
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    s_skew = v;
    return reg_write8(REG_SKEW, v);
}

esp_err_t dap_phy_fpga_set_fast(bool enable)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    s_fast = enable;
    return ESP_OK;
}

bool dap_phy_fpga_is_fast(void)
{
    return s_fast;
}

esp_err_t dap_phy_fpga_set_fast_timing(uint8_t lag, bool edge1, bool edge2)
{
    if (lag > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    s_skew = (uint8_t)((s_skew & SKEW_TAPS) | (edge1 ? 0x10u : 0u) |
                       (edge2 ? 0x20u : 0u) | (uint8_t)(lag << 6));
    return reg_write8(REG_SKEW, s_skew);
}

/*
 * The whole frame as fast mode sends it (the fabric's raw-frame path; it does
 * not assemble frames a bit per clock).  Narrow: start bit, CMD, LEN, DATA,
 * CRC6, trailing zero.  Wide, as the fabric pairs it (even bits DAP1, odd
 * DAP2): the start bit on both lines, CMD and an odd DATA each padded with a
 * zero, the CRC over the padded fields, a trailing pair.  False when it does
 * not fit the 63-bit DATA register.
 */
static bool raw_frame(uint8_t cmd, uint8_t len_field, uint64_t data, size_t data_bits,
                      bool wide, uint64_t *word, uint8_t *bits)
{
    uint8_t payload[5 + 1 + 6 + 63 + 1];
    size_t  n = 0;

    if (data_bits > 63) {
        return false;
    }
    for (size_t i = 0; i < 5; i++) {
        payload[n++] = (uint8_t)((cmd >> i) & 1u);
    }
    if (wide) {
        payload[n++] = 0;
    }
    for (size_t i = 0; i < 6; i++) {
        payload[n++] = (uint8_t)((len_field >> i) & 1u);
    }
    for (size_t i = 0; i < data_bits; i++) {
        payload[n++] = (uint8_t)((data >> i) & 1u);
    }
    if (wide && (data_bits & 1u)) {
        payload[n++] = 0;
    }
    const size_t edge  = wide ? 2 : 1;          /* start bit, trailing zero */
    const size_t total = edge + n + 6 + edge;
    if (total > 63) {
        return false;
    }
    const uint8_t crc = dap_crc6(payload, n);
    uint64_t w = wide ? 3u : 1u;
    for (size_t i = 0; i < n; i++) {
        w |= (uint64_t)payload[i] << (edge + i);
    }
    w |= (uint64_t)(crc & 0x3Fu) << (edge + n);
    *word = w;
    *bits = (uint8_t)total;
    return true;
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

/*
 * CMD (0x03) through DATA (0x17) in one auto-incrementing burst.  The
 * registers in between are rewritten from their shadows; LEVEL (0x0D/0x0E)
 * is read-only and ignores the write.
 */
static esp_err_t load_frame_lead(uint8_t cmd, uint8_t len_field, uint64_t data,
                                 size_t data_bits, size_t reply_bits, uint8_t parcels,
                                 uint8_t lead);

static esp_err_t load_frame_parcels(uint8_t cmd, uint8_t len_field, uint64_t data,
                                    size_t data_bits, size_t reply_bits, uint8_t parcels)
{
    return load_frame_lead(cmd, len_field, data, data_bits, reply_bits, parcels, s_lead);
}

static esp_err_t load_frame_lead(uint8_t cmd, uint8_t len_field, uint64_t data,
                                 size_t data_bits, size_t reply_bits, uint8_t parcels,
                                 uint8_t lead)
{
    uint8_t b[REG_DATA + 8 - REG_CMD];
    uint8_t flags = s_flags;

    /* Fast mode sends the frame raw; one that does not fit goes at DIV. */
    if (s_fast) {
        uint64_t word = 0;
        uint8_t  bits = 0;
        if (raw_frame(cmd, len_field, data, data_bits, (s_flags & FLAG_WIDE) != 0u,
                      &word, &bits)) {
            data      = word;
            data_bits = bits;
            flags    |= FLAG_FAST | FLAG_RAW_FRAME;
        }
    }

    b[REG_CMD - REG_CMD]         = (uint8_t)(cmd & 0x1F);
    b[REG_LEN - REG_CMD]         = (uint8_t)(len_field & 0x3F);
    b[REG_DBITS - REG_CMD]       = (uint8_t)(data_bits & 0x3F);
    b[REG_RBITS - REG_CMD]       = (uint8_t)(reply_bits & 0x7F);
    b[REG_TRAIL - REG_CMD]       = s_trail;
    b[REG_MAXWAIT - REG_CMD]     = (uint8_t)s_maxwait;
    b[REG_MAXWAIT + 1 - REG_CMD] = (uint8_t)(s_maxwait >> 8);
    b[REG_PARCELS - REG_CMD]     = parcels;
    b[REG_FLAGS - REG_CMD]       = flags;
    b[REG_LEAD - REG_CMD]        = lead;
    b[REG_LEVEL - REG_CMD]       = 0;
    b[REG_LEVEL + 1 - REG_CMD]   = 0;
    b[REG_SKEW - REG_CMD]        = s_skew;
    for (size_t i = 0; i < 8; i++) {
        b[REG_DATA - REG_CMD + i] = (uint8_t)(data >> (8 * i));
    }
    return reg_write(REG_CMD, b, sizeof(b));
}

static esp_err_t load_frame(uint8_t cmd, uint8_t len_field,
                            uint64_t data, size_t data_bits, size_t reply_bits)
{
    return load_frame_parcels(cmd, len_field, data, data_bits, reply_bits, 0);
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

    /* REPLY, RCRC and WAIT are contiguous: one read. */
    if (reply != NULL || wait_cycles != NULL) {
        uint8_t raw[REG_WAIT + 2 - REG_REPLY] = {0};
        reg_read(REG_REPLY, raw, sizeof(raw));
        if (reply != NULL) {
            *reply = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
                     ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
        }
        if (wait_cycles != NULL) {
            *wait_cycles = (uint16_t)((uint16_t)raw[REG_WAIT - REG_REPLY] |
                                      ((uint16_t)raw[REG_WAIT + 1 - REG_REPLY] << 8));
        }
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

/*
 * Chained block reads.  The fabric holds a start written while it is busy and
 * takes it the moment the running block ends, so the next block's frame is
 * loaded and queued as soon as the previous queued one has been taken: blocks
 * run back to back while the FIFO drains continuously, and the tail of one
 * block's drain overlaps the next block's parcels.  Words land in `words` in
 * request order (the FIFO carries each word LSB first, as the S3 stores it).
 *
 * The fabric flow-controls: with the FIFO full it stops clocking between
 * parcels until the host has drained, so a slow or preempted host costs time,
 * never data.
 */
static uint8_t s_chain_lead = 2;

void dap_phy_fpga_set_chain_lead(uint8_t clocks)
{
    s_chain_lead = (clocks > 63) ? 63 : clocks;
}

static esp_err_t queue_block(const dap_fpga_block_t *b, bool first)
{
    if (load_frame_lead(0x0Au, (uint8_t)b->payload_bits, b->payload, b->payload_bits,
                        0, (uint8_t)(b->count - 1), first ? s_lead : s_chain_lead) != ESP_OK) {
        return ESP_FAIL;
    }
    return reg_write8(REG_CTRL, first ? (CTRL_CLEAR_FIFO | CTRL_START_BLOCK)
                                      : CTRL_START_BLOCK);
}

static esp_err_t dap_phy_fpga_blockread_chain_locked(const dap_fpga_block_t *blocks,
                                                     size_t n, uint32_t *words)
{
    if (!s_ready || blocks == NULL || words == NULL || n == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t bytes = 0;
    for (size_t i = 0; i < n; i++) {
        if (blocks[i].count == 0 || blocks[i].count > 256) {
            return ESP_ERR_INVALID_ARG;
        }
        bytes += blocks[i].count * 4u;
    }
    if (queue_block(&blocks[0], true) != ESP_OK) {
        return ESP_FAIL;
    }

    uint8_t      *dst  = (uint8_t *)words;
    size_t        got  = 0;
    size_t        next = 1;                     /* the next block to queue */
    uint8_t       low[REG_LEVEL + 2];
    const int64_t deadline = esp_timer_get_time() +
                             (int64_t)OP_TIMEOUT_MS * 1000 * (int64_t)((n + 7) / 8);

    for (;;) {
        /* STATUS through LEVEL (queued flag in its high byte) in one read. */
        if (reg_read(REG_STATUS, low, sizeof(low)) != ESP_OK) {
            return ESP_FAIL;
        }
        s_stats.polls++;
        const uint8_t status = low[REG_STATUS];
        const bool    queued = (low[REG_LEVEL + 1] & LEVEL_HI_QUEUED) != 0;
        size_t avail = (size_t)low[REG_LEVEL] | ((size_t)(low[REG_LEVEL + 1] & 0x0F) << 8);
        /*
         * The two LEVEL bytes are sampled a byte time apart while the block
         * fills the FIFO: a carry out of the low byte in between reads 256 too
         * many, and the surplus would come back as zeros (an empty FIFO does
         * not pop), shifting every later byte of the chain.  Near a wrap, take
         * the lower reading; draining less is harmless.
         */
        if (low[REG_LEVEL] >= 0xF0 && avail >= 0x100) {
            avail -= 0x100;
        }

        /* The last start has been taken, so its frame registers are free. */
        if (next < n && !queued && !(status & (ST_TIMED_OUT | ST_OVERRUN))) {
            if (queue_block(&blocks[next], false) != ESP_OK) {
                return ESP_FAIL;
            }
            next++;
        }

        if (avail > bytes - got) {
            avail = bytes - got;
        }
        if (avail > DRAIN_MAX) {
            avail = DRAIN_MAX;
        }
        const bool idle = !(status & ST_BUSY) && !queued;

        /* Take a full chunk while running, or the remainder once idle. */
        if (avail && (avail >= DRAIN_CHUNK || idle || got + avail == bytes)) {
            if (reg_read(REG_FIFO, dst + got, avail) != ESP_OK) {
                return ESP_FAIL;
            }
            got += avail;
            if (got == bytes) {
                return ESP_OK;
            }
            continue;
        }

        if (idle && next == n) {
            if (status & ST_TIMED_OUT) {
                ESP_LOGW(TAG, "a parcel timed out; the chain stopped after %u of "
                              "%u bytes", (unsigned)got, (unsigned)bytes);
                return ESP_ERR_TIMEOUT;
            }
            if (status & ST_OVERRUN) {
                /* Only an abort ends a flow-controlled block this way. */
                ESP_LOGE(TAG, "the chain was aborted on a full FIFO after %u of %u bytes",
                         (unsigned)got, (unsigned)bytes);
                return ESP_ERR_NO_MEM;
            }
            if (avail == 0) {
                ESP_LOGW(TAG, "the chain ended with %u of %u bytes",
                         (unsigned)got, (unsigned)bytes);
                return ESP_ERR_INVALID_SIZE;
            }
        }
        if (idle && next < n && (status & (ST_TIMED_OUT | ST_OVERRUN)) && avail == 0) {
            /* A failed block drops the queued start; report it. */
            ESP_LOGW(TAG, "block %u of %u failed (status 0x%02X)", (unsigned)next,
                     (unsigned)n, status);
            return (status & ST_OVERRUN) ? ESP_ERR_NO_MEM : ESP_ERR_TIMEOUT;
        }
        if (esp_timer_get_time() >= deadline) {
            ESP_LOGE(TAG, "the chain never finished (status 0x%02X, %u of %u)",
                     status, (unsigned)got, (unsigned)bytes);
            /* A block stalled on flow control would wait forever. */
            reg_write8(REG_CTRL, CTRL_ABORT | CTRL_CLEAR_FIFO);
            return ESP_ERR_TIMEOUT;
        }
    }
}

static esp_err_t dap_phy_fpga_blockread_locked(uint64_t cmd_payload, size_t payload_bits,
                                               uint32_t *words, size_t count)
{
    const dap_fpga_block_t b = {
        .payload = cmd_payload, .payload_bits = (uint8_t)payload_bits, .count = (uint16_t)count,
    };
    return dap_phy_fpga_blockread_chain_locked(&b, 1, words);
}

/* client_blockwrite of up to DAP_FPGA_BLOCK_WORDS words from the write FIFO. */
static esp_err_t dap_phy_fpga_block_write_locked(uint32_t address, const uint32_t *words,
                                                 size_t count)
{
    uint8_t status = 0;

    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    /* In wide mode block-write parcels arrive corrupted (bit 31 lost at
     * 24 MHz) and a bad one can leave the device unresponsive; callers fall
     * back to word writes, which are sound. */
    if (s_flags & FLAG_WIDE) {
        return ESP_ERR_NOT_SUPPORTED;
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

    if (load_frame_parcels(0x09u, 40, payload, 40, 0, (uint8_t)(count - 1)) != ESP_OK) {
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

esp_err_t dap_phy_fpga_blockread_chain(const dap_fpga_block_t *blocks, size_t n,
                                       uint32_t *words)
{
    dap_lock();
    const esp_err_t err = dap_phy_fpga_blockread_chain_locked(blocks, n, words);
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
