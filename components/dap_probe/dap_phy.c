#include "dap_phy.h"

#include <inttypes.h>
#include <string.h>

#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "DAP_PHY";

static dap_phy_cfg_t s_cfg;
static bool          s_ready;
static uint32_t      s_half_ticks;   /* CPU cycles per half clock period */
static bool          s_use_spi;      /* route clocking through GP-SPI */
static size_t        s_exp_wait = 1;      /* busy cycles the reply window allows for */

/* Busy-wait on the CPU cycle counter (sub-microsecond; jitter is harmless). */
static inline void delay_ticks(uint32_t ticks)
{
    if (ticks == 0) {
        return;                     /* the stores themselves are the delay */
    }
    const uint32_t start = esp_cpu_get_cycle_count();
    while ((esp_cpu_get_cycle_count() - start) < ticks) {
        /* spin */
    }
}

/* Precomputed set/clear/input registers and masks: one store per edge. */
static volatile uint32_t *s_clk_set_reg;
static volatile uint32_t *s_clk_clr_reg;
static volatile uint32_t *s_dat_set_reg;
static volatile uint32_t *s_dat_clr_reg;
static volatile uint32_t *s_dat_in_reg;
static uint32_t           s_clk_mask;
static uint32_t           s_dat_mask;

static void pin_regs_init(void)
{
    const int clk = s_cfg.clk_pin;
    const int dat = s_cfg.dat_pin;

    if (clk < 32) {
        s_clk_set_reg = &GPIO.out_w1ts;
        s_clk_clr_reg = &GPIO.out_w1tc;
        s_clk_mask    = 1u << clk;
    } else {
        s_clk_set_reg = &GPIO.out1_w1ts.val;
        s_clk_clr_reg = &GPIO.out1_w1tc.val;
        s_clk_mask    = 1u << (clk - 32);
    }

    if (dat < 32) {
        s_dat_set_reg = &GPIO.out_w1ts;
        s_dat_clr_reg = &GPIO.out_w1tc;
        s_dat_in_reg  = &GPIO.in;
        s_dat_mask    = 1u << dat;
    } else {
        s_dat_set_reg = &GPIO.out1_w1ts.val;
        s_dat_clr_reg = &GPIO.out1_w1tc.val;
        s_dat_in_reg  = &GPIO.in1.val;
        s_dat_mask    = 1u << (dat - 32);
    }
}

static inline void clk_set(int level)
{
    *(level ? s_clk_set_reg : s_clk_clr_reg) = s_clk_mask;
}

static inline void dat_set(int level)
{
    *(level ? s_dat_set_reg : s_dat_clr_reg) = s_dat_mask;
}

static inline int dat_get(void)
{
    return (*s_dat_in_reg & s_dat_mask) ? 1 : 0;
}

/* One full clock period with `bit` presented to the target. */
static inline void clock_out_bit(int bit)
{
    dat_set(bit);
    delay_ticks(s_half_ticks);   /* satisfies t16 setup with room to spare */
    clk_set(1);                  /* target latches here */
    delay_ticks(s_half_ticks);
    clk_set(0);
}

/* One full clock period with the target driving; sampled just before the rise. */
static inline int clock_in_bit(void)
{
    delay_ticks(s_half_ticks);   /* low phase: target's t19 elapses here */
    const int bit = dat_get();
    clk_set(1);
    delay_ticks(s_half_ticks);
    clk_set(0);
    return bit;
}

static inline uint32_t cpu_freq_hz(void)
{
    return (uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000u;
}

void dap_phy_set_clock(uint32_t clock_hz)
{
    const uint32_t cpu_hz = cpu_freq_hz();

    if (clock_hz == 0) {
        clock_hz = 1000000u;
    }
    s_cfg.clock_hz = clock_hz;
    /* Half period in CPU cycles, less the store overhead; a ceiling above ~8 MHz. */
    s_half_ticks = cpu_hz / (2u * clock_hz);
    if (s_half_ticks > DAP_EDGE_OVERHEAD_TICKS) {
        s_half_ticks -= DAP_EDGE_OVERHEAD_TICKS;
    } else {
        s_half_ticks = 0;
    }
    /* Keep both backends at the same nominal rate. */
    if (dap_phy_spi_ready()) {
        dap_phy_spi_set_clock(clock_hz);
    }
}

esp_err_t dap_phy_init(const dap_phy_cfg_t *cfg)
{
    if (cfg == NULL || cfg->clk_pin < 0 || cfg->dat_pin < 0 || cfg->dir_pin < 0) {
        ESP_LOGE(TAG, "DAP pins are not available on this board");
        return ESP_ERR_NOT_SUPPORTED;
    }

    s_cfg = *cfg;
    pin_regs_init();

    uint64_t mask = (1ULL << cfg->clk_pin) | (1ULL << cfg->dat_pin) |
                    (1ULL << cfg->dir_pin);
    if (cfg->trst_pin >= 0) {
        mask |= (1ULL << cfg->trst_pin);
    }

    gpio_config_t io = {
        .pin_bit_mask = mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    const esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }

    /* Park idle: probe drives, clock low, data high, TRST released. */
    gpio_set_level((gpio_num_t)cfg->dir_pin, 0);
    clk_set(0);
    dat_set(1);
    if (cfg->trst_pin >= 0) {
        gpio_set_level((gpio_num_t)cfg->trst_pin, 1);
    }

    dap_phy_set_clock(cfg->clock_hz);
    s_ready = true;

    ESP_LOGI(TAG, "DAP0=%d DAP1=%d dir=%d trst=%d at ~%" PRIu32 " Hz (%" PRIu32 " ticks/half)",
             cfg->clk_pin, cfg->dat_pin, cfg->dir_pin, cfg->trst_pin,
             s_cfg.clock_hz, s_half_ticks);
    return ESP_OK;
}

bool dap_phy_ready(void)
{
    return s_ready;
}

void dap_phy_idle_clocks(size_t count, int level)
{
    if (s_use_spi) {
        /* Must go through SPI: the pads ignore GPIO while routed to it. */
        uint8_t bits[64];
        const uint8_t v = level ? 1u : 0u;

        memset(bits, v, sizeof(bits));
        while (count) {
            const size_t n = count > sizeof(bits) ? sizeof(bits) : count;
            if (dap_phy_spi_write_bits(bits, n) != ESP_OK) {
                break;
            }
            count -= n;
        }
        return;
    }
    for (size_t i = 0; i < count; i++) {
        clock_out_bit(level);
    }
}

/*
 * SPI start-bit search granularity.  Must stay 1: clocks past a bare
 * acknowledge run into the next telegram and get it discarded.
 */
#define DAP_SPI_AWAIT_CHUNK 1

/* Bits clocked in during the start-bit search; dap_phy_read_bits() drains them first. */
static uint8_t  s_rxq[64];
static size_t   s_rxq_len;
static size_t   s_rxq_pos;

static void rxq_reset(void)
{
    s_rxq_len = 0;
    s_rxq_pos = 0;
}

/* One bit from the queue, refilling it from the wire when it runs dry. */
static int rxq_next(void)
{
    if (s_rxq_pos >= s_rxq_len) {
        rxq_reset();
        if (dap_phy_spi_read_bits(s_rxq, DAP_SPI_AWAIT_CHUNK) != ESP_OK) {
            return -1;
        }
        s_rxq_len = DAP_SPI_AWAIT_CHUNK;
    }
    return s_rxq[s_rxq_pos++];
}

void dap_phy_use_spi(bool enable)
{
    s_use_spi = enable && dap_phy_spi_ready();
    rxq_reset();
    dap_phy_spi_route(s_use_spi);
    if (!s_use_spi) {
        /* Restore the parked idle the bit-bang path assumes. */
        clk_set(0);
        dat_set(1);
    }
}

bool dap_phy_is_spi(void)
{
    return s_use_spi;
}

void dap_phy_write_frame_with_lead(const dap_frame_t *frame, size_t lead)
{
    if (s_use_spi) {
        /* Lead clocks and frame in one transfer, with no gap between them. */
        uint8_t buf[DAP_FRAME_MAX_BITS + 16];

        if (lead + frame->len <= sizeof(buf)) {
            memset(buf, 0, lead);
            memcpy(buf + lead, frame->bit, frame->len);
            if (dap_phy_spi_write_bits(buf, lead + frame->len) == ESP_OK) {
                return;
            }
        }
    }
    dap_phy_idle_clocks(lead, 0);
    for (size_t i = 0; i < frame->len; i++) {
        clock_out_bit(frame->bit[i]);
    }
}

void dap_phy_write_frame(const dap_frame_t *frame)
{
    if (s_use_spi && dap_phy_spi_write_bits(frame->bit, frame->len) == ESP_OK) {
        return;
    }
    for (size_t i = 0; i < frame->len; i++) {
        clock_out_bit(frame->bit[i]);
    }
}

void dap_phy_turnaround_to_read(void)
{
    /* Switch the FPGA buffer first, then release the S3 driver, so they never fight. */
    gpio_set_level((gpio_num_t)s_cfg.dir_pin, 1);
    gpio_set_direction((gpio_num_t)s_cfg.dat_pin, GPIO_MODE_INPUT);
}

void dap_phy_turnaround_to_write(void)
{
    /* Anything still queued belongs to the reply that just ended. */
    rxq_reset();

    gpio_set_direction((gpio_num_t)s_cfg.dat_pin, GPIO_MODE_OUTPUT);
    if (s_use_spi) {
        /* gpio_set_direction() reset the pad to GPIO; route SPI back. */
        dap_phy_spi_route(true);
    }
    gpio_set_level((gpio_num_t)s_cfg.dir_pin, 0);
}

void dap_phy_read_bits(uint8_t *bits, size_t nbits)
{
    if (s_use_spi) {
        size_t got = 0;

        /* Whatever the start-bit search over-clocked comes first. */
        while (got < nbits && s_rxq_pos < s_rxq_len) {
            bits[got++] = s_rxq[s_rxq_pos++];
        }
        /* The rest in one exact-length transfer. */
        if (got == nbits || dap_phy_spi_read_bits(bits + got, nbits - got) == ESP_OK) {
            return;
        }
    }
    for (size_t i = 0; i < nbits; i++) {
        bits[i] = (uint8_t)clock_in_bit();
    }
}

int dap_phy_await_start_bit(uint32_t max_cycles)
{
    /*
     * Skip busy zeros, take the first one.  No leading zero is required: the
     * bit-bang path's first sample is our own frame's trailing level.  A
     * floating-high line is caught later by the reply CRC.
     */
    if (s_use_spi) {
        rxq_reset();
        for (uint32_t i = 0; i < max_cycles; i++) {
            const int bit = rxq_next();
            if (bit < 0) {
                break;                  /* transfer failed: fall through */
            }
            if (bit) {
                /*
                 * One more clock past the start bit, as bit-bang issues; the
                 * device needs it after an acknowledge.  The bit is queued as
                 * the first payload bit.
                 */
                rxq_reset();
                if (dap_phy_spi_read_bits(s_rxq, 1) == ESP_OK) {
                    s_rxq_len = 1;
                }
                return (int)i;
            }
        }
        return -1;
    }

    for (uint32_t i = 0; i < max_cycles; i++) {
        const int bit = clock_in_bit();
        if (bit) {
            return (int)i;              /* wait cycles before the start bit */
        }
    }
    return -1;                          /* stayed low: a real timeout */
}

int dap_phy_read_reply(uint8_t *bits, size_t reply_bits, uint32_t max_wait)
{
    /* Reply: [busy zeros][start bit][payload][CRC6]. */
    const size_t need = reply_bits ? reply_bits + 6 : 0;

    if (s_use_spi) {
        uint8_t win[DAP_REPLY_WINDOW_MAX];
        /* Expected busy cycles, start bit, then the reply (or one clock for an ack). */
        size_t take = s_exp_wait + 1 + (need ? need : 1);

        if (take > sizeof(win)) {
            take = sizeof(win);
        }
        if (dap_phy_spi_read_bits(win, take) != ESP_OK) {
            return -1;
        }

        if (win[s_exp_wait] && (s_exp_wait == 0 || !win[s_exp_wait - 1])) {
            for (size_t k = 0; k < need; k++) {
                bits[k] = win[s_exp_wait + 1 + k];
            }
            return (int)s_exp_wait;
        }

        /* Start bit misplaced: adopt the observed wait and report a miss for retry. */
        size_t found = 0;
        while (found < take && !win[found]) {
            found++;
        }
        if (found < take) {
            if (found != s_exp_wait) {
                ESP_LOGD(TAG, "reply wait was %u, not %u - adopting it",
                         (unsigned)found, (unsigned)s_exp_wait);
                s_exp_wait = found;
            }
        }
        return -1;
    }

    const int wait = dap_phy_await_start_bit(max_wait);
    if (wait < 0) {
        return wait;
    }
    if (need) {
        dap_phy_read_bits(bits, need);
    }
    return wait;
}

void dap_phy_set_trst(bool asserted)
{
    if (s_cfg.trst_pin >= 0) {
        gpio_set_level((gpio_num_t)s_cfg.trst_pin, asserted ? 0 : 1);
    }
}

int dap_phy_sample_target_idle(int samples)
{
    int high = 0;

    dap_phy_turnaround_to_read();
    for (int i = 0; i < samples; i++) {
        high += dat_get();
        delay_ticks(s_half_ticks);
    }
    dap_phy_turnaround_to_write();
    return high;
}

bool dap_phy_self_drive_check(void)
{
    bool ok = true;

    /* Reads the S3's own pad: the FPGA buffer and connector are not tested. */
    gpio_set_level((gpio_num_t)s_cfg.dir_pin, 0);
    gpio_set_direction((gpio_num_t)s_cfg.dat_pin, GPIO_MODE_INPUT_OUTPUT);

    dat_set(1);
    delay_ticks(s_half_ticks * 4);
    ok = ok && (dat_get() == 1);

    dat_set(0);
    delay_ticks(s_half_ticks * 4);
    ok = ok && (dat_get() == 0);

    dat_set(1);
    gpio_set_direction((gpio_num_t)s_cfg.dat_pin, GPIO_MODE_OUTPUT);
    return ok;
}

bool dap_phy_pad_toggle_check(void)
{
    const int pins[2] = { s_cfg.clk_pin, s_cfg.dat_pin };
    const char *names[2] = { "DAP0/clk", "DAP1/data" };
    bool all_ok = true;

    for (int i = 0; i < 2; i++) {
        gpio_set_direction((gpio_num_t)pins[i], GPIO_MODE_INPUT_OUTPUT);
        gpio_set_level((gpio_num_t)pins[i], 1);
        delay_ticks(s_half_ticks * 4);
        const int hi = gpio_get_level((gpio_num_t)pins[i]);
        gpio_set_level((gpio_num_t)pins[i], 0);
        delay_ticks(s_half_ticks * 4);
        const int lo = gpio_get_level((gpio_num_t)pins[i]);
        const bool ok = (hi == 1) && (lo == 0);

        ESP_LOGI(TAG, "pad %s (GPIO%d): drove 1 read %d, drove 0 read %d - %s",
                 names[i], pins[i], hi, lo, ok ? "GPIO controls it" : "NOT UNDER GPIO CONTROL");
        all_ok = all_ok && ok;
    }

    /* Leave them the way the bit-bang path expects. */
    gpio_set_direction((gpio_num_t)s_cfg.clk_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)s_cfg.dat_pin, GPIO_MODE_OUTPUT);
    clk_set(0);
    dat_set(1);
    return all_ok;
}

