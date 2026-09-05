#include "dap_phy.h"

#include <inttypes.h>

#include "driver/gpio.h"
#include "esp_cpu.h"
#include "esp_log.h"

static const char *TAG = "DAP_PHY";

static dap_phy_cfg_t s_cfg;
static bool          s_ready;
static uint32_t      s_half_ticks;   /* CPU cycles per half clock period */

/*
 * Busy-wait on the CPU cycle counter.  esp_rom_delay_us() bottoms out at 1 us,
 * which caps the bit rate at 500 kHz; the cycle counter gets us to a few MHz
 * with tolerable jitter.  Jitter is harmless here - see the file comment.
 */
static inline void delay_ticks(uint32_t ticks)
{
    const uint32_t start = esp_cpu_get_cycle_count();
    while ((esp_cpu_get_cycle_count() - start) < ticks) {
        /* spin */
    }
}

static inline void clk_set(int level)
{
    gpio_set_level((gpio_num_t)s_cfg.clk_pin, level);
}

static inline void dat_set(int level)
{
    gpio_set_level((gpio_num_t)s_cfg.dat_pin, level);
}

static inline int dat_get(void)
{
    return gpio_get_level((gpio_num_t)s_cfg.dat_pin);
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

/*
 * One full clock period with the target driving.  The target updates DAP1
 * after the falling edge, so sample at the end of the low phase, just before
 * the rising edge that advances it again.
 */
static inline int clock_in_bit(void)
{
    delay_ticks(s_half_ticks);   /* low phase: target's t19 elapses here */
    const int bit = dat_get();
    clk_set(1);
    delay_ticks(s_half_ticks);
    clk_set(0);
    return bit;
}

/* The configured maximum is what the bring-up path actually runs at. */
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
    /* Half a period, minus nothing: the GPIO calls themselves add delay, so
     * the real rate comes out below the request.  Bring-up cares about
     * correctness, not calibration; the SPI path is where rate matters. */
    s_half_ticks = cpu_hz / (2u * clock_hz);
    if (s_half_ticks == 0) {
        s_half_ticks = 1;
    }
}

esp_err_t dap_phy_init(const dap_phy_cfg_t *cfg)
{
    if (cfg == NULL || cfg->clk_pin < 0 || cfg->dat_pin < 0 || cfg->dir_pin < 0) {
        ESP_LOGE(TAG, "DAP pins are not available on this board");
        return ESP_ERR_NOT_SUPPORTED;
    }

    s_cfg = *cfg;

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
    for (size_t i = 0; i < count; i++) {
        clock_out_bit(level);
    }
}

void dap_phy_write_frame(const dap_frame_t *frame)
{
    for (size_t i = 0; i < frame->len; i++) {
        clock_out_bit(frame->bit[i]);
    }
}

void dap_phy_turnaround_to_read(void)
{
    /*
     * Order matters: hand the FPGA's buffer over first, then release the S3's
     * own driver, so the two are never fighting.  The gap this opens costs
     * nothing, because the timeout counts clocks and none are issued here.
     */
    gpio_set_level((gpio_num_t)s_cfg.dir_pin, 1);
    gpio_set_direction((gpio_num_t)s_cfg.dat_pin, GPIO_MODE_INPUT);
}

void dap_phy_turnaround_to_write(void)
{
    gpio_set_direction((gpio_num_t)s_cfg.dat_pin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)s_cfg.dir_pin, 0);
}

void dap_phy_read_bits(uint8_t *bits, size_t nbits)
{
    for (size_t i = 0; i < nbits; i++) {
        bits[i] = (uint8_t)clock_in_bit();
    }
}

int dap_phy_await_start_bit(uint32_t max_cycles)
{
    for (uint32_t i = 0; i < max_cycles; i++) {
        if (clock_in_bit()) {
            return (int)i;   /* wait cycles consumed before the start bit */
        }
    }
    return -1;
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

    /* INPUT_OUTPUT keeps the input path alive while the S3 drives, so the pad
     * can be read back.  The FPGA's buffer is in the loop as well, but its
     * outbound enable is what we are testing here. */
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

void dap_phy_training_pattern(int reps)
{
    for (int r = 0; r < reps; r++) {
        clock_out_bit(1);
        clock_out_bit(0);
        clock_out_bit(1);
        clock_out_bit(0);
    }
}
