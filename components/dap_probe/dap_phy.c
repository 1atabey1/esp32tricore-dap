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

/*
 * Busy-wait on the CPU cycle counter.  esp_rom_delay_us() bottoms out at 1 us,
 * which caps the bit rate at 500 kHz; the cycle counter gets us to a few MHz
 * with tolerable jitter.  Jitter is harmless here - see the file comment.
 */
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

/*
 * Precomputed register masks for the clock and data pins.
 *
 * Both DAP pins on this board are above GPIO31, so they live in the second
 * bank (out1/in1).  The masks are set up once at init and the hot path is then
 * a single store per edge.
 */
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
    /*
     * Half a period in CPU cycles, less what the register writes themselves
     * cost.  Below about 8 MHz the spin dominates and the rate comes out close
     * to the request; above that the stores set the pace and the request is a
     * ceiling rather than a setting.
     */
    s_half_ticks = cpu_hz / (2u * clock_hz);
    if (s_half_ticks > DAP_EDGE_OVERHEAD_TICKS) {
        s_half_ticks -= DAP_EDGE_OVERHEAD_TICKS;
    } else {
        s_half_ticks = 0;
    }
    /* Keep both backends at the same nominal rate, so a sweep compares like
     * with like without every caller having to know which one is active. */
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
        /*
         * These are not decoration.  Two clocks with the line held low before
         * every frame is what makes the target accept the frame at all, and
         * the long low burst is the documented flush after an error.  Left on
         * the bit-bang path while the pads point at SPI they would be silently
         * dropped - gpio_set_level() on a pad driven by a peripheral does
         * nothing - and every frame would go out without its lead-in.
         */
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
 * Over-clocked bits, kept so nothing is lost.
 *
 * The bit-bang await stops on the exact clock that carries the start bit.  SPI
 * cannot: a transfer has to finish before its result can be looked at, so
 * anything after the start bit in that transfer is already out on the wire.
 * Throwing those away would desynchronise every reply - the payload would be
 * read starting some bits late.  They go in this queue instead, and
 * dap_phy_read_bits() drains it before clocking anything new.
 *
 * The await chunk is small for the same reason: it bounds how far past the
 * start bit a search can run.
 */
/*
 * One bit per chunk, deliberately.
 *
 * Over-clocking is free when a reply carries data - the extra bits go in the
 * queue and the read drains them - but a bare acknowledge has no read after
 * it, so anything clocked past its start bit runs into the next telegram and
 * gets the *following* command discarded.  That is a documented way to turn
 * one fault into an intermittent failure of an unrelated command, and it is
 * not worth eight bits of transfer economy.  Waits here are 0 to 2 cycles in
 * practice, so this costs a handful of transfers per exchange.
 */
#define DAP_SPI_AWAIT_CHUNK 1

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
        /*
         * One transfer for the lead clocks and the frame together.
         *
         * Split across two transfers there is a software gap between them with
         * the clock parked, and while DAP is synchronous and should not care,
         * the lead-in is the one part of the framing that was found
         * empirically rather than read out of a document.  Keeping it
         * contiguous removes the question, and costs one transfer instead of
         * two.
         */
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
    /* Anything still queued belongs to the reply that just ended. */
    rxq_reset();

    gpio_set_direction((gpio_num_t)s_cfg.dat_pin, GPIO_MODE_OUTPUT);
    if (s_use_spi) {
        /*
         * That call just pointed the pad back at the GPIO output register - it
         * always does, to guarantee no peripheral is left driving - so the SPI
         * data signal has to be wired up again before the next frame.
         */
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
        /* The rest in one transfer, exactly as long as it needs to be, so no
         * clock is spent past the end of the reply. */
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
    bool seen_low = false;

    /*
     * A reply is busy stuffing - the target holding DAP1 low - followed by a
     * start bit, so this is "skip zeros, take the first one".
     *
     * It does not require a zero first, and that matters.  The bit-bang path
     * samples before raising the clock, so its first sample is the level left
     * over from our own frame's trailing zero rather than target data; the SPI
     * path samples on the clock edge and has no such artifact.  A rule that
     * demanded a leading zero was satisfied by that artifact on one backend
     * and skipped the real start bit on the other, reading every reply two
     * bits late.
     *
     * What the rule was originally guarding against - a floating-high line
     * reported as a confident all-ones reply - is now caught where it belongs,
     * by the reply CRC check in the transaction layer.
     */
    if (s_use_spi) {
        /*
         * Same rule, searched a chunk at a time.  Sampling one bit per SPI
         * transaction would cost a whole transfer setup per clock and give
         * back everything this backend buys; a chunk of eight bounds the
         * overshoot to seven clocks, and those seven are kept in the queue for
         * dap_phy_read_bits() rather than thrown away.
         */
        rxq_reset();
        for (uint32_t i = 0; i < max_cycles; i++) {
            const int bit = rxq_next();
            if (bit < 0) {
                break;                  /* transfer failed: fall through */
            }
            if (bit) {
                /*
                 * One clock past the start bit, always, and keep the bit.
                 *
                 * This is what makes the two backends clock-for-clock
                 * identical.  The bit-bang path samples before raising the
                 * clock, so after finding the start bit it has issued one more
                 * clock than cells it has looked at - and the device needs
                 * that clock.  Measured: an acknowledge followed by no further
                 * clocks leaves the next command ignored, which showed up as
                 * CLIENT_ID reading 0xFFFF right after a client_set that had
                 * been acknowledged.  With a data reply the bit is the first
                 * payload bit and the queue hands it to the read, so nothing
                 * is lost either way.
                 */
                rxq_reset();
                if (dap_phy_spi_read_bits(s_rxq, 1) == ESP_OK) {
                    s_rxq_len = 1;
                }
                return (int)i;
            }
            seen_low = true;
        }
        return -1;
    }

    for (uint32_t i = 0; i < max_cycles; i++) {
        const int bit = clock_in_bit();
        if (bit) {
            return (int)i;              /* wait cycles before the start bit */
        }
        seen_low = true;
    }
    (void)seen_low;
    return -1;                          /* stayed low: a real timeout */
}

void dap_phy_set_expected_wait(size_t cycles)
{
    s_exp_wait = cycles;
}

size_t dap_phy_get_expected_wait(void)
{
    return s_exp_wait;
}

int dap_phy_read_reply(uint8_t *bits, size_t reply_bits, uint32_t max_wait)
{
    /*
     * A reply is [busy zeros][start bit][payload][CRC6].  This clocks a window
     * big enough for all of it in one go, then locates the start bit in what
     * came back and copies out the payload and CRC that follow.
     *
     * The slack past the end of the reply is deliberate and small: those
     * clocks land in the trailer region after the CRC, where the device
     * expects the telegram to be run out, and keeping them inside the same
     * transfer means the clock never stops mid-reply.
     *
     * Returns the number of busy cycles before the start bit, or -1.
     */
    const size_t need = reply_bits ? reply_bits + 6 : 0;

    if (s_use_spi) {
        uint8_t win[DAP_REPLY_WINDOW_MAX];
        /*
         * Exactly what the bit-bang path would clock: the expected busy
         * cycles, the start bit, and then the reply - or one further clock
         * when there is no reply to read, which is what an acknowledge wants.
         */
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

        /*
         * The start bit was not where the window assumed.  Find it, adopt the
         * new count, and report a miss: the caller retries, and the retry is
         * clocked correctly.  Adapting beats guessing - the value differs per
         * command and there is no document that gives it.
         */
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

    /*
     * NOTE ON WHAT THIS DOES NOT PROVE.  In INPUT_OUTPUT mode the S3 reads back
     * its *own* pad, so the FPGA's buffer is not in the loop and this passes
     * whatever the direction pin is doing.  It therefore only rules out a dead
     * S3 output or a misconfigured pin - it says nothing about whether the data
     * reaches the connector.  Only watching the pads settles that; see
     * dap_phy_force_dir() and the analyser harness in main.c.
     */
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
    /*
     * Can plain GPIO still drive both DAP pads?
     *
     * This is the question the existing self-drive check does not answer: it
     * only ever tested the data pin.  After the SPI backend has owned these
     * pads, "the target went quiet" has two very different causes - the pads
     * no longer being under GPIO control, or the target being desynchronised -
     * and they need different fixes.  Driving each pad and reading its own
     * level back separates them.
     */
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

void dap_phy_force_dir(int level)
{
    gpio_set_level((gpio_num_t)s_cfg.dir_pin, level);
    gpio_set_direction((gpio_num_t)s_cfg.dat_pin, GPIO_MODE_OUTPUT);
}

void dap_phy_drive_data(int level)
{
    dat_set(level);
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
