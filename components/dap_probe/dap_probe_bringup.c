/*
 * The ordered bring-up report: each step names its own failure.
 */

#include <inttypes.h>
#include <string.h>

#include "board_profile.h"
#include "dap_frame.h"
#include "dap_phy.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "dap_probe_priv.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DAP";

esp_err_t dap_probe_spi_bringup(void)
{
#if !AEL_BOARD_HAS_DAP_PROBE
    return ESP_ERR_NOT_SUPPORTED;
#else
    dap_exchange_t x;
    int failures = 0;

    ESP_LOGW(TAG, "=== Phase 1c: GP-SPI backend ===");

    /* Confirm the target answers on bit-bang first, so SPI failures are attributable. */
    {
        const esp_err_t pre = dap_probe_attach(&x, 3);
        ESP_LOGW(TAG, "  before SPI, bit-bang sync -> 0x%08" PRIX64 " %s", x.reply,
                 (pre == ESP_OK && x.reply == DAP_SYNC_EXPECT) ? "(target is answering)"
                                                               : "(TARGET ALREADY SILENT)");
        if (pre != ESP_OK || x.reply != DAP_SYNC_EXPECT) {
            ESP_LOGE(TAG, "  nothing to measure against - fix the bit-bang attach first");
            return ESP_ERR_INVALID_STATE;
        }
    }

    esp_err_t err = dap_phy_spi_init(AEL_DAP0_PIN, AEL_DAP1_PIN,
                                     CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI backend init failed: %s", esp_err_to_name(err));
        return err;
    }

    dap_phy_use_spi(true);
    ESP_LOGI(TAG, "backend is now %s", dap_phy_is_spi() ? "GP-SPI" : "bit-bang");

    dap_phy_spi_log_routing();

    /* Prove the peripheral reaches the pads before asking the target anything. */
    if (dap_phy_spi_loopback(0xA5) != ESP_OK) {
        ESP_LOGE(TAG, "the SPI path does not even reach its own pads");
        dap_phy_use_spi(false);
        return ESP_FAIL;
    }

    /* And that the clock pad moves: it is the one signal with no readback. */
    if (dap_phy_spi_clock_pad_check() != ESP_OK) {
        ESP_LOGE(TAG, "no clock on the DAP0 pad - nothing downstream can work");
        dap_phy_use_spi(false);
        return ESP_FAIL;
    }

    /* And that a 19-bit frame is serialised the way the frame layer means it. */
    {
        dap_frame_t f;
        uint8_t     echo[DAP_FRAME_MAX_BITS];
        char        a[DAP_FRAME_MAX_BITS + 1], b[DAP_FRAME_MAX_BITS + 1];

        if (dap_frame_build(&f, DAP_CMD_SYNC, 63, 0, 0) &&
            dap_phy_spi_echo_bits(f.bit, f.len, echo) == ESP_OK) {
            size_t i;
            for (i = 0; i < f.len; i++) {
                a[i] = f.bit[i] ? '1' : '0';
                b[i] = echo[i]  ? '1' : '0';
            }
            a[i] = '\0';
            b[i] = '\0';
            ESP_LOGW(TAG, "  sync frame out %s", a);
            ESP_LOGW(TAG, "  read back      %s  %s", b,
                     memcmp(f.bit, echo, f.len) == 0 ? "(identical)" : "DIFFERS");
        }
    }

    /* 1: sync.  A wrong sampling edge shows up here and nowhere cheaper. */
    err = dap_probe_attach(&x, 3);
    ESP_LOGW(TAG, "  sync -> 0x%08" PRIX64 " wait %d crc 0x%02X %s %s", x.reply,
             x.wait_cycles, x.reply_crc, x.crc_ok ? "residue-ok" : "residue-BAD",
             x.reply == DAP_SYNC_EXPECT ? "(expected)" : "MISMATCH");
    if (err != ESP_OK || x.reply != DAP_SYNC_EXPECT) {
        failures++;
        /* Raw window on both backends: separates sampling phase from length errors. */
        ESP_LOGW(TAG, "  raw window, GP-SPI:");
        dap_probe_dump_sync_reply(64);
        dap_phy_use_spi(false);
        dap_phy_idle_clocks(dap_probe_max_wait, 0);
        ESP_LOGW(TAG, "  raw window, bit-bang:");
        dap_probe_dump_sync_reply(64);
        dap_phy_use_spi(true);
    }

    /* 2: a selected client and its hard-wired ID. */
    if (!failures) {
        /* Sent twice: the first exchange after a backend switch is not trusted. */
        dap_probe_client_set(1, &x);
        dap_probe_client_set(1, &x);
        for (int attempt = 0; attempt < 3; attempt++) {
            err = dap_probe_client_read(DAP_IO_CLIENT_ID, 4, 16, &x);
            if (err == ESP_OK && x.reply == DAP_CLIENT_ID_EXPECT) {
                break;
            }
            dap_phy_idle_clocks(dap_probe_max_wait, 0);
            dap_probe_clear_error_state();
        }
        ESP_LOGW(TAG, "  CLIENT_ID -> 0x%04" PRIX64 " %s", x.reply,
                 x.reply == DAP_CLIENT_ID_EXPECT ? "(expected)" : "MISMATCH");
        if (x.reply != DAP_CLIENT_ID_EXPECT) {
            failures++;
        }
    }

    /* 3: a bus read that moves, so a stuck wire cannot pass. */
    if (!failures) {
        uint32_t a = 0, b = 0;
        dap_probe_clear_error_state();
        dap_probe_set_rw_mode(true);
        if (dap_probe_read32(0xF0001010u, &a) == ESP_OK &&
            dap_probe_read32(0xF0001010u, &b) == ESP_OK && a != b) {
            ESP_LOGW(TAG, "  STM0_TIM0 0x%08" PRIX32 " -> 0x%08" PRIX32
                          " (+%" PRIu32 ")", a, b, b - a);
        } else {
            ESP_LOGE(TAG, "  STM0_TIM0 read did not advance: 0x%08" PRIX32
                          " / 0x%08" PRIX32, a, b);
            failures++;
        }
    }

    /* 4: multi-parcel framing, word for word against single reads. */
    if (!failures) {
        const uint32_t base = 0x70000000u;   /* DSPR: readable without OCDS */
        uint32_t blk[8] = {0}, one = 0;
        int mismatch = 0;

        if (dap_probe_blockread(base, blk, 8) == ESP_OK) {
            for (size_t i = 0; i < 8; i++) {
                if (dap_probe_read32(base + 4u * i, &one) != ESP_OK || one != blk[i]) {
                    mismatch++;
                }
            }
            ESP_LOGW(TAG, "  blockread 8 words: %s",
                     mismatch ? "MISMATCH vs single reads" : "matches single reads");
        } else {
            ESP_LOGE(TAG, "  blockread drew no parcels");
            mismatch++;
        }
        if (mismatch) {
            failures++;
        }
        dap_probe_clear_error_state();
    }

    if (failures) {
        dap_probe_dump_raw_attach("GP-SPI");
        dap_probe_set_trailer_bits(0);
        ESP_LOGE(TAG, "=== GP-SPI backend failed %d check%s: reverting to bit-bang ===",
                 failures, failures == 1 ? "" : "s");
        dap_phy_use_spi(false);
        /* Flush a device possibly left mid-telegram before re-checking bit-bang. */
        dap_phy_idle_clocks(dap_probe_max_wait * 2u, 0);
        if (!dap_phy_pad_toggle_check()) {
            ESP_LOGE(TAG, "the pads are no longer under GPIO control");
        }
        dap_probe_clear_error_state();
        if (dap_probe_attach(&x, 5) == ESP_OK && x.reply == DAP_SYNC_EXPECT) {
            ESP_LOGI(TAG, "bit-bang path still good after the revert");
        } else {
            ESP_LOGE(TAG, "bit-bang path is broken too - the pad routing did not "
                          "come back and a reset is needed");
        }
        return ESP_FAIL;
    }

    ESP_LOGW(TAG, "=== GP-SPI backend passes every correctness check ===");

    /* The same measurements as the bit-bang path, for a direct comparison. */
    dap_probe_rate_test();
    dap_probe_block_throughput();

    /* Rates beyond the bit-bang ceiling (~1 MHz effective). */
    {
        static const uint32_t fast[] = { 8000000u, 12000000u, 16000000u, 20000000u };
        static uint32_t buf[256];

        ESP_LOGW(TAG, "--- GP-SPI only: rates above the bit-bang ceiling ---");
        for (size_t r = 0; r < sizeof(fast) / sizeof(fast[0]); r++) {
            dap_phy_set_clock(fast[r]);

            /* Re-verify at the new rate before trusting its throughput. */
            dap_probe_clear_error_state();
            if (dap_probe_client_read(DAP_IO_CLIENT_ID, 4, 16, &x) != ESP_OK ||
                x.reply != DAP_CLIENT_ID_EXPECT) {
                ESP_LOGW(TAG, "  %8" PRIu32 " Hz: CLIENT_ID reads 0x%04" PRIX64
                              " - past the usable rate", fast[r], x.reply);
                continue;
            }

            int ok = 0;
            const int64_t t0 = esp_timer_get_time();
            for (int i = 0; i < 8; i++) {
                if (dap_probe_blockread(0x70000000u, buf, 256) == ESP_OK) {
                    ok++;
                } else {
                    dap_probe_clear_error_state();
                }
            }
            const int64_t us = esp_timer_get_time() - t0;
            if (ok && us > 0) {
                const int kbps = (int)((int64_t)ok * 1024 * 1000000 / us / 1024);
                ESP_LOGW(TAG, "  %8" PRIu32 " Hz: %d/8 blocks -> %d kB/s",
                         fast[r], ok, kbps);
            } else {
                ESP_LOGW(TAG, "  %8" PRIu32 " Hz: no blocks completed", fast[r]);
            }
        }
        dap_phy_set_clock(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
    }

    return ESP_OK;
#endif
}

esp_err_t dap_probe_bringup_report(void)
{
    dap_exchange_t x;
    esp_err_t      err;
    int            failures = 0;

    ESP_LOGI(TAG, "=== DAP bring-up: DAP0=GPIO%d DAP1=GPIO%d dir=GPIO%d trst=GPIO%d ===",
             AEL_DAP0_PIN, AEL_DAP1_PIN, AEL_DAP1_DIR_PIN, AEL_DAP_TRST_PIN);

    /* Wire check: is our own drive path working before any protocol step. */
    const bool self_ok = dap_phy_self_drive_check();
    const int  idle_high = dap_phy_sample_target_idle(32);
    ESP_LOGI(TAG, "0 wire check          self-drive %s, DAP1 idle %d/32 high",
             self_ok ? "ok" : "FAILED", idle_high);
    if (!self_ok) {
        ESP_LOGE(TAG, "   the S3 cannot read back its own drive: check Port C mode "
                      "(cfgpc) and that swd_gpio selects the GPIO path");
    }
    /* DAP1 is unpulled, so the idle level is residual charge, not the target. */
    ESP_LOGI(TAG, "   (idle level is residual charge on an unpulled net, not "
                  "evidence of a target)");
    (void)idle_high;

    /* Checkpoint 1: sync must draw the 0xAAAAAAAA training pattern. */
    err = dap_probe_attach(&x, 3);
    dap_probe_log_exchange("1 sync", &x);
    if (x.sent_word != DAP_SYNC_WIRE_WORD) {
        ESP_LOGE(TAG, "   frame assembly is wrong: expected wire word 0x%05X", DAP_SYNC_WIRE_WORD);
        failures++;
    }
    if (err != ESP_OK || x.reply != DAP_SYNC_EXPECT) {
        ESP_LOGE(TAG, "   expected 0x%08" PRIX32 " - target silent or framing off",
                 (uint32_t)DAP_SYNC_EXPECT);
        failures++;
        /* Everything downstream needs sync; try the attach variants instead. */
        if (dap_probe_clock_pin_search() == ESP_OK ||
            dap_probe_attach_sweep() == ESP_OK) {
            ESP_LOGW(TAG, "=== a sweep combination answered: adopt it and re-run ===");
        } else {
            ESP_LOGE(TAG, "=== target silent on every attach variant ===");
        }
        return ESP_FAIL;
    }

    /* The whole handshake with no host work between frames. */
    {
        static const char *names[6] = { "sync", "dapisc", "client_set(1)",
                                        "client_read ID", "spare", "spare" };
        dap_exchange_t seq[6];
        memset(seq, 0, sizeof(seq));
        dap_probe_attach_now(seq);
        ESP_LOGW(TAG, "--- gapless handshake ---");
        for (int i = 0; i < 6; i++) {
            ESP_LOGW(TAG, "  %-15s wait %-5d reply 0x%08" PRIX64 " %s",
                     names[i], seq[i].wait_cycles, seq[i].reply,
                     seq[i].idle_high ? "(idle high)"
                                      : (seq[i].timed_out ? "(held low)" : ""));
        }
        if (seq[3].reply == DAP_CLIENT_ID_EXPECT) {
            ESP_LOGW(TAG, "  CLIENT_ID 0x0260 - the gapless sequence works");
        }
    }

    /* Checkpoint 2: DAPISC.  Dump the long-form reply window first. */
    {
        dap_frame_t df;
        const uint64_t d = ((uint64_t)DAP_DAPISC_SIGNATURE << 16) | DAP_DAPISC_VALUE;
        if (dap_frame_build(&df, DAP_CMD_DAPISC, 48, d, 48)) {
            uint8_t raw[200];
            char    text[208];
            dap_phy_write_frame(&df);
            dap_phy_turnaround_to_read();
            dap_phy_read_bits(raw, sizeof(raw));
            dap_phy_turnaround_to_write();
            size_t n = 0;
            for (size_t i = 0; i < sizeof(raw) && n < sizeof(text) - 1; i++) {
                text[n++] = raw[i] ? '1' : '0';
            }
            text[n] = '\0';
            ESP_LOGW(TAG, "dapisc reply window, %u bits raw:", (unsigned)sizeof(raw));
            ESP_LOGW(TAG, "  %s", text);
        }
    }
    err = dap_probe_dapisc(DAP_DAPISC_VALUE, false, &x);
    dap_probe_log_exchange("2 dapisc short", &x);
    if (err != ESP_OK) {
        /* Not Active after all?  Try the initialisation telegram. */
        err = dap_probe_dapisc(DAP_DAPISC_VALUE, true, &x);
        dap_probe_log_exchange("2 dapisc long ", &x);
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "   DAPISC now 0x%04" PRIX64 " (wrote 0x%04X)",
                 x.reply, DAP_DAPISC_VALUE);
    }

    if (err != ESP_OK) {
        /* Not counted as a failure. */
        ESP_LOGI(TAG, "   no reply, as expected for the LEN-48 write form");
    } else {
        ESP_LOGI(TAG, "   MAXWAIT8=%u MW8E=%u -> %u wait clocks allowed",
                 (unsigned)((x.reply >> 8) & 0x1Fu), (unsigned)((x.reply >> 13) & 1u),
                 (unsigned)(((x.reply >> 8) & 0x1Fu) * 8u *
                            (1u + ((x.reply >> 13) & 1u) * 15u)));
    }

    /* Checkpoint 3: select the Cerberus IOClient. */
    err = dap_probe_client_set(1, &x);
    dap_probe_log_exchange("3 client_set(1)", &x);
    if (err != ESP_OK) {
        failures++;
    }

    /* Checkpoint 4: CLIENT_ID must read 0x0260. */
    for (int attempt = 0; attempt < 3; attempt++) {
        err = dap_probe_client_read(DAP_IO_CLIENT_ID, 4, 16, &x);
        if (err == ESP_OK && x.reply == DAP_CLIENT_ID_EXPECT) {
            break;
        }
        /* Flush and retry; the first try after a board reset often fails. */
        dap_phy_idle_clocks(dap_probe_max_wait, 0);
        dap_probe_clear_error_state();
    }
    dap_probe_log_exchange("4 client_read ID", &x);
    {
        /* 32-bit form replicates the halfword: expect 0x02600260. */
        dap_exchange_t w;
        if (dap_probe_client_read(DAP_IO_CLIENT_ID, 5, 32, &w) == ESP_OK) {
            ESP_LOGI(TAG, "   32-bit CLIENT_ID reads 0x%08" PRIX64 "%s",
                     w.reply, w.reply == 0x02600260u ? " (expected)" : "");
        }
        dap_exchange_t info;
        if (dap_probe_client_read(DAP_IO_INFO, 4, 16, &info) == ESP_OK) {
            ESP_LOGI(TAG, "   IOINFO reads 0x%04" PRIX64, info.reply);
        }
    }
    if (err != ESP_OK || x.reply != DAP_CLIENT_ID_EXPECT) {
        ESP_LOGE(TAG, "   expected CLIENT_ID 0x%04X", DAP_CLIENT_ID_EXPECT);
        failures++;
    } else {
        ESP_LOGI(TAG, "   CLIENT_ID 0x0260: the probe is talking to Cerberus");
    }

    /*
     * Checkpoint 5: target memory reads at several addresses, each followed
     * by IOINFO to show bus errors.  STM0_TIM0 counts, so two reads must differ.
     */
    {
        static const struct { uint32_t addr; const char *what; } probes[] = {
            { 0xF0001010u, "STM0_TIM0 (counts up)" },
            { 0xF0001010u, "STM0_TIM0 again" },
            { 0x70000000u, "CPU0 DSPR" },
            { 0xA0000000u, "PFLASH0, non-cached alias" },
            { 0xF0000480u, "OSTATE (OCDS block)" },
        };
        uint32_t word = 0, again = 0;

        /* Required order: clear Error State, set RW mode, then address and read. */
        dap_probe_clear_error_state();

        /* IOClient registers first; OJCONF stays readable even with the bus locked. */
        static const struct { uint8_t instr; const char *what; } regs[] = {
            { 0xEu, "OJCONF (0xE)" },
            { 0xBu, "IOINFO (0xB)" },
        };
        for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
            dap_exchange_t r;
            if (dap_probe_client_read(regs[i].instr, 4, 16, &r) == ESP_OK) {
                ESP_LOGI(TAG, "   %s = 0x%04" PRIX64, regs[i].what, r.reply);
            } else {
                ESP_LOGW(TAG, "   %s no reply", regs[i].what);
            }
        }
        esp_err_t mode = dap_probe_set_rw_mode(true);
        ESP_LOGI(TAG, "   IOCONF <- RW + supervisor: %s",
                 mode == ESP_OK ? "acknowledged" : "no acknowledge");
        int reads_ok = 0;
        for (size_t i = 0; i < sizeof(probes) / sizeof(probes[0]); i++) {
            if (dap_probe_read32(probes[i].addr, &word) == ESP_OK) {
                reads_ok++;
                ESP_LOGI(TAG, "5 read 0x%08" PRIX32 " = 0x%08" PRIX32 "  %s",
                         probes[i].addr, word, probes[i].what);
            } else {
                ESP_LOGW(TAG, "5 read 0x%08" PRIX32 "   no reply       %s",
                         probes[i].addr, probes[i].what);
            }
            dap_exchange_t inf;
            if (dap_probe_client_read(DAP_IO_INFO, 4, 16, &inf) == ESP_OK) {
                ESP_LOGI(TAG, "     IOINFO 0x%04" PRIX64 "%s", inf.reply,
                         (inf.reply & ~0x0020u) ? "  <- bits beyond PWR_DWN" : "");
            }
        }
        /* A counter reading the same value twice is not being read. */
        if (dap_probe_read32(0xF0001010u, &word) == ESP_OK &&
            dap_probe_read32(0xF0001010u, &again) == ESP_OK) {
            if (word != again) {
                ESP_LOGI(TAG, "5 STM0_TIM0 advanced: 0x%08" PRIX32 " -> 0x%08" PRIX32
                              "  (+%" PRIu32 ")", word, again, again - word);
            } else {
                ESP_LOGW(TAG, "5 STM0_TIM0 read the same value twice: 0x%08" PRIX32
                              " - not a live read", word);
                failures++;
            }
        } else {
            failures++;
        }

        if (!reads_ok) {
            failures++;
        }
    }

    /* Checkpoint 6: enable OCDS; miniMCDS ID reads a constant only when it is on. */
    {
        uint32_t id = 0;
        if (dap_probe_read32(DAP_ADDR_MCDS_ID, &id) == ESP_OK) {
            ESP_LOGI(TAG, "6 miniMCDS ID before enable: 0x%08" PRIX32, id);
        } else {
            ESP_LOGI(TAG, "6 miniMCDS ID before enable: no reply (OCDS off)");
        }
        dap_probe_clear_error_state();

        if (dap_probe_enable_ocds() == ESP_OK) {
            ESP_LOGI(TAG, "6 OCDS enabled");
            uint32_t clc = 0, ct = 0;
            if (dap_probe_read32(DAP_ADDR_MCDS_CLC, &clc) == ESP_OK) {
                ESP_LOGI(TAG, "6 miniMCDS CLC = 0x%08" PRIX32 " (DISR=%" PRIu32 ")",
                         clc, clc & 1u);
            }
            if (dap_probe_read32(DAP_ADDR_MCDS_ID, &id) == ESP_OK) {
                ESP_LOGI(TAG, "6 miniMCDS ID = 0x%08" PRIX32 "%s", id,
                         id == DAP_MCDS_ID_EXPECT ? "  (expected)" : "  UNEXPECTED");
            } else {
                ESP_LOGW(TAG, "6 miniMCDS still unreadable after enable");
            }
            if (dap_probe_read32(DAP_ADDR_MCDS_CT, &ct) == ESP_OK) {
                ESP_LOGI(TAG, "6 miniMCDS CT  = 0x%08" PRIX32, ct);
            }
        } else {
            ESP_LOGW(TAG, "6 OCDS enable did not take");
        }
        dap_probe_clear_error_state();
    }

    /* Checkpoint 7: a block read, cross-checked against single-word reads. */
    {
        const uint32_t base = DAP_ADDR_MCDS_BASE;   /* readable now OCDS is on */
        uint32_t blk[8] = {0}, one[8] = {0};

        if (dap_probe_blockread(base, blk, 8) == ESP_OK) {
            int mismatch = 0;
            for (size_t i = 0; i < 8; i++) {
                if (dap_probe_read32(base + 4u * i, &one[i]) != ESP_OK ||
                    one[i] != blk[i]) {
                    mismatch++;
                }
            }
            ESP_LOGI(TAG, "7 blockread 8 words from 0x%08" PRIX32 ": %s",
                     base, mismatch ? "MISMATCH vs single reads" : "matches single reads");
            for (size_t i = 0; i < 4; i++) {
                ESP_LOGI(TAG, "   [%u] block 0x%08" PRIX32 "  single 0x%08" PRIX32,
                         (unsigned)i, blk[i], one[i]);
            }
            if (mismatch) {
                failures++;
            }
        } else {
            ESP_LOGW(TAG, "7 blockread drew no parcels");
            failures++;
        }
        dap_probe_clear_error_state();
    }

    /*
     * Checkpoint 8: trace FIFO registers and TRAM reachable by block read.
     * Tracing is not configured here, so FIFONOW is expected to be static.
     */
    {
        static const struct { uint32_t addr; const char *name; } fifo[] = {
            { DAP_ADDR_FIFONOW,    "FIFONOW  (write pointer)" },
            { DAP_ADDR_FIFOBOT,    "FIFOBOT  (buffer bottom)" },
            { DAP_ADDR_FIFOTOP,    "FIFOTOP  (buffer top)" },
            { DAP_ADDR_FIFOCTL,    "FIFOCTL" },
            { DAP_ADDR_FIFOOVRCNT, "FIFOOVRCNT" },
        };
        uint32_t v = 0;
        int fifo_ok = 0;

        ESP_LOGI(TAG, "8 trace FIFO registers:");
        for (size_t i = 0; i < sizeof(fifo) / sizeof(fifo[0]); i++) {
            if (dap_probe_read32(fifo[i].addr, &v) == ESP_OK) {
                ESP_LOGI(TAG, "   %-26s = 0x%08" PRIX32, fifo[i].name, v);
                fifo_ok++;
            } else {
                ESP_LOGW(TAG, "   %-26s   no reply", fifo[i].name);
            }
        }

        /* And the buffer behind it, one paragraph at a time. */
        static uint32_t para[DAP_TRAM_PARAGRAPH / 4];
        if (dap_probe_blockread(DAP_ADDR_TRAM_BASE, para, 256) == ESP_OK) {
            /* Configuration seeds five 0xFFFFFFFF words (<endoftrace>): empty buffer. */
            int ones = 0;
            for (int i = 0; i < 5; i++) {
                ones += (para[i] == 0xFFFFFFFFu);
            }
            ESP_LOGI(TAG, "8 TRAM 0x%08" PRIX32 ": %08" PRIX32 " %08" PRIX32
                          " %08" PRIX32 " %08" PRIX32 " (%d of the first 5 words "
                          "are all-ones%s)",
                     DAP_ADDR_TRAM_BASE, para[0], para[1], para[2], para[3], ones,
                     ones == 5 ? ", so it is seeded and empty" : "");
        } else {
            ESP_LOGW(TAG, "8 TRAM block read drew no parcels");
            failures++;
        }
        if (fifo_ok == 0) {
            failures++;
        }
        dap_probe_clear_error_state();
    }

    if (!failures) {
        dap_probe_rate_test();
        dap_probe_block_throughput();
    }

    ESP_LOGI(TAG, "=== bring-up %s (%d failure%s) ===",
             failures ? "INCOMPLETE" : "PASSED", failures, failures == 1 ? "" : "s");
    return failures ? ESP_FAIL : ESP_OK;
}
