/*
 * miniMCDS configuration for data and watch-point trace (TC3xx TS ch. 9).
 *
 * Slot j uses DTU comparator set j: TCXEA* (address), TCXAC* (access pulse,
 * one cycle per transaction), TCXWD* (value).  They are ANDed in MCXEVT8
 * (slot 0) or MCXEVT10 (slot 1), the event columns of Table 394 that carry
 * all six DTU triggers, and the events drive the MCX actions.
 */

#include "dap_mcds.h"

#include <inttypes.h>
#include <string.h>

#include "dap_lock.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "dap_probe_priv.h"
#include "dap_trace.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DAP_MCDS";

#define MCDS(off)           (DAP_ADDR_MCDS_BASE + (off))
#define REG_CT              MCDS(0x0010u)
#define REG_MUX             MCDS(0x0014u)
#define REG_MUX_TC_RC       MCDS(0x0020u)
#define REG_FIFOPRE         MCDS(0x0208u)
#define REG_TSUEMUCNT       MCDS(0x0408u)
#define REG_MCXEVT(x)       MCDS(0x0800u + (x) * 4u)
#define REG_MCXACT(x)       MCDS(0x0880u + (x) * 4u)
#define REG_TCXEABND(j)     MCDS(0x2400u + (j) * 0x10u)
#define REG_TCXEARNG(j)     MCDS(0x2404u + (j) * 0x10u)
#define REG_TCXWDBND(j)     MCDS(0x2480u + (j) * 0x20u)
#define REG_TCXWDRNG(j)     MCDS(0x2488u + (j) * 0x20u)
#define REG_TCXWDMSK(j)     MCDS(0x2490u + (j) * 0x20u)
#define REG_TCXWDSGN(j)     MCDS(0x249Cu + (j) * 0x20u)
#define REG_TCXACBND(j)     MCDS(0x2500u + (j) * 0x18u)
#define REG_TCXACRNG(j)     MCDS(0x2504u + (j) * 0x18u)
#define REG_TCXACMSK(j)     MCDS(0x2508u + (j) * 0x18u)

#define CT_SETE             0x00008000u
#define CT_SETR             0x80000000u     /* request an MCDS reset */
#define CT_RES              (1u << 29)

#define FIFOCTL_FFE         (1u << 1)
#define FIFOCTL_TRON        0x00000400u
#define FIFOCTL_TROFF       0x00000800u
#define FIFOCTL_CLR         0x00004000u     /* clear Flush: start */
#define FIFOCTL_SET         0x00008000u     /* set Flush: stop */

#define TRAM_BASE           0xB8000000u
#define TRAM_BYTES          0x2000u

/* TCXACMSK: SVM 0, MASTER 4:1, SUBCHANNEL 11:5, WR 12, RD 13, MSK 14. */
#define AC_WR               (1u << 12)
#define AC_RD               (1u << 13)
#define AC_MSK              (1u << 14)

/* MCXEVT column per slot, and the EIQ bit of each DTU trigger in it. */
static const struct {
    uint8_t evt, ea, dat, acc;
} k_slot_evt[DAP_MCDS_SLOTS] = {
    { 8,  2, 4, 6 },    /* ea0, dat0, acc0 */
    { 10, 3, 5, 7 },    /* ea1, dat1, acc1 */
};

/* MCX actions (Table 398). */
#define ACT_TSU_REL_EN      0
#define ACT_TSU_REL_SYNC    1
#define ACT_WTU_ENABLE(n)   (5 + (n))
#define ACT_TICK_ENABLE     21
#define ACT_DTU_WDAT        27
#define ACT_DTU_WADR        28
#define ACT_DTU_RDAT        29
#define ACT_DTU_RADR        30
#define ACT_COUNT           42

/* One MCXACT input slot: AIS 4:0, AIQ 6:5 (10 direct, 11 constant 1), LV 7. */
#define AIN_LEVEL(ev)       (0xC0u | (ev))
#define AIN_EDGE(ev)        (0x40u | (ev))
#define AIN_ALWAYS          0xE0u
#define EVENT_SYNC_RQ       20u              /* pulses at each new paragraph */

static bool s_running;

void dap_mcds_default_config(dap_mcds_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->source     = DAP_MCDS_SRC_CPU;
    cfg->mode       = DAP_MCDS_MODE_FULL;
    cfg->payload    = DAP_MCDS_PAYLOAD_ADDR_DATA;
    cfg->timestamps = DAP_MCDS_TS_HIT;
    cfg->dap_div    = 0;                     /* 24 MHz */
    for (int j = 0; j < DAP_MCDS_SLOTS; j++) {
        cfg->slot[j].size = 4;
        cfg->slot[j].wr = true;
        cfg->slot[j].value_mask = 0xFFFFFFFFu;
        cfg->slot[j].value_hi = 0xFFFFFFFFu;
    }
}

bool dap_mcds_running(void)
{
    return s_running;
}

/* Collects writes so one failure aborts the rest with a clear log line. */
typedef struct {
    esp_err_t err;
} wr_t;

static void wr(wr_t *w, uint32_t addr, uint32_t value)
{
    if (w->err == ESP_OK) {
        w->err = dap_probe_write32(addr, value);
        if (w->err != ESP_OK) {
            ESP_LOGE(TAG, "write 0x%08" PRIX32 " = 0x%08" PRIX32 " failed", addr, value);
        }
    }
}

/* Append an input to an MCXACT value (up to four, ORed). */
static uint32_t act_add(uint32_t act, uint32_t input)
{
    for (int q = 0; q < 4; q++) {
        if (((act >> (8 * q)) & 0xFFu) == 0u) {
            return act | (input << (8 * q));
        }
    }
    return act;
}

static uint32_t evt_connect(uint32_t evt, uint8_t bit)
{
    return (evt & ~(3u << (2 * bit))) | (2u << (2 * bit));    /* EIQ = 10B */
}

/* Value filter in the comparator's byte lanes (Table 371): data narrower
 * than a word is replicated, so the variable's lane is picked by address. */
static void program_value(wr_t *w, int j, const dap_mcds_slot_t *s)
{
    if (!s->value_en) {
        return;
    }
    const uint32_t width = (s->size >= 4) ? 32u : s->size * 8u;
    const uint32_t shift = (s->size >= 4) ? 0u : 8u * (s->addr & 3u);
    const uint32_t full  = (width == 32u) ? 0xFFFFFFFFu : ((1u << width) - 1u);
    const uint32_t mask  = s->value_mask & full;
    const uint32_t lo    = s->value_lo & mask;
    const uint32_t hi    = s->value_hi & mask;

    wr(w, REG_TCXWDMSK(j), mask << shift);
    wr(w, REG_TCXWDBND(j), lo << shift);
    wr(w, REG_TCXWDRNG(j), (uint32_t)((hi - lo) & full) << shift);
    wr(w, REG_TCXWDSGN(j), s->value_signed ? (shift + width - 1u) : 0u);
}

static esp_err_t mcds_reset(void)
{
    uint32_t ct = 0;

    dap_probe_write32(REG_CT, CT_SETR);
    for (int i = 0; i < 20; i++) {
        if (dap_probe_read32(REG_CT, &ct) == ESP_OK && !(ct & CT_RES)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return dap_probe_write32(REG_CT, CT_SETE);
}

static uint32_t measure_emu_hz(void)
{
    uint32_t a = 0, b = 0;

    const int64_t t0 = esp_timer_get_time();
    dap_probe_read32(REG_TSUEMUCNT, &a);
    vTaskDelay(pdMS_TO_TICKS(50));
    dap_probe_read32(REG_TSUEMUCNT, &b);
    const int64_t us = esp_timer_get_time() - t0;
    return us > 0 ? (uint32_t)((uint64_t)(b - a) * 1000000u / (uint64_t)us) : 0u;
}

/* ECC-initialise the TRAM and put <endoftrace> at every paragraph start. */
static esp_err_t tram_init(void)
{
    static uint32_t ones[DAP_FPGA_BLOCK_WORDS];

    for (size_t i = 0; i < DAP_FPGA_BLOCK_WORDS; i++) {
        ones[i] = 0xFFFFFFFFu;
    }
    for (uint32_t off = 0; off < TRAM_BYTES; off += sizeof(ones)) {
        esp_err_t err = ESP_FAIL;
        if (dap_phy_fpga_in_use()) {
            err = dap_phy_fpga_block_write(TRAM_BASE + off, ones, DAP_FPGA_BLOCK_WORDS);
        }
        if (err != ESP_OK) {
            for (uint32_t k = 0; k < sizeof(ones); k += 4) {
                if ((err = dap_probe_write32(TRAM_BASE + off + k, 0xFFFFFFFFu)) != ESP_OK) {
                    return err;
                }
            }
        }
    }
    /* Block writes can drop their first word; rewrite each paragraph head. */
    dap_probe_clear_error_state();
    for (uint32_t off = 0; off < TRAM_BYTES; off += 0x400u) {
        dap_probe_write32(TRAM_BASE + off, 0xFFFFFFFFu);
    }
    uint32_t check = 0;
    if (dap_probe_read32(TRAM_BASE + 0x404u, &check) != ESP_OK || check != 0xFFFFFFFFu) {
        ESP_LOGE(TAG, "TRAM init did not stick (0x%08" PRIX32 ")", check);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t configure(const dap_mcds_config_t *cfg)
{
    wr_t w = { ESP_OK };
    uint32_t act[ACT_COUNT] = { 0 };
    uint32_t hit_inputs[DAP_MCDS_SLOTS];
    int      nhits = 0;

    /* Clean slate: MCDS reset clears every MCX/TCX/FIFO register. */
    if (mcds_reset() != ESP_OK) {
        return ESP_FAIL;
    }
    wr(&w, DAP_ADDR_FIFOCTL, FIFOCTL_SET);
    wr(&w, DAP_ADDR_FIFOCTL, FIFOCTL_TROFF);

    /* Source: MUX_TC_RC before MUX. */
    const uint32_t cpu = cfg->cpu % 6u;
    if (cfg->source == DAP_MCDS_SRC_LMU0) {
        wr(&w, REG_MUX, 0x80u | 0x9u);
    } else {
        const uint32_t tc = (cfg->source == DAP_MCDS_SRC_MEMSLAVE) ? 2u : 0u;
        wr(&w, REG_MUX_TC_RC, 0x8000u | (tc << (2u * cpu)));
        wr(&w, REG_MUX, 0x80u | (cpu + 1u));
    }

    /* Actions. */
    for (int j = 0; j < DAP_MCDS_SLOTS; j++) {
        const dap_mcds_slot_t *s = &cfg->slot[j];
        if (!s->enabled || !(s->rd || s->wr)) {
            continue;
        }
        const uint8_t ev = k_slot_evt[j].evt;
        hit_inputs[nhits++] = AIN_EDGE(ev);

        if (cfg->mode == DAP_MCDS_MODE_COMPACT) {
            act[ACT_WTU_ENABLE(j)] = AIN_EDGE(ev);
            continue;
        }
        const bool data = cfg->payload != DAP_MCDS_PAYLOAD_ADDR;
        const bool addr = cfg->payload != DAP_MCDS_PAYLOAD_DATA;
        if (s->wr && data) act[ACT_DTU_WDAT] = act_add(act[ACT_DTU_WDAT], AIN_LEVEL(ev));
        if (s->wr && addr) act[ACT_DTU_WADR] = act_add(act[ACT_DTU_WADR], AIN_LEVEL(ev));
        if (s->rd && data) act[ACT_DTU_RDAT] = act_add(act[ACT_DTU_RDAT], AIN_LEVEL(ev));
        if (s->rd && addr) act[ACT_DTU_RADR] = act_add(act[ACT_DTU_RADR], AIN_LEVEL(ev));
    }
    if (nhits == 0) {
        ESP_LOGE(TAG, "no enabled slot");
        return ESP_ERR_INVALID_ARG;
    }

    /* Time: a TSR at every paragraph start keeps each paragraph decodable on
     * its own; per hit on request. */
    act[ACT_TSU_REL_EN]   = AIN_ALWAYS;
    act[ACT_TSU_REL_SYNC] = AIN_EDGE(EVENT_SYNC_RQ);
    if (cfg->timestamps == DAP_MCDS_TS_HIT) {
        for (int i = 0; i < nhits; i++) {
            act[ACT_TSU_REL_SYNC] = act_add(act[ACT_TSU_REL_SYNC], hit_inputs[i]);
        }
    }
    act[ACT_TICK_ENABLE] = (cfg->timestamps == DAP_MCDS_TS_TICKS) ? AIN_ALWAYS : 0u;

    for (int a = 0; a < ACT_COUNT; a++) {
        wr(&w, REG_MCXACT(a), act[a]);
    }

    /* Events: EA and AC always, WD when filtering on the value. */
    for (int j = 0; j < DAP_MCDS_SLOTS; j++) {
        const dap_mcds_slot_t *s = &cfg->slot[j];
        uint32_t evt = 0;                              /* never */
        if (s->enabled && (s->rd || s->wr)) {
            evt = 0xFFFFFFFFu;
            evt = evt_connect(evt, k_slot_evt[j].ea);
            evt = evt_connect(evt, k_slot_evt[j].acc);
            if (s->value_en) {
                evt = evt_connect(evt, k_slot_evt[j].dat);
            }
        }
        wr(&w, REG_MCXEVT(k_slot_evt[j].evt), evt);
    }

    /* Comparators.  AC pattern bit 12 = write, 13 = read (one cycle each). */
    for (int j = 0; j < DAP_MCDS_SLOTS; j++) {
        const dap_mcds_slot_t *s = &cfg->slot[j];
        const uint32_t size = s->size ? s->size : 1u;

        wr(&w, REG_TCXEABND(j), s->addr);
        wr(&w, REG_TCXEARNG(j), size - 1u);

        const uint32_t acmsk = (s->wr ? AC_WR : 0u) | (s->rd ? AC_RD : 0u) |
                               (cfg->masters ? 0u : AC_MSK);
        const uint32_t lo = s->wr ? AC_WR : AC_RD;
        const uint32_t hi = s->rd ? AC_RD : AC_WR;
        wr(&w, REG_TCXACMSK(j), acmsk);
        wr(&w, REG_TCXACBND(j), lo);
        wr(&w, REG_TCXACRNG(j), hi - lo);
        program_value(&w, j, s);
    }

    /* Buffer: the whole 8 kB as a ring (PRE = TOP - BOTTOM, no trace_done). */
    wr(&w, DAP_ADDR_FIFOBOT, 0u);
    wr(&w, DAP_ADDR_FIFOTOP, TRAM_BYTES - 1u);
    wr(&w, REG_FIFOPRE, (TRAM_BYTES - 1u) & 0x1FE0u);
    wr(&w, DAP_ADDR_FIFOWARN0, 0u);
    wr(&w, DAP_ADDR_FIFOWARN1, 0u);
    wr(&w, DAP_ADDR_FIFOOVRCNT, 1u << 15);
    return w.err;
}

esp_err_t dap_mcds_start(const dap_mcds_config_t *cfg, dap_mcds_info_t *info)
{
    esp_err_t err;

    if (s_running) {
        dap_mcds_stop();
    }
    dap_lock();
    if (dap_phy_fpga_in_use()) {
        dap_phy_fpga_set_div(cfg->dap_div);
    }
    err = dap_probe_enable_ocds();
    if (err == ESP_OK) {
        err = configure(cfg);
    }
    if (err == ESP_OK) {
        err = tram_init();
    }
    if (err == ESP_OK) {
        info->emu_hz = measure_emu_hz();
        dap_probe_read32(REG_TSUEMUCNT, &info->tsu_start);
        dap_probe_write32(DAP_ADDR_FIFOCTL, FIFOCTL_TRON);
        err = dap_probe_write32(DAP_ADDR_FIFOCTL, FIFOCTL_CLR);
    }
    if (err == ESP_OK) {
        err = dap_trace_start();
    }
    dap_unlock();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "trace setup failed: %s", esp_err_to_name(err));
        return err;
    }
    s_running = true;
    ESP_LOGI(TAG, "tracing: mode %s, emulation clock %" PRIu32 " Hz",
             cfg->mode == DAP_MCDS_MODE_COMPACT ? "compact" : "full", info->emu_hz);
    return ESP_OK;
}

esp_err_t dap_mcds_stop(void)
{
    uint32_t ctl = 0;

    /* Flush first (tracing stops, the drain keeps reading), then stop the
     * drain, then collect the rest. */
    dap_probe_write32(DAP_ADDR_FIFOCTL, FIFOCTL_SET);
    for (int i = 0; i < 50; i++) {
        if (dap_probe_read32(DAP_ADDR_FIFOCTL, &ctl) == ESP_OK && (ctl & FIFOCTL_FFE)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    dap_trace_stop();

    dap_lock();
    dap_trace_finish();
    mcds_reset();                        /* no trace, no heartbeat left armed */
    if (dap_phy_fpga_in_use()) {
        dap_phy_fpga_set_div(5);         /* the attach default */
    }
    dap_unlock();
    s_running = false;
    ESP_LOGI(TAG, "tracing stopped");
    return ESP_OK;
}
