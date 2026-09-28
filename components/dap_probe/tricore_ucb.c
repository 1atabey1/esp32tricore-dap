/*
 * TC3xx user configuration blocks, written from an image.  A port of
 * tas-debug's ucb.py (catalogue, formal checks, state) and ucb_program.py
 * (plan, writer); keep the two in step.
 *
 * This is the one write path that can leave a board that does not boot, so
 * the reference's rules matter more than its sequences:
 *   - only the eight boot mode headers are written; every other block the
 *     image carries is listed and skipped (SWAP is runtime state, OTP, DBG,
 *     HSM and friends install protection no debugger takes back);
 *   - a block is written only while DMU_HF_CONFIRMx reports it UNREAD or
 *     UNLOCKED, only when the image's bytes pass the formal check, and never
 *     when they would confirm it;
 *   - the page holding the confirmation word goes in last, so an interrupted
 *     write leaves a block the boot ROM skips rather than one that claims to
 *     be valid.
 * The command sequences are the MCAL flash loader's, as the reference uses
 * them: data flash page mode (0x5D), 8-byte pages as two 32-bit loads, and a
 * one-sector erase.  The command addresses are not swap-translated - the
 * swap exchanges program flash bank groups and has no bearing on data flash.
 */

#include "tricore_ucb.h"
#include "tricore_flash_priv.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "dap_probe.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "tricore.h"

static const char *TAG = "TRICORE_UCB";

/* -- the catalogue (tc3xx_ucb.json, generated from Lauterbach's tc3xx-ucb.cmm) */

typedef struct {
    const char *name;
    uint32_t    base;
    bool        dual;
    uint8_t     confirm_register;
    uint32_t    confirm_mask;
    bool        is_bmhd;
} ucb_block_t;

static const ucb_block_t BLOCKS[] = {
    { "BMHD0_ORIG",    0xAF400000u, true,  0, 0x00000003u, true  },
    { "BMHD1_ORIG",    0xAF400200u, true,  0, 0x0000000Cu, true  },
    { "BMHD2_ORIG",    0xAF400400u, true,  0, 0x00000030u, true  },
    { "BMHD3_ORIG",    0xAF400600u, true,  0, 0x000000C0u, true  },
    { "SSW",           0xAF400800u, false, 0, 0x00000100u, false },
    { "BMHD0_COPY",    0xAF401000u, true,  0, 0x00030000u, true  },
    { "BMHD1_COPY",    0xAF401200u, true,  0, 0x000C0000u, true  },
    { "BMHD2_COPY",    0xAF401400u, true,  0, 0x00300000u, true  },
    { "BMHD3_COPY",    0xAF401600u, true,  0, 0x00C00000u, true  },
    { "RETEST",        0xAF401E00u, false, 0, 0xC0000000u, false },
    { "PFLASH_ORIG",   0xAF402000u, true,  1, 0x00000003u, false },
    { "DFLASH_ORIG",   0xAF402200u, true,  1, 0x0000000Cu, false },
    { "DBG_ORIG",      0xAF402400u, true,  1, 0x00000030u, false },
    { "HSM_ORIG",      0xAF402600u, true,  1, 0x000000C0u, false },
    { "HSMCOTP0_ORIG", 0xAF402800u, true,  1, 0x00000300u, false },
    { "HSMCOTP1_ORIG", 0xAF402A00u, true,  1, 0x00000C00u, false },
    { "ECPRIO_ORIG",   0xAF402C00u, true,  1, 0x00003000u, false },
    { "SWAP_ORIG",     0xAF402E00u, true,  1, 0x0000C000u, false },
    { "PFLASH_COPY",   0xAF403000u, true,  1, 0x00030000u, false },
    { "DFLASH_COPY",   0xAF403200u, true,  1, 0x000C0000u, false },
    { "DBG_COPY",      0xAF403400u, true,  1, 0x00300000u, false },
    { "HSM_COPY",      0xAF403600u, true,  1, 0x00C00000u, false },
    { "HSMCOTP0_COPY", 0xAF403800u, true,  1, 0x03000000u, false },
    { "HSMCOTP1_COPY", 0xAF403A00u, true,  1, 0x0C000000u, false },
    { "ECPRIO_COPY",   0xAF403C00u, true,  1, 0x30000000u, false },
    { "SWAP_COPY",     0xAF403E00u, true,  1, 0xC0000000u, false },
    { "OTP0_ORIG",     0xAF404000u, true,  2, 0x00000003u, false },
    { "OTP1_ORIG",     0xAF404200u, true,  2, 0x0000000Cu, false },
    { "OTP2_ORIG",     0xAF404400u, true,  2, 0x00000030u, false },
    { "OTP3_ORIG",     0xAF404600u, true,  2, 0x000000C0u, false },
    { "OTP4_ORIG",     0xAF404800u, true,  2, 0x00000300u, false },
    { "OTP5_ORIG",     0xAF404A00u, true,  2, 0x00000C00u, false },
    { "OTP6_ORIG",     0xAF404C00u, true,  2, 0x00003000u, false },
    { "OTP7_ORIG",     0xAF404E00u, true,  2, 0x0000C000u, false },
    { "OTP0_COPY",     0xAF405000u, true,  2, 0x00030000u, false },
    { "OTP1_COPY",     0xAF405200u, true,  2, 0x000C0000u, false },
    { "OTP2_COPY",     0xAF405400u, true,  2, 0x00300000u, false },
    { "OTP3_COPY",     0xAF405600u, true,  2, 0x00C00000u, false },
    { "OTP4_COPY",     0xAF405800u, true,  2, 0x03000000u, false },
    { "OTP5_COPY",     0xAF405A00u, true,  2, 0x0C000000u, false },
    { "OTP6_COPY",     0xAF405C00u, true,  2, 0x30000000u, false },
    { "OTP7_COPY",     0xAF405E00u, true,  2, 0xC0000000u, false },
};
#define BLOCK_COUNT (sizeof(BLOCKS) / sizeof(BLOCKS[0]))

#define CONFIRM_BASE     0xF8040020u         /* DMU_HF_CONFIRM0, stride 4 */
#define CODE_UNLOCKED    0x43211234u
#define CODE_CONFIRMED   0x57B5327Fu
#define STATE_OFFSET     0x1F0u              /* the confirmation word; its copy at 0x1F8 */
#define ERASED_BYTE      0x00u               /* data flash erases to zero */

#define BMHD_ID          0xB359u

/* -- the writer's sequences (ucb_program.py) -------------------------------- */

#define CYCLE_55F4             0x55F4u
#define CMD_ENTER_PAGE_DFLASH  0x5Du   /* 0x50 would report page mode on the wrong bank */
#define CMD_WRITE_PAGE         0xA0u
#define CMD_PROGRAM            0xAAu
#define STATUS_DFPAGE          (1u << 20)
#define UCB_PAGE_SIZE          8u
#define ERASE_TIMEOUT_MS       5000u
#define PROGRAM_TIMEOUT_MS     10000u

/* The blocks an image owns, in the order they are considered. */
static const char *const IMAGE_OWNED[] = {
    "BMHD0_ORIG", "BMHD1_ORIG", "BMHD2_ORIG", "BMHD3_ORIG",
    "BMHD0_COPY", "BMHD1_COPY", "BMHD2_COPY", "BMHD3_COPY",
};
#define OWNED_COUNT (sizeof(IMAGE_OWNED) / sizeof(IMAGE_OWNED[0]))

/* Why the rest are not, by name prefix, in the reference's words. */
static const struct { const char *prefix, *reason; } NOT_IMAGE_OWNED[] = {
    { "SWAP",   "the bootloader maintains it at runtime -- it records which bank was "
                "swapped in, and resetting it changes which bank the part boots" },
    { "OTP",    "it installs one-time protection, which no debugger takes back" },
    { "HSM",    "it belongs to the HSM and can lock the part out of further programming" },
    { "DBG",    "it configures the debug interface and can lock the probe out" },
    { "PFLASH", "it installs flash write protection" },
    { "DFLASH", "it installs flash write protection" },
    { "ECPRIO", "it configures error correction priorities" },
    { "RETEST", "it is Infineon's" },
    { "SSW",    "it is Infineon's" },
};

/* -- the report ------------------------------------------------------------ */

EXT_RAM_BSS_ATTR static char s_report[4096];
static size_t s_report_len;

static void report(const char *fmt, ...)
{
    va_list ap;

    if (s_report_len >= sizeof(s_report) - 1) {
        return;
    }
    va_start(ap, fmt);
    const int n = vsnprintf(s_report + s_report_len, sizeof(s_report) - s_report_len, fmt, ap);
    va_end(ap);
    if (n > 0) {
        s_report_len += (size_t)n;
        if (s_report_len > sizeof(s_report) - 1) {
            s_report_len = sizeof(s_report) - 1;
        }
    }
}

const char *tricore_ucb_report(void)
{
    return s_report;
}

const char *tricore_ucb_slot_name(uint32_t slot)
{
    for (size_t i = 0; i < BLOCK_COUNT; i++) {
        if (BLOCKS[i].base == TRICORE_UCB_BASE + slot * TRICORE_UCB_SIZE) {
            return BLOCKS[i].name;
        }
    }
    return NULL;
}

static uint32_t slot_of(const ucb_block_t *block)
{
    return (block->base - TRICORE_UCB_BASE) / TRICORE_UCB_SIZE;
}

static const ucb_block_t *by_name(const char *name)
{
    for (size_t i = 0; i < BLOCK_COUNT; i++) {
        if (strcmp(BLOCKS[i].name, name) == 0) {
            return &BLOCKS[i];
        }
    }
    return NULL;
}

static bool is_owned(const ucb_block_t *block)
{
    for (size_t i = 0; i < OWNED_COUNT; i++) {
        if (strcmp(IMAGE_OWNED[i], block->name) == 0) {
            return true;
        }
    }
    return false;
}

static const char *not_owned_reason(const char *name)
{
    for (size_t i = 0; i < sizeof(NOT_IMAGE_OWNED) / sizeof(NOT_IMAGE_OWNED[0]); i++) {
        if (strncmp(name, NOT_IMAGE_OWNED[i].prefix, strlen(NOT_IMAGE_OWNED[i].prefix)) == 0) {
            return NOT_IMAGE_OWNED[i].reason;
        }
    }
    return "it is not a boot mode header";
}

/* -- the checks (ucb.py) ----------------------------------------------------- */

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

/* The confirmation words' offsets: a dual block carries one, a single block
 * its own and its copy's. */
static size_t code_offsets(const ucb_block_t *block, uint32_t out[2])
{
    out[0] = 0x1F0u;
    out[1] = 0x1F8u;
    return block->dual ? 1u : 2u;
}

/* CRC32 over the first eight bytes, each 32-bit word byte-swapped first, as
 * `Data.SUM ... /Long /ByteSWAP /CRC32` computes it. */
static uint32_t crc_bmhd(const uint8_t *blob)
{
    uint8_t swapped[8];

    for (int w = 0; w < 2; w++) {
        for (int b = 0; b < 4; b++) {
            swapped[4 * w + b] = blob[4 * w + 3 - b];
        }
    }
    return tricore_flash_crc32(swapped, sizeof(swapped));
}

typedef struct {
    char   text[640];
    size_t len;
    int    count;
} problems_t;

static void problem(problems_t *p, const char *fmt, ...)
{
    va_list ap;

    if (p->count++ && p->len < sizeof(p->text) - 2) {
        p->len += (size_t)snprintf(p->text + p->len, sizeof(p->text) - p->len, "; ");
    }
    if (p->len >= sizeof(p->text) - 1) {
        return;
    }
    va_start(ap, fmt);
    const int n = vsnprintf(p->text + p->len, sizeof(p->text) - p->len, fmt, ap);
    va_end(ap);
    if (n > 0) {
        p->len += (size_t)n;
        if (p->len > sizeof(p->text) - 1) {
            p->len = sizeof(p->text) - 1;
        }
    }
}

/* Everything formally wrong with a block's contents; p->count 0 when it passes. */
static void check(const ucb_block_t *block, const uint8_t *blob, problems_t *p)
{
    memset(p, 0, sizeof(*p));

    bool erased = true;
    for (uint32_t i = 0; i < TRICORE_UCB_SIZE && erased; i++) {
        erased = (blob[i] == ERASED_BYTE);
    }
    if (erased) {
        problem(p, "erased (all 0x%02X)", ERASED_BYTE);
        return;
    }

    uint32_t offsets[2];
    const size_t n = code_offsets(block, offsets);
    for (size_t i = 0; i < n; i++) {
        const uint32_t word = le32(blob + offsets[i]);
        if (word != CODE_UNLOCKED && word != CODE_CONFIRMED) {
            problem(p, "confirmation word at +0x%03" PRIX32 " is 0x%08" PRIX32 ", neither "
                       "unlocked (0x%08" PRIX32 ") nor confirmed (0x%08" PRIX32 ")",
                    offsets[i], word, CODE_UNLOCKED, CODE_CONFIRMED);
        }
    }
    if (!block->is_bmhd) {
        return;
    }

    const uint16_t id = le16(blob + 2);
    if (id != BMHD_ID) {
        problem(p, "BMHDID is 0x%04X, not 0x%04X", id, BMHD_ID);
    }
    const unsigned hwcfg = (le16(blob + 0) >> 1) & 0x7u;
    if (hwcfg != 3 && hwcfg != 4 && hwcfg != 6 && hwcfg != 7) {
        problem(p, "BMI.HWCFG is %u; only 3, 4, 6, 7 are valid", hwcfg);
    }
    const uint32_t stad = le32(blob + 4);
    if (stad % 4u) {
        problem(p, "STAD 0x%08" PRIX32 " is not word aligned", stad);
    } else if ((stad & 0xDF000000u) != 0x80000000u) {
        /* the vendor script's own test: both program flash windows pass it */
        problem(p, "STAD 0x%08" PRIX32 " is outside program flash "
                   "(0x80000000--0x80FFFFFF or 0xA0000000--0xA0FFFFFF)", stad);
    }
    const uint32_t expected = crc_bmhd(blob);
    const uint32_t crc = le32(blob + 8), complement = le32(blob + 12);
    if (crc != expected) {
        problem(p, "CRCBMHD is 0x%08" PRIX32 ", computed 0x%08" PRIX32, crc, expected);
    }
    if (complement != ~expected) {
        problem(p, "CRCBMHD_N is 0x%08" PRIX32 ", expected 0x%08" PRIX32, complement,
                ~expected);
    }
}

/* What DMU_HF_CONFIRMx says the block's state is. */
static void state_on_target(const ucb_block_t *block, char *out, size_t size)
{
    static const char *const STATES[] = { "UNREAD", "UNLOCKED", "CONFIRMED", "ERRORED" };
    uint32_t word = 0;

    if (dap_probe_read32(CONFIRM_BASE + 4u * block->confirm_register, &word) != ESP_OK) {
        dap_probe_clear_error_state();
        snprintf(out, size, "unreadable");
        return;
    }
    if (__builtin_popcount(block->confirm_mask) != 2) {
        /* SSW: a single-bit flag, which the vendor's four-state table has no answer for */
        snprintf(out, size, "bit %d %s", 31 - __builtin_clz(block->confirm_mask),
                 (word & block->confirm_mask) ? "set" : "clear");
        return;
    }
    unsigned shift = 0;
    for (unsigned offset = 0; offset < 32; offset += 2) {
        if (((block->confirm_mask >> offset) & 0x3u) == 0x3u) {
            shift = offset;
            break;
        }
    }
    snprintf(out, size, "%s", STATES[(word & block->confirm_mask) >> shift & 0x3u]);
}

static esp_err_t read_block(const ucb_block_t *block, uint8_t *out)
{
    return tricore_read_mem(block->base, out, TRICORE_UCB_SIZE);
}

/* Differing bytes between two versions of a block, and the first of them. */
static uint32_t differences(const uint8_t *image, const uint8_t *target, uint32_t *first)
{
    uint32_t n = 0;

    for (uint32_t i = TRICORE_UCB_SIZE; i-- > 0; ) {
        if (image[i] != target[i]) {
            n++;
            *first = i;
        }
    }
    return n;
}

/* -- the plan (ucb_program.plan) ---------------------------------------------- */

typedef enum { ACT_WRITE, ACT_SKIP, ACT_REFUSE } action_t;

typedef struct {
    const ucb_block_t *block;
    action_t           action;
    char               state[16];
    char               reason[480];
} decision_t;

EXT_RAM_BSS_ATTR static decision_t s_decisions[TRICORE_UCB_SLOTS];
static size_t     s_decision_count;

static decision_t *decide(const ucb_block_t *block, action_t action, const char *fmt, ...)
{
    decision_t *d = &s_decisions[s_decision_count++];
    va_list ap;

    memset(d, 0, sizeof(*d));
    d->block  = block;
    d->action = action;
    va_start(ap, fmt);
    vsnprintf(d->reason, sizeof(d->reason), fmt, ap);
    va_end(ap);
    return d;
}

static const uint8_t *blob_of(const tricore_ucb_image_t *image, const ucb_block_t *block)
{
    return image->data + (block->base - TRICORE_UCB_BASE);
}

static void plan_block(const tricore_ucb_image_t *image, const ucb_block_t *block)
{
    static uint8_t on_target[TRICORE_UCB_SIZE];
    const uint8_t *blob = blob_of(image, block);
    problems_t p;

    if (!is_owned(block)) {
        decide(block, ACT_SKIP, "%s", not_owned_reason(block->name));
        return;
    }
    check(block, blob, &p);
    if (p.count) {
        decide(block, ACT_REFUSE, "%s", p.text);
        return;
    }

    uint32_t offsets[2];
    const size_t n = code_offsets(block, offsets);
    char confirmed_at[24] = "";
    for (size_t i = 0; i < n; i++) {
        if (le32(blob + offsets[i]) == CODE_CONFIRMED) {
            const size_t len = strlen(confirmed_at);
            snprintf(confirmed_at + len, sizeof(confirmed_at) - len, "%s+0x%03" PRIX32,
                     len ? ", " : "", offsets[i]);
        }
    }
    if (confirmed_at[0]) {
        decide(block, ACT_REFUSE, "the image would confirm the block (%s holds 0x%08" PRIX32
                                  "), and this tool has no unlock path to undo that",
               confirmed_at, CODE_CONFIRMED);
        return;
    }

    char state[16];
    state_on_target(block, state, sizeof(state));
    if (strcmp(state, "UNREAD") != 0 && strcmp(state, "UNLOCKED") != 0) {
        decision_t *d = decide(block, ACT_REFUSE, "the device reports %s; only UNREAD or "
                                                  "UNLOCKED can be written without the "
                                                  "block's password", state);
        snprintf(d->state, sizeof(d->state), "%s", state);
        return;
    }

    decision_t *d;
    uint32_t first = 0;
    if (read_block(block, on_target) != ESP_OK) {
        d = decide(block, ACT_REFUSE, "the device will not let us read the block (bus "
                                      "error), so a write could not be verified");
    } else {
        const uint32_t differing = differences(blob, on_target, &first);
        d = differing ? decide(block, ACT_WRITE, "differs from the image in %" PRIu32 " bytes",
                               differing)
                      : decide(block, ACT_SKIP, "already holds the image's bytes");
    }
    snprintf(d->state, sizeof(d->state), "%s", state);
}

/* Every block the image carries gets a line: the headers, then the rest. */
static void plan(const tricore_ucb_image_t *image)
{
    s_decision_count = 0;
    for (size_t i = 0; i < OWNED_COUNT; i++) {
        const ucb_block_t *block = by_name(IMAGE_OWNED[i]);
        if (image->present & (1ull << slot_of(block))) {
            plan_block(image, block);
        }
    }
    for (size_t i = 0; i < BLOCK_COUNT; i++) {
        if (!is_owned(&BLOCKS[i]) && (image->present & (1ull << slot_of(&BLOCKS[i])))) {
            plan_block(image, &BLOCKS[i]);
        }
    }
}

/* -- the writer (ucb_program.UcbWriter) ------------------------------------- */

EXT_RAM_BSS_ATTR static char s_why[480];

static esp_err_t why(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(s_why, sizeof(s_why), fmt, ap);
    va_end(ap);
    return ESP_FAIL;
}

static esp_err_t cycle(uint32_t offset, uint32_t value)
{
    return dap_probe_write32(CSI_BASE + offset, value);
}

static esp_err_t clear_status(void)
{
    return cycle(CYCLE_5554, CMD_CLEAR_STATUS);
}

static const char *error_names(uint32_t errsr)
{
    static char names[160];

    snprintf(names, sizeof(names), "%s%s%s%s%s",
             (errsr & 1u) ? "OPER (operation error) " : "",
             (errsr & 2u) ? "SQER (command sequence error) " : "",
             (errsr & 4u) ? "PROER (protection error) " : "",
             (errsr & 8u) ? "PVER (program verify error) " : "",
             (errsr & 16u) ? "EVER (erase verify error) " : "");
    const size_t len = strlen(names);
    if (len) {
        names[len - 1] = '\0';
    }
    return names;
}

static esp_err_t wait_idle(const char *what, uint32_t address, uint32_t timeout_ms)
{
    tricore_flash_status.errsr = 0;
    const esp_err_t err = tricore_flash_wait_idle(address, timeout_ms);
    if (err == ESP_ERR_TIMEOUT) {
        return why("%s did not finish within %" PRIu32 " s", what, timeout_ms / 1000u);
    }
    if (err != ESP_OK) {
        return why("%s failed: %s", what, error_names(tricore_flash_status.errsr));
    }
    return ESP_OK;
}

/* Data flash page mode, confirmed before relying on it: with the program
 * flash command every page would report no error and write nothing. */
static esp_err_t check_page_mode(const ucb_block_t *block)
{
    uint32_t status = 0, errsr = 0;

    clear_status();
    if (cycle(CYCLE_5554, CMD_ENTER_PAGE_DFLASH) != ESP_OK ||
        dap_probe_read32(DMU_HF_STATUS, &status) != ESP_OK) {
        return why("data flash did not enter page mode for %s (the probe lost the target)",
                   block->name);
    }
    dap_probe_read32(DMU_HF_ERRSR, &errsr);
    clear_status();
    if (!(status & STATUS_DFPAGE)) {
        return why("data flash did not enter page mode for %s (%s)", block->name,
                   (errsr & ERRSR_FAILED) ? error_names(errsr) : "no error reported");
    }
    return ESP_OK;
}

/* Erase one block and confirm it really is erased: a refused sequence
 * latches an error but leaves the interpreter idle. */
static esp_err_t erase(const ucb_block_t *block, uint8_t *after)
{
    char what[40];

    snprintf(what, sizeof(what), "erase of %s", block->name);
    clear_status();
    if (cycle(CYCLE_AA50, block->base) != ESP_OK ||
        cycle(CYCLE_AA58, 1u) != ESP_OK ||
        cycle(CYCLE_AAA8, CMD_ERASE_SETUP) != ESP_OK ||
        cycle(CYCLE_AAA8, CMD_ERASE) != ESP_OK) {
        return why("%s: the probe lost the target", what);
    }
    if (wait_idle(what, block->base, ERASE_TIMEOUT_MS) != ESP_OK) {
        return ESP_FAIL;
    }
    clear_status();

    if (read_block(block, after) != ESP_OK) {
        return why("%s could not be read back after the erase", block->name);
    }
    uint32_t unerased = 0, first = 0;
    for (uint32_t i = TRICORE_UCB_SIZE; i-- > 0; ) {
        if (after[i] != ERASED_BYTE) {
            unerased++;
            first = i;
        }
    }
    if (unerased) {
        return why("%s is not erased after the erase command: %" PRIu32 " bytes still set, "
                   "first at +0x%03" PRIX32, block->name, unerased, first);
    }
    return ESP_OK;
}

static esp_err_t program_page(uint32_t address, const uint8_t *page)
{
    char what[40];

    snprintf(what, sizeof(what), "program of 0x%08" PRIX32, address);
    if (cycle(CYCLE_5554, CMD_CLEAR_STATUS) != ESP_OK ||
        cycle(CYCLE_5554, CMD_ENTER_PAGE_DFLASH) != ESP_OK ||
        cycle(CYCLE_55F0, le32(page)) != ESP_OK ||
        cycle(CYCLE_55F4, le32(page + 4)) != ESP_OK ||
        cycle(CYCLE_AA50, address) != ESP_OK ||
        cycle(CYCLE_AA58, 0u) != ESP_OK ||
        cycle(CYCLE_AAA8, CMD_WRITE_PAGE) != ESP_OK ||
        cycle(CYCLE_AAA8, CMD_PROGRAM) != ESP_OK) {
        return why("%s: the probe lost the target", what);
    }
    if (wait_idle(what, address, PROGRAM_TIMEOUT_MS) != ESP_OK) {
        return ESP_FAIL;
    }
    return clear_status();
}

/* Erase and program one block, then read it back and check what is there.
 * `problems` is filled when the read-back fails the formal check. */
static esp_err_t write_block(const ucb_block_t *block, const uint8_t *blob, problems_t *p)
{
    static uint8_t after[TRICORE_UCB_SIZE];
    const uint32_t state_page = STATE_OFFSET - (STATE_OFFSET % UCB_PAGE_SIZE);

    memset(p, 0, sizeof(*p));
    if (!tricore_is_halted(0)) {
        /* firmware touching flash mid-sequence turns it into a sequence error,
         * and by then the block is already erased */
        return why("the core must be halted to write %s; it is running", block->name);
    }
    if (check_page_mode(block) != ESP_OK || erase(block, after) != ESP_OK) {
        return ESP_FAIL;
    }
    for (uint32_t offset = 0; offset < TRICORE_UCB_SIZE; offset += UCB_PAGE_SIZE) {
        if (offset != state_page &&
            program_page(block->base + offset, blob + offset) != ESP_OK) {
            return ESP_FAIL;
        }
    }
    if (program_page(block->base + state_page, blob + state_page) != ESP_OK) {
        return ESP_FAIL;
    }

    if (read_block(block, after) != ESP_OK) {
        return why("%s could not be read back after programming", block->name);
    }
    uint32_t first = 0;
    const uint32_t differing = differences(blob, after, &first);
    check(block, after, p);
    if (differing) {
        return why("read-back differs in %" PRIu32 " bytes, first at +0x%03" PRIX32
                   " (wrote 0x%02X, read 0x%02X)", differing, first, blob[first], after[first]);
    }
    if (p->count) {
        return why("programmed, but the block does not pass the formal check");
    }
    snprintf(s_why, sizeof(s_why), "programmed and verified");
    return ESP_OK;
}

/* -- the whole run (flash_target.program_ucbs) -------------------------------- */

esp_err_t tricore_ucb_program(const tricore_ucb_image_t *image)
{
    static const char *const LABELS[] = { "write", "skip", "REFUSED" };
    size_t writing = 0, refused = 0, failed = 0;

    s_report_len = 0;
    s_report[0] = '\0';
    if (image == NULL || image->data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    plan(image);
    report("user configuration blocks:\n");
    if (s_decision_count == 0) {
        report("  the image carries none\n");
    }
    for (size_t i = 0; i < s_decision_count; i++) {
        const decision_t *d = &s_decisions[i];
        report("  %-14s %-8s %s%s%s%s\n", d->block->name, LABELS[d->action],
               d->state[0] ? "[" : "", d->state, d->state[0] ? "] " : "", d->reason);
        writing += (d->action == ACT_WRITE);
        refused += (d->action == ACT_REFUSE);
    }
    ESP_LOGI(TAG, "%u block(s) to write, %u refused", (unsigned)writing, (unsigned)refused);
    if (!writing) {
        report("  nothing to write\n");
        return refused ? ESP_FAIL : ESP_OK;
    }

    /* A failure does not abandon the rest: four originals and four copies
     * exist so that one bad one is survivable. */
    report("\nprogramming %u block(s) ...\n", (unsigned)writing);
    for (size_t i = 0; i < s_decision_count; i++) {
        const decision_t *d = &s_decisions[i];
        if (d->action != ACT_WRITE) {
            continue;
        }
        problems_t p;
        const int64_t t0 = esp_timer_get_time();
        const bool ok = write_block(d->block, blob_of(image, d->block), &p) == ESP_OK;
        const uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

        failed += !ok;
        report("  %-14s %s: %s (%" PRIu32 ".%" PRIu32 " s)\n", d->block->name,
               ok ? "ok" : "FAILED", s_why, ms / 1000u, (ms % 1000u) / 100u);
        if (p.count) {
            report("                 %s\n", p.text);
        }
        ESP_LOGI(TAG, "%s %s: %s", d->block->name, ok ? "ok" : "FAILED", s_why);
    }
    clear_status();
    if (failed) {
        report("  %u block(s) failed\n", (unsigned)failed);
    }
    return (failed || refused) ? ESP_FAIL : ESP_OK;
}
