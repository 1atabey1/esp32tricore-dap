/*
 * TC3xx cores as Black Magic Probe targets.
 *
 * BMP supplies the GDB server on port 4242 (RSP, extended-remote, qXfer, flash
 * packets); this file adapts it to the TriCore run control in tricore.c and the
 * flasher in tricore_flash.c.  One BMP target per core, registered from outside
 * the BMP component through target_new().
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dap_lock.h"
#include "dap_phy.h"
#include "dap_phy_fpga.h"
#include "dap_probe.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "tricore.h"
#include "tricore_bmp.h"
#include "tricore_flash.h"

static const char *TAG = "TRICORE_BMP";

/* The Cerberus trigger line that drives break-in; line 0 does not halt. */
#define HALT_LINE 1

#define CORE_OF(target) ((int)(intptr_t)(target)->target_storage)

/*
 * Register layout, in the order and with the names tricore-elf-gdb expects
 * (`maint print registers`): it rejects any other set.  PCXI is `pcx`, and
 * lcx/fcx precede it.  A11 is typed as a code pointer so GDB can unwind.
 */
static const char k_target_xml[] =
    "<?xml version=\"1.0\"?>"
    "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">"
    "<target version=\"1.0\">"
    "<architecture>TriCore:V1_6_2</architecture>"
    "<feature name=\"org.gnu.gdb.tricore.core\">"
    "<reg name=\"d0\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d1\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d2\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d3\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d4\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d5\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d6\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d7\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d8\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d9\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d10\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d11\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d12\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d13\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d14\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"d15\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"a0\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a1\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a2\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a3\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a4\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a5\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a6\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a7\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a8\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a9\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a10\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a11\" bitsize=\"32\" type=\"code_ptr\"/>"
    "<reg name=\"a12\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a13\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a14\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"a15\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"lcx\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"fcx\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"pcx\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"psw\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>"
    "<reg name=\"icr\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"isp\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"btv\" bitsize=\"32\" type=\"code_ptr\"/>"
    "<reg name=\"biv\" bitsize=\"32\" type=\"code_ptr\"/>"
    "<reg name=\"syscon\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"pcon0\" bitsize=\"32\" type=\"uint32\"/>"
    "<reg name=\"dcon0\" bitsize=\"32\" type=\"uint32\"/>"
    "</feature>"
    "</target>";

/* Sticky: set by a failed transaction, reported and cleared by check_error. */
static bool s_error;

/* The last resume of this core was a single step; the step leaves no trigger
 * behind by the time BMP asks why the core stopped. */
static bool s_stepped[TRICORE_MAX_CORES];

/* GDB resumed this core since attaching.  BMP keeps a target attached when
 * the GDB connection drops, and the next session's '?' makes it wait for a
 * halt; a core restarted meanwhile (web flash, reset, power cycle) would
 * never report one and every packet would be swallowed. */
static bool s_resumed[TRICORE_MAX_CORES];

/* -- memory ---------------------------------------------------------------- */

static void tricore_mem_read(target_s *target, void *dest, target_addr64_t src,
                             size_t len) {
  (void)target;
  if (tricore_read_mem((uint32_t)src, (uint8_t *)dest, len) != ESP_OK) {
    memset(dest, 0, len);
    s_error = true;
  }
}

static void tricore_mem_write(target_s *target, target_addr64_t dest,
                              const void *src, size_t len) {
  (void)target;
  if (tricore_write_mem((uint32_t)dest, (const uint8_t *)src, len) != ESP_OK) {
    s_error = true;
  }
}

static bool tricore_check_error(target_s *target) {
  (void)target;
  const bool had = s_error;
  s_error = false;
  return had;
}

/* -- registers ------------------------------------------------------------- */

static const char *tricore_regs_description(target_s *target) {
  (void)target;
  return strdup(k_target_xml); /* gdb_main.c frees it */
}

static void tricore_regs_read(target_s *target, void *data) {
  uint32_t regs[TRICORE_NUM_REGS];

  if (tricore_read_all_regs(CORE_OF(target), regs) != ESP_OK) {
    memset(data, 0, sizeof(regs));
    s_error = true;
    return;
  }
  memcpy(data, regs, sizeof(regs));
}

static void tricore_regs_write(target_s *target, const void *data) {
  const uint32_t *regs = (const uint32_t *)data;

  for (int i = 0; i < TRICORE_NUM_REGS; i++) {
    if (tricore_write_reg(CORE_OF(target), i, regs[i]) != ESP_OK) {
      s_error = true;
    }
  }
}

static size_t tricore_reg_read(target_s *target, uint32_t reg, void *data,
                               size_t max) {
  uint32_t value = 0;

  if (max < sizeof(value) || reg >= TRICORE_NUM_REGS) {
    return 0;
  }
  if (tricore_read_reg(CORE_OF(target), (int)reg, &value) != ESP_OK) {
    s_error = true;
    return 0;
  }
  memcpy(data, &value, sizeof(value));
  return sizeof(value);
}

static size_t tricore_reg_write(target_s *target, uint32_t reg,
                                const void *data, size_t size) {
  uint32_t value;

  if (size < sizeof(value) || reg >= TRICORE_NUM_REGS) {
    return 0;
  }
  memcpy(&value, data, sizeof(value));
  if (tricore_write_reg(CORE_OF(target), (int)reg, value) != ESP_OK) {
    s_error = true;
    return 0;
  }
  return sizeof(value);
}

/* -- run control ----------------------------------------------------------- */

static void tricore_halt_request_cb(target_s *target) {
  if (tricore_halt_request(CORE_OF(target), HALT_LINE) != ESP_OK) {
    s_error = true;
  }
}

/* Why the core stopped.  The trigger accumulator (clear-on-read) names the
 * comparator, and what occupies that slot says breakpoint or watchpoint. */
static target_halt_reason_e tricore_halt_poll_cb(target_s *target,
                                                 target_addr64_t *watch) {
  const int core = CORE_OF(target);

  if (!tricore_halt_poll(core)) {
    if (!s_resumed[core]) {
      /* Running without GDB having resumed it: stop it and say so. */
      tricore_halt(core, HALT_LINE, 200);
      if (tricore_is_halted(core)) {
        return TARGET_HALT_REQUEST;
      }
    }
    return TARGET_HALT_RUNNING;
  }
  s_resumed[core] = false;

  uint32_t acc = 0;
  if (tricore_trigger_acc(core, &acc) == ESP_OK && acc != 0) {
    for (int slot = 0; slot < TRICORE_NUM_TRIGGERS; slot++) {
      if (!(acc & (1u << slot))) {
        continue;
      }
      const tricore_bp_t *bp = tricore_bp_of_slot(core, slot);
      if (bp == NULL) {
        continue;
      }
      if (bp->kind == TRICORE_BP_WATCH) {
        if (watch != NULL) {
          *watch = bp->addr;
        }
        return TARGET_HALT_WATCHPOINT;
      }
      if (bp->kind == TRICORE_BP_USER) {
        return TARGET_HALT_BREAKPOINT;
      }
    }
  }

  /* A completed step is SIGTRAP, not the SIGINT a plain request would give. */
  if (s_stepped[core]) {
    s_stepped[core] = false;
    return TARGET_HALT_STEPPING;
  }
  return TARGET_HALT_REQUEST;
}

static void tricore_halt_resume(target_s *target, bool step) {
  const int core = CORE_OF(target);

  if (step) {
    tricore_step_t result;
    if (tricore_step(core, 500, &result) != ESP_OK) {
      s_error = true;
    }
    s_stepped[core] = true;
    return;
  }

  /* Step off a breakpoint under the PC first, or it fires again at once. */
  uint32_t pc = 0;
  if (tricore_is_halted(core) && tricore_read_pc(core, &pc) == ESP_OK) {
    const tricore_bp_t *bp = tricore_bp_at(core, pc);
    if (bp != NULL && bp->kind == TRICORE_BP_USER) {
      tricore_step_t stepped;
      tricore_step(core, 500, &stepped);
    }
  }
  s_resumed[core] = true;
  tricore_request_resume(core);
}

/* -- breakpoints and watchpoints ------------------------------------------ */

/* Software breakpoints are served from address triggers too: patching a DEBUG
 * instruction into flash would cost a sector erase per toggle. */
static int tricore_breakwatch_set(target_s *target, breakwatch_s *bw) {
  const int core = CORE_OF(target);
  int slot;

  switch (bw->type) {
  case TARGET_BREAK_SOFT:
  case TARGET_BREAK_HARD:
    slot = tricore_bp_add(core, (uint32_t)bw->addr, TRICORE_BP_USER);
    break;
  case TARGET_WATCH_WRITE:
    slot = tricore_bp_add_watch(core, (uint32_t)bw->addr, bw->size, false, true);
    break;
  case TARGET_WATCH_READ:
    slot = tricore_bp_add_watch(core, (uint32_t)bw->addr, bw->size, true, false);
    break;
  case TARGET_WATCH_ACCESS:
    slot = tricore_bp_add_watch(core, (uint32_t)bw->addr, bw->size, true, true);
    break;
  default:
    return 1;
  }
  if (slot < 0) {
    ESP_LOGW(TAG, "CPU%d: all %d triggers in use", core, TRICORE_NUM_TRIGGERS);
    return -1;
  }
  return 0;
}

static int tricore_breakwatch_clear(target_s *target, breakwatch_s *bw) {
  tricore_bp_remove(CORE_OF(target), (uint32_t)bw->addr);
  return 0;
}

/* A reset takes every trigger with it; drop the bookkeeping to match. */
static void forget_triggers(void) {
  for (int i = 0; i < tricore_core_count(); i++) {
    const int core = tricore_core_index(i);
    tricore_bp_clear_kind(core, TRICORE_BP_USER);
    tricore_bp_clear_kind(core, TRICORE_BP_STEP);
    tricore_bp_clear_kind(core, TRICORE_BP_WATCH);
    tricore_bp_clear_kind(core, TRICORE_BP_WATCH_HI);
    tricore_freeze_timer(core, true);
  }
}

/* -- flash: GDB `load` ----------------------------------------------------- */

/* One load is one flasher session: begin on the first flash packet, end on
 * vFlashDone.  Per-region prepare/done stay empty because BMP calls them again
 * between erase and write, which would reset the target twice. */
static bool tricore_enter_flash_mode(target_s *target) {
  (void)target;
  return tricore_flash_begin() == ESP_OK;
}

static bool tricore_exit_flash_mode(target_s *target) {
  (void)target;
  const bool ok = tricore_flash_end(false) == ESP_OK;
  forget_triggers();
  return ok;
}

/* Undo a failed session, so the next `load` starts a fresh one. */
static bool flash_failed(target_flash_s *flash) {
  tricore_flash_end(false);
  flash->t->flash_mode = false;
  return false;
}

static bool pflash_erase(target_flash_s *flash, target_addr_t addr, size_t len) {
  return tricore_flash_erase((uint32_t)addr, (uint32_t)len) == ESP_OK
             ? true : flash_failed(flash);
}

static bool pflash_write(target_flash_s *flash, target_addr_t dest,
                         const void *src, size_t len) {
  return tricore_flash_program((uint32_t)dest, (const uint8_t *)src,
                               (uint32_t)len) == ESP_OK
             ? true : flash_failed(flash);
}

/* UCBs and data flash are never programmed from here, as in the reference:
 * an ELF's boot-mode headers are skipped, not written. */
static bool ucb_erase(target_flash_s *flash, target_addr_t addr, size_t len) {
  (void)flash;
  (void)addr;
  (void)len;
  return true;
}

static bool ucb_write(target_flash_s *flash, target_addr_t dest,
                      const void *src, size_t len) {
  (void)flash;
  (void)src;
  ESP_LOGW(TAG, "load: skipped %u bytes at 0x%08lX (UCB/data flash)",
           (unsigned)len, (unsigned long)dest);
  return true;
}

static void add_flash(target_s *target, uint32_t start, uint32_t length,
                      size_t block, flash_erase_func erase,
                      flash_write_func write) {
  target_flash_s *flash = calloc(1, sizeof(*flash));
  if (flash == NULL) {
    return;
  }
  flash->start = start;
  flash->length = length;
  flash->blocksize = block;
  flash->writesize = block;
  flash->erased = TRICORE_FLASH_ERASED;
  flash->erase = erase;
  flash->write = write;
  target_add_flash(target, flash);
}

/*
 * The memory map, from tas-debug's tc38x profile and the project's linker
 * script.  GDB treats anything unlisted as inaccessible, so it has to cover
 * everything; unpopulated gaps inside a region just bus-error as before.
 *
 * Kept coarse on purpose: BMP renders the map into a 1024-byte stack buffer
 * (gdb_main.c, exec_q_memory_map) and overruns it once it is full, so each
 * core's DSPR and PSPR are one region and the local aliases another.
 */
static void add_memory_map(target_s *target) {
  static const struct { uint32_t start, len; } k_ram[] = {
      {0x70000000u, 0x110000u},   /* CPU0 DSPR .. PSPR */
      {0x60000000u, 0x110000u},   /* CPU1 */
      {0x50000000u, 0x110000u},   /* CPU2 */
      {0x40000000u, 0x110000u},   /* CPU3 */
      {0x90000000u, 0x60000u},    /* DLMU0-3, LMU0, cached */
      {0xB0000000u, 0x60000u},    /* the same, uncached */
      {0xC0000000u, 0x10010000u}, /* own DSPR (0xC...) and PSPR (0xD...) */
      {0xF0000000u, 0x10000000u}, /* peripherals, CSFRs */
  };
  for (size_t i = 0; i < sizeof(k_ram) / sizeof(k_ram[0]); i++) {
    target_add_ram32(target, k_ram[i].start, k_ram[i].len);
  }

  const tricore_flash_range_t *ranges;
  const size_t n = tricore_flash_layout(&ranges);
  for (size_t i = 0; i < n; i++) {
    const uint32_t len = ranges[i].end - ranges[i].start;
    add_flash(target, ranges[i].start - 0x20000000u, len, TRICORE_FLASH_SECTOR,
              pflash_erase, pflash_write);                      /* cached */
    add_flash(target, ranges[i].start, len, TRICORE_FLASH_SECTOR,
              pflash_erase, pflash_write);                      /* uncached */
  }
  add_flash(target, 0xAF000000u, 0x01000000u, 0x200u, ucb_erase, ucb_write);
}

/* -- attach and detach ----------------------------------------------------- */

static bool tricore_attach(target_s *target) {
  const int core = CORE_OF(target);

  if (!tricore_core_started(core)) {
    ESP_LOGW(TAG, "CPU%d was never started by the application (boot halt); "
                  "nothing to attach to", core);
    return false;
  }

  tricore_disarm_reset_trigger();
  tricore_freeze_timer(core, true);
  s_error = false;
  s_resumed[core] = false;

  /* With OCDS on but the core running, register reads bus-error, so attach
   * halts.  The halt arrives over a trigger line, hence the poll. */
  target_halt_request(target);
  target_halt_reason_e reason = TARGET_HALT_RUNNING;
  for (int i = 0; i < 200 && reason == TARGET_HALT_RUNNING; i++) {
    reason = target_halt_poll(target, NULL);
    if (reason == TARGET_HALT_RUNNING) {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
  if (reason == TARGET_HALT_ERROR || reason == TARGET_HALT_RUNNING) {
    tricore_halt_diag(core, "attach");
    tricore_halt_release(core); /* or every later halt fails too */
    return false;
  }
  return true;
}

/* Leave it running, with none of our triggers armed. */
static void tricore_detach(target_s *target) {
  const int core = CORE_OF(target);

  /* GDB gone mid-load: BMP detaches without ending the flash session. */
  if (target->flash_mode) {
    tricore_flash_end(false);
    forget_triggers();
    target->flash_mode = false;
  }

  tricore_bp_clear_kind(core, TRICORE_BP_USER);
  tricore_bp_clear_kind(core, TRICORE_BP_STEP);
  tricore_bp_clear_kind(core, TRICORE_BP_WATCH);
  tricore_bp_clear_kind(core, TRICORE_BP_WATCH_HI);
  tricore_clear_debug_events(core);
  tricore_freeze_timer(core, false);
  s_resumed[core] = false;
  if (tricore_request_resume(core) != ESP_OK || tricore_is_halted(core)) {
    ESP_LOGW(TAG, "CPU%d did not resume on detach", core);
  }
}

/* -- probe and reset ------------------------------------------------------- */

/* Attach over the fabric when the DAP bitstream is loaded (the CPU path then
 * reads only 0xFFFFFFFF), else over the CPU-driven path. */
static esp_err_t attach_dap(void) {
  dap_exchange_t x;

  esp_err_t err = dap_probe_init(CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ);
  if (err != ESP_OK) {
    return err;
  }
  if (dap_phy_fpga_attach() != ESP_OK) {
    dap_phy_fpga_use(false);
    if (dap_probe_attach(&x, 3) != ESP_OK || x.reply != 0xAAAAAAAAu) {
      ESP_LOGE(TAG, "the target did not answer sync");
      return ESP_ERR_INVALID_STATE;
    }
    dap_probe_client_set(1, &x);
  }
  dap_probe_clear_error_state();
  dap_probe_set_rw_mode(true);
  if (dap_probe_enable_ocds() != ESP_OK) {
    ESP_LOGE(TAG, "OCDS did not come up");
    return ESP_ERR_INVALID_STATE;
  }
  return ESP_OK;
}

static bool wait_for_target(int attempts) {
  for (int i = 0; i < attempts; i++) {
    vTaskDelay(pdMS_TO_TICKS(20));
    if (tricore_discover() == ESP_OK) {
      return true;
    }
    dap_probe_clear_error_state();
    attach_dap();
  }
  return false;
}

/*
 * GDB `run` / `monitor reset`: restart the application with halt-after-reset
 * armed, so the cores stop at the entry point.  An OCDS application reset
 * first; the reset pin if the target does not come back from that.
 */
static void tricore_reset(target_s *target) {
  (void)target;

  const bool armed = (tricore_set_halt_after_reset(true) == ESP_OK);
  tricore_request_application_reset();

  if (!wait_for_target(20)) {
    ESP_LOGW(TAG, "no answer after the OCDS reset; pulsing the reset pin");
    dap_phy_set_trst(true);
    vTaskDelay(pdMS_TO_TICKS(20));
    dap_phy_set_trst(false);
    vTaskDelay(pdMS_TO_TICKS(50));
    if (attach_dap() != ESP_OK || !wait_for_target(10)) {
      ESP_LOGE(TAG, "the target did not come back after the reset");
      s_error = true;
      return;
    }
  }
  if (armed) {
    tricore_set_halt_after_reset(false); /* or the next reset halts too */
  }
  tricore_disarm_reset_trigger();
  forget_triggers();
}

/* BMP's target list; a probe scan (swd_scan, auto_scan) frees it wholesale. */
extern target_s *target_list;

static target_s *s_targets[TRICORE_MAX_CORES];

static bool still_listed(const target_s *t) {
  for (const target_s *it = target_list; it; it = it->next) {
    if (it == t) {
      return true;
    }
  }
  return false;
}

static esp_err_t probe_locked(void) {
  static char s_names[TRICORE_MAX_CORES][24];

  /* A session whose GDB died during `continue` leaves BMP polling forever
   * without accepting connections; forgetting the resume lets the next poll
   * halt the core and end that loop. */
  memset(s_resumed, 0, sizeof(s_resumed));

  esp_err_t err = attach_dap();
  if (err != ESP_OK) {
    return err;
  }
  err = tricore_discover();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "no TriCore cores answered");
    return err;
  }

  for (int i = 0; i < tricore_core_count(); i++) {
    const int core = tricore_core_index(i);

    snprintf(s_names[core], sizeof(s_names[core]), "CPU%d%s", core,
             tricore_core_started(core) ? "" : " (not started)");
    if (s_targets[core] != NULL && still_listed(s_targets[core])) {
      continue;
    }

    target_s *target = target_new();
    if (target == NULL) {
      return ESP_ERR_NO_MEM;
    }
    s_targets[core] = target;

    target->target_storage = (void *)(intptr_t)core;
    target->driver = "TriCore TC3xx";
    target->core = s_names[core];

    target->attach = tricore_attach;
    target->detach = tricore_detach;
    target->check_error = tricore_check_error;

    target->mem_read = tricore_mem_read;
    target->mem_write = tricore_mem_write;

    target->regs_size = TRICORE_NUM_REGS * sizeof(uint32_t);
    target->regs_description = tricore_regs_description;
    target->regs_read = tricore_regs_read;
    target->regs_write = tricore_regs_write;
    target->reg_read = tricore_reg_read;
    target->reg_write = tricore_reg_write;

    target->halt_request = tricore_halt_request_cb;
    target->halt_poll = tricore_halt_poll_cb;
    target->halt_resume = tricore_halt_resume;

    target->breakwatch_set = tricore_breakwatch_set;
    target->breakwatch_clear = tricore_breakwatch_clear;

    target->reset = tricore_reset;
    target->extended_reset = tricore_reset;

    target->enter_flash_mode = tricore_enter_flash_mode;
    target->exit_flash_mode = tricore_exit_flash_mode;
    add_memory_map(target);

    ESP_LOGI(TAG, "registered %s as a GDB target", s_names[core]);
  }
  return ESP_OK;
}

/* Safe to call again (boot, then /api/dap_gdb/attach): a core that already has
 * a target keeps it, and only its name is refreshed. */
esp_err_t tricore_bmp_probe(void) {
  dap_lock();
  const esp_err_t err = probe_locked();
  dap_unlock();
  return err;
}
