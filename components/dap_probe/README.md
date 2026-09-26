# dap_probe

Infineon DAP master for AURIX TC3xx (tested: TC38x), plus TriCore run control,
flash programming, trace drain and a Black Magic Probe (BMP) GDB target.

## Layers

```
 GDB (port 4242)        web API (/api/dap_*, /api/flash/*)
        |                         |
 tricore_bmp.c  ---- BMP target_s glue: regs, memory, breakpoints, flash map
        |
 tricore.c / tricore_regs.c ----- halt/resume/step, CSA registers, triggers
 tricore_flash*.c --------------- PFLASH erase/program/verify via loader stub
 dap_trace.c -------------------- TRAM drain task
        |
 dap_probe.c ------------------- DAP telegrams: sync, client_set/read/write,
        |                         read32/write32, blockread/blockwrite, OCDS
 dap_frame.c ------------------- frame build, CRC6 (host-testable)
        |
 dap_phy.c        bit-bang GPIO  (bring-up reference)
 dap_phy_spi.c    GP-SPI         (CPU path, <= 12 MHz)
 dap_phy_fpga.c   iCE40 fabric DAP master over SPI  (default, <= 24 MHz, wide)
```

## Files

| File | Role |
|---|---|
| `dap_lock.c`, `include/dap_lock.h` | Recursive DAP mutex shared by all tasks |
| `dap_frame.c`, `include/dap_frame.h` | Frame layout, command/instruction constants (single source), CRC6 |
| `dap_phy.c`, `include/dap_phy.h` | Backend switch + bit-bang PHY |
| `dap_phy_spi.c` | GP-SPI PHY |
| `dap_phy_fpga.c`, `include/dap_phy_fpga.h` | Fabric register file: div, trail, wide, taps, TRST, exchange, block ops |
| `dap_probe.c`, `include/dap_probe.h` | DAP transactions |
| `dap_probe_bringup.c`, `dap_probe_diag.c` | Checkpoint bring-up + diagnostics (`/api/dap_bringup`, `/api/dap_spi`) |
| `dap_fpga_route.c` | Fabric attach (`dap_phy_fpga_attach`), route check, throughput sweep |
| `dap_dapisc.c` | dapisc telegram (CMD 0x11): write/read DAPISC, narrow revert |
| `dap_wide.c` | Wide mode bring-up + check (`/api/dap_fpga?wide=1`) |
| `dap_fpga_priv.h` | Private constants/prototypes of the three files above |
| `tricore.c`, `tricore_regs.c`, `include/tricore.h`, `tricore_priv.h` | Core discovery, run control, registers, breakpoints |
| `tricore_bmp.c`, `include/tricore_bmp.h` | BMP target: tdesc, memory map, flash callbacks |
| `tricore_flash.c`, `include/tricore_flash.h` | Flash layout, erase/program/verify, sessions |
| `tricore_flash_loader.c`, `tricore_flash_priv.h` | Loader stub (from tas-debug), reset-and-halt, trap dump |
| `dap_trace.c`, `include/dap_trace.h` | TRAM drain, stream records |
| `dap_mcds.c`, `include/dap_mcds.h` | miniMCDS setup for data trace (DTU) / watch-points (WTU) |
| `test/` | Host unit test for `dap_frame.c` (`make -C test`) |

Web glue: `main/network/dap_web.c` (DAP, trace, FPGA image, GDB registration),
`main/network/flash_web.c` (flasher). Usage: `docs/tricore_usage.md`.
Fabric RTL lives in `fpga/dap_master/` (`make sim` runs the testbench).

## Concurrency

`dap_lock.c`: one recursive mutex. Single DAP ops lock themselves; a flash
session (web or GDB `load`), diagnostics (`dap_capture_begin/end`) and target
registration hold it throughout. `/api/dap_gdb/status` waits 300 ms, then
reports `busy`.

## Attach sequence (fabric)

1. `sync` -> `0xAAAAAAAA` (TRST pulse + retry if silent: stale wide mode)
2. dapisc long form `0x4ABBAF53_0F00`, no reply
3. IOINFO read (clear error state), `sync`, `client_set(1)`
4. `client_read(CLIENT_ID)` -> `0x0260`
5. OCDS enable -> miniMCDS ID `0x00D6C007`

## Wide mode (DAPISC.MODE = 01B)

- DAP1 = even bits, DAP2 (target P21.7) = odd bits.
- P21.7 must be an input: app cores halted, IOCR4 set to input+pull-up, restored after.
- Mode change goes out narrow; its reply comes back wide.
- Wide sync reply = `0xAAAAAAAA` per line -> reassembled `0xCCCCCCCC`; CRC not checkable.
- Start bit doubled in wide replies (skip 2).
- Device swallows the first two dapisc telegrams after attach (`dap_dapisc_prime`).
- `client_set` does not answer wide; client selected in narrow carries over.
- Everything else in the firmware runs narrow; `dap_wide_check` reverts.

## Flash

- Layout from `SCU_CHIPID.FSIZE` (0xC: 10 MB contiguous at 0xA0000000; 0xB: 8 MB in two groups).
- Sequence per tas-debug: reset-and-halt, halt all started cores, load loader
  blob to CPU0 PSPR `0x70100000` (verify readback), ENDINIT, erase sectors,
  program pages, safe shutdown, verify (CRC32 on target), reset, resume.
- Loader blob byte-identical to tas-debug; `PROGRAM_SETTLE` patched 4000 -> 16000.
- Block write loses its first parcel: after each block, drain IO_SUPERVISOR and rewrite word 0.
- UCB range (0xAF000000) is mapped but writes are refused.
- Sessions: `tricore_flash_begin` / `erase` / `program` / `end` (GDB `load`),
  or `tricore_flash_write` (web, all-in-one).

## GDB target

- One BMP target per present core, named `CPU<n>`, `(not started)` if SYSCON.BHALT.
- Attaching a not-started core is refused with a log message.
- tdesc architecture `TriCore:V1_6_2`; memory map: RAMs, cached+uncached PFLASH, UCB.
- `GDB_PACKET_BUFFER_SIZE=4096` (top-level CMakeLists) for load throughput.
- Reset (`run` / `monitor reset`): OCDS app reset with halt-after-reset armed, reset pin fallback.
- Halts use Cerberus trigger line 1, shared by all cores: `tricore_halt_request` disarms
  EXEVT on the other running cores first, so only the requested core stops.
- `gdb_thread` stack is 16 KB (`main/main.c`); `/api/dap_gdb/status` reports `gdb_stack_free`.

## Data / watch-point trace (miniMCDS)

Spec extract: `docs/minimcds_trace_spec.md`. Host tool: `tools/mcds_trace`.

- Slot j = DTU comparator set j: `TCXEA*` address range, `TCXAC*` access pulse
  (pattern bit 12 WR / 13 RD, one cycle per transaction), `TCXWD*` value (byte lane from address).
- ANDed in MCXEVT8 (slot 0) / MCXEVT10 (slot 1): the Table 394 columns carrying all six DTU triggers.
- Full: MCXACT27-30 (`dtu_wdat/wadr/rdat/radr`) level on the slot events.
  Compact: MCXACT5/6 (`wtu_enable[j]`) edge → 12-bit WPS.
- Time: TSR per paragraph (`tsu_rel_sync` on `sync_rq`), plus per hit (`hit`), or `<tick>` (`ticks`, ~850 kB/s idle).
- Start: OCDS/EECTRC, MCDS reset, MUX_TC_RC then MUX, TROFF, actions, events, comparators,
  FIFO ring (BOT 0, TOP 1FFF, PRE 1FE0, WARN 0), TRAM filled with FFFFFFFF, TRON, CLR.
  Stop: SET (flush), drain incl. the last partial paragraph, MCDS reset.
- Drain: core 0, prio 12, busy-poll (FreeRTOS tick is 10 ms; the 8 kB TRAM fills in ~2.5 ms at
  full rate); keeps 3 paragraphs margin to the writer. Paragraphs are read as one chain of block
  reads, each followed by a one-word FIFONOW read (torn → dropped, flagged).
  1 MB PSRAM ring → `/ws/trace` (core 1, 16 kB frames). WiFi/lwIP pinned to core 1.
- Sessions run the DAP wide at 24 MHz (`"wide"`/`"dap_div"` in the config; narrow if P21.7 is
  driven). The start snapshot of the watched ranges is in `GET /api/mcds/config` (`"snapshot"`).
- Measured: the drain sustains ~2.7 MB/s of trace data against a source writing 5-6 MB/s
  (paragraph reads 318 µs/kB while tracing vs 257 µs/kB idle - the trace RAM itself is not the
  limit, the DAP is). Lower offered rates are lossless.

## Fabric link (dap_phy_fpga.c, fpga/dap_master)

- SPI at 40 MHz (GPIO matrix limit on these pins); register-level transfers: the 64-byte CPU
  buffer for short ones, GDMA (the driver's idle channels, LL calls from IRAM) above 96 bytes.
  7-byte register read 5.4 µs, 1 kB FIFO read 214 µs (4.7 MB/s).
- Block reads chain: a CTRL start written while the sequencer is busy is queued (LEVEL high byte
  bit 4) and taken on idle, so blocks run back to back while the host drains. With the reply FIFO
  full the fabric pauses between parcels (DAP0 stopped, DAP1 released) instead of overrunning;
  CTRL bit 2 aborts such a block.
- Wide mode calibrates its capture taps at the session's clock against a pattern written narrow
  into the trace RAM; taps chosen at one divider are wrong at another (24 MHz returned corrupt
  data with div-5 taps). Block writes are refused in wide mode (parcels arrive corrupted).
- 1 kB block reads: narrow 24 MHz 2.3 MB/s, wide 24 MHz 3.65-3.9 MB/s chained, data verified
  (`/api/dap_bench?div=0&wide=1&chain=16`).
- A 48 MHz DAP clock is within the TC38x's limits (160 MHz) but not this fabric's: device-to-host
  data is only guaranteed valid for 8-10 ns per bit with no fixed position, which needs sub-5 ns
  sampling; the 48 MHz UP5K fabric samples in 20.8 ns steps (10.4 ns with DDR inputs), and it
  closes timing at 50 MHz with little margin. The link (4.7 MB/s) would cap the gain at ~20%.

## Known limits

- Root cause of the first-parcel loss in block write not found (worked around).
- GDB 14 shows `e0..e14` / `p0..p14` pseudo-registers wrong (GDB-side, no packets involved).
- BMP renders the memory map into 1024 bytes and overflows silently; map is kept coarse.
- TRST pin on the new adapter untested.
- A GDB killed without `detach` leaves its core halted until the next attach/detach.
- BMP's ESP32 port never detaches on a dropped connection, and while a target runs
  it does not accept new ones. A GDB killed during `continue` therefore blocks port
  4242 until `/api/dap_gdb/attach` (which halts the orphaned core). Upstream fix:
  return `'\x04'` on close in `gdb_if_getchar` and accept in `gdb_if_getchar_to`.
