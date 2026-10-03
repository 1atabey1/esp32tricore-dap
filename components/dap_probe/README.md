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
| `tricore_ucb.c`, `include/tricore_ucb.h` | Boot mode header (UCB) plan and writer, from tas-debug's `ucb.py` / `ucb_program.py` |
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
- Loader blob byte-identical to tas-debug's (`loader_blob.py`, 516 bytes): it programs 256-byte
  Write Bursts where the address allows (pages for the rest) and polls `DMU_HF_STATUS` until the
  bank is idle, timed out in STM ticks. The STM's frequency is measured at install (100 MHz here:
  our OCDS reset keeps the application's PLL); STM0's suspend-while-halted, which a killed GDB
  session leaves on, is lifted for the measurement. Only if the STM does not count does the stub
  use its spin loop, whose `PROGRAM_SETTLE` is patched 4000 -> 12000 wherever the `mov` sits.
- Differential, as tas-debug's `differential` mode: before erasing, the loader CRCs the
  bytes the image programs into each sector (only those); matching sectors are neither
  erased nor written. Unreadable (erased, no valid ECC) counts as changed. `?full=1`
  on `/api/flash/start` programs everything.
- Consecutive changed sectors are erased by one Erase Logical Sector Range command (up
  to 32 sectors inside a 1 MB physical sector; tMERP = tERP = 0.5 s max). tas-debug saw
  counts above one refused; this device accepts them. A refusal (SQER) falls back to one
  sector per command until reboot.
- Sessions run the DAP narrow at 24 MHz (the attach default is 4 MHz, restored after).
  Wide mode would not help: it cannot carry block writes, and verification is a CRC on
  the target.
- A 736 kB image: **2.8 s** in full (erase 0.4 s, programming 1.2 s in 2880 bursts, buffer
  transfers 0.8 s) - 7.3 s with page-by-page programming; 0.8 s when unchanged.
  `/api/flash/status` reports `compare_ms`, `erase_ms`, `write_ms`, `loader_ms`, `skipped`,
  `bursts` and `op_us_max` (longest polled operation; 378 us measured, 530 us data-sheet max).
- GDB `load` erases nothing on `vFlashErase`: the sectors BMP hands over (whole 16 KB,
  padded with the erased value) are collected in PSRAM and applied on `vFlashDone` with
  the same differential plan. An interrupted load leaves the flash as it was.
- Block write loses its first parcel: after each block, drain IO_SUPERVISOR and rewrite word 0.
- UCB range (0xAF000000) is mapped, but GDB writes to it are refused.
- UCBs: web flasher only, `?ucb=1` (the page's checkbox, off by default), as tas-debug's
  `--ucb`. After the program flash verifies and before the reset, `tricore_ucb.c` plans
  every block the HEX carries (0xAF400000..0xAF406000, padded with 0x00): only
  BMHD0..3 ORIG/COPY are candidates; SWAP, OTP, DBG, HSM, ... are listed and skipped.
  A header is refused when it fails the formal check (BMHDID, BMI.HWCFG, STAD, CRCBMHD
  pair, confirmation word), would be CONFIRMED by the image, is not UNREAD/UNLOCKED in
  `DMU_HF_CONFIRMx`, or cannot be read; identical headers are skipped. A write is a
  one-sector erase (read back as all 0x00), 8-byte data flash pages (`0x5D`) with the
  confirmation-word page last, read-back compare and re-check: ~0.1 s per block.
  The target is started either way; a refusal or failure sets `phase=failed ucb=failed`.
  Report: `/api/flash/ucb`.
- Sessions: `tricore_flash_begin` / `apply` / `end` (GDB `load`), or `tricore_flash_write`
  (web, all-in-one).

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
- Sessions run the DAP wide and fast by default (`"wide"`, `"fast"`, `"dap_div"` in the config):
  wide calibrates its taps at `dap_div` (0, 24 MHz), then `dap_fast_enter` finds fast mode's
  receive timing against the same reference block (known-good LAG/edge/tap settings first, three
  clean reads to accept, the DAP checked at DIV after each miss) and falls back to DIV if none
  carries data; narrow if P21.7 is driven. `fast_active`/`wide_active` in `GET /api/mcds/config`
  say what the session got; the start log line also names the FIFO drain (one or two lines).
  The start snapshot of the watched ranges is in `GET /api/mcds/config` (`"snapshot"`).
- `/ws/trace?z=1` frames carry an 8-byte header (`DTRZ`, u32 raw length, top bit = stored) and
  are LZ4-compressed where it pays: the sender measures the compressor's speed C and ratio r and
  the wire rate L as it streams and compresses a frame only when n/C + n/(rL) < n/L (probing one
  frame in 16 otherwise). On the probe's WiFi it does not pay: the compressor runs ~5-6 MB/s on
  the S3 (after replacing memcpy loads, which made it ~3 MB/s), live traces compress 1.5-1.85x,
  and the link takes ~3.5-4.4 MB/s, so break-even would need ~8-9 MB/s; on a weak link it
  switches on by itself. `tools/mcds_trace` asks for the framing; the web pages take the raw
  stream. The sender logs C, r, L and how many frames it compressed when it stops.
- Measured (TC387, CPU2 watches on the mlpc PXROS application, 4.4-5.5 MB/s sources), against
  the previous firmware on the same board: the drain reads 5.15-5.3 MB/s (186-190 µs per
  paragraph), was 3.06 MB/s (319 µs; wide at 24 MHz, one-line FIFO reads). FIFONOW is read after
  every fourth paragraph (`TRACE_NOW_EVERY`), not after each, and a block's frame and its start go
  in one transfer (GO). What the session keeps is now set by WiFi: 2.1-2.8 MB/s of raw trace
  delivered (it varies with the link), the 4 MB ring covering bursts at the drain's rate for 1-2 s;
  a fuller ring drops whole paragraphs (`queue_dropped`). Note `paragraphs`+`lost` in the stats
  understates the source: ring drops are counted in `queue_dropped` only.

## Fabric link (dap_phy_fpga.c, fpga/dap_master)

- SPI at 40 MHz (GPIO matrix limit on these pins); register-level transfers: the 64-byte CPU
  buffer for short ones, GDMA (the driver's idle channels, LL calls from IRAM) above 96 bytes.
  7-byte register read 5.4 µs, 1 kB FIFO read 214 µs (4.7 MB/s), single line.
- The reply FIFO's read side runs on SCK (`reply_fifo.v`: block RAM with separate clocks, Gray
  pointers), so a burst read costs no fabric clocks per byte. Before, each byte crossed to the
  fabric and back (~8 fabric clocks against 7.5 SCK periods): SCK could not exceed ~0.9x the
  fabric clock, and 40 MHz already failed at the industrial HFOSC corner in simulation.
- `0x50` reads the FIFO over two lines (the S3's half-duplex dual read: header on MOSI, eight dummy
  clocks to turn SI round, data on MOSI+MISO), opening with a prefix - the level when the header
  ended and {busy, timed_out, overrun, queued} - then exactly that many bytes; clocking more reads
  zeros and pops nothing. One transfer replaces the LEVEL read and the FIFO read, and the torn
  LEVEL above cannot happen. At init `TESTPUSH` (0x51) proves 0x40 and 0x50 with no target, and
  the S3's input timing for the two lines (din_mode/din_num, plus a ninth dummy clock) is swept
  over a pattern and its complement; the middle of the longest passing run is used, the drain
  stays on one line if none passes. In simulation: byte-exact at SCK 40/50/80 MHz against fabric
  clocks 38.4-52.8 MHz; the SCK domain closes at 40 MHz on every seed (54.4 on the pinned one).
  On the board the window is dummy 9 + din_mode 1/din_num 1 (four points pass). A ninth dummy
  clock gives the fabric one clock more than the host samples, so a byte pops only after its last
  clock (the next one prefetched): popping on the first lost a byte per read on hardware, and the
  calibration now reads each pattern in two parts to catch exactly that.
- Measured on the board, 1 kB blocks chained, data verified, previous firmware in brackets:
  narrow 24 MHz 2.43 MB/s (2.37), wide 24 MHz 4.19 (3.89), fast narrow 4.28 (3.88), fast wide
  6.28 MB/s (3.93). read32_fast 22 µs at 24 MHz (27), 32 µs fast (36).
- Block reads chain: a CTRL start written while the sequencer is busy is queued (LEVEL high byte
  bit 4) and taken on idle, so blocks run back to back while the host drains. With the reply FIFO
  full the fabric pauses between parcels (DAP0 stopped, DAP1 released) instead of overrunning;
  CTRL bit 2 aborts such a block.
- LEVEL's two bytes are sampled a byte time apart while a block fills the FIFO, so a carry out of
  the low byte in between reads 256 too high. The drain takes the lower reading near a wrap: an
  over-read returns zeros (an empty FIFO does not pop) and shifts every later byte of the chain,
  which showed up as trace paragraphs starting with ~150-235 zero bytes.
- Wide mode calibrates its capture taps at the session's clock against a pattern written narrow
  into the trace RAM; taps chosen at one divider are wrong at another (24 MHz returned corrupt
  data with div-5 taps). Block writes are refused in wide mode (parcels arrive corrupted).
- 1 kB block reads: narrow 24 MHz 2.3 MB/s, wide 24 MHz 3.65-3.9 MB/s chained, data verified
  (`/api/dap_bench?div=0&wide=1&chain=16`).
- Fast mode (`dap_phy_fpga_set_fast`, FLAGS bit 6): a 48 MHz DAP clock, one bit per
  fabric clock. DAP0 comes from the pad's DDR register, high in the second half of each cycle;
  DAP1/DAP2 are sampled in the I/O cell, on either edge (SKEW bits 4/5). Only cycles where a DAP0
  clock lands are samples (SKEW [7:6] LAG + tap + 1 clocks after it), and DAP0 stops by budget so
  the target gets exactly each reply's clocks. Frames go out raw, built by the firmware; one over
  44 data bits narrow (40 wide) goes at DIV.
  - Measured on the TC387 (`/api/dap_bench?div=0&fast=1&skew=0x40`): narrow 3.0 MB/s single,
    3.65 MB/s chained (24 MHz: 2.1 / 2.3), data verified, block writes and word writes clean.
    Wide 3.26 MB/s: past 48 MHz the SPI drain (214 µs per kB) is the limit, not the DAP.
  - Timing window, narrow: LAG 0-1 any edge, LAG 2 rising edge, any tap 0-2; LAG 3 fails. Wide
    passes up to LAG 3 rising edge. A failing setting can lose the DAP (re-attach).
  - Single-word reads are ~10 µs slower than at 24 MHz (read32_fast 36 vs 26 µs), not from
    MAXWAIT (doubled for fast frames); cause not found.
  - Timing closes at 53 MHz, the HFOSC's +10% corner (48 MHz +-10% commercial), not just the
    nominal 48: 57.7 MHz on the pinned seed, which also covers the industrial +20% corner (57.6),
    built without `--timing-allow-fail`. Seeds 1-48 span 46.9-57.7 (median 51.1, 12 pass 53).
    Before: 46.7 MHz on the best of 120, median 44.5. The remaining critical paths are the
    receiver's start-bit decisions and the transmitter's field ends.
  - Every DAP output and output enable now leaves from a register in its I/O cell. Driven from
    fabric, DAP1's enable reached its pad 13.9 ns after the clock (seed-dependent), later than
    fast mode's DAP0 edge at half a cycle. The pins lag the engines by one clock, all alike;
    the receiver takes its bit a clock later (and adds one to LAG internally), so the line is
    sampled where it was relative to the pins and SKEW/LAG settings keep their meaning.
  - What got the fabric there (iCE40 tiles share one clock enable per eight flops, and
    routing was 60-75% of every critical path): payload/CRC capture as shift registers with a
    fixed entry point instead of index-decoded writes (one enable per field, not per bit);
    down-counters with constant compares for the half-period tick, both engines' field ends and
    MAXWAIT; the fast-mode clock budget pipelined and its counter enable-free; one-hot receiver
    state; register writes and the read mux taken through local address/data registers (the
    read one rides a spare pipeline cycle, so no added latency); STATUS timed_out as a set/clear
    register; the read mux as AND-OR over a registered one-hot select; per-reply parameters
    loaded every idle clock instead of on start; a block read's word stored by a pusher beside
    the sequencer (which also drops four dead clocks per parcel); DAP1's sample duplicated, one
    register for the payload, one for the decisions. Each step was checked on 24-48 seeds; an
    explicit `CLOCK_ENABLE(1'b1)` on the pads makes no difference, nor does yosys
    `-dffe_min_ce_use`.

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
