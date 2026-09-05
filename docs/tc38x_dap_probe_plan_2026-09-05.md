## TC38x DAP Probe Plan

Date: 2026-09-05

### Goal

Add an Infineon DAP master to this board so it can debug and, more importantly,
*trace* AURIX TC3xx targets at a useful rate.

The driver for this is a measured ceiling on the existing tooling. A miniWiggler
V3.1 driven through Infineon's DAS/TAS stack reads target memory at about
38 kB/s:

| read size | throughput | per read |
|---|---|---|
| 256 bytes | 31.6 kB/s | 7.91 ms |
| 1024 bytes | 38.1 kB/s | 26.26 ms |
| 4096 bytes | 38.3 kB/s | 104.31 ms |

4 KB costs exactly four times what 1 KB costs, so this is bandwidth-bound on the
wire, not latency-bound on USB. Batching does not help.

What that ceiling costs in practice: a TC38x miniMCDS has an 8 kB trace buffer
(TRAM) and the application under test publishes three signals from a 20 kHz task,
which is 60000 trace messages/s at ~6.6 bytes each, about 400 kB/s. The host
drain cannot keep up, the buffer laps, and the byte stream handed to the decoder
has holes in it. Holes cost far more than the bytes they contain: the decoder
resynchronises mid-record and emits messages assembled partly from trace filler,
which pass every validity flag it offers. Measured on that target, publishing
every cycle: payloads of +/-2.1e9 among signal values of 0..135.

Today the workaround is to publish less often, at a divider chosen so the offered
rate stays near 5000 messages/s. That is a 12x reduction in sample rate to fit a
probe, and it is the thing this project removes.

### Blockers, found by code review

Checked against the tree, not assumed. The first has been designed around rather
than solved, and may yet dissolve if the source can be obtained; the other two
are still open and one of them limits what Phase 5 may promise.

**1. There is no FPGA source in this repository - so do not edit the bitstream,
add a second one.** `main/ice40up5k/` holds `bitstream.bin` (104 kB), `ice.c` and
`ice.h`: no Verilog and no constraints file. An earlier draft of this plan
assumed Phases 2 and 4 would extend that design, which without source means
re-authoring logic-analyser capture, the four-port mux, reset generation,
SPI2JTAG and the XVC path faithfully enough that nothing shipping regresses.
That is the schedule killer, not the DAP state machine.

**Ask for the source before designing around its absence.** The tree does name
where it lives, twice: `esp32jtag_common.h:57` cites
`/nvme1t/work/esp32jtag_v1d3_ice4kup/la_src/top.v` and `main.c:69` repeats the
path, both quoting that file's `data_reg_0` port assignment. So `top.v` exists on
the FPGA developer's machine, with an `la_src/` tree around it and therefore a
`.pcf` - which *is* the Phase 2 pinout gate, exactly and without scope-reading a
schematic image. One request is cheaper than three weeks of work planned around
not having it. If it arrives, the second-bitstream approach below stays worth
choosing for its isolation, but it becomes a choice rather than a forced move.

The way around it is to **never modify the stock design**. Keep it as it is and
add a second, small, purpose-built bitstream that contains only a DAP master, and
load whichever the current mode needs. The board already supports this:
`ICE_FPGA_Config(const uint8_t *bitmap, uint32_t size)` takes any buffer, drives
CRESET and polls CDONE, and is simply called once at boot (`main.c:194`) - there
is nothing boot-specific about it. A second `EMBED_FILES` entry costs one line
and ~104 kB of application image, or the file can live in the 528 kB `storage`
partition and be updated over OTA on its own.

What that changes: **the blocker stops being "obtain the HDL" and becomes "obtain
the pinout"** - and the pinout turns out to be readable out of the bitstream
itself, with `docs/esp32jtag_sch_v1.4.png` as the cross-check. The next section
records what came back: 35 used package pins with their directions, four free
pins, the PLL's clock plan, and confirmation that no memory contents need
reproducing. What is left is attaching *names* to those pins, which is a
different size of problem from reproducing five subsystems from a binary.

**2. The ESP32-to-FPGA APIs this plan cited are dead code.**
`ICE_FPGA_Serial_Write/Read` and `ICE_PSRAM_Write/Read` sit inside
`#if 0 //these functions are not used by esp32jtag project` (`ice.c` around
812-930), and `ICE_PSRAM_*` is upstream code for an FPGA-attached PSRAM this
board does not have: `psram_buffer` is ESP32 SPIRAM
(`heap_caps_malloc(..., MALLOC_CAP_SPIRAM)`, `ice.c:945`). The interfaces that
actually exist are the logic-analyser register frame - four bytes over
`gbl_spi_h1`, `ice.c:1265-1290` - and a three-byte config frame,
`spi_device4_transfer_data(data_reg_1|0x80, data_reg_0)`, `main.c:846,869`. Any
FPGA work extends those.

**3. Flashing needs true 64-bit writes, and the IOClient catalog has none.**
`flashing/flash.py:884` notes that program-flash page loads are refused as a
sequence error unless they are 64-bit, and issues 32 x `write64` per page;
`tas_proto._split` emits `PL0_CMD_RD64` for 8-byte reads. The documented Cerberus
IO instructions cover 8, 16 and 32 bits only, and block transfers are
word-wide with the address shifted right by two. **Until it is shown that
Cerberus can issue a 64-bit SRI transaction** - a dword instruction, or a
two-parcel block write that the bridge merges - the flasher does not come across
and Phase 5 must not promise it.

### What the bitstream itself gives up

Blocker 1 assumed the FPGA is opaque without `la_src/`. It is not. Project
IceStorm reads this bitstream, and the parts of the design a second bitstream has
to match come out of it. Measured on `main/ice40up5k/bitstream.bin`, in the
devcontainer, with `apt-get install fpga-icestorm` and
`iceunpack bitstream.bin > bs.asc`:

- **The header names the build.** `Lattice / DiamondNG / Part: iCE40UP5K-SG48 /
  Date: Feb 27 2026 18:02:36` - Radiant, the part this plan assumes, six months
  old.
- **The package pin map is recoverable, with directions.** Join the unpacked IO
  tiles against `icebox.pinloc_db["5k-sg48"]` and read each block's `PINTYPE`
  (bits 1:0 the input path, bits 5:2 the output path). 35 of the 39 user IO pins
  are configured:

| direction | `PINTYPE` | package pins |
|---|---|---|
| input only | `0b000001` | 2, 3, 4, 6, 9, 11, 15, 16, 17, 21, 35, 38, 39, 40, 41, 43, 44, 46, 48 (19) |
| output only | `0b011000` | 10, 12, 14, 20 (4) |
| bidirectional, tristate output | `0b101001` | 23, 25, 26, 27, 28, 31, 32, 34, 36, 37, 42, 45 (12) |
| not configured | - | 13, 18, 19, 47 (4) |

The directions check themselves against something known: pins 14-17 are the
part's configuration SPI, and they read as one output (14, `SPI_SO`) and three
inputs (`SPI_SI`, `SPI_SCK`, `SPI_SS`) - exactly a slave being configured by the
S3. The 19 pure inputs are consistent with 16 logic-analyser channels plus that
SPI; the 12 bidirectional pins are the candidate set for the four ports' JTAG and
SWD lines; and four pins are free for DAP2/DAP3 or a mode-indicator output.

- **The clock plan is pinned.** One `SB_PLL40_CORE` at tile (12,31),
  `FEEDBACK_PATH("SIMPLE")`, `PLLOUT_SELECT("GENCLK")`, `DIVR=0`, `DIVF=21`,
  `DIVQ=3`, `FILTER_RANGE=4`. That is `f_ref * 22 / 8`, and `FILTER_RANGE` 4 puts
  the reference in roughly the 40-66 MHz band, so a 48 MHz reference giving
  132 MHz is the reading that fits - which independently confirms the 132 MHz
  this document already carried and tells a new bitstream what reference to
  expect at that pin.
- **`.warmboot disabled`.** The stock image does not use `SB_WARMBOOT`, so the
  two-bitstream switch goes through CRESET and `ICE_FPGA_Config` as planned, with
  nothing in the stock image contending for the mechanism.
- **Nothing is hidden in the memories.** 15 of 30 BRAMs are configured and every
  `.ram_data` section is all zeros, so there is no initialised table or ROM to
  reproduce - the block RAM is FIFO, not data.
- **A netlist can be extracted, with a one-line fix.** `icebox_vlog` dies on this
  bitstream with `KeyError: 'SHIFTREG_DIV_MODE'`, because the UP5K PLL table
  names that bit `SHIFTREG_DIV_MODE_0`/`_1` while the script looks up the bare
  name; falling back to the `_0` suffix in `get_pll_bit` is enough. It then emits
  a flattened `module chip(...)`: 1342 `assign`s, 638 registers, the PLL, and a
  port list in `io_x_y_b` form that maps straight onto the pin table above.

**What it does not give.** Net *names*: signals are `n1234` and there is no
hierarchy, so recovering the capture engine or SPI2JTAG from 1342 flattened
assignments is reverse engineering rather than source recovery, and it is not on
this plan's path. Nor pin *functions*: the bitstream says pin 27 is
bidirectional, not that it is Port B's TMS. And `icetime -d up5k` reports
`Can't find chipdb file for device 5k` in the packaged build, so the stock
design's timing cannot be checked from here - not a loss, since a new design's
closure comes from nextpnr.

So the reconnaissance closes the ball-assignment half of the pinout gate cheaply
and leaves the naming half, which the schematic or one request for `la_src/`
answers.

### What the schematic adds, and how it checks against the bitstream

`docs/esp32jtag_sch_v1.4.png` (rev 1.4, 11 Oct 2025) answers most of the naming
half, and every fact below cross-checks against the pin directions read out of
the bitstream above - two independent sources agreeing, which is worth more than
either alone.

**The target connector is one 2x20 header, J3 (`CON40`), signal/ground
interleaved.** Odd pins carry signal, and **every even pin 2-40 is tied to
ground**:

| pins | signal |
|---|---|
| 1, 3, 5, 7 | `PA01`..`PA04` (Port A) |
| 11, 13, 15, 17 | `PB01`..`PB04` (Port B) |
| 21, 23, 25, 27 | `PC01`..`PC04` (Port C) |
| 31, 33, 35, 37 | `PD01`..`PD04` (Port D) |
| 9, 19, 29, 39 and all even pins | GND |

Each line carries a 22 ohm series resistor (`R104`-`R107`) and an `RCLAMP3324P`
TVS array per port (`U4`-`U7`). A ground return beside every signal is what makes
20 MHz on a ribbon plausible at all.

**The FPGA is the level shifter; there is no separate buffer.** The sixteen port
nets `IO00`..`IO15` land directly on ICE40 **BANK0**, whose `VCCIO_0` is
`VIO_VAR` - the board-generated, PWM-adjustable target IO rail (`LEDC` on
**GPIO 16**, `INITIAL_VIO_DUTY 8` for about 3.30 V, table for 3.3/2.5/1.8/1.5/1.2
V, with an ADC reading it back). The S3 talks to BANK1 and BANK2, both 3V3. So
"specify the buffer and its direction control" in Phase 0 is really "BANK0 is the
buffer, and its output enable is in the bitstream". For a TC38x, whose DAP pads
sit in the 3.3 V `VEXT`/`VDDP3` domain, the board's default rail is already
correct and no translation part is needed.

**Not every port pin can drive.** Joining the schematic's net-to-ball map against
the `PINTYPE` directions from the bitstream gives, for BANK0:

| port | pins | drive capability in the stock bitstream |
|---|---|---|
| A | `PA01`, `PA03`, `PA04` | **input only** - cannot drive |
| A | `PA02` | bidirectional |
| B | `PB01`, `PB03` | **input only** |
| B | `PB02`, `PB04` | bidirectional (UART TX, SReset) |
| **C** | `PC01`..`PC04` | **all four bidirectional** |
| D | `PD01`..`PD04` | all four bidirectional |

That explains a line in the firmware that otherwise looks like a leftover:
`main.c:1214` reads `bool b_use_porta = false; //gbl_pa_cfg == PA_BMP_SWD_JTAG;`.
Port A's BMP mode is disabled in software because the stock bitstream physically
cannot drive three of its four pins. **Port C is the only port that can carry a
bidirectional debug interface**, which is also why `AEL_DEFAULT_PC_CFG` is 1.
Any DAP wiring goes to Port C.

**The S3-to-FPGA map is fully recovered** and matches the bitstream's directions:

| net | S3 GPIO | ICE40 pin | bank | direction per bitstream |
|---|---|---|---|---|
| `SWDIO` | 41 | 45 | 2 | bidirectional |
| `SWCLK` | 47 | 48 | 2 | input to FPGA |
| `RDnWR` | 45 | 46 | 2 | input to FPGA |
| `TDI` | 40 | 44 | 2 | input to FPGA |
| SPI `SCK`/`SI`/`SO`/`SS` | 38 / 14 / 39 / 21 | 16 / 15 / 14 / 17 | 1 | in, in, **out**, in |
| `creset_b` / `CDONE` | 46 / 42 | 8 / 7 | config | - |
| `ESP32TXD` / `ESP32RXD` | - | 21 / 20 | 1 | in, **out** |
| `TEST3` | - | 47 | 2 | **unconfigured** |

`SWDIO` being the one bidirectional pin in that group, and `SPI_SO` and
`ESP32RXD` being the only outputs, is exactly what the signal directions demand.
Nothing in the two sources disagrees.

### The mux, recovered from the bitstream - the pinout gate is closed

The last unknown was *which* of `PC01`..`PC04` the stock bitstream connects to
`SWCLK`, `SWDIO`, `TDI` and `TDO`. That is answered, and without a scope:
`icebox_vlog` recovers a flattened netlist, and tracing dependencies backwards
from each pad's tristate driver to the module's input ports says exactly which
ESP32-S3 pin reaches which package pin. `tools/bitstream/trace_nets.py` does the
trace; the one-line `SHIFTREG_DIV_MODE` fix noted above is needed first.

Every pad the design can drive, and what drives it:

| package pin | driven combinationally by | notes |
|---|---|---|
| 32 | `SWCLK` / GPIO 47 | output only |
| 31 | `SWDIO` / GPIO 41 | **bidirectional** |
| 28 | `TDI` / GPIO 40 | output only |
| 42 (`PA02`) | `TDI` / GPIO 40 | Port A's single drivable pin |
| 34 | nothing from a debug GPIO | register state only |

Joined with the connector map, Port C in BMP mode is:

| connector | pin | stock signal | S3 side | capability |
|---|---|---|---|---|
| `PC01` | 34 | register-driven output | config frame on CS3 | slow output only |
| `PC02` | 32 | `SWCLK` / `TCK` | GPIO 47 | output only |
| `PC03` | 31 | `SWDIO` / `TMS` | GPIO 41, direction GPIO 45 | **bidirectional** |
| `PC04` | 28 | `TDI` | GPIO 40 | output only |

**Two findings that shape Phase 1.**

First, **the turnaround control is already in the bitstream**. The output enable
of pin 31 *and* the output enable of the S3-facing pad both depend
combinationally on GPIO 45 - `RDnWR`. Pins 32 and 28 share a register-controlled
enable instead, so they are outputs and nothing more. Phase 1 does not have to
build a direction mechanism; it has to drive one that already exists.

Second, **pin 31 to GPIO 41 is the only target-to-S3 data path in the entire
design.** Everything the target says has to come back on GPIO 41. Note in
passing that `AEL_BMP_TDO_PIN 15` in `board_profile.h` corresponds to no FPGA
connection at all, so JTAG's TDO cannot be read there either.

**Therefore DAP1 must be wired to `PC03`**, since it is the one signal that both
reads and writes; DAP0 belongs on `PC02` or `PC04`; and `PC01`, being slow and
register-driven, is the natural home for `TRST` or a parked DAP2. Wide mode is
impossible on the stock bitstream regardless: DAP2 would need a second
bidirectional line and there is only one.

This constraint belongs to the *stock* bitstream alone. All four Port C pins are
physically bidirectional, so once Phase 2 supplies its own bitstream any
assignment works.

### Target rates

| consumer | needs |
|---|---|
| three signals at 20 kHz | ~400 kB/s |
| six signals at 20 kHz | ~800 kB/s |
| flashing, run control, register access | latency-bound, not rate-bound |

### What this board already has that fits

- **Lattice ICE40UP5K**, already clocked at 132 MHz with a 264 MHz sampling path,
  reached over SPI2 from the ESP32-S3 through the two frames that exist - the
  logic-analyser register frame on CS0 and the config frame on CS3. Not through
  `ICE_FPGA_Serial_*` or `ICE_PSRAM_*`; see blocker 2.
- **A hardware serialiser in the S3 itself.** GP-SPI does LSB-first, arbitrary
  bit lengths and DMA, which is what a DAP frame needs and what bit-banged GPIO
  cannot give. See "where the PHY belongs" below.
- **A direction-controlled data pin.** `AEL_BMP_SWDIO_RDNWR_PIN` (GPIO 45) already
  drives an external buffer's direction, which is exactly what DAP1's half-duplex
  turnaround needs.
- Pins for the rest: `AEL_BMP_SWCLK_PIN` 47 to DAP0, `AEL_BMP_SWDIO_PIN` 41 to
  DAP1, and TDI 40 / TDO 15 free for DAP2/DAP3 if wide mode is wanted later.
- Port muxing, a web UI to select modes, and an existing CMSIS-DAP/BMP stack to
  sit beside.

### What does not fit, and where the PHY belongs

CherryDAP bit-bangs its pins at `DAP_DEFAULT_SWJ_CLOCK` 1 MHz; the board default
is `AEL_BMP_DEFAULT_FREQUENCY_HZ` 4 MHz. The pads want 4 ns of data setup at
160 MHz, relaxing to 5 ns at 40 MHz, with 2 ns of hold; bit-banged GPIO on an
ESP32-S3 cannot place an edge that precisely at any useful clock rate. The
numbers are tabulated in "Timing and MAXWAIT8" below.

**The argument is rate, not continuity** - an earlier draft of this document said
both, which was wrong and would have made Phase 1 look impossible. DAP is
entirely probe-clocked and synchronous: the device advances on `DAP0` edges, so a
gap *anywhere* costs nothing, including inside the reply window. That last part
was the open question in earlier drafts and it is now settled from the
documentation: `MAXWAIT8` counts **DAP0 clock cycles the probe itself supplies**,
not device time, so a probe that stops clocking freezes the timeout counter
rather than racing it.

**But the S3 does not have to bit-bang, and an earlier draft's conclusion - that
the PHY must therefore go in the FPGA - does not follow.** GP-SPI on this part is
a hardware serialiser with everything a DAP frame wants: LSB-first
(`SPI_DEVICE_BIT_LSBFIRST`), a transfer length expressed in *bits* rather than
bytes, full-duplex sampling, and DMA. A 1 kB `client_blockread` is 256 parcels of
32 data bits plus a 1-bit ACK - about 1056 bytes on the wire - which is **one DMA
transaction, roughly 0.42 ms at 20 MHz, no CPU in the loop and no interrupts
disabled**, with the 33-bit units realigned in software afterwards. That is the
right Phase 1 PHY, and it removes the reason the earlier draft wanted a
2 ms critical section for a single block read.

Three things SPI cannot express. None of them is fatal, and the timing spec is
the reason:

- **No GPIO flip mid-transaction.** `DAP1`'s turnaround needs `RDnWR` (GPIO 45)
  to change direction inside the frame, and no SPI phase drives it, so the frame
  splits into command-out, flip, reply-in. Earlier drafts called the gap between
  those two transactions the single risk that decided the project. **It is free.**
  The timeout counts probe-supplied clocks, so a gap with the clock stopped does
  not advance it, and the device's internal SRI access continues in real time
  meanwhile - a pause strictly improves the odds that data is ready when clocking
  resumes. Phase 1 confirms this on silicon rather than discovering it.
- **No length that depends on a sampled bit.** Busy-wait and the reply's start
  bit cannot steer the transfer, so the reply window is over-clocked at a fixed
  length and parsed afterwards. Busy stuffing is `DAP1` held low and the start
  bit is a one, so the parse is "skip zeros, find the first one" - see the sizing
  rule below, which is the one piece of arithmetic this design has to get right.
- **No help with the analogue end.** Setup and hold are easy at 20 MHz; what is
  not guaranteed is edge placement through the GPIO matrix and a level shifter,
  so 20 MHz remains the honest target for an S3-hosted PHY even though the pads
  go to 160.

So Phase 2's case is narrower than it was, and worth stating plainly: not "the S3
cannot clock fast enough" and no longer "the turnaround may not survive", but the
pin count and per-line phase alignment wide mode needs, and above all **Phase 4 -
draining the TRAM without a host round trip per read**, which is the thing that
actually fixes the original problem. Phase 2 can reasonably be deferred until
Phase 4 needs it.

### Timing and MAXWAIT8, from the DAP spec

The numbers below come from the TC38x DAP tolerance and `MAXWAIT8` specification,
and they change this plan's conclusions rather than decorating them. Everything
in this section is quoted or derived from that document; where it contradicts an
earlier draft of this plan, it wins.

**AC timing at the pads.** Clock period `t11` is 6.25 ns minimum, so the pads
themselves reach 160 MHz; high and low pulse widths are 2 ns minimum.

| parameter | at 40 MHz | at 80 MHz | at 160 MHz |
|---|---|---|---|
| `t16` data setup, probe to device | 5.0 ns | - | 4.0 ns |
| `t17` data hold, probe to device | 2.0 ns | 2.0 ns | 2.0 ns |
| `t14`/`t15` clock rise/fall, max | 4.0 ns | 2.0 ns | 1.0 ns |
| `t19` device data valid delay, max | 10.0 ns (C_L 50 pF) | 8.0 ns (20 pF) | 4.0 ns (20 pF) |

`t19` is a *delay measured from the falling edge of `DAP0`*, not a valid window -
an earlier draft of this plan had that wrong. It is what sets the sampling
margin: at 20 MHz the half period is 25 ns, so sampling the next rising edge
leaves about 15 ns of margin over a 10 ns valid delay. That is comfortable, and
it is why an S3-hosted PHY at 20 MHz is a reasonable proposition rather than a
stretch. Downstream, 5 ns of setup and 2 ns of hold at 40 MHz are met by any
hardware serialiser; only bit-banged GPIO fails them.

**`MAXWAIT8` is a count of probe clocks.** During an SRI or SPB access the device
enters `Active::SEND` and stuffs `DAP1` low until data is ready. The probe must
keep clocking while that happens, up to

```
  T_timeout = MAXWAIT8 * 8 * (1 + MW8E * 15)     DAP0 clock cycles
```

from `DAPISC` (DAP address `0x11`): `MAXWAIT8` is bits 12:8, `MW8E` is bit 13, so
the step is 8 clocks with `MW8E` clear and 128 with it set, and the range runs
from 8 clocks to 3968. `MAXWAIT8 = 0` disables the timeout, which means an
internal bus lockup hangs the probe indefinitely - do not.

Three consequences, and they are the load-bearing part of this document:

1. **Gaps are free.** The counter advances only on clocks the probe supplies, so
   stopping the clock - to flip `RDnWR`, to let a DMA transaction end, to service
   an interrupt - freezes it. The turnaround gap costs nothing, and neither does
   an interrupt landing mid-frame. **Nothing in Phase 1 needs a critical
   section**, which retires the 2 ms interrupts-off problem entirely rather than
   working around it.
2. **The timeout stops at the start bit.** On TC3xx the counter monitors only the
   interval up to the upstream start bit; once the device drives that one, the
   counter is deactivated and the payload can be clocked out with no timing
   constraint at all. (On AUDO MAX it covered the whole transaction - relevant
   only if porting older probe code.)
3. **Block transfers re-arm per parcel.** The limit is restored after every
   acknowledged parcel, and a stall aborts the block back to `Active::RECEIVE`
   without locking the interface, leaving `IOINFO` readable for diagnosis. So a
   long block read is a sequence of independent 248-clock windows, not one.

**Sizing the reply transaction**, which is the arithmetic the SPI PHY has to get
right. At the `DAPISC` reset value the allowance is 248 clocks, so a single word
read needs a reply transaction long enough for 248 wait bits plus the start bit,
32 data bits and CRC6 - about 287 bits, so allocate 36 bytes and parse. Size it
shorter and a slow access looks like a timeout that is really a truncated read.

For a 1 kB block the worst case is 248 clocks *per parcel*, which would be about
9 kB of transaction if sized for the worst case throughout. Do not: issue a
transaction sized for the expected case, and if the parse shows the block did not
complete, issue a further read-only transaction to keep clocking. That is safe
precisely because of consequence 1 - the frame is still open, and the gap between
transactions does not count against anything.

**Cold attach: leave `DAPISC` alone.** Its reset value is `0x1F00` - `MAXWAIT8`
31, `MW8E` 0, exactly 248 clocks - and that generosity is deliberate. Before the
startup software brings the PLLs up, the target runs on an untrimmed backup
oscillator with wide thermal and voltage drift, and a small timeout there NACKs
every transaction. Tune `MAXWAIT8` down for throughput only after the application
has established its clocks, and never during bring-up.

**Wide mode needs per-line phase alignment.** The silicon does not guarantee
matching skew between `DAP1` and `DAP2` outputs, so a probe running wide mode has
to shift its capture point per line, calibrating against the `0xAAAAAAAA` pattern
that `sync` returns. That is an FPGA job - a fixed SPI sampling point cannot do
it - and it is now the clearest reason wide mode belongs to Phase 2 and later
rather than to the S3.

One caveat on provenance: this section is derived from a specification document
supplied to the project rather than from a datasheet in this repository. The
claims that carry the most weight are the clock-counted timeout and the start-bit
deactivation, so confirm both against Infineon's own DAP specification before the
schedule leans on them - Phase 1 will confirm them on silicon regardless.

### Throughput budget

`client_blockread` moves up to 256 words (1 KB) per command with the start
address issued by the DAP controller itself, streaming 32-bit parcels with a
1-bit ACK, so roughly 33 bits per 4 bytes:

| stage | rate |
|---|---|
| ESP32-S3 bit-banged DAP, ~4 MHz | 150-250 kB/s realistic |
| **S3 GP-SPI DAP with DMA @ 20 MHz, two-pin** | **~2.4 MB/s at the pins** |
| FPGA DAP @ 20 MHz, two-pin | ~2.4 MB/s at the pins |
| FPGA DAP @ 40 MHz, two-pin | ~4.8 MB/s at the pins |
| S3 to FPGA over SPI, 10 MHz, one blocking transfer at a time | ~1.1 MB/s |
| the same SPI at 20 MHz, noted as working in `ice.c` | ~2.2 MB/s |
| the same SPI at 40 MHz queued, the GPIO-matrix limit | ~3-5 MB/s |
| ESP32-S3 USB, full speed | ~1 MB/s |

**The SPI hop to the FPGA is today's ceiling, not the hardware's.** `ice.c`
configures the device at 10 MHz - with `//It works at 20M!` commented directly
above it, `ice.c:185` - and moves everything with `spi_device_transmit`, one
blocking transfer at a time, capped at `ICE_SPI_MAX_XFER` 4096 bytes;
`read_and_return_capture` reads 128 kB as 64 separate 2048-byte transfers. DMA is
already enabled on the bus (`spi_bus_initialize(SPI2_HOST, ..., SPI_DMA_CH_AUTO)`,
`ice.c:200`), so the DMA engine moves the bytes while the calling task waits on
each transaction and the bus idles between them.

Raising it is configuration work, not new hardware: queue two or more
transactions with `spi_device_queue_trans`/`spi_device_get_trans_result` instead
of blocking, raise `max_transfer_sz`, and raise the clock. The limit is that
GPIO 38/39/14/21 are not this part's FSPI IOMUX pins, so the bus runs through the
GPIO matrix, which caps a master at about 40 MHz and needs `input_delay_ns` set
above roughly 26 MHz or reads corrupt. Call it 3-5 MB/s reachable, gated by
timing closure on the FPGA side. Phase 2 measures it rather than assuming either
number.

**Note this only matters for the FPGA path.** An S3-hosted SPI PHY talks to the
target directly and never crosses this hop at all.

Wide mode, if it is ever used: `DAP1` carries the even bits and `DAP2` the odd
ones, the start bit is driven on both lines in parallel as an alignment check,
and any field with an odd bit count is padded by one bit which *is* covered by
the CRC6. The device ignores the padding except on `CMD`, where the pad bit must
be `0`. Note the skew caveat in the timing section: the two upstream lines are
not guaranteed to match, so wide mode needs a per-line capture phase calibrated
against `sync`'s `0xAAAAAAAA`, which is FPGA work.

**Host link: WiFi for streaming, USB for bring-up.** USB full speed caps near
1 MB/s, which is under the six-signal case's 800 kB/s once framing is counted, so
WiFi is the streaming link and USB is the bring-up and no-network fallback. It is
not a wide margin either way: the six-signal case is the first thing that makes
the choice matter, and it is the case worth measuring rather than arguing. An
earlier draft warned that the firmware defaults to AP-only and would drag the
host off its own network; that is a property of `sdkconfig.defaults` and not of
the board as it runs. Measured on the bench, it comes up in station mode and
takes a DHCP lease on the lab network, so the concern is retired.

**Where the trace is buffered, given the memory that exists.** An ICE40UP5K has
128 kB of SPRAM plus roughly 15 kB of block RAM, and there is no FPGA-attached
PSRAM on this board - the part the upstream code targets is absent, and the
capture buffer is ESP32 SPIRAM. The stock design spends the SPRAM on
logic-analyser capture, so under the two-bitstream approach the question answers
itself: **in trace mode the logic analyser is not loaded and all 128 kB is the
trace ring**, worth about 160 ms at 800 kB/s.

That is enough to ride out host-side stalls but not a sustained shortfall, so the
design still needs to drop and mark rather than pretend: a gap marker is
something the decoder already handles, and the drain-side recovery protocol is
documented below.

**The S3 side has far more room, and it is measured rather than assumed.** The
module is an `ESP32-S3-WROOM-1-N16R8`: 16 MB flash, 8 MB PSRAM, and the running
firmware reports `[APP] Free memory: 8102852 bytes` after the logic analyser has
already taken its 131072-byte capture buffer from PSRAM. So roughly 8 MB is
available for a trace ring - about **ten seconds at 800 kB/s**, against 160 ms in
the ICE40's SPRAM. For the Phase 1 shape, where the PHY is in the S3 and there is
no SPI hop in the path at all, that ring absorbs any plausible WiFi stall
directly. For the Phase 2/4 shape it sits behind the SPI hop and so absorbs host
jitter only, which is the case worth measuring before sizing anything.

Note also that Phase 1 alone, at 150-250 kB/s bit-banged or several times that
through GP-SPI, is already well past the 38 kB/s this project set out to beat. It
is worth shipping on its own merits, not only as a stepping stone.

### Architecture

Two shapes, and Phase 1's is not a cut-down version of Phase 2's - the FPGA is
simply not in the path:

```
  Phase 1, PHY in the S3:

  TC38x  <--DAP0/DAP1-->  ESP32-S3                          host
                          +-----------------------+
                          | GP-SPI + DMA, LSB-first|
                          | frame assembly, CRC6   |<--WiFi-->  tas_debug
                          | RDnWR turnaround       |   (USB     adapter
                          | ring in SPIRAM         | fallback)
                          +-----------------------+

  Phase 2/4, PHY and TRAM drain in the FPGA:

  TC38x  <--DAP0/DAP1(/DAP2)-->  ICE40UP5K            ESP32-S3          host
                                 +-------------+      +----------+
                                 | DAP master  |  SPI | command  |
                                 | CRC6        |<---->| layer    |<-WiFi->  tas_debug
                                 | parcel FIFO |queued| ring in  |  (USB    adapter
                                 | trace ring  |  DMA | SPIRAM   | fallback)
                                 +-------------+      +----------+
```

The FPGA-side SPI is not a new bus: it is the existing link, reached through the
logic-analyser register frame (four bytes over `gbl_spi_h1`, CS0) and the
three-byte config frame (`spi_device4_transfer_data`, CS3). `ICE_FPGA_Serial_*`
and `ICE_PSRAM_*` are compiled out and must not be built on.

**Which SPI host the S3-hosted PHY uses.** SPI2 is the FPGA link (`ice.c:45`) and
SPI3 carries the LCD and XVC (`esp32jtag_common.h:7`, `main.c:246`), so on paper
both are taken. In practice neither the LCD nor XVC is wanted while the board is
acting as a DAP probe, so **SPI3 is the PHY's bus**: add the DAP device to it
with its own CS, and have the DAP mode take the host while LCD and XVC give it
up. That is a mode rule to enforce in firmware rather than a resource conflict to
solve, and it is the same exclusivity the two-bitstream approach already assumes
of the logic analyser.

The host seam is wider than an earlier draft of this document claimed. Memory
access is eight methods - `read`, `write`, `read32`, `write32`, `write64`,
`read_many`, `write_many`, `execute` - but callers also use `server_connect`,
`get_targets`, `session_start`, `device_connect`, `disconnect` and `.connected`,
and `execute()` sizes its batches from `con_info.pl0_max_num_rw`,
`max_pl2rq_pkt_size` and `max_pl2rsp_pkt_size`, so a probe has to report
equivalents of those or the batching arithmetic breaks. `TasClient(...)` is also
constructed directly at 24 sites in 18 files across both tools - nine of them in
shipped `src/`: the DAP adapter, the flash and UCB tools, the trace sources, the
MCDS recovery path, with the rest in tests and `tas-trace/scripts/` - and there
is no injection point anywhere.

So the work is an adapter *plus* a factory with an environment or config switch,
and small edits at every construction site. Still modest, but it is a change in
both repositories rather than "no changes to either".

### Bring-up checkpoints

Every step below has a known answer, which is what makes the early phases cheap
to debug:

| step | expected reply |
|---|---|
| 8 slow clocks, then `sync` (CMD 0x10, LEN 63, CRC 9 - the wire word `0x09FE1`) | `0xAAAAAAAA` |
| `dapisc` (CMD 0x11, LEN 16; LEN 48 with signature `0x4ABBAF53` when cold) | the register echoed back |
| `client_set(1)` (CMD 0x1C, LEN 3, DATA 1) | start-bit acknowledge |
| `client_read(IO_CLIENT_ID)` (CMD 0x1A, instruction `FH`) | **`0x0260`** |
| `client_write(IO_SET_ADDRESS)` then `client_read(IO_READ_WORD, 32)` | a word of target memory |

`CLIENT_ID` reading `0x0260` is the moment the probe is real. Halting the target
is then one write: `client_write` of `0x0003` to `OJCONF` (local offset `0x0E`,
instruction `EH`) sets HARR with its paired write-protect bit.

The `dapisc` step reads the register; it should not *change* it during bring-up.
The reset value `0x1F00` gives the full 248-clock wait allowance, which is what
makes a cold attach survive the target's untrimmed backup oscillator - see
"Timing and MAXWAIT8".

The read frames are fully pinned. A `client_read` payload is `LEN = 7`: a 4-bit
IO instruction followed by a 3-bit size exponent, both LSB-first. There is a
separate instruction per width, and the whole payload is one byte:

| read | instruction | exponent | payload bits, LSB-first |
|---|---|---|---|
| 8-bit | `IO_READ_BYTE` `9H` | 3 | `0x39` |
| 16-bit | `IO_READ_HWORD` `7H` | 4 | `0x47` |
| 32-bit | `IO_READ_WORD` `5H` | 5 | `0x55` |

Block transfers select their address form by `LEN` alone - `10` for none, `24`
for a 14-bit address, `40` for a 30-bit one - and the address is transmitted
right-shifted by two, since every block access is word-wide. The 14-bit form
therefore addresses within a 64 kB window and the 30-bit form the whole space.

### Phases

#### Phase 0 - Electrical and pin plan (2-3 days)

Map DAP0/DAP1 (and optionally DAP2/DAP3) onto the port pins, specify the buffer
and its direction control from GPIO 45, decide target voltage domain and
protection, and bring out TRST plus, if the target board allows, PORST. Capture a
reference trace of a working miniWiggler attaching to the same target, to compare
framing against later.

The FPGA is **already in the drive path**: `data_reg_0` bit 6 (`SWD_GPIO`)
selects whether SWCLK/SWDIO come from the S3's GPIO or from the FPGA's SPI2JTAG
block (`esp32jtag_common.h:58,66`). So this phase verifies buffer direction and
levels; it does not have to invent a way for the FPGA to reach the target pins.

Confirmed on the running board rather than inferred: the boot log shows
`set_cfga: global_data_reg_0=0xc3 ... jtag_swdio=1, swd_gpio=1` with `cfgpc=1`,
so the pass-through from S3 GPIO to Port C is the live configuration, and
`rx=0x00 80 c3` shows the CS3 config frame reading back. The path Phase 1 needs
is already up.

Since Phase 2 is now gated on the pinout rather than on obtaining HDL, extract
it here - but note the gap is narrower than it looks, because **the ESP32 side is
already in the source**:

| net | S3 pin | where |
|---|---|---|
| FPGA SPI SCK / MISO / MOSI | 38 / 39 / 14 | `ice.c:54-56` |
| CS0, logic-analyser frame and bitstream load | 21 | `ice.c:57` |
| CS3, config frame (`data_reg_0/1`) | 5 | `main.c:310`, `board_profile.h:35` |
| CRESET / CDONE | 46 / 42 | `ice.c:58-59` |
| SWCLK / SWDIO / RDnWR / TDI / TDO | 47 / 41 / 45 / 40 / 15 | `board_profile.h:26-30` |

The **ICE40 ball assignment** is no longer unknown either: icestorm reads it out
of `bitstream.bin` - 35 used pins with directions, four unconfigured (13, 18, 19,
47) - and the schematic supplies the net names for them. Both readings agree; the
result is tabulated under "What the schematic adds". Two conclusions fix this
phase's shape:

- **The target goes on Port C, on header J3, and nowhere else.** It is the only
  port whose four pins are all bidirectional in the stock bitstream. Signals on
  `J3` odd pins 21, 23, 25, 27; ground on any even pin, and 9/19/29/39.
- **No level shifter is needed.** BANK0 runs on `VIO_VAR`, the board's own
  adjustable target rail, which defaults to about 3.30 V - already the TC38x's
  `VEXT`/`VDDP3` domain.

So the only pin question left for the bench is **which of `PC01`..`PC04` the
stock bitstream maps `SWCLK`, `SWDIO`, `TDI` and `TDO` onto**. Drive one signal
at a time from the S3 and scope the four header pins; a scope is already on the
bench. Do this before soldering anything, because a wrong name is silent, and ask
for `la_src/` (blocker 1) if you would rather read it than measure it.

Exit: schematic delta agreed, reference capture stored in `docs/`, and the Port C
signal-to-pin map confirmed on the bench.

#### Phase 1 - DAP on the ESP32-S3, bit-bang then GP-SPI (1-1.5 weeks) - the deliverable

More than a stepping stone, and after the GP-SPI finding it may be most of the
project. It is also where every protocol value in this document meets silicon for
the first time, so a failure here is a protocol failure, cleanly separated from
the RTL and pinout risks of Phase 2. Ship it before starting Phase 2 even if
Phase 2 is certain to happen.

**1a, framing by bit-bang at 1-2 MHz.** Slow enough to single-step: start bit,
CMD(5), LEN(6), DATA, CRC6, trailing zero; busy-wait detection on the reply;
turnaround; the hot-attach sequence. Reuse CherryDAP's pin layer and the existing
direction pin. **No critical sections and no core pinning** - the timeout counts
probe clocks, so an interrupt landing mid-frame merely stops the clock, and an
earlier draft's 2 ms interrupts-off block read was solving a problem that does
not exist. Leave `DAPISC` at its reset `0x1F00` throughout.

**1b, confirm the two spec claims that the design leans on.** Not a gamble any
more, but still the first thing to check on silicon, because everything after it
assumes both: that a stopped clock does not advance the timeout, and that the
timeout releases at the upstream start bit. Test the first by inserting a
deliberate delay - milliseconds, not microseconds - between the command and reply
transactions while `RDnWR` flips, and confirming the read still returns data.
Test the second by clocking the payload out slowly and in pieces after the start
bit arrives. Compare against the miniWiggler reference capture from Phase 0.

**1c, GP-SPI with DMA at 10-20 MHz.** The real PHY: LSB-first, bit-length
transfers, one DMA transaction per direction, the reply over-clocked to cover the
248-clock allowance and parsed for its start bit afterwards, `RDnWR` flipped
between the two, and a continuation transaction issued when a block read does not
finish inside the expected length. A 1 kB block read is about 1056 bytes of DMA
in the expected case - roughly 0.42 ms at 20 MHz - with no interrupts disabled at
all. Use SPI3, taking the host from the LCD and XVC for the duration of DAP mode.

Exit: all five bring-up checkpoints pass, including a word read back from a known
address; 1b's two claims confirmed and written down; and a 1 kB block read
running through DMA at a measured rate, including one that needed a continuation
transaction. The CRC6 rule is settled on paper, so a silent target here means a
framing or timing fault rather than a checksum one.

#### Phase 2 - A second, DAP-only bitstream (2-3 weeks, and possibly deferrable)

Schedule this against Phase 4 rather than ahead of it. With the turnaround
question answered from the spec, Phase 2's value is the autonomous drain, wide
mode's per-line phase alignment, and rate beyond 20 MHz - not the rate case
itself, which the S3 covers.

Two bitstreams, one loaded at a time: the stock one for logic analyser, XVC,
SPI2JTAG and the four-port mux, and a new one for tracing. The board switches by
calling `ICE_FPGA_Config` again with the other buffer. Nothing in the stock
design is read, modified or reproduced.

**What the board gives up while the DAP bitstream is loaded.** The stock design
gates all four ports through one register - `{cfgpc, SWD_GPIO, LA_input_sel,
cfgpd, cfgpb, cfgpa, njtag_swdio, sreset} = data_reg_0[7:0]`
(`esp32jtag_common.h:58-67`) - so the second bitstream does not only displace the
logic analyser:

- `CFGPB` gates Port B, which is the **target UART and SReset**. A trace session
  would lose the target console and the reset line unless the new design passes
  them through. Decide deliberately: watching UART while tracing at 20 kHz is
  exactly how this gets debugged.
- `SRESET` on Port B pin 3 is FPGA-generated, so GDB reset has no path in the new
  mode unless reimplemented.
- `CFGPA` and `CFGPC` gate the BMP and CMSIS-DAP ports, so debugging any *other*
  target through this board stops for the duration.
- The LCD and XVC are not gated by the FPGA but are given up anyway, since DAP
  mode takes SPI3 for the PHY. That is intended, not a casualty.

**And the config frame has to keep working.** `set_cfga`, `set_sreset`,
`set_portd_output` and `set_la_input_sel` all write `data_reg_0/1` as a three-byte
frame on CS3 (`main.c:846,869,895,936`). Either the new bitstream decodes that
same frame, or the firmware gates those calls per mode - otherwise the writes
land in a design that does not answer them and the web UI silently does nothing.

**What the new design contains.** A DAP master - shift engine, CRC6, busy-wait
and turnaround handling, `MAXWAIT8` timeout and NACK recovery, parcel streaming
for `client_blockread`/`client_blockwrite` - an SPI slave with a register map we
define ourselves, a trace ring in SPRAM, and routing for one port's pins. It does
*not* contain capture, muxing or any other shipping feature, which is exactly why
this is three weeks rather than a rewrite. Simulate against a testbench modelling
the device state machine; the protocol is documented well enough to write one.

**Two consequences worth having.** All 128 kB of SPRAM is available in this mode,
because the logic analyser is not loaded - about 160 ms of buffering at 800 kB/s,
which answers the buffer question in the mode where it matters. And the stock
bitstream cannot regress, because it is never touched: the risk moves from "did I
faithfully reproduce five subsystems" to "did I get the pinout right", which is
testable.

**The remaining gate is net naming - not the HDL, and no longer the ball
assignment.** The bitstream gives the used pins and their directions; what it
does not give is which pin is which net. Needed: the configuration SPI (pins
14-17, with 14 the FPGA's output) and its chip select; CRESET and CDONE on their
dedicated pins; the port pins carrying DAP0, DAP1 and optionally DAP2/DAP3; the
buffer direction line; and any pin the stock design holds in a defined state that
the board or the target relies on - reset generation being the one to look for
first. The 12 bidirectional pins are the candidate set for the port lines, and
13, 18, 19 and 47 are free. Confirm on the bench with a pattern-generator
bitstream and a scope before trusting it; a wrong guess here is silent.

**Mode-switch hazards to design for.** During CRESET-low and configuration the
ICE40's I/Os are tri-stated with weak pull-ups, so DAP0/DAP1/TRST can twitch and
disturb an attached target. Park the buffers before switching - GPIO 45 controls
direction, and it is worth checking whether an output-enable exists as well.
Worst case a switch costs the debug link, which hot attach re-establishes. A
failed load must fall back to the stock bitstream rather than leaving the FPGA
unconfigured; the existing three-retry loop is the place to add that.

Also decide here where the trace is buffered, and measure the SPI hop's real
throughput and worst stall rather than assuming 10 MHz x 8 bits. Raising that hop
is firmware work on the S3 side and belongs in this phase: queued transactions
rather than one blocking transfer at a time, a larger `max_transfer_sz`, a higher
clock, and `input_delay_ns` set once the clock passes roughly 26 MHz.

Exit: the five bring-up checkpoints at 20 MHz through the new bitstream, a 1 kB
block read verified against `get_CRCup`, a measured end-to-end rate through the
SPI hop, and a mode switch in both directions that leaves the stock features
working and the target undisturbed.

Cost of the approach: two bitstreams mean two test matrices, and the open
toolchain (yosys, nextpnr-ice40, icestorm) has to be set up and its timing
closure checked at 20-40 MHz. Neither is on the critical path, and the icestorm
half is already installed and exercised in the devcontainer. Budget for matching
the board's clock arrangement as read off the PLL - a reference near 48 MHz
feeding a 132 MHz fabric clock - unless the new design brings its own.

#### Phase 3 - Cerberus access layer and OCDS enable (3-5 days)

Thin, now that the register map is known: `IOADDR`/`IODATA` with the 4-bit IO
instructions for single-word access, `OJCONF` for halt and reset, `IOINFO` for
bus error diagnosis, and `client_reset` to clear a locked transaction.

The part that is easy to miss is enabling OCDS, without which run control does
not work and the whole miniMCDS register space bus-errors. See "OCDS enable
sequence" below; the vendor tool does exactly this, and its own configuration
list starts with the four pattern writes.

Exit: memory read/write and halt/resume driven from the host, `OSTATE.OEN` reads
back as 1, and a read of a miniMCDS register returns data instead of a bus
error.

#### Phase 4 - Autonomous TRAM drain (1-1.5 weeks)

The part that actually fixes the original problem. The FPGA follows the trace
write pointer and empties the buffer without a host round trip per read, and
signals overrun explicitly rather than lapping silently - a gap marker in the
stream is something the decoder already handles, whereas a silent hole is what
produces corrupt samples.

Exit: three signals at 20 kHz captured with no dropped paragraphs and no decoder
resyncs, sustained over a minute.

#### Phase 5 - Host integration (1-1.5 weeks)

Adapter behind the `tas_debug` client interface, plus the factory and switch the
construction sites need, plus `con_info` equivalents for the batching arithmetic.
Then point the existing trace tooling at it.

Two things to settle before promising anything here:

- **Flashing needs 64-bit writes** (blocker 3). Test a dword access through
  Cerberus early; if there is none, the flasher stays on DAS and only trace and
  run control move over.
- **The MCDS session needs splitting.** `McdsSession.open()` does
  `ensure_server()`, library init *and* a device connect, and only one probe can
  own DAP0/DAP1 at a time - so the new probe and the miniWiggler cannot both be
  attached while the configuration is generated. The libraries themselves do not
  need a device: building a configuration with `configure_data_trace()` and
  reading it back with `config_writes()` works with nothing connected, which is
  how the 54-write list in this document was captured. So the split is a wrapper
  change, not a library problem - the one detail to carry is that
  `mcdscl_lib_init` and `mcdsdl_lib_init` currently run only inside `open()`
  behind `_LIBS_INITIALISED` (`mcds/session.py:225-234`), so a no-device path has
  to call them itself. What is still untested is whether the *decoder* accepts a
  configuration built with no device attached; if it does not, 6a stops being
  optional.

Note DAS here is Linux-only (`/opt/Tools/DAS/8.3.0/lib`, and the server helper
uses `fcntl`), so whichever host talks to the probe over WiFi has to be that
devcontainer.

Exit: the existing validation harness reports complete capture at a publish
divider of 1, with flashing either working or explicitly out of scope.

#### Phase 6 - Own MCDS configuration and decoder (optional)

Phases 0-5 leave the host still using Infineon's MCDS libraries, which by then do
no hardware access at all: they compute a register list and decode bytes. Two
further steps remove even that, and they are worth very different amounts.

**6a - table-driven configuration (2-4 days, recommended).** A configuration is
**54 register writes to 47 distinct addresses, and the order and the repeats are
part of it.** Three addresses are written more than once, and one of them is the
OCDS enable pattern:

```
0xF0000478 <- 0xA1, 0x5E, 0xA1, 0x5E     OEC.PAT, the OCDS enable sequence
0xFB718210 <- 0x8800, 0x4400, 0x8800, 0x4400    FIFOCTL stop then clear, twice
0xFB718A64 <- 0x00, 0xEFFF
```

So the table is an ordered list, not a map. Recording it into a dictionary keyed
by address - which is how this was first measured, giving a count of 47 - keeps
only the last value per address and would silently drop OCDS enabling entirely.

The writes barely move with the parameters:

| change | registers that differ, of 47 |
|---|---|
| address filter | 2 - `0xFB71A400` base, `0xFB71A404` mask (span-1) |
| trace mode (`obdtm`) | 1-2 around `0xFB7188EC` |
| timestamps on or off | 2 - `0xFB718880`, `0xFB7188D4` |
| buffer size 4 kB vs 8 kB | 0 - bounds come from FIFOBOT/FIFOTOP instead |
| suspend-on-full instead of gaps | 13 |

The list is deterministic: five successive calls and a fresh library instance
produce byte-identical output, and generating it needs no target, because the
configurator is a pure host-side computation. The writes land in three blocks -
`0xFB71xxxx` (38, MCDS), `0xB80000xx` (6, trace RAM), `0xF00004xx` (3, SCU) - all
reachable through Cerberus, and `TramStreamer.start()` already applies them over
the debug link today. So this is recording a table, computing three words from
the parameters, and a CI test that regenerates the list with the vendor library
and diffs it so drift is caught rather than discovered on a bench. Note the table
is per device type.

**6b - own trace decoder (1.5-2.5 weeks, only for licensing reasons).** The
decoder cannot be fed from a register table: it takes an opaque configuration
object from the configurator. Writing our own is now well defined for the subset
this project needs - DTU write accesses, TSU timestamps, `<skip>`,
`<endoftrace>`, `ERR` - and the encoding is recorded under "Trace stream
encoding" below.

There is one non-licensing reason to want it, in Phase 4's territory: a decoder
that understands paragraphs recovers from a lapped buffer by construction, where
the vendor decoder's opaque continuous state does not. Measured on the current
setup, it yielded 75 messages/s and stayed stuck on its first error until the
state was thrown away, then 589/s.

Total: roughly 5-7 weeks for one engineer through Phase 5, Phase 2 dominating;
6a adds a few days, 6b two to three weeks. The shape matters more than the total,
because the value does not arrive at the end: **Phase 1 alone is 4-6x the current
throughput bit-banged, and several times that again through GP-SPI with DMA**,
all of it with no FPGA work; Phase 3 makes run control and register access work;
and Phase 2 buys the autonomous drain that Phase 4 needs.

The total is probably smaller than that, because the timing spec removed the
reason Phase 2 had to come first. The S3's own PHY covers the rate case, Phase 2
shrinks to what Phase 4 and wide mode strictly require, and the FPGA work can be
scheduled on evidence rather than on the assumption this document opened with.
Phase 1b still runs first - two confirmations on a bench, cheap, and everything
after them assumes both.

### CRC6: generate with Galois seed 0x20, check with the manual's function

There are two functions in play and they are not interchangeable, which is where
every wrong turn here comes from.

**Checking** is the manual's `CalcCrc`: a right-shifting *Fibonacci* LFSR,
polynomial `0x03`, seeded `0x3F`. Shift the frame and then its CRC6 through it and
the state must end at zero. That function is correct and is what the silicon
implements.

**Generating** is not the manual's `GetCrc`. Over 302 payloads, the six bits that
`GetCrc` serialises leave a zero residue only 3 times; nor does the raw Fibonacci
state, its right shift, or its reversal. What does work, every time, is a
right-shifting **Galois** LFSR with polynomial `0x30` and **seed `0x20`**:

```
  crc = 0x20
  for each payload bit, LSB first:
      crc = (crc ^ bit) & 1 ? ((crc >> 1) ^ 0x30) : (crc >> 1)
  append crc as six bits, LSB first
```

| rule tested over 302 payloads | frames whose residue is zero |
|---|---|
| Galois, poly `0x30`, seed `0x20` | **302 / 302** |
| Galois, poly `0x30`, seed `0x3F` | 0 / 302 |
| Fibonacci state used directly | 13 / 302 |
| the manual's `GetCrc` | 3 / 302 |

The seed differing between the two topologies is the whole trick: Fibonacci `0x3F`
and Galois `0x20` are the same initial condition expressed in different state
spaces. Seeding a Galois implementation with `0x3F` - the obvious move, and one I
made - produces a checksum that is self-consistent and that the target rejects.

**Test vectors**, both verified to leave a zero residue under the manual's
checker:

| frame | payload, LSB first | CRC6 | full wire word |
|---|---|---|---|
| `sync`, `LEN = 63` | `CMD 0x10` + `LEN 63` | **9** | `0x09FE1` |
| `sync`, `LEN = 0` | `CMD 0x10` + `LEN 0` | **25** | - |

**All three now reproduce in code**, in `components/dap_probe/` with host tests
under `components/dap_probe/test/`. Two notes from writing it:

- **`0x09FE1` pins the bit packing, not just the checksum.** The frame is 19
  bits - start, `CMD` 5, `LEN` 6, `CRC6` 6, trailing zero - and the documented
  word only comes out if the *first bit on the wire is bit 0* of the integer.
  The other convention gives `0x23FC8`. So the vector doubles as a statement
  about byte order, which is worth having before an SPI peripheral packs it.
- **The checker has to be a genuinely different topology to be worth running.**
  Writing `CalcCrc` with a Galois step instead of a Fibonacci one fails every
  frame while the generator is perfectly correct - the same seed-versus-topology
  trap described above, met from the other side. The Fibonacci form (taps
  `0x03`, seed `0x3F`) zeroes on every valid frame and rejects all 135
  single-bit flips across the bring-up set. The Galois generator run over
  payload-plus-CRC also zeroes, and is a tempting shortcut, but an oracle that
  shares the generator's structure only proves self-consistency.

`sync` is the one command that goes out with `LEN` all ones, as a JTAG-TAP safety
measure while the pins may still be shared: the run of ones parks the TAP
harmlessly instead of risking a drift into instruction execution during a
hot-plug. Every other command uses its catalogued length, because after the
`dapisc` handshake the shared JTAG logic is bypassed on-chip. So the first frame
a probe ever sends is `0x09FE1`, and `poll`, `jtag_reset`, `get_CRCdown`,
`get_CRCup` and `client_reset` all use `LEN = 0` with a CRC computed the same way.

### OCDS enable sequence and miniMCDS access

Needed from Phase 3 onwards. Everything here is quoted from the vendor
documentation and corroborated against what the vendor tool actually emits.

**The locking trap.** The miniMCDS register space and the TRAM behind it are
accessible **only while `OSTATE.OEN == 1`**. Any read or write in
`0xFB718000..0xFB71FFFF` with OCDS disabled raises a bus error on the SRI slave
interface. Accesses must also be 32-bit single words - no bursts, no
read-modify-write, no unaligned access.

**The sequence**, after `client_set(1)` has selected Cerberus:

| step | write | meaning |
|---|---|---|
| 1 | `0xF0000478` (`OEC.PAT`) `= 0xA1`, then `0x5E`, then `0xA1`, then `0x5E` | the OCDS enable pattern |
| 2 | read `0xF0000480` (`OSTATE`) | bit 0 `OEN` must now be 1 |
| 3 | `0xF000047C` (`OCNTRL`) `= 0x0300` | `OC4` with its protection bit, which sets `OSTATE.EECTRC` and routes the core trace lines to the miniMCDS |
| 4 | `0xFB718010` (`CT`) `= 0x8000` | `SETE`, which unlocks writes to the rest of the miniMCDS space |

The four pattern writes must be contiguous: any other write to `OEC` in between,
or any deviation in the values, resets the hardware matcher to its first step.
Writing `1` to `OEC.DS` (bit 8) cleanly disables OCDS again.

Every `OSTATE` bit that matters is written through a paired protection bit in
`OEC` or `OCNTRL` - the value must be set together with its `_P` companion in the
same access, exactly like `OJCONF`. `HARR` is reachable both ways: `OCNTRL.OJC0`
over the system bus, or `OJCONF.OJC0` through the IOClient, which is the one that
still works when the bus is locked or unclocked.

`IOCONF` (offset `0x00`, instruction `0H`) is write-only and carries `MODE` bit 0
(0 = communication mode, 1 = read/write mode), `TRIG_EN`, `EX_BUS_TRC`,
`EX_BUS_HW` width select, `FPI_PRIO`, `SVM_MODE` privilege at bit 7, a 3-bit
`CHANNEL` and `CMBT`. It has **no auto-increment control**: single-word
instructions leave `IOADDR` alone, and the block instructions the DAP controller
issues internally post-increment it by four. So a probe only needs `IOCONF` to
choose RW mode and privilege.

**miniMCDS registers this project uses**, base `0xFB718000`:

| offset | name | use |
|---|---|---|
| `0x0000` | `CLC` | clock control; the configuration writes 0 to enable |
| `0x0010` | `CT` | write gate - `SETE` bit 15 unlocks, `CLRE` bit 14 locks |
| `0x0200` | `FIFONOW` | write pointer the drain follows |
| `0x0204` / `0x020C` | `FIFOBOT` / `FIFOTOP` | buffer bounds, written by the streamer rather than by the configuration |
| `0x0210` | `FIFOCTL` | `CLR` bit 14 starts, `SET` bit 15 flushes, `FLSH` bit 13 reads back |
| `0x0214` / `0x0218` | `FIFOWARN0/1` | threshold comparators; zeroed so the buffer is not halted for draining |
| `0x021C` | `FIFOOVRCNT` | overflow counter |

`ID` at `0x0008` reads `0x00D6C007` on a miniMCDS, which makes a good
post-enable sanity check: if OCDS is off it bus-errors, and if it is on it
returns a known constant.

Two blocks outside that table matter because the configuration writes them:

- **`0xFB71A400` / `0xFB71A404` are `TCXEABND0` / `TCXEARNG0`**, the DTU address
  comparator's lower bound and range, matching `BOUND <= address <= BOUND +
  RANGE`. That is why a traced window of `0x10000` bytes appears as a range of
  `0xFFFF`. A second comparator sits at `0xFB71A410` / `0xFB71A414`, so two
  disjoint windows can be traced without touching anything else.
- **`0xB8000000..0xB8001FFF` is the TRAM itself** over the non-cached SRI alias.
  The configuration seeds `0xFFFFFFFF` into the first five words and the last
  one: a run of ones decodes as `<endoftrace>`, so a decoder that reads past the
  written paragraphs into untouched buffer halts instead of parsing noise. Worth
  keeping in any table-driven configuration for exactly that reason.

`0xF000043C` in the Cerberus block is `TRMC`, and the configuration's
`0x00100000` sets `BRKIN`, connecting the external break-in trigger pins to the
core debug trigger lines.

### Trace stream encoding

Needed only for Phase 6b. Recorded here because parts of it are corrections to,
or derivations from, the vendor documents rather than quotations of them.

**Framing.** The TRAM is divided into paragraphs on 1 kB boundaries. Every trace
unit's first message in a paragraph is uncompressed, trailing space is filled
with a `<skip>` message, and the last written paragraph ends with
`<endoftrace>`. Messages pack forward from the low address of the paragraph and
every field is LSB-first. Fields are latched in this order:

```
<length>  ->  <core_ID>  ->  <trace_type>  ->  <trace_data>
 4 or 12       5 bits         3 bits          variable
```

**Length.** The 4-bit `<symlength>` gives the size of the *body*, which is the
data plus the 8 bits of type and core ID - it does **not** include the length
field itself. So `data = body - 8` and the parser consumes `body + 4` bits.
`1111B` escapes to an 8-bit `<bicount>` carrying the body size directly; a
message whose compressed data strips to zero bits has body 8, which is not in the
symbolic table and therefore always uses the escape.

This contradicts the vendor table, which labels the same numbers "total message
size". The correction is forced by the packet catalog: under the published
reading every packet is four bits short and `DTWD` cannot hold a 32-bit write
(42 - 12 - 2 = 28). Under the rule above, all eight catalog entries decompose
exactly:

| packet | decomposition | catalog max |
|---|---|---|
| `DTWD` | 8 header + 32 data + 2 size | 42 |
| `DTWA` | 8 + 44 address field + 2 subtype | 54 |
| `DTW` | 8 + 4 length1 + 2 size + 32 data1 + 44 data2 | 90 |
| `DTRD` | 8 + 32 + 2 size + 2 subtype | 44 |
| `DTRA` | 8 + 44 + 2 | 54 |
| `DTR` | 8 + 4 + 2 + 32 + 44 | 90 |
| `DTA` | 8 + 44 + 2 | 54 |
| `ERR` | 7 lost count + 5 core | 12 |

**Write access (`DTW`, TT4 uncompressed / TT5 compressed).** After the control
headers, in transmission order: `<length1>` (4 bits, giving the width of the
written data), `data size` (2 bits: 00 byte, 01 halfword, 10 word, 11 double),
`<data 1>` (the written value), then `<data 2>`, whose width is
`body - 4 - 2 - |data 1| - 8`. `<data 2>` is 44 bits, split as:

```
 43        37 36    33 32 31                              0
+------------+--------+---+--------------------------------+
| SUBCHANNEL | MASTER |SVM|      transaction address        |
+------------+--------+---+--------------------------------+
```

So the address is the low 32 bits, bit 32 is supervisor/user, bits 36:33 are the
bus master tag (`0001B` = CPU0, `1000B` = DMA/Cerberus) and bits 43:37 the
subchannel or DMA channel. Masking those fields through `TCXACMSK` zeroes them,
after which the XOR-delta compression strips them out entirely - which is why a
real capture averages ~6.6 bytes per message rather than the 11 a full packet
would take. Reads (`DTR`) are the same with TT6/TT7.

**Subtype** is 2 bits on TT2/TT3 packets: `0` = `DTA`, `1` = `DTWA`, `2` =
`DTRD`, `3` = `DTRA`.

**Compression** is XOR against a reference with leading zeros stripped, and the
caches are keyed per DTU core and per *field*, not per packet type: one address
cache updated by `DTWA`, `DTRA`, `DTA` and the address half of `DTW`/`DTR`, and
one data cache updated by `DTWD`, `DTRD` and the data half of `DTW`/`DTR`. Even
trace types are uncompressed and reset the cache; odd ones are deltas. The
decoder never needs to know `MAXCNT` - the type field says which each message is.

**Timestamps** precede the messages they stamp. The DMC writes the `TSR`/`TSA` or
`<tick>`/`<multick>` packet first, then the trace messages sharing that time tag,
so a parser reading upward latches a stamp and applies it to everything after it
until the next one.

**Recovering from a lapped buffer.** A TRAM lap produces no `ERR` - those are for
observation-unit FIFO overflows. Instead: read `FIFONOW`, discard the whole
paragraph containing it, resume at the next 1 kB boundary with both caches
zeroed, which is safe because each unit's first message there is uncompressed.

### Open questions to settle on hardware

Nothing in the protocol is unresolved on paper any more. Answered from the vendor
documentation: the block-transfer wire order and address forms, `client_read`'s
frame layout, every packet field width, the compression cache keys, timestamp
ordering, the CRC6 generator with its test vectors, which length the `sync` frame
carries, wide-mode framing, `IOCONF`, the `data 2` split, the DTU address
comparators, the OCDS enable sequence and the miniMCDS map.

Two answers came back negative, which is just as useful:

- **`TRADDR` is not a trace shortcut.** The triggered-transfer path moves one
  static location per trigger, with no auto-increment on either the source or the
  destination pointer. Draining a buffer through it would mean rewriting `IOADDR`
  per word, slower than the pointer-following loop. Phase 4 stands as written.
- **There is no auto-increment flag to set.** Incrementing is a property of the
  block instructions the DAP controller issues internally, not of `IOCONF`.

What is left is not documentation but confirmation against silicon: every value
in this document has been derived or read, and none of it has met a target. The
three checks that turn it from a careful reading into a working probe are the
first frame (`0x09FE1` drawing `0xAAAAAAAA`), the first `client_read` payload
(`0x55` returning a word), and `CLIENT_ID` reading `0x0260`.

The turnaround gap was the last open protocol question and it is now closed on
paper: `MAXWAIT8` counts probe-supplied `DAP0` clocks, releases at the upstream
start bit, and re-arms per parcel in a block transfer, so stopping the clock to
flip `RDnWR` costs nothing. See "Timing and MAXWAIT8". What remains is
confirmation on silicon, which is Phase 1b, plus one provenance check: that
section is derived from a supplied specification rather than a datasheet in this
repository, and the two claims the design leans on are worth verifying against
Infineon's own document.

### Out of scope

- AGBT/Aurora trace. TC38x has no such pins, and a part that does would not need
  any of this.
- PTU, WTU and DCU decoding. Phase 6b covers only what this project consumes:
  data-trace writes and timestamps. Program trace is a much larger job and no
  current use case wants it.

### Where the pin layer actually is

An earlier draft of this document said Phase 1 builds on `debug_gpio.h` in the
`blackmagic_esp32` submodule. That is wrong: no such file exists anywhere in the
tree, and the submodule is checked out without one. The pin layer Phase 1 extends
is `components/CherryDAP/projects/esp32s3/main/DAP_config.h`, which carries the
read/not-write direction control, together with `port_common.h` beside it.
