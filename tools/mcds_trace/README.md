# mcds_trace

Host side of the TriCore data / watch-point trace (probe page `datatrace.html`,
firmware `components/dap_probe/dap_mcds.c`). Own decoder, no Infineon libraries.

Two front ends share one package: **`mcds-trace-ui`**, an interactive
application (Flet), and **`mcds-trace`**, a command line for scripting.

## MCDS Trace (the application)

```sh
cd tools/mcds_trace
uv run mcds-trace-ui                                   # desktop window (Windows, Linux, macOS)
uv run mcds-trace-ui --elf app.elf                     # symbols loaded at start
uv run mcds-trace-ui run.mcds                          # open a capture
uv run mcds-trace-ui --web                             # in the browser instead of a window
uv run mcds-trace-ui --serve --port 8550 --bind 0.0.0.0   # serve only, open http://HOST:8550
uv run mcds-trace-ui --host 192.168.178.99 --workspace bench.mcdsws
```

Pure Python (flet, matplotlib, numpy, Pillow, pyelftools); `uv` resolves
everything on each platform, nothing is compiled.

The trace data path - the probe's stream, the capture file, decoding and
signal extraction - runs in a worker process of its own, so neither a busy
UI nor plotting can slow the stream down; the UI receives the samples in
batches and draws its frames directly (a few milliseconds for dozens of
signals, 12 frames/s while tracing). In the browser (`--web`, `--serve`) a
reload or a network drop only pauses drawing; a running trace carries on.
`MCDS_TRACE_PERF=1` prints frame times.

1. **Symbols** - open the ELF of the traced application (the one that is
   flashed). Search by name: variables, and struct members at any depth
   (typing `mCyclesUntil` finds `app.mCyclesUntilSecond`). Expand structs,
   unions, classes and arrays; add a numeric member with its `+`, or every
   numeric member of an aggregate with the list button. Numbers, floats,
   enums (shown by name), bools, pointers and bitfields are traceable. A raw
   address works without an ELF.
2. **Plots** - every picked signal gets its own plot; drag signals from the
   **Signals** tab onto a plot to overlay them. A plot's menu: step / lines /
   points, normalize (compare shapes of different scales), statistics of the
   view (n, min, max, mean, typical interval), Y range and log scale, split
   into one plot per signal, merge into the plot above, height, order.
3. **Trace live** - press **Start** (or F5). The trace hardware watches two
   address ranges; they are chosen from the selected signals so that as few
   extra bytes as possible are watched (**Capture** tab shows the plan, and a
   value filter when a range holds one signal). The traced core defaults to
   the owner of the memory. Every session is recorded to the capture folder
   while the plots follow the newest data; *Stop after* ends it by itself.
   Values start from a memory snapshot the probe takes at start, so
   variables that never change show up and partial writes decode correctly.
4. **Navigate** - mouse wheel zooms time around the pointer, drag pans,
   double-click shows everything (live: resumes following). Hovering shows
   each signal's value at that instant in the plot headers. Click sets a
   marker; the toolbar then shows the time to it and its frequency;
   right-click removes it.
5. **Offline** - open a `.mcds` file (toolbar, recent list, or command line).
   Selection changes re-extract from the decoded events without decoding
   again; variables the capture has accesses to are offered after opening.
6. **Export** - PNG of the view, CSV of the visible samples; workspaces
   (`.mcdsws`: ELF, signals, plots, capture settings) save and load.

Compact mode records hit times per range instead of values. Gaps (lost
paragraphs) are marked with red dashed lines; nothing is interpolated.

Throughput: the probe reads the trace RAM at about 3.2 MB/s while tracing
(wide DAP at 24 MHz); sources that write faster lap the 8 kB trace RAM and
lose paragraphs (flagged). A 30 s trace of 27 members of a 20 kHz task
(2.2 MB/s) arrives without a gap. The application decodes and extracts about
4 MB/s; if it ever falls behind, it skips ahead in the live view only - the
file is always complete. After Stop, the whole file is loaded for review if
the live view had skipped, trimmed its history or dropped old events.

## Command line

```sh
uv run mcds-trace capture 192.168.178.99 run.mcds --cpu 2 --watch 0x5000220C:2:w:mCyclesUntilSecond --seconds 2
uv run mcds-trace capture 192.168.178.99 run.mcds --config cfg.json --mode compact --seconds 2
uv run mcds-trace capture 192.168.178.99 run.mcds --seconds 5 --start   # config already on the probe
uv run mcds-trace decode run.mcds                     # summary: message types, gaps, ERR, clock
uv run mcds-trace decode run.mcds --plot              # value/time per variable + interval histogram
uv run mcds-trace decode run.mcds --plot out.png --window 0.10:0.11
uv run mcds-trace decode run.mcds --csv events.csv --elf app.elf --signed
uv run pytest                                         # decoder, DWARF, extraction, session tests
```

## File format

`MCDSCAP1 | u32 json_len | config JSON (incl. emu_hz, snapshot) | DTRP record*`;
record = `u32 "DTRP" | u32 seq | u32 tram_offset | u16 len | u16 flags(bit0 gap) | u32 lost | 1 kB paragraph`.
The web page writes the same layout.

## Modules

| Layer | Module |
|---|---|
| paragraph → messages (`<length> <core_ID> <trace_type> <data>`, LSB first, nibble packed) | `tram.py` |
| DTU (DTW/DTR/DTWD/DTRD/DTA/…), WTU (WPS/WPM/EVC), TSU (TSR/TSA), compression, time | `decode.py` |
| ELF/DWARF variables and types, browsable tree, leaf decoding | `elfsyms.py` |
| shadow memory, per-signal samples, slot planning | `signals.py` |
| live capture / capture files, event log, probe HTTP | `session.py` |
| decimated off-screen plot frames | `plotrender.py` |
| the application | `ui/` |
| CLI series, CSV, plots | `analyse.py`, `cli.py` |

- Compression: XOR with the last reconstructed value per engine (DTU address, DTU data, WPS, WPM, EVC, TSR, TSA).
  After a gap every engine is unsynchronised until its next uncompressed message.
- Time: TSR = 32-bit emulation clock (unwrapped); ticks mode adds `<tick>`=1 / `<multick>`=n cycles.
  `emu_hz` is measured by the probe at start.
- Loss is reported, never hidden: probe gaps (lapped TRAM, dropped ring records), target `ERR`
  messages (DTU/WTU FIFO overflow), unsynchronised / malformed messages, TSR back-steps.
- DWARF: statics of inlined functions have no location in GCC's output; they are placed from the
  ELF symbol table (`_ZZ…E<name>`) and typed from DWARF. Data symbols without debug information
  appear untyped. Parsed symbols are cached per ELF (size + mtime) in `~/.cache/mcds-trace`.
