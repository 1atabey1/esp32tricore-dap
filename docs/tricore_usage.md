# TriCore probe: usage

Probe `http://<board>` (bench: `192.168.178.99`), basic auth (default `admin:admin`).
All endpoints below need auth: `curl -u admin:admin ...`.

## 0. Toolchain (WSL)

```sh
tricore-elf-gdb --version        # ~/.local/opt/trigcc/bin, on PATH via ~/.profile and ~/.bashrc
tricore-elf-gcc ...
```

## 1. FPGA image

DAP master bitstream is the default. Port C must be in SWD/JTAG mode.

```sh
curl -u admin:admin 'http://<board>/api/fpga_image'            # show current
curl -u admin:admin 'http://<board>/api/fpga_image?sel=stock'  # logic analyser / XVC image
curl -u admin:admin 'http://<board>/api/fpga_image?sel=dap'    # DAP master (default)
```

## 2. Flash via web

Browser: `http://<board>/flash.html` -> choose `.hex` -> Flash. Target is reset and started afterwards.

CLI:

```sh
curl -u admin:admin --data-binary @app.hex http://<board>/api/flash/upload   # "ok records=.. bytes=.."
curl -u admin:admin -X POST http://<board>/api/flash/start                   # block writes (~15.5 s / 700 kB)
curl -u admin:admin -X POST 'http://<board>/api/flash/start?slow=1'          # word writes (fallback)
curl -u admin:admin http://<board>/api/flash/status                          # phase=done verified=1 ...
```

Only program flash is written (0xA0000000, or the cached alias 0x80000000); other records (UCB, DFLASH) are skipped.

## 3. GDB

Targets are registered at boot; re-register after a target power cycle:

```sh
curl -u admin:admin http://<board>/api/dap_gdb/attach
curl -u admin:admin http://<board>/api/dap_gdb/status   # cores=4 cpu0=running ... gdb_stack_free=N
```

```
$ tricore-elf-gdb app.elf
(gdb) target extended-remote <board>:4242
(gdb) monitor targets          # CPU0..CPU2, "CPU3 (not started)" = boot-halted, cannot attach
(gdb) attach 1                 # 1 = CPU0; halts it
(gdb) load                     # erases touched sectors, programs (~21 KB/s over Wi-Fi)
(gdb) compare-sections -r
(gdb) hbreak func / break func # hardware triggers (break is served by triggers too)
(gdb) watch var / rwatch / awatch
(gdb) continue / Ctrl-C / stepi / next / finish / bt
(gdb) monitor reset / run      # app reset, halts at entry
(gdb) detach                   # clears triggers, core keeps running
```

Architecture comes from the target description; no `set architecture` needed.
Status shows `busy` while a flash or diagnostic run holds the probe.
Port 4242 not answering after a GDB crashed during `continue`: `curl .../api/dap_gdb/attach`.
GDB 14 shows `e*`/`p*` pseudo-registers wrong; use `d*`/`a*`.

## 4. Fabric DAP check / wide mode

```sh
curl -u admin:admin http://<board>/api/dap_fpga          # attach, CLIENT_ID, miniMCDS ID, narrow kB/s per clock
curl -u admin:admin 'http://<board>/api/dap_fpga?wide=1' # + wide-mode bring-up, taps, wide kB/s, back to narrow
```

Wide mode halts the application while P21.7 is borrowed as DAP2, then restores it and resumes.

## 5. Trace (miniMCDS TRAM drain)

Browser: `http://<board>/trace.html` (Start capture / Stop / Self-test / Save; stream over `ws://<board>/ws/trace`).
This page drains a TRAM configured by someone else (e.g. a host tool); for probe-configured
data/watch-point trace use 5b.

```sh
curl -u admin:admin http://<board>/api/dap_trace/start
curl -u admin:admin http://<board>/api/dap_trace/stats      # paragraphs, lost, laps, ...
curl -u admin:admin http://<board>/api/dap_trace/stream > t.bin   # ~1 s per request, repeat
curl -u admin:admin http://<board>/api/dap_trace/stop
curl -u admin:admin http://<board>/api/dap_trace/selftest   # drain check without a tracing target
```

Stream = `dap_trace_record_t` header (magic `DTRP`, seq, offset, length, gap flag, lost) + 1 kB paragraph.
One WebSocket client at a time.

## 5b. Data trace / watch-points (DTU + WTU)

Browser: `http://<board>/datatrace.html` (tab "TriCore Data Trace").

1. Source: CPU pipeline (what that CPU loads/stores), CPU memory slave (any master into its DSPR/PSPR), LMU0.
2. Mode: *Full trace* (address and/or value of every matching access) or *Compact watchpoint timestamping*
   (one 12-bit WPS per hit).
3. Up to two slots: address, size (bytes; larger = watch a whole struct), access r / w / rw,
   optional value filter (low..high after mask, signed).
4. Timestamps: per hit (default), ticks (every cycle, +~850 kB/s), none.
5. *Start* asks for a file (Chrome/Edge save picker; other browsers download at the end), streams,
   *Stop* flushes and closes the file.

CLI instead of the page (same file format):

```sh
cd tools/mcds_trace
# posts the config, starts, streams 2 s, stops (up to two --watch slots)
uv run mcds-trace capture <board> run.mcds --cpu 2 --watch 0x5000220C:2:w:mCyclesUntilSecond --seconds 2
uv run mcds-trace capture <board> run.mcds --config cfg.json --seconds 2   # flags override the file
uv run mcds-trace decode run.mcds --plot                    # or --plot out.png --window 0.1:0.11
uv run mcds-trace decode run.mcds --csv events.csv --elf app.elf
```

`--watch ADDR:SIZE[:ACCESS[:NAME]]`, plus `--source`, `--cpu`, `--mode`, `--payload`,
`--timestamps`, `--dap-div`. Value filters need `--config`. `cfg.json`:

```json
{"source": "cpu", "cpu": 0, "mode": "full", "payload": "addr_data", "timestamps": "hit",
 "slots": [{"enabled": true, "name": "tick", "addr": "0x7001A08C", "size": 4, "access": "w",
            "value": {"enabled": false, "lo": "0", "hi": "0xFFFFFFFF", "mask": "0xFFFFFFFF"}},
           {"enabled": false}]}
```

The decode summary always states losses: gaps (probe lapped / ring full), ERR (target FIFO overflow).
The probe drains ~2.7 MB/s of trace (wide DAP at 24 MHz, the default; narrow when P21.7 is driven by
the application); sources writing faster lose (flagged) paragraphs.

**Application:** `uv run mcds-trace-ui` (desktop, or `--web`) picks variables from the ELF by name -
struct members, array elements, bitfields - plans the two watch ranges, traces and plots live, and
opens captures offline. See `tools/mcds_trace/README.md`.
Addresses are compared as the source sees them (cached 0x8…/0x9… ≠ uncached 0xA…/0xB…).

## 6. Bring-up diagnostics (CPU DAP path)

```sh
curl -u admin:admin http://<board>/api/dap_bringup   # bit-bang checkpoints: sync, dapisc, client_set, CLIENT_ID, memory
curl -u admin:admin http://<board>/api/dap_spi       # same over GP-SPI + throughput
```

Needs the stock image or Port C passthrough. Rate: `CONFIG_AEL_DAP_BRINGUP_CLOCK_HZ` (menuconfig).

## 7. RTL

```sh
cd fpga/dap_master && make sim     # testbench, must print PASSED
make bitstream                     # build/dap_master.bin -> copy to main/ice40up5k/, rebuild firmware
```
