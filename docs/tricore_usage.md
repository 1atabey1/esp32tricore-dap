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
curl -u admin:admin -X POST http://<board>/api/flash/start                   # block writes (~20 s / 700 kB)
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
MCDS trace configuration is the host's job; the probe only drains.

```sh
curl -u admin:admin http://<board>/api/dap_trace/start
curl -u admin:admin http://<board>/api/dap_trace/stats      # paragraphs, lost, laps, ...
curl -u admin:admin http://<board>/api/dap_trace/stream > t.bin   # ~1 s per request, repeat
curl -u admin:admin http://<board>/api/dap_trace/stop
curl -u admin:admin http://<board>/api/dap_trace/selftest   # drain check without a tracing target
```

Stream = `dap_trace_record_t` header (magic `DTRP`, seq, offset, length, gap flag, lost) + 1 kB paragraph.
One WebSocket client at a time.

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
