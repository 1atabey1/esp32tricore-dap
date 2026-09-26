# ESP32JTAG: Infineon TriCore / AURIX TC38x probe

Wi-Fi debug probe for AURIX TC38x on the ESP32-S3 + iCE40UP5K ESP32JTAG board.
The FPGA speaks DAP to the target; the firmware adds GDB, flash programming
and a live data trace. A host application plots traced variables in real time.

| Feature | |
|---|---|
| DAP link | FPGA DAP master, wide mode at 24 MHz, ~3.9 MB/s block reads (verified) |
| GDB | Server on port 4242, one target per core: hw breakpoints, watchpoints, step, reset, `load` |
| Flash | Program flash from the browser or GDB, CRC-verified on the target |
| Data trace | miniMCDS DTU + WTU: any variable, full values or compact hit times, lossless at 2.2 MB/s |
| Host app | `mcds-trace-ui`: ELF symbol picker, live and offline plots, captures, export |

## Build and flash the firmware

ESP-IDF v5.5.2 or newer.

```bash
git clone --recursive https://github.com/EZ32Inc/esp32jtag_firmware.git
cd esp32jtag_firmware
. $IDF_PATH/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash          # later: OTA via the web UI
```

First boot: Wi-Fi AP `esp32jtag` / `esp32jtag`; web UI and API with basic
auth `admin` / `admin`. Put Port C in SWD/JTAG mode; the DAP bitstream is the
default FPGA image.

## Use

| | |
|---|---|
| Flash | `http://<probe>/flash.html`, or `curl --data-binary @app.hex .../api/flash/upload` + `-X POST .../api/flash/start` |
| Debug | `tricore-elf-gdb app.elf` → `target extended-remote <probe>:4242` → `attach 1` |
| Trace | `http://<probe>/datatrace.html`, or the host app below |

```bash
cd tools/mcds_trace
uv run mcds-trace-ui --elf app.elf              # desktop window (Windows, Linux, macOS)
uv run mcds-trace-ui --web                      # in the browser
uv run mcds-trace capture <probe> run.mcds ...  # command line
```

Pick variables (numbers, arrays, structs, bitfields) from the ELF, press
Start, and watch them live; every session is recorded to a `.mcds` file for
offline review. Subplots take any number of signals.

## Documentation

| | |
|---|---|
| [docs/tricore_usage.md](docs/tricore_usage.md) | All endpoints, GDB, flash, trace, diagnostics |
| [tools/mcds_trace/README.md](tools/mcds_trace/README.md) | Host application and command line |
| [components/dap_probe/README.md](components/dap_probe/README.md) | Firmware internals, FPGA link, limits |
| [docs/minimcds_trace_spec.md](docs/minimcds_trace_spec.md) | Trace message format (own decoder, no Infineon libraries) |

## Layout

```
components/dap_probe/   DAP protocol, TriCore GDB target, flash, miniMCDS trace
fpga/dap_master/        DAP master RTL (iCE40UP5K)
main/network/           Web pages and HTTP / WebSocket API
tools/mcds_trace/       Host decoder, UI and CLI (Python, uv)
```

## License

Apache License 2.0 (see [LICENSE](LICENSE)). The GDB server uses the
BlackMagic Probe submodule (GPL-3.0).
