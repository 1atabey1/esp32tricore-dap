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

The host app lives in the [tricore-debug-machine repository](https://github.com/1atabey1/tricore-debug-machine/tree/board-bringup/sw/tools/mcds_trace)
(`sw/tools/mcds_trace`):

```bash
cd tricore-debug-machine/sw/tools/mcds_trace
uv run mcds-trace-ui --elf app.elf              # desktop window (Windows, Linux, macOS)
uv run mcds-trace-ui --web                      # in the browser
uv run mcds-trace capture <probe> run.mcds ...  # command line
```

Pick variables (numbers, arrays, structs, bitfields) from the ELF, press
Start, and watch them live; every session is recorded to a `.mcds` file for
offline review. Subplots take any number of signals.

## HTTP API

Everything on port 80 needs HTTP basic auth (`admin` / `admin` until changed). Examples and
parameters: [docs/tricore_usage.md](docs/tricore_usage.md).

| Endpoint | Method | Purpose |
|---|---|---|
| **Control port 8081** | | Separate server with its own task: reachable while port 80 is busy or wedged |
| `:8081/api/reboot` | POST | Reboot the probe (after 0.5 s) |
| **System** | | |
| `/api/version` | GET | Firmware, hardware, IDF and git versions |
| `/ota_upload` | POST | Firmware update (multipart, field `firmware`); reboots |
| `/reset_to_factory` | POST | Erase all settings and credentials; reboots |
| `/set_credentials`, `/get_credentials` | POST, GET | Web login |
| `/upload/*`, `/delete/*` | POST | Files on the probe's FAT storage |
| `/log_error` | POST | Browser-side error log |
| `/test/start`, `/test/status`, `/test/result` | POST, GET, GET | GPIO loopback self-test |
| **Target reset** | | |
| `/api/trst` | POST | Pulse TRST / PC01: `{"pulse_ms": 1-1000}` (default 20); 409 if the probe is busy |
| `/api/reset_target` | POST | Pulse SRESET (Port B must be Vtarget+UART+SReset) |
| `/api/sreset_config` | POST | SRESET polarity and pulse width |
| **Flash** | | |
| `/api/flash/upload` | POST | Upload an Intel HEX body |
| `/api/flash/start` | POST | Program: `?full=1`, `?slow=1`, `?ucb=1` |
| `/api/flash/status`, `/api/flash/ucb` | GET | Progress and result; boot mode header report |
| **GDB and DAP** | | GDB server itself is TCP 4242 |
| `/api/dap_gdb/attach`, `/api/dap_gdb/status` | GET | Register the cores with GDB; core states and heap |
| `/api/dap_fpga` | GET | Fabric DAP check; `?wide=1` adds wide mode |
| `/api/fpga_image` | GET | Show or select the FPGA image: `?sel=stock\|dap` |
| `/api/fpga_load` | POST | Load a bitstream (body, up to 512 kB) until reboot |
| `/api/dap_bringup`, `/api/dap_spi`, `/api/dap_bench` | GET | Bring-up checks over bit-bang and SPI; block-read benchmark |
| **Trace** | | |
| `/api/dap_trace/start`, `/stop`, `/stats`, `/stream`, `/selftest` | GET | Raw trace capture and drain |
| `/ws/trace` | WebSocket | Trace stream (`?z=1` compressed) |
| `/api/mcds/config` | GET, POST | miniMCDS trace configuration |
| `/api/mcds/start`, `/api/mcds/stop` | GET | Start and stop an MCDS trace |
| **Port, UART and logic analyser** | | |
| `/ws` | WebSocket | UART bridge |
| `/api/uart_debug` | GET | UART state and last received data |
| `/api/portd_output`, `/api/portd_freq` | POST | Port D output pattern; square-wave generator |
| `/la_start_capture`, `/instant_capture`, `/la_status`, `/capture_data` | GET | Logic analyser capture |
| `/la_configure`, `/la_get_settings` | POST, GET | Logic analyser settings |
| **Pages** | | |
| `/`, `/help`, `/flash.html`, `/trace.html`, `/datatrace.html`, `/logic_analyzer.html` | GET | Web UI |

The control port is not reachable through a port forward unless you forward it too. It uses the
same credentials and plain HTTP, so keep it on a trusted network or behind a VPN.

## Documentation

| | |
|---|---|
| [docs/tricore_usage.md](docs/tricore_usage.md) | All endpoints, GDB, flash, trace, diagnostics |
| [mcds_trace](https://github.com/1atabey1/tricore-debug-machine/tree/board-bringup/sw/tools/mcds_trace) | Host application and command line (tricore-debug-machine repository) |
| [components/dap_probe/README.md](components/dap_probe/README.md) | Firmware internals, FPGA link, limits |
| [docs/minimcds_trace_spec.md](docs/minimcds_trace_spec.md) | Trace message format (own decoder, no Infineon libraries) |

## Layout

```
components/dap_probe/   DAP protocol, TriCore GDB target, flash, miniMCDS trace
fpga/dap_master/        DAP master RTL (iCE40UP5K)
main/network/           Web pages and HTTP / WebSocket API
tools/mcds_trace/       Pointer: the host tool moved to tricore-debug-machine
```

## License

Apache License 2.0 (see [LICENSE](LICENSE)). The GDB server uses the
BlackMagic Probe submodule (GPL-3.0).
