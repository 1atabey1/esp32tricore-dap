# mcds_trace (moved)

The miniMCDS trace host tool - live data trace plotting, program trace
recording and analysis (coverage, Perfetto), command line and desktop/web
UI - now lives in the tricore-debug-machine repository, history included:

    https://github.com/1atabey1/tricore-debug-machine/tree/board-bringup/sw/tools/mcds_trace

It talks to this probe over WiFi exactly as before (pick the probe's address
in the UI, or `mcds-trace capture <probe> ...`), and to the
tricore-debug-machine probe over USB.

```bash
git clone git@github.com:1atabey1/tricore-debug-machine.git
cd tricore-debug-machine/sw/tools/mcds_trace
uv run mcds-trace-ui --elf app.elf
```
