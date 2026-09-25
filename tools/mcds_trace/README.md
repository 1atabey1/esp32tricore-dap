# mcds_trace

Host side of the TriCore data / watch-point trace (probe page `datatrace.html`,
firmware `components/dap_probe/dap_mcds.c`). Own decoder, no Infineon libraries.

```sh
cd tools/mcds_trace
uv run mcds-trace capture 192.168.178.99 run.mcds --cpu 2 --watch 0x5000220C:2:w:mCyclesUntilSecond --seconds 2
uv run mcds-trace capture 192.168.178.99 run.mcds --config cfg.json --mode compact --seconds 2
uv run mcds-trace capture 192.168.178.99 run.mcds --seconds 5 --start   # config already on the probe
uv run mcds-trace decode run.mcds                     # summary: message types, gaps, ERR, clock
uv run mcds-trace decode run.mcds --plot              # value/time per variable + interval histogram
uv run mcds-trace decode run.mcds --plot out.png --window 0.10:0.11
uv run mcds-trace decode run.mcds --csv events.csv --elf app.elf --signed
uv run pytest                                         # decoder tests (incl. the TS Table 342 example)
```

## File format

`MCDSCAP1 | u32 json_len | config JSON (incl. emu_hz) | DTRP record*`;
record = `u32 "DTRP" | u32 seq | u32 tram_offset | u16 len | u16 flags(bit0 gap) | u32 lost | 1 kB paragraph`.
The web page writes the same layout.

## Decoder (TC3xx TS ch. 9, see docs/minimcds_trace_spec.md)

| Layer | Module |
|---|---|
| paragraph → messages (`<length> <core_ID> <trace_type> <data>`, LSB first, nibble packed) | `tram.py` |
| DTU (DTW/DTR/DTWD/DTRD/DTA/…), WTU (WPS/WPM/EVC), TSU (TSR/TSA), compression, time | `decode.py` |
| per-variable series, CSV, plots | `analyse.py` |

- Compression: XOR with the last reconstructed value per engine (DTU address, DTU data, WPS, WPM, EVC, TSR, TSA).
  After a gap every engine is unsynchronised until its next uncompressed message.
- Time: TSR = 32-bit emulation clock (unwrapped); ticks mode adds `<tick>`=1 / `<multick>`=n cycles.
  `emu_hz` is measured by the probe at start.
- Loss is reported, never hidden: probe gaps (lapped TRAM, dropped ring records), target `ERR`
  messages (DTU/WTU FIFO overflow), unsynchronised / malformed messages, TSR back-steps.
