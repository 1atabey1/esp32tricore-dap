#!/bin/bash -l
# usage: run.sh <device flags...>  -> synth once, seeds 1..16, Fmax per clock
cd ~/.cache/dapexp/hx
R=~/repos/esp32jtag_firmware/fpga/dap_master/rtl
RTL="$R/dap_crc6.v $R/dap_frame_tx.v $R/dap_frame_rx.v $R/reply_fifo.v $R/spi_slave.v $R/dap_top.v"
[ -f hx.json ] || yosys -q -p "read_verilog $RTL hx_top.v; synth_ice40 -top hx_top -json hx.json" || exit 1
tag=$(echo "$*" | tr -d ' -')
for s in $(seq 1 16); do
  nextpnr-ice40 "$@" --json hx.json --freq 200 --timing-allow-fail --seed $s --quiet --log pnr_${tag}_$s.log >/dev/null 2>&1
  c=$(grep -E "Max frequency for clock +'clk" pnr_${tag}_$s.log | tail -1 | grep -oE "[0-9.]+ MHz" | head -1)
  k=$(grep -E "Max frequency for clock +'spi_sck" pnr_${tag}_$s.log | tail -1 | grep -oE "[0-9.]+ MHz" | head -1)
  echo "$tag seed $s clk $c sck $k"
done
grep -E "ICESTORM_LC|ICESTORM_RAM|SB_IO|SB_GB" pnr_${tag}_1.log | head -6
