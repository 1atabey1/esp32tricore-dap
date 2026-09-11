#!/bin/bash
# Which placement seed closes timing, for a given netlist?
#
#   ./seedsweep.sh <netlist.json> <seed> [seed ...]
#
# nextpnr's placer is randomised and the spread between seeds is several MHz on
# a design this full, so the seed is worth searching rather than accepting.  The
# winner belongs in the Makefile: a bitstream that only meets timing on an
# unrecorded seed is not a design that meets timing.
cd "$(dirname "$0")" || exit 1
json=$1
shift
for s in "$@"; do
    # Two clock domains now, and nextpnr pads their names into columns, so the
    # pattern has to tolerate the whitespace it uses to line them up.
    f=$(nextpnr-ice40 --up5k --package sg48 --json "$json" \
            --pcf dap_master.pcf --asc "/tmp/seed_$s.asc" --freq 48 --seed "$s" 2>&1 |
        grep -oE "Max frequency for clock +'clk': [0-9.]+ MHz" | tail -1)
    echo "seed $s: ${f:-did not report}"
done
