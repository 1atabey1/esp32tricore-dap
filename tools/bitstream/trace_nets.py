#!/usr/bin/env python3
"""Trace signal dependencies through an icebox_vlog flattened netlist.

The stock ICE40 bitstream on this board has no HDL source, but `iceunpack`
plus `icebox_vlog` recover a flattened `module chip(...)` whose port names are
`io_<x>_<y>_<block>`.  Joined against `icebox.pinloc_db` those map onto package
pins, and the schematic maps package pins onto board nets.  That is enough to
answer the one question the bitstream alone cannot: which ESP32-S3 GPIO the
stock port mux connects to which target-connector pin.

Usage:
    trace_nets.py <chip.v> --from <net> [--to <net>] [--depth N]

Reports, for a starting net, the module inputs it transitively depends on and
whether the path is purely combinational or passes through a flip-flop.
"""

from __future__ import annotations

import argparse
import re
import sys
from collections import deque

#: `assign n123 = <expr>;` - the statement may be *preceded* by a /* LUT x y z */
#: or /* FF x y z */ comment on the same line, so this is deliberately not
#: anchored at the start of the line.
ASSIGN_RE = re.compile(r"assign\s+(\S+)\s*=\s*(.*?);")

#: `/* FF x y z */ always @(posedge nCLK, ...) if (...) nQ <= ...; else if (nEN) nQ <= nD;`
FF_RE = re.compile(r"always\s*@\s*\(posedge\s+(\S+?)\s*,.*?\)\s*.*?else if\s*\((\S+?)\)\s*(\S+)\s*<=\s*(\S+)\s*;")

NET_RE = re.compile(r"\b(n\d+|io_\d+_\d+_\d+)\b")


def parse(path: str):
    """Return (deps, ff_nets, inputs, outputs, drivers).

    `deps[net]` is the set of nets that net is computed from.  `ff_nets` are
    the nets driven by a flip-flop, so a path through them is not combinational.
    """
    deps: dict[str, set[str]] = {}
    ff_nets: set[str] = set()
    inputs: set[str] = set()
    outputs: set[str] = set()
    drivers: dict[str, str] = {}  # net -> the source line, for reporting

    text = open(path, encoding="utf-8", errors="replace").read()

    module = re.search(r"module\s+chip\s*\((.*?)\)\s*;", text, re.S)
    if module:
        for decl in module.group(1).split(","):
            decl = decl.strip()
            name = decl.split()[-1] if decl else ""
            if decl.startswith("input"):
                inputs.add(name)
            elif decl.startswith("output"):
                outputs.add(name)
            elif decl.startswith("inout"):
                inputs.add(name)  # readable as well as drivable
                outputs.add(name)

    for line in text.splitlines():
        m = ASSIGN_RE.search(line)
        if m:
            lhs, rhs = m.group(1), m.group(2)
            rhs_nets = set(NET_RE.findall(rhs)) - {lhs}
            deps.setdefault(lhs, set()).update(rhs_nets)
            drivers.setdefault(lhs, line.strip())
            continue
        m = FF_RE.search(line)
        if m:
            clk, en, q, d = m.groups()
            deps.setdefault(q, set()).update({d, en, clk} & set(NET_RE.findall(f"{d} {en} {clk}")))
            ff_nets.add(q)
            drivers.setdefault(q, line.strip())

    return deps, ff_nets, inputs, outputs, drivers


def trace(start: str, deps, ff_nets, inputs, max_depth: int):
    """Breadth-first walk backwards from `start`, collecting reached inputs."""
    seen = {start}
    found: dict[str, tuple[int, bool]] = {}  # input -> (depth, crossed a FF)
    queue = deque([(start, 0, False)])
    while queue:
        net, depth, via_ff = queue.popleft()
        if depth > max_depth:
            continue
        if net in inputs and net != start:
            prev = found.get(net)
            if prev is None or depth < prev[0]:
                found[net] = (depth, via_ff)
            # An input is a leaf: nothing upstream of it inside the module.
            continue
        for parent in deps.get(net, ()):  # noqa: SIM118
            if parent in seen:
                continue
            seen.add(parent)
            queue.append((parent, depth + 1, via_ff or net in ff_nets))
    return found


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("netlist")
    ap.add_argument("--from", dest="start", required=True)
    ap.add_argument("--depth", type=int, default=40)
    ap.add_argument("--show-driver", action="store_true")
    args = ap.parse_args()

    deps, ff_nets, inputs, outputs, drivers = parse(args.netlist)
    if args.show_driver and args.start in drivers:
        print(f"driver: {drivers[args.start]}")

    found = trace(args.start, deps, ff_nets, inputs, args.depth)
    if not found:
        print(f"{args.start}: reaches no module input within depth {args.depth}")
        return 1
    print(f"{args.start} depends on:")
    for net, (depth, via_ff) in sorted(found.items(), key=lambda kv: kv[1][0]):
        kind = "through FF" if via_ff else "combinational"
        print(f"  {net:<14} depth {depth:>2}  {kind}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
