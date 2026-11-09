"""The code of a TriCore ELF as a program trace decoder needs it.

Every instruction of the executable sections with its length, its control
flow kind and (for direct branches) its target, the function it belongs to
and its source line.  The disassembly comes from the TriCore binutils
(tricore-elf-objdump -d -l), which resolves branch targets and line numbers
in one fast pass; function bounds come from the ELF symbol table.

Parsing takes a few seconds on a large ELF, so the result is pickled next to
the user cache, keyed by path, size and mtime.
"""

from __future__ import annotations

import bisect
import hashlib
import os
import pickle
import re
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass, field

import numpy as np

CACHE_VERSION = 5

# Control flow kinds of an instruction.
K_SEQ = 0        # not a branch
K_JUMP = 1       # direct, unconditional (j, ja, loopu)
K_COND = 2       # direct, conditional (jeq, jnz.t, loop, ...): target if taken
K_CALL = 3       # direct call (call, calla, fcall, fcalla)
K_JL = 4         # direct jump and link (jl, jla): return address in A11
K_ICALL = 5      # indirect call (calli, fcalli)
K_IJL = 6        # indirect jump and link (jli)
K_IJUMP = 7      # indirect jump (ji)
K_JIRET = 8      # ji %a11: the return of a jl
K_RET = 9        # ret, fret
K_RFE = 10       # rfe, rfm: return from an interrupt or trap
K_TRAP = 11      # syscall, trap*, debug: enter a trap handler
KIND_NAMES = ('seq', 'jump', 'cond', 'call', 'jl', 'icall', 'ijl', 'ijump', 'jiret', 'ret',
              'rfe', 'trap')
DIRECT = (K_JUMP, K_COND, K_CALL, K_JL)
CALLS = (K_CALL, K_JL, K_ICALL, K_IJL)

_UNCOND = {'j': K_JUMP, 'ja': K_JUMP, 'loopu': K_JUMP,
           'call': K_CALL, 'calla': K_CALL, 'fcall': K_CALL, 'fcalla': K_CALL,
           'jl': K_JL, 'jla': K_JL,
           'calli': K_ICALL, 'fcalli': K_ICALL, 'jli': K_IJL, 'ji': K_IJUMP,
           'ret': K_RET, 'fret': K_RET, 'rfe': K_RFE, 'rfm': K_RFE,
           'syscall': K_TRAP, 'debug': K_TRAP, 'trapv': K_TRAP, 'trapsv': K_TRAP}

_RE_INSN = re.compile(r'^([0-9a-f]+):\t((?:[0-9a-f]{2} )+)\s*\t?(\S*)\s*(.*)$')
_RE_FUNC = re.compile(r'^([0-9a-f]+) <(.+)>:$')
_RE_LINE = re.compile(r'^(.+):(\d+)(?: \(discriminator \d+\))?$')
# the branch target: the first address operand that objdump follows with
# <symbol>; demangled symbols hold '<', '>', ',' and ' ' themselves
_RE_TARGET = re.compile(r'(?:^|[ ,])([0-9a-f]{1,8}) <')


def kind_of(mnemonic: str, operands: str) -> int:
    m = mnemonic.lower()
    k = _UNCOND.get(m)
    if k is not None:
        if k == K_IJUMP and operands.strip() in ('%a11', 'a11'):
            return K_JIRET
        return k
    if m.startswith('j') or m == 'loop':
        return K_COND
    return K_SEQ


@dataclass
class CodeModel:
    path: str
    addr: np.ndarray            # uint32, ascending
    size: np.ndarray            # uint8, bytes
    kind: np.ndarray            # uint8, K_*
    target: np.ndarray          # uint32, direct branch target (0: none)
    line: np.ndarray            # int32, index into `lines` (-1: none)
    lines: list                 # (file index, line number)
    files: list                 # source paths
    func_start: np.ndarray      # uint32, ascending
    func_end: np.ndarray        # uint32, exclusive
    func_name: list             # demangled
    _index: dict = field(default_factory=dict, repr=False)
    _end_index: dict = field(default_factory=dict, repr=False)

    def __post_init__(self):
        self._build_index()

    def _build_index(self):
        a = self.addr.tolist()
        s = self.size.tolist()
        self._index = {x: i for i, x in enumerate(a)}
        self._end_index = {x + n: i for i, (x, n) in enumerate(zip(a, s))}

    def __getstate__(self):
        d = dict(self.__dict__)
        d['_index'] = {}
        d['_end_index'] = {}
        return d

    def __setstate__(self, d):
        self.__dict__.update(d)
        self._build_index()

    def __len__(self) -> int:
        return len(self.addr)

    def index(self, addr: int) -> int:
        """Instruction index at `addr`, -1 if no instruction starts there."""
        i = self._index.get(addr)
        if i is None and addr >> 28 == 0xA:          # the non-cached alias of flash
            i = self._index.get(addr - 0x20000000)
        return -1 if i is None else i

    def index_ending(self, end: int) -> int:
        """Instruction index whose last byte is just before `end`, -1 if none."""
        i = self._end_index.get(end)
        if i is None and end >> 28 == 0xA:
            i = self._end_index.get(end - 0x20000000)
        return -1 if i is None else i

    def function_at(self, addr: int) -> int:
        """Function index containing `addr`, -1 if none."""
        if addr >> 28 == 0xA:
            addr -= 0x20000000
        k = bisect.bisect_right(self.func_start, addr) - 1
        if k >= 0 and addr < self.func_end[k]:
            return k
        return -1

    def function_name(self, addr: int) -> str:
        k = self.function_at(addr)
        return self.func_name[k] if k >= 0 else '0x%08x' % addr

    def source_of(self, i: int) -> tuple[str, int] | None:
        li = int(self.line[i])
        if li < 0:
            return None
        f, n = self.lines[li]
        return self.files[f], n


def _code_sections(elf: str) -> list[str]:
    """The executable sections (SHF_EXECINSTR) by name."""
    from elftools.elf.elffile import ELFFile
    with open(elf, 'rb') as f:
        return [s.name for s in ELFFile(f).iter_sections()
                if s['sh_flags'] & 0x4 and s['sh_size'] and s['sh_type'] == 'SHT_PROGBITS']


def _tool_command(tool: str, env_var: str, args: list[str], path: str | None = None) -> list[str]:
    """A TriCore binutils program (tool = 'objdump', 'c++filt'): $env_var,
    tricore-elf-<tool> on PATH, the trigcc install, or (on Windows) the same in
    WSL.  `path`, a file argument, goes last (translated for WSL)."""
    name = 'tricore-elf-' + tool
    tail = [path] if path else []
    env = os.environ.get(env_var)
    if env:
        return shlex.split(env) + args + tail
    exe = shutil.which(name)
    if exe:
        return [exe] + args + tail
    local = os.path.expanduser('~/.local/opt/trigcc/bin/' + name)
    if os.path.exists(local):
        return [local] + args + tail
    if sys.platform == 'win32' and shutil.which('wsl'):
        script = ('exe=$(command -v %s || echo "$HOME/.local/opt/trigcc/bin/%s"); f="$1"; shift; '
                  'if [ -n "$f" ]; then exec "$exe" "$@" "$(wslpath -u "$f")"; '
                  'else exec "$exe" "$@"; fi' % (name, name))
        return ['wsl', '-e', 'bash', '-lc', script, tool,
                os.path.abspath(path) if path else ''] + args
    raise FileNotFoundError('%s not found: put it on PATH or set %s' % (name, env_var))


def _objdump_command(elf: str) -> list[str]:
    """-D over the executable sections: -d shows objects typed as data (a jump
    table such as PxSvDisptab) as bytes, but the core runs them.  -C: labels
    demangled."""
    args = ['-D', '-l', '-C']
    for name in _code_sections(elf):
        args += ['-j', name]
    return _tool_command('objdump', 'MCDS_OBJDUMP', args, elf)


def demangle_all(names: list[str]) -> list[str]:
    """C++ (Itanium) names demangled by c++filt, in one run; names it does
    not know come back as they are, and so does everything without c++filt."""
    mangled = [k for k, n in enumerate(names) if n.startswith('_Z')]
    if not mangled:
        return list(names)
    try:
        cmd = _tool_command('c++filt', 'MCDS_CXXFILT', [])
        out = subprocess.run(cmd, input='\n'.join(names[k] for k in mangled) + '\n',
                             capture_output=True, text=True, errors='replace', check=True,
                             timeout=120).stdout.split('\n')
    except (OSError, subprocess.SubprocessError, FileNotFoundError):
        return list(names)
    res = list(names)
    for k, d in zip(mangled, out):
        if d.strip():
            res[k] = d.strip()
    return res


def _functions(path: str) -> tuple[list, list, list]:
    from elftools.elf.elffile import ELFFile
    with open(path, 'rb') as f:
        elf = ELFFile(f)
        tab = elf.get_section_by_name('.symtab')
        code = [(s['sh_addr'], s['sh_addr'] + s['sh_size']) for s in elf.iter_sections()
                if s['sh_flags'] & 0x4 and s['sh_size']]
        seen = {}
        if tab is not None:
            for sym in tab.iter_symbols():
                kind = sym['st_info']['type']
                a, n = sym['st_value'] & ~1, sym['st_size']
                if not sym.name or not a:
                    continue
                # functions; and sized objects in executable sections (a jump
                # table such as PxSvDisptab runs too)
                if kind != 'STT_FUNC' and not (kind == 'STT_OBJECT' and n and
                                               any(lo <= a < hi for lo, hi in code)):
                    continue
                if a not in seen or (n and not seen[a][1]):
                    seen[a] = (sym.name, n)
    starts = sorted(seen)
    names, ends = [], []
    for k, a in enumerate(starts):
        name, n = seen[a]
        nxt = starts[k + 1] if k + 1 < len(starts) else a + max(n, 2)
        ends.append(a + n if n else nxt)
        names.append(name)
    return starts, ends, demangle_all(names)


def parse_objdump(text: str):
    """(addr, size, kind, target, line, lines, files, labels) from `objdump -d -l` output."""
    addr, size, kind, target, line = [], [], [], [], []
    lines, line_ids, files, file_ids = [], {}, [], {}
    labels = {}
    cur_line = -1
    for raw in text.splitlines():
        if not raw:
            continue
        m = _RE_INSN.match(raw)
        if m:
            a = int(m.group(1), 16)
            n = len(m.group(2).split())
            mn, ops = m.group(3), m.group(4)
            k = kind_of(mn, ops)
            t = 0
            if k in DIRECT:
                tm = _RE_TARGET.search(ops)
                if tm:
                    t = int(tm.group(1), 16)
                else:
                    k = K_SEQ if k == K_COND else k
            addr.append(a)
            size.append(n)
            kind.append(k)
            target.append(t)
            line.append(cur_line)
            continue
        if raw[0] in '0123456789abcdef':
            m = _RE_FUNC.match(raw)
            if m:
                labels[int(m.group(1), 16)] = m.group(2)
                cur_line = -1
                continue
        if raw.startswith('Disassembly of section'):
            cur_line = -1
            continue
        m = _RE_LINE.match(raw)
        if m and ('/' in m.group(1) or '\\' in m.group(1)):
            fn = m.group(1)
            fi = file_ids.get(fn)
            if fi is None:
                fi = file_ids[fn] = len(files)
                files.append(fn)
            key = (fi, int(m.group(2)))
            li = line_ids.get(key)
            if li is None:
                li = line_ids[key] = len(lines)
                lines.append(key)
            cur_line = li
    return addr, size, kind, target, line, lines, files, labels


def _cache_path(path: str) -> str:
    st = os.stat(path)
    key = '%s|%d|%d|%d' % (os.path.abspath(path), st.st_size, int(st.st_mtime), CACHE_VERSION)
    base = os.environ.get('XDG_CACHE_HOME') or os.path.join(os.path.expanduser('~'), '.cache')
    d = os.path.join(base, 'mcds-trace')
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, hashlib.sha1(key.encode()).hexdigest() + '.code')


def build(path: str, objdump_text: str | None = None) -> CodeModel:
    if objdump_text is None:
        objdump_text = subprocess.run(_objdump_command(path), check=True, capture_output=True,
                                      text=True, errors='replace').stdout
    addr, size, kind, target, line, lines, files, labels = parse_objdump(objdump_text)
    order = np.argsort(np.asarray(addr, dtype=np.uint64), kind='stable')
    starts, ends, names = _functions(path)
    if not starts:                                   # stripped: objdump's labels
        starts = sorted(labels)
        ends = starts[1:] + [starts[-1] + 2] if starts else []
        names = [labels[a] for a in starts]
    return CodeModel(path,
                     np.asarray(addr, dtype=np.uint32)[order],
                     np.asarray(size, dtype=np.uint8)[order],
                     np.asarray(kind, dtype=np.uint8)[order],
                     np.asarray(target, dtype=np.uint32)[order],
                     np.asarray(line, dtype=np.int32)[order],
                     lines, files,
                     np.asarray(starts, dtype=np.uint32), np.asarray(ends, dtype=np.uint32),
                     names)


def load(path: str, use_cache: bool = True) -> CodeModel:
    cache = None
    if use_cache:
        try:
            cache = _cache_path(path)
            with open(cache, 'rb') as f:
                model = pickle.load(f)
            if isinstance(model, CodeModel):
                return model
        except (OSError, pickle.UnpicklingError, EOFError, AttributeError, ImportError):
            pass
    model = build(path)
    if cache:
        try:
            tmp = cache + '.tmp'
            with open(tmp, 'wb') as f:
                pickle.dump(model, f, protocol=pickle.HIGHEST_PROTOCOL)
            os.replace(tmp, cache)
        except (OSError, pickle.PicklingError):
            pass
    return model
