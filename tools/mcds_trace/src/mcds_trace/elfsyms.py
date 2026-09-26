"""Global variables and their types from an ELF's DWARF, for picking signals.

Only what a data trace can watch is collected: objects at a fixed address
(DW_OP_addr) - globals, namespace members, C++ static members and function
statics.  Types are resolved into a small model (CType) and a variable is
browsed as a tree of Nodes: structs and unions expand into members, arrays
into elements, numbers, enums, pointers and bitfields are leaves that can be
plotted.

Parsing takes seconds on a large ELF, so the result is pickled next to the
user cache, keyed by path, size and mtime.
"""

from __future__ import annotations

import hashlib
import os
import pickle
import struct
from dataclasses import dataclass, field
from typing import Callable, Iterator

CACHE_VERSION = 5

# Leaf kinds a Node can decode.
NUMERIC_KINDS = ('base', 'enum', 'pointer')


@dataclass
class Member:
    name: str
    offset: int                  # bytes from the start of the aggregate
    type: 'CType'
    bit_size: int = 0            # 0: not a bitfield
    bit_shift: int = 0           # bitfield: LSB position within the storage unit


@dataclass
class CType:
    kind: str                    # base enum pointer struct union class array void func unknown
    name: str = ''
    size: int = 0
    encoding: str = ''           # base/enum: signed unsigned float bool char
    members: list = field(default_factory=list)       # struct/union/class: [Member]
    elem: 'CType | None' = None                        # array / pointer target (arrays only)
    count: int = 0               # array: elements (0: unknown / flexible)
    enumerators: dict = field(default_factory=dict)    # enum: value -> name
    target: str = ''             # pointer: pointee type name

    @property
    def is_aggregate(self) -> bool:
        return self.kind in ('struct', 'union', 'class', 'array')

    def display(self) -> str:
        if self.kind == 'array':
            inner = self.elem.display() if self.elem else '?'
            return '%s[%s]' % (inner, self.count or '')
        if self.kind == 'pointer':
            return '%s*' % (self.target or 'void')
        if self.kind in ('struct', 'union', 'class') and self.name:
            return self.name
        return self.name or self.kind


@dataclass
class Variable:
    name: str                    # qualified: ns::Class::member, func::static
    addr: int
    type: CType
    file: str = ''               # compilation unit


@dataclass
class Node:
    """One browsable piece of a variable: the variable itself, a member or an
    element.  Leaves (numbers, enums, pointers, bitfields) can be traced."""
    path: str
    name: str
    addr: int
    type: CType
    bit_size: int = 0
    bit_shift: int = 0

    @property
    def size(self) -> int:
        return self.type.size

    @property
    def is_leaf(self) -> bool:
        return self.bit_size > 0 or self.type.kind in NUMERIC_KINDS

    @property
    def expandable(self) -> bool:
        t = self.type
        if t.kind == 'array':
            return t.count > 0 and t.elem is not None
        return t.kind in ('struct', 'union', 'class') and bool(t.members)

    def type_name(self) -> str:
        if self.bit_size:
            return '%s:%d' % (self.type.display(), self.bit_size)
        return self.type.display()

    def child_count(self) -> int:
        t = self.type
        if t.kind == 'array':
            return t.count
        if t.kind in ('struct', 'union', 'class'):
            return len(t.members)
        return 0

    def children(self, start: int = 0, limit: int | None = None) -> list['Node']:
        t = self.type
        out = []
        if t.kind == 'array' and t.elem is not None:
            stop = t.count if limit is None else min(t.count, start + limit)
            esize = t.elem.size
            for i in range(start, stop):
                out.append(Node('%s[%d]' % (self.path, i), '[%d]' % i,
                                self.addr + i * esize, t.elem))
        elif t.kind in ('struct', 'union', 'class'):
            mem = t.members[start:] if limit is None else t.members[start:start + limit]
            for m in mem:
                out.append(Node('%s.%s' % (self.path, m.name), m.name, self.addr + m.offset,
                                m.type, m.bit_size, m.bit_shift))
        return out

    def leaves(self, max_leaves: int = 100000) -> Iterator['Node']:
        """Every traceable leaf below this node, depth first."""
        stack = [self]
        n = 0
        while stack and n < max_leaves:
            node = stack.pop()
            if node.is_leaf:
                n += 1
                yield node
            elif node.expandable:
                stack.extend(reversed(node.children()))


def node_of(var: Variable) -> Node:
    return Node(var.name, var.name, var.addr, var.type)


def resolve_path(variables: dict[str, Variable], path: str) -> Node | None:
    """'a.b[3].c' -> its Node, or None when it no longer exists (ELF rebuilt)."""
    var = variables.get(path)
    if var is not None:
        return node_of(var)
    # The longest variable name that prefixes the path (names may hold '::').
    best = None
    for name in _prefix_candidates(path):
        if name in variables:
            best = name
            break
    if best is None:
        return None
    node = node_of(variables[best])
    rest = path[len(best):]
    while rest:
        if rest.startswith('.'):
            end = min([i for i in (rest.find('.', 1), rest.find('[', 1)) if i > 0] or [len(rest)])
            key, rest = rest[1:end], rest[end:]
            if node.type.kind not in ('struct', 'union', 'class'):
                return None
            match = [c for c in node.children() if c.name == key]
            if not match:
                return None
            node = match[0]
        elif rest.startswith('['):
            end = rest.find(']')
            if end < 0 or node.type.kind != 'array':
                return None
            idx = int(rest[1:end])
            if not 0 <= idx < node.type.count:
                return None
            node = node.children(idx, 1)[0]
            rest = rest[end + 1:]
        else:
            return None
    return node


def _prefix_candidates(path: str) -> list[str]:
    cuts = [i for i, ch in enumerate(path) if ch in '.[']
    return [path[:i] for i in reversed(cuts)]


# -- value decoding -----------------------------------------------------------

def decode_leaf(node: Node, raw: bytes) -> float | int | None:
    """The value of a leaf from its bytes (little-endian, as on TriCore)."""
    t = node.type
    if len(raw) < t.size or t.size == 0:
        return None
    if node.bit_size:
        unit = int.from_bytes(raw[:t.size], 'little')
        v = (unit >> node.bit_shift) & ((1 << node.bit_size) - 1)
        if t.encoding == 'signed' and v & (1 << (node.bit_size - 1)):
            v -= 1 << node.bit_size
        return v
    if t.kind == 'base' and t.encoding == 'float':
        if t.size == 4:
            return struct.unpack('<f', raw[:4])[0]
        if t.size == 8:
            return struct.unpack('<d', raw[:8])[0]
        return None
    signed = t.encoding == 'signed' or (t.kind == 'enum' and t.encoding == 'signed')
    return int.from_bytes(raw[:t.size], 'little', signed=signed)


def format_value(node: Node, value) -> str:
    if value is None:
        return '?'
    t = node.type
    if t.kind == 'enum':
        name = t.enumerators.get(int(value))
        return '%s (%d)' % (name, value) if name else str(value)
    if t.kind == 'pointer':
        return '0x%08X' % int(value)
    if t.encoding == 'bool':
        return 'true' if value else 'false'
    if isinstance(value, float):
        return '%.6g' % value
    return str(value)


# -- DWARF --------------------------------------------------------------------

_BASE_ENC = {
    0x01: 'unsigned',   # address
    0x02: 'bool',
    0x04: 'float',
    0x05: 'signed',
    0x06: 'char',       # signed_char
    0x07: 'unsigned',
    0x08: 'unsigned',   # unsigned_char
    0x10: 'unsigned',   # UTF
}


def _uleb(data, pos: int = 0) -> tuple[int, int]:
    v = shift = 0
    while True:
        b = data[pos]
        pos += 1
        v |= (b & 0x7F) << shift
        shift += 7
        if not b & 0x80:
            return v, pos


def _location_addr(attr) -> int | None:
    """A static location: DW_OP_addr <addr> (possibly followed by nothing)."""
    val = attr.value
    if not isinstance(val, (list, tuple, bytes)) or len(val) < 5 or val[0] != 0x03:
        return None
    if len(val) not in (5, 9):
        return None                    # an expression, not a plain address
    return int.from_bytes(bytes(val[1:]), 'little')     # 32- or 64-bit targets


class _Parser:
    def __init__(self, dwarf, progress: Callable[[float], None] | None = None):
        self.dwarf = dwarf
        self.types: dict[int, CType] = {}
        self.defs: dict[str, object] | None = None      # complete aggregates by name
        self.progress = progress
        # Function statics DWARF gives no location (the function was inlined):
        # (function source names, variable name, type), placed from the
        # symbol table afterwards.
        self.unplaced: list[tuple[list[str], str, CType]] = []

    # types -------------------------------------------------------------------

    def _type_die(self, die):
        attr = die.attributes.get('DW_AT_type')
        if attr is None:
            return None
        try:
            return die.get_DIE_from_attribute('DW_AT_type')
        except Exception:
            return None

    def type_of(self, die) -> CType:
        tdie = self._type_die(die)
        if tdie is None:
            # A definition of a declared object (C++ statics, namespace
            # variables) keeps its type on the declaration.
            for attr in ('DW_AT_specification', 'DW_AT_abstract_origin'):
                if attr in die.attributes:
                    try:
                        return self.type_of(die.get_DIE_from_attribute(attr))
                    except Exception:
                        pass
            return CType('void', 'void')
        return self.resolve(tdie)

    def resolve(self, die, depth: int = 0) -> CType:
        key = die.offset
        cached = self.types.get(key)
        if cached is not None:
            return cached
        if depth > 64:
            return CType('unknown', '?')
        tag = die.tag
        name = _name(die)
        size = _int_attr(die, 'DW_AT_byte_size', 0)

        if tag in ('DW_TAG_typedef', 'DW_TAG_const_type', 'DW_TAG_volatile_type',
                   'DW_TAG_restrict_type', 'DW_TAG_atomic_type', 'DW_TAG_immutable_type',
                   'DW_TAG_packed_type', 'DW_TAG_shared_type'):
            inner_die = self._type_die(die)
            inner = self.resolve(inner_die, depth + 1) if inner_die is not None else CType('void', 'void')
            if tag == 'DW_TAG_typedef' and name and inner.kind in NUMERIC_KINDS:
                # Keep the typedef's name (uint16, Ifx_UReg_32Bit) on numbers.
                t = CType(inner.kind, name, inner.size, inner.encoding,
                          enumerators=inner.enumerators, target=inner.target)
            else:
                t = inner
            self.types[key] = t
            return t

        if tag == 'DW_TAG_base_type':
            enc = _int_attr(die, 'DW_AT_encoding', 0x07)
            t = CType('base', name, size, _BASE_ENC.get(enc, 'unsigned'))
        elif tag == 'DW_TAG_enumeration_type':
            enums = {}
            for c in die.iter_children():
                if c.tag == 'DW_TAG_enumerator':
                    enums[_int_attr(c, 'DW_AT_const_value', 0)] = _name(c)
            under = self._type_die(die)
            enc = 'unsigned'
            if under is not None:
                enc = self.resolve(under, depth + 1).encoding or 'unsigned'
            elif any(v < 0 for v in enums):
                enc = 'signed'
            t = CType('enum', name or 'enum', size or 4, enc, enumerators=enums)
        elif tag in ('DW_TAG_pointer_type', 'DW_TAG_reference_type',
                     'DW_TAG_rvalue_reference_type', 'DW_TAG_ptr_to_member_type'):
            target = self._type_die(die)
            tname = _name(target) if target is not None else 'void'
            t = CType('pointer', '', size or 4, 'unsigned', target=tname or '?')
        elif tag in ('DW_TAG_structure_type', 'DW_TAG_union_type', 'DW_TAG_class_type'):
            kind = {'DW_TAG_structure_type': 'struct', 'DW_TAG_union_type': 'union',
                    'DW_TAG_class_type': 'class'}[tag]
            if 'DW_AT_declaration' in die.attributes and name:
                full = self._definition(name, tag)
                if full is not None and full.offset != die.offset:
                    t = self.resolve(full, depth + 1)
                    self.types[key] = t
                    return t
            t = CType(kind, name, size)
            self.types[key] = t                     # before members: self-reference
            t.members = self._members(die, size, depth)
            return t
        elif tag == 'DW_TAG_array_type':
            elem_die = self._type_die(die)
            elem = self.resolve(elem_die, depth + 1) if elem_die is not None else CType('unknown', '?')
            dims = []
            for c in die.iter_children():
                if c.tag == 'DW_TAG_subrange_type':
                    if 'DW_AT_count' in c.attributes:
                        dims.append(_int_attr(c, 'DW_AT_count', 0))
                    elif 'DW_AT_upper_bound' in c.attributes:
                        ub = c.attributes['DW_AT_upper_bound'].value
                        lb = _int_attr(c, 'DW_AT_lower_bound', 0)
                        dims.append(ub - lb + 1 if isinstance(ub, int) else 0)
                    else:
                        dims.append(0)
            if not dims:
                dims = [0]
            t = elem
            for n in reversed(dims):
                t = CType('array', '', n * t.size, elem=t, count=n)
        elif tag == 'DW_TAG_subroutine_type':
            t = CType('func', 'function', 0)
        elif tag == 'DW_TAG_unspecified_type':
            t = CType('void', name or 'void', 0)
        else:
            t = CType('unknown', name or tag, size)
        self.types[key] = t
        return t

    def _members(self, die, size: int, depth: int) -> list[Member]:
        members = []
        for c in die.iter_children():
            if c.tag == 'DW_TAG_inheritance':
                base = self.type_of(c)
                members.append(Member('<%s>' % (base.name or 'base'), _member_offset(c), base))
                continue
            if c.tag != 'DW_TAG_member':
                continue
            if 'DW_AT_external' in c.attributes or 'DW_AT_declaration' in c.attributes:
                continue                            # C++ static member: defined elsewhere
            mt = self.type_of(c)
            off = _member_offset(c)
            bits = _int_attr(c, 'DW_AT_bit_size', 0)
            shift = 0
            if bits:
                unit = _int_attr(c, 'DW_AT_byte_size', mt.size) or mt.size
                if 'DW_AT_data_bit_offset' in c.attributes:       # DWARF 4+
                    dbo = _int_attr(c, 'DW_AT_data_bit_offset', 0)
                    off = (dbo // (unit * 8)) * unit
                    shift = dbo - off * 8
                elif 'DW_AT_bit_offset' in c.attributes:          # DWARF 2/3, from the MSB
                    shift = unit * 8 - _int_attr(c, 'DW_AT_bit_offset', 0) - bits
                if unit != mt.size and mt.kind in ('base', 'enum'):
                    mt = CType(mt.kind, mt.name, unit, mt.encoding, enumerators=mt.enumerators)
            members.append(Member(_name(c) or '<anon@%d>' % off, off, mt, bits, max(shift, 0)))
        return members

    def _definition(self, name: str, tag: str):
        """A complete struct/class/union of this name anywhere in the ELF."""
        if self.defs is None:
            self.defs = {}
            for cu in self.dwarf.iter_CUs():
                for d in cu.iter_DIEs():
                    if d.tag in ('DW_TAG_structure_type', 'DW_TAG_class_type',
                                 'DW_TAG_union_type') and 'DW_AT_declaration' not in d.attributes:
                        n = _name(d)
                        if n and (n, d.tag) not in self.defs:
                            self.defs[(n, d.tag)] = d
        return self.defs.get((name, tag)) or self.defs.get((name, 'DW_TAG_structure_type')) \
            or self.defs.get((name, 'DW_TAG_class_type'))

    # variables ---------------------------------------------------------------

    def variables(self) -> dict[str, Variable]:
        out: dict[str, Variable] = {}
        cus = list(self.dwarf.iter_CUs())
        for i, cu in enumerate(cus):
            top = cu.get_top_DIE()
            cu_name = _name(top)
            self._scan(top, [], cu_name, out, in_func=None)
            if self.progress and (i % 16 == 0 or i == len(cus) - 1):
                self.progress((i + 1) / len(cus))
        return out

    def _scan(self, die, scope: list[str], cu_name: str, out: dict, in_func: str | None):
        for c in die.iter_children():
            tag = c.tag
            if tag == 'DW_TAG_variable':
                loc = c.attributes.get('DW_AT_location')
                if loc is None:
                    continue
                addr = _location_addr(loc)
                if not addr:
                    continue          # none, or discarded (address 0)
                name, spec_scope = self._var_name(c)
                if not name:
                    continue
                parts = (spec_scope if spec_scope is not None else scope) + [name]
                if in_func:
                    parts = [in_func] + parts
                qname = '::'.join(parts)
                try:
                    t = self.type_of(c)
                except Exception:
                    t = CType('unknown', '?')
                if t.size == 0 and t.kind not in ('array',):
                    continue
                key = qname
                prev = out.get(key)
                if prev is not None and prev.addr != addr:
                    key = '%s@%08X' % (qname, addr)      # file statics with one name
                out.setdefault(key, Variable(key, addr, t, cu_name))
            elif tag == 'DW_TAG_namespace':
                self._scan(c, scope + [_name(c) or '(anonymous)'], cu_name, out, in_func)
            elif tag in ('DW_TAG_subprogram',):
                if 'DW_AT_declaration' in c.attributes:
                    continue
                fname = self._func_name(c, scope)
                self._scan_func(c, fname, cu_name, out, 'DW_AT_inline' in c.attributes)

    def _scan_func(self, die, fname: str, cu_name: str, out: dict, inlined: bool = False):
        for c in die.iter_children():
            if c.tag == 'DW_TAG_variable' and 'DW_AT_location' not in c.attributes \
                    and 'DW_AT_declaration' not in c.attributes and _name(c) and inlined:
                # A static of an inlined function: only the symbol table knows
                # where it lives.  (Automatic variables of inlined functions
                # look the same and simply find no symbol.)
                try:
                    self.unplaced.append((fname.split('::'), _name(c), self.type_of(c)))
                except Exception:
                    pass
                continue
            if c.tag == 'DW_TAG_variable' and 'DW_AT_location' in c.attributes:
                addr = _location_addr(c.attributes['DW_AT_location'])
                if not addr:
                    continue          # none, or discarded (address 0)
                n = _name(c)
                if not n:
                    continue
                qname = '%s::%s' % (fname, n)
                try:
                    t = self.type_of(c)
                except Exception:
                    continue
                if t.size == 0:
                    continue
                key = qname if qname not in out or out[qname].addr == addr else \
                    '%s@%08X' % (qname, addr)
                out.setdefault(key, Variable(key, addr, t, cu_name))
            elif c.tag == 'DW_TAG_lexical_block':
                self._scan_func(c, fname, cu_name, out, inlined)

    # symbol table --------------------------------------------------------------

    def place_from_symbols(self, symbols: list[tuple[str, int, int]], out: dict) -> None:
        """Use the ELF symbol table (name, addr, size) for what DWARF left out:
        statics of inlined functions (typed from DWARF), then every other data
        object as an untyped variable."""
        by_local: dict[str, list[tuple[str, int, int]]] = {}
        for sym in symbols:
            m = _LOCAL_RE.search(sym[0]) if sym[0].startswith('_ZZ') else None
            if m:
                by_local.setdefault(m.group(2), []).append(sym)
        for fparts, vname, t in self.unplaced:
            fn = fparts[-1] if fparts else ''
            tag = '%d%s' % (len(fn), fn)
            cands = {(a, s) for n, a, s in by_local.get(vname, ()) if tag in n}
            if len(cands) != 1:
                continue
            addr, _ = cands.pop()
            key = '::'.join(fparts + [vname])
            if key in out and out[key].addr != addr:
                key = '%s@%08X' % (key, addr)
            out.setdefault(key, Variable(key, addr, t, ''))

        covered = sorted((v.addr, v.addr + max(v.type.size, 1)) for v in out.values())
        starts = [lo for lo, _ in covered]
        import bisect
        for name, addr, size in symbols:
            i = bisect.bisect_right(starts, addr) - 1
            if i >= 0 and covered[i][0] <= addr < covered[i][1]:
                continue
            pretty = demangle(name)
            if size in (1, 2, 4, 8):
                t = CType('base', 'uint%d' % (size * 8), size, 'unsigned')
            else:
                byte = CType('base', 'uint8', 1, 'unsigned')
                t = CType('array', '', size, elem=byte, count=size)
            key = pretty if pretty not in out else '%s@%08X' % (pretty, addr)
            out.setdefault(key, Variable(key, addr, t, '(symbol table, no debug info)'))

    def _func_name(self, die, scope: list[str]) -> str:
        n = _name(die)
        spec_scope = None
        for attr in ('DW_AT_specification', 'DW_AT_abstract_origin'):
            if not n and attr in die.attributes:
                try:
                    spec = die.get_DIE_from_attribute(attr)
                    n = _name(spec)
                    spec_scope = _scope_of(spec)
                except Exception:
                    pass
        return '::'.join((spec_scope if spec_scope is not None else scope) + [n or '?'])

    def _var_name(self, die) -> tuple[str, list[str] | None]:
        n = _name(die)
        if 'DW_AT_specification' in die.attributes:
            try:
                spec = die.get_DIE_from_attribute('DW_AT_specification')
                return (n or _name(spec)), _scope_of(spec)
            except Exception:
                pass
        return n, None


# _ZZ<function>E<len><name>[_<discriminator>]: a function-local static.
import re as _re
_LOCAL_RE = _re.compile(r'E(\d+)([A-Za-z_]\w*?)(?:_\d+)?$')


def _source_names(s: str, i: int) -> tuple[list[str], int]:
    """<len><id> sequence at s[i:] (stops at anything else)."""
    parts = []
    while i < len(s) and s[i].isdigit():
        j = i
        while j < len(s) and s[j].isdigit():
            j += 1
        n = int(s[i:j])
        ident = s[j:j + n]
        parts.append('(anonymous)' if ident.startswith('_GLOBAL__N') else ident)
        i = j + n
        while i < len(s) and s[i] in 'KVr':
            i += 1
    return parts, i


def demangle(name: str) -> str:
    """Enough Itanium demangling for data symbols: nested names and function
    statics; anything else is returned as is."""
    try:
        if name.startswith('_ZZ'):
            m = _LOCAL_RE.search(name)
            body = name[3:]
            parts, _ = _source_names(body[1:], 0) if body.startswith('N') else _source_names(body, 0)
            if m and parts:
                return '::'.join(parts + [m.group(2)])
        elif name.startswith('_ZN'):
            parts, i = _source_names(name[3:], 0)
            if parts and name[3 + i:3 + i + 1] == 'E':
                return '::'.join(parts)
        elif name.startswith('_Z') and name[2:3].isdigit():
            parts, _ = _source_names(name[2:], 0)
            if parts:
                return parts[0]
    except (ValueError, IndexError):
        pass
    return name


def _scope_of(die) -> list[str]:
    """Enclosing namespaces / classes of a declaration."""
    parts = []
    p = die.get_parent()
    while p is not None and p.tag != 'DW_TAG_compile_unit':
        if p.tag in ('DW_TAG_namespace', 'DW_TAG_structure_type', 'DW_TAG_class_type',
                     'DW_TAG_union_type'):
            parts.append(_name(p) or '(anonymous)')
        p = p.get_parent()
    return list(reversed(parts))


def _name(die) -> str:
    if die is None:
        return ''
    a = die.attributes.get('DW_AT_name')
    if a is None:
        return ''
    v = a.value
    return v.decode(errors='replace') if isinstance(v, bytes) else str(v)


def _int_attr(die, name: str, default: int) -> int:
    a = die.attributes.get(name)
    if a is None:
        return default
    v = a.value
    if isinstance(v, int):
        return v
    if isinstance(v, (list, tuple)) and v and v[0] == 0x23:      # DW_OP_plus_uconst
        return _uleb(v, 1)[0]
    return default


def _member_offset(die) -> int:
    a = die.attributes.get('DW_AT_data_member_location')
    if a is None:
        return 0
    v = a.value
    if isinstance(v, int):
        return v
    if isinstance(v, (list, tuple)) and v:
        if v[0] == 0x23:                         # DW_OP_plus_uconst
            return _uleb(v, 1)[0]
        if 0x30 <= v[0] <= 0x4F:                 # DW_OP_lit0..31
            return v[0] - 0x30
        if v[0] in (0x08, 0x0A, 0x0C) and len(v) > 1:    # DW_OP_constNu
            return int.from_bytes(bytes(v[1:]), 'little')
    return 0


# -- loading ------------------------------------------------------------------

@dataclass
class SymbolTable:
    path: str
    variables: dict[str, Variable]
    machine: str = ''

    def search(self, text: str, limit: int = 500) -> list[Variable]:
        """Case-insensitive substring match on the name, prefix matches first."""
        text = text.strip().lower()
        if not text:
            return sorted(self.variables.values(), key=lambda v: v.name.lower())[:limit]
        pre, sub = [], []
        for v in self.variables.values():
            n = v.name.lower()
            leaf = n.rsplit('::', 1)[-1]
            if leaf.startswith(text) or n.startswith(text):
                pre.append(v)
            elif text in n:
                sub.append(v)
        pre.sort(key=lambda v: (len(v.name), v.name.lower()))
        sub.sort(key=lambda v: (len(v.name), v.name.lower()))
        return (pre + sub)[:limit]

    def by_address(self, addr: int) -> Variable | None:
        for v in self.variables.values():
            if v.addr <= addr < v.addr + max(v.type.size, 1):
                return v
        return None


def _data_symbols(elf) -> list[tuple[str, int, int]]:
    """(name, addr, size) of every sized data object in .symtab."""
    tab = elf.get_section_by_name('.symtab')
    if tab is None:
        return []
    out = []
    for sym in tab.iter_symbols():
        if sym['st_info']['type'] == 'STT_OBJECT' and sym['st_size'] > 0 and sym.name:
            out.append((sym.name, sym['st_value'], sym['st_size']))
    return out


def _cache_path(path: str) -> str:
    st = os.stat(path)
    key = '%s|%d|%d|%d' % (os.path.abspath(path), st.st_size, int(st.st_mtime), CACHE_VERSION)
    base = os.environ.get('XDG_CACHE_HOME') or os.path.join(os.path.expanduser('~'), '.cache')
    d = os.path.join(base, 'mcds-trace')
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, hashlib.sha1(key.encode()).hexdigest() + '.syms')


def load(path: str, progress: Callable[[float], None] | None = None,
         use_cache: bool = True) -> SymbolTable:
    """Parse (or load from the cache) the traceable variables of an ELF."""
    cache = None
    if use_cache:
        try:
            cache = _cache_path(path)
            with open(cache, 'rb') as f:
                table = pickle.load(f)
            if isinstance(table, SymbolTable):
                if progress:
                    progress(1.0)
                return table
        except (OSError, pickle.UnpicklingError, EOFError, AttributeError, ImportError):
            pass
    from elftools.elf.elffile import ELFFile
    with open(path, 'rb') as f:
        elf = ELFFile(f)
        if not elf.has_dwarf_info():
            raise ValueError('%s has no debug information (build with -g)' % path)
        parser = _Parser(elf.get_dwarf_info(), progress)
        variables = parser.variables()
        parser.place_from_symbols(_data_symbols(elf), variables)
        table = SymbolTable(path, variables, elf['e_machine'])
    if cache:
        try:
            tmp = cache + '.tmp'
            with open(tmp, 'wb') as f:
                pickle.dump(table, f, protocol=pickle.HIGHEST_PROTOCOL)
            os.replace(tmp, cache)
        except (OSError, pickle.PicklingError, RecursionError):
            pass
    return table
