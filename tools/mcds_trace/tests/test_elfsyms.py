"""ELF/DWARF symbol model, against small programs built with the host compiler."""

import os
import shutil
import struct
import subprocess
import textwrap

import pytest

from mcds_trace import elfsyms

C_SRC = textwrap.dedent('''
    #include <stdint.h>
    typedef enum { MODE_OFF = 0, MODE_ON = 1, MODE_FAULT = -1 } mode_t_;
    typedef struct {
        uint8_t  a : 3;
        uint8_t  b : 5;
        int16_t  c : 12;
        uint16_t d : 4;
    } bits_t;
    struct inner { float f; int32_t s32; };
    struct outer {
        uint32_t        u32;
        struct inner    in;
        uint16_t        arr[4];
        struct inner    grid[2][3];
        mode_t_         mode;
        bits_t          bits;
        double          dbl;
        const volatile uint8_t *ptr;
    };
    struct outer g_outer = { 1 };
    volatile uint64_t g_u64 = 7;
    static int16_t s_counter;
    int16_t *use(void) { static uint32_t fn_static; fn_static++; return &s_counter; }
    int main(void) { return (int)g_outer.u32 + (int)*use(); }
''')

CPP_SRC = textwrap.dedent('''
    #include <cstdint>
    namespace app {
    class Ctrl {
    public:
        static uint32_t instances;
        uint16_t count = 0;
        int32_t  level = 0;
    };
    uint32_t Ctrl::instances = 3;
    Ctrl theCtrl;
    }
    int main() { return app::theCtrl.count + app::Ctrl::instances; }
''')


def _build(tmp_path, src: str, name: str, lang: str, dwarf: int) -> str:
    cc = shutil.which('g++' if lang == 'c++' else 'gcc')
    if not cc:
        pytest.skip('no host compiler')
    s = tmp_path / ('%s.%s' % (name, 'cpp' if lang == 'c++' else 'c'))
    s.write_text(src)
    out = str(tmp_path / name)
    subprocess.run([cc, '-g', '-gdwarf-%d' % dwarf, '-O0', '-no-pie', '-o', out, str(s)],
                   check=True, capture_output=True)
    return out


def _nm(path: str) -> dict:
    out = subprocess.run(['nm', '-S', path], capture_output=True, text=True).stdout
    syms = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) == 4:
            syms[p[3]] = int(p[0], 16)
    return syms


@pytest.mark.parametrize('dwarf', [2, 4])
def test_c_types(tmp_path, dwarf):
    elf = _build(tmp_path, C_SRC, 'prog%d' % dwarf, 'c', dwarf)
    tab = elfsyms.load(elf, use_cache=False)
    syms = _nm(elf)
    v = tab.variables['g_outer']
    assert v.addr == syms['g_outer']
    assert v.type.kind == 'struct' and v.type.size == 104 or v.type.size > 60
    node = elfsyms.node_of(v)
    kids = {c.name: c for c in node.children()}
    assert kids['u32'].is_leaf and kids['u32'].addr == v.addr
    assert kids['in'].expandable and not kids['in'].is_leaf
    f = [c for c in kids['in'].children() if c.name == 'f'][0]
    assert f.type.encoding == 'float' and f.size == 4
    arr = kids['arr']
    assert arr.type.kind == 'array' and arr.type.count == 4 and arr.child_count() == 4
    e2 = arr.children(2, 1)[0]
    assert e2.path == 'g_outer.arr[2]' and e2.addr == arr.addr + 4
    grid = kids['grid']
    assert grid.type.count == 2 and grid.children()[1].type.count == 3
    g12 = grid.children()[1].children()[2]
    assert g12.addr == grid.addr + (1 * 3 + 2) * 8
    mode = kids['mode']
    assert mode.type.kind == 'enum' and mode.type.enumerators[1] == 'MODE_ON'
    assert kids['ptr'].type.kind == 'pointer'
    assert kids['dbl'].type.size == 8
    # Every leaf resolves back from its path.
    for leaf in node.leaves():
        r = elfsyms.resolve_path(tab.variables, leaf.path)
        assert r is not None and r.addr == leaf.addr and r.bit_shift == leaf.bit_shift, leaf.path
    # Statics: file scope and function scope.
    assert tab.variables['s_counter'].addr == syms['s_counter']
    fn = [k for k in tab.variables if k.endswith('fn_static')]
    assert fn and tab.variables[fn[0]].type.size == 4
    assert tab.variables['g_u64'].type.size == 8


@pytest.mark.parametrize('dwarf', [2, 4])
def test_bitfields_decode(tmp_path, dwarf):
    elf = _build(tmp_path, C_SRC, 'bits%d' % dwarf, 'c', dwarf)
    tab = elfsyms.load(elf, use_cache=False)
    bits = [c for c in elfsyms.node_of(tab.variables['g_outer']).children() if c.name == 'bits'][0]
    f = {c.name: c for c in bits.children()}
    assert f['a'].bit_size == 3 and f['b'].bit_size == 5 and f['c'].bit_size == 12
    # Place a=5, b=17, c=-3, d=9 as gcc lays them out (little-endian, LSB first).
    raw = bytearray(bits.size)
    byte0 = 5 | (17 << 3)
    raw[0] = byte0
    unit = int.from_bytes(raw[2:4], 'little')
    raw[2:4] = ((unit & ~0xFFF) | ((-3) & 0xFFF) | (9 << 12)).to_bytes(2, 'little')

    def val(n):
        base = n.addr - bits.addr
        return elfsyms.decode_leaf(n, bytes(raw[base:base + n.size]))
    assert val(f['a']) == 5
    assert val(f['b']) == 17
    assert val(f['c']) == -3
    assert val(f['d']) == 9


def test_cpp_members_and_statics(tmp_path):
    elf = _build(tmp_path, CPP_SRC, 'cpp', 'c++', 4)
    tab = elfsyms.load(elf, use_cache=False)
    assert 'app::theCtrl' in tab.variables
    assert 'app::Ctrl::instances' in tab.variables
    node = elfsyms.node_of(tab.variables['app::theCtrl'])
    names = [c.name for c in node.children()]
    assert names == ['count', 'level']            # the static member is not a field
    r = elfsyms.resolve_path(tab.variables, 'app::theCtrl.level')
    assert r is not None and r.addr == node.addr + 4


def test_search_and_cache(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path / 'cache'))
    elf = _build(tmp_path, C_SRC, 'cache', 'c', 4)
    t1 = elfsyms.load(elf)
    t2 = elfsyms.load(elf)                        # from the cache
    assert set(t1.variables) == set(t2.variables)
    hits = t1.search('outer')
    assert hits and hits[0].name == 'g_outer'
    assert t1.by_address(t1.variables['g_outer'].addr + 5).name == 'g_outer'


def test_decode_values():
    u16 = elfsyms.CType('base', 'uint16', 2, 'unsigned')
    s16 = elfsyms.CType('base', 'int16', 2, 'signed')
    f32 = elfsyms.CType('base', 'float', 4, 'float')
    assert elfsyms.decode_leaf(elfsyms.Node('x', 'x', 0, u16), b'\xff\xff') == 65535
    assert elfsyms.decode_leaf(elfsyms.Node('x', 'x', 0, s16), b'\xff\xff') == -1
    assert elfsyms.decode_leaf(elfsyms.Node('x', 'x', 0, f32), struct.pack('<f', 1.5)) == 1.5
    assert elfsyms.decode_leaf(elfsyms.Node('x', 'x', 0, u16), b'\x01') is None


def test_demangle():
    assert elfsyms.demangle('_ZZN12_GLOBAL__N_16getAppEvE3app') == '(anonymous)::getApp::app'
    assert elfsyms.demangle('_ZN3app4Ctrl9instancesE') == 'app::Ctrl::instances'
    assert elfsyms.demangle('plain_c') == 'plain_c'


REAL_ELF = '/home/atabey/repos/barcelonamain/build/Tricore/Debug/bin/mlpc-tricore-pxros-generic.elf'


@pytest.mark.skipif(not os.path.exists(REAL_ELF), reason='TriCore application ELF not present')
def test_tricore_application():
    tab = elfsyms.load(REAL_ELF)
    app = tab.by_address(0x5000220C)
    assert app is not None and app.name.endswith('getApp::app')
    leaf = elfsyms.resolve_path(tab.variables, app.name + '.mCyclesUntilSecond')
    assert leaf is not None and leaf.addr == 0x5000220C and leaf.size == 2
