"""Motorola CPU16 (68HC16) instruction interpreter.

Written against the CPU16 Reference Manual conventions and cross-checked with
the locally corrected SLEIGH model in ../recovered/tooling/68HC16.slaspec.
Only the instruction forms that occur in VIP_2A03.ori (plus their regular
siblings) are implemented; anything else raises IllegalInstruction so that a
gap is never silently executed as something else.

Conventions
  * All addresses are 20-bit linear values (bank << 16 | offset).
  * self.pc is the address of the instruction being executed. The
    architectural PC (address + 6) is only materialised for branches and for
    values pushed on the stack, so stack contents match real hardware.
  * The stack pointer addresses the next free word: push stores then
    decrements by two; TSX yields SP + 2.
"""

M20 = 0xFFFFF


class IllegalInstruction(Exception):
    pass


class CpuStopped(Exception):
    pass


def s8(v):
    return v - 0x100 if v & 0x80 else v


def s16(v):
    return v - 0x10000 if v & 0x8000 else v


# --------------------------------------------------------------------------
# Arithmetic helpers. Each takes the CPU (for flags) and returns the result,
# or None when the instruction does not store one.
# --------------------------------------------------------------------------
def add8(c, a, m, cin=0):
    r = a + m + cin
    c.H = ((a & 15) + (m & 15) + cin) >> 4
    c.C = r >> 8
    r &= 0xFF
    c.V = ((a ^ r) & (m ^ r)) >> 7
    c.N = r >> 7
    c.Z = 1 if r == 0 else 0
    return r


def sub8(c, a, m, cin=0):
    r = a - m - cin
    c.C = 1 if r < 0 else 0
    r &= 0xFF
    c.V = ((a ^ m) & (a ^ r)) >> 7
    c.N = r >> 7
    c.Z = 1 if r == 0 else 0
    return r


def add16(c, a, m, cin=0):
    r = a + m + cin
    c.C = r >> 16
    r &= 0xFFFF
    c.V = ((a ^ r) & (m ^ r)) >> 15
    c.N = r >> 15
    c.Z = 1 if r == 0 else 0
    return r


def sub16(c, a, m, cin=0):
    r = a - m - cin
    c.C = 1 if r < 0 else 0
    r &= 0xFFFF
    c.V = ((a ^ m) & (a ^ r)) >> 15
    c.N = r >> 15
    c.Z = 1 if r == 0 else 0
    return r


def nz8(c, r):
    c.N = r >> 7
    c.Z = 1 if r == 0 else 0
    c.V = 0
    return r


def nz16(c, r):
    c.N = r >> 15
    c.Z = 1 if r == 0 else 0
    c.V = 0
    return r


def _alu(add, sub, nz):
    def cmp_(c, a, m):
        sub(c, a, m)

    def bit_(c, a, m):
        nz(c, a & m)

    return [
        lambda c, a, m: sub(c, a, m),
        lambda c, a, m: add(c, a, m),
        lambda c, a, m: sub(c, a, m, c.C),
        lambda c, a, m: add(c, a, m, c.C),
        lambda c, a, m: nz(c, a ^ m),
        lambda c, a, m: nz(c, m),
        lambda c, a, m: nz(c, a & m),
        lambda c, a, m: nz(c, a | m),
        cmp_,
        bit_,
    ]


ALU8 = _alu(add8, sub8, nz8)
ALU16 = _alu(add16, sub16, nz16)
ALU_NAMES = ["SUB", "ADD", "SBC", "ADC", "EOR", "LD", "AND", "OR", "CMP", "BIT"]


# Read-modify-write family, shared by memory and register forms.
def _rmw():
    def com(c, v, bits):
        r = ~v & ((1 << bits) - 1)
        c.N = r >> (bits - 1); c.Z = 1 if r == 0 else 0; c.V = 0; c.C = 1
        return r

    def dec(c, v, bits):
        r = (v - 1) & ((1 << bits) - 1)
        c.V = 1 if v == 1 << (bits - 1) else 0
        c.N = r >> (bits - 1); c.Z = 1 if r == 0 else 0
        return r

    def neg(c, v, bits):
        r = -v & ((1 << bits) - 1)
        c.V = 1 if v == 1 << (bits - 1) else 0
        c.C = 1 if v else 0
        c.N = r >> (bits - 1); c.Z = 1 if r == 0 else 0
        return r

    def inc(c, v, bits):
        r = (v + 1) & ((1 << bits) - 1)
        c.V = 1 if v == (1 << (bits - 1)) - 1 else 0
        c.N = r >> (bits - 1); c.Z = 1 if r == 0 else 0
        return r

    def asl(c, v, bits):
        c.C = v >> (bits - 1)
        r = (v << 1) & ((1 << bits) - 1)
        c.N = r >> (bits - 1); c.Z = 1 if r == 0 else 0; c.V = c.N ^ c.C
        return r

    def clr(c, v, bits):
        c.N = 0; c.Z = 1; c.V = 0; c.C = 0
        return 0

    def tst(c, v, bits):
        c.N = v >> (bits - 1); c.Z = 1 if v == 0 else 0; c.V = 0; c.C = 0
        return None

    def rol(c, v, bits):
        r = ((v << 1) | c.C) & ((1 << bits) - 1)
        c.C = v >> (bits - 1)
        c.N = r >> (bits - 1); c.Z = 1 if r == 0 else 0; c.V = c.N ^ c.C
        return r

    def asr(c, v, bits):
        c.C = v & 1
        r = (v >> 1) | (v & (1 << (bits - 1)))
        c.N = r >> (bits - 1); c.Z = 1 if r == 0 else 0; c.V = c.N ^ c.C
        return r

    def ror(c, v, bits):
        r = (v >> 1) | (c.C << (bits - 1))
        c.C = v & 1
        c.N = r >> (bits - 1); c.Z = 1 if r == 0 else 0; c.V = c.N ^ c.C
        return r

    def lsr(c, v, bits):
        c.C = v & 1
        r = v >> 1
        c.N = 0; c.Z = 1 if r == 0 else 0; c.V = c.C
        return r

    return {0x0: ("COM", com), 0x1: ("DEC", dec), 0x2: ("NEG", neg),
            0x3: ("INC", inc), 0x4: ("ASL", asl), 0x5: ("CLR", clr),
            0x6: ("TST", tst), 0xC: ("ROL", rol), 0xD: ("ASR", asr),
            0xE: ("ROR", ror), 0xF: ("LSR", lsr)}


RMW = _rmw()

# --------------------------------------------------------------------------
# Effective-address resolvers: f(cpu, operand_address) -> 20-bit address.
# --------------------------------------------------------------------------
IDX = {
    "X": lambda c: (c.xk << 16) | c.ix,
    "Y": lambda c: (c.yk << 16) | c.iy,
    "Z": lambda c: (c.zk << 16) | c.iz,
}


def ea_ind8(r, at=0):
    g = IDX[r]
    return lambda c, p: (g(c) + c.r8(p + at)) & M20


def ea_ind16(r, at=0):
    g = IDX[r]
    return lambda c, p: (g(c) + s16(c.r16(p + at))) & M20


def ea_eoff(r):
    g = IDX[r]
    return lambda c, p: (g(c) + s16(c.e)) & M20


def ea_ext(at=0):
    return lambda c, p: (c.ek << 16) | c.r16(p + at)


class CPU16:
    def __init__(self, bus):
        self.bus = bus
        self.r8 = bus.read8
        self.r16 = bus.read16
        self.w8 = bus.write8
        self.w16 = bus.write16
        self.a = self.b = self.e = 0
        self.ix = self.iy = self.iz = 0
        self.ek = self.xk = self.yk = self.zk = 0
        self.sp = 0            # 20-bit, SK in the top nibble
        self.pc = 0            # 20-bit address of the current instruction
        self.S = self.MV = self.H = self.EV = 0
        self.N = self.Z = self.V = self.C = 0
        self.IP = 7
        self.SM = 0
        self.am = 0            # 36-bit MAC accumulator
        self.stopped = False
        # Set by instructions that rewrite the interrupt mask; the machine
        # then lets one more instruction run before recognising interrupts.
        self.irq_delay = 0
        self.icount = 0
        self.ops = OPS

    # ---- composite registers -------------------------------------------
    def getd(self):
        return (self.a << 8) | self.b

    def setd(self, v):
        self.a = (v >> 8) & 0xFF
        self.b = v & 0xFF

    def get_ccr(self, pk=None):
        if pk is None:
            pk = self.pc >> 16
        return (self.S << 15 | self.MV << 14 | self.H << 13 | self.EV << 12 |
                self.N << 11 | self.Z << 10 | self.V << 9 | self.C << 8 |
                self.IP << 5 | self.SM << 4 | pk)

    def set_ccr_flags(self, v):
        """Load every CCR field except PK."""
        self.S = v >> 15 & 1; self.MV = v >> 14 & 1; self.H = v >> 13 & 1
        self.EV = v >> 12 & 1; self.N = v >> 11 & 1; self.Z = v >> 10 & 1
        self.V = v >> 9 & 1; self.C = v >> 8 & 1
        self.IP = v >> 5 & 7; self.SM = v >> 4 & 1

    def get_k(self):
        return self.ek << 12 | self.xk << 8 | self.yk << 4 | self.zk

    def set_k(self, v):
        self.ek = v >> 12 & 15; self.xk = v >> 8 & 15
        self.yk = v >> 4 & 15; self.zk = v & 15

    # ---- stack ----------------------------------------------------------
    def push16(self, v):
        self.w16(self.sp, v)
        self.sp = (self.sp - 2) & M20

    def pull16(self):
        self.sp = (self.sp + 2) & M20
        return self.r16(self.sp)

    # ---- reset / exceptions --------------------------------------------
    def reset(self):
        w0 = self.r16(0)
        self.zk = w0 >> 8 & 15
        sk = w0 >> 4 & 15
        pk = w0 & 15
        self.pc = pk << 16 | self.r16(2)
        self.sp = sk << 16 | self.r16(4)
        self.iz = self.r16(6)
        self.S = self.MV = self.H = self.EV = 0
        self.IP = 7
        self.SM = 0
        self.ek = self.xk = self.yk = 0
        self.stopped = False

    def exception(self, vector, resume, level=None):
        """Stack PC and CCR, then enter the handler for `vector`.

        `resume` is the address execution continues at after RTI.
        """
        self.push16((resume + 6) & 0xFFFF)
        self.push16(self.get_ccr(resume >> 16))
        if level is not None:
            self.IP = level
        self.pc = self.r16(vector * 2)      # handlers start in bank 0
        self.stopped = False

    # ---- execution ------------------------------------------------------
    def step(self):
        pc = self.pc
        op = self.r8(pc)
        if op == 0x17 or op == 0x27 or op == 0x37:
            op = op << 8 | self.r8(pc + 1)
            ent = self.ops.get(op)
            if ent is None:
                raise IllegalInstruction("%05X: opcode %04X" % (pc, op))
            ent[2](self, pc + 2)
        else:
            ent = self.ops.get(op)
            if ent is None:
                raise IllegalInstruction("%05X: opcode %02X" % (pc, op))
            ent[2](self, pc + 1)
        self.icount += 1

    def state(self):
        return ("PC=%05X A=%02X B=%02X E=%04X IX=%X:%04X IY=%X:%04X IZ=%X:%04X "
                "SP=%05X EK=%X CCR=%04X" % (
                    self.pc, self.a, self.b, self.e, self.xk, self.ix, self.yk,
                    self.iy, self.zk, self.iz, self.sp, self.ek, self.get_ccr()))


# --------------------------------------------------------------------------
# Opcode table: key -> (mnemonic, length, handler). Keys are the opcode byte
# for page 0, or prefix << 8 | opcode for the 17/27/37 pages.
# --------------------------------------------------------------------------
OPS = {}


def op(key, name, n):
    def deco(fn):
        if key in OPS:
            raise AssertionError("duplicate opcode %X" % key)
        OPS[key] = (name, n, fn)
        return fn
    return deco


def define(key, name, n, fn):
    op(key, name, n)(fn)


# ---- 8-bit accumulator families ------------------------------------------
def mk_acc8(reg, k, ea, n):
    if k == 10:
        def h(c, p):
            v = getattr(c, reg)
            c.w8(ea(c, p), v)
            c.N = v >> 7; c.Z = 1 if v == 0 else 0; c.V = 0
            c.pc += n
        return h
    fn = ALU8[k]
    if ea is None:
        def h(c, p):
            r = fn(c, getattr(c, reg), c.r8(p))
            if r is not None:
                setattr(c, reg, r)
            c.pc += n
    else:
        def h(c, p):
            r = fn(c, getattr(c, reg), c.r8(ea(c, p)))
            if r is not None:
                setattr(c, reg, r)
            c.pc += n
    return h


def acc8_name(k, r):
    if k == 10:
        return "STA" + r
    base = ALU_NAMES[k]
    return base + (r if base in ("SUB", "ADD", "SBC", "ADC", "EOR", "AND",
                                 "CMP", "BIT") else "A" + r)


for reg, R, row in (("a", "A", 0x40), ("b", "B", 0xC0)):
    for k in range(11):
        nm = acc8_name(k, R)
        for i, X in enumerate("XYZ"):
            define(row + 16 * i + k, nm, 2, mk_acc8(reg, k, ea_ind8(X), 2))
            define(0x1700 | (row + 16 * i + k), nm, 4, mk_acc8(reg, k, ea_ind16(X), 4))
            define(0x2700 | (row + 16 * i + k), nm, 2, mk_acc8(reg, k, ea_eoff(X), 2))
        if k != 10:
            define(row + 0x30 + k, nm, 2, mk_acc8(reg, k, None, 2))
        define(0x1700 | (row + 0x30 + k), nm, 4, mk_acc8(reg, k, ea_ext(), 4))


# ---- 16-bit accumulator families (D and E) --------------------------------
def mk_acc16(get, put, k, ea, n, imm=None):
    if k == 10:
        def h(c, p):
            v = get(c)
            c.w16(ea(c, p), v)
            c.N = v >> 15; c.Z = 1 if v == 0 else 0; c.V = 0
            c.pc += n
        return h
    fn = ALU16[k]
    if imm == 16:
        def h(c, p):
            r = fn(c, get(c), c.r16(p))
            if r is not None:
                put(c, r)
            c.pc += n
    elif imm == 8:
        def h(c, p):
            r = fn(c, get(c), s8(c.r8(p)) & 0xFFFF)
            if r is not None:
                put(c, r)
            c.pc += n
    else:
        def h(c, p):
            r = fn(c, get(c), c.r16(ea(c, p)))
            if r is not None:
                put(c, r)
            c.pc += n
    return h


def _sete(c, v):
    c.e = v


GETD, SETD = CPU16.getd, CPU16.setd
GETE = lambda c: c.e


def acc16_name(k, r):
    if k == 10:
        return "ST" + r
    base = ALU_NAMES[k]
    return {"CMP": "CP"}.get(base, base) + r


for k in list(range(9)) + [10]:
    nm = acc16_name(k, "D")
    for i, X in enumerate("XYZ"):
        define(0x80 + 16 * i + k, nm, 2, mk_acc16(GETD, SETD, k, ea_ind8(X), 2))
        define(0x3700 | (0xC0 + 16 * i + k), nm, 4, mk_acc16(GETD, SETD, k, ea_ind16(X), 4))
        define(0x2700 | (0x80 + 16 * i + k), nm, 2, mk_acc16(GETD, SETD, k, ea_eoff(X), 2))
    if k != 10:
        define(0x3700 | (0xB0 + k), nm, 4, mk_acc16(GETD, SETD, k, None, 4, imm=16))
    define(0x3700 | (0xF0 + k), nm, 4, mk_acc16(GETD, SETD, k, ea_ext(), 4))

    nm = acc16_name(k, "E")
    for i, X in enumerate("XYZ"):
        define(0x3700 | (0x40 + 16 * i + k), nm, 4, mk_acc16(GETE, _sete, k, ea_ind16(X), 4))
    if k != 10:
        define(0x3700 | (0x30 + k), nm, 4, mk_acc16(GETE, _sete, k, None, 4, imm=16))
    define(0x3700 | (0x70 + k), nm, 4, mk_acc16(GETE, _sete, k, ea_ext(), 4))

define(0x7C, "ADDE", 2, mk_acc16(GETE, _sete, 1, None, 2, imm=8))
define(0xFC, "ADDD", 2, mk_acc16(GETD, SETD, 1, None, 2, imm=8))


# ---- index register / stack pointer load, store, compare ------------------
def _idx_get(r):
    if r == "S":
        return lambda c: c.sp & 0xFFFF
    a = "i" + r.lower()
    return lambda c: getattr(c, a)


def _idx_put(r):
    if r == "S":
        def f(c, v):
            c.sp = (c.sp & 0xF0000) | v
        return f
    a = "i" + r.lower()
    return lambda c, v: setattr(c, a, v)


for j, R in enumerate("XYZS"):
    g, s = _idx_get(R), _idx_put(R)
    for i, X in enumerate("XYZ"):
        define(0x4C + 16 * i + j, "CP" + R, 2, mk_acc16(g, s, 8, ea_ind8(X), 2))
        define(0x1700 | (0x4C + 16 * i + j), "CP" + R, 4, mk_acc16(g, s, 8, ea_ind16(X), 4))
        define(0xCC + 16 * i + j, "LD" + R, 2, mk_acc16(g, s, 5, ea_ind8(X), 2))
        define(0x1700 | (0xCC + 16 * i + j), "LD" + R, 4, mk_acc16(g, s, 5, ea_ind16(X), 4))
        define(0x8C + 16 * i + j, "ST" + R, 2, mk_acc16(g, s, 10, ea_ind8(X), 2))
        define(0x1700 | (0x8C + 16 * i + j), "ST" + R, 4, mk_acc16(g, s, 10, ea_ind16(X), 4))
    define(0x1700 | (0x7C + j), "CP" + R, 4, mk_acc16(g, s, 8, ea_ext(), 4))
    define(0x3700 | (0x7C + j), "CP" + R, 4, mk_acc16(g, s, 8, None, 4, imm=16))
    define(0x1700 | (0xFC + j), "LD" + R, 4, mk_acc16(g, s, 5, ea_ext(), 4))
    define(0x3700 | (0xBC + j), "LD" + R, 4, mk_acc16(g, s, 5, None, 4, imm=16))
    define(0x1700 | (0xBC + j), "ST" + R, 4, mk_acc16(g, s, 10, ea_ext(), 4))


# ---- read-modify-write on memory ------------------------------------------
def mk_rmw8(fn, ea, n):
    def h(c, p):
        addr = ea(c, p)
        r = fn(c, c.r8(addr), 8)
        if r is not None:
            c.w8(addr, r)
        c.pc += n
    return h


def mk_clr(ea, n, word):
    # CLR writes without a preceding read, which matters for I/O registers.
    def h(c, p):
        (c.w16 if word else c.w8)(ea(c, p), 0)
        c.N = 0; c.Z = 1; c.V = 0; c.C = 0
        c.pc += n
    return h


def mk_rmw16(fn, ea, n):
    def h(c, p):
        addr = ea(c, p)
        r = fn(c, c.r16(addr), 16)
        if r is not None:
            c.w16(addr, r)
        c.pc += n
    return h


for lo, (nm, fn) in RMW.items():
    for i, X in enumerate("XYZ"):
        k = 16 * i + lo
        if nm == "CLR":
            define(k, nm, 2, mk_clr(ea_ind8(X), 2, False))
            define(0x1700 | k, nm, 4, mk_clr(ea_ind16(X), 4, False))
            define(0x2700 | k, nm + "W", 4, mk_clr(ea_ind16(X), 4, True))
        else:
            define(k, nm, 2, mk_rmw8(fn, ea_ind8(X), 2))
            define(0x1700 | k, nm, 4, mk_rmw8(fn, ea_ind16(X), 4))
            define(0x2700 | k, nm + "W", 4, mk_rmw16(fn, ea_ind16(X), 4))
    if nm == "CLR":
        define(0x1730 | lo, nm, 4, mk_clr(ea_ext(), 4, False))
        define(0x2730 | lo, nm + "W", 4, mk_clr(ea_ext(), 4, True))
    else:
        define(0x1730 | lo, nm, 4, mk_rmw8(fn, ea_ext(), 4))
        define(0x2730 | lo, nm + "W", 4, mk_rmw16(fn, ea_ext(), 4))


# ---- read-modify-write on registers ----------------------------------------
def mk_rmw_reg(fn, get, put, bits):
    def h(c, p):
        r = fn(c, get(c), bits)
        if r is not None:
            put(c, r)
        c.pc += 2
    return h


def _seta(c, v):
    c.a = v


def _setb(c, v):
    c.b = v


for lo, (nm, fn) in RMW.items():
    define(0x3700 | lo, nm + "A", 2, mk_rmw_reg(fn, lambda c: c.a, _seta, 8))
    define(0x3710 | lo, nm + "B", 2, mk_rmw_reg(fn, lambda c: c.b, _setb, 8))
    if nm not in ("DEC", "INC"):
        define(0x2770 | lo, nm + "E", 2, mk_rmw_reg(fn, GETE, _sete, 16))
        define(0x27F0 | lo, nm + "D", 2, mk_rmw_reg(fn, GETD, SETD, 16))


# ---- bit set / clear and bit-test branches ---------------------------------
def mk_bit(set_, ea, n):
    def h(c, p):
        mask = c.r8(p)
        addr = ea(c, p)
        m = c.r8(addr)
        m = (m | mask) if set_ else (m & ~mask & 0xFF)
        c.w8(addr, m)
        c.N = m >> 7; c.Z = 1 if m == 0 else 0; c.V = 0
        c.pc += n
    return h


def mk_brbit(set_, ea, n, rel16):
    def h(c, p):
        mask = c.r8(p)
        m = c.r8(ea(c, p))
        taken = ((~m & mask) == 0) if set_ else ((m & mask) == 0)
        if taken:
            rel = s16(c.r16(p + 3)) if rel16 else s8(c.r8(p + 2))
            c.pc = (c.pc + 6 + rel) & M20
        else:
            c.pc += n
    return h


for i, X in enumerate("XYZ"):
    define(0x08 + 16 * i, "BCLR", 4, mk_bit(False, ea_ind16(X, 1), 4))
    define(0x09 + 16 * i, "BSET", 4, mk_bit(True, ea_ind16(X, 1), 4))
    define(0x1708 + 16 * i, "BCLR", 4, mk_bit(False, ea_ind8(X, 1), 4))
    define(0x1709 + 16 * i, "BSET", 4, mk_bit(True, ea_ind8(X, 1), 4))
    define(0x0A + 16 * i, "BRCLR", 6, mk_brbit(False, ea_ind16(X, 1), 6, True))
    define(0x0B + 16 * i, "BRSET", 6, mk_brbit(True, ea_ind16(X, 1), 6, True))
    define(0xCB + 16 * i, "BRCLR", 4, mk_brbit(False, ea_ind8(X, 1), 4, False))
    define(0x8B + 16 * i, "BRSET", 4, mk_brbit(True, ea_ind8(X, 1), 4, False))
define(0x38, "BCLR", 4, mk_bit(False, ea_ext(1), 4))
define(0x39, "BSET", 4, mk_bit(True, ea_ext(1), 4))
define(0x3A, "BRCLR", 6, mk_brbit(False, ea_ext(1), 6, True))
define(0x3B, "BRSET", 6, mk_brbit(True, ea_ext(1), 6, True))


# ---- branches ---------------------------------------------------------------
CONDS = [
    ("RA", lambda c: True), ("RN", lambda c: False),
    ("HI", lambda c: not (c.C | c.Z)), ("LS", lambda c: c.C | c.Z),
    ("CC", lambda c: not c.C), ("CS", lambda c: c.C),
    ("NE", lambda c: not c.Z), ("EQ", lambda c: c.Z),
    ("VC", lambda c: not c.V), ("VS", lambda c: c.V),
    ("PL", lambda c: not c.N), ("MI", lambda c: c.N),
    ("GE", lambda c: not (c.N ^ c.V)), ("LT", lambda c: c.N ^ c.V),
    ("GT", lambda c: not (c.Z | (c.N ^ c.V))), ("LE", lambda c: c.Z | (c.N ^ c.V)),
]


def mk_branch(cond, long_):
    if long_:
        def h(c, p):
            if cond(c):
                c.pc = (c.pc + 6 + s16(c.r16(p))) & M20
            else:
                c.pc += 4
    else:
        def h(c, p):
            if cond(c):
                c.pc = (c.pc + 6 + s8(c.r8(p))) & M20
            else:
                c.pc += 2
    return h


for i, (nm, cond) in enumerate(CONDS):
    define(0xB0 + i, "B" + nm, 2, mk_branch(cond, False))
    define(0x3780 + i, "LB" + nm, 4, mk_branch(cond, True))


# ---- jumps, calls, returns ----------------------------------------------------
def _call(c, target, next_):
    c.push16((next_ + 2) & 0xFFFF)
    c.push16(c.get_ccr(next_ >> 16))
    c.pc = target


@op(0x7A, "JMP", 4)
def _(c, p):
    c.pc = (c.r8(p) & 15) << 16 | c.r16(p + 1)


@op(0xFA, "JSR", 4)
def _(c, p):
    _call(c, (c.r8(p) & 15) << 16 | c.r16(p + 1), c.pc + 4)


def _disp20(c, p):
    v = (c.r8(p) & 15) << 16 | c.r16(p + 1)
    return v - 0x100000 if v & 0x80000 else v


for i, X in enumerate("XYZ"):
    a = "i" + X.lower()

    def jmp(c, p, a=a):
        c.pc = (getattr(c, a) + _disp20(c, p)) & M20

    def jsr(c, p, a=a):
        _call(c, (getattr(c, a) + _disp20(c, p)) & M20, c.pc + 4)

    define(0x4B + 16 * i, "JMP", 4, jmp)
    define(0x89 + 16 * i, "JSR", 4, jsr)


@op(0x36, "BSR", 2)
def _(c, p):
    _call(c, (c.pc + 6 + s8(c.r8(p))) & M20, c.pc + 2)


@op(0x27F9, "LBSR", 4)
def _(c, p):
    _call(c, (c.pc + 6 + s16(c.r16(p))) & M20, c.pc + 4)


@op(0x27F7, "RTS", 2)
def _(c, p):
    pk = c.pull16() & 15
    c.pc = (pk << 16 | c.pull16()) - 2 & M20


@op(0x2777, "RTI", 2)
def _(c, p):
    ccr = c.pull16()
    c.set_ccr_flags(ccr)
    c.pc = ((ccr & 15) << 16 | c.pull16()) - 6 & M20


# ---- stack ----------------------------------------------------------------------
# Byte pushes move SP by one: CPU16RM gives PSHA as SP+1, store, SP-2 and
# PULA as SP+2, load, SP-1. (The Ghidra model moves SP by two here; the ROM's
# argument frames at 0292xx only line up with one.)
@op(0x3708, "PSHA", 2)
def _(c, p):
    c.w8((c.sp + 1) & M20, c.a); c.sp = (c.sp - 1) & M20; c.pc += 2


@op(0x3718, "PSHB", 2)
def _(c, p):
    c.w8((c.sp + 1) & M20, c.b); c.sp = (c.sp - 1) & M20; c.pc += 2


@op(0x3709, "PULA", 2)
def _(c, p):
    c.sp = (c.sp + 1) & M20; c.a = c.r8((c.sp + 1) & M20); c.pc += 2


@op(0x3719, "PULB", 2)
def _(c, p):
    c.sp = (c.sp + 1) & M20; c.b = c.r8((c.sp + 1) & M20); c.pc += 2


@op(0x34, "PSHM", 2)
def _(c, p):
    m = c.r8(p)
    if m & 0x01: c.push16(c.getd())
    if m & 0x02: c.push16(c.e)
    if m & 0x04: c.push16(c.ix)
    if m & 0x08: c.push16(c.iy)
    if m & 0x10: c.push16(c.iz)
    if m & 0x20: c.push16(c.get_k())
    if m & 0x40: c.push16(c.get_ccr())
    c.pc += 2


@op(0x35, "PULM", 2)
def _(c, p):
    m = c.r8(p)
    if m & 0x01:
        c.set_ccr_flags(c.pull16())             # PK is left alone
        c.irq_delay = 1
    if m & 0x02: c.set_k(c.pull16())
    if m & 0x04: c.iz = c.pull16()
    if m & 0x08: c.iy = c.pull16()
    if m & 0x10: c.ix = c.pull16()
    if m & 0x20: c.e = c.pull16()
    if m & 0x40: c.setd(c.pull16())
    c.pc += 2


# ---- inherent ----------------------------------------------------------------------
def inh(key, name):
    def deco(fn):
        def h(c, p):
            fn(c)
            c.pc += 2
        define(key, name, 2, h)
        return fn
    return deco


@inh(0x274C, "NOP")
def _(c):
    pass


@inh(0x3707, "TBA")
def _(c):
    c.a = nz8(c, c.b)


@inh(0x3717, "TAB")
def _(c):
    c.b = nz8(c, c.a)


@inh(0x371A, "XGAB")
def _(c):
    c.a, c.b = c.b, c.a


@inh(0x370A, "SBA")
def _(c):
    c.a = sub8(c, c.a, c.b)


@inh(0x370B, "ABA")
def _(c):
    c.a = add8(c, c.a, c.b)


@inh(0x371B, "CBA")
def _(c):
    sub8(c, c.a, c.b)


@inh(0x2778, "ADE")
def _(c):
    c.e = add16(c, c.e, c.getd())


@inh(0x2779, "SDE")
def _(c):
    c.e = sub16(c, c.e, c.getd())


@inh(0x277A, "XGDE")
def _(c):
    d = c.getd(); c.setd(c.e); c.e = d


@inh(0x277B, "TDE")
def _(c):
    c.e = nz16(c, c.getd())


@inh(0x27FB, "TED")
def _(c):
    c.setd(nz16(c, c.e))


@inh(0x27F8, "SXT")
def _(c):
    c.a = 0xFF if c.b & 0x80 else 0
    c.N = c.a & 1
    c.Z = 1 if (c.a | c.b) == 0 else 0


@inh(0x27FA, "TBEK")
def _(c):
    c.ek = c.b & 15


@inh(0x379C, "TBXK")
def _(c):
    c.xk = c.b & 15


@inh(0x379D, "TBYK")
def _(c):
    c.yk = c.b & 15


@inh(0x379E, "TBZK")
def _(c):
    c.zk = c.b & 15


@inh(0x379F, "TBSK")
def _(c):
    c.sp = (c.b & 15) << 16 | (c.sp & 0xFFFF)


@inh(0x27BB, "TEKB")
def _(c):
    c.b = c.ek


@inh(0x37AC, "TXKB")
def _(c):
    c.b = c.xk


@inh(0x37AD, "TYKB")
def _(c):
    c.b = c.yk


@inh(0x37AE, "TZKB")
def _(c):
    c.b = c.zk


@inh(0x37AF, "TSKB")
def _(c):
    c.b = c.sp >> 16


@inh(0x37FC, "TPA")
def _(c):
    c.a = c.get_ccr() >> 8


@inh(0x37FD, "TAP")
def _(c):
    c.set_ccr_flags(c.a << 8 | (c.get_ccr() & 0xFF))
    c.irq_delay = 1


def _set20(c, r, v):
    v &= M20
    if r == "X":
        c.xk, c.ix = v >> 16, v & 0xFFFF
    elif r == "Y":
        c.yk, c.iy = v >> 16, v & 0xFFFF
    else:
        c.zk, c.iz = v >> 16, v & 0xFFFF


for i, R in enumerate("XYZ"):
    g = IDX[R]

    def tsr(c, p, R=R):
        _set20(c, R, c.sp + 2); c.pc += 2

    def trs(c, p, g=g):
        c.sp = (g(c) - 2) & M20; c.pc += 2

    def abr(c, p, R=R, g=g):
        _set20(c, R, g(c) + c.b); c.pc += 2

    def adr(c, p, R=R, g=g):
        _set20(c, R, g(c) + s16(c.getd())); c.pc += 2

    def aer(c, p, R=R, g=g):
        _set20(c, R, g(c) + s16(c.e)); c.pc += 2

    def xgd(c, p, a="i" + R.lower()):
        d = c.getd(); c.setd(getattr(c, a)); setattr(c, a, d); c.pc += 2

    def xge(c, p, a="i" + R.lower()):
        e = c.e; c.e = getattr(c, a); setattr(c, a, e); c.pc += 2

    def ai8(c, p, R=R, g=g, a="i" + R.lower()):
        _set20(c, R, g(c) + s8(c.r8(p))); c.Z = 1 if getattr(c, a) == 0 else 0; c.pc += 2

    def ai16(c, p, R=R, g=g, a="i" + R.lower()):
        _set20(c, R, g(c) + s16(c.r16(p))); c.Z = 1 if getattr(c, a) == 0 else 0; c.pc += 4

    define(0x274F + 16 * i, "TS" + R, 2, tsr)
    define(0x374E + 16 * i, "T" + R + "S", 2, trs)
    define(0x374F + 16 * i, "AB" + R, 2, abr)
    define(0x37CD + 16 * i, "AD" + R, 2, adr)
    define(0x374D + 16 * i, "AE" + R, 2, aer)
    define(0x37CC + 16 * i, "XGD" + R, 2, xgd)
    define(0x374C + 16 * i, "XGE" + R, 2, xge)
    define(0x3C + i, "AI" + R, 2, ai8)
    define(0x373C + i, "AI" + R, 4, ai16)


def _xfer(dst, src):
    gs = IDX[src]

    def h(c, p):
        _set20(c, dst, gs(c)); c.pc += 2
    return h


define(0x275C, "TXY", 2, _xfer("Y", "X"))
define(0x276C, "TXZ", 2, _xfer("Z", "X"))
define(0x274D, "TYX", 2, _xfer("X", "Y"))
define(0x276D, "TYZ", 2, _xfer("Z", "Y"))
define(0x274E, "TZX", 2, _xfer("X", "Z"))
define(0x275E, "TZY", 2, _xfer("Y", "Z"))


@op(0x3F, "AIS", 2)
def _(c, p):
    c.sp = (c.sp + s8(c.r8(p))) & M20; c.pc += 2


@op(0x373F, "AIS", 4)
def _(c, p):
    c.sp = (c.sp + s16(c.r16(p))) & M20; c.pc += 4


@op(0x373A, "ANDP", 4)
def _(c, p):
    c.set_ccr_flags(c.get_ccr() & c.r16(p)); c.pc += 4; c.irq_delay = 1


@op(0x373B, "ORP", 4)
def _(c, p):
    c.set_ccr_flags(c.get_ccr() | c.r16(p)); c.pc += 4; c.irq_delay = 1


@op(0x2771, "LDED", 4)
def _(c, p):
    a = c.ek << 16 | c.r16(p)
    c.e = c.r16(a); c.setd(c.r16((a + 2) & M20)); c.pc += 4


@op(0x2773, "STED", 4)
def _(c, p):
    a = c.ek << 16 | c.r16(p)
    c.w16(a, c.e); c.w16((a + 2) & M20, c.getd()); c.pc += 4


@op(0x37FE, "MOVB", 6)
def _(c, p):
    v = c.r8(c.ek << 16 | c.r16(p))
    c.w8(c.ek << 16 | c.r16(p + 2), nz8(c, v)); c.pc += 6


@op(0x37FF, "MOVW", 6)
def _(c, p):
    v = c.r16(c.ek << 16 | c.r16(p))
    c.w16(c.ek << 16 | c.r16(p + 2), nz16(c, v)); c.pc += 6


# ---- multiply / divide ---------------------------------------------------------------
@inh(0x3724, "MUL")
def _(c):
    d = c.a * c.b
    c.setd(d); c.C = d >> 7 & 1


@inh(0x3725, "EMUL")
def _(c):
    r = c.e * c.getd()
    c.e = r >> 16; c.setd(r & 0xFFFF)
    c.N = r >> 31; c.Z = 1 if r == 0 else 0; c.C = r >> 15 & 1


@inh(0x3726, "EMULS")
def _(c):
    r = (s16(c.e) * s16(c.getd())) & 0xFFFFFFFF
    c.e = r >> 16; c.setd(r & 0xFFFF)
    c.N = r >> 31; c.Z = 1 if r == 0 else 0; c.C = r >> 15 & 1


DIVIDE_BY_ZERO_VECTOR = 8


@op(0x3728, "EDIV", 2)
def _(c, p):
    if c.ix == 0:
        c.exception(DIVIDE_BY_ZERO_VECTOR, c.pc + 2)
        return
    n = c.e << 16 | c.getd()
    q, r = divmod(n, c.ix)
    c.V = 1 if q > 0xFFFF else 0
    c.C = 1 if 2 * r >= c.ix else 0
    c.ix = q & 0xFFFF; c.setd(r)
    c.N = c.ix >> 15; c.Z = 1 if c.ix == 0 else 0
    c.pc += 2


@inh(0x372A, "IDIV")
def _(c):
    c.V = 0
    if c.ix == 0:
        c.C = 1; c.ix = 0xFFFF; c.Z = 0
        return
    c.C = 0
    q, r = divmod(c.getd(), c.ix)
    c.ix = q; c.setd(r); c.Z = 1 if q == 0 else 0


@inh(0x372B, "FDIV")
def _(c):
    d = c.getd()
    c.C = 1 if c.ix == 0 else 0
    if d >= c.ix:
        c.V = 1; c.ix = 0xFFFF; c.Z = 0
        return
    c.V = 0
    q, r = divmod(d << 16, c.ix)
    c.ix = q; c.setd(r); c.Z = 1 if q == 0 else 0


# ---- MAC transfers used by the ROM ------------------------------------------------------
@inh(0x27B2, "TEM")
def _(c):
    c.am = (0xF00000000 if c.e & 0x8000 else 0) | c.e << 16
    c.MV = 0; c.EV = 0


@inh(0x27B5, "TMET")
def _(c):
    if c.SM and (c.EV or c.MV):
        c.e = 0x8000 if c.am >> 35 else 0x7FFF
    else:
        c.e = c.am >> 16 & 0xFFFF
    c.N = c.e >> 15; c.Z = 1 if c.e == 0 else 0


# ---- processor control ---------------------------------------------------------------------
@op(0x27F3, "WAI", 2)
def _(c, p):
    c.pc += 2; c.stopped = True


@op(0x27F1, "LPSTOP", 2)
def _(c, p):
    c.pc += 2; c.stopped = True


@op(0x3720, "SWI", 2)
def _(c, p):
    c.exception(6, c.pc + 2)


@op(0x37A6, "BGND", 2)
def _(c, p):
    raise CpuStopped("BGND at %05X" % c.pc)
