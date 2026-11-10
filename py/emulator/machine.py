"""Board model around the CPU16 core: memory map, MCU peripherals, interrupts.

This is a behavioural model sized for one job: letting the VIP_2A03 main-CPU
firmware boot far enough to run its SCI diagnostic handler. It is not a
cycle-accurate MC68HC16Z model. Everything that is a guess about hardware
outside the dump (co-processor replies, analog inputs, engine position) is
isolated in ExternalWorld so the assumptions stay visible.
"""
import collections

from cpu16 import CPU16, M20

ROM_SIZE = 0x40000
RAM_LO, RAM_HI = 0xF8000, 0xF9000      # refined from observed accesses
IO_LO = 0xFF000

# Timebase. Instruction timing is not modelled; one instruction is charged a
# flat number of system clocks. Only ratios between timers matter here.
# 16.000 MHz is inferred (ENGINE_SIGNAL_SPEC.md section 5): it makes the SCI
# rate exactly 7812.5 baud and the slowest task group exactly one second.
F_SYS = 16_000_000
SYS_PER_REF = 512                      # SYNCR = 7F00: system clock = 512 x reference
CLOCKS_PER_INSN = 8

SCSR, SCDR, SCCR0, SCCR1 = 0xFFC0C, 0xFFC0E, 0xFFC08, 0xFFC0A
QILR, QIVR = 0xFFC04, 0xFFC05
SPCR1, SPCR2, SPSR = 0xFFC1A, 0xFFC1C, 0xFFC1F
QSPI_RX, QSPI_TX, QSPI_CMD = 0xFFD00, 0xFFD20, 0xFFD40
PICR, PITR, SWSR, SYNCR = 0xFFA22, 0xFFA24, 0xFFA27, 0xFFA04
TCNT, TFLG1, TFLG2, TMSK1, TMSK2 = 0xFF90A, 0xFF922, 0xFF923, 0xFF920, 0xFF921
ADCSTAT = 0xFF70E
TIC1, TIC2, GPT_ICR = 0xFF90E, 0xFF910, 0xFF904
INSNS_PER_SECOND = F_SYS // CLOCKS_PER_INSN
# TCNT runs at system clock / 16 and wraps every 65536 counts.
TOF_INSNS = 65536 * 16 // CLOCKS_PER_INSN
SYPCR = 0xFFA21
# Software watchdog: SYPCR = B4 enables it with a 2^15 reference-clock
# timeout, about one second. In instructions at 8 clocks each:
WATCHDOG_INSNS = 32768 * SYS_PER_REF // CLOCKS_PER_INSN
ADCTL0, ADCTL1 = 0xFF70A, 0xFF70C
ADC_RESULTS = range(0xFF710, 0xFF740)   # right-justified, left signed, left unsigned

TDRE, TC, RDRF = 0x0100, 0x0080, 0x0040


class ExternalWorld:
    """Everything the firmware talks to that is not in the flash dump."""

    def qspi_transfer(self, m, entries):
        """entries: list of (queue_index, command_byte, tx_word). Return rx words."""
        return [0] * len(entries)

    def analog_read(self, m, channel):
        """10-bit conversion result for analog input 0..7."""
        return 0

    def io_write(self, m, addr, value):
        """Told about every register write, after it is stored."""

    def tick(self, m, icount):
        """Called when `icount` reaches the value this last returned.

        For things that happen on the board's own schedule, such as sensor
        edges. Return the instruction count to be called at next.
        """
        return 1 << 62

    def io_read(self, m, addr):
        """Return a byte for an input the board drives, or None for the latch."""
        return None

    def ext_read(self, m, addr):
        """Read outside ROM/RAM/registers. None means nothing is mapped there."""
        return None

    def ext_write(self, m, addr, value):
        """Write outside RAM/registers. Return True if a device took it."""
        return False

    def irq_sources(self, m):
        """Extra asserted interrupts as (level, vector, name) tuples."""
        return ()


class Machine:
    def __init__(self, rom, world=None):
        if len(rom) != ROM_SIZE:
            raise ValueError("expected a %d-byte image" % ROM_SIZE)
        self.rom = bytes(rom)
        self.mem = bytearray(1 << 20)
        self.world = world or ExternalWorld()
        self.cpu = CPU16(self)
        self.io_log = collections.OrderedDict()   # (addr, 'r'/'w') -> [count, first_pc, last_value]
        self.unmapped = collections.Counter()
        self.rom_writes = collections.Counter()
        self.log_io = True
        self.irq_level = 0
        self.sci_rx = collections.deque()         # bytes waiting to reach the PCM
        self.sci_tx = []                          # (icount, byte) sent by the PCM
        self.sci_rx_at = 0
        self.sci_tx_done_at = None
        self.sci_tx_byte = None
        self.scsr = TDRE | TC
        self.next_pit = None
        self.next_world = 0
        self.wdog_armed = False                   # 55 written, waiting for AA
        self.wdog_last = 0
        self.next_tof = TOF_INSNS
        self.watchdog_resets = 0
        self.events = 0
        self.trace = None
        self.cov = None                           # optional bytearray(ROM_SIZE): executed addresses
        self.cpu.reset()

    # ---- time -------------------------------------------------------------
    def clocks(self):
        return self.cpu.icount * CLOCKS_PER_INSN

    def sci_byte_insns(self):
        br = ((self.mem[SCCR0] << 8) | self.mem[SCCR0 + 1]) & 0x1FFF
        if br == 0:
            return 1 << 60
        # baud = F_SYS / (32 * br); ten bit times per byte
        return max(1, (320 * br) // CLOCKS_PER_INSN)

    def sci_baud(self):
        br = ((self.mem[SCCR0] << 8) | self.mem[SCCR0 + 1]) & 0x1FFF
        return F_SYS / (32 * br) if br else 0

    # ---- bus --------------------------------------------------------------
    def read8(self, a):
        a &= M20
        if a < ROM_SIZE:
            return self.rom[a]
        if RAM_LO <= a < RAM_HI:
            return self.mem[a]
        if a >= IO_LO:
            return self.io_read8(a)
        v = self.world.ext_read(self, a)
        if v is None:
            self.unmapped[(a & 0xFFF00, "r")] += 1
            return 0xFF
        return v

    def read16(self, a):
        a &= M20
        if a < ROM_SIZE - 1:
            return self.rom[a] << 8 | self.rom[a + 1]
        if RAM_LO <= a < RAM_HI - 1:
            return self.mem[a] << 8 | self.mem[a + 1]
        if a >= IO_LO:
            return self.io_read16(a)
        return self.read8(a) << 8 | self.read8(a + 1)

    def write8(self, a, v):
        a &= M20
        if RAM_LO <= a < RAM_HI:
            self.mem[a] = v
        elif a >= IO_LO:
            self.io_write8(a, v)
        elif a < ROM_SIZE:
            self.rom_writes[a] += 1
        elif not self.world.ext_write(self, a, v):
            self.unmapped[(a & 0xFFF00, "w")] += 1

    def write16(self, a, v):
        a &= M20
        if RAM_LO <= a < RAM_HI - 1:
            self.mem[a] = v >> 8
            self.mem[a + 1] = v & 0xFF
        elif a >= IO_LO:
            self.io_write16(a, v)
        else:
            self.write8(a, v >> 8)
            self.write8(a + 1, v & 0xFF)

    # ---- I/O --------------------------------------------------------------
    def _note(self, a, rw, v):
        if self.log_io:
            e = self.io_log.get((a, rw))
            if e is None:
                self.io_log[(a, rw)] = [1, self.cpu.pc, v]
            else:
                e[0] += 1
                e[2] = v

    def io_read16(self, a):
        if a == SCSR:
            v = self.scsr
        elif a == SCDR:
            v = self._sci_read_data()
        elif a == TCNT:
            v = self.tcnt()
        elif a == ADCSTAT:
            v = 0x8000 | (self.mem[a + 1])       # conversion sequence complete
        elif a == SYNCR:
            v = (self.mem[a] << 8 | self.mem[a + 1]) | 0x0008   # PLL locked
        else:
            return self.io_read8(a) << 8 | self.io_read8(a + 1)
        self._note(a, "r", v)
        return v

    def io_read8(self, a):
        if a == SCSR:
            v = self.scsr >> 8
        elif a == SCSR + 1:
            v = self.scsr & 0xFF
        elif a == SCDR + 1:
            v = self._sci_read_data() & 0xFF
        elif a == SPSR:
            v = self.mem[a]
        elif a == TCNT or a == TCNT + 1:
            t = self.tcnt()
            v = t >> 8 if a == TCNT else t & 0xFF
        elif a == ADCSTAT:
            v = 0x80
        elif a in ADC_RESULTS:
            w = self._adc_result(a & ~1)
            v = w >> 8 if not a & 1 else w & 0xFF
        elif a == SYNCR + 1:
            v = self.mem[a] | 0x08
        else:
            v = self.world.io_read(self, a)
            if v is None:
                v = self.mem[a]
        self._note(a, "r", v)
        return v

    def io_write16(self, a, v):
        if a == SCDR:
            self._note(a, "w", v)
            self._sci_write_data(v & 0xFF)
            return
        self.io_write8(a, v >> 8)
        self.io_write8(a + 1, v & 0xFF)

    def io_write8(self, a, v):
        self._note(a, "w", v)
        if a == SCDR + 1:
            self._sci_write_data(v)
            return
        if a == SCSR or a == SCSR + 1:
            return                                 # status is read-only
        if a == SWSR:
            if v == 0x55:
                self.wdog_armed = True
            elif v == 0xAA and self.wdog_armed:
                self.wdog_armed = False
                self.wdog_last = self.cpu.icount
            return
        if a == TFLG1 or a == TFLG2 or a == SPSR:
            self.mem[a] &= v                       # written value clears flags it has as 0
            self._update_irq()
            return
        self.mem[a] = v
        if a == SPCR1 and v & 0x80:
            self._qspi_run()
        elif a in (PICR, PICR + 1, PITR, PITR + 1):
            self._pit_rearm()
        self.world.io_write(self, a, v)
        self._update_irq()

    # ---- timer ------------------------------------------------------------
    def tcnt(self):
        """Free-running counter: TMSK2 = 02 selects system clock / 16."""
        return (self.clocks() >> 4) & 0xFFFF

    def capture_ic2(self):
        """A rising edge on input capture 2 (the vehicle speed sensor input).

        No interrupt: the periodic interrupt polls the flag (02872E)."""
        t = self.tcnt()
        self.mem[TIC2] = t >> 8
        self.mem[TIC2 + 1] = t & 0xFF
        self.mem[TFLG1] |= 0x02

    def capture_ic1(self):
        """A rising edge on input capture 1 (the crank sensor input)."""
        t = self.tcnt()
        self.mem[TIC1] = t >> 8
        self.mem[TIC1 + 1] = t & 0xFF
        self.mem[TFLG1] |= 0x01
        self._update_irq()

    # ---- ADC --------------------------------------------------------------
    def _adc_result(self, a):
        """Result register word. Conversions are instant and always current."""
        idx = a - ADC_RESULTS.start
        slot, fmt = (idx & 0xF) >> 1, idx >> 4
        mode = self.mem[ADCTL1 + 1]
        ch = mode & 0x0F
        if mode & 0x20:                            # MULT: a group of channels
            ch = (ch & 8) | slot if mode & 0x10 else (ch & 0xC) | (slot & 3)
        if ch < 8:
            v = self.world.analog_read(self, ch) & 0x3FF
        else:
            v = {0xC: 0x3FF, 0xD: 0, 0xE: 0x1FF}.get(ch, 0)   # VRH, VRL, midpoint
        if not self.mem[ADCTL0 + 1] & 0x80:        # 8-bit resolution
            v &= 0x3FC
        if fmt == 0:
            return v if self.mem[ADCTL0 + 1] & 0x80 else v >> 2
        return ((v << 6) ^ (0x8000 if fmt == 1 else 0)) & 0xFFFF

    # ---- SCI --------------------------------------------------------------
    def _sci_read_data(self):
        self.scsr &= ~RDRF
        self._update_irq()
        return self.mem[SCDR] << 8 | self.mem[SCDR + 1]

    def _sci_write_data(self, v):
        self.scsr &= ~(TDRE | TC)
        self.sci_tx_byte = v
        self.sci_tx_done_at = self.cpu.icount + self.sci_byte_insns()
        self._update_irq()

    def sci_send(self, data):
        """Queue bytes from the scan tool to the PCM."""
        self.sci_rx.extend(data)

    def _sci_service(self):
        ic = self.cpu.icount
        if self.sci_tx_done_at is not None and ic >= self.sci_tx_done_at:
            self.sci_tx.append((ic, self.sci_tx_byte))
            self.sci_tx_done_at = None
            self.scsr |= TDRE | TC
            self._update_irq()
        if self.sci_rx and not self.scsr & RDRF and ic >= self.sci_rx_at:
            if self.mem[SCCR1 + 1] & 0x04:         # receiver enabled
                self.mem[SCDR] = 0
                self.mem[SCDR + 1] = self.sci_rx.popleft()
                self.scsr |= RDRF
                self.sci_rx_at = ic + self.sci_byte_insns()
                self._update_irq()

    # ---- QSPI -------------------------------------------------------------
    def _qspi_run(self):
        """Complete the queued transfer at once and raise SPIF."""
        spcr2 = self.mem[SPCR2] << 8 | self.mem[SPCR2 + 1]
        newqp, endqp = spcr2 & 15, spcr2 >> 8 & 15
        entries = []
        i = newqp
        while True:
            cmd = self.mem[QSPI_CMD + i]
            tx = self.mem[QSPI_TX + 2 * i] << 8 | self.mem[QSPI_TX + 2 * i + 1]
            entries.append((i, cmd, tx))
            if i == endqp:
                break
            i = (i + 1) & 15
        rx = self.world.qspi_transfer(self, entries)
        for (i, _, _), w in zip(entries, rx):
            self.mem[QSPI_RX + 2 * i] = w >> 8 & 0xFF
            self.mem[QSPI_RX + 2 * i + 1] = w & 0xFF
        self.mem[SPSR] = (self.mem[SPSR] & 0x70) | 0x80 | endqp
        if not spcr2 & 0x4000:                     # no wraparound: SPE clears
            self.mem[SPCR1] &= 0x7F
        self.qspi_count = getattr(self, "qspi_count", 0) + 1
        self._update_irq()

    # ---- interrupts -------------------------------------------------------
    def _pit_rearm(self):
        pitr = self.mem[PITR] << 8 | self.mem[PITR + 1]
        pitm = pitr & 0xFF
        if pitm == 0:
            self.next_pit = None
            return
        clocks = pitm * 4 * (512 if pitr & 0x100 else 1) * SYS_PER_REF
        self.pit_period = max(1, clocks // CLOCKS_PER_INSN)
        self.next_pit = self.cpu.icount + self.pit_period

    def _sources(self):
        """Yield (level, vector, name) for every asserted interrupt."""
        out = []
        if getattr(self, "pit_pending", False):
            picr = self.mem[PICR] << 8 | self.mem[PICR + 1]
            if picr >> 8 & 7:
                out.append((picr >> 8 & 7, picr & 0xFF, "pit"))
        sccr1 = self.mem[SCCR1 + 1]
        if ((sccr1 & 0x20 and self.scsr & RDRF) or (sccr1 & 0x80 and self.scsr & TDRE)
                or (sccr1 & 0x40 and self.scsr & TC)):
            lvl = self.mem[QILR] & 7
            if lvl:
                out.append((lvl, self.mem[QIVR] & 0xFE, "sci"))
        if self.mem[SPCR2] & 0x80 and self.mem[SPSR] & 0x80:
            lvl = self.mem[QILR] >> 3 & 7
            if lvl:
                out.append((lvl, self.mem[QIVR] | 1, "qspi"))
        if self.mem[TMSK1] & self.mem[TFLG1] & 0x01:
            # GPT interrupts share one level; the vector is the base nibble
            # in ICR plus the source number (1 = input capture 1).
            level = self.mem[GPT_ICR] & 7
            if level:
                out.append((level, (self.mem[GPT_ICR + 1] & 0xF0) | 1, "ic1"))
        out.extend(self.world.irq_sources(self))
        return out

    def _update_irq(self):
        src = self._sources()
        self.irq_level = max(s[0] for s in src) if src else 0

    def _take_irq(self):
        level, vector, name = max(self._sources())
        if name == "pit":
            self.pit_pending = False
        cpu = self.cpu
        cpu.exception(vector, cpu.pc, level)
        self.irq_counts = getattr(self, "irq_counts", collections.Counter())
        self.irq_counts[name] += 1
        self._update_irq()

    def watchdog_reset(self):
        """What the MCU does when the watchdog expires: registers return to
        their reset state and execution restarts from the reset vector. RAM
        keeps its contents, which the firmware relies on (it checksums its
        retained block on the way back up)."""
        self.watchdog_resets += 1
        self.mem[IO_LO:] = bytes(len(self.mem) - IO_LO)
        self.scsr = TDRE | TC
        self.sci_tx_done_at = None
        self.next_pit = None
        self.pit_pending = False
        self.wdog_armed = False
        self.wdog_last = self.cpu.icount
        self.cpu.reset()
        self._update_irq()

    # ---- run loop ---------------------------------------------------------
    def run(self, max_insns, until=None):
        cpu = self.cpu
        step = cpu.step
        end = cpu.icount + max_insns
        trace = self.trace
        cov = self.cov
        while cpu.icount < end:
            if cpu.irq_delay:
                # One-instruction latency after the mask is rewritten. The
                # firmware depends on it: 029346 restores the mask and 029348
                # clears the queue-busy flag that the periodic interrupt tests.
                cpu.irq_delay = 0
            elif self.irq_level > cpu.IP:
                self._take_irq()
            if trace is not None:
                trace.append(cpu.pc)
            if cov is not None and cpu.pc < ROM_SIZE:
                cov[cpu.pc] = 1
            if cpu.stopped:
                cpu.icount += 16                   # idle until something happens
            else:
                step()
            ic = cpu.icount
            if ic >= self.next_tof:
                # Timer overflow flag. The firmware polls it (028646) to
                # extend capture times past 16 bits; slow speed pulses need it.
                self.next_tof += TOF_INSNS
                self.mem[TFLG2] |= 0x80
            if ic >= self.next_world:
                self.next_world = self.world.tick(self, ic)
            if ic - self.wdog_last > WATCHDOG_INSNS and self.mem[SYPCR] & 0x80:
                self.watchdog_reset()
            if self.next_pit is not None and ic >= self.next_pit:
                self.next_pit += self.pit_period
                self.pit_pending = True
                self._update_irq()
            if self.sci_tx_done_at is not None or self.sci_rx:
                self._sci_service()
            if until is not None and until(self):
                return True
        return False
