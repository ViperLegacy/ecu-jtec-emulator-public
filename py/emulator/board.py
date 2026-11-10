"""Assumptions about the hardware around the main CPU on the Viper JTEC board.

Nothing here comes from a real board. Each behaviour is the minimum the
firmware needs in order to proceed, and says which firmware check it answers.
Inputs a person can set (analog channels, port pins, scripted serial replies)
live here too; which sensor or switch each one is on the real board is not
established.
"""
from machine import ExternalWorld

PACNT = 0xFF90D
PFPAR = 0xFFA1F
PORTADA = 0xFF707
PORTC = 0xFFA41
MUX_A, MUX_B = 0x10, 0x40    # port C outputs the periodic interrupt alternates

# What each analog input feeds, found by changing one input at a time and
# watching the firmware's own reports (probe_inputs.py, INPUT_MAP.md). Names
# come from the existing research; "unnamed" means the firmware stores the
# value but its meaning is still open.
ANALOG_SIGNALS = {
    "a": ["Intake air temp (C3-15)", "Coolant temp (C3-16)", "Throttle position (C3-23)",
          "Right upstream O2, bank 1 (C3-26)", "Left upstream O2, bank 2 (C3-24)",
          "Right downstream O2 (C3-25?)", "Left downstream O2 (C3-29?)",
          "F8040: unused in this calibration"],
    "b": [None, None, "Speed-control switch", "Manifold pressure (C3-27)",
          "Fuel pressure? (unused here)", "Governor pressure (auto trans, unused)",
          "F804A: unused in this calibration", "Fuel level (C3-14)"],
}

# One line per port bit (index = bit number) for the GUI, from INPUT_MAP.md.
# "?" marks an inference.
PIN_NOTES = {
    "E": ["switch bit 08", "switch bit 10 (idle-air logic)", "switch bit 20", "not used",
          "bus strobe, not I/O", "bus strobe, not I/O",
          "switch bit 02, active low: A/C request? (C1-22/23)",
          "switch bit 04, active low: A/C request? (C1-22/23)"],
    "F": ["ignition / power present (C3-2?)", "diagnostic-link condition: high stops SCI live data",
          "tested by one unnamed routine", "camshaft position level (C3-18): low for crank events 0-9, high for 10-19",
          "sampled, no use found", "serial peripheral ready, active low (IRQ5)",
          "sampled, no use found", "switch bit 80: stop lamp switch? (C1-24)"],
    "GP": ["input capture 1: crankshaft position edges (C3-8)", "input capture 2: vehicle speed pulses (C2-28)",
           "input capture 3, disabled", "output with read-back check", "output with read-back check",
           "output: reverse-lockout request (C2-21)", "output with read-back check",
           "timer output compare 5, load open"],
    "QS": ["QSPI data in", "QSPI data out", "QSPI clock", "QSPI select (code 1E)", "QSPI select (code 1D)",
           "speed-control related input",
           "switch bit 40, active low: configuration / park-neutral style gate",
           "SCI transmit (C1-27)"],
}
SELECT_LATCH = 0x40001
IRQ5_AUTOVECTOR = 0x15

# name -> (data register addresses, direction register)
PORTS = {
    "E": ((0xFFA11, 0xFFA13), 0xFFA15),
    "F": ((0xFFA19, 0xFFA1B), 0xFFA1D),
    "GP": ((0xFF907,), 0xFF906),
    "QS": ((0xFFC15,), 0xFFC17),
}
PORT_AT = {a: name for name, (addrs, _) in PORTS.items() for a in addrs}

IGNITION_BIT = 0x01      # port F bit 0
CAM_BIT = 0x08           # port F bit 3
CRANK_TIMESTAMP = range(0x50004, 0x50008)   # external edge-time latch on CS0
# Instructions between crank events at 1 rpm (ENGINE_SIGNAL_SPEC.md section 5):
# ten slots per revolution in five pairs, 18 degrees inside a pair and 54
# between pairs. The emulator charges 8 clocks per instruction at 16 MHz.
INSNS_18_DEG, INSNS_54_DEG = 6_000_000, 18_000_000
INSNS_PER_SECOND = 2_000_000             # 16 MHz system clock, 8 clocks per instruction

# Drivetrain, from the 1997 service manual: T56 ratios (PDF page 808), 3.07
# final drive (page 118), 335/35 ZR17 rear tyres (page 835).
GEAR_RATIOS = {1: 2.66, 2: 1.78, 3: 1.30, 4: 1.00, 5: 0.74, 6: 0.50, "R": 2.90}
FINAL_DRIVE = 3.07
REAR_TYRE_DIAMETER_M = (17 * 25.4 + 2 * 335 * 0.35) / 1000      # nominal, unloaded
# Speed-sensor pulses per km of road. The real figure is not established;
# this default is replaced by the value build_speed_calibration.py finds by
# asking the firmware (speed_calibration.json), so the speed the user sets
# is the speed the PCM reports.
DEFAULT_PULSES_PER_KM = 5000.0


def road_speed_kmh(rpm, gear):
    """Speed implied by engine rpm in a gear, with no clutch slip."""
    ratio = GEAR_RATIOS.get(gear)
    if not ratio or rpm <= 0:
        return 0.0
    wheel_rpm = rpm / (ratio * FINAL_DRIVE)
    return wheel_rpm * 3.141592653589793 * REAR_TYRE_DIAMETER_M * 60 / 1000
READY_BIT = 0x20         # port F bit 5, low = serial peripheral ready


class ViperBoard(ExternalWorld):
    def __init__(self):
        self.last_tx = {}       # per chip-select: previous byte the CPU sent
        self.qspi_log = []      # (icount, pc, select, tx_word, rx_word)
        self.qspi_count = 0
        self.rules = []         # (select, previous_tx_byte or None, reply_byte)
        self.last_word = {}     # per chip-select: previous full word the CPU sent
        # Co-processor EEPROM bytes the stand-in answers with real values
        # instead of junk, keyed by logical address. The firmware reads one
        # with a 01xx transfer (low address byte) followed by 02yy (page + 1).
        #
        # D88 (ROM 0944): stored state read once per key cycle into F865F by
        # service_stored_state_and_fuel_inhibit_qualification (0250C6). With
        # bit 01 clear the routine sets fuel inhibit F865E.20 about 22 ticks
        # after key-on; with it set it qualifies and fuel stays enabled. What
        # the byte is for has not been identified; 01 is the smallest value
        # that behaves like a healthy car.
        self.eeprom = {0xD88: 0x01}
        self.latch = 0          # last byte written to the external latch at 040001
        self.analog = [0] * 8   # AN0..AN7 as 10-bit conversion results
        # AN2..AN7 each carry a second signal. The periodic interrupt at
        # 028502 alternates port C bits 10 and 40 and files the six results
        # at F8036.. after one and at F8042.. after the other, which is what
        # an external 2:1 analog multiplexer would need. Inferred, not seen
        # on a board.
        self.analog_b = [0] * 8
        self.mux = MUX_A
        # Synthetic engine: an RPM setting drives crank edges on input
        # capture 1 and the cam level on port F bit 3. Perfectly even speed,
        # nominal slot angles, cam edges mid-way through the long gap.
        self.rpm = 0
        self.crank_event = 19          # index of the last event, 0..19 over two revolutions
        self.crank_stamp = 0           # 28-bit edge time the firmware reads at 050004
        self.crank_edges = 0
        self._next_edge = None
        self._cam_at = None
        # Synthetic vehicle speed sensor on input capture 2. Either a speed
        # set directly, or derived from rpm when a gear is selected.
        self.speed_kmh = 0.0
        self.gear = 0                 # 0 = speed set directly; 1-6 or "R" = follow rpm
        self.pulses_per_km = _calibrated_pulses_per_km()
        self.vss_pulses = 0
        self._next_vss = None
        # Level on each pin when it is an input. QS matches what the first
        # version of this model returned, so startup behaves as before.
        self.pins = {"E": 0x00, "F": IGNITION_BIT, "GP": 0x00, "QS": 0x60}

    # Port F bit 0 is the power input tested at 013A66: low for long enough
    # sends the firmware down its save-and-wait-for-power-loss path (it then
    # spins at 013D04). Assumption: high means ignition on.
    @property
    def ignition(self):
        return bool(self.pins["F"] & IGNITION_BIT)

    @ignition.setter
    def ignition(self, on):
        self.pins["F"] = (self.pins["F"] & ~IGNITION_BIT) | (IGNITION_BIT if on else 0)

    def analog_read(self, m, channel):
        if channel >= 2 and self.mux == MUX_B:
            return self.analog_b[channel]
        return self.analog[channel]

    def io_write(self, m, addr, value):
        # The interrupt clears both select bits while it reads the results,
        # so remember the last one that was actually driven.
        if addr == PORTC and value & (MUX_A | MUX_B) in (MUX_A, MUX_B):
            self.mux = value & (MUX_A | MUX_B)

    def port_state(self, m, name):
        addrs, ddr_addr = PORTS[name]
        return {"pins": self.pins[name], "ddr": m.mem[ddr_addr], "latch": m.mem[addrs[0]]}

    def io_read(self, m, addr):
        if addr == PACNT:
            # Startup at 020510 spins until the pulse accumulator is nonzero,
            # i.e. until some external device has produced edges on PAI.
            # Assumption: edges arrive continuously.
            return ((m.cpu.icount >> 6) & 0xFF) or 1
        name = PORT_AT.get(addr)
        if name is not None:
            # Output bits read back the latch; input bits read the pin.
            # 02054E etc. wait for port F bit 5 to read low between serial
            # exchanges, so that pin defaults low.
            ddr = m.mem[PORTS[name][1]]
            return (m.mem[addr] & ddr) | (self.pins[name] & ~ddr & 0xFF)
        if addr == PORTADA:
            # The analog pins double as digital inputs.
            return sum(1 << i for i, v in enumerate(self.analog) if v >= 512)
        return None

    def ext_read(self, m, addr):
        if addr in CRANK_TIMESTAMP:
            # The engine-event interrupt (028F38) reads the time of the edge
            # from here, not from the MCU's own capture register. What the
            # device is has not been established; it behaves as a counter at
            # half the system clock, latched on each crank edge.
            return (self.crank_stamp >> (8 * (CRANK_TIMESTAMP.stop - 1 - addr))) & 0xFF
        return None

    def road_speed(self):
        """Road speed being generated, km/h."""
        if self.gear:
            return road_speed_kmh(self.rpm, self.gear)
        return max(0.0, float(self.speed_kmh))

    def tick(self, m, ic):
        return min(self._tick_vss(m, ic), self._tick_crank(m, ic))

    def _tick_vss(self, m, ic):
        hz = self.road_speed() * self.pulses_per_km / 3600
        if hz < 0.2:                              # below about 0.2 km/h: no pulses
            self._next_vss = None
            return ic + 4000
        if self._next_vss is None:
            self._next_vss = ic
        if ic >= self._next_vss:
            m.capture_ic2()
            self.vss_pulses += 1
            self._next_vss = ic + max(64, int(INSNS_PER_SECOND / hz))
        return self._next_vss

    def _tick_crank(self, m, ic):
        if self.rpm <= 0:
            self._next_edge = self._cam_at = None
            return ic + 4000                       # look again shortly for a speed change
        if self._next_edge is None:
            self._next_edge = ic
        if self._cam_at is not None and ic >= self._cam_at:
            # Cam is low for events 0-9 and high for 10-19.
            if self.crank_event == 9:
                self.pins["F"] |= CAM_BIT
            else:
                self.pins["F"] &= ~CAM_BIT
            self._cam_at = None
        if ic >= self._next_edge:
            self.crank_event = (self.crank_event + 1) % 20
            self.crank_stamp = (ic * 4) & 0x0FFFFFFF
            self.crank_edges += 1
            m.capture_ic1()
            rpm = max(30, self.rpm)
            if self.crank_event % 2 == 0:
                self._next_edge = ic + INSNS_18_DEG // rpm
            else:
                gap = INSNS_54_DEG // rpm
                self._next_edge = ic + gap
                if self.crank_event in (9, 19):
                    self._cam_at = ic + gap // 2
        return min(self._next_edge, self._cam_at or self._next_edge)

    def ext_write(self, m, addr, value):
        # service_qspi_command_queue (02957E) copies the select code of each
        # queued command to 040001 before starting the transfer. CS5 is
        # programmed for a block at 040000, so this is taken to be an
        # external chip-select latch.
        if addr == SELECT_LATCH:
            self.latch = value
            return True
        return False

    def irq_sources(self, m):
        # Vector 15 (level-5 autovector) runs the command-queue service, which
        # first turns PF5 back into a port pin and re-enables it as IRQ5 only
        # while work is pending. Assumption: the device holds the line low
        # (ready) whenever the firmware is listening.
        if m.mem[PFPAR] & READY_BIT and not self.pins["F"] & READY_BIT:
            return ((5, IRQ5_AUTOVECTOR, "irq5"),)
        return ()

    def qspi_transfer(self, m, entries):
        """Serial peripherals selected by the QSPI command byte.

        Startup (020516..0205D6) sends one byte to select 1E and to select 1D,
        then clocks a zero and requires reply + byte == 0 (mod 256). The
        modelled device therefore answers each transfer with the two's
        complement of the byte it received in the previous one. Everything
        else these devices do is unknown.

        A rule (select, previous byte, reply) overrides that stand-in for the
        transfer that follows a matching byte.
        """
        out = []
        for _, cmd, tx in entries:
            sel = (cmd & 0x1F) or self.latch
            prev = self.last_tx.get(sel, 0)
            rx = -prev & 0xFF
            pw = self.last_word.get(sel, 0)
            if tx >> 8 == 0x02 and pw >> 8 == 0x01:
                addr = ((tx & 0xFF) - 1) << 8 | (pw & 0xFF)
                if addr in self.eeprom:
                    rx = self.eeprom[addr]
            self.last_word[sel] = tx
            for r_sel, r_prev, r_reply in self.rules:
                if r_sel == sel and (r_prev is None or r_prev == prev):
                    rx = r_reply
                    break
            self.last_tx[sel] = tx & 0xFF
            self.qspi_log.append((m.cpu.icount, m.cpu.pc, sel, tx, rx))
            if len(self.qspi_log) > 5000:
                del self.qspi_log[:2500]
            self.qspi_count += 1
            out.append(rx)
        return out


def _calibrated_pulses_per_km():
    import json
    import pathlib
    path = pathlib.Path(__file__).resolve().parent / "speed_calibration.json"
    try:
        return float(json.loads(path.read_text(encoding="utf-8"))["pulses_per_km"])
    except (OSError, ValueError, KeyError):
        return DEFAULT_PULSES_PER_KM


def parse_rules(text):
    """Parse lines like '1E 0A -> 55' or '1D * -> 00'. Returns (rules, errors)."""
    rules, errors = [], []
    for n, line in enumerate(text.splitlines(), 1):
        line = line.split("#")[0].strip()
        if not line:
            continue
        try:
            left, reply = line.split("->")
            sel, prev = left.split()
            rule = (int(sel, 16), None if prev == "*" else int(prev, 16), int(reply, 16))
            if not (0 <= rule[0] <= 0xFF and 0 <= rule[2] <= 0xFF and (rule[1] is None or 0 <= rule[1] <= 0xFF)):
                raise ValueError
            rules.append(rule)
        except ValueError:
            errors.append("line %d: expected 'SELECT PREVIOUS -> REPLY' in hex, e.g. 1E 0A -> 55" % n)
    return rules, errors
