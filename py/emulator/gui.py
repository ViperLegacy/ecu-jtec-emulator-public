"""Browser front end for the emulator.

  python gui.py            then open http://127.0.0.1:8716/

Standard library only. The emulator runs in one background thread; HTTP
handlers take a lock to read state or issue commands. Bound to localhost.
"""
import argparse
import base64
import bisect
import collections
import csv
import hashlib
import json
import pathlib
import sys
import threading
import time
try:
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
except ImportError:          # the browser build (Pyodide) has no sockets; only api() is used there
    BaseHTTPRequestHandler, ThreadingHTTPServer = object, None
from urllib.parse import parse_qs, unquote, urlparse

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from board import ANALOG_SIGNALS, PIN_NOTES, PORTS, ViperBoard, parse_rules  # noqa: E402
from cpu16 import OPS, CpuStopped, IllegalInstruction  # noqa: E402
from machine import INSNS_PER_SECOND, ROM_SIZE, Machine  # noqa: E402
import emu  # noqa: E402

LISTING = HERE.parent / "recovered" / "instructions.tsv"
MAP_W, MAP_H = 512, 128
STATIC = {"/driver.js": "text/javascript; charset=utf-8", "/driver.css": "text/css; charset=utf-8"}
DATA_FILES = {"gauges": "gauges.json", "faults": "faults.json", "outputs": "outputs.json",
              "tables": "tables_view.json", "sensors": "sensor_curves.json", "scenarios": "scenarios.json"}
EVIDENCE = HERE.parent / "recovered" / "data" / "service_manual_evidence.json"
# Connector cavity -> where its live level comes from (INPUT_MAP.md). The
# last field marks ties that are inferences.
CAVITY_SOURCES = {
    "C3-15": ("analog", "a", 0, ""), "C3-16": ("analog", "a", 1, ""), "C3-23": ("analog", "a", 2, ""),
    "C3-27": ("analog", "b", 3, ""), "C3-26": ("analog", "a", 3, ""), "C3-24": ("analog", "a", 4, ""),
    "C3-25": ("analog", "a", 5, "?"), "C3-29": ("analog", "a", 6, "?"), "C3-14": ("analog", "b", 7, ""),
    "C3-2": ("pin", "F", 0, "?"), "C1-24": ("pin", "F", 7, "?"),
}
# The listing and function names were recovered from this one image.
REFERENCE_SHA256 = "82c2832024feae3d5427df4baddbaedcc80cdd0958e3e33155f4b78e1c21c2c0"
BYTES_PER_PIXEL = ROM_SIZE // (MAP_W * MAP_H)


class Session:
    def __init__(self, rom_path, threaded=True):
        """threaded=False is for the browser build (Pyodide has no threads):
        the host then calls slice() repeatedly instead."""
        self.lock = threading.RLock()
        self.running = True
        self.rate = 0.0
        # Emulation speed as a fraction of real time (the PCM's own clock),
        # or None for as fast as this machine can go. Enforced as a token
        # bucket so a pause or slow frame does not cause a burst afterwards.
        self.sim_rate = None
        self.requests_waiting = 0
        self._tokens, self._tok_t = 0.0, time.time()
        path = pathlib.Path(rom_path)
        self.load(path.read_bytes(), path.name)
        # Start from believable sensor readings rather than every pin at 0 V,
        # which the firmware correctly reports as a row of failed sensors.
        start = self.data("scenarios").get("start")
        if start:
            try:
                self.apply_scenario(start)
            except ValueError:
                pass
        self._rate_t, self._rate_i = time.time(), 0
        if threaded:
            threading.Thread(target=self._loop, daemon=True).start()

    def load(self, data, name):
        """Replace the flash image and restart from its reset vector."""
        if len(data) != ROM_SIZE:
            raise ValueError("a JTEC main-flash dump is %d bytes; this file is %d"
                             % (ROM_SIZE, len(data)))
        with self.lock:
            self.rom = bytes(data)
            self.rom_name = name
            self.sha256 = hashlib.sha256(self.rom).hexdigest()
            self.listing = {}
            if self.sha256 == REFERENCE_SHA256 and LISTING.exists():
                for r in csv.DictReader(open(LISTING, encoding="utf-8"), delimiter="\t"):
                    self.listing[int(r["address"], 16)] = (r["bytes"], r["assembly"], r["function"])
            self.addrs = sorted(self.listing)
            self.class_map = self._class_map()
            self.breakpoints = set()
            self.reset()
            self.running = True

    # ---- static description of the image ---------------------------------
    def _class_map(self):
        """One byte per map pixel: 0 erased fill, 1 data/undecoded, 2 code."""
        kind = bytearray(ROM_SIZE)                      # 0 unknown
        for a, (b, _, _) in self.listing.items():
            n = len(b) // 2
            kind[a:a + n] = b"\x02" * n
        out = bytearray(MAP_W * MAP_H)
        for i in range(len(out)):
            lo = i * BYTES_PER_PIXEL
            chunk = kind[lo:lo + BYTES_PER_PIXEL]
            if 2 in chunk:
                out[i] = 2
            elif self.rom[lo:lo + BYTES_PER_PIXEL] != b"\xFF" * BYTES_PER_PIXEL:
                out[i] = 1
        return bytes(out)

    def identity(self):
        rom = self.rom
        known = self.sha256 == REFERENCE_SHA256
        # 002738: four packed-BCD bytes then a two-letter ASCII revision.
        suffix = rom[0x273C:0x273E]
        cal = rom[0x2738:0x273C].hex().upper() + suffix.decode("ascii", "replace")
        return {
            "file": self.rom_name,
            "reference": known,
            "analog_signals": ANALOG_SIGNALS,
            "pin_notes": PIN_NOTES,
            "size": len(rom),
            "sha256": self.sha256,
            "calibration": cal if known and suffix.isalpha() else "",
            "reset_pc": "%05X" % ((rom[1] & 15) << 16 | rom[2] << 8 | rom[3]),
            "reset_sp": "%X%04X" % (rom[1] >> 4 & 15, rom[4] << 8 | rom[5]),
            "instructions": len(self.listing),
            "map": {"w": MAP_W, "h": MAP_H, "bytes_per_pixel": BYTES_PER_PIXEL,
                    "classes": base64.b64encode(self.class_map).decode()},
        }

    # ---- lifecycle ---------------------------------------------------------
    def reset(self):
        with self.lock:
            old = getattr(self, "board", None)
            self.board = ViperBoard()
            if old is not None:                        # inputs survive a reset
                self.board.analog = list(old.analog)
                if hasattr(old, "rpm") and hasattr(self.board, "rpm"):
                    self.board.rpm = old.rpm
                if hasattr(old, "speed_kmh") and hasattr(self.board, "speed_kmh"):
                    self.board.speed_kmh, self.board.gear = old.speed_kmh, old.gear
                self.board.analog_b = list(old.analog_b)
                self.board.pins = dict(old.pins)
                self.board.rules = list(old.rules)
            self.m = Machine(self.rom, self.board)
            self.m.cov = bytearray(ROM_SIZE)
            self.requests = collections.deque(maxlen=200)
            self.req_serial = 0
            self.pending = collections.deque()         # (request, byte)
            self.awaiting = None                       # (request, deadline)
            self.tx_seen = 0
            self.stop_reason = ""

    def _loop(self):
        while True:
            if not self.slice(40000):
                time.sleep(0.005 if self.running else 0.02)
            else:
                # Step aside only while a page request is waiting for the
                # lock; a fixed sleep after every slice cost about half the
                # speed, and none at all starved the page.
                time.sleep(0.002 if self.requests_waiting else 0)

    def _allowance(self, want):
        """How many instructions the speed limit allows right now."""
        now = time.time()
        if self.sim_rate is None:
            self._tokens, self._tok_t = 0.0, now
            return want
        per_s = self.sim_rate * INSNS_PER_SECOND
        self._tokens = min(self._tokens + (now - self._tok_t) * per_s, max(per_s * 0.1, 64))
        self._tok_t = now
        n = int(min(want, self._tokens))
        if n < min(want, 64):
            return 0
        self._tokens -= n
        return n

    def slice(self, insns=40000):
        """Run one bounded piece of emulation. Returns the instructions run:
        0 when paused or held back by the speed limit."""
        ran = 0
        if self.running:
            n = self._allowance(4000 if (self.pending or self.awaiting) else insns)
            if n:
                with self.lock:
                    before = self.m.cpu.icount
                    self._advance(n)
                    ran = self.m.cpu.icount - before
        now, ic = time.time(), self.m.cpu.icount
        if not self.running:
            self._rate_t, self._rate_i = now, ic
        elif now - self._rate_t >= 0.5:
            self.rate = max(0.0, (ic - self._rate_i) / (now - self._rate_t))
            self._rate_t, self._rate_i = now, ic
        return ran

    def _advance(self, n):
        m = self.m
        try:
            if self.breakpoints:
                bps = self.breakpoints
                if m.run(n, until=lambda m: m.cpu.pc in bps):
                    self.running = False
                    self.stop_reason = "breakpoint at %05X" % m.cpu.pc
            else:
                m.run(n)
        except (IllegalInstruction, CpuStopped) as e:
            self.running = False
            self.stop_reason = str(e)
        self._sci_pump()

    # ---- SCI: send one byte, wait for its echo, send the next ----------------
    def _sci_pump(self):
        m = self.m
        new = m.sci_tx[self.tx_seen:]
        self.tx_seen = len(m.sci_tx)
        if new:
            if not self.requests:
                self._new_request([], "unsolicited")
            self.requests[-1]["rx"].extend(b for _, b in new)
            if self.awaiting:
                self.awaiting = None
        if self.awaiting and m.cpu.icount > self.awaiting[1]:
            req = self.awaiting[0]
            req["status"] = "no echo"
            self.pending = collections.deque(p for p in self.pending if p[0] is not req)
            self.awaiting = None
        if not self.awaiting and self.pending:
            req, b = self.pending.popleft()
            if req is not self.requests[-1]:
                # keep replies attached to the request being sent
                self.requests.remove(req)
                self.requests.append(req)
            m.sci_send([b])
            req["sent"] += 1
            self.awaiting = (req, m.cpu.icount + 60 * m.sci_byte_insns())

    def _new_request(self, data, status=""):
        self.req_serial += 1
        req = {"id": self.req_serial, "tx": list(data), "rx": [], "sent": 0,
               "status": status, "at": self.m.cpu.icount}
        self.requests.append(req)
        return req

    def _hs_kinds(self, data):
        """What each byte's reply will be, following the high-speed protocol.

        In high-speed mode (after 12) most bytes are answered with one RAM
        byte instead of an echo; the console needs to know which is which.
        Returns a list of 'echo', 'none' or a RAM address per byte.
        """
        hs = getattr(self, "_hs", None)
        if hs is None or (not self.pending and not self.awaiting and self.m.sci_baud() < 20000):
            hs = {"on": False, "page": 0, "want_page": False}
        kinds = []
        for b in data:
            if not hs["on"]:
                kinds.append("echo")
                if b == 0x12:
                    hs.update(on=True, page=0, want_page=False)
            elif hs["want_page"]:
                hs["want_page"] = False
                if b < 0x10:
                    hs["page"] = b
                    kinds.append("echo")
                else:
                    hs["on"] = False            # page 10 or above: silent exit
                    kinds.append("none")
            elif b == 0xF6:
                hs["want_page"] = True
                kinds.append("echo")
            elif b == 0xF2:
                hs["page"] = 0
                kinds.append("echo")
            elif b in (0xF4, 0xF5):
                hs["page"] = None               # table read: address not known here
                kinds.append("echo")
            elif b == 0xFE:
                hs["on"] = False
                kinds.append("echo")
            else:
                kinds.append("table" if hs["page"] is None else "%05X" % (0xF8000 + hs["page"] * 0x100 + b))
        self._hs = hs
        return kinds

    def sci_send(self, data):
        with self.lock:
            kinds = self._hs_kinds(data)
            req = self._new_request(data)
            if any(k != "echo" for k in kinds) or 0x12 in data:
                req["kinds"] = kinds
            self.pending.extend((req, b) for b in data)

    # ---- car-level view over the optional data files ---------------------------
    def data(self, name):
        """Parsed data file, reloaded when it changes on disk; {} if absent."""
        if name == "connectors":
            path = EVIDENCE
        else:
            path = HERE / DATA_FILES[name]
        cache = self.__dict__.setdefault("_data", {})
        try:
            mtime = path.stat().st_mtime
        except OSError:
            return {}
        if cache.get(name, (None,))[0] != mtime:
            try:
                obj = json.loads(path.read_text(encoding="utf-8"))
            except ValueError:
                obj = {}
            if name == "connectors":
                obj = {"connectors": obj.get("connectors", [])}
            cache[name] = (mtime, obj)
        return cache[name][1]

    def _peek_n(self, addr, size, signed=False):
        v = 0
        for i in range(size):
            v = v << 8 | self.peek(addr + i)
        if signed and v >> (8 * size - 1):
            v -= 1 << (8 * size)
        return v

    def apply_scenario(self, scenario_id):
        """Set sensor inputs from a scenario, in the sensors' own units, then key-cycle.

        The restart matters: once the firmware has logged a sensor fault it
        substitutes a default for that sensor until the next start.
        """
        sc = next((x for x in self.data("scenarios").get("scenarios", []) if x.get("id") == scenario_id), None)
        if sc is None:
            raise ValueError("no such scenario")
        curves = {x["id"]: x for x in self.data("sensors").get("sensors", [])}
        with self.lock:
            for sid, want in sc.get("sensors", {}).items():
                c = curves.get(sid)
                if not c:
                    continue
                pts = sorted(c["points"], key=lambda p: p[1])
                adc = pts[-1][0]
                for (a0, v0), (a1, v1) in zip(pts, pts[1:]):
                    if want <= v1:
                        adc = a0 if v1 == v0 else a0 + (want - v0) / (v1 - v0) * (a1 - a0)
                        break
                if want <= pts[0][1]:
                    adc = pts[0][0]
                target = self.board.analog_b if c["input"]["set"] == "b" else self.board.analog
                target[c["input"]["ch"]] = max(0, min(1023, int(round(adc))))
            if "ignition" in sc:
                self.board.ignition = bool(sc["ignition"])
            if hasattr(self.board, "rpm"):
                self.board.rpm = int(sc.get("rpm", 0))
            if hasattr(self.board, "speed_kmh"):
                self.board.speed_kmh = float(sc.get("speed_kmh", 0))
                self.board.gear = sc.get("gear", 0) or 0
            self.reset()
            self.stop_reason = ""
            self.running = True

    def car(self):
        out = {"gauges": {}, "outputs": {}, "faults": [], "live": {}, "cavities": {}}
        for g in self.data("gauges").get("gauges", []):
            try:
                raw = self._peek_n(int(g["address"], 16), int(g.get("size", 1)), bool(g.get("signed")))
                out["gauges"][g["id"]] = [raw, raw * float(g.get("scale", 1)) + float(g.get("offset", 0))]
            except (ValueError, KeyError, TypeError):
                continue
        for o in self.data("outputs").get("outputs", []):
            try:
                # Prefer the byte as last sent to the co-processor over the
                # request still waiting to go.
                raw = self.peek(int(o.get("shadow_address") or o["address"], 16)) & int(o["mask"], 16)
                if o.get("kind") == "value":
                    on = raw
                else:
                    on = bool(raw) if o.get("active_high", True) else not raw
            except (ValueError, KeyError, TypeError):
                continue
            out["outputs"][o["id"]] = on
            if o.get("cavity"):
                out["cavities"][o["cavity"]] = str(on) if o.get("kind") == "value" else ("ON" if on else "off")
        f = self.data("faults")
        try:
            base, n = int(f.get("list_address", "F8868"), 16), int(f.get("list_length", 8))
            for i in range(n):
                b = self.peek(base + i)
                if b == 0:
                    break
                out["faults"].append("%02X" % b)
        except (ValueError, TypeError):
            pass
        t = self.data("tables")
        for tab in t.get("tables", []):
            for ax in (tab.get("x"), tab.get("y")):
                if ax and ax.get("live_address"):
                    try:
                        out["live"][ax["live_address"]] = self._peek_n(int(ax["live_address"], 16), int(ax.get("live_size", 1)))
                    except ValueError:
                        pass
        for cav, (kind, a, b, q) in CAVITY_SOURCES.items():
            if kind == "analog":
                v = (self.board.analog_b if a == "b" else self.board.analog)[b]
                out["cavities"][cav] = "%.2f V%s" % (v * 5 / 1023, q)
            else:
                out["cavities"][cav] = ("high" if self.board.pins[a] >> b & 1 else "low") + q
        return out

    # ---- views ---------------------------------------------------------------
    def peek(self, a):
        a &= 0xFFFFF
        return self.rom[a] if a < ROM_SIZE else self.m.mem[a]

    def raw_rows(self, start, n):
        """Decode straight from memory: mnemonic and operand bytes only."""
        out = []
        a = start & 0xFFFFE
        for _ in range(n):
            op = self.peek(a)
            key = op << 8 | self.peek(a + 1) if op in (0x17, 0x27, 0x37) else op
            ent = OPS.get(key)
            size = ent[1] if ent else 2
            raw = "".join("%02X" % self.peek(a + k) for k in range(size))
            head = 4 if key > 0xFF else 2
            text = (ent[0] + " " + raw[head:]).strip() if ent else "?? " + raw
            out.append({"a": "%05X" % a, "b": raw, "t": text, "f": ""})
            a += size
        return out

    def rows(self, start, n):
        """Listing rows beginning at the first known instruction >= start."""
        if not self.listing:
            return self.raw_rows(start, n)
        out = []
        i = bisect.bisect_left(self.addrs, start)
        for a in self.addrs[i:i + n]:
            b, asm, fn = self.listing[a]
            out.append({"a": "%05X" % a, "b": b, "t": asm, "f": fn})
        return out

    def rows_around(self, pc, before, n):
        if not self.listing:
            return self.raw_rows(pc, n)
        i = bisect.bisect_left(self.addrs, pc)
        if i < len(self.addrs) and self.addrs[i] == pc:
            return self.rows(self.addrs[max(0, i - before)], n)
        # PC is not at a listed instruction: show raw bytes there, then the listing.
        op = self.peek(pc)
        key = op << 8 | self.peek(pc + 1) if op in (0x17, 0x27, 0x37) else op
        ent = OPS.get(key)
        n_bytes = ent[1] if ent else 2
        raw = "".join("%02X" % self.peek(pc + k) for k in range(n_bytes))
        row = {"a": "%05X" % pc, "b": raw, "t": (ent[0] if ent else "??") + "  (not in listing)", "f": ""}
        return [row] + self.rows(pc + 1, n - 1)

    def state(self, list_at=None):
        with self.lock:
            m, c = self.m, self.m.cpu
            pc = c.pc
            fn = self.listing.get(pc, ("", "", ""))[2]
            stack = []
            a = c.sp + 2
            for _ in range(12):
                if not 0xF8000 <= a < 0xF8FFF:
                    break
                stack.append(["%05X" % a, "%04X" % (m.mem[a] << 8 | m.mem[a + 1])])
                a += 2
            return {
                "running": self.running, "stop_reason": self.stop_reason,
                "icount": c.icount, "rate": round(self.rate),
                "realtime": round(self.rate / INSNS_PER_SECOND, 3),
                "sim_rate": self.sim_rate,
                "pc": "%05X" % pc, "function": fn,
                "regs": {
                    "A": "%02X" % c.a, "B": "%02X" % c.b, "E": "%04X" % c.e,
                    "IX": "%X:%04X" % (c.xk, c.ix), "IY": "%X:%04X" % (c.yk, c.iy),
                    "IZ": "%X:%04X" % (c.zk, c.iz), "SP": "%05X" % c.sp,
                    "EK": "%X" % c.ek, "CCR": "%04X" % c.get_ccr(),
                },
                "flags": {"S": c.S, "MV": c.MV, "H": c.H, "EV": c.EV, "N": c.N,
                          "Z": c.Z, "V": c.V, "C": c.C},
                "ip": c.IP,
                "irq": dict(getattr(m, "irq_counts", {})),
                "baud": round(m.sci_baud()),
                "ignition": self.board.ignition,
                "qspi": self.board.qspi_count,
                "analog": list(self.board.analog),
                "analog_b": list(self.board.analog_b),
                "ports": {n: self.board.port_state(m, n) for n in PORTS},
                "spi": [["%d" % ic, "%05X" % pc, "%02X" % sel, "%04X" % tx, "%02X" % rx]
                        for ic, pc, sel, tx, rx in self.board.qspi_log[-14:]],
                "spi_rules": getattr(self, "rules_text", ""),
                "spi_rule_errors": getattr(self, "rule_errors", []),
                "rom_writes": sum(m.rom_writes.values()),
                "unmapped": sum(m.unmapped.values()),
                "covered": sum(m.cov),
                "car": self.car(),
                "watchdog_resets": m.watchdog_resets,
                "run_timer": m.mem[0xF8096] << 8 | m.mem[0xF8097],
                "engine_model": hasattr(self.board, "rpm"),
                "rpm_setting": getattr(self.board, "rpm", 0),
                "speed_model": hasattr(self.board, "speed_kmh"),
                "speed_setting": getattr(self.board, "speed_kmh", 0),
                "gear": getattr(self.board, "gear", 0),
                "road_speed": round(self.board.road_speed(), 1) if hasattr(self.board, "road_speed") else 0,
                "breakpoints": ["%05X" % b for b in sorted(self.breakpoints)],
                "stack": stack,
                "listing": self.rows(list_at, 24) if list_at is not None else self.rows_around(pc, 6, 24),
                "sci": [dict(r, tx=bytes(r["tx"]).hex(), rx=bytes(r["rx"]).hex())
                        for r in list(self.requests)[-40:]],
            }

    def coverage(self):
        with self.lock:
            cov = bytes(self.m.cov)
        n = BYTES_PER_PIXEL
        bits = bytearray(MAP_W * MAP_H // 8)
        for i in range(MAP_W * MAP_H):
            if 1 in cov[i * n:i * n + n]:
                bits[i >> 3] |= 1 << (i & 7)
        return base64.b64encode(bits).decode()

    def command(self, body):
        cmd = body.get("cmd")
        with self.lock:
            if cmd == "run":
                self.stop_reason = ""
                if self.breakpoints and self.m.cpu.pc in self.breakpoints:
                    self.m.run(1)                       # step off the breakpoint
                self.running = True
            elif cmd == "pause":
                self.running = False
                self.stop_reason = "paused"
            elif cmd == "step":
                self.running = False
                self.stop_reason = "stepped"
                saved, self.breakpoints = self.breakpoints, set()
                self._advance(int(body.get("n", 1)))
                self.breakpoints = saved
            elif cmd == "reset":
                self.reset()
            elif cmd == "sci":
                self.sci_send(bytes.fromhex(body["hex"]))
            elif cmd == "ignition":
                self.board.ignition = bool(body["on"])
            elif cmd == "analog":
                v = max(0, min(1023, int(body["value"])))
                chans = range(8) if body.get("ch") == "all" else [int(body["ch"])]
                for ch in chans:
                    if body.get("set", "a") in ("a", "both"):
                        self.board.analog[ch] = v
                    if body.get("set", "a") in ("b", "both"):
                        self.board.analog_b[ch] = v
            elif cmd == "pin":
                name, bit = body["port"], 1 << int(body["bit"])
                if name not in self.board.pins or bit > 0x80:
                    raise ValueError("no such pin")
                self.board.pins[name] = (self.board.pins[name] & ~bit) | (bit if body["on"] else 0)
                self.m._update_irq()
            elif cmd == "poke":
                a = int(body["addr"], 16)
                data = bytes.fromhex(body["hex"])
                if not data or not 0 <= a <= 0xFFFFF - len(data) + 1:
                    raise ValueError("nothing to write, or address out of range")
                if a < ROM_SIZE:
                    raise ValueError("flash is read-only here; edit the dump file and open it instead")
                for i, b in enumerate(data):
                    self.m.write8(a + i, b)             # registers get their side effects
            elif cmd == "spi_rules":
                rules, errors = parse_rules(body.get("text", ""))
                self.rules_text, self.rule_errors = body.get("text", ""), errors
                if not errors:
                    self.board.rules = rules
            elif cmd == "breakpoint":
                a = int(body["addr"], 16)
                self.breakpoints.symmetric_difference_update({a})
            elif cmd == "rpm":
                if not hasattr(self.board, "rpm"):
                    raise ValueError("no engine model in this build")
                self.board.rpm = max(0, min(8000, int(body["value"])))
            elif cmd == "speed":
                if not hasattr(self.board, "speed_kmh"):
                    raise ValueError("no vehicle speed model in this build")
                self.board.speed_kmh = max(0.0, min(320.0, float(body["value"])))
            elif cmd == "gear":
                g = body.get("value", 0)
                if g not in (0, 1, 2, 3, 4, 5, 6, "R"):
                    raise ValueError("gear must be 0 (speed set directly), 1-6 or R")
                if g and not self.board.gear:
                    self.board.speed_kmh = 0.0
                self.board.gear = g
            elif cmd == "skip_warmup":
                # Several monitors (e.g. the O2 shorted-to-voltage checks at
                # 01BCAC) wait until the run timer's high byte reaches ROM 6F0E.
                need = self.rom[0x6F0E]
                if self.m.mem[0xF8096] < need:
                    self.m.mem[0xF8096] = need
                    self.m.mem[0xF8097] = 0
            elif cmd == "scenario":
                self.apply_scenario(body["id"])
            elif cmd == "erase_codes":
                # The handler wants the request repeated before it acts.
                self.sci_send(b"\x17" * 32)
            elif cmd == "sim_rate":
                v = body.get("value")
                if v is not None:
                    v = float(v)
                    if not 0.001 <= v <= 1.0:
                        raise ValueError("speed must be between 0.1 % and 100 % of real time, or unlimited")
                self.sim_rate = v
                self._tokens, self._tok_t = 0.0, time.time()
            elif cmd == "clear_coverage":
                self.m.cov = bytearray(ROM_SIZE)
            else:
                raise ValueError("unknown command %r" % cmd)


def api(s, method, path, query="", body=b"", filename="upload.bin"):
    """Answer one request. Returns (status, content_type, bytes).

    Used by the local HTTP server and, in the published site, by the browser
    worker that runs this module under Pyodide.
    """
    def ok(obj, code=200):
        return code, "application/json", obj if isinstance(obj, bytes) else json.dumps(obj).encode()

    q = {k: v[0] for k, v in parse_qs(query).items()}
    s.requests_waiting += 1
    try:
        if method == "GET":
            if path in STATIC:
                return 200, STATIC[path], (HERE / path[1:]).read_bytes()
            if path == "/api/data":
                name = q.get("name", "")
                if name != "connectors" and name not in DATA_FILES:
                    raise ValueError("unknown data set")
                return ok(s.data(name))
            if path == "/api/identity":
                return ok(s.identity())
            if path == "/api/state":
                return ok(s.state(int(q["list"], 16) if q.get("list") else None))
            if path == "/api/mem":
                a = int(q.get("addr", "0"), 16) & 0xFFFFF
                n = min(int(q.get("len", "256")), 1024)
                with s.lock:
                    data = bytes(s.peek(a + i) for i in range(n))
                return ok({"addr": "%05X" % a, "hex": data.hex()})
            if path == "/api/commands":
                f = HERE / "sci_commands.json"
                return ok(f.read_bytes() if f.exists() else b'{"session": [], "commands": []}')
            if path == "/api/coverage":
                return ok({"bits": s.coverage()})
            return ok({"error": "not found"}, 404)
        if method == "POST":
            if path == "/api/load":
                if len(body) != ROM_SIZE:
                    raise ValueError("a JTEC main-flash dump is %d bytes; this file is %d" % (ROM_SIZE, len(body)))
                name = pathlib.PurePath(unquote(filename)).name
                s.load(bytes(body), name[:80] or "upload.bin")
                return ok({"ok": True})
            if path == "/api/cmd":
                s.command(json.loads(bytes(body) or b"{}"))
                return ok({"ok": True})
        return ok({"error": "not found"}, 404)
    except (ValueError, KeyError) as e:
        return ok({"error": str(e)}, 400)
    finally:
        s.requests_waiting -= 1


class Handler(BaseHTTPRequestHandler):
    session = None

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            self._send(200, (HERE / "gui.html").read_bytes(), "text/html; charset=utf-8")
            return
        code, ctype, body = api(self.session, "GET", u.path, u.query)
        self._send(code, body, ctype)

    def do_POST(self):
        u = urlparse(self.path)
        # Same-origin only: this server drives a local process.
        origin = self.headers.get("Origin")
        if origin and urlparse(origin).netloc != self.headers.get("Host"):
            self._send(403, b'{"error": "cross-origin request refused"}')
            return
        size = int(self.headers.get("Content-Length", "0"))
        if u.path == "/api/load" and size != ROM_SIZE:
            self.close_connection = True          # do not read an oversized upload
            body = b""
        else:
            body = self.rfile.read(size)
        if u.path == "/api/load" and size != ROM_SIZE:
            code, ctype, out = 400, "application/json", json.dumps(
                {"error": "a JTEC main-flash dump is %d bytes; this file is %d" % (ROM_SIZE, size)}).encode()
        else:
            code, ctype, out = api(self.session, "POST", u.path, u.query, body,
                                   self.headers.get("X-Filename", "upload.bin"))
        self._send(code, out, ctype)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rom", default=str(emu.ROM_PATH))
    ap.add_argument("--port", type=int, default=8716)
    args = ap.parse_args()
    Handler.session = Session(args.rom)
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print("emulator GUI on http://127.0.0.1:%d/" % args.port, flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
