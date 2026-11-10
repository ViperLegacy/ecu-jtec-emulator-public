"""Convenience layer: boot the image and talk to it like a scan tool would."""
import pathlib

from board import ViperBoard
from machine import Machine

ROM_PATH = pathlib.Path(__file__).resolve().parent.parent / "dumps" / "VIP_2A03.ori"
IDLE_PC = 0x28500           # foreground idle branch the firmware parks in


class BootError(RuntimeError):
    pass


def boot(rom_path=ROM_PATH, max_insns=5_000_000, board=None):
    """Return a Machine that has finished initialisation and is idling."""
    m = Machine(pathlib.Path(rom_path).read_bytes(), board or ViperBoard())
    if not m.run(max_insns, until=lambda m: m.cpu.pc == IDLE_PC):
        raise BootError("did not reach %05X: %s" % (IDLE_PC, m.cpu.state()))
    return m


def transact(m, request, echo_timeout=40, quiet=12, max_reply=64):
    """Send `request` one byte at a time, as the real protocol requires.

    Each byte must be echoed before the next is sent. After the last echo,
    collect whatever else the PCM transmits until the line has been quiet for
    `quiet` byte-times. Timeouts are in byte-times at the current baud rate.

    Returns (echoes, reply) as bytes. `echoes` is shorter than `request` if
    the PCM stopped echoing.
    """
    byte = m.sci_byte_insns()
    echoes = bytearray()
    for b in bytes(request):
        mark = len(m.sci_tx)
        m.sci_send([b])
        m.run(echo_timeout * byte, until=lambda m: len(m.sci_tx) > mark)
        if len(m.sci_tx) == mark:
            return bytes(echoes), b""
        echoes.append(m.sci_tx[mark][1])
    reply = bytearray()
    while len(reply) < max_reply:
        mark = len(m.sci_tx)
        m.run(quiet * byte, until=lambda m: len(m.sci_tx) > mark)
        if len(m.sci_tx) == mark:
            break
        reply.append(m.sci_tx[mark][1])
    return bytes(echoes), bytes(reply)


def read_memory(m, address):
    """SCI service 26: read one byte of main-CPU memory. None if refused."""
    req = bytes([0x26, address >> 16 & 0xFF, address >> 8 & 0xFF, address & 0xFF])
    echoes, reply = transact(m, req)
    if echoes != req or len(reply) != 1:
        return None
    return reply[0]
