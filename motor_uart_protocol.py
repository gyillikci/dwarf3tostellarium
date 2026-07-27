"""
DWARF3 motor-MCU UART wire protocol, reverse-engineered from the `magni`
firmware via Ghidra (see memory: dwarf3-tracking-protocol.md, "motor UART
protocol decoded"). This is the protocol `magni` itself speaks to the motor
controller over a physical serial link -- NOT the WsCmd/WebSocket protocol
used by the phone app. It's only reachable if you have direct access to the
motor MCU's UART lines (e.g. after opening the telescope), not over the
network.

Transport: /dev/ttyS1 on the device, 921600 baud, 8N1, raw mode.

Wire frame:
    byte 0:        '>' (0x3E)                          start marker
    byte 1:        <verb, ASCII>
    byte 2..N-4:   <payload, verb-specific>
    byte N-3:      checksum = (verb + all payload bytes) mod 256
    byte N-2..N-1: '\\r' '\\n'                            terminator

Addressing: every request encodes a 2-bit `id` field (request/response
correlation id, NOT a fixed per-axis constant -- the firmware maintains a
registry of live axis objects and demuxes replies to the right one by
matching this field). Axis-affecting verbs additionally multiply the axis
number (0-3; see dwarf3-tracking-protocol.md for what each axis is) by 4 and
OR/add it into the same byte: `cmd_byte = (id & 3) + axis * 4`.

Multi-byte values on the wire are big-endian (MSB first), confirmed from
both the `setFrequency` request encoding and the position-read reply
decoding.

This module implements frame construction/parsing and all fully-decoded
verbs. It has no hardware dependency for build_frame/parse_frame/checksum,
so those are unit-testable without a real serial link; MotorLink (using
pyserial) is the optional part that actually talks to hardware.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

START_BYTE = 0x3E  # '>'
TERMINATOR = b"\r\n"

DEVICE_PATH = "/dev/ttyS1"
BAUD_RATE = 921600

# ---- Verbs -------------------------------------------------------------
# Each entry: verb byte, firmware debug name, whether it's an
# (id&3)+axis*4-style axis command (True) or a raw single-byte/no-payload
# command (False). Verbs marked reply_only or push are never sent by a host.

VERB_READ_ABSOLUTE_POSITION = ord("P")      # readAbsolutePosition
VERB_READ_RELATIVE_POSITION = ord("p")      # readRelativePosition
VERB_SET_ZERO_RELATIVE_POSITION = ord("z")  # setZeroRelativePosition
VERB_READ_ZERO_RELATIVE_POSITION = ord("{")  # readZeroRelativePosition
VERB_READ_PE_SWITCH_STATE = ord("y")        # readPeSwitchState
VERB_CHECK_MOTION_STATE = ord("E")          # checkMotionState
VERB_SET_DIRECTION = ord("F")               # setDirection
VERB_SET_MSTEP = ord("X")                   # setMStep
VERB_SET_FREQUENCY = ord("S")               # setFrequency
VERB_SET_RESET_FREQUENCY = ord("|")         # setResetFrequency
VERB_RESET = ord("R")                       # reset (per-axis home)
VERB_DISABLE_LIMIT = ord("J")               # disableLimit
VERB_ENABLE_LIMIT = ord("M")                # enableLimit
VERB_FORCE_RESET = 0x8D                     # forceReset (deliberately non-ASCII)
VERB_RESET_MCU = ord("w")                   # resetMcu (global, no axis)

# reply-only / unprompted push, never sent by a host -- included for parsing
VERB_REPLY_UNKNOWN_D = ord("D")             # reply-only; send side not located
VERB_LIMIT_TRIGGERED_PUSH = ord("L")        # unprompted "limit switch hit" notification


def checksum(verb: int, payload: bytes) -> int:
    """Additive checksum: sum(verb byte + all payload bytes) mod 256."""
    return (verb + sum(payload)) & 0xFF


def build_frame(verb: int, payload: bytes = b"") -> bytes:
    """Build a complete wire frame for the given verb and payload."""
    body = bytes([verb]) + payload
    return bytes([START_BYTE]) + body + bytes([checksum(verb, payload)]) + TERMINATOR


def cmd_byte(axis_request_id: int, axis: Optional[int] = None) -> int:
    """The common `(id & 3) + axis * 4` payload byte used by most axis
    commands. Pass axis=None for verbs that take a raw id with no axis
    multiplier (e.g. checkMotionState)."""
    b = axis_request_id & 3
    if axis is not None:
        b += (axis & 0xFF) * 4
    return b & 0xFF


class ChecksumError(ValueError):
    pass


def parse_frame(frame: bytes) -> Tuple[int, bytes]:
    """Parse a complete frame (start byte through terminator, inclusive) and
    return (verb, payload). Raises ValueError/ChecksumError on malformed
    input. Does not attempt to demultiplex by id -- see FrameParser for a
    connection-level reader with id-matching."""
    if len(frame) < 5:
        raise ValueError(f"frame too short: {len(frame)} bytes")
    if frame[0] != START_BYTE:
        raise ValueError(f"bad start byte: {frame[0]:#x}")
    if frame[-2:] != TERMINATOR:
        raise ValueError(f"bad terminator: {frame[-2:]!r}")
    verb = frame[1]
    payload = frame[2:-2]
    if not payload:
        raise ValueError("frame has no payload/checksum byte")
    *payload, recv_checksum = payload
    payload = bytes(payload)
    expect = checksum(verb, payload)
    if recv_checksum != expect:
        raise ChecksumError(f"checksum mismatch: got {recv_checksum:#x}, expected {expect:#x}")
    return verb, payload


# ---- Request builders ----------------------------------------------------
# Each returns a ready-to-write frame. `req_id` is the 2-bit correlation id
# the reply will echo back; `axis` is 0-3 (see tracking-protocol.md: 1/2 are
# the mount motors, 0 is the derotator, 3 is a secondary focus motor).

def read_absolute_position(axis: int, req_id: int = 0) -> bytes:
    return build_frame(VERB_READ_ABSOLUTE_POSITION, bytes([cmd_byte(req_id, axis)]))


def read_relative_position(raw_byte: int) -> bytes:
    # firmware passes a raw byte here with no (id&3)+axis*4 encoding
    return build_frame(VERB_READ_RELATIVE_POSITION, bytes([raw_byte & 0xFF]))


def set_zero_relative_position(raw_byte: int) -> bytes:
    return build_frame(VERB_SET_ZERO_RELATIVE_POSITION, bytes([raw_byte & 0xFF]))


def read_zero_relative_position(raw_byte: int) -> bytes:
    return build_frame(VERB_READ_ZERO_RELATIVE_POSITION, bytes([raw_byte & 0xFF]))


def read_pe_switch_state(raw_byte: int) -> bytes:
    return build_frame(VERB_READ_PE_SWITCH_STATE, bytes([raw_byte & 0xFF]))


def check_motion_state(req_id: int = 0) -> bytes:
    # no axis multiplier -- just the low 2 bits
    return build_frame(VERB_CHECK_MOTION_STATE, bytes([req_id & 3]))


def set_direction(axis: int, req_id: int = 0) -> bytes:
    return build_frame(VERB_SET_DIRECTION, bytes([cmd_byte(req_id, axis)]))


def set_mstep(axis: int, req_id: int = 0) -> bytes:
    return build_frame(VERB_SET_MSTEP, bytes([cmd_byte(req_id, axis)]))


def _pack_verb_with_two_u16(verb: int, axis: int, req_id: int, param_a: int, param_b: int) -> bytes:
    payload = bytes([cmd_byte(req_id, axis)]) + param_a.to_bytes(2, "big") + param_b.to_bytes(2, "big")
    return build_frame(verb, payload)


def set_frequency(axis: int, param_a: int, param_b: int, req_id: int = 0) -> bytes:
    """param_a/param_b: two big-endian uint16 fields (exact semantics -- e.g.
    target frequency vs. ramp rate -- weren't individually attributed)."""
    return _pack_verb_with_two_u16(VERB_SET_FREQUENCY, axis, req_id, param_a, param_b)


def set_reset_frequency(axis: int, param_a: int, param_b: int, req_id: int = 0) -> bytes:
    return _pack_verb_with_two_u16(VERB_SET_RESET_FREQUENCY, axis, req_id, param_a, param_b)


def reset(axis: int, req_id: int = 0) -> bytes:
    return build_frame(VERB_RESET, bytes([cmd_byte(req_id, axis)]))


def disable_limit(axis: int, req_id: int = 0) -> bytes:
    return build_frame(VERB_DISABLE_LIMIT, bytes([cmd_byte(req_id, axis)]))


def enable_limit(axis: int, req_id: int = 0) -> bytes:
    return build_frame(VERB_ENABLE_LIMIT, bytes([cmd_byte(req_id, axis)]))


def force_reset(axis: int, req_id: int = 0) -> bytes:
    return build_frame(VERB_FORCE_RESET, bytes([cmd_byte(req_id, axis)]))


def reset_mcu() -> bytes:
    return build_frame(VERB_RESET_MCU, bytes([0x01]))


# ---- Reply decoding --------------------------------------------------------

@dataclass
class PositionReply:
    req_id: int
    value: int
    is_error: bool = False


def decode_position_reply(verb: int, payload: bytes) -> PositionReply:
    """Decodes 'P'/'R' style replies: 1 addressing byte + 4-byte big-endian
    value. 'P' has an error-sentinel case (type nibble == 2 -> 0xfffffffe)."""
    if len(payload) < 5:
        raise ValueError(f"reply payload too short for position decode: {payload!r}")
    addr_byte = payload[0]
    value = int.from_bytes(payload[1:5], "big")
    is_error = value == 0xFFFFFFFE
    return PositionReply(req_id=addr_byte & 3, value=value, is_error=is_error)


def decode_signed_zero_relative_reply(payload: bytes) -> PositionReply:
    """Decodes '{' readZeroRelativePosition replies: addr byte, explicit
    sign byte (1 = negative), then 4-byte big-endian magnitude."""
    if len(payload) < 6:
        raise ValueError(f"reply payload too short: {payload!r}")
    addr_byte = payload[0]
    sign = payload[1]
    magnitude = int.from_bytes(payload[2:6], "big")
    value = -magnitude if sign == 1 else magnitude
    return PositionReply(req_id=addr_byte & 3, value=value)


# ---- Optional hardware link -------------------------------------------------

class MotorLink:
    """Thin synchronous wrapper around a real serial connection. Requires
    pyserial (`pip install pyserial`). Only useful if you have direct access
    to the motor MCU's UART (e.g. after physically opening the telescope) --
    this is not reachable over the network/WsCmd protocol.

    This implements a simple synchronous send-then-read-one-frame model,
    not the firmware's own async multi-object registry -- fine for manual
    probing/testing, not a drop-in replacement for magni's own logic.
    """

    def __init__(self, port: str = DEVICE_PATH, baud: int = BAUD_RATE, timeout: float = 1.0):
        import serial  # local import: optional dependency
        self._serial = serial.Serial(port, baud, timeout=timeout,
                                      bytesize=serial.EIGHTBITS,
                                      parity=serial.PARITY_NONE,
                                      stopbits=serial.STOPBITS_ONE)

    def close(self):
        self._serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def send(self, frame: bytes) -> None:
        self._serial.write(frame)

    def recv_frame(self) -> Tuple[int, bytes]:
        """Reads until a \\r\\n-terminated frame arrives (or timeout)."""
        buf = bytearray()
        while True:
            b = self._serial.read(1)
            if not b:
                raise TimeoutError("no frame received before timeout")
            buf += b
            if len(buf) >= 2 and buf[-2:] == TERMINATOR:
                return parse_frame(bytes(buf))

    def request(self, frame: bytes) -> Tuple[int, bytes]:
        self.send(frame)
        return self.recv_frame()


if __name__ == "__main__":
    # sanity-check frame construction/parsing round-trips without hardware
    f = read_absolute_position(axis=1, req_id=0)
    print("readAbsolutePosition(axis=1):", f)
    verb, payload = parse_frame(f)
    assert verb == VERB_READ_ABSOLUTE_POSITION
    print("round-trip OK, payload:", payload)

    f2 = set_frequency(axis=2, param_a=1000, param_b=50, req_id=0)
    print("setFrequency(axis=2, 1000, 50):", f2)
    verb2, payload2 = parse_frame(f2)
    assert verb2 == VERB_SET_FREQUENCY
    print("round-trip OK, payload:", payload2)

    f3 = reset_mcu()
    print("resetMcu:", f3)
    assert f3 == bytes([0x3E, ord("w"), 0x01, 0x78]) + TERMINATOR, "mismatch vs. firmware-observed frame"
    print("resetMcu matches firmware-observed byte sequence exactly")
