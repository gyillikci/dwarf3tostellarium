#!/usr/bin/env python3
"""
factory_test_protocol.py — DWARF 3 factory/QC test module (WsCmd MODULE_TEST,
proto `factoryTest.proto`).

SAFETY: this module is DwarfLab's own manufacturing-floor diagnostics/burn-in
test set, NOT a consumer feature — SN provisioning, MCU reset, fatigue/burn-in
endurance testing for the motors/IR-cut/lights/camera, gyro calibration,
password-encryption state, dark/bias/flat calibration-frame capture, TEC/fan
control. Per dwarf3-tracking-protocol memory: "do NOT casually send commands
in this range" on real hardware. This file is DELIBERATELY encode/decode-only
— there is no `send()`/transport wiring and no one-line "just do it" trigger
methods, unlike dwarflab_controller.py's other modules. If you wire this up to
a live socket, do it deliberately, one command at a time, understanding what
you're about to trigger.

CMD-ID CONFIDENCE — read before using: only `15800` (ReqSetSN) is confirmed by
a zlog debug string cross-check against the real firmware disassembly. The
dispatcher (`magni@FUN_00857ba0`) has CONFIRMED GAPS in its case-label range
(15803-15806 and 15825-15832 have no case at all), which proves the naive
"assign IDs by proto declaration order" trick — safely used for the other new
modules this session (panorama/shooting_schedule) where no such gap evidence
exists — is UNSAFE here: at least one, likely several, of the 38 `Req*`
messages below either sit at a different offset than their declaration order
implies, or aren't wired into this firmware build's switch statement at all
("exist in factoryTest.proto's message list but aren't in this switch" per the
same memory entry). Rather than publish 37 guessed IDs where the one thing we
know for certain is that simple sequential counting is wrong somewhere in the
list, CMD_IDS below only contains the one ID we can actually stand behind.
Every message's ENCODE/DECODE shape is still schema-exact (from the real
recovered protobuf descriptor, not a guess) — only the wire command NUMBER
each one is sent under is the open question. Finding the rest requires
re-running the Ghidra switch-case walk (`ghidra_scripts/`, see
dwarf3-tracking-protocol memory "MODULE_TEST's real base command ID") and
zlog-cross-checking each case individually, the same way 15800 was pinned down.

Run directly for a self-test (encode/decode round-trip on every message, no
device/network needed):

    python3 factory_test_protocol.py
"""
from __future__ import annotations
import struct
from typing import Any

# ── shared low-level primitives (mirrors dwarflab_controller.py's _varint/_field
# and dwarf_protobuf.py's read_varint — duplicated here so this file has zero
# dependency on the rest of the repo and can be dropped in standalone) ──────────
def _varint(v: int) -> bytes:
    v &= 0xFFFFFFFFFFFFFFFF
    b = bytearray()
    while True:
        b.append(v & 0x7F); v >>= 7
        if not v:
            break
    for i in range(len(b) - 1):
        b[i] |= 0x80
    return bytes(b)


def _read_varint(buf: bytes, i: int) -> tuple[int, int]:
    r = s = 0
    while True:
        c = buf[i]; i += 1
        r |= (c & 0x7F) << s
        if not (c & 0x80):
            return r, i
        s += 7


def _field(fn: int, wt: int, val: Any) -> bytes:
    tag = _varint((fn << 3) | wt)
    if wt == 0:
        return tag + _varint(val)
    if wt == 1:
        return tag + struct.pack("<d", val)
    if wt == 2:
        if isinstance(val, str):
            val = val.encode()
        return tag + _varint(len(val)) + val
    if wt == 5:
        return tag + struct.pack("<f", val)
    raise ValueError(f"bad wire type {wt}")


# field spec: (field_no, name, type) — type in {"str","int","bool","double","float"}
Field = tuple[int, str, str]

# ── message schemas, exact from the recovered factoryTest.proto descriptor ──────
SCHEMAS: dict[str, list[Field]] = {
    "ReqSetSN": [(1, "sn", "str")],
    "ReqGetSN": [],
    "ReqGetNTCValue": [],
    "ReqGetEMMCValue": [],
    "ReqSetAUDSTART": [],
    "ReqSetAUDSTOP": [],
    "ReqSetAUDPLAYSTART": [],
    "ReqSetAUDPLAYSTOP": [],
    "ReqSetFatigueTestMode": [(1, "mode", "int")],
    "ReqSetRgb": [],
    "ReqGetELE": [],
    "ReqGetFactoryTestState": [],
    "ReqSetWiFiStartUpMode": [(1, "mode", "int")],
    "ReqFocusMotorBacklashTest": [],
    "ReqCaptureTeleDark": [],
    "ReqCaptureTeleBias": [],
    "ReqCaptureTeleFlat": [],
    "ReqCaptureWideDark": [],
    "ReqCaptureWideBias": [],
    "ReqCaptureWideFlat": [],
    "ReqCaptureGuideFlat": [],
    "ReqCaptureGuideBias": [],
    "ReqResetMotorMcu": [],
    "ReqTestOpenAllCamera": [],
    "ReqFactoryTestSuccess": [],
    "ReqGetRssi": [],
    "ReqGetMCUUART": [],
    "ReqGetFIRMWAREVERISON": [],
    "ReqTestRgbUart": [(1, "count", "int")],
    "ReqPasswordEncryption": [],
    "ReqPasswordEncryptionState": [],
    "ReqRestPasswordEncryption": [],
    "ReqFactoryTecState": [(1, "enable", "bool"), (2, "time", "int")],
    "ReqFactoryFanState": [(1, "enable", "bool"), (2, "speed", "int"), (3, "time", "int")],
    "ReqFactoryHeadTapeState": [(1, "enable", "bool"), (2, "time", "int")],
    "ReqSetGyroCal": [(1, "enable", "bool")],
    "ReqFactoryGyroAttitude": [(1, "mode", "int")],
    "ReqFactoryMoveIrcutFloor": [(1, "floor", "int"), (2, "dir", "int")],

    # ── responses (device -> phone), included for decode-side completeness ──────
    "ResGetFactoryTestState": [(1, "sn", "str"), (2, "fatigue_test_mode", "int"),
                               (3, "wifi_start_up", "bool"), (4, "mac_address", "str")],
    "ResEMMCInAadOutSpeed": [(1, "cmd", "int"), (2, "code", "int"),
                             (3, "readspeed", "float"), (4, "writespeed", "float")],
    "ResErrorTeststart": [],
    "ResErrorTestState": [(1, "state", "int"), (2, "angle", "double"), (3, "dir", "int")],
    "ResFactoryGyroAttitude": [(1, "cmd", "int"), (2, "code", "int"), (3, "pitch", "double"),
                               (4, "yaw", "double"), (5, "roll", "double")],
    "ResAllTemperature": [(2, "code", "int")],  # `temps` (repeated TemperatureItem) omitted,
                                                  # decode via decode_temperature_items() below
    "ResFactoryMicRms": [(2, "code", "int")],   # `all_mic_rms` (repeated double) similarly
}

# nested-message schemas used inside fatigue-test config and repeated response fields
FATIGUE_IRCUT: list[Field] = [(1, "enable", "bool"), (2, "interval", "int"), (3, "time", "int")]
FATIGUE_MOTOR: list[Field] = [(1, "enable", "bool"), (2, "m_step", "int"), (3, "speed", "int"),
                               (4, "time", "int"), (5, "interval", "int")]
FATIGUE_LIGHTS: list[Field] = [(1, "enable", "bool"), (2, "interval", "int"), (3, "time", "int")]
FATIGUE_CAMERA: list[Field] = [(1, "enable", "bool")]
TEMPERATURE_ITEM: list[Field] = [(1, "id", "int"), (2, "name", "str"), (3, "value", "double")]

# The one cmd id this file will actually assert as fact. See the module
# docstring for why the other 37 are deliberately NOT enumerated here.
CMD_IDS: dict[str, int] = {
    "ReqSetSN": 15800,  # zlog-confirmed ("setSN"), dwarf3-tracking-protocol memory
}


# ── generic encode/decode over the SCHEMAS table ────────────────────────────────
def encode(msg_name: str, **kwargs) -> bytes:
    """Build the wire bytes for `msg_name` from field-name kwargs. Unset optional
    fields (proto3 zero-value) are simply omitted, matching real proto3 wire
    behavior."""
    schema = SCHEMAS[msg_name]
    out = b""
    for fn, name, typ in schema:
        if name not in kwargs:
            continue
        val = kwargs[name]
        if typ == "str":
            if val:
                out += _field(fn, 2, val)
        elif typ == "bool":
            if val:
                out += _field(fn, 0, 1)
        elif typ == "int":
            if val:
                out += _field(fn, 0, val)
        elif typ == "double":
            if val:
                out += _field(fn, 1, val)
        elif typ == "float":
            if val:
                out += _field(fn, 5, val)
        else:
            raise ValueError(f"unknown field type {typ!r} for {msg_name}.{name}")
    return out


def decode(msg_name: str, data: bytes) -> dict:
    """Decode wire bytes for `msg_name` into a {field_name: value} dict."""
    schema = SCHEMAS[msg_name]
    by_num = {fn: (name, typ) for fn, name, typ in schema}
    out: dict[str, Any] = {}
    i, n = 0, len(data)
    while i < n:
        tag, i = _read_varint(data, i)
        fn, wt = tag >> 3, tag & 7
        name_typ = by_num.get(fn)
        if wt == 0:
            v, i = _read_varint(data, i)
            if name_typ:
                name, typ = name_typ
                out[name] = bool(v) if typ == "bool" else v
            else:
                out[f"f{fn}"] = v
        elif wt == 1:
            v = struct.unpack_from("<d", data, i)[0]; i += 8
            out[(name_typ[0] if name_typ else f"f{fn}")] = v
        elif wt == 5:
            v = struct.unpack_from("<f", data, i)[0]; i += 4
            out[(name_typ[0] if name_typ else f"f{fn}")] = v
        elif wt == 2:
            ln, i = _read_varint(data, i)
            seg = data[i:i + ln]; i += ln
            try:
                out[(name_typ[0] if name_typ else f"f{fn}")] = seg.decode("utf-8")
            except UnicodeDecodeError:
                out[(name_typ[0] if name_typ else f"f{fn}")] = seg.hex(" ")
        else:
            break
    return out


def _encode_nested(schema: list[Field], **kwargs) -> bytes:
    out = b""
    for fn, name, typ in schema:
        if name not in kwargs or not kwargs[name]:
            continue
        val = kwargs[name]
        wt = {"str": 2, "bool": 0, "int": 0, "double": 1, "float": 5}[typ]
        out += _field(fn, 0 if typ == "bool" else wt, 1 if typ == "bool" and val else val)
    return out


def encode_fatigue_test_param(ircut=None, motor_rotate=None, motor_pitch=None,
                               motor_af=None, light=None, camera=None) -> bytes:
    """FatigueTestParam — the burn-in test configuration bundle. Each argument is
    a dict of that sub-message's fields (e.g. ircut={"enable": True, "interval":
    30, "time": 3600}). See the FATIGUE_* schemas above for each shape.
    **This configures endurance/burn-in testing of physical motors, IR-cut, and
    lights — do not build+send this against real hardware without understanding
    the consequence (repeated mechanical cycling for the configured duration).**
    """
    out = b""
    if ircut:
        out += _field(1, 2, _encode_nested(FATIGUE_IRCUT, **ircut))
    if motor_rotate:
        out += _field(2, 2, _encode_nested(FATIGUE_MOTOR, **motor_rotate))
    if motor_pitch:
        out += _field(3, 2, _encode_nested(FATIGUE_MOTOR, **motor_pitch))
    if motor_af:
        out += _field(4, 2, _encode_nested(FATIGUE_MOTOR, **motor_af))
    if light:
        out += _field(5, 2, _encode_nested(FATIGUE_LIGHTS, **light))
    if camera:
        out += _field(6, 2, _encode_nested(FATIGUE_CAMERA, **camera))
    return out


def decode_temperature_items(data: bytes) -> list[dict]:
    """Decode ResAllTemperature's repeated `temps` (field 1, TemperatureItem)."""
    items = []
    i, n = 0, len(data)
    while i < n:
        tag, i = _read_varint(data, i)
        fn, wt = tag >> 3, tag & 7
        if fn == 1 and wt == 2:
            ln, i = _read_varint(data, i)
            items.append(_decode_nested(TEMPERATURE_ITEM, data[i:i + ln]))
            i += ln
        elif wt == 0:
            _, i = _read_varint(data, i)
        elif wt == 1:
            i += 8
        elif wt == 5:
            i += 4
        elif wt == 2:
            ln, i = _read_varint(data, i); i += ln
        else:
            break
    return items


def _decode_nested(schema: list[Field], data: bytes) -> dict:
    by_num = {fn: (name, typ) for fn, name, typ in schema}
    out: dict[str, Any] = {}
    i, n = 0, len(data)
    while i < n:
        tag, i = _read_varint(data, i)
        fn, wt = tag >> 3, tag & 7
        name_typ = by_num.get(fn)
        if wt == 0:
            v, i = _read_varint(data, i)
            if name_typ:
                out[name_typ[0]] = bool(v) if name_typ[1] == "bool" else v
        elif wt == 1:
            v = struct.unpack_from("<d", data, i)[0]; i += 8
            if name_typ:
                out[name_typ[0]] = v
        elif wt == 2:
            ln, i = _read_varint(data, i); seg = data[i:i + ln]; i += ln
            if name_typ and name_typ[1] == "str":
                out[name_typ[0]] = seg.decode("utf-8", "replace")
        else:
            break
    return out


# ── self-test ────────────────────────────────────────────────────────────────────
def _selftest() -> int:
    print("factory_test_protocol self-test\n" + "=" * 40)
    fails = []

    def check(label, cond):
        print(("PASS " if cond else "FAIL ") + label)
        if not cond:
            fails.append(label)

    # every message with fields round-trips
    for name, schema in SCHEMAS.items():
        if not schema:
            continue
        sample = {}
        for fn, fname, typ in schema:
            sample[fname] = {"str": "X", "bool": True, "int": 7,
                              "double": 3.5, "float": 1.5}[typ]
        enc = encode(name, **sample)
        dec = decode(name, enc)
        ok = all(
            (dec.get(fname) == sample[fname]) if typ != "float"
            else abs(dec.get(fname, 0) - sample[fname]) < 1e-5
            for _, fname, typ in schema
        )
        check(f"{name} round-trip", ok)

    # empty-payload messages encode to b""
    check("ReqGetSN empty payload", encode("ReqGetSN") == b"")

    # fatigue-test bundle (nested sub-messages)
    bundle = encode_fatigue_test_param(
        ircut={"enable": True, "interval": 30, "time": 3600},
        motor_rotate={"enable": True, "m_step": 100, "speed": 5, "time": 60, "interval": 10},
        camera={"enable": True},
    )
    check("fatigue bundle non-empty", len(bundle) > 0)

    # ReqSetSN is the one CONFIRMED cmd id
    check("only ReqSetSN has a confirmed cmd id", list(CMD_IDS.keys()) == ["ReqSetSN"])
    check("ReqSetSN cmd id == 15800", CMD_IDS["ReqSetSN"] == 15800)

    print()
    if fails:
        print(f"{len(fails)} FAILURE(S):", fails)
        return 1
    print(f"ALL {len(SCHEMAS)} SCHEMAS + fatigue-bundle + cmd-id checks PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
