"""
DWARF3 RGB/Power-controller UART protocol -- PARTIAL, unlike
motor_uart_protocol.py. See memory: dwarf3-tracking-protocol.md, "RGB_POWER's
UART is a genuinely separate, sibling protocol".

RGB_POWER is a separate MCU from the motor controller, reachable over its own
serial link. Despite the name, it's a general system power-management
controller (power sequencing, scheduled auto-wake for unattended imaging
sessions, and battery charge-protection), not just RGB lighting.

Transport: /dev/ttyS4 on the device, 460800 baud, 8N1 (confirmed).

Wire framing philosophy matches the motor protocol's house style
('>'-prefixed, CRLF-terminated ASCII) but the SPECIFICS differ and were only
decoded for the init handshake:

    "> I I \\r\\n"   -- identify/ping. Verb byte repeated instead of a
                        computed checksum (unlike the motor protocol's real
                        additive checksum). 7-byte reply, echoes 'I', returns
                        two info bytes (likely hardware version/type).
    "> U U \\r\\n"   -- second handshake step. 9-byte reply, echoes 'U',
                        returns a full 32-bit BYTE-SWAPPED value (explicit
                        big-endian<->little-endian conversion seen in the
                        decompiled code) -- likely a firmware version or
                        serial number.

Known commands (real zlog debug names, purpose inferred from the name and
WsCmd context) -- **byte-level payload encoding was NOT decoded for any of
these**. Unlike the motor protocol, this file does not implement
build_frame/parse_frame for the general command set, only the two handshake
frames above, because the payload layout for the commands below is unknown.

    powerOn                    power on
    powerDown                  power down (matches CMD_RGB_POWER_POWER_DOWN)
    reboot                     hardware reboot (matches RgbPower::reboot(),
                                the watchdog capability in magni_backend_monitoring)
    powerIndOn / powerIndOff   power indicator LED on/off
                                (matches CMD_RGB_POWER_POWERIND_ON/OFF)
    custom_mode_2               RGB lighting effect mode (at least one more,
    custom_mode_4               custom_mode_4, exists -- likely more not found)
    setTimeStamp_               sets a timestamp on the controller
    setScheduledPowerOnTime_    scheduled auto-wake -- ties to the
                                SHOOTING_SCHEDULE WsCmd module: the device can
                                power itself on at a preset time for an
                                unattended imaging session
    setControlDevice_           generic device control, purpose not narrowed
    setEleSwitch_               electronic switch control (likely a physical
                                relay/power rail)
    disableReverseCharging      battery charge-protection logic -- confirms
                                this MCU also manages battery safety

RGB_POWER's own receive thread (a ~700-line function, zlog name
"operator()", mirroring the motor protocol's receive-thread structure) was
located but not decompiled in full, so its verb-dispatch switch (the
equivalent of what revealed the motor protocol's full verb table) was never
walked. That's the concrete next step if this file is worth completing --
same technique as motor_uart_protocol.py: find the frame-accumulate loop,
find its parser call, read the verb switch.
"""

START_BYTE = 0x3E  # '>'
TERMINATOR = b"\r\n"

DEVICE_PATH = "/dev/ttyS4"
BAUD_RATE = 460800

VERB_IDENTIFY = ord("I")
VERB_UNKNOWN_HANDSHAKE = ord("U")


def build_handshake_frame(verb: int) -> bytes:
    """The only two frames whose exact bytes are confirmed: the verb byte is
    simply repeated, with no computed checksum (unlike the motor protocol)."""
    return bytes([START_BYTE, verb, verb]) + TERMINATOR


def identify_frame() -> bytes:
    return build_handshake_frame(VERB_IDENTIFY)


def unknown_handshake_frame() -> bytes:
    return build_handshake_frame(VERB_UNKNOWN_HANDSHAKE)


if __name__ == "__main__":
    f = identify_frame()
    print("identify:", f)
    assert f == bytes([0x3E, ord("I"), ord("I"), 0x0D, 0x0A]), \
        "mismatch vs. firmware-observed frame (0x0D49493E + trailing 0x0A)"
    print("matches firmware-observed byte sequence exactly")

    f2 = unknown_handshake_frame()
    print("second handshake step:", f2)
    assert f2 == bytes([0x3E, ord("U"), ord("U"), 0x0D, 0x0A]), \
        "mismatch vs. firmware-observed frame (0xA0D55553E, low 5 bytes)"
    print("matches firmware-observed byte sequence exactly")
