#!/usr/bin/env python3
"""
ws_sniff.py — man-in-the-middle WebSocket logger for the Dwarf3 protocol.

Sits between the DWARF Lab app and the Dwarf3 telescope, forwarding the
ws:// connection byte-for-byte in both directions while decoding every binary
frame with this repo's own parser (parse_ws_packet / _dvarint).  Use it to
discover command IDs + protobuf payloads that dwarflab_controller.py does not
yet implement — e.g. the ROI / subject-tracking command.

Why this works: the Dwarf3 WebSocket (port 9900) is plaintext ws:// — no TLS —
so once the app's connection is routed through this proxy you can read every
command in the clear.

Topology (Banana Pi as Wi-Fi AP, phone + Dwarf3 both joined — see SNIFFING.md):

    app ──ws──► [ this proxy on the Pi : 9900 ] ──ws──► Dwarf3 : 9900

Redirect the app to the proxy with a source-matched DNAT rule on the Pi so the
proxy's OWN upstream connection is not also redirected (which would loop):

    iptables -t nat -A PREROUTING -s <PHONE_IP> -d <DWARF_IP> \
             -p tcp --dport 9900 -j DNAT --to-destination <PI_IP>:9900

Usage:
    python3 ws_sniff.py --upstream <DWARF_IP> [--listen 0.0.0.0:9900]

Output (one line per frame):
    ▶ APP→DWARF  cmd=11013 START_ONE_CLICK_GOTO_DSO  module=3  type=0
                 data=120a... | field1=f64:10.68 field2=f64:41.27 field3=bytes:'M31'
    ◀ DWARF→APP  cmd=15211 NOTIFY_STATE_ASTRO_GOTO   module=9  type=1  ...

The byte stream is relayed verbatim; parsing is a best-effort read of a copy,
so a parse error never disturbs the live connection.
"""

import argparse
import socket
import struct
import threading

import dwarflab_controller as dlc
from dwarflab_controller import _dvarint, _module_for_cmd, parse_ws_packet

# cmd-number → short name, built from the CMD_* constants in the controller.
_CMD_NAMES = {
    v: k[4:] for k, v in vars(dlc).items()
    if k.startswith("CMD_") and isinstance(v, int)
}
_WT = {0: "varint", 1: "f64", 2: "bytes", 5: "f32"}


# ── protobuf raw decoder (schema-free, like `protoc --decode_raw`) ─────────────
def decode_raw(data: bytes) -> str:
    """Best-effort dump of a protobuf message: 'field1=f64:10.68 field2=bytes:..'."""
    parts = []
    i = 0
    try:
        while i < len(data):
            tag, i = _dvarint(data, i)
            fn, wt = tag >> 3, tag & 7
            if wt == 0:
                v, i = _dvarint(data, i)
                parts.append(f"f{fn}=varint:{v}")
            elif wt == 1:
                v = struct.unpack_from("<d", data, i)[0]; i += 8
                parts.append(f"f{fn}=f64:{v:g}")
            elif wt == 5:
                v = struct.unpack_from("<f", data, i)[0]; i += 4
                parts.append(f"f{fn}=f32:{v:g}")
            elif wt == 2:
                ln, i = _dvarint(data, i)
                pay = data[i:i + ln]; i += ln
                try:
                    s = pay.decode("utf-8")
                    if s.isprintable():
                        parts.append(f"f{fn}=str:'{s}'")
                        continue
                except UnicodeDecodeError:
                    pass
                parts.append(f"f{fn}=bytes[{len(pay)}]:{pay.hex()}")
            else:
                break
    except (IndexError, struct.error):
        parts.append("<truncated>")
    return " ".join(parts)


def log_frame(direction: str, payload: bytes):
    """Decode and print one application (binary) WebSocket frame."""
    pkt = parse_ws_packet(payload)
    cmd = pkt["cmd"]
    name = _CMD_NAMES.get(cmd, "?")
    inner = decode_raw(pkt["data"]) if pkt["data"] else ""
    arrow = "▶ APP→DWARF" if direction == "c2s" else "◀ DWARF→APP"
    line = (f"{arrow}  cmd={cmd} {name}  module={_module_for_cmd(cmd)}  "
            f"type={pkt['type']}")
    if inner:
        line += f"\n             | {inner}"
    print(line, flush=True)


# ── minimal WebSocket frame parser (logging only; stream is forwarded raw) ────
def iter_frames(buf: bytearray):
    """Yield (opcode, payload) for every complete frame in buf, trimming buf.

    Handles client masking and 7/16/64-bit length forms. Continuation frames
    (opcode 0) are concatenated onto the previous data/text frame by the caller.
    """
    while True:
        if len(buf) < 2:
            return
        b0, b1 = buf[0], buf[1]
        opcode = b0 & 0x0F
        masked = b1 & 0x80
        ln = b1 & 0x7F
        idx = 2
        if ln == 126:
            if len(buf) < 4:
                return
            ln = int.from_bytes(buf[2:4], "big"); idx = 4
        elif ln == 127:
            if len(buf) < 10:
                return
            ln = int.from_bytes(buf[2:10], "big"); idx = 10
        if masked:
            if len(buf) < idx + 4:
                return
            mask = buf[idx:idx + 4]; idx += 4
        if len(buf) < idx + ln:
            return
        payload = bytes(buf[idx:idx + ln])
        if masked:
            payload = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        del buf[:idx + ln]
        yield opcode, payload


def relay(src: socket.socket, dst: socket.socket, direction: str):
    """Forward src→dst verbatim, decoding binary frames on a copy as we go."""
    buf = bytearray()
    msg = bytearray()          # reassembles fragmented binary messages
    try:
        while True:
            chunk = src.recv(65536)
            if not chunk:
                break
            dst.sendall(chunk)        # byte-exact relay
            buf += chunk
            for opcode, payload in iter_frames(buf):
                if opcode in (0x1, 0x2):      # text / binary: start of message
                    msg = bytearray(payload)
                elif opcode == 0x0:           # continuation
                    msg += payload
                elif opcode == 0x8:           # close
                    return
                else:
                    continue                  # ping/pong: ignore
                # On a non-fragmented frame the whole message is in `payload`;
                # for fragmented ones `msg` holds the reassembly so far. We log
                # opportunistically — parse_ws_packet tolerates partial input.
                if msg:
                    log_frame(direction, bytes(msg))
    except OSError:
        pass
    finally:
        for s in (src, dst):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def handle_client(client: socket.socket, upstream_host: str, upstream_port: int):
    up = socket.create_connection((upstream_host, upstream_port))
    threading.Thread(target=relay, args=(client, up, "c2s"), daemon=True).start()
    relay(up, client, "s2u")
    for s in (client, up):
        try:
            s.close()
        except OSError:
            pass


def main():
    p = argparse.ArgumentParser(description="Dwarf3 WebSocket MITM logger")
    p.add_argument("--upstream", required=True,
                   help="real Dwarf3 IP (the proxy connects here)")
    p.add_argument("--upstream-port", type=int, default=dlc.DwarfLab.WS_PORT)
    p.add_argument("--listen", default="0.0.0.0:9900",
                   help="host:port to accept the app's connection on")
    args = p.parse_args()

    host, _, port = args.listen.rpartition(":")
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host or "0.0.0.0", int(port)))
    srv.listen(5)
    print(f"ws_sniff: listening on {host or '0.0.0.0'}:{port} → "
          f"upstream {args.upstream}:{args.upstream_port}", flush=True)
    while True:
        conn, addr = srv.accept()
        print(f"-- client {addr[0]}:{addr[1]} connected", flush=True)
        threading.Thread(
            target=handle_client,
            args=(conn, args.upstream, args.upstream_port),
            daemon=True,
        ).start()


if __name__ == "__main__":
    main()
