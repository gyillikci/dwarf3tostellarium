#!/usr/bin/env python3
"""
dwarf_capture_decode.py — offline DWARF 3 traffic / tracking-protocol decoder.

EXTENSION to dwarf3tostellarium. Where dwarflab_controller.py was reconstructed
from the decompiled APK, this tool verifies (and corrects) that model against a
*real packet capture* of the iOS app <-> DWARF 3 session. Point it at a .pcapng
(from Wireshark, or `pktmon etl2pcap`) and it reconstructs the on-wire protocol:

    * de-duplicates pktmon's multi-component frames (each packet logged N times)
    * parses BOTH Ethernet (linktype 1) and raw 802.11+LLC/SNAP frames
      (pktmon on a Windows Mobile-Hotspot/Wi-Fi-Direct adapter captures 802.11)
    * reassembles the TCP control stream on port 9900 (the DWARF WebSocket)
    * unwraps WebSocket framing  (server->client unmasked, client->masked)
    * decodes the WsCmd protobuf envelope and every notify command
    * extracts the live tracking boxes {x,y,w,h} and their extents
    * decodes the UDP :9900 "txtl" heartbeat

WHAT THIS CAPTURE PROVED the APK-only model in dwarflab_controller.py missed
(see TRACKING_FINDINGS.md for the full write-up):

  1. cmd 15284  — a NOTIFY the firmware emits during tracking ({1:1, 2:1}); it
     is absent from the controller's command table. Mapped here as
     CMD_NOTIFY_WIDE_TRACK_STATE.
  2. App-level keepalives — the app sends WebSocket TEXT "ping" frames AND a
     UDP :9900 protobuf heartbeat {1:1, 2:<unix_ms>, 3:"txtl"}. The controller
     relies only on websocket-client's protocol-level PING; it never sends the
     app's "ping"/"txtl" keepalives.
  3. Box coordinate space — boxes are TOP-LEFT (x,y)+(w,h) in a FIXED ~1280x720
     reference (observed x+w -> ~1280, y+h -> ~720), NOT the live decoded frame
     size. roi_gui scales by the decoded frame dimensions, which only matches if
     the wide RTSP stream happens to be 1280x720.
  4. "No target" sentinel — x and y arrive as -100 (negative varint). The
     controller already handles this; documented here for completeness.

NEW — schema-aware deciphering (via dwarf_protobuf.py):
  * payloads decode to NAMED, TYPED fields instead of anonymous fN blobs — a goto
    request reads {ra, dec, name, goto_only}, a location {lat, lon, alt}, a joystick
    {angle_deg, length}; the envelope `type` resolves to REQUEST/RESPONSE/NOTIFY/ACK.
  * --decode-requests dumps the deciphered phone->device command payloads (the old
    tool only looked at device->phone notifies).
  * --json writes every deciphered frame (both directions) as NDJSON for analysis.
  * --mot-layout scores candidate field layouts for the still-unverified multi-
    object-track notifies (15238/15251) from a real capture.
  * even the "unknown payload" dump now shows the double/float interpretation of
    wire-type 1/5 fields (RA/Dec/lat/lon are doubles, not the raw uint64 shown before).

Usage:
    python3 dwarf_capture_decode.py capture.pcapng
    python3 dwarf_capture_decode.py capture.pcapng --port 9900 --boxes-csv boxes.csv
    python3 dwarf_capture_decode.py capture.pcapng --decode-requests --json frames.ndjson
    python3 dwarf_capture_decode.py capture.pcapng --mot-layout
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
from collections import defaultdict

# Schema-aware deciphering layer (typed values, named fields, enum resolution,
# request-payload decode). Optional — the tool still runs its transport/box
# analysis if this module is missing, it just won't decipher payloads.
try:
    import dwarf_protobuf as PB  # type: ignore
except Exception:  # pragma: no cover
    PB = None

# Command-id -> name. Imported from the controller when available so the two
# files never drift; falls back to an inline subset for standalone use.
try:
    import dwarflab_controller as D  # type: ignore
    CMD_NAMES = {v: k[4:] for k, v in vars(D).items()
                 if k.startswith("CMD_") and isinstance(v, int)}
    CMD_NAMES.setdefault(15284, "NOTIFY_WIDE_TRACK_STATE(new)")
except Exception:  # pragma: no cover - standalone fallback
    CMD_NAMES = {
        15201: "NOTIFY_ELE", 15203: "NOTIFY_TEMPERATURES",
        15211: "NOTIFY_STATE_ASTRO_GOTO", 15212: "NOTIFY_STATE_ASTRO_TRACKING",
        15225: "NOTIFY_TRACK_RESULT", 15232: "NOTIFY_SENTRY_MODE_TRACK_RESULT",
        15238: "NOTIFY_MULTI_TRACK_RESULT", 15251: "NOTIFY_WIDE_MULTI_TRACK_RESULT",
        15252: "NOTIFY_WIDE_TRACK_RESULT", 15284: "NOTIFY_WIDE_TRACK_STATE(new)",
        14800: "TRACK_START_TRACK", 14801: "TRACK_STOP_TRACK",
    }

# single-target track-result commands carrying {x,y,w,h}
BOX_CMDS = {15225, 15232, 15252}


# ── protobuf primitives ────────────────────────────────────────────────────────
def read_varint(buf: bytes, i: int):
    r = s = 0
    while True:
        b = buf[i]; i += 1
        r |= (b & 0x7F) << s
        if not (b & 0x80):
            return r, i
        s += 7


def to_signed(v: int) -> int:
    v &= 0xFFFFFFFFFFFFFFFF
    return v - 0x10000000000000000 if v >= 0x8000000000000000 else v


def decode_fields(data: bytes) -> dict:
    """Shallow decode: {field_number: value_or_bytes}. varints -> signed int."""
    out, i, n = {}, 0, len(data)
    try:
        while i < n:
            tag, i = read_varint(data, i)
            fn, wt = tag >> 3, tag & 7
            if wt == 0:
                v, i = read_varint(data, i); out[fn] = to_signed(v)
            elif wt == 2:
                ln, i = read_varint(data, i); out[fn] = data[i:i + ln]; i += ln
            elif wt == 1:
                out[fn] = struct.unpack_from("<Q", data, i)[0]; i += 8
            elif wt == 5:
                out[fn] = struct.unpack_from("<I", data, i)[0]; i += 4
            else:
                break
    except (IndexError, struct.error):
        pass
    return out


def parse_wscmd(raw: bytes) -> dict:
    """Decode the DWARF WsCmd envelope (mirrors controller.parse_ws_packet)."""
    f = decode_fields(raw)
    return {
        "major": f.get(1, 0), "minor": f.get(2, 0), "device_id": f.get(3, 0),
        "module_id": f.get(4, 0), "cmd": f.get(5, 0), "type": f.get(6, 0),
        "data": f.get(7, b"") if isinstance(f.get(7), (bytes, bytearray)) else b"",
        "client_id": f.get(8, b"").decode("utf-8", "replace")
                     if isinstance(f.get(8), (bytes, bytearray)) else "",
    }


def dump_pb(data: bytes, indent: int = 0) -> list:
    """Recursive pretty-dump of an unknown protobuf message."""
    out, pad, i, n = [], "  " * indent, 0, len(data)
    try:
        while i < n:
            tag, i = read_varint(data, i)
            fn, wt = tag >> 3, tag & 7
            if wt == 0:
                v, i = read_varint(data, i)
                out.append(f"{pad}f{fn} varint = {to_signed(v)}")
            elif wt == 2:
                ln, i = read_varint(data, i); seg = data[i:i + ln]; i += ln
                if seg and all(9 <= c <= 13 or 32 <= c <= 126 for c in seg):
                    out.append(f'{pad}f{fn} str = "{seg.decode()}"')
                else:
                    out.append(f"{pad}f{fn} bytes({ln}) = {seg.hex(' ')}")
                    out += dump_pb(seg, indent + 1)
            elif wt == 1:
                u = struct.unpack_from('<Q', data, i)[0]
                dbl = struct.unpack_from('<d', data, i)[0]; i += 8
                out.append(f"{pad}f{fn} i64 = {u}  (double {dbl:.6g})")
            elif wt == 5:
                u = struct.unpack_from('<I', data, i)[0]
                flt = struct.unpack_from('<f', data, i)[0]; i += 4
                out.append(f"{pad}f{fn} i32 = {u}  (float {flt:.6g})")
            else:
                break
    except (IndexError, struct.error):
        pass
    return out


# ── pcapng reader (Enhanced Packet Blocks only) ────────────────────────────────
def iter_pcapng_packets(path: str):
    """Yield raw link-layer frames from a little-endian pcapng file."""
    with open(path, "rb") as fh:
        blob = fh.read()
    pos, n = 0, len(blob)
    while pos + 12 <= n:
        btype, blen = struct.unpack_from("<II", blob, pos)
        if blen < 12 or pos + blen > n:
            break
        body = blob[pos + 8: pos + blen - 4]
        pos += blen
        if btype == 0x00000006:  # Enhanced Packet Block
            cap_len = struct.unpack_from("<I", body, 12)[0]
            yield body[20:20 + cap_len]


def l2_to_ipv4(frame: bytes):
    """Return the IPv4 payload offset, or None. Handles Ethernet and 802.11+SNAP."""
    if len(frame) < 14:
        return None
    # Try plain Ethernet first
    eth = struct.unpack_from(">H", frame, 12)[0]
    off = 14
    if eth == 0x8100:  # VLAN
        eth = struct.unpack_from(">H", frame, 16)[0]; off = 18
    if eth == 0x0800:
        return off
    # Try 802.11 data frame + LLC/SNAP (what pktmon yields on a Wi-Fi adapter)
    fc = frame[0]
    if (fc >> 2) & 3 == 2:                       # type = Data
        st = (fc >> 4) & 0xF
        if st & 4:                               # null/qos-null: no payload
            return None
        flags = frame[1]
        hdr = 24
        if (flags & 1) and (flags & 2):          # ToDS & FromDS -> addr4
            hdr += 6
        if st & 8:                               # QoS Data -> QoS control
            hdr += 2
        snap = hdr
        if frame[snap:snap + 2] != b"\xAA\xAA":
            if frame[snap + 8:snap + 10] == b"\xAA\xAA":   # CCMP header present
                snap += 8
            else:
                return None
        if struct.unpack_from(">H", frame, snap + 6)[0] == 0x0800:
            return snap + 8
    return None


# ── TCP/UDP extraction for one port ────────────────────────────────────────────
def extract_streams(path: str, port: int):
    """Reassemble both directions of the TCP <port> stream (dedup by seq) and
    collect UDP <port> payloads. Returns (tcp_dirs, udp_list)."""
    tcp = defaultdict(dict)           # (src,sport,dst,dport) -> {seq: payload}
    udp = []                          # (src, dst, payload)
    for frame in iter_pcapng_packets(path):
        off = l2_to_ipv4(frame)
        if off is None:
            continue
        ihl = (frame[off] & 0x0F) * 4
        proto = frame[off + 9]
        src = ".".join(str(b) for b in frame[off + 12:off + 16])
        dst = ".".join(str(b) for b in frame[off + 16:off + 20])
        tot = struct.unpack_from(">H", frame, off + 2)[0]
        l4 = off + ihl
        if proto == 6:                # TCP
            sport, dport = struct.unpack_from(">HH", frame, l4)
            if sport != port and dport != port:
                continue
            seq = struct.unpack_from(">I", frame, l4 + 4)[0]
            thl = ((frame[l4 + 12] >> 4) & 0xF) * 4
            plen = tot - ihl - thl
            if plen <= 0:
                continue
            payload = frame[l4 + thl:l4 + thl + plen]
            tcp[(src, sport, dst, dport)].setdefault(seq, payload)
        elif proto == 17:             # UDP
            sport, dport = struct.unpack_from(">HH", frame, l4)
            if dport != port and sport != port:
                continue
            ulen = struct.unpack_from(">H", frame, l4 + 4)[0] - 8
            if ulen > 0:
                udp.append((src, dst, frame[l4 + 8:l4 + 8 + ulen]))
    # assemble each TCP direction in sequence order
    dirs = {}
    for key, segs in tcp.items():
        dirs[key] = b"".join(segs[s] for s in sorted(segs))
    return dirs, udp


# ── WebSocket frame walker ─────────────────────────────────────────────────────
def iter_ws_frames(stream: bytes):
    """Yield (opcode, payload) for each WebSocket frame in a TCP byte stream."""
    pos, n = 0, len(stream)
    while pos + 2 <= n:
        b0, b1 = stream[pos], stream[pos + 1]; pos += 2
        op = b0 & 0x0F
        masked = b1 & 0x80
        ln = b1 & 0x7F
        if ln == 126:
            ln = struct.unpack_from(">H", stream, pos)[0]; pos += 2
        elif ln == 127:
            ln = struct.unpack_from(">Q", stream, pos)[0]; pos += 8
        mask = stream[pos:pos + 4] if masked else b""
        pos += 4 if masked else 0
        if pos + ln > n:
            break
        payload = bytearray(stream[pos:pos + ln]); pos += ln
        if mask:
            for i in range(len(payload)):
                payload[i] ^= mask[i & 3]
        yield op, bytes(payload)


# ── MOT layout scorer ───────────────────────────────────────────────────────────
def score_mot_layout(samples: list) -> None:
    """Infer the field layout of the multi-object-track notifies (15238/15251),
    whose sub-message schema is still unverified (TRACKING_FINDINGS.md §6). Each
    top-level length-delimited field is treated as one detected object; we gather
    per-field value statistics across every object, then score two candidate maps
    against a plausible pixel frame."""
    per_field = defaultdict(list)   # field_no -> [values]
    obj_count = 0
    for data in samples:
        i, n = 0, len(data)
        try:
            while i < n:
                tag, i = read_varint(data, i); fn, wt = tag >> 3, tag & 7
                if wt == 2:
                    ln, i = read_varint(data, i); sub = data[i:i + ln]; i += ln
                    f = decode_fields(sub)
                    if f:
                        obj_count += 1
                        for k, v in f.items():
                            if isinstance(v, int):
                                per_field[k].append(v)
                elif wt == 0:
                    _, i = read_varint(data, i)
                elif wt == 1:
                    i += 8
                elif wt == 5:
                    i += 4
                else:
                    break
        except (IndexError, struct.error):
            pass

    if not per_field:
        print(f"  {len(samples)} sample(s) but no nested sub-messages decoded — "
              "the layout may not be nested-per-object; dump one raw with dump_pb.")
        return
    print(f"  {len(samples)} notify sample(s), {obj_count} object sub-message(s)")
    print("  per sub-field value ranges:")
    for fn in sorted(per_field):
        vals = per_field[fn]
        print(f"    f{fn}: count={len(vals):<5} min={min(vals):>6} "
              f"max={max(vals):>6} distinct={len(set(vals))}")

    def plausible_box(xf, yf, wf, hf):
        for f in (xf, yf, wf, hf):
            if f not in per_field:
                return None
        ws = per_field[wf]; hs = per_field[hf]
        if min(ws) <= 0 or min(hs) <= 0:
            return None  # width/height must be positive
        xw = max(per_field[xf]) + max(ws); yh = max(per_field[yf]) + max(hs)
        # sane if extents stay within a generous wide-frame band
        return xw <= 4096 and yh <= 4096

    candidates = [
        ("{1:id, 2:x, 3:y, 4:w, 5:h}", 2, 3, 4, 5),
        ("{1:x, 2:y, 3:w, 4:h}",       1, 2, 3, 4),
        ("{1:id, 2:cls, 3:x, 4:y, 5:w, 6:h}", 3, 4, 5, 6),
    ]
    print("  candidate layout scoring:")
    for label, xf, yf, wf, hf in candidates:
        verdict = plausible_box(xf, yf, wf, hf)
        mark = "n/a (missing fields)" if verdict is None else ("PLAUSIBLE" if verdict
               else "rejected (impossible extents/size)")
        print(f"    {label:<38} {mark}")
    print("  NOTE: confirm against a capture where the app is actively AI-tracking "
          "multiple subjects; a single-object sample cannot disambiguate id vs x.")


# ── report ─────────────────────────────────────────────────────────────────────
def main() -> int:
    ap = argparse.ArgumentParser(description="Offline DWARF 3 tracking-protocol decoder")
    ap.add_argument("pcapng", help="capture file (Wireshark or `pktmon etl2pcap` output)")
    ap.add_argument("--port", type=int, default=9900, help="DWARF control/WebSocket port")
    ap.add_argument("--boxes-csv", help="write the full track-box time series to CSV")
    ap.add_argument("--json", dest="json_out", metavar="FILE",
                    help="write every deciphered WsCmd frame (both directions, "
                         "typed + schema-named payloads) as NDJSON")
    ap.add_argument("--decode-requests", action="store_true",
                    help="also decipher client->device request payloads "
                         "(goto/location/joystick/track-ROI), not just notifies")
    ap.add_argument("--mot-layout", action="store_true",
                    help="score candidate field layouts for the multi-object "
                         "track notifies (15238/15251) from the capture")
    args = ap.parse_args()

    dirs, udp = extract_streams(args.pcapng, args.port)
    if not dirs and not udp:
        print(f"No port-{args.port} traffic found in {args.pcapng}.", file=sys.stderr)
        return 1

    print(f"== TCP :{args.port} directions ==")
    for (src, sp, dst, dp), data in sorted(dirs.items(), key=lambda kv: -len(kv[1])):
        print(f"  {src}:{sp} -> {dst}:{dp}   {len(data)} bytes")

    hist = defaultdict(int)
    boxes = []                # (cmd, x, y, w, h)
    text_frames = defaultdict(int)
    unknown_dumps = {}
    timeline = []             # deciphered frames, in capture order, for NDJSON
    requests = {}             # cmd -> one deciphered request payload (phone->device)
    mot_samples = []          # raw payloads of 15238/15251 for layout scoring
    for (src, sp, dst, dp), data in dirs.items():
        # Direction from the TCP port: to the device port = request, from it = notify
        to_device = dp == args.port
        direction = "phone->device" if to_device else "device->phone"
        for op, payload in iter_ws_frames(data):
            if op == 1:
                text_frames[payload.decode("ascii", "replace")] += 1
            elif op in (8, 9, 10):
                text_frames[{8: "<CLOSE>", 9: "<PING>", 10: "<PONG>"}[op]] += 1
            elif op == 2:
                pk = parse_wscmd(payload)
                cmd = pk["cmd"]; hist[cmd] += 1
                if cmd in BOX_CMDS:
                    f = decode_fields(pk["data"])
                    boxes.append((cmd, f.get(1, -100), f.get(2, -100),
                                  f.get(3, 0), f.get(4, 0)))
                elif cmd not in CMD_NAMES and cmd not in unknown_dumps:
                    unknown_dumps[cmd] = dump_pb(pk["data"])
                if cmd in (15238, 15251) and pk["data"]:
                    mot_samples.append(pk["data"])
                # Schema-aware decipher (typed values, enum names) for export + requests
                if PB is not None:
                    dec = PB.decode_wscmd(payload)
                    dec["direction"] = direction
                    timeline.append(dec)
                    if to_device and cmd not in requests:
                        requests[cmd] = dec

    print("\n== WsCmd histogram ==")
    for cmd in sorted(hist):
        name = CMD_NAMES.get(cmd, "*** UNKNOWN — not in dwarflab_controller ***")
        print(f"  {cmd:<6} {hist[cmd]:>5}  {name}")

    if unknown_dumps:
        print("\n== Unknown command payloads (repo gap) ==")
        for cmd, lines in unknown_dumps.items():
            print(f"  cmd {cmd}:")
            for ln in lines:
                print(f"    {ln}")

    if text_frames:
        print("\n== WebSocket text/control frames (app keepalives) ==")
        for name, c in text_frames.items():
            print(f"  {c:>5}  {name!r}")

    valid = [b for b in boxes if b[1] > -100 and b[2] > -100]
    if boxes:
        print(f"\n== Track boxes: {len(boxes)} total, "
              f"{len(boxes) - len(valid)} 'no-target' (-100) ==")
        if valid:
            xs = [b[1] for b in valid]; ys = [b[2] for b in valid]
            ws = [b[3] for b in valid]; hs = [b[4] for b in valid]
            xw = [b[1] + b[3] for b in valid]; yh = [b[2] + b[4] for b in valid]

            def rng(a):
                return f"min={min(a):>5} max={max(a):>5} avg={sum(a)//len(a):>5}"
            print(f"  x   {rng(xs)}\n  y   {rng(ys)}\n  w   {rng(ws)}\n  h   {rng(hs)}")
            print(f"  x+w {rng(xw)}   <- approaches reference WIDTH")
            print(f"  y+h {rng(yh)}   <- approaches reference HEIGHT")
            print("  => coordinates are TOP-LEFT (x,y)+(w,h) in a fixed reference "
                  "frame (≈1280x720), not normalised.")

    for src, dst, payload in udp[:1]:
        f = decode_fields(payload)
        tag = f.get(3, b"")
        tag = tag.decode("ascii", "replace") if isinstance(tag, (bytes, bytearray)) else tag
        print(f"\n== UDP :{args.port} heartbeat ({len(udp)} packets) ==")
        print(f"  {src} -> {dst}: {{1:{f.get(1)}, 2:{f.get(2)} (unix-ms), 3:{tag!r}}}")

    # Deciphered request payloads (phone -> device) — schema-decoded
    if args.decode_requests and requests:
        print("\n== Deciphered request payloads (phone -> device) ==")
        for cmd in sorted(requests):
            dec = requests[cmd]
            flag = "" if dec.get("payload_schema") == "known" else "  [generic — repo gap]"
            print(f"  {cmd:<6} {dec['cmd_name']}{flag}")
            print(f"         {json.dumps(dec['payload'], ensure_ascii=False)}")

    # Multi-object-track layout scoring (15238/15251) — the still-unverified field map
    if args.mot_layout:
        print("\n== Multi-object track layout analysis (15238/15251) ==")
        if not mot_samples:
            print("  no 15238/15251 samples in this capture — "
                  "record the app running AI/MOT tracking to populate it.")
        else:
            score_mot_layout(mot_samples)

    # Full NDJSON export of every deciphered frame
    if args.json_out and timeline:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            for dec in timeline:
                fh.write(json.dumps(dec, ensure_ascii=False) + "\n")
        print(f"\nWrote {len(timeline)} deciphered frames to {args.json_out}")
    elif args.json_out and PB is None:
        print("\n--json requested but dwarf_protobuf.py is not importable; skipped.")

    if args.boxes_csv and boxes:
        with open(args.boxes_csv, "w", encoding="utf-8") as fh:
            fh.write("idx,cmd,x,y,w,h\n")
            for i, (cmd, x, y, w, h) in enumerate(boxes):
                fh.write(f"{i},{cmd},{x},{y},{w},{h}\n")
        print(f"\nWrote {len(boxes)} boxes to {args.boxes_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
