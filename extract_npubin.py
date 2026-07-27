"""
Extract sections (including raw NPU weights) from a DWARF 3 .npubin model
container, based on AR_NPUBIN_HEADER_S recovered from libhal_npu.so via
Ghidra (dwarf3/firmware/ghidra_npubin_structs.txt).

Header layout (248 bytes, all AR_U32 = uint32, assumed little-endian on this
ARM AR9341 target):

  0x00 magic              0x04 toolBuildTime     0x08 length
  0x0c checksum           0x10 socTarget         0x14 headVersion
  0x18 scuOffset          0x1c scuLen
  0x20 weightOffset       0x24 weightLen          <- the actual model weights
  0x28 ifcOffset          0x2c ifcLen
  0x30 cbOffset           0x34 cbLen
  0x38 scuLogOffset       0x3c scuLogLen
  0x40 outputOffset       0x44 outputLen
  0x48 postprocessOffset  0x4c postprocessLen
  0x50 performanceOffset  0x54 performanceLen
  0x58 inputOffset        0x5c inputLen
  0x60 iniOffset          0x64 iniLen
  0x68 npuFreq            0x6c sramSize
  0x70 onnxOffset         0x74 onnxLen
  0x78 gitID[128]
"""
import struct
import sys
import os
import json

HEADER_FMT = "<30I128s"
HEADER_SIZE = 248

FIELDS = [
    "magic", "toolBuildTime", "length", "checksum", "socTarget", "headVersion",
    "scuOffset", "scuLen", "weightOffset", "weightLen", "ifcOffset", "ifcLen",
    "cbOffset", "cbLen", "scuLogOffset", "scuLogLen", "outputOffset", "outputLen",
    "postprocessOffset", "postprocessLen", "performanceOffset", "performanceLen",
    "inputOffset", "inputLen", "iniOffset", "iniLen", "npuFreq", "sramSize",
    "onnxOffset", "onnxLen",
]

SECTIONS = [
    ("scuLog", "scuLogOffset", "scuLogLen", "scu_log.json"),
    ("ifc", "ifcOffset", "ifcLen", "ifc.json"),
    ("scu", "scuOffset", "scuLen", "scu.bin"),
    ("weight", "weightOffset", "weightLen", "weights.bin"),
    ("output", "outputOffset", "outputLen", "output.json"),
    ("input", "inputOffset", "inputLen", "input.json"),
    ("cb", "cbOffset", "cbLen", "callback.json"),
    ("postprocess", "postprocessOffset", "postprocessLen", "post_process.json"),
    ("performance", "performanceOffset", "performanceLen", "performance.bin"),
    ("ini", "iniOffset", "iniLen", "config.ini"),
    ("onnx", "onnxOffset", "onnxLen", "model.onnx"),
]


def parse_header(buf):
    raw = struct.unpack(HEADER_FMT, buf[:HEADER_SIZE])
    header = dict(zip(FIELDS, raw[:-1]))
    git_raw = raw[-1]
    header["gitID"] = git_raw.split(b"\x00", 1)[0].decode("ascii", "replace")
    return header


def dump_npubin(path, out_dir):
    with open(path, "rb") as f:
        buf = f.read()

    magic_bytes = buf[0:4]
    header = parse_header(buf)

    print("file: %s (%d bytes)" % (path, len(buf)))
    print("magic: %r (0x%08x)" % (magic_bytes, header["magic"]))
    print("toolBuildTime: %d  length(claimed): %d  checksum: 0x%08x" % (
        header["toolBuildTime"], header["length"], header["checksum"]))
    print("socTarget: %d  headVersion: %d  npuFreq: %d  sramSize: %d" % (
        header["socTarget"], header["headVersion"], header["npuFreq"], header["sramSize"]))
    if header["headVersion"] > 2:
        print("gitID: %s" % header["gitID"])

    os.makedirs(out_dir, exist_ok=True)

    for name, off_key, len_key, fname in SECTIONS:
        off = header[off_key]
        ln = header[len_key]
        print("%-12s offset=0x%08x (%9d)  len=%9d" % (name, off, off, ln))
        if ln == 0:
            continue
        if off + ln > len(buf):
            print("  !! section extends past EOF (off+len=%d, file=%d) - skipping" % (
                off + ln, len(buf)))
            continue
        section = buf[off:off + ln]
        out_path = os.path.join(out_dir, fname)
        with open(out_path, "wb") as f:
            f.write(section)
        print("  -> wrote %s" % out_path)

    return header


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: extract_npubin.py <model.npubin> [out_dir]")
        sys.exit(1)
    npubin_path = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(npubin_path)[0] + "_extracted"
    dump_npubin(npubin_path, out_dir)
