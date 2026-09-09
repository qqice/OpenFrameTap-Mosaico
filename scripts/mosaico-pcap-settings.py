"""Extract only CRC-valid named camera-setting DUML frames; never decode media."""
import argparse
import hashlib
import json
import struct
from pathlib import Path

KEYS = (b"camcap_video_format", b"cam_video_param_v2", b"cam_lens_state",
        b"cam_fov", b"cam_photo_param_new", b"camcap_photo_size",
        b"camcap_photo_storage_format", b"camcap_zoom")

def crc(data, value, polynomial):
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value

def main():
    p = argparse.ArgumentParser()
    p.add_argument("capture", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    if args.output.exists():
        raise SystemExit("Evidence output exists")
    found, seen = [], set()
    with args.capture.open("rb") as f:
        magic = f.read(24)
        order = {b"\xd4\xc3\xb2\xa1": "<", b"\xa1\xb2\xc3\xd4": ">"}.get(magic[:4])
        if not order:
            raise SystemExit("Requires classic pcap with microsecond timestamps")
        packet = 0
        while header := f.read(16):
            if len(header) != 16:
                raise SystemExit("Truncated record")
            seconds, micros, size, _ = struct.unpack(order + "4I", header)
            if size > 1_048_576:
                raise SystemExit("Record exceeds bound")
            data = f.read(size)
            if len(data) != size:
                raise SystemExit("Truncated packet")
            packet += 1
            for key in KEYS:
                if sum(x["key"] == key.decode() for x in found) >= 10:
                    continue
                at = data.find(key)
                if at < 0:
                    continue
                for start in range(max(0, at - 96), at):
                    if data[start] != 0x55 or start + 13 > len(data):
                        continue
                    length = (data[start+1] | data[start+2] << 8) & 1023
                    raw = data[start:start+length]
                    if length < 13 or len(raw) != length or raw[9:11] != b"\x00\x99":
                        continue
                    if key not in raw or crc(raw[:3], 0x77, 0x8c) != raw[3] or crc(raw[:-2], 0x3692, 0x8408) != int.from_bytes(raw[-2:], "little"):
                        continue
                    identity = (key, raw[4], raw[11:])
                    if identity not in seen:
                        seen.add(identity)
                        found.append(dict(packet=packet, timestamp=seconds+micros/1e6,
                                          key=key.decode(), sender=raw[4], receiver=raw[5],
                                          raw_sha256=hashlib.sha256(raw).hexdigest(), raw_hex=raw.hex()))
                    break
    with args.capture.open("rb") as f:
        digest = hashlib.file_digest(f, "sha256").hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(source=str(args.capture.resolve()), sha256=digest,
                                          metadata_only=True, frames=found), indent=2), encoding="utf-8")
    print(json.dumps(dict(frames=len(found), keys=sorted({x["key"] for x in found}), output=str(args.output))))

if __name__ == "__main__":
    main()
