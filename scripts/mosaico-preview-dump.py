"""Transfer the MCU-generated BMP without decoding any video on the computer."""
import argparse
import base64
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import re
import time
import serial
from serial.tools import list_ports

p=argparse.ArgumentParser()
p.add_argument('--port')
p.add_argument('--timeout',type=int,default=30)
args=p.parse_args()
ports=[s for s in list_ports.comports() if s.vid==0x303A and s.pid==0x4001 and (not args.port or s.device==args.port)]
if len(ports)!=1:raise SystemExit('Expected one running Mosaico CDC device; re-enumerate its USB port.')
if not 1<=args.timeout<=120:raise SystemExit('timeout must be1..120s')
root=Path(__file__).resolve().parent.parent/'artifacts/mosaico/preview'
root.mkdir(parents=True,exist_ok=True)
stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
log_path=root/(stamp+'-transfer.log');image_path=root/(stamp+'-real-frame.bmp')
blocks={};expected=None;digest=None;finished=False
with serial.Serial(ports[0].device,115200,timeout=.2) as uart,log_path.open('xb') as log:
    uart.write(b'preview-dump\n');uart.flush();end=time.monotonic()+args.timeout;pending=b''
    while time.monotonic()<end:
        data=uart.read(4096)
        if not data:continue
        log.write(data);pending+=data
        while b'\n' in pending:
            row,pending=pending.split(b'\n',1);row=row.rstrip(b'\r')
            if b'OFT_IMAGE_NOT_READY' in row:raise SystemExit('MCU has not decoded a frame yet.')
            match=re.search(rb'OFT_IMAGE_BEGIN bytes=(\d+) sha256=([0-9a-f]{64})',row)
            if match:
                expected=int(match[1]);digest=match[2].decode()
                if expected>1024*1024:raise SystemExit('Refusing an oversized image transfer.')
            match=re.search(rb'^OFT_IMAGE_DATA (\d+) ([A-Za-z0-9+/=]+)$',row)
            if match and expected is not None:
                index=int(match[1]);chunk=base64.b64decode(match[2],validate=True)
                if index>4096 or len(chunk)>288:raise SystemExit('Invalid transfer chunk.')
                if index in blocks and blocks[index]!=chunk:raise SystemExit('Conflicting transfer chunk.')
                blocks[index]=chunk
            if row==b'OFT_IMAGE_END':finished=True;break
        if finished:break
        if len(pending)>8192:raise SystemExit('Malformed serial transfer line.')
if not finished or expected is None:raise SystemExit(f'Transfer incomplete; raw log retained: {log_path}')
count=(expected+287)//288
if set(blocks)!=set(range(count)):raise SystemExit(f'Missing transfer chunks; raw log retained: {log_path}')
raw=b''.join(blocks[i] for i in range(count))
if len(raw)!=expected or hashlib.sha256(raw).hexdigest()!=digest:raise SystemExit('MCU/host image hash mismatch; image not accepted.')
with image_path.open('xb') as output:output.write(raw)
print(f'IMAGE={image_path}\nSHA256={digest}\nBytes={len(raw)}; BMP generated on Mosaico; host transferred bytes only.')
