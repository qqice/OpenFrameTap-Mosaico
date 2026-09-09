"""Inspect partition metadata in a bounded flash prefix; never write the device."""
import hashlib
from pathlib import Path
import struct
import sys

data=Path(sys.argv[1]).read_bytes()
found=0
for start in range(0,len(data),4096):
    if data[start:start+2]!=b'\xaa\x50':continue
    rows=[]
    for offset in range(start,min(start+4096,len(data)),32):
        entry=data[offset:offset+32]
        if entry[:2]==b'\xeb\xeb':
            if entry[16:]!=hashlib.md5(data[start:offset]).digest():rows=[]
            break
        if entry==b'\xff'*32:break
        if len(entry)!=32 or entry[:2]!=b'\xaa\x50':rows=[];break
        _,kind,sub,address,size,label,flags=struct.unpack('<HBBII16sI',entry)
        rows.append((kind,sub,address,size,label.rstrip(b'\0').decode('ascii',errors='replace')))
    if rows:
        found+=1
        print(f'PARTITION_TABLE offset=0x{start:x}')
        for kind,sub,address,size,label in rows:
            print(f'  type=0x{kind:02x} subtype=0x{sub:02x} offset=0x{address:x} size=0x{size:x} label={label}')
print(f'tables_found={found}; prefix_sha256={hashlib.sha256(data).hexdigest()}')
