"""Bounded CDC log capture; optional explicit 1200-baud ROM-download request."""
import argparse
from datetime import datetime,timezone
from pathlib import Path
import time
import re
import serial
from serial.tools import list_ports

p=argparse.ArgumentParser()
p.add_argument('--port')
p.add_argument('--seconds',type=int,default=15)
actions=p.add_mutually_exclusive_group()
actions.add_argument('--enter-download',action='store_true')
actions.add_argument('--selftest',action='store_true')
actions.add_argument('--ble-scan',action='store_true',help='Restart local BLE discovery; no raw camera command')
actions.add_argument('--gauge',choices=('audit','apply','restore','access','history','model'),help='Typed local BQ27220 RAM diagnostic/configuration; no camera writes')
actions.add_argument('--settings',choices=('refresh','video-test','zoom-test','photo-test','photo-file-test','photo-size-test','hd-test','27k-test','mode-photo','mode-video'),help='Typed camera capability refresh or one setting change/readback/restore; never shoots')
actions.add_argument('--ui-hold-test',action='store_true',help='Shadow GUI long press with camera writes isolated')
actions.add_argument('--ui-menu-test',action='store_true',help='Local radial overlay geometry only; no camera action')
actions.add_argument('--video',choices=('keyframe','live','fast','quick','1500','1200','1000','off','show','hide','fresh','balance','trace-on','trace','bench','loss-test','burst','burst6','idr'),help='Typed09/A8 tests, page lifecycle, bounded trace, loss/burst trial or on-board replay; never presses Hold or sends nonzero motion')
actions.add_argument('--haptic-test',action='store_true',help='One50ms motor pulse in JOYSTICK mode; no camera command')
actions.add_argument('--haptic-double-test',action='store_true',help='Two50ms acknowledgement pulses; no camera input')
actions.add_argument('--guided-limits',action='store_true',help='300s manual pose/limit guide; serial writes are timestamp markers only')
actions.add_argument('--guided-opposite-pitch','--guided-selfie-pitch',dest='guided_selfie_pitch',action='store_true',help='90s missing opposite-pole pitch endpoints; verify actual yaw after Center')
actions.add_argument('--probe',choices=('yaw+','yaw-','pitch+','pitch-'),help='One low-offset 200ms motion probe; prepare a clear space around the gimbal first')
args=p.parse_args()
if not 1<=args.seconds<=900:raise SystemExit('seconds must be 1..900 (bounded capture)')
if args.guided_limits:args.seconds=300
if args.guided_selfie_pitch:args.seconds=90
guided=args.guided_limits or args.guided_selfie_pitch
guide=[
 (0,'baseline','选择 JOYSTICK；Pocket 固定不动，保持当前朝向。点击 Center，等待。'),
 (15,'yawL1','当前朝向：Center 后等3秒，摇杆缓慢向左至自行停止，立刻松手，余下时间静置。'),
 (40,'yawR1','Center 后等3秒，摇杆缓慢向右至自行停止，立刻松手，余下时间静置。'),
 (65,'pitchU1','Center 后等3秒，摇杆缓慢向上至自行停止，立刻松手，余下时间静置。'),
 (90,'pitchD1','Center 后等3秒，摇杆缓慢向下至自行停止，立刻松手，余下时间静置。'),
 (115,'flip','点击 Center，等3秒，再点击180 deg；等待转完，勿移动 Pocket 机身。'),
 (135,'yawL2','另一朝向：Center 后等3秒，摇杆缓慢向左至自行停止，立刻松手，余下时间静置。'),
 (160,'yawR2','Center 后等3秒，摇杆缓慢向右至自行停止，立刻松手，余下时间静置。'),
 (185,'pitchU2','Center 后等3秒，摇杆缓慢向上至自行停止，立刻松手，余下时间静置。'),
 (210,'pitchD2','Center 后等3秒，摇杆缓慢向下至自行停止，立刻松手，余下时间静置。'),
 (235,'restore','点击 Center，等3秒，再点击180 deg返回原朝向；松开所有控件。'),
 (250,'magnetic','只转 Mosaico，不碰屏幕：依次让屏幕朝上/朝下/上边朝上/下边朝上/左边朝上/右边朝上，各停3秒；勿拉扯USB线。'),
 (290,'finish','将 Mosaico 放平，不操作；等待采集结束。'),
]
if args.guided_selfie_pitch:
    guide=[
        (0,'selfiePrep','选择 JOYSTICK。Center后看终端朝向提示：若未就绪，点180 deg使yaw接近正负180度。就绪后不要再点Center。'),
        (15,'selfieU','目标朝向已核对：摇杆缓慢向上至自行停止，立刻松手，保持。不要用手掰镜头。'),
        (40,'selfieSet','点击Center后重新看终端朝向提示；若未就绪，点180 deg。必须恢复yaw接近正负180度，再保持不动。'),
        (55,'selfieD','目标朝向已核对：摇杆缓慢向下至自行停止，立刻松手，保持。'),
        (80,'selfieDone','松开所有控件；可按Center恢复。等待结束，并报告两段开始时实际镜头朝向。'),
    ]
ports=[s for s in list_ports.comports() if s.vid==0x303A]
if args.port:
    ports=[s for s in ports if s.device==args.port]
if len(ports)!=1:raise SystemExit('Expected one identified Espressif USB port; re-enumerate or specify --port')
port=ports[0].device
root=Path(__file__).resolve().parent.parent/'artifacts/mosaico/logs'
root.mkdir(parents=True,exist_ok=True)
path=root/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-cdc.txt')
uart=serial.Serial(port=None,baudrate=1200 if args.enter_download else 115200,timeout=.2)
uart.port=port;uart.dtr=True;uart.rts=False
probe_accepted=False;probe_complete=False;probe_receipt=False
probe_nonzero=0;probe_zero=0;pending_line=''
print(f'CDC port={port} log={path} download_request={args.enter_download}',flush=True)
try:
    uart.open()
    if args.selftest:uart.write(b'mark-check\nselftest\n');uart.flush()
    if args.ble_scan:uart.write(b'ble-scan\n');uart.flush()
    if args.gauge:uart.write(('gauge-'+args.gauge+'\n').encode('ascii'));uart.flush()
    if args.settings:uart.write(('settings-'+args.settings+'\n').encode('ascii'));uart.flush()
    if args.ui_hold_test:uart.write(b'ui-hold-test\n');uart.flush()
    if args.ui_menu_test:uart.write(b'ui-menu-test\n');uart.flush()
    if args.video:uart.write(('video-'+args.video+'\n').encode('ascii'));uart.flush()
    if args.haptic_test:uart.write(b'haptic-test\n');uart.flush()
    if args.haptic_double_test:uart.write(b'haptic-double-test\n');uart.flush()
    if args.probe:uart.write(('probe-'+args.probe+'\n').encode('ascii'));uart.flush()
    start=time.monotonic();end=start+(1 if args.enter_download else args.seconds);phase=0;marker_ack=False
    guide_yaw=None;guide_yaw_at=0;pose_ready_previous=None;guide_clock_us=0;guide_sample_us=0
    if guided:
        print(f'手动采集{args.seconds}秒：软件不自动移动相机；到端点立刻松手，不用手掰镜头。\n任何异常先松开摇杆。Ctrl+C只结束采集，不替你松开摇杆。',flush=True)
        if args.guided_limits:print('注意：Center可能改变前后朝向，阶段编号不能证明是不同朝向。需补自拍pitch时请使用 --guided-selfie-pitch。',flush=True)
    with path.open('wb') as log:
        while time.monotonic()<end:
            if guided and time.monotonic()-start>10 and not marker_ack:
                raise SystemExit('未收到开发板时间标记确认，停止指导。请松开摇杆，保留日志。')
            if guided and phase<len(guide) and time.monotonic()-start>=guide[phase][0]:
                at,name,instruction=guide[phase]
                if args.guided_selfie_pitch and name in ('selfieU','selfieD'):
                    if guide_yaw is None or abs(guide_yaw)<150 or time.monotonic()-guide_yaw_at>2 or not guide_clock_us or abs(guide_clock_us-guide_sample_us)>2000000:
                        raise SystemExit(f'目标朝向未就绪或反馈过期（yaw={guide_yaw}），停止指导。请勿开始动作；不按按钮次数推断朝向。')
                uart.write(('mark-'+name+'\n').encode('ascii'));uart.flush()
                print(f'\n[{at:03d}/{args.seconds}s] {instruction}',flush=True);phase+=1
            try:data=uart.read(4096)
            except serial.SerialException:
                if args.enter_download:break
                raise
            if data:
                log.write(data);log.flush()
                if not guided:print(data.decode('utf-8',errors='replace'),end='',flush=True)
                if guided:
                    pending_line+=data.decode('utf-8',errors='replace').replace('\r','')
                    while '\n' in pending_line:
                        line,pending_line=pending_line.split('\n',1)
                        if 'OFT_MARK ' in line:marker_ack=True
                        clock_match=re.search(r'OFT_BOOT .*uptime_ms=(\d+)',line)
                        if clock_match:guide_clock_us=int(clock_match[1])*1000
                        m=re.search(r'OFT_GIMBAL_SNAPSHOT us=(\d+) duml=([0-9a-f]{124})',line)
                        if args.guided_selfie_pitch and m:
                            raw=bytes.fromhex(m[2]);guide_sample_us=int(m[1]);guide_yaw=int.from_bytes(raw[27:29],'little',signed=True)/100;guide_yaw_at=time.monotonic()
                            ready=abs(guide_yaw)>=150
                            if phase in (1,3) and ready!=pose_ready_previous:
                                print(f'朝向: yaw={guide_yaw:.2f} -> '+('目标朝向已就绪，请保持' if ready else '目标朝向未就绪，请按180 deg后再看提示'),flush=True)
                            pose_ready_previous=ready if phase in (1,3) else None
                        if 'OFT_UDP state=5 ' in line or any(s in line for s in ('Guru Meditation','assert failed','Brownout detector','panic_abort')):
                            print(line,flush=True)
                            raise SystemExit('检测到开发板异常，停止指导。请立即松开摇杆；不自动重试。')
                    pending_line=pending_line[-4096:]
                if args.probe:
                    pending_line+=data.decode('utf-8',errors='replace').replace('\r','')
                    while '\n' in pending_line:
                        line,pending_line=pending_line.split('\n',1)
                        if 'OFT_PROBE request=' in line and 'accepted=1' in line:probe_accepted=True
                        if probe_accepted:
                            match=re.search(r'oft_udp: stick .*yaw_offset=(-?\d+) pitch_offset=(-?\d+)',line)
                            if match:
                                probe_receipt=False # require a receipt after the latest stop, not a priming zero
                                if int(match[1]) or int(match[2]):probe_nonzero+=1
                                else:probe_zero+=1
                            if 'probe complete; three priming zeros, one nonzero, two stop zeros' in line:probe_complete=True
                            if 'OFT_UDP_STOP receipt=1' in line:probe_receipt=True
                    pending_line=pending_line[-4096:]
finally:
    uart.close()
if guided:print(f'采集结束，原始数据：{path}。请说明是否完成全部端点、是否有异常。',flush=True)
if args.probe:
    passed=probe_accepted and probe_complete and probe_nonzero==1 and probe_zero==5 and probe_receipt
    print(f'Probe transport check: accepted={probe_accepted} nonzero={probe_nonzero} zeros={probe_zero} (3 priming + 2 stop) receipt={probe_receipt}; physical direction/stop still require observation.')
    if not passed:raise SystemExit('Probe acceptance incomplete: inspect the archived log; do not increase output or repeat blindly.')
