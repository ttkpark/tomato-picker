#!/usr/bin/env python3
# controller_drive.py — 게임패드 통합 드라이버 (Switch Pro/hid-nintendo & Xbox360/xpad 양쪽 지원, 안전강화).
#   ★ 데드맨: LT(왼쪽 트리거/ZL)를 "누르는 동안만" 주행. 놓으면 즉시 정지 → 드리프트/latch 폭주 불가.
#   주행: 왼스틱=전진/평행, 오른스틱 X=회전, D패드=디지털 이동, RT(ZR)=부스트, B=즉시정지
#   팔  : LB(L)=이전 프리셋, RB(R)=다음 프리셋 재생 (리더 없이 팔로워만, 저장자세로 보간)
# 실행: ~/lerobot/.venv/bin/python ~/controller_drive.py [--no-arm] [--dry] [--always]
import evdev, sys, time, glob, json, os, select
from evdev import ecodes

DRY      = '--dry'    in sys.argv
NO_ARM   = '--no-arm' in sys.argv
ALWAYS   = '--always' in sys.argv
DEADZONE = 7000
MAXV     = 180
BOOSTV   = 255
DIG      = 140
TRIG_ON  = 60          # 아날로그 트리거(LT/RT) 눌림 판정
RATE     = 0.05
BAUD     = 115200
PRESET_FILE = os.path.expanduser('~/arm_presets.json')

def find_pad():
    for p in evdev.list_devices():
        d = evdev.InputDevice(p)
        n = d.name
        if 'IMU' in n:
            continue
        if ('Pro Controller' in n) or ('X-Box' in n) or ('Xbox' in n) or ('X360' in n):
            return d
    return None

def find_motor():
    for pat in ('/dev/serial/by-id/*USB_Serial*', '/dev/ttyUSB*'):
        g = sorted(glob.glob(pat))
        if g:
            return g[0]
    return None

pad = find_pad()
if not pad:
    print('게임패드 없음 (Pro Controller/Xbox360 연결 확인)'); sys.exit(1)
print('pad :', pad.path, '|', pad.name)

ser = None
_last_open = 0.0
if not DRY:
    import serial as _pyserial

def open_motor():
    """모터보드 시리얼 (재)연결. CH340 USB가 끊겨도 자동 복구(1.5s 쿨다운)."""
    global ser, _last_open
    if DRY:
        return
    now = time.time()
    if now - _last_open < 1.5:
        return
    _last_open = now
    try:
        if ser:
            try: ser.close()
            except Exception: pass
            ser = None
        p = find_motor()
        if p:
            # exclusive=True: 대시보드(voice 서비스의 MotorLink)도 같은 포트를
            # 상시 점유하므로, 둘이 동시에 쓰면 지령이 섞인다. 먼저 잡은 쪽이
            # 이기고 나중 쪽은 "busy"로 명확히 실패하게 한다.
            ser = _pyserial.Serial(p, BAUD, timeout=0.1, exclusive=True)
            print('motor (re)opened:', p)
    except Exception as e:
        print('motor open 실패:', e)
        print('  ↳ "busy"라면 대시보드(tomato-voice)가 이미 모터보드를 쓰는 중이다.')
        print('    둘 중 하나만 켤 것:  sudo systemctl stop tomato-voice   (또는 controller-drive)')
        ser = None

open_motor()
if not DRY and ser is None:
    print('모터보드 아직 없음 — loop에서 재시도')

follower = None
presets = {}
if not NO_ARM:
    try:
        from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
        follower = SOFollower(SOFollowerRobotConfig(port='/dev/ttyACM0', id='tomato_follower'))
        print('arm 연결 중...'); follower.connect(calibrate=False); follower.bus.disable_torque()
        raw = json.load(open(PRESET_FILE))
        # "__meta__"(이름·높이앵커)는 자세가 아니다 — 숫자 슬롯만 골라낸다.
        presets = {k: v for k, v in raw.items() if k.isdigit() and v}
        print('arm OK. 프리셋:', sorted(presets.keys(), key=int))
    except Exception as e:
        print('arm 초기화 실패 → 주행만:', e); follower = None

def pose_only(d):
    return {k: float(v) for k, v in d.items() if k.endswith('.pos')}

def play_preset(n, secs=1.5, fps=50):
    if follower is None:
        print('[preset %s] arm 없음' % n); return
    t = presets.get(str(n))
    if not t:
        print('[preset %s] 비어있음' % n); return
    send('S')
    print('[preset %s] 재생...' % n)
    follower.bus.enable_torque()
    cur = pose_only(follower.get_observation())
    steps = max(2, int(secs * fps))
    for s in range(1, steps + 1):
        a = {k: cur.get(k, t[k]) + (t[k] - cur.get(k, t[k])) * s / steps for k in t}
        follower.send_action(a); time.sleep(secs / steps)
    print('[preset %s] 완료' % n)

def framed(cmd):
    """펌웨어 v2의 XOR 체크섬 형식 "<payload>*HH".

    v2는 올바른 체크섬을 한 번 받으면 그 뒤로 무체크섬 V를 거부한다(strict 모드).
    대시보드(MotorLink)가 체크섬을 쓰므로 여기서도 반드시 붙여야 한다 —
    안 붙이면 게임패드 주행만 조용히 먹히지 않는다."""
    crc = 0
    for ch in cmd:
        crc ^= ord(ch)
    return '%s*%02X\n' % (cmd, crc)


def send(cmd):
    global ser
    if DRY:
        if cmd != 'V 0 0 0': print(cmd)
        return
    if ser is None:
        open_motor()
        if ser is None:
            return
    try:
        ser.write(framed(cmd).encode())
    except Exception as e:
        print('motor write 실패 → 재연결:', e)
        try: ser.close()
        except Exception: pass
        ser = None
        open_motor()

# 상태
raw = {ecodes.ABS_X: 0, ecodes.ABS_Y: 0, ecodes.ABS_RX: 0}
az = arz = 0            # LT / RT 아날로그값(X360)
zl_btn = zr_btn = False # ZL / ZR 디지털(Switch)
hatx = haty = 0
pidx = -1        # 채워진 슬롯 목록의 인덱스. 첫 RB에서 0(첫 슬롯)이 되게 -1로 시작

def pump():
    global az, arz, zl_btn, zr_btn, hatx, haty, pidx
    r, _, _ = select.select([pad], [], [], RATE)
    for d in r:
        try:
            for e in d.read():
                if e.type == ecodes.EV_ABS:
                    if   e.code in raw:              raw[e.code] = e.value
                    elif e.code == ecodes.ABS_Z:     az = e.value
                    elif e.code == ecodes.ABS_RZ:    arz = e.value
                    elif e.code == ecodes.ABS_HAT0X: hatx = e.value
                    elif e.code == ecodes.ABS_HAT0Y: haty = e.value
                elif e.type == ecodes.EV_KEY:
                    if   e.code == ecodes.BTN_TL2:   zl_btn = bool(e.value)
                    elif e.code == ecodes.BTN_TR2:   zr_btn = bool(e.value)
                    elif e.value == 1:
                        # LB/RB = 저장된 슬롯 목록을 순환(0~19 중 **채워진 것만**).
                        # 예전엔 1~4 고정이라 슬롯을 늘려도 못 돌았다.
                        slots = sorted((int(k) for k in presets), key=int)
                        if   e.code == ecodes.BTN_TR and slots:
                            send('S'); pidx = (pidx + 1) % len(slots); play_preset(slots[pidx])
                        elif e.code == ecodes.BTN_TL and slots:
                            send('S'); pidx = (pidx - 1) % len(slots); play_preset(slots[pidx])
                        elif e.code == ecodes.BTN_EAST: send('S')
        except BlockingIOError:
            pass

# 중립 보정
print('중립 보정: 스틱에서 손 떼세요... (1초)')
t0 = time.time()
while time.time() - t0 < 1.0:
    pump()
center = dict(raw)
print('center =', {'X': center[ecodes.ABS_X], 'Y': center[ecodes.ABS_Y], 'RX': center[ecodes.ABS_RX]})

def scale(code, mv):
    v = raw[code] - center[code]
    if abs(v) < DEADZONE:
        return 0
    s = min(1.0, (abs(v) - DEADZONE) / (32767 - DEADZONE))
    o = int(s * mv)
    return o if v > 0 else -o

mode = 'ALWAYS(데드맨 없음)' if ALWAYS else 'LT(ZL) 누르는 동안만 주행'
print('--- 준비완료 [%s]. 왼스틱=이동, 오른스틱=회전, D패드=디지털, RT=부스트, B=정지, LB/RB=프리셋 ---' % mode)
last = 0.0
try:
    while True:
        pump()
        now = time.time()
        if now - last >= RATE:
            last = now
            enable = ALWAYS or zl_btn or (az > TRIG_ON)
            boost  = zr_btn or (arz > TRIG_ON)
            if not enable:
                send('V 0 0 0')
                continue
            mv = BOOSTV if boost else MAXV
            vx = -scale(ecodes.ABS_Y, mv)
            vy =  scale(ecodes.ABS_X, mv)
            w  =  scale(ecodes.ABS_RX, mv)
            if vx == 0 and haty != 0: vx = -haty * DIG
            if vy == 0 and hatx != 0: vy =  hatx * DIG
            send('V %d %d %d' % (vx, vy, w))
except KeyboardInterrupt:
    pass
finally:
    send('S')
    if follower is not None:
        try: follower.bus.disable_torque(); follower.disconnect()
        except Exception: pass
    print('\n정지(S)·정리 완료.')
