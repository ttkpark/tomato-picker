#!/usr/bin/env python3
# jetson_pad_probe.py — evdev로 Switch Duocon 게임패드를 찾아 축/버튼을 실시간 매핑.
import evdev, sys, time, select
from evdev import ecodes

def codename(table, code):
    n = table.get(code, code)
    if isinstance(n, list): n = n[0]
    return n

def is_gamepad(d):
    caps = d.capabilities()
    keys = caps.get(ecodes.EV_KEY, [])
    abss = [c for c, _ in caps.get(ecodes.EV_ABS, [])]
    has_btn = (ecodes.BTN_GAMEPAD in keys) or (ecodes.BTN_SOUTH in keys) or (ecodes.BTN_A in keys)
    has_abs = ecodes.ABS_X in abss
    return has_btn and has_abs

dev = None
for p in evdev.list_devices():
    d = evdev.InputDevice(p)
    if 'Consumer' in d.name:
        continue
    if is_gamepad(d):
        dev = d; break
if dev is None:
    print('게임패드(축+버튼) 없음. 현재 장치:')
    for p in evdev.list_devices():
        print('  ', p, evdev.InputDevice(p).name)
    sys.exit(1)

print('USING', dev.path, '|', dev.name)
caps = dev.capabilities()
if ecodes.EV_ABS in caps:
    print('=== ABS 축 ===')
    for code, ai in caps[ecodes.EV_ABS]:
        print('  ABS %-14s code=%d min=%d max=%d' % (codename(ecodes.ABS, code), code, ai.min, ai.max))
if ecodes.EV_KEY in caps:
    btns = [codename(ecodes.BTN, c) if c in ecodes.BTN else codename(ecodes.KEY, c) for c in caps[ecodes.EV_KEY]]
    print('=== 버튼 %d개 ===' % len(btns))
    print('  ', ', '.join(str(b) for b in btns)[:500])

dur = int(sys.argv[1]) if len(sys.argv) > 1 else 25
print('--- %ds: 스틱/버튼 움직여보세요 (변화만 출력) ---' % dur)
last = {}
end = time.time() + dur
while time.time() < end:
    r, _, _ = select.select([dev], [], [], 0.5)
    for d in r:
        try:
            for e in d.read():
                if e.type == ecodes.EV_ABS:
                    prev = last.get(e.code)
                    is_hat = e.code in (16, 17)
                    if prev is None or (is_hat and e.value != prev) or (not is_hat and abs(e.value - prev) >= 5):
                        print('ABS %-12s = %d' % (codename(ecodes.ABS, e.code), e.value)); last[e.code] = e.value
                elif e.type == ecodes.EV_KEY and e.value in (0, 1):
                    print('KEY %-14s = %d' % (codename(ecodes.BTN, e.code) if e.code in ecodes.BTN else codename(ecodes.KEY, e.code), e.value))
        except BlockingIOError:
            pass
print('--- probe done ---')
