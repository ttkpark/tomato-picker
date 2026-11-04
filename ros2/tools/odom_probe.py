#!/usr/bin/env python3
"""엔코더 없이 몸체 움직임을 얼마나 아나 — 펄스마다 Astra 전후 두 장 ICP + IMU 회전·진동.
    ~/lerobot/.venv/bin/python ros2/tools/odom_probe.py      (조작대가 주행 포트를 안 쥔 때)
결과 해석 = docs/space-map.md "엔코더 없는 몸체 움직임".  ⚠ 바퀴가 움직인다 — 정면 70cm 안이면 전진은 건너뛴다."""
import sys, os, time, json, math
import numpy as np
R = os.path.expanduser("~/tomato-picker")
sys.path[:0] = [os.path.join(R, "src"), os.path.join(R, "ros2/tools")]
import space_map as sm
from base_nudge import open_link, drive
m = sm.mount()

def imu():
    try: return json.load(open("/dev/shm/imu.json"))
    except Exception: return {}

def shot(after):
    d, meta = sm.frame(after, timeout=10)
    return sm.flat(sm.cloud(d, meta, m), m, 900), d, meta

def front_clear(d, meta):
    h, w = d.shape; c = d[h//3:2*h//3, w//3:2*w//3].astype(float) * meta["depth_scale_mm"]
    c = c[c > 0]
    return (np.percentile(c, 10) if c.size > 200 else 0), meta["valid_frac"]

link = open_link()
if link is None: sys.exit(1)
time.sleep(1.0)
TESTS = [("정지(잡음)", 0, 0, 0, 0.8), ("후진 150", -150, 0, 0, 0.8), ("전진 150", 150, 0, 0, 0.8),
         ("좌 150", 0, 150, 0, 0.8), ("우 150", 0, -150, 0, 0.8),
         ("반시계 200", 0, 0, 200, 1.0), ("시계 200", 0, 0, -200, 1.0),
         ("전진 80(문턱 아래)", 80, 0, 0, 0.8)]
rows = []
try:
    for rep in range(2):
        for name, vx, vy, w, secs in TESTS:
            src0, d0, meta0 = shot(time.time())
            if vx > 0:
                p10, vf = front_clear(d0, meta0)
                if vf < 0.08 or p10 < 700:
                    print(f"  [건너뜀] {name}: 정면 p10={p10:.0f}mm valid={vf:.2f}"); continue
            y0 = imu().get("yaw_deg", 0.0)
            rates, still = [], []
            t0 = time.time()
            import threading
            th = threading.Thread(target=drive, args=(link, vx, vy, w, secs)); th.start()
            while th.is_alive() or time.time() - t0 < secs + 0.8:
                r = imu(); rates.append(abs(r.get("rate_dps", 0))); still.append(r.get("still", True)); time.sleep(0.02)
            th.join()
            y1 = imu().get("yaw_deg", 0.0); dy = (y1 - y0 + 180) % 360 - 180
            src1, d1, meta1 = shot(time.time() + 0.2)
            pose, frac, res = sm.icp2d(src1, src0, [0.0, 0.0, math.radians(dy)])
            moved_imu = (not all(still)) or max(rates) > 3
            row = dict(rep=rep, name=name, icp_dx=pose[0]*1000, icp_dy=pose[1]*1000, icp_dyaw=math.degrees(pose[2]),
                       imu_dyaw=dy, max_rate=max(rates), moved_imu=moved_imu, frac=frac, res=res*1000)
            rows.append(row)
            print(f"{rep} {name:14s} ICP 앞{row['icp_dx']:+6.0f} 왼{row['icp_dy']:+6.0f}mm 회전{row['icp_dyaw']:+6.1f}° (짝{frac:.2f} 잔차{row['res']:.0f}mm) | IMU 회전{dy:+6.1f}° 최대{max(rates):5.1f}°/s 움직임={moved_imu}")
            time.sleep(0.5)
finally:
    link.stop(); link.close()
