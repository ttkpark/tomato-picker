#!/usr/bin/env python3
"""**잡기 반복** — 노란 띠를 스스로 찾아 물고, 놓고, 다시 걸리면 또 문다.

    ~/lerobot/.venv/bin/python ros2/tools/grasp_repeat.py                 # 10회
    ~/lerobot/.venv/bin/python ros2/tools/grasp_repeat.py --trials 3 --no-rehang   # 사람 없이 그대로 3회

한 회 = 시작 자세 → 띠가 2초 가만히 보이면 **줄기 쪽으로 몸을 돌리고**(어깨, 넘으면 회전판) 띠를 보며 높이·거리(≈105mm)를 맞춘 뒤 그 자리를 클릭으로 → `stem_grasp`(조작대 [잡으러 간다]와 같은 인자,
걸음만 12mm) → 물었으면 --hold초 들고 있다가 놓기(열기 → 왼쪽 12 → 뒤로) → 시작 자세.
물었던 다음 회는 **띠가 한 번 사라졌다 다시 보여야**(사람이 토마토를 뗐다가 다시 걸어야) 시작한다. 빗나간 뒤엔 곧장 다시.

⚠ 띠 = 줄기에 감은 노란 테이프(grip_uv_measure.band_uvz). 띠가 없으면 시작하지 않는다.
⚠ 기록: ~/grasp_repeat/log.jsonl (한 줄 = 한 회) + 회마다 시작·끝 사진.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

import numpy as np

try:
    sys.stdout.reconfigure(line_buffering=True)      # 조작대 로그가 실시간이어야 멈춘 자리가 보인다
except Exception:                                      # noqa: BLE001
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import grip_uv_measure as gm                                # noqa: E402

PY = sys.executable
OUT = os.path.expanduser("~/grasp_repeat")
CLICK = os.path.expanduser("~/click_target.json")
# 11/02 22회(물었다)의 시작 자세 — 띠까지 ≈100mm, 문 높이·피치
START = "-7.1,75.5,132.2,-28.8,7.1"


STEP_TIMEOUT = 240.0     # 한 도구 호출 상한(초) — 11/03 20회 시험 4회차에서 어딘가 20분 멈췄다(어디인지 로그가 버퍼라 못 봄)


def run(name, *a):
    """도구를 부른다 — 출력은 **그 자리에서** 흘리고(멈추면 어디서 멈췄는지 보이게), 상한을 넘으면 죽인다(rc 124)."""
    cmd = [PY, "-u", os.path.join(HERE, name), *a]
    out = []
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    t_end = time.time() + STEP_TIMEOUT
    try:
        for line in p.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            out.append(line)
            if time.time() > t_end:
                raise TimeoutError
        p.wait(timeout=max(1.0, t_end - time.time()))
    except (TimeoutError, subprocess.TimeoutExpired):
        p.kill()
        p.wait()
        print("  ⚠ %s가 %.0f초 넘게 안 끝나 죽였다" % (name, STEP_TIMEOUT))
        return 124, "".join(out)
    return p.returncode, "".join(out)


REF = "auto"      # band | stem | auto(띠 있으면 띠, 없으면 토마토 꼭대기 기준)


def frame():
    import cv2
    try:
        bgr = cv2.imread(gm.COLOR)
        sc = float(json.load(open(gm.META)).get("depth_scale_mm", 1.0))
        dep = np.load(gm.DEPTH).astype(float) * sc
    except Exception:                                      # noqa: BLE001
        return None
    if bgr is None or dep.shape[:2] != bgr.shape[:2]:
        return None
    return bgr, dep


def stem_ref(bgr, dep, cfg=None, intr=None):
    """맨 줄기(띠 없음)의 기준점 — 토마토 꼭대기에서 (옆 stem_du_mm, 위 stem_dz_mm, 깊이 stem_dd_mm)만큼 옮긴 자리.
    깊이도 옮긴다: 카메라가 숙여 보니 토마토 윗가장자리가 줄기보다 뒤다(11/03 띠 151 vs 꼭대기 168mm).
    두 값은 ~/grip_uv.json에 있고 띠를 걸어 둔 채 `--calib-stem`으로 잰다(줄기 길이가 정하는 물리값).
    안 재 놨으면 None — 맨 줄기는 쓰지 않는다(엉뚱한 데를 자신 있게 무는 것보다 낫다)."""
    t = gm.tomato_top_uvz(bgr, dep)
    if t is None:
        return None
    try:
        cfg = cfg or json.load(open(gm.GRIP_UV))
        intr = intr or json.load(open(gm.META))["intrinsics"]
        du, dz, dd = float(cfg["stem_du_mm"]), float(cfg["stem_dz_mm"]), float(cfg.get("stem_dd_mm", 0.0))
        fx, fy = float(intr["fx"]), float(intr["fy"])
    except (OSError, ValueError, KeyError):
        return None
    u, vtop, z = t
    return u + du * fx / z, vtop - dz * 0.8 * fy / z, z + dd


def band():
    """지금 프레임의 기준점 (u, v, z) 또는 None — 띠, 또는(REF) 맨 줄기."""
    f = frame()
    if f is None:
        return None
    b = gm.band_uvz(*f) if REF != "stem" else None
    if b is None and REF != "band":
        b = stem_ref(*f)
    return b


def calib_stem(frames=15):
    """띠를 걸어 둔 채, 띠 ↔ 토마토 꼭대기의 차이를 mm로 재서 ~/grip_uv.json에 적는다(stem_du_mm, stem_dz_mm)."""
    k = json.load(open(gm.META))["intrinsics"]
    fx, fy = float(k["fx"]), float(k["fy"])
    dus, dzs, dds = [], [], []
    for _ in range(frames):
        f = frame()
        b = gm.band_uvz(*f) if f else None
        t = gm.tomato_top_uvz(*f) if f else None
        if b and t:
            dus.append((b[0] - t[0]) * b[2] / fx)
            dzs.append((t[1] - b[1]) * b[2] / (0.8 * fy))
            dds.append(b[2] - t[2])
            print("  띠 (%.0f,%.0f) %.0fmm · 토마토 꼭대기 (%.0f,%.0f) %.0fmm → 옆 %+.1f 위 %+.1f 깊이 %+.1f mm"
                  % (b[0], b[1], b[2], t[0], t[1], t[2], dus[-1], dzs[-1], dds[-1]))
        time.sleep(0.4)
    if len(dus) < 5:
        print("띠와 토마토가 같이 보이는 프레임이 %d장뿐 — 안 적는다" % len(dus))
        return 1
    du, dz, dd = float(np.median(dus)), float(np.median(dzs)), float(np.median(dds))
    print("중앙값: 옆 %+.1f 위 %+.1f 깊이 %+.1f mm (n=%d, 퍼짐 %.1f/%.1f/%.1f)"
          % (du, dz, dd, len(dus), np.std(dus), np.std(dzs), np.std(dds)))
    cfg = json.load(open(gm.GRIP_UV))
    cfg["stem_du_mm"], cfg["stem_dz_mm"], cfg["stem_dd_mm"] = round(du, 1), round(dz, 1), round(dd, 1)
    json.dump(cfg, open(gm.GRIP_UV, "w"), indent=1)
    print("~/grip_uv.json에 적었다")
    return 0


def wait_band(need_gone, timeout, steady_s=2.0, tol_px=8.0, last=None, moved_px=60.0, blink_s=1.0):
    """띠가 steady_s초 동안 tol_px 안에서 가만히 보이면 그 자리.
    need_gone이면 먼저 '다시 걸렸다'를 봐야 한다 — 한 번 안 보였거나, 지난 회 자리(last)에서 moved_px 넘게 옮겨졌거나.
    ⚠ moved_px 60: 놓을 때 줄기가 회마다 10~40px 밀려 20이면 사람 없이 다시 걸린 것으로 읽었다(11/02 2차).
    ⚠ 한두 프레임 못 찾는 건(blink_s 안) 가만히 재기를 끊지 않는다. timeout ≤ 0이면 끝없이 기다린다."""
    t_end = time.time() + timeout if timeout > 0 else float("inf")
    gone = not need_gone
    since, ref, miss_t = None, None, None
    while time.time() < t_end:
        b = band()
        now = time.time()
        if b is None:
            miss_t = miss_t or now
            if now - miss_t >= blink_s:
                if not gone:
                    print("  띠가 사라졌다 — 다시 걸리길 기다린다")
                gone, since, ref = True, None, None
        else:
            miss_t = None
            if not gone and last is not None and abs(b[0] - last[0]) + abs(b[1] - last[1]) > moved_px:
                print("  띠가 옮겨졌다 — 다시 걸린 것으로 본다")
                gone = True
            if gone:
                if ref is None or abs(b[0] - ref[0]) > tol_px or abs(b[1] - ref[1]) > tol_px:
                    since, ref = now, b      # 움직이는 중(사람 손) — 처음부터 다시 잰다
                elif now - since >= steady_s:
                    return b
        time.sleep(0.3)
    return None


def heading_turn(c, du, z, geom, fx, stop_z):
    """줄기를 집게의 곧은 길(접근축의 수평 성분) 위에 놓으려면 base 각을 얼마 돌려야 하나 → (pan 관절 변화°, base 각 변화°).

    줄기 자리 S = TCP + 앞으로 (z−stop_z)/0.8 + 옆으로 L(= du·z/fx, du>0이면 화면 오른쪽). 5축이라 곧은 길은
    어깨 축에서 나가는 반지름 방향이다 — 그래서 옆 평행이동(시작 맞춤)이 아니라 **회전**으로 맞춘다(11/02 사람 제안)."""
    import math
    import grasp_probe as gp
    from tomato_picker.hardware import kinematics as kin
    p = kin.forward(c, geom)
    h = gp.approach_dir(c, geom) * np.array([1.0, 1.0, 0.0])
    h = h[:2] / max(float(np.linalg.norm(h)), 1e-6)
    left = np.array([h[1], -h[0]])                  # 화면 왼쪽(tool_jog --left와 같은 정의, 실측)
    L = du * z / fx
    S = np.array([p.x, p.y]) + h * max(0.0, (z - stop_z) / 0.8) - left * L
    dphi = math.atan2(S[1], S[0]) - math.atan2(p.y, p.x)
    dphi = (dphi + math.pi) % (2 * math.pi) - math.pi
    e = dict(c)
    e["shoulder_pan"] += 1.0
    p1 = kin.forward(e, geom)
    per = math.atan2(p1.y, p1.x) - math.atan2(p.y, p.x)     # pan 1°가 돌리는 base 각(rad)
    return (dphi / per if abs(per) > 1e-6 else 0.0), math.degrees(dphi)


def face_stem(b, stop_z, tol_px=15.0, iters=3):
    """줄기 쪽으로 몸을 돌린다 — 어깨(pan)로, 어깨 범위를 넘으면 회전판으로. 돌린 뒤 다시 본 띠를 돌려준다.
    회전판 부호는 한 번 돌려 보고 화면 어긋남이 커지면 뒤집는다(판 축이 어깨 축과 달라 근사다)."""
    sys.path.insert(0, os.path.join(HERE, "..", "..", "src"))
    sys.path.insert(0, os.path.join(HERE, "..", "src", "tomato_bridge"))
    import arm_calib
    from stem_grasp import uv_at_depth
    from tomato_bridge.follower_io import FollowerIO
    from tomato_picker.config import BASE_YAW_LIMIT_DEG
    cal = arm_calib.Calib()
    try:
        by_z = json.load(open(os.path.expanduser("~/grip_uv.json")))["by_z"]
        fx = float(json.load(open(gm.META))["intrinsics"]["fx"])
    except Exception as exc:                               # noqa: BLE001
        print("  몸 돌리기 건너뜀(표·내부값 없음: %s)" % exc)
        return b
    yaw_sign = 1.0
    for _ in range(iters):
        u, v, z = b
        du = u - uv_at_depth(by_z, z)[0]
        if abs(du) <= tol_px:
            break
        io = FollowerIO(hold_torque=True)
        try:
            c = cal.to_deg(io.read())
            yaw = io.read_yaw()
        finally:
            io.hold_close()
        dpan, dphi = heading_turn(c, du, z, cal.geom, fx, stop_z)
        cand = dict(c)
        cand["shoulder_pan"] += dpan
        if abs(dpan) < 0.3:
            break
        if cal.legal(cand, c)[0]:
            print("  몸 돌리기: 줄기가 곧은 길에서 %+.0fpx → 어깨 %+.1f°" % (du, dpan))
            run("arm_stage.py", "--set", "shoulder_pan=%.1f" % cand["shoulder_pan"])
        else:
            ny = yaw + yaw_sign * dphi
            if abs(ny) > BASE_YAW_LIMIT_DEG:
                print("  몸 돌리기: 어깨도 회전판(%.1f° → %.1f°)도 범위 밖 — 로봇을 옮겨야 한다" % (yaw, ny))
                break
            print("  몸 돌리기: 어깨 범위 밖 → 회전판 %.1f° → %.1f°" % (yaw, ny))
            run("base_yaw.py", "%.1f" % ny)
        nb = wait_band(need_gone=False, timeout=15.0)
        if nb is None:
            print("  돌리고 나니 띠가 안 보인다")
            return None
        if not cal.legal(cand, c)[0] and abs(nb[0] - uv_at_depth(by_z, nb[2])[0]) > abs(du):
            yaw_sign = -yaw_sign                  # 회전판을 반대로 돌렸다 — 다음엔 뒤집는다
        b = nb
    return b


def close_in(b, stop_z, near_z=105.0, tol_px=8.0, iters=6):     # 위아래 15px면 ≈5mm 높게 남아 빗나갔다(11/02 옮긴 자리 2회)
    """노란 띠로 다가간다 — 위아래는 높이로, 거리는 수평 전진으로 깊이 ≈near_z까지. 그 뒤는 stem_grasp의 짧은 접근이
    한다(11/02 78~104mm에서 10/10). ⚠ stem_grasp의 클릭 추적은 큰 맞춤(옆 40·위 17mm) 뒤에 줄기를 놓쳤다(0/10).
    띠 검출은 매 프레임 새로 하니 놓칠 게 없다. 위아래: 화면 1px ≈ z/(0.8·fy) mm(카메라가 숙여 있어 fy/z보다 작다)."""
    from stem_grasp import uv_at_depth
    try:
        by_z = json.load(open(os.path.expanduser("~/grip_uv.json")))["by_z"]
        fy = float(json.load(open(gm.META))["intrinsics"]["fy"])
    except Exception as exc:                               # noqa: BLE001
        print("  다가가기 건너뜀(%s)" % exc)
        return b
    for i in range(iters):
        b = face_stem(b, stop_z)
        if b is None:
            return None
        u, v, z = b
        dv = v - uv_at_depth(by_z, z)[1]
        # ⚠ 게인 0.6 — 1.0이면 +29 → −9 → +14px로 진동하다 끝났다(11/02~03 빈손 4회 전부 이것).
        # 마지막 반복만 1.0 — 0.6으로는 −13px가 남은 채 횟수를 다 써 빗나갔다(11/03 20회 시험의 빈손 1회).
        gain = 1.0 if i == iters - 1 else 0.6
        dz = float(np.clip(-gain * dv * z / (0.8 * fy), -30.0, 30.0)) if abs(dv) > tol_px else 0.0
        fwd = float(np.clip((z - near_z) / 0.8, 0.0, 60.0)) if z > near_z + 10.0 else 0.0
        if not dz and not fwd:
            break
        print("  다가가기: 띠 %.0fmm, 겨냥점보다 %+.0fpx 아래 → 높이 %+.0f · 앞으로 %.0fmm" % (z, dv, dz, fwd))
        run("tool_jog.py", "--dz", "%.1f" % dz, "--horiz", "%.1f" % fwd)
        b = wait_band(need_gone=False, timeout=15.0, steady_s=1.0)
        if b is None:
            print("  다가가니 띠가 안 보인다")
            return None
    return b


def look_around(pans=(8.0, 20.0, -20.0, -35.0, 35.0)):
    """시작 자세에서 띠가 안 보이면 어깨만 돌려 가며 찾는다 — 로봇을 옮기면 줄기가 화면 밖으로 나간다(11/02 자리 4)."""
    if band() is not None:
        return True
    for p in pans:
        rc, _ = run("arm_stage.py", "--set", "shoulder_pan=%.1f" % p)
        time.sleep(0.5)
        if rc == 0 and band() is not None:
            print("  어깨 %.0f°에서 띠를 찾았다" % p)
            return True
    print("  어깨를 돌려 봐도 띠가 안 보인다")
    return False


def snap(tag):
    try:
        shutil.copy(gm.COLOR, os.path.join(OUT, "%s_%s.jpg" % (time.strftime("%m%d-%H%M%S"), tag)))
    except OSError:
        pass


def selftest():
    global band
    seq = iter([(400, 300, 100)] * 3 + [None] * 6 + [(300, 200, 90), (420, 310, 100)] + [(421, 311, 100)] * 30)
    clock = [0.0]
    real_band, real_time, real_sleep = band, time.time, time.sleep
    band = lambda: next(seq)                                       # noqa: E731
    time.time = lambda: clock[0]
    time.sleep = lambda s: clock.__setitem__(0, clock[0] + s)
    try:
        b = wait_band(need_gone=True, timeout=60)
        assert b is not None and abs(b[0] - 421) < 1, b          # 사라짐 → 움직임(손) 무시 → 가만히 2초
        seq = iter([None] * 300)
        assert wait_band(need_gone=False, timeout=5) is None
        seq = iter([(400, 300, 100)] * 3 + [None] + [(400, 300, 100)] * 30)   # 한 프레임 깜빡임은 무시
        assert wait_band(need_gone=False, timeout=60) is not None
        seq = iter([(470, 300, 100)] * 30)                                     # 안 사라졌어도 옮겨 걸었으면 된다
        assert wait_band(need_gone=True, timeout=60, last=(400, 300, 100)) is not None
        seq = iter([(400, 300, 100)] * 300)                                    # 그대로면 기다린다
        assert wait_band(need_gone=True, timeout=30, last=(400, 300, 100)) is None
        # 맨 줄기 기준: 토마토 꼭대기(400,300)@100mm, 옆 +10·위 +20mm, fx=fy=400 → u 440, v 300-64
        real_top = gm.tomato_top_uvz
        gm.tomato_top_uvz = lambda *_: (400.0, 300.0, 100.0)
        try:
            r = stem_ref(None, None, {"stem_du_mm": 10, "stem_dz_mm": 20, "stem_dd_mm": -5}, {"fx": 400, "fy": 400})
            assert stem_ref(None, None, {}, {"fx": 400, "fy": 400}) is None     # 안 재 놨으면 안 쓴다
        finally:
            gm.tomato_top_uvz = real_top
        assert r and abs(r[0] - 440) < 1e-6 and abs(r[1] - 236) < 1e-6 and r[2] == 95, r
    finally:
        band, time.time, time.sleep = real_band, real_time, real_sleep
    print("selftest OK")


def main():
    global REF
    if "--selftest" in sys.argv:
        return selftest()
    if "--calib-stem" in sys.argv:
        return calib_stem()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--start", default=START, help="시작 자세(관절 5개, 도)")
    ap.add_argument("--stop-z", type=float, default=75.0)
    ap.add_argument("--adv", type=float, default=12.0, help="나아가는 한 걸음(mm) — 8이면 느리다")
    ap.add_argument("--hold", type=float, default=3.0, help="물었으면 이만큼 들고 있다가 놓는다(초)")
    ap.add_argument("--wait", type=float, default=0.0, help="띠를 기다리는 최대 시간(초, 0=끝없이 — 사람이 거는 데 몇 분 걸린다)")
    ap.add_argument("--no-rehang", action="store_true", help="띠가 사라졌다 다시 보이길 안 기다린다(사람 없이 반복)")
    ap.add_argument("--ref", choices=("auto", "band", "stem"), default="auto",
                    help="기준점: band=노란 띠만, stem=토마토 꼭대기(맨 줄기, --calib-stem 먼저), auto=띠 없으면 stem")
    a = ap.parse_args()
    REF = a.ref
    os.makedirs(OUT, exist_ok=True)
    a.trials = max(1, a.trials)
    # 물고 있을 수도 있다 — 놓지 않고 시작 자세로 가면 토마토를 끌고 간다
    run("tool_jog.py", "--grip", "78", "--left", "12")
    run("tool_jog.py", "--horiz", "-35")
    held_n, last_held, last_band = 0, False, None
    for n in range(1, a.trials + 1):
        print("\n=== %d/%d ===" % (n, a.trials))
        rc, _ = run("arm_stage.py", "--target=" + a.start)
        if rc:
            print("시작 자세로 못 갔다 — 멈춘다")
            return 1
        look_around()
        # 물었던 다음 회만 '뗐다 다시 걸기'를 기다린다 — 빗나간 뒤엔 그대로 걸려 있으니 곧장 다시 문다
        b = wait_band(need_gone=(last_held and not a.no_rehang), timeout=a.wait, last=last_band)
        if b is None:
            print("띠가 %.0f초 안에 가만히 보이지 않았다 — 멈춘다" % a.wait)
            break
        b = close_in(b, a.stop_z)
        if b is None:
            continue
        u, v, z = b
        last_band = b
        print("  띠 (%.0f, %.0f) %.0fmm — 문다" % (u, v, z))
        json.dump({"u": int(round(u)), "v": int(round(v)), "z": z, "when": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "by": "grasp_repeat"}, open(CLICK, "w"), indent=1)
        snap("%02d_start" % n)
        t0 = time.time()
        rc, out = run("stem_grasp.py", "--aim", "click", "--steps", "16", "--adv", "%.0f" % a.adv,
                      "--max-turn", "2", "--gain", "0.35", "--tol", "12", "--rejacobian", "6",
                      "--stop-z", "%.0f" % a.stop_z, "--max-adv", "0", "--max-dz", "25",
                      "--skip-aim-px", "50", "--trust-first-z")
        held = rc == 0 and "**물었다**" in out
        secs = time.time() - t0
        snap("%02d_%s" % (n, "held" if held else "miss"))
        held_n += held
        last_held = held
        print("  → %s (%.0f초) · 누적 %d/%d" % ("물었다" if held else "못 물었다", secs, held_n, n))
        with open(os.path.join(OUT, "log.jsonl"), "a") as f:
            f.write(json.dumps({"when": time.strftime("%Y-%m-%d %H:%M:%S"), "n": n, "held": held,
                                "secs": round(secs, 1), "band": [round(u), round(v), round(z)]}) + "\n")
        if held:
            time.sleep(a.hold)
        # 놓기 — 조작대 [놓기]와 같은 순서(열기 → 고정 턱 떼기 → 뒤로). 빈손이어도 연다.
        run("tool_jog.py", "--grip", "78", "--left", "12")
        run("tool_jog.py", "--horiz", "-35")
        # 놓은 뒤 확인 — 11/03 9회째 놓다가 토마토가 고리에서 떨어졌는데 사진이 없어 원인을 못 봤다
        if held and wait_band(need_gone=False, timeout=5.0, steady_s=1.0) is None:
            print("  ⚠ 놓은 뒤 띠가 안 보인다 — 토마토가 떨어졌을 수 있다")
            snap("%02d_fell" % n)
    print("\n끝: %d회 중 %d회 물었다" % (n, held_n))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
