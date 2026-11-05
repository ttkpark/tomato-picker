#!/usr/bin/env python3
"""**같은 자세로 두 번 찍기** — 띠 있는 사진의 라벨을 띠 없는 진짜 맨 줄기 사진에 그대로 옮긴다.

    # 1) 줄기 정면 자세에서 27자세를 돌며 찍고 관절값을 남긴다(띠 있든 없든 — 먼저 찍는 쪽 tag)
    ~/lerobot/.venv/bin/python ros2/tools/stem_pairs.py capture --tag bare
    # 2) 사람이 띠를 감는다(또는 뗀다). 같은 관절값으로 다시 돌며 찍는다
    ~/lerobot/.venv/bin/python ros2/tools/stem_pairs.py replay --tag band
    # 3) (PC) 띠 사진으로 상자를 만들어 맨 줄기 사진의 라벨로 쓴다
    python ros2/tools/stem_pairs.py label <내려받은 pairs 폴더> data/stem_yolo

왜: 띠를 지운(inpaint) 사진으로만 배운 v1은 띠 없는 실기 사진 12장 중 6장만 찾았다(11/04). 지운 자국과 진짜 줄기가
다르기 때문 — 진짜 맨 줄기 사진에 정확한 라벨이 필요하다. 팔은 같은 관절값으로 ≈0.4° 안에 돌아온다(arm_stage 실측).
⚠ 두 번 찍는 사이에 **토마토·지지대를 건드리지 않는다**(띠만 뗀다). 처짐은 같은 자세면 같다.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
OUT = os.path.expanduser(os.environ.get("STEM_PAIRS_DIR", "~/stem_pairs"))
COLOR = "/dev/shm/d405_color.jpg"
PY = sys.executable

# 줄기 정면 자세에서의 상대 이동 — 앞으로(mm) × 높이(mm) × 어깨(도). 105~150mm, 위아래 ±8mm, 옆 ±2°.
FWD = (0.0, 15.0, 30.0, 45.0)     # 45 = 잡기 직전 거리(≈105mm) — v2는 이 거리가 없어 근접 사진 12장 중 1장만 찾았다
DZ = (-8.0, 0.0, 8.0)
PAN = (-3.0, 0.0, 3.0)
WIDE_PAN = (-12.0, -9.0, -6.0, -3.0, 0.0, 3.0, 6.0, 9.0, 12.0)    # 11/05: v2는 줄기가 가운데서 벗어나면(u 553) 확신 0.03


def grid(wide):
    """(앞, 위, 어깨) 자세 목록. wide = 줄기가 화면 가장자리까지 가게 — 가까이(앞 30 이상)선 집게가 줄기·고리에 닿으니 어깨를 좁힌다."""
    if not wide:
        return [(f, dz, p) for f in FWD for dz in DZ for p in PAN]
    return [(f, dz, p) for f in FWD[:2] for dz in DZ for p in WIDE_PAN] + [(f, 0.0, p) for f in FWD[2:] for p in PAN]


def sh(name, *a):
    r = subprocess.run([PY, os.path.join(HERE, name)] + list(a), capture_output=True, text=True, timeout=240)
    return r.returncode, r.stdout + r.stderr


def joints():
    """지금 관절(도) — 읽은 자리."""
    sys.path.insert(0, os.path.join(HERE, "..", "src", "tomato_bridge"))
    import arm_calib
    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    try:
        d = arm_calib.Calib().to_deg(io.read())
    finally:
        io.hold_close()
    return d


def stage(j):
    order = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
    return sh("arm_stage.py", "--target=" + ",".join("%.2f" % j[k] for k in order))


def snap(path):
    t = time.time()
    while os.path.getmtime(COLOR) < t + 0.6:     # 멈춘 뒤 새 프레임
        time.sleep(0.1)
    time.sleep(0.2)
    shutil.copy(COLOR, path)


def capture(tag, wide=False):
    os.makedirs(OUT, exist_ok=True)
    import grasp_repeat as gr
    # 띠 사진은 띠로, 맨 줄기 사진은 학습 검출기로 정면을 잡는다 — 토마토 꼭대기 기준은 가려지면 38mm 어긋난다(11/05)
    gr.REF = "model" if tag == "bare" else "band"
    b = gr.wait_band(need_gone=False, timeout=20, steady_s=1.0)
    if b is None:
        print("줄기가 안 보인다 — 시작 자세에서 토마토가 보이게 하고 다시")
        return 1
    gr.face_stem(b, 75.0)
    p0 = joints()
    poses = []
    for f, dz, dp in grid(wide):
        stage(p0)
        if f or dz:
            sh("tool_jog.py", "--horiz", "%.1f" % f, "--dz", "%.1f" % dz)
        if dp:
            j = joints()
            sh("arm_stage.py", "--set", "shoulder_pan=%.2f" % (j["shoulder_pan"] + dp))
        j = joints()
        k = len(poses)
        snap(os.path.join(OUT, "%02d_%s.jpg" % (k, tag)))
        poses.append({"k": k, "fwd": f, "dz": dz, "dpan": dp, "joints": j})
        print("  %02d  앞 %+.0f 위 %+.0f 어깨 %+.0f → 찍음" % (k, f, dz, dp))
    json.dump({"p0": p0, "poses": poses, "tag": tag, "when": time.strftime("%Y-%m-%d %H:%M:%S")},
              open(os.path.join(OUT, "poses.json"), "w"), indent=1)
    stage(p0)
    sh("tool_jog.py", "--horiz", "-30", "--dz", "10")
    print("자세 %d개 → %s" % (len(poses), OUT))
    return 0


def replay(tag):
    meta = json.load(open(os.path.join(OUT, "poses.json")))
    for p in meta["poses"]:
        stage(p["joints"])
        snap(os.path.join(OUT, "%02d_%s.jpg" % (p["k"], tag)))
        print("  %02d → 찍음" % p["k"])
    stage(meta["p0"])
    sh("tool_jog.py", "--horiz", "-30", "--dz", "10")
    print("다시 찍기 %d장" % len(meta["poses"]))
    return 0


def label(src, dst, band_tag="band", bare_tag="bare"):
    """띠 사진에서 상자를 내고 같은 번호의 맨 줄기 사진에 붙인다. 띠 사진(지운 것)도 함께 넣는다."""
    import cv2
    import stem_autolabel as al
    n = 0
    for sub in ("images/train", "labels/train"):
        os.makedirs(os.path.join(dst, sub), exist_ok=True)
    for f in sorted(os.listdir(src)):
        if not f.endswith("_%s.jpg" % band_tag):
            continue
        k = f.split("_")[0]
        bare = os.path.join(src, "%s_%s.jpg" % (k, bare_tag))
        band_img = cv2.imread(os.path.join(src, f))
        r = al.label_one(band_img)
        if r is None or not os.path.exists(bare):
            continue
        clean, (cx, cy, w, h), _ratio = r
        bare_img = cv2.imread(bare)
        # ⚠ 같은 관절값으로 돌아와도 ≈1° 차이로 화면이 0~25px 어긋났다(11/04 실측) → 위상상관으로 재서 상자를 옮긴다.
        #   응답이 약하면(장면이 달라졌거나 못 잼) 그 짝은 버린다 — 틀린 라벨이 없는 라벨보다 나쁘다.
        (dx, dy), resp = shift(band_img, bare_img)
        if resp < 0.15:
            print("  %s: 어긋남을 못 쟀다(응답 %.2f) — 버린다" % (k, resp))
            continue
        H, W = bare_img.shape[:2]
        stamp = time.strftime("%m%d")
        for name, img, (bx, by) in (("pair%s_%s_bare" % (stamp, k), bare_img, (cx + dx / W, cy + dy / H)),
                                    ("pair%s_%s_band" % (stamp, k), clean, (cx, cy))):
            cv2.imwrite(os.path.join(dst, "images", "train", name + ".jpg"), img)
            with open(os.path.join(dst, "labels", "train", name + ".txt"), "w") as fh:
                fh.write("0 %.6f %.6f %.6f %.6f\n" % (bx, by, w, h))
        n += 1
    print("짝 %d개 → %s (맨 줄기 %d장 + 띠 지운 %d장)" % (n, dst, n, n))
    return 0


def shift(a_bgr, b_bgr):
    """b가 a에서 (dx, dy)만큼 옮겨진 화면이라 보고 그 양과 응답(0~1)."""
    import cv2
    import numpy as np
    a = np.float32(cv2.cvtColor(a_bgr, cv2.COLOR_BGR2GRAY))
    b = np.float32(cv2.cvtColor(b_bgr, cv2.COLOR_BGR2GRAY))
    return cv2.phaseCorrelate(a, b, cv2.createHanningWindow(a.shape[::-1], cv2.CV_32F))


def selftest():
    import cv2
    import numpy as np
    rng = np.random.default_rng(0)
    a = cv2.GaussianBlur((rng.random((480, 848, 3)) * 255).astype(np.uint8), (7, 7), 0)
    b = np.roll(np.roll(a, 12, axis=1), -7, axis=0)            # 오른쪽 12, 위 7
    (dx, dy), r = shift(a, b)
    assert abs(dx - 12) < 0.5 and abs(dy + 7) < 0.5 and r > 0.3, (dx, dy, r)
    assert len(grid(False)) == 36
    assert all(abs(p) <= 3.0 for f, _, p in grid(True) if f >= 30.0) and max(p for _, _, p in grid(True)) == 12.0
    print("selftest OK")
    return 0


def main():
    if "--selftest" in sys.argv:
        return selftest()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("capture", "replay", "label"))
    ap.add_argument("args", nargs="*")
    ap.add_argument("--tag", default="")
    ap.add_argument("--wide", action="store_true", help="capture: 어깨 ±12°까지(줄기가 화면 가장자리)")
    a = ap.parse_args()
    if a.cmd == "capture":
        return capture(a.tag or "band", a.wide)
    if a.cmd == "replay":
        return replay(a.tag or "bare")
    return label(*a.args)


if __name__ == "__main__":
    raise SystemExit(main())
