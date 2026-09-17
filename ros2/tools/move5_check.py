#!/usr/bin/env python3
"""졸업 기준 3번 — 임의의 자리에 둔 표적으로 `/arm/move_to_point` 5회 시험.

    python ros2/tools/move5_check.py --dry-run
        ROS 없이도 돈다. 좌표는 forward()로 만든 진짜 도달 가능한 점이고,
        판정은 arm_node와 **같은 IK**(kinematics.inverse)를 로컬에서 그대로
        돌린다 — 서보가 실제로 그 자리에 가는지는 확인하지 못한다, 도구
        자체(기록 형식·집계)가 맞는지만 검증한다.

    python ros2/tools/move5_check.py --points points.json
        ROS2 stage1(arm_node)이 떠 있을 때 실기. points.json은
        `[{"x":220,"y":0,"z":100,"pitch":-20}, ...]` (mm, 도) 5개.
        표적을 눈으로 보고 정한 자리를 그대로 넣는다 — 무작위로 만들지 않는다.
        기본(파일 없음)은 forward()로 만든 사거리 안 임의 점 5개를 쓴다.

매 시도를 `docs/시험기록/move-to-point-<오늘날짜>.jsonl`에 한 줄로 남긴다:
    trial, commanded{x,y,z,pitch}, standoff_mm, ok, reached{x,y,z}(관절 FK),
    error_mm, stage(실패 단계: timeout/step/tf/pose/ik/joint/None), detail, dry_run

⚠ **실패 단계**를 구분하는 게 이 도구의 요점이다 — 검출은 이 도구 밖(표적을 사람이
놓는다), 여기서는 응답없음(timeout) / 한 걸음 상한(step) / 좌표계 변환(tf) /
시작 자세(pose) / 목표가 무리(ik) / 이동 후 오차(joint)를 구분한다. 분류는
`classify_stage()` 하나가 하고 `ros_selfcheck`의 [단계]가 그것을 못 박는다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))

from tomato_picker.hardware import kinematics as kin  # noqa: E402

RECORD_DIR = os.environ.get("TOMATO_RECORD_DIR") or os.path.join(REPO, "docs", "시험기록")
STANDOFF_MM = 30.0


# 실패 단계 분류 — arm_node는 **왜 거절했는지를 문장으로만** 돌려준다(구조화된
# 코드가 없다). 그래서 문장을 읽는 수밖에 없는데, 예전 판정은 한 줄이었다:
#     stage = "tf" if "TF" in detail or "좌표" in detail else "ik"
# '좌표'라는 낱말이 **가드 안내문 안에** 들어 있어서(cartesian.py:479 "좌표 이동을
# 쓰세요" / :519 "좌표로 다듬는") 09-18에 두 번 다 엉뚱하게 tf로 적혔다 —
# T21의 자세 가드 5줄, T26의 한 걸음 상한 5줄. 기록은 다음 사이클이 사실로
# 믿는 물건이라, "TF가 깨졌다"는 오독은 하루를 잡아먹는다.
#
# 그래서 **좁은 표지부터** 차례로 본다. 순서가 곧 규칙이다:
#   timeout  응답이 없다 (서비스가 돌려주지 않았다 — 단계를 알 수 없다)
#   step     한 걸음 상한 (cartesian._check_step) — 쪼개면 되는 거절 → T30
#   tf       좌표계 변환 실패 (arm_node._to_arm_base_mm)
#   pose     **지금 자세**가 좌표 이동을 못 받는다 (cartesian._require_state)
#   ik       그 밖 — 사거리·관절한계·너무 작은 지령 등 목표 자체의 문제
# ⚠ 'pose'와 'ik'를 가르는 것은 **'목표'라는 낱말**이다. 같은 "수평거리"가
# 자세 가드(지금 자세)에도 사거리 초과(목표)에도 나오므로, 목표를 가리키는
# 문장은 ik로 보낸다.
STAGES = ("timeout", "step", "tf", "pose", "ik", "joint")


def classify_stage(detail: str | None) -> str:
    """거절 문장 하나를 실패 단계 이름으로 바꾼다. 위 표의 순서대로 본다."""
    text = (detail or "").strip()
    if not text or "응답 없음" in text or "타임아웃" in text:
        return "timeout"
    if "한 번에" in text and "상한" in text:
        return "step"
    if "TF" in text or "transform" in text.lower():
        return "tf"
    if "목표" not in text and ("몸통 뒤로" in text or "거의 수직" in text
                              or "수평거리" in text):
        return "pose"
    if "영점" in text and "없습니다" in text:
        return "pose"       # 영점 미등록도 '지금 상태'의 문제다 — 목표는 죄가 없다
    return "ik"


def sample_points(n: int, geom: kin.ArmGeometry, seed: int) -> list[dict]:
    """forward()로 사거리 안 임의 점을 만든다 — IK가 반드시 풀리는 점이다."""
    rng = random.Random(seed)
    pts = []
    while len(pts) < n:
        joints = {
            "shoulder_pan": rng.uniform(-50.0, 50.0),
            "shoulder_lift": rng.uniform(20.0, 80.0),
            "elbow_flex": rng.uniform(-100.0, -20.0),
            "wrist_flex": rng.uniform(-40.0, 40.0),
        }
        pose = kin.forward(joints, geom)
        if pose.z < -kin.ArmGeometry().z0 + 40.0:   # 바닥 근처는 피한다
            continue
        pts.append({"x": round(pose.x, 1), "y": round(pose.y, 1),
                    "z": round(pose.z, 1), "pitch": round(pose.pitch, 1)})
    return pts


def record(path: str, row: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(row, ensure_ascii=False) + "\n")


def run_dry(points: list[dict], geom: kin.ArmGeometry, out_path: str) -> int:
    """ROS 없이 — 같은 IK를 로컬에서 돌려 도구 자체를 검증한다."""
    ok_count = 0
    for i, p in enumerate(points, 1):
        pose = kin.ToolPose(x=p["x"], y=p["y"], z=p["z"], pitch=p["pitch"])
        standoff_mm = STANDOFF_MM
        dx, dy, dz = kin.offset_in_tool_frame(pose, -standoff_mm, 0.0, 0.0)
        target_pose = pose.replace(x=pose.x + dx, y=pose.y + dy, z=pose.z + dz)
        row = {"trial": i, "dry_run": True, "commanded": p, "standoff_mm": standoff_mm}
        try:
            joints = kin.inverse(target_pose, geom)
            reached = kin.forward(joints, geom)
            err = math.dist((reached.x, reached.y, reached.z),
                             (target_pose.x, target_pose.y, target_pose.z))
            row.update(ok=True, stage=None, error_mm=round(err, 3),
                       reached=reached.as_dict(),
                       detail=f"IK 풀림: {', '.join(f'{j}={v:.1f}°' for j, v in joints.items())}")
            ok_count += 1
        except kin.Unreachable as exc:
            row.update(ok=False, stage="ik", error_mm=None, reached=None, detail=str(exc))
        print(f"  trial {i}: ok={row['ok']} stage={row.get('stage')} "
              f"err={row.get('error_mm')}")
        record(out_path, row)
    return ok_count


def run_real(points: list[dict], out_path: str) -> int:
    """ROS2 stage1이 떠 있어야 한다 — /arm/move_to_point를 실제로 부른다."""
    import rclpy
    from geometry_msgs.msg import PointStamped
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from tf2_ros import Buffer, TransformListener

    from tomato_msgs.srv import MoveToPoint

    rclpy.init()
    node = Node("move5_check")
    latest_js: dict = {"msg": None}
    node.create_subscription(JointState, "/joint_states",
                              lambda m: latest_js.__setitem__("msg", m), 10)
    buf = Buffer()
    TransformListener(buf, node)
    cli = node.create_client(MoveToPoint, "/arm/move_to_point")
    if not cli.wait_for_service(timeout_sec=10.0):
        print("❌ /arm/move_to_point 서비스가 안 보인다 — arm_node가 떠 있는지 확인하라")
        rclpy.shutdown()
        return -1

    def spin(secs: float) -> None:
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

    ok_count = 0
    for i, p in enumerate(points, 1):
        req = MoveToPoint.Request()
        req.target = PointStamped()
        req.target.header.frame_id = "arm_base"
        req.target.point.x = p["x"] / 1000.0
        req.target.point.y = p["y"] / 1000.0
        req.target.point.z = p["z"] / 1000.0
        req.approach_pitch_deg = float(p["pitch"])
        req.standoff_m = STANDOFF_MM / 1000.0
        req.dry_run = False
        fut = cli.call_async(req)
        # ⚠ 먼 목표는 **여러 걸음**으로 간다(cartesian.travel_to) — 720mm면
        # 9걸음이고 걸음마다 보간 0.6초 + 관절 읽기가 든다. 20초로는 정상
        # 이동을 타임아웃으로 적게 된다(그러면 기록이 또 거짓말을 한다).
        rclpy.spin_until_future_complete(node, fut, timeout_sec=90.0)
        res = fut.result()
        row = {"trial": i, "dry_run": False, "commanded": p, "standoff_mm": STANDOFF_MM}
        if res is None:
            row.update(ok=False, stage=classify_stage(None), error_mm=None,
                       reached=None, detail="응답 없음(타임아웃)")
            record(out_path, row)
            print(f"  trial {i}: 응답 없음")
            continue
        if not res.ok:
            stage = classify_stage(res.detail)
            row.update(ok=False, stage=stage, error_mm=None, reached=None, detail=res.detail)
            record(out_path, row)
            print(f"  trial {i}: FAIL[{stage}] {res.detail}")
            continue
        spin(2.0)   # 이동 정착 대기
        js = latest_js["msg"]
        if js is None:
            row.update(ok=False, stage="joint", error_mm=None, reached=None,
                       detail="/joint_states가 안 온다")
            record(out_path, row)
            print(f"  trial {i}: FAIL[joint] /joint_states 없음")
            continue
        degs = {n: math.degrees(v) for n, v in zip(js.name, js.position)}
        pose = kin.forward(degs, kin.ArmGeometry())
        target_reached_mm = (res.reached.x * 1000.0, res.reached.y * 1000.0,
                              res.reached.z * 1000.0)
        err = math.dist((pose.x, pose.y, pose.z), target_reached_mm)
        success = err <= 15.0
        row.update(ok=success, stage=None if success else "joint",
                   error_mm=round(err, 2), reached=pose.as_dict(), detail=res.detail)
        record(out_path, row)
        print(f"  trial {i}: ok={success} err={err:.2f}mm")
        if success:
            ok_count += 1
    rclpy.shutdown()
    return ok_count


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--points", default="")
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    geom = kin.ArmGeometry()
    if args.points:
        points = json.load(open(args.points, encoding="utf-8"))
    else:
        points = sample_points(args.n, geom, args.seed)

    today = time.strftime("%Y-%m-%d")
    out_path = os.path.join(RECORD_DIR, f"move-to-point-{today}.jsonl")

    print(f"{len(points)}개 표적, 기록: {out_path}")
    if args.dry_run:
        ok = run_dry(points, geom, out_path)
    else:
        ok = run_real(points, out_path)
        if ok < 0:
            return 1

    print(f"\n{ok}/{len(points)} 성공")
    return 0 if ok == len(points) else 1


if __name__ == "__main__":
    raise SystemExit(main())
