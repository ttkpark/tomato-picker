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
    error_mm, stage(실패 단계: tf/ik/joint/None), detail, dry_run

⚠ **실패 단계**를 구분하는 게 이 도구의 요점이다 — 검출은 이 도구 밖(표적을 사람이
놓는다), 여기서는 TF(좌표계 변환) / IK(사거리 밖) / 관절(이동 후 오차)만 구분한다.
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

RECORD_DIR = os.path.join(REPO, "docs", "시험기록")
STANDOFF_MM = 30.0


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
        rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
        res = fut.result()
        row = {"trial": i, "dry_run": False, "commanded": p, "standoff_mm": STANDOFF_MM}
        if res is None:
            row.update(ok=False, stage="tf", error_mm=None, reached=None,
                       detail="응답 없음(타임아웃)")
            record(out_path, row)
            print(f"  trial {i}: 응답 없음")
            continue
        if not res.ok:
            stage = "tf" if "TF" in res.detail or "좌표" in res.detail else "ik"
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
