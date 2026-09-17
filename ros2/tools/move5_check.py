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
    error_mm, stage(실패 단계: timeout/step/tf/pose/ik/joint/None), detail, dry_run,
    limits(표적을 뽑을 때 **이 팔의 가동범위를 알고 있었나** — 보정표 경로 또는
    "none". none이면 그 시험은 갈 수 없는 자리를 시험했을 수 있다)

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

from tomato_picker.config import ARM_CART_R_MIN, ARM_CART_Z_MIN  # noqa: E402
from tomato_picker.hardware import cartesian as cart  # noqa: E402
from tomato_picker.hardware import kinematics as kin  # noqa: E402

RECORD_DIR = os.environ.get("TOMATO_RECORD_DIR") or os.path.join(REPO, "docs", "시험기록")


def local_zone() -> str:
    """지금 날짜를 어느 시각대로 셌는지 한 토막으로. 기록 이름이 날짜라서 중요하다.

    도커 기본값은 UTC다 — 한국 자정~09시에 컨테이너에서 돌리면 파일이 **어제**
    이름으로 열린다(2026-09-18 실측). compose가 호스트의 /etc/localtime을 물려
    막았지만, `docker run`으로 직접 띄우면 다시 UTC가 된다. 그래서 고친 자리를
    믿지 말고 **매 실행이 스스로 말하게** 한다.
    """
    off = -(time.altzone if time.daylight and time.localtime().tm_isdst
            else time.timezone)
    sign = "+" if off >= 0 else "-"
    hours, mins = divmod(abs(off) // 60, 60)
    return f"{time.strftime('%Z')} UTC{sign}{hours}" + (f":{mins:02d}" if mins else "")


STANDOFF_MM = 30.0
# 표적 뽑기가 포기하기까지 던져 보는 횟수. 가동범위가 좁으면 기각이 잦은데,
# 무한히 도는 것보다 "이 한계로는 n개를 못 뽑는다"고 말하고 멈추는 편이 낫다.
MAX_SAMPLE_TRIES = 20000
# 표적을 뽑을 때 훑는 '무대 앞'의 상식적인 관절 구간(도). 여기에 이 팔의
# **가동범위를 교집합**해서 던진다 — 구간 자체가 한계가 아니다.
SAMPLE_JOINT_DEG = {
    "shoulder_pan": (-50.0, 50.0),
    "shoulder_lift": (20.0, 80.0),
    "elbow_flex": (-100.0, -20.0),
    "wrist_flex": (-40.0, 40.0),
}


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


def standoff_pose(p: dict, standoff_mm: float = STANDOFF_MM) -> kin.ToolPose:
    """표적 점 → **실제로 명령되는 자리**(접근축 반대로 물러난 지점).

    arm_node._on_move(:153)이 하는 것과 같은 계산이다. 관절 한계를 볼 때도
    여기를 봐야 한다 — 팔이 가는 곳은 표적이 아니라 이 자리다.
    """
    pose = kin.ToolPose(x=p["x"], y=p["y"], z=p["z"], pitch=p["pitch"])
    if not standoff_mm:
        return pose
    dx, dy, dz = kin.offset_in_tool_frame(pose, -standoff_mm, 0.0, 0.0)
    return pose.replace(x=pose.x + dx, y=pose.y + dy, z=pose.z + dz)


def target_from_stand(pose: kin.ToolPose,
                      standoff_mm: float = STANDOFF_MM) -> dict:
    """팔이 **설 자리** → 그 앞의 표적 점. `standoff_pose`의 역이다.

    표적을 이렇게 만드는 이유: 팔이 가는 곳은 표적이 아니라 스탠드오프 지점
    이므로, 관절 한계를 지켜야 하는 것도 그쪽이다. 서 있을 자리를 먼저 뽑고
    표적을 그 앞에 두면 **뽑은 자리가 곧 갈 수 있는 자리**가 된다. 반대로
    표적을 먼저 뽑으면 스탠드오프가 팔꿈치를 30° 가까이 더 굽히는 바람에
    (이 팔에서 실측: 중앙값 −30.7°) 통과시킨 표적이 실기에서 거절당한다.
    """
    dx, dy, dz = kin.offset_in_tool_frame(pose, standoff_mm, 0.0, 0.0)
    return {"x": round(pose.x + dx, 1), "y": round(pose.y + dy, 1),
            "z": round(pose.z + dz, 1), "pitch": round(pose.pitch, 1)}


def workspace_reject(pose: kin.ToolPose, geom: kin.ArmGeometry) -> str:
    """`cartesian._check_workspace`가 거절할 자리인가. 거절 이유 또는 "".

    ⚠ 상수를 여기 박지 않는다 — 바닥(z)과 몸통 반경(r)은 config가 정하고,
    실제로 거절하는 곳도 거기다. 뽑는 쪽이 다른 숫자를 들고 있으면 도구가
    통과시킨 자리를 팔이 거절한다(2026-09-18 실기 5번째: 수평 76mm < 90mm).
    """
    if pose.z < ARM_CART_Z_MIN:
        return f"z={pose.z:.0f}mm < {ARM_CART_Z_MIN:.0f}mm(바닥)"
    r = math.hypot(pose.x, pose.y)
    if r < ARM_CART_R_MIN:
        return f"수평 {r:.0f}mm < {ARM_CART_R_MIN:.0f}mm(몸통)"
    if r > geom.reach_max:
        return f"수평 {r:.0f}mm > 사거리 {geom.reach_max:.0f}mm"
    return ""


def limit_violations(p: dict, geom: kin.ArmGeometry, limits,
                     standoff_mm: float = STANDOFF_MM) -> dict[str, float]:
    """이 표적을 그 팔이 갈 수 있나. 못 가면 {관절: 정규화값}, 갈 수 있으면 {}."""
    try:
        joints = kin.inverse(standoff_pose(p, standoff_mm), geom)
    except kin.Unreachable:
        return {"__unreachable__": 0.0}
    return limits.violations(joints)


def sample_joint_ranges(limits=None) -> dict[str, tuple[float, float]]:
    """뽑기 구간(도) — 상식적인 구간 ∩ 이 팔의 가동범위. limits가 없으면 그대로."""
    out = {}
    for j, (lo, hi) in SAMPLE_JOINT_DEG.items():
        if limits is not None:
            a, b = limits.degree_range(j)
            lo, hi = max(lo, a), min(hi, b)
        if lo >= hi:
            raise RuntimeError(
                f"{j}: 뽑기 구간 {SAMPLE_JOINT_DEG[j]}°와 이 팔의 가동범위 "
                f"{limits.degree_range(j) if limits else '?'}°가 겹치지 않는다 — "
                "SAMPLE_JOINT_DEG를 이 팔에 맞게 넓히거나 --points로 자리를 직접 주라."
            )
        out[j] = (lo, hi)
    return out


def sample_points(n: int, geom: kin.ArmGeometry, seed: int,
                  limits=None, standoff_mm: float = STANDOFF_MM) -> list[dict]:
    """임의 표적 n개. **팔이 설 자리를 뽑고 표적은 그 앞 스탠드오프에 둔다.**

    ⚠ **사거리 안 ≠ 갈 수 있는 자리.** 사거리는 링크 길이(기구학)가 정하고
    가동범위는 캘리브레이션이 정한다. 2026-09-18 실기에서 5/5가 `elbow_flex`
    정규화 −121~−138(한계 ±98)로 거절됐는데, 그 0/5는 팔이 만든 0이 아니라
    **이 함수가 만든 0**이었다 — 뽑기 구간 −100~−20°가 이 팔에서는 −38.9°
    까지만 살아 있기 때문이다. 그래서 두 가지를 고쳤다:
      ① 뽑기 구간에 가동범위를 교집합한다(`sample_joint_ranges`).
      ② 뽑은 관절값을 **스탠드오프 지점**으로 삼는다 — 팔이 서는 곳이 거기다.
         표적을 먼저 뽑으면 스탠드오프가 팔꿈치를 다시 굽혀(중앙값 −30.7°)
         통과시킨 표적이 실기에서 거절당한다.
    limits가 None이면(팔 보정표를 못 읽는 PC) 옛날 구간 그대로 뽑되, 부르는
    쪽이 기록에 limits=none을 남겨 그 시험이 무엇을 못 봤는지 남게 한다.
    """
    rng = random.Random(seed)
    ranges = sample_joint_ranges(limits)
    pts: list[dict] = []
    tries = 0
    while len(pts) < n:
        tries += 1
        if tries > MAX_SAMPLE_TRIES:
            raise RuntimeError(
                f"{MAX_SAMPLE_TRIES}번 던져 {len(pts)}/{n}개밖에 못 뽑았다 — 뽑기 "
                f"구간 {ranges}이 이 팔에서 거의 비어 있다는 뜻이다. "
                "SAMPLE_JOINT_DEG를 넓히거나 --points로 직접 자리를 주라."
            )
        stand = {j: rng.uniform(lo, hi) for j, (lo, hi) in ranges.items()}
        pose = kin.forward(stand, geom)
        if workspace_reject(pose, geom):   # 팔이 설 자리 — 바닥·몸통·사거리
            continue
        p = target_from_stand(pose, standoff_mm)
        # ⚠ 반올림한 **뒤에** 본다 — 기록에 남는 숫자가 곧 명령이 되는 숫자다.
        if workspace_reject(standoff_pose(p, standoff_mm), geom):
            continue
        if limits is not None and limit_violations(p, geom, limits, standoff_mm):
            continue
        pts.append(p)
    return pts


def record(path: str, row: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(row, ensure_ascii=False) + "\n")


def run_dry(points: list[dict], geom: kin.ArmGeometry, out_path: str,
            limits_tag: str = "none") -> int:
    """ROS 없이 — 같은 IK를 로컬에서 돌려 도구 자체를 검증한다."""
    ok_count = 0
    for i, p in enumerate(points, 1):
        standoff_mm = STANDOFF_MM
        target_pose = standoff_pose(p, standoff_mm)
        row = {"trial": i, "dry_run": True, "commanded": p, "standoff_mm": standoff_mm,
               "limits": limits_tag}
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


def _arm_node_geometry() -> kin.ArmGeometry:
    """arm_node._geometry()와 **같은 규칙**으로 기하를 고른다 (T37 감사, 2026-09-18).

    arm_node.py는 `~/arm_cartesian.json`의 `geometry` 키가 있으면 그걸 쓰고
    없으면 코드 기본값으로 떨어진다. 여기서 `kin.ArmGeometry()` 기본값만 쓰면
    그 파일이 `geometry`를 채우는 순간(예: 실측으로 링크를 다시 잰 뒤) **이
    도구가 재는 자와 arm_node가 계획하는 자가 다른 팔 길이를 믿게 된다** —
    말없이 갈라진다. arm_node.py를 직접 import할 수 없어(rclpy가 든다) 같은
    폴백 규칙을 여기서도 그대로 편다.
    """
    try:
        return cart.FrameConfig().geometry()
    except Exception:  # noqa: BLE001 - 파일이 없으면 코드 기본값(arm_node와 동일)
        return kin.ArmGeometry()


def run_real(points: list[dict], out_path: str, limits_tag: str = "none") -> int:
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
        row = {"trial": i, "dry_run": False, "commanded": p,
               "standoff_mm": STANDOFF_MM, "limits": limits_tag}
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
        pose = kin.forward(degs, _arm_node_geometry())
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
    # 이 팔이 **실제로 갈 수 있는 범위**. 팔을 열지 않고 파일에서 읽는다
    # (포트는 ROS가 쥐고 있다). 못 읽으면 None이고, 그 사실이 기록에 남는다.
    limits, limits_note = cart.load_norm_limits()
    limits_tag = limits.source if limits else "none"
    print(f"관절 가동범위: {limits_note}")

    if args.points:
        points = json.load(open(args.points, encoding="utf-8"))
        if limits:
            # 사람이 고른 자리도 갈 수 있는지는 봐 준다 — 막지는 않는다
            # (눈으로 본 자리가 맞고 보정표가 낡았을 수도 있다).
            for i, p in enumerate(points, 1):
                over = limit_violations(p, geom, limits)
                if not over:
                    continue
                why = ("IK가 안 풀린다" if "__unreachable__" in over else
                       ", ".join(f"{j} {v:+.0f}(한계 ±{limits.limit:.0f})"
                                 for j, v in sorted(over.items())))
                print(f"  ⚠ 표적 {i}는 이 팔의 가동범위 밖으로 보인다 — {why}")
    else:
        points = sample_points(args.n, geom, args.seed, limits=limits)

    today = time.strftime("%Y-%m-%d")
    out_path = os.path.join(RECORD_DIR, f"move-to-point-{today}.jsonl")

    print(f"{len(points)}개 표적, 기록: {out_path}  [{local_zone()}]")
    if args.dry_run:
        ok = run_dry(points, geom, out_path, limits_tag)
    else:
        ok = run_real(points, out_path, limits_tag)
        if ok < 0:
            return 1

    print(f"\n{ok}/{len(points)} 성공")
    return 0 if ok == len(points) else 1


if __name__ == "__main__":
    raise SystemExit(main())
