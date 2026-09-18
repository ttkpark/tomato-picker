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

매 시도를 `docs/시험기록/move-to-point-<오늘날짜>.jsonl`에 한 줄로 남긴다
(⚠ `--dry-run`은 **연습 자리**(임시 디렉터리)에 남긴다 — 연습 한 번이 그날의 진짜
기록을 오염시킨 적이 있다, T43. 연습도 진짜 기록에 남기려면 `--record`):
    trial, commanded{x,y,z,pitch}, standoff_mm, ok, reached{x,y,z}(관절 FK),
    error_mm, stage(실패 단계: timeout/step/tf/pose/ik/joint/None), detail, dry_run,
    joints_cmd(이 표적으로 **보내려는** 관절 5개, 도 — IK가 안 풀리면 null),
    joints_actual(이동이 끝난 뒤 **되읽은** 관절 5개, 도 — 못 읽으면 null),
    limits(표적을 뽑을 때 **이 팔의 가동범위를 알고 있었나** — 보정표 경로 또는
    "none". none이면 그 시험은 갈 수 없는 자리를 시험했을 수 있다),
    load_limits(표적을 뽑을 때 **팔이 그 자리를 들 수 있는지 알고 있었나** —
    경계의 출처 또는 "none". 가동범위와 다른 종류의 한계다, 아래 참고),
    prep(**어떤 시작 자세에서 출발했나** — needed/extended/signed_r/detail.
    0/5를 볼 때 그것이 팔의 0인지 시작 자세의 0인지 여기서 가른다)

⚠ **시작 자세는 이 도구가 스스로 만든다**(T49, 2026-09-18). 자세 가드
(signed_radius < 90mm) 안에서 그냥 돌리면 5회가 5회 다 거절되고, 그 0/5는 팔이
만든 0이 아니라 시작 자세가 만든 0이다 — 사이클35가 실제로 그렇게 기록됐다.
그래서 실기 전에 관절공간으로 먼저 뻗는다(`/arm/joint_command`, 규칙은
`hardware/escape.py`로 `arm_extend.py`와 공유). 끄려면 `--no-prep`.

⚠ **뽑는 자리는 '들 수 있는 자리'라야 한다**(T41, 2026-09-18). 가동범위를
지나고도 서보가 못 버티는 자리가 있다 — 2026-09-18 실기에서 r=331mm의 자세는
관절각으로는 풀리는데 어깨가 한계에 눌려 z가 73mm 처진 채 끝났다(§24). 그 경계는
`hardware/load_limits.py`가 `config.ARM_LOAD_R_MAX` 또는 `~/arm_load_limits.json`
에서 읽는다. **못 읽으면 거르지 않고, 그 사실을 기록에 `load_limits=none`으로**
남긴다 — 그러면 그 0/5가 팔의 0인지 도구의 0인지 나중에 가릴 수 있다.

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
import re
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))

from tomato_picker.config import ARM_CART_R_MIN, ARM_CART_Z_MIN  # noqa: E402
from tomato_picker.hardware import cartesian as cart  # noqa: E402
from tomato_picker.hardware import escape as es  # noqa: E402
from tomato_picker.hardware import kinematics as kin  # noqa: E402
from tomato_picker.hardware import load_limits as ld  # noqa: E402

RECORD_DIR_ENV = os.environ.get("TOMATO_RECORD_DIR")
RECORD_DIR = RECORD_DIR_ENV or os.path.join(REPO, "docs", "시험기록")
# 연습(--dry-run)이 떨어지는 자리. 저장소 밖이라 무엇을 몇 번 돌려도 진짜 기록이
# 더러워지지 않는다 — 지우는 것도 사람이 신경 쓸 일이 아니다.
DRY_RECORD_DIR = os.path.join(tempfile.gettempdir(), "tomato-move5-dry")


def record_dir_for(dry_run: bool, record: bool) -> tuple[str, str]:
    """이 판의 기록이 **어디에 떨어지는가**. (디렉터리, 왜)

    이 갈림이 있는 이유 (2026-09-18, T43): 사이클35의 빌더가 도구를 고친 뒤
    `--dry-run --n 2`를 스모크 테스트로 한 번 돌렸더니 그날의 진짜 시험기록
    (`move-to-point-2026-09-18.jsonl`)에 연습 줄 2개가 섞였다(git checkout으로
    되돌렸다). 기록은 다음 사이클이 **사실로 믿는** 유일한 물건인데 연습 한 번이
    그것을 오염시킨다 — 기본값이 틀려 있었던 것이다. 그래서 연습은 기본이
    연습 자리고, 진짜 기록에 남기려면 `--record`로 **말해야** 한다.

    ⚠ 실기(run_real)의 기본은 건드리지 않는다 — 실기 판정은 늘 진짜 기록이다.
    ⚠ `TOMATO_RECORD_DIR`을 준 것도 '말한 것'으로 본다(그 자리에 모으려고
      일부러 준 값을 연습이라고 빼앗으면 놀란다).
    """
    if dry_run and not record and not RECORD_DIR_ENV:
        return DRY_RECORD_DIR, "연습 — 진짜 기록에 남기려면 --record"
    if dry_run:
        why = "--record" if record else "TOMATO_RECORD_DIR"
        return RECORD_DIR, f"연습이지만 {why}로 진짜 기록 자리를 지정했다"
    return RECORD_DIR, "실기 기록"


def record_home(path: str) -> tuple[str, str]:
    """이 기록이 **저장소로 돌아갈 길이 있는가**. ("repo"|"volatile"|"unwritable", 설명)

    이 함수가 있는 이유 (2026-09-18, T58): 실기는 젯슨의 도커 안에서 돈다.
    컨테이너에 물린 것은 **젯슨 트리**(`~/tomato-picker/`)이고 그 트리는 git
    저장소가 아니다(실측: `git rev-parse` → "not a git repository"). 거기 쓴 줄은
    커밋될 길이 없다 — 사이클42의 실기 5회가 산문에만 남고 jsonl에 없던 이유다.
    기록은 다음 사이클이 사실로 믿는 유일한 물건이라 **조용히 사라지는 것만은
    안 된다.** 그래서 매 실행이 자기 기록이 어디에 떨어지는지 스스로 말한다.

    판정은 `.git`이 있느냐 하나다 — 마운트 경로를 짐작하는 것보다 이쪽이
    직접적이다(같은 젯슨 트리를 어떤 경로로 물리든 결론이 같다).
    """
    probe = os.path.join(path, ".probe-%d" % os.getpid())
    try:
        os.makedirs(path, exist_ok=True)
        with open(probe, "w") as fh:
            fh.write("x")
        os.remove(probe)
    except OSError as exc:
        return "unwritable", f"{path} — 쓸 수 없다: {exc}"
    here = os.path.abspath(path)
    while True:
        if os.path.exists(os.path.join(here, ".git")):
            return "repo", here
        parent = os.path.dirname(here)
        if parent == here:
            return "volatile", f"{path} — git 저장소 밖이다(커밋될 길이 없다)"
        here = parent


RECORD_HOME, RECORD_HOME_WHY = record_home(RECORD_DIR)


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
#   path     **목표는 멀쩡한데 가는 길이 막혔다** (cartesian._walk/_run_path)
#   pose     **지금 자세**가 좌표 이동을 못 받는다 (cartesian._require_state)
#   ik       그 밖 — 사거리·관절한계·너무 작은 지령 등 목표 자체의 문제
# ⚠ 'pose'와 'ik'를 가르는 것은 **'목표'라는 낱말**이다. 같은 "수평거리"가
# 자세 가드(지금 자세)에도 사거리 초과(목표)에도 나오므로, 목표를 가리키는
# 문장은 ik로 보낸다.
# ⚠ 'path'를 따로 둔 이유(T34) — 사이클20 실기 9줄이 전부 stage=ik로 적혔는데
# ik는 "목표가 무리"라는 뜻이고 **목표는 멀쩡했다**(사전검사 5/5 통과). 막힌
# 것은 첫 걸음이다. 같은 문장이 자세 가드의 낱말("수평거리")을 그대로 물고
# 오기도 해서 pose로도 샜다(감사 사이클40 재현). 걸음 표지를 ik/pose보다
# **먼저** 보면 둘 다 막힌다 — 고칠 곳이 목표가 아니라 경로임을 기록이 말한다.
# 다만 **한 걸음 상한만은 그대로 step**이다(걸음 안에서 나기에 문구가 겹치지만,
# 고치는 방법이 "쪼개라"로 다르다 → T30).
STAGES = ("timeout", "step", "tf", "path", "pose", "ik", "joint")

# `cartesian`이 경로 실패에 붙이는 표지들. **살아 있는 문장에서 그대로 따왔고**
# ros_selfcheck [단계]가 그 문장이 아직 이 모양인지 지킨다.
#   _walk      "{N}걸음 중 {i}번째에서 막힌다 — …"        (걷기 전에 막힘)
#   _run_path  "{N}걸음 중 {i}번째에서 멈췄습니다 — …"    (걷다가 막힘)
#   _run_path  "{n}걸음에서 더 안 갑니다 — …"             (서보가 안 따라옴)
#   _run_path  "{budget}걸음을 걷고도 목표에 못 닿았습니다 — …"
#   travel_to  "갈 길이 없습니다 — 직선 경로: … / 관절공간 경로: …"
PATH_STEP_RE = re.compile(r"\d+\s*걸음\s*중\s*\d+\s*번째")
PATH_MARKERS = ("걸음에서 더 안 갑니다", "걸음을 걷고도", "갈 길이 없습니다")

# `_run_path`가 성공 문장 **끝에** 붙이는 해설 — " [직선 경로가 막혀 관절공간으로
# 돌아갔다(3걸음 중 1번째에서 막힌다 — …)]". 이미 버리고 딴 길로 간 경로의
# 이야기라 실패 이유가 아니다. 안 떼면 성공 줄까지 path로 읽힌다.
_WHY_TAIL_RE = re.compile(r"\s*\[[^\[\]]*\]\s*$")


def classify_stage(detail: str | None) -> str:
    """거절 문장 하나를 실패 단계 이름으로 바꾼다. 위 표의 순서대로 본다."""
    text = _WHY_TAIL_RE.sub("", (detail or "").strip()).strip()
    if not text or "응답 없음" in text or "타임아웃" in text:
        return "timeout"
    if "한 번에" in text and "상한" in text:
        return "step"
    if "TF" in text or "transform" in text.lower():
        return "tf"
    if PATH_STEP_RE.search(text) or any(m in text for m in PATH_MARKERS):
        return "path"
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


def commanded_joints(p: dict, geom: kin.ArmGeometry,
                     standoff_mm: float = STANDOFF_MM) -> dict[str, float] | None:
    """이 표적으로 **실제로 보내지는** 관절 각(도) — arm_node가 IK로 풀 값과 같다.

    사이클50 플래너가 성공/실패를 가르려고 기록의 TCP를 `kin.inverse`로
    역산해야 했다(역산은 유일해가 아니다 — elbow_up 두 해 중 하나를 짐작으로
    골랐다). 지령 관절을 뽑는 시점에 그대로 적어 두면 다음 사이클이 역산할
    필요가 없다. IK가 안 풀리면 None — 그 표적은 애초에 갈 수 없다는 뜻이라
    짐작으로 메우지 않는다(load_limits.py와 같은 규칙).
    """
    try:
        return {j: round(v, 2) for j, v
                in kin.inverse(standoff_pose(p, standoff_mm), geom).items()}
    except kin.Unreachable:
        return None


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
                  limits=None, standoff_mm: float = STANDOFF_MM,
                  load: ld.LoadLimits | None = None) -> list[dict]:
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

    ⚠ **갈 수 있는 자리 ≠ 들 수 있는 자리**(T41). 위 ①②를 다 지나고도 서보가
    못 버티는 자리가 있다 — 2026-09-18 실기 5회 중 셋이 그것이었다(z를 115~190mm
    떨군 채 멈췄다). `load`를 주면 팔이 **설 자리**의 수평 사거리로 그것도 거른다
    (표적이 아니라 설 자리다 — 팔이 가는 곳이 거기다). `load`가 None이면 못 거르고,
    limits와 같은 규칙으로 부르는 쪽이 기록에 load_limits=none을 남긴다.
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
                f"구간 {ranges}이 이 팔에서 거의 비어 있다는 뜻이다"
                + (f" (드는 한계 {load.r_max:.0f}mm도 같이 자른다)" if load else "")
                + ". SAMPLE_JOINT_DEG를 넓히거나 --points로 직접 자리를 주라."
            )
        stand = {j: rng.uniform(lo, hi) for j, (lo, hi) in ranges.items()}
        pose = kin.forward(stand, geom)
        if workspace_reject(pose, geom):   # 팔이 설 자리 — 바닥·몸통·사거리
            continue
        p = target_from_stand(pose, standoff_mm)
        # ⚠ 반올림한 **뒤에** 본다 — 기록에 남는 숫자가 곧 명령이 되는 숫자다.
        stand_pose = standoff_pose(p, standoff_mm)
        if workspace_reject(stand_pose, geom):
            continue
        if load is not None and load.rejects(stand_pose.x, stand_pose.y, stand_pose.z):
            continue   # 갈 수는 있지만 **못 드는** 자리 — 서보 토크의 벽(§24)
        if limits is not None and limit_violations(p, geom, limits, standoff_mm):
            continue
        pts.append(p)
    return pts


# ── 시험 전 준비(prep) — 가드에 갇힌 자세면 스스로 뻗는다 ───────────────────
# 왜 여기 있나 (T49, 2026-09-18): 사이클35는 A자세에서 이 도구를 그대로 돌려
# **5/5 자세 가드 거절**을 받았다. 사람이 `arm_extend --hold`를 따로 돌린 뒤에야
# 1/5이 나왔다. 그 수동 단계는 어디에도 안 적혀 있어서, 잊으면 기록에 **거짓
# 0/5**가 남는다 — 그리고 일지와 작업판은 다음 사이클이 사실로 믿는다.
# 규칙(12°·바닥·한계·목표자세)은 베끼지 않는다: `hardware/escape.py` 하나가 갖고
# `arm_extend.py`도 같은 것을 쓴다.
# ⚠ "지령과 실제가 N° 벌어지면 걸린 것"이라는 상수는 여기 없다(2026-09-18 T56에
# 지웠다). 한 걸음 뒤의 처짐은 정상이고(4~5°), 걸렸다는 진짜 증거는 "한 걸음을
# 더 보내도 남은 길이 안 줄어든다"이다 — 그 판정은 `escape.walk`의
# MIN_GAIN_DEG가 한 자리에서 한다.


def prep_plan(now_deg: dict[str, float], limits, geom: kin.ArmGeometry,
              target_deg: dict[str, float] | None = None) -> dict:
    """시험 전에 뻗어야 하나, 뻗는다면 어떤 걸음들인가. **팔을 안 건드린다.**

    돌려주는 것: {"needed", "signed_r", "steps", "target", "clamped", "blocked"}.
    `needed`가 False면 걸음은 비어 있다 — **이미 가드 밖이면 안 뻗는다**(뻗는 것
    자체가 시작 자세를 바꾸므로, 필요 없을 때 하면 시험을 흔든다).
    """
    r_now = kin.signed_radius(now_deg, geom)
    out = {"needed": r_now < es.GUARD_MM, "signed_r": round(r_now, 1),
           "steps": [], "target": None, "clamped": [], "blocked": []}
    if not out["needed"]:
        return out
    now_norm = limits.norms(now_deg)
    want = {**now_deg, **(target_deg or es.TARGET_DEG)}
    clamped, hit = es.clamp_norm(limits.norms(want), limits.limit)
    # 자른 뒤의 목표는 정규화 → 각도로 되돌려야 한다(자른 값이 곧 목표다).
    target = cart.norms_to_degrees({**now_norm, **clamped}, zero=limits.zero,
                                   ref=limits.ref, signs=limits.signs,
                                   deg_per_norm=limits.deg_per_norm)
    steps = es.plan(now_deg, target, now_norm, limits.norms, geom,
                    limit=limits.limit)
    out.update(steps=steps, target=target, clamped=sorted(hit),
               blocked=es.blocked(steps),
               signed_r_target=round(kin.signed_radius(target, geom), 1))
    return out


def record(path: str, row: dict) -> None:
    # 줄마다 **어디서 태어났는지**를 남긴다 — 저장소에서 바로 자란 줄인지
    # 손으로 옮겨 와야 하는 줄인지를 다음 사이클이 줄만 보고 알아야 한다(T58).
    row.setdefault("record_home", RECORD_HOME)
    # 시각이 없으면 두 판을 구분할 길이 없다 — 2026-09-18 실측으로 같은 표적의
    # 같은 거절이 여러 판에 걸쳐 글자까지 같았고, 회수 도구가 그것을 한 판으로
    # 접을 뻔했다. 시각 하나가 그 애매함을 없앤다(T58).
    row.setdefault("t", time.strftime("%Y-%m-%d %H:%M:%S") + f" {local_zone()}")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(row, ensure_ascii=False) + "\n")


def run_dry(points: list[dict], geom: kin.ArmGeometry, out_path: str,
            limits_tag: str = "none", load_tag: str = "none",
            geom_tag: str = "config.ARM_GEOM_*",
            graduation_blocked: bool = False) -> int:
    """ROS 없이 — 같은 IK를 로컬에서 돌려 도구 자체를 검증한다."""
    ok_count = 0
    for i, p in enumerate(points, 1):
        standoff_mm = STANDOFF_MM
        target_pose = standoff_pose(p, standoff_mm)
        row = {"trial": i, "dry_run": True, "commanded": p, "standoff_mm": standoff_mm,
               "limits": limits_tag, "load_limits": load_tag,
               "geometry": geom_tag,
               "graduation_blocked": graduation_blocked}
        try:
            joints = kin.inverse(target_pose, geom)
            reached = kin.forward(joints, geom)
            err = math.dist((reached.x, reached.y, reached.z),
                             (target_pose.x, target_pose.y, target_pose.z))
            joints_rounded = {j: round(v, 2) for j, v in joints.items()}
            row.update(ok=True, stage=None, error_mm=round(err, 3),
                       reached=reached.as_dict(),
                       joints_cmd=joints_rounded,
                       # dry-run엔 실제 서보가 없다 — IK가 곧 "이상적 실행"이므로
                       # 되읽은 값도 지령과 같다(서보 오차는 이 모드가 원래
                       # 못 본다, 파일 머리말에 이미 적혀 있다).
                       joints_actual=joints_rounded,
                       detail=f"IK 풀림: {', '.join(f'{j}={v:.1f}°' for j, v in joints.items())}")
            ok_count += 1
        except kin.Unreachable as exc:
            row.update(ok=False, stage="ik", error_mm=None, reached=None,
                       joints_cmd=None, joints_actual=None, detail=str(exc))
        print(f"  trial {i}: ok={row['ok']} stage={row.get('stage')} "
              f"err={row.get('error_mm')}")
        record(out_path, row)
    return ok_count


def _arm_node_geometry() -> tuple[kin.ArmGeometry, str]:
    """arm_node._geometry()와 **같은 규칙**으로 기하를 고른다 (T37 감사, 2026-09-18).

    arm_node.py는 `~/arm_cartesian.json`의 `geometry` 키가 있으면 그걸 쓰고
    없으면 코드 기본값으로 떨어진다. 여기서 `kin.ArmGeometry()` 기본값만 쓰면
    그 파일이 `geometry`를 채우는 순간(예: 실측으로 링크를 다시 잰 뒤) **이
    도구가 재는 자와 arm_node가 계획하는 자가 다른 팔 길이를 믿게 된다** —
    말없이 갈라진다. arm_node.py를 직접 import할 수 없어(rclpy가 든다) 같은
    폴백 규칙을 여기서도 그대로 편다.

    ⚠ **뽑는 쪽도 이 함수를 써야 한다**(T53). T37은 재는 쪽(run_real)만 고쳤고
    `main()`은 `kin.ArmGeometry()` 기본값으로 표적을 뽑고 있었다 — 그러면 그
    파일이 채워지는 날 표적을 뽑은 팔과 성공을 재는 팔이 다른 길이가 된다.
    기준3의 점수 자체가 못 믿을 것이 되므로 **이 파일에 기하를 만드는 자리는
    여기 하나뿐**이고, ros_selfcheck [표적]이 그것을 강제한다.

    되돌려 주는 둘째 값은 **어디서 왔는지**다. 기록 줄에 그대로 남는다 —
    limits·load_limits와 같은 이유로, 뒤에서 읽는 사람이 "어떤 팔 길이로 뽑고
    쟀는지"를 짐작하지 않아도 되게.
    """
    try:
        cfg = cart.FrameConfig()
        geom = cfg.geometry()
        if geom != kin.ArmGeometry():
            return geom, f"file:{cfg.path}"
        # 파일은 있어도 geometry 칸이 비면 코드 기본값과 같다 — 같은 숫자를
        # 두 이름으로 부르지 않는다.
        return geom, "config.ARM_GEOM_*"
    except Exception:  # noqa: BLE001 - 파일이 없으면 코드 기본값(arm_node와 동일)
        return kin.ArmGeometry(), "config.ARM_GEOM_*"


def run_real(points: list[dict], out_path: str, limits_tag: str = "none",
             limits=None, prep: bool = True, load_tag: str = "none",
             force: bool = False, graduation_blocked: bool = False) -> int:
    """ROS2 stage1이 떠 있어야 한다 — /arm/move_to_point를 실제로 부른다."""
    import rclpy
    from geometry_msgs.msg import PointStamped
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    from tomato_msgs.srv import MoveToPoint

    rclpy.init()
    node = Node("move5_check")
    latest_js: dict = {"msg": None}
    node.create_subscription(JointState, "/joint_states",
                              lambda m: latest_js.__setitem__("msg", m), 10)
    # 뻗기(prep) 전용 입구 — arm_node가 팔의 주인이므로 관절도 그쪽으로 보낸다.
    jcmd = node.create_publisher(JointState, "/arm/joint_command", 10)
    jres: dict = {"msg": None}
    node.create_subscription(String, "/arm/joint_command_result",
                              lambda m: jres.__setitem__("msg", m.data), 10)
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

    def joints_now(timeout: float = 5.0) -> dict[str, float] | None:
        """/joint_states 최신 한 장을 도(°)로. 안 오면 None."""
        latest_js["msg"] = None
        end = time.monotonic() + timeout
        while latest_js["msg"] is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        js = latest_js["msg"]
        if js is None:
            return None
        return {n: math.degrees(v) for n, v in zip(js.name, js.position)}

    def run_prep() -> dict:
        """시험 전에 자세를 본다. 가드 안이면 **뻗고 나서** 시험한다.

        ⚠ 걷는 것은 `escape.walk`가 한다 — 걸음마다 실제 자세를 되읽어 남은
        길을 다시 짠다. 처음에 짠 걸음들을 그대로 보내면 처짐이 얹혀 두 걸음째가
        바로 거절당한다(T56). 자세 거절 없이 통과한 걸음만 살린다.
        """
        now = joints_now()
        out = {"needed": None, "extended": False, "detail": ""}
        if now is None:
            out["detail"] = "/joint_states 응답 없음 — 뻗기 판단 불가"
            print(f"  prep: {out['detail']}")
            return out
        geom_prep, _ = _arm_node_geometry()
        r_now = round(kin.signed_radius(now, geom_prep), 1)
        out["signed_r_before"] = r_now
        needed = es.needs_escape(now, geom_prep)
        out["needed"] = needed
        if not needed:
            out.update(extended=True, detail=f"이미 가드 밖 (signed_r {r_now}mm >= {es.GUARD_MM:.0f}mm)")
            print(f"  prep: {out['detail']}")
            return out

        if limits is None:
            out["detail"] = "관절 가동범위(limits) 없음 — 안전한 뻗기 경로를 짤 수 없다"
            print(f"  prep: {out['detail']}")
            return out

        planned = es.plan_escape(now, limits.norms, geom=geom_prep, limit=limits.limit)
        out["planned_steps"] = len(planned["steps"])
        if planned["blocked"]:
            out["detail"] = (f"뻗기 계획 실패 — "
                             f"{planned['blocked']}번째: "
                             + " ".join(planned["steps"][planned["blocked"][0] - 1]["notes"]))
            return out
        print(f"  prep: signed_r {planned['signed_r']}mm < {es.GUARD_MM:.0f}mm — "
              f"{len(planned['steps'])}걸음으로 뻗는다"
              + (f" (목표 자름: {planned['clamped']})" if planned["clamped"] else ""))
        # ⚠ 짜 둔 걸음들을 **그대로 차례로 보내면 두 걸음째부터 거절된다** —
        #   받는 쪽(arm_source.move_joints_deg)은 지금 실제 자세에서 다시 재는데,
        #   한 걸음(12°) 뒤 팔은 중력으로 4~5° 처져 있어 다음 목표가
        #   12+4.3=16.3°가 되기 때문이다(사이클42 실기 기록의 그 숫자, T56).
        #   되읽고 다시 짜는 일은 escape.walk 하나가 한다(ROS 쪽과 같은 규칙).
        class _PrepStop(Exception):
            """뻗기를 더 못 잇는 이유 — walk 바깥으로 그대로 올린다."""

        def measure() -> dict:
            got = joints_now()
            if got is None:
                raise _PrepStop("/joint_states가 안 온다 — 자세를 몰라 멈춘다")
            return got

        def send(degs: dict) -> None:
            jres["msg"] = None
            msg = JointState()
            msg.name = list(degs.keys())
            msg.position = [math.radians(v) for v in degs.values()]
            jcmd.publish(msg)
            end = time.monotonic() + 5.0
            while jres["msg"] is None and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            answer = jres["msg"]
            if answer is None:
                raise _PrepStop("/arm/joint_command 응답 없음(타임아웃)")
            if not answer.startswith("ok|"):
                raise _PrepStop("걸음 거절 — " + answer.split("|", 1)[-1])

        try:
            walked = es.walk(planned["target"], measure=measure, send=send,
                             to_norm=limits.norms, geom=geom_prep, limit=limits.limit)
        except _PrepStop as stop:
            got = joints_now()
            out["detail"] = f"뻗다가 멈췄다 — {stop}"
            if got is not None:
                out["signed_r_after"] = round(kin.signed_radius(got, geom_prep), 1)
            print(f"  prep: {out['detail']}")
            return out
        r_after = round(kin.signed_radius(walked["last"], geom_prep), 1)
        # **"뻗었다"의 뜻은 '가드를 넘었다'**이지 '걸음을 다 보냈다'가 아니다.
        # 처짐 때문에 목표에 1~2° 못 미쳐도 가드 밖이면 시험은 돌아가고, 반대로
        # 걸음을 다 보내고도 가드 안이면 5회가 전부 거절당한다 — 그 거짓 0/5를
        # 기록에서 가려내는 것이 이 표지의 일이다.
        out.update(extended=r_after >= es.GUARD_MM, signed_r_after=r_after,
                   sent=walked["sent"], gap_deg=walked["gap_deg"],
                   detail=(f"{walked['detail']} — signed_r "
                           f"{out.get('signed_r_before')} → {r_after}mm"))
        print(f"  prep: {out['detail']}")
        return out

    geom, geom_tag = _arm_node_geometry()
    if prep:
        prep_row = run_prep()
    else:
        prep_row = {"needed": None, "extended": False,
                    "detail": "--no-prep — 뻗기를 껐다(자세는 부르는 쪽 책임이다)"}
    prep_failed = bool(prep_row.get("needed")) and not prep_row.get("extended")
    if prep_failed and not force:
        # **실패와 '시험 자체가 성립 안 함'은 다른 사실이다**(T59). 옛 동작은
        # 여기서 그대로 5회를 돌려 전부 자세 가드에 거절당한 거짓 0/5를 남겼다
        # (사이클42) — 그 0/5가 기준3의 점수처럼 읽혀 다음 사이클을 속였다.
        # 팔을 움직이지 않고 이유 한 줄만 남긴 뒤 rc로 실패를 알린다.
        detail = f"prep 실패: {prep_row['detail']} — 5회를 돌리지 않았다(--force로 강행)"
        print(f"  ❌ {detail}")
        record(out_path, {"trial": 0, "dry_run": False, "invalid": True,
                          "stage": "no-prep", "ok": False, "error_mm": None,
                          "reached": None, "limits": limits_tag,
                          "load_limits": load_tag, "geometry": geom_tag,
                          "graduation_blocked": graduation_blocked,
                          "prep": prep_row, "joints_cmd": None,
                          "joints_actual": None, "detail": detail})
        rclpy.shutdown()
        return -2
    if prep_failed:
        print(f"  ⚠ prep 실패: {prep_row['detail']} — --force로 그래도 5회를 그대로 시험한다"
              "(자세 가드 거절이 기록에 남는다)")

    ok_count = 0
    for i, p in enumerate(points, 1):
        # arm_node가 안에서 풀 값과 같은 것 — 걸음이 이 지령대로 안 됐다면
        # joints_actual과 나란히 놓고 어느 관절이 못 버텼는지 뒤에서 읽는다.
        joints_cmd = commanded_joints(p, geom)
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
               "standoff_mm": STANDOFF_MM, "limits": limits_tag,
               # 뽑을 때 **들 수 있는 자리인지 알고 있었나**(T41). none이면 그
               # 실패는 팔의 실패가 아니라 도구가 고른 자리의 실패일 수 있다.
               "load_limits": load_tag,
               # **어떤 팔 길이로** 뽑고 쟀는지(T53). 뽑는 쪽과 재는 쪽이 갈리면
               # 점수 자체가 못 믿을 것이 되므로 그 사실이 줄에 남아야 한다.
               "geometry": geom_tag,
               "graduation_blocked": graduation_blocked,
               # 이 시험이 **어떤 시작 자세에서** 출발했는지 남긴다 — 기준3의
               # 0/5가 팔의 0인지 시작 자세의 0인지 나중에 가릴 수 있어야 한다.
               "prep": prep_row,
               "joints_cmd": joints_cmd}
        if res is None:
            row.update(ok=False, stage=classify_stage(None), error_mm=None,
                       reached=None, joints_actual=None, detail="응답 없음(타임아웃)")
            record(out_path, row)
            print(f"  trial {i}: 응답 없음")
            continue
        if not res.ok:
            stage = classify_stage(res.detail)
            row.update(ok=False, stage=stage, error_mm=None, reached=None,
                       joints_actual=None, detail=res.detail)
            record(out_path, row)
            print(f"  trial {i}: FAIL[{stage}] {res.detail}")
            continue
        spin(2.0)   # 이동 정착 대기
        js = latest_js["msg"]
        if js is None:
            row.update(ok=False, stage="joint", error_mm=None, reached=None,
                       joints_actual=None,
                       detail="/joint_states가 안 온다 — 관절을 못 읽었다")
            record(out_path, row)
            print(f"  trial {i}: FAIL[joint] /joint_states 없음")
            continue
        degs = {n: math.degrees(v) for n, v in zip(js.name, js.position)}
        joints_actual = {j: round(degs[j], 2) for j in kin.JOINTS if j in degs}
        pose = kin.forward(degs, geom)   # 뽑을 때와 같은 기하다 (T53)
        target_reached_mm = (res.reached.x * 1000.0, res.reached.y * 1000.0,
                              res.reached.z * 1000.0)
        err = math.dist((pose.x, pose.y, pose.z), target_reached_mm)
        success = err <= 15.0
        row.update(ok=success, stage=None if success else "joint",
                   error_mm=round(err, 2), reached=pose.as_dict(),
                   joints_actual=joints_actual, detail=res.detail)
        record(out_path, row)
        print(f"  trial {i}: ok={success} err={err:.2f}mm")
        if success:
            ok_count += 1
    rclpy.shutdown()
    return ok_count


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--record", action="store_true",
                    help="연습(--dry-run)도 진짜 시험기록에 남긴다 (기본은 "
                         "임시 연습 자리 — 연습 한 번이 기록을 오염시키지 "
                         "않게)")
    ap.add_argument("--points", default="")
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-prep", action="store_true",
                    help="시험 전 자동 뻗기를 끈다 (기본은 켬 — 자세 가드에 걸린 "
                         "자세에서 그냥 돌리면 5/5가 거절되고 거짓 0/5가 기록된다)")
    ap.add_argument("--force", action="store_true",
                    help="prep이 실패해도 5회를 그대로 시험한다(옛 동작). 기본은 "
                         "돌리지 않고 '시험 못 함'을 기록한다(T59) — 자세 가드 거절 "
                         "5줄이 기준3의 0/5처럼 읽히는 것을 막는다")
    args = ap.parse_args()

    # 표적을 뽑는 기하는 **재는 기하와 같은 함수에서 와야 한다**(T53) —
    # arm_node가 믿는 길이로 뽑지 않으면 우리가 고른 자리가 그 팔의 자리가 아니다.
    geom, geom_tag = _arm_node_geometry()
    print(f"팔 기하: {geom_tag} (l1={geom.l1} l2={geom.l2} l3={geom.l3})")
    # 이 팔이 **실제로 갈 수 있는 범위**. 팔을 열지 않고 파일에서 읽는다
    # (포트는 ROS가 쥐고 있다). 못 읽으면 None이고, 그 사실이 기록에 남는다.
    limits, limits_note = cart.load_norm_limits()
    limits_tag = limits.source if limits else "none"
    print(f"관절 가동범위: {limits_note}")
    # **들 수 있는 자리**의 경계는 또 다른 한계다(§24). 이것도 파일에서만 온다 —
    # 못 읽으면 거르지 않고 기록에 load_limits=none이 남는다.
    load, load_note = ld.load_load_limits()
    load_tag = load.source if load else "none"
    print(f"드는 한계: {load_note}")
    # arm_load_limits.json 없이 실행되면 이 판은 졸업 인정 불가다(T71, OBJECTIVE.md).
    # 코드 기본값(config.ARM_LOAD_*)이나 none으로 통과한 5/5는 '임의의 자리'를 좁혀
    # 얻은 거짓 졸업이 될 수 있다.
    graduation_blocked = (load is None) or load_tag.startswith("config.")
    if graduation_blocked:
        print("  경고: arm_load_limits.json 없음 — 이 판은 기준3 졸업 인정 불가")

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
        if load:
            # 사람이 고른 자리도 **드는지**는 봐 준다 — 막지는 않는다(위와 같은 이유).
            for i, p in enumerate(points, 1):
                stand_pose = standoff_pose(p)
                over = load.rejects(stand_pose.x, stand_pose.y, stand_pose.z)
                if over:
                    print(f"  ⚠ 표적 {i}는 팔이 못 드는 자리로 보인다 — {over}")
    else:
        points = sample_points(args.n, geom, args.seed, limits=limits, load=load)

    today = time.strftime("%Y-%m-%d")
    out_dir, out_why = record_dir_for(args.dry_run, args.record)
    out_path = os.path.join(out_dir, f"move-to-point-{today}.jsonl")
    # 이 판이 어디에 떨어지는지는 **줄에도** 남아야 한다(T58) — 연습 자리는
    # 저장소 밖이지만 젯슨의 volatile과 뜻이 다르다(회수할 것이 없다).
    # 그래서 세 갈래에 "practice"를 하나 더 둔다. 못 쓰는 자리만은 그대로
    # unwritable로 둬서 아래의 정지가 연습에서도 먹게 한다.
    global RECORD_HOME, RECORD_HOME_WHY
    RECORD_HOME, RECORD_HOME_WHY = record_home(out_dir)
    practice = out_dir == DRY_RECORD_DIR
    if practice and RECORD_HOME != "unwritable":
        RECORD_HOME = "practice"
        RECORD_HOME_WHY = f"{out_dir} — 연습 자리(커밋되지 않고 회수할 것도 없다)"

    print(f"{len(points)}개 표적, 기록: {out_path}  [{local_zone()}]  "
          f"[{RECORD_HOME}]")
    if practice:
        print(f"   ↳ {out_why}")
    # **못 쓰는 자리면 팔을 움직이기 전에 멈춘다.** 기록 없는 5회는 다음 사이클에
    # 아무것도 아니다 — 팔만 움직이고 사실은 남지 않는다(T58).
    if RECORD_HOME == "unwritable":
        print(f"❌ 기록을 쓸 수 없다 — {RECORD_HOME_WHY}")
        print("   TOMATO_RECORD_DIR을 쓸 수 있는 자리로 지정하고 다시 돌려라.")
        return 2
    if RECORD_HOME == "volatile":
        print(f"⚠ 이 기록은 커밋되지 않는다 — {RECORD_HOME_WHY}")

    if args.dry_run:
        ok = run_dry(points, geom, out_path, limits_tag, load_tag, geom_tag,
                     graduation_blocked=graduation_blocked)
    else:
        ok = run_real(points, out_path, limits_tag, limits=limits,
                      prep=not args.no_prep, load_tag=load_tag, force=args.force,
                      graduation_blocked=graduation_blocked)
        if ok == -2:
            print("\n시험 못 함 — prep 실패로 5회를 돌리지 않았다(--force로 강행 가능)")
            return 1
        if ok < 0:
            return 1

    print(f"\n{ok}/{len(points)} 성공")
    if RECORD_HOME == "volatile":
        # 여기서 끝내면 이 줄들은 젯슨에서 늙다가 사라진다. 그래서 **가져오는
        # 명령 한 줄**을 그대로 찍는다. scp가 아니라 record_pull인 이유: 같은
        # 이름의 파일이 두 곳에서 따로 자라서(실측 저장소 57줄 / 젯슨 20줄)
        # 덮어쓰면 한쪽을 잃는다 — 붙이기만 하는 도구가 따로 있다.
        print("\n⚠ 이 기록은 저장소 밖에 있다 — PC에서 아래를 돌려 가져와라:")
        print(f"     python ros2/tools/record_pull.py --date {today}")
    return 0 if ok == len(points) else 1


if __name__ == "__main__":
    raise SystemExit(main())
