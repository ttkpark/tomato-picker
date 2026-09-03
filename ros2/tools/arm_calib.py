#!/usr/bin/env python3
"""**관절 한계 판정은 여기 하나뿐이다** — 정규화값(서보에 보내는 -100..100)
↔ 도(度) 변환과 "이 자세로 가도 되는가"를 계산한다.

────────────────────────────────────────────────────────────────────────
왜 만들었나

`tool_jog.py`·`grasp_probe.py`·`stem_grasp.py`·`arm_stage.py`·`cam_frame.py`가
이 계산을 각자 복붙해 갖고 있었다. 처음엔 같은 코드였는데 조금씩 다르게
고쳐지며 어긋났다 — `cam_frame.py`는 정규값 한계로 96을 썼고 나머지는 98을
썼다. 그 결과 조작대 오른쪽 패널(`click_server.py`의 3D 미리보기·관절값
직접편집, 이것도 같은 계산을 자바스크립트로 다시 짠 것)은 "갈 수 있다"는데
`tool_jog.py`의 조그 버튼은 거절하는 일이 2026-09-03에 실제로 있었다.

그래서 **정규값 ±98 같은 임의 숫자 대신, 서보 캘리브레이션(`range_min`/
`range_max`)에서 나온 진짜 물리 가동범위**를 쓴다 — 조작대 패널이 쓰는 것과
정확히 같은 공식(`joint_range`)이다. 거기서 3%만 여유로 뺀다(`LIMIT_MARGIN`).
"지금 자리가 이미 한계를 넘었으면 한 발짝도 더 못 간다"는 예전의 되돌이
조항(ratchet)도 없앴다 — 진짜 범위를 쓰면 지금 자리는 애초에 범위 안에
있으므로 그 조항이 필요 없다.
"""

from __future__ import annotations

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))

from tomato_picker.hardware import kinematics as kin       # noqa: E402

DEG_PER_TICK = 360.0 / 4096.0
CAL = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/tomato_follower.json")
CART = os.path.expanduser("~/arm_cartesian.json")
# 팔 base(마운트)가 바닥보다 이만큼 위 — 실측
# (`ros2/src/tomato_description/config/so101_geometry.yaml`의 mount.z와 같은 값).
MOUNT_Z_MM = 76.5
FLOOR_MARGIN_MM = 10.0
LIMIT_MARGIN = 0.03          # 물리 가동범위(정규 ±100) 끝에서 이 비율만큼은 안 쓴다


class Calib:
    """`tomato_follower.json`(서보 틱 범위) + `arm_cartesian.json`(영점·부호·
    축척)을 한 번 읽어, 도(度)↔정규값 변환과 관절 한계 판정을 준다."""

    def __init__(self, cal_path: str = CAL, cart_path: str = CART):
        cal = json.load(open(cal_path))
        self.spans: dict[str, float] = {}
        for name, c in cal.items():
            try:
                self.spans[name] = abs(int(c["range_max"]) - int(c["range_min"])) * DEG_PER_TICK
            except (KeyError, TypeError, ValueError):
                pass
        cart = json.load(open(cart_path))
        self.zero = cart["zero"]
        self.ref = cart["ref_deg"]
        self.signs = cart.get("signs", {})
        self.over = cart.get("deg_per_norm") or {}
        self.geom = kin.ArmGeometry()
        self._ranges: dict[str, tuple[float, float] | None] = {}

    def sign(self, j: str) -> float:
        v = self.signs.get(j)
        return -1.0 if (v is not None and float(v) < 0) else 1.0

    def per(self, j: str) -> float:
        v = self.over.get(j)
        if v:
            return abs(float(v))
        s = self.spans.get(j)
        return abs(s) / 200.0 if s else (1.8 if j == "wrist_roll" else 0.9)

    def to_deg(self, n: dict) -> dict:
        return {j: self.ref.get(j, 0.0) + self.sign(j) *
                (float(n.get(j, 0.0)) - self.zero.get(j, 0.0)) * self.per(j)
                for j in kin.JOINTS}

    def to_norm(self, d: dict) -> dict:
        return {j: self.zero.get(j, 0.0) + (float(d[j]) - self.ref.get(j, 0.0)) /
                (self.sign(j) * self.per(j)) for j in d if j in kin.JOINTS}

    def joint_range(self, j: str) -> tuple[float, float] | None:
        """실제 물리 가동범위(도) — 정규값 ±100에 대응. click_server.py의
        3D 미리보기·관절값 직접편집이 쓰는 것과 같은 공식이다."""
        if j in self._ranges:
            return self._ranges[j]
        s = self.spans.get(j)
        if not s:
            self._ranges[j] = None
            return None
        z, r, p, sg = self.zero.get(j, 0.0), self.ref.get(j, 0.0), self.per(j), self.sign(j)
        d100, dm100 = r + sg * (100.0 - z) * p, r + sg * (-100.0 - z) * p
        lim = (min(d100, dm100), max(d100, dm100))
        self._ranges[j] = lim
        return lim

    def legal(self, d: dict, cur: dict, margin: float = LIMIT_MARGIN):
        """목표 자세 d(도)가 관절 한계·바닥 안인가. cur(도)는 바닥 판정에만 쓴다
        (지금 높이보다 더 못 내려가게)."""
        for j, v in d.items():
            lim = self.joint_range(j)
            if lim is None:
                continue
            lo, hi = lim
            m = (hi - lo) * margin
            band_lo, band_hi = lo + m, hi - m
            if band_lo <= v <= band_hi:
                continue
            # ⚠ **이미 여유 밖(중력처짐 등)인 관절을 그 자리에 가두면 안 된다.**
            #   2026-09-03 실측: shoulder_lift가 -15.0°로 하한(-14.6°)을 살짝
            #   넘은 채 서 있었더니, 그 관절을 안 건드리는 dz·dy·피치 요청까지
            #   전부 "shoulder_lift 한계"로 막혔다 — 팔이 통째로 얼어붙었다.
            #   더 나빠지지만(여유 밖으로 더 나가지만) 않으면 통과시킨다.
            cv = float(cur.get(j, v))
            exc_v = max(band_lo - v, v - band_hi, 0.0)
            exc_c = max(band_lo - cv, cv - band_hi, 0.0)
            if exc_v > exc_c + 1e-6:
                return False, "%s 한계" % j
        floor = min(-MOUNT_Z_MM + FLOOR_MARGIN_MM, kin.forward(cur, self.geom).z - 1.0)
        if kin.forward(d, self.geom).z < floor:
            return False, "바닥"
        return True, ""
