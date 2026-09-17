"""**되먹임으로 처짐을 지운다** — 마지막 걸음의 "한 번 쓰고 끝"을 고치는 자리.

    지령을 보낸다 → 실제를 읽는다 → **직전 지령에 오차를 얹는다** → 다시 읽는다 → …

        지령 ← 지령 + (목표 - 실제)      ⚠ `목표 + (목표-실제)`가 아니다

⚠ **같은 값을 다시 보내는 것과 다르다.** 서보는 중력이 누르는 만큼 못 미친
   자리에서 **멈춰 있다**(정상상태). 같은 지령을 또 보내면 그 자리에 그대로
   선다 — 2026-09-18 실기에서 `cartesian`의 "걸음마다 다시 짜기"가 32걸음
   예산을 다 태운 것이 그것이다.
⚠ **보정을 내려놓아도 안 된다.** 매회 `목표 + (목표-실제)`로 다시 세우면 직전에
   얹은 초과지령이 사라지고 중력이 도로 끌어내린다 — 2026-09-18 실기 실측
   4.83° → 1.23° → **1.67° → 2.21°**(되돌아갔다). 이 팔은 목표에 서 있으려면
   그 자세에서 ~5°의 초과지령을 **계속 물고 있어야** 한다.

이 파일에 **팔도 포트도 없다.** 부르는 쪽이 `measure`(읽기)와 `send`(쓰기)를
주고, 여기서는 **언제 멈출지**만 정한다. 그래서 PC에서 전부 검증된다
(`tools/arm_cartesian_check.py`의 ⑨).

────────────────────────────────────────────────────────────────────────
멈추는 규칙이 셋인 이유 (docs/인수인계-2026-09-04.md §22-1 실측)

  ① **수렴** — 최대 오차가 `stop_deg` 아래로 내려왔다. 다 지웠다.
  ② **예산** — `rounds`회를 다 썼다. 실측은 3회로도 0.5°에 못 닿았다
     (4.92° → 3.70 → 3.50 → 2.84°). 그래도 **덜 지운 채로 멈추는 것**이
     영원히 도는 것보다 낫다.
  ③ **포화** — 연속 `stall_rounds`회 동안 직전 대비 `min_gain`만큼도 안 줄었다.
     정지마찰·백래시 성분은 되먹임으로 안 지워진다. 안 줄어드는데 계속 미는 것은
     서보를 Overload로 미는 짓이다.

⚠ **소프트 한계에 눌린 관절은 되먹임 대상에서 뺀다.** §22-1에서 `elbow_flex`는
   정규화 -98에 이미 잘려 있었다 — "지령"만 매회 멀어지고 실제는 굳은 채라
   보고되는 오차가 **지울수록 커졌다.** 그건 처짐이 아니라 **애초에 못 보낸
   지령**이다. 그걸 오차로 세면 포화 판정이 거짓말을 한다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from ..config import (
    ARM_CART_MIN_JOINT_DEG,
    ARM_SETTLE_MIN_GAIN,
    ARM_SETTLE_ROUNDS,
    ARM_SETTLE_STALL_ROUNDS,
    ARM_SETTLE_STOP_DEG,
)
from .kinematics import wrap180

# 멈춘 이유 — 기록(jsonl)에 그대로 들어간다. 사람이 읽는 말은 아래 REASON_KO.
CONVERGED = "converged"
BUDGET = "budget"
SATURATED = "saturated"
UNDELIVERABLE = "undeliverable"
OFF = "off"

REASON_KO = {
    CONVERGED: "수렴(문턱 아래)",
    BUDGET: "예산 소진",
    SATURATED: "포화(더 안 줄어듦)",
    UNDELIVERABLE: "보낼 수 없는 크기(분해능 아래이거나 전부 한계에 눌림)",
    OFF: "되먹임 꺼짐",
}


@dataclass(frozen=True)
class SettleConfig:
    """세 숫자는 `config.py`에서 온다 — 여기에 박지 않는다."""

    rounds: int = ARM_SETTLE_ROUNDS
    stop_deg: float = ARM_SETTLE_STOP_DEG
    min_gain: float = ARM_SETTLE_MIN_GAIN
    stall_rounds: int = ARM_SETTLE_STALL_ROUNDS
    # 서보 분해능 아래 보정은 보내도 0이다(이 저장소의 1번 병) — 보냈다고 말하지 않는다.
    min_cmd_deg: float = ARM_CART_MIN_JOINT_DEG


@dataclass
class SettleRound:
    """한 회차 = 한 번 읽은 결과. 0회차는 **되먹임 전**(도착 직후)이다."""

    n: int
    err_deg: float
    worst: str
    gain: float | None = None          # 직전 대비 줄어든 비율(0회차는 None)
    clamped: tuple[str, ...] = ()      # 이 회차에 한계에 눌려 못 보낸 관절

    def as_record(self) -> dict:
        row = {"n": self.n, "err_deg": round(self.err_deg, 3), "worst": self.worst}
        if self.gain is not None:
            row["gain"] = round(self.gain, 4)
        if self.clamped:
            row["clamped"] = list(self.clamped)
        return row


@dataclass
class SettleResult:
    stop: str
    start_deg: float                   # 되먹임 전 최대 오차
    err_deg: float                     # 마지막에 남은 최대 오차
    rounds: list[SettleRound] = field(default_factory=list)
    clamped: tuple[str, ...] = ()      # 끝까지 한계에 눌려 있던 관절
    sent: int = 0                      # 실제로 보낸 되먹임 지령 수
    restored: bool = False             # 나빠져서 가장 좋았던 지령으로 되돌렸는가

    @property
    def improved_deg(self) -> float:
        return self.start_deg - self.err_deg

    def as_record(self) -> dict:
        """기록(jsonl) 한 칸. 횟수와 최종 오차가 반드시 들어간다."""
        return {
            "stop": self.stop,
            "sent": self.sent,
            "start_err_deg": round(self.start_deg, 3),
            "err_deg": round(self.err_deg, 3),
            "clamped": list(self.clamped),
            "restored": self.restored,
            "rounds": [r.as_record() for r in self.rounds],
        }

    def describe(self) -> str:
        tail = f" · 한계에 눌림 {','.join(self.clamped)}" if self.clamped else ""
        if self.restored:
            tail += " · 나빠져서 되돌림"
        return (f"되먹임 {self.sent}회 — 오차 {self.start_deg:.2f}° → {self.err_deg:.2f}° "
                f"({REASON_KO.get(self.stop, self.stop)}){tail}")


def joint_error(desired: dict[str, float], got: dict[str, float],
                skip: tuple[str, ...] | set[str] = ()) -> tuple[float, str]:
    """가장 많이 어긋난 관절과 그 크기(°). `skip`은 세지 않는다."""
    worst, name = 0.0, ""
    for j, want in desired.items():
        if j in skip:
            continue
        err = abs(wrap180(float(want) - float(got.get(j, want))))
        if err >= worst:
            worst, name = err, j
    return worst, name


def settle(desired: dict[str, float],
           measure: Callable[[], dict[str, float]],
           send: Callable[[dict[str, float]], None],
           *,
           deliver: Callable[[dict[str, float]], dict[str, float]] | None = None,
           cfg: SettleConfig | None = None) -> SettleResult:
    """`desired`(관절각 °)에 실제로 서게 될 때까지 **오차만큼 더** 준다.

    measure() → 지금 관절각(°) · send(지령 °) → 그 지령을 보내고 멈출 때까지 기다림.
    deliver(지령) → **실제로 보낼 수 있는 지령**(소프트 한계·안전 검사를 거친 값).
      돌려준 값이 달라진 관절은 "한계에 눌린" 것으로 보고 **그 관절은 뺀다** —
      뺀 뒤에도 남은 관절이 없으면 보낼 것이 없는 것이다.

    팔을 안 움직이는 경우에도 **반드시 0회차 측정은 한다** — "되먹임을 껐다"와
    "오차가 없었다"를 기록에서 가를 수 있어야 한다.
    """
    cfg = cfg or SettleConfig()
    clamped: set[str] = set()
    got = dict(measure())
    err, worst = joint_error(desired, got)
    res = SettleResult(stop=OFF, start_deg=err, err_deg=err,
                       rounds=[SettleRound(0, err, worst)])
    if cfg.rounds <= 0:
        return res

    # 마지막 걸음이 실제로 보낸 지령 = 목표 그 자체다. 여기서부터 쌓는다.
    cmd = {j: float(v) for j, v in desired.items()}
    # 가장 좋았던 지령 — 처음에는 "마지막 걸음이 보낸 것"(=목표) 그 자체다.
    best_err, best_cmd = res.err_deg, dict(cmd)
    stalled = 0
    while True:
        if res.err_deg <= cfg.stop_deg:
            res.stop = CONVERGED
            break
        if res.sent >= cfg.rounds:
            res.stop = BUDGET
            break
        if stalled >= cfg.stall_rounds:
            res.stop = SATURATED
            break

        # 오차만큼 **더** 준다 — 단 **직전 지령 위에 얹는다**(누적).
        # ⚠ `목표 + (목표-실제)`로 매회 다시 세우면 **직전에 얹은 보정이 사라진다.**
        #   2026-09-18 실기(젯슨, B자세)가 그것을 가렸다: 4.83° → 1.23°(+74.6%)로
        #   좋아진 뒤 1.67° → 2.21°로 **되돌아갔다.** 이 팔은 목표에 서 있으려면
        #   그 자세에서 늘 ~5°의 초과지령을 **계속 물고 있어야** 한다 — 보정을
        #   내려놓는 순간 중력이 도로 끌어내린다. 그래서 바탕은 목표가 아니라
        #   **직전에 실제로 보낸 지령**이다(§22-1의 `지령 = 지령 + (지령-실제)`).
        want = {}
        for j, tgt in desired.items():
            tgt = float(tgt)
            if j in clamped:
                want[j] = float(cmd[j])     # 더 밀지 않는다(와인드업 방지)
                continue
            want[j] = float(cmd[j]) + wrap180(tgt - float(got.get(j, tgt)))
        sendable = dict(deliver(want)) if deliver else dict(want)

        # deliver가 깎은 관절 = 그 방향으로 더 못 간다 → 이번부터 오차에서 뺀다.
        newly = tuple(sorted(
            j for j, v in sendable.items()
            if j not in clamped and abs(float(v) - float(want[j])) > 1e-6))
        clamped.update(newly)

        # 남은(눌리지 않은) 관절 중 **실제로 움직일 크기**가 있는가.
        step = max((abs(wrap180(float(sendable[j]) - float(got.get(j, sendable[j]))))
                    for j in sendable if j not in clamped), default=0.0)
        if step < cfg.min_cmd_deg:
            res.stop = UNDELIVERABLE
            if newly:
                res.rounds[-1].clamped = tuple(sorted(clamped))
            break

        send(sendable)
        cmd = {j: float(v) for j, v in sendable.items()}   # 다음 회차가 이 위에 얹는다
        res.sent += 1
        before = got
        got = dict(measure())
        # ⚠ 직전 오차도 **지금의 skip으로 다시 재서** 비교한다 — 이번에 한계로
        #   빠진 관절 때문에 "갑자기 좋아진 것처럼" 보이면 포화 판정이 거짓말을 한다.
        prev = joint_error(desired, before, skip=clamped)[0]
        err, worst = joint_error(desired, got, skip=clamped)
        gain = (prev - err) / prev if prev > 1e-9 else 0.0
        res.rounds.append(SettleRound(res.sent, err, worst, gain,
                                      tuple(sorted(clamped))))
        res.err_deg = err
        if err < best_err:
            best_err, best_cmd = err, dict(sendable)
        stalled = stalled + 1 if gain < cfg.min_gain else 0

    # ⚠ **나빠진 채로 끝내지 않는다.** 2026-09-18 실기(A자세): 1.17° → 0.61°로
    #   좋아진 뒤 1.16° → 1.70°로 되돌아갔다(한 관절을 고치면 다른 관절이 딸려
    #   움직인다 — 5축이 서로 묶여 있다). 되먹임이 팔을 **더 틀어 놓고** 끝나면
    #   안 쓰느니만 못하다. 되돌리는 것은 **한 번뿐**이다(여기서 또 재면 루프가 된다).
    if res.sent and res.err_deg > best_err + cfg.min_cmd_deg:
        send(best_cmd)
        res.sent += 1
        res.restored = True
        res.err_deg = joint_error(desired, dict(measure()), skip=clamped)[0]
        res.rounds.append(SettleRound(res.sent, res.err_deg, "되돌림",
                                      clamped=tuple(sorted(clamped))))
    res.clamped = tuple(sorted(clamped))
    return res
