#!/usr/bin/env python3
"""**잡기 수련장** — 물리값이 전부 다른 팔 수천 대를 GPU에서 동시에 돌려 다가가기·닫기 정책을 배운다 (PC, 장치 없이).

    python ros2/tools/grasp_gym.py selftest
    python ros2/tools/grasp_gym.py bench            # 이 GPU의 처리량
    python ros2/tools/grasp_gym.py calib            # 레시피가 실기 성공률에 맞는가
    python ros2/tools/grasp_gym.py train --hours 10 # ~/grasp_gym/ 에 체크포인트·로그·report.md (다시 부르면 이어 한다)

팔은 arm_twin과 같은 준정적 모형(평형 = 목표 − 중력토크/K, ±f 안이면 안 움직임, 틱 분해능)에 tool_jog의
press_to(도착할 때까지 밀어 넣기)와 --horiz/--dz/--left 걸음을 그대로 옮겼다. 카메라는 그림이 아니라 "줄기 위치 추정값 +
잡음"이다 — 화면·접촉 물리·검출기 자체는 없다. 범위(RANGES)는 **아직 실측 전 초안**이다: 실측 후 고치고 다시 돌린다.
⚠ 여기 숫자는 실기 결과가 아니다. 체크포인트 판정은 실기로만 한다.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import torch
import torch.nn as nn

OUT = os.environ.get("GRASP_GYM_OUT") or os.path.expanduser("~/grasp_gym")
# 옮긴 자리에서 모형이 설명 못 하는 몫: 실기 (9+15)/(14+20)=71% ÷ 같은 레시피 모형 89%. 편향을 키워도 83% 밑으로 안 내려갔다(11/07).
# 실기 기록은 11/03 수정 전이라 이 값도 잠정 — 수정 뒤 옮긴 자리 20회를 재면 바꾼다.
REAL_GAP = 0.80
DEG = math.pi / 180.0
TICK = 360.0 / 4096.0 * DEG
Z0, D0, L1, L2, L3 = 119.5, -31.5, 116.5, 138.0, 168.0     # kinematics.ArmGeometry 실측값
START = (-7.1, 75.5, 132.2, -28.8)                         # grasp_repeat.START(도), FK r≈−292
PITCH = sum(START[1:])                                     # 집게 피치를 지킨다(tool_jog keep)
LIM = ((-107.7, 135.7), (5.9, 211.6), (-41.0, 165.7), (-102.1, 105.7))  # arm_calib.joint_range(3% 여유 전, 젯슨 11/06 값)
MAX_REACH = 385.0                                          # tool_jog.MAX_REACH_MM
M1, M_ELBOW, M2, M_HAND, HAND_D = 0.06, 0.055, 0.115, 0.21, 70.0   # arm_twin과 같다
CAM_BACK, LOCK_Z, T_LIMIT = 75.0, 128.0, 60.0              # 손목 D405↔TCP · 맨 줄기 근접 기준 · 규약 상한

# 이름: (낮음, 높음). 초안 — 출처 표시 없는 칸은 실측이 필요하다.
RANGES = {
    "K": (14.0, 56.0),          # arm_twin 맞춤 28의 0.5~2배
    "f": (0.0, 0.5),            # 도. 맞춤 0.2
    "len": (-0.01, 0.01),       # 링크 길이 믿음 오차
    "sd_depth": (1.0, 8.0),     # mm, 가만히 둔 중앙값 기준(한 프레임은 더 크다)
    "p_out": (0.0, 0.10),       # 깊이 튐 확률(±30mm, 11/01)
    "sd_lat": (0.3, 2.0),
    "sd_up": (0.5, 3.0),
    "couple": (0.2, 0.9),       # 깊이 오차 1mm → 위아래 겨냥 오차 mm (시차표 3px/mm, 11/03)
    "bias_d": (0.0, 6.0),       # 회마다 |편향| 상한
    "bias_l": (0.0, 4.0),
    "bias_u": (0.0, 6.0),
    "p_miss": (0.0, 0.20),      # 프레임 검출 실패
    "p_lost": (0.0, 0.30),      # 그 회 내내 못 찾음(11/06 맨 줄기 13/20 = 0.65, 실제 사진 문제라 학습은 '안 닫기'만 배운다)
    "p_stale": (0.0, 0.10),     # 멈추기 전 프레임
    "near": (1.0, 3.0),         # LOCK_Z 안쪽 잡음 배수(가까이선 기준이 흔들림, 11/04)
    "sway": (0.0, 0.8),         # 걸음마다 줄기 흔들림 mm
    "p_event": (0.0, 0.01),     # 걸음당 토크 끊김 등 순간 낙하
    "gap": (10.0, 16.0),        # 물리는 가로 창 — checkpoint_sim 14
    # 물리는 앞뒤 창(줄기 지나침 mm, 깊이 z=75−0.8×이 값). 실기: z 70 빈손(11/01 손끝 안쪽 목) · 74·80·86 물림(11/02) → 대략 z 72~90
    "d_lo": (-24.0, -14.0),
    "d_hi": (2.0, 8.0),
    "band": (14.0, 24.0),       # 물리는 위아래 창 — checkpoint_sim 20
    "ratio": (0.75, 0.85),      # 전진 1mm당 깊이 감소(믿는 값 0.8)
    # ⚠ 엔코더에 안 보이는 처짐(링크 휨·혼 유격·차체 기울어짐) — 전부 미측정 추정. 카메라도 같이 내려가므로
    #   화면으로 고치는 동안은 상쇄되고, 뻗는 만큼 변하는 몫이 눈 감고 가는 구간에 남는다.
    "flex": (0.0, 4.0),         # 손끝 반경 100mm당 추가 낙하 mm
    "play": (0.0, 1.5),         # 닫는 순간 손끝 위치의 무작위 어긋남 σ mm(카메라↔손끝 사이 유격)
}
CENTER = {k: (a + b) / 2 for k, (a, b) in RANGES.items()}
NAMES = list(RANGES)
PUSH, POT_BELOW = 40.0, 90.0   # 줄기를 이만큼 지나치면·줄기 아래로 이만큼 내려가면 충돌

RUN, OK, EMPTY, WRONG, CRASH, TIMEOUT, LOST = 0, 1, 2, 3, 4, 5, 6
CLS = {OK: "성공", EMPTY: "빈손", WRONG: "엉뚱한 곳", CRASH: "충돌", TIMEOUT: "시간초과", LOST: "못 찾음"}
# 무대: 시작 깊이 가운데, 깊이·옆·위 반폭 mm. 고정 = 11/06 123~127. 규약 = 110~150, 좌우 ±5cm + 회전 ±20°의 옆 몫.
# 3차 = space reach가 REACH_MM 110까지 데려오고, 접근 실패 기준 ±5cm 안에서 넘겨받음.
SCENES = {"고정": (125.0, 0.0, 0.0, 0.0), "규약": (130.0, 20.0, 80.0, 15.0), "3차": (110.0, 50.0, 100.0, 40.0)}
FRAME = 15     # 관측 한 칸 크기
HIST = 4       # 최근 칸 수 — 이력에서 그날 팔 상태를 읽는다
ACT = torch.tensor([40.0, 15.0, 20.0])  # 정책 출력 1.0 = 앞·옆·위 mm


def gravity(q):
    """[N,4] 라디안 → lift·elbow·wrist 중력 토크(arm_twin.gravity와 같은 식). 양수 = 각을 줄이는 쪽."""
    a1 = q[:, 1]
    a2 = a1 + q[:, 2]
    a3 = a2 + q[:, 3]
    c1, c2, c3 = torch.cos(a1), torch.cos(a2), torch.cos(a3)
    wrist = M_HAND * HAND_D * c3
    elbow = wrist + M_HAND * L2 * c2 + M2 * L2 / 2 * c2
    lift = elbow + (M_HAND + M2 + M_ELBOW) * L1 * c1 + M1 * L1 / 2 * c1
    return lift, elbow, wrist


def fk(q, lens):
    """→ x, y, z, 부호 있는 반경 r. 잡는 자세는 r<0(FK −x 쪽)."""
    a1 = q[:, 1]
    a2 = a1 + q[:, 2]
    a3 = a2 + q[:, 3]
    r = D0 + lens[:, 0] * torch.cos(a1) + lens[:, 1] * torch.cos(a2) + lens[:, 2] * torch.cos(a3)
    z = Z0 + lens[:, 0] * torch.sin(a1) + lens[:, 1] * torch.sin(a2) + lens[:, 2] * torch.sin(a3)
    return r * torch.cos(q[:, 0]), r * torch.sin(q[:, 0]), z, r


def ik(x, y, z):
    """믿는 기하로 (x,y,z) + 고정 피치 → 관절, 풀림 여부. r<0 가지."""
    p = PITCH * DEG
    r = -torch.sqrt(x * x + y * y)
    rw = r - D0 - L3 * math.cos(p)
    zw = z - Z0 - L3 * math.sin(p)
    c = (rw * rw + zw * zw - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    ok = c.abs() < 0.999
    e = torch.acos(c.clamp(-0.999, 0.999))
    lift = torch.atan2(zw, rw) - torch.atan2(L2 * torch.sin(e), L1 + L2 * torch.cos(e))
    return torch.stack([torch.atan2(-y, -x), lift, e, p - lift - e], 1), ok


class Gym:
    def __init__(self, n, device, spread=1.0, scene="3차", seed=0, seed_comp=False, fix=None):
        self.n, self.d, self.fix = n, device, fix or {}
        self.spread, self.z_mid, self.hw, self.seed_comp = spread, SCENES[scene][0], SCENES[scene][1:], seed_comp
        self.seen = torch.zeros(n, device=device)
        self.lost = torch.zeros(n, dtype=torch.bool, device=device)
        self.gen = torch.Generator(device).manual_seed(seed)
        self.p = {k: torch.zeros(n, device=device) for k in NAMES}
        self.lens = torch.zeros((n, 3), device=device)
        self.q = torch.zeros((n, 4), device=device)
        self.g = torch.zeros((n, 4), device=device)
        self.stem = torch.zeros((n, 3), device=device)
        self.bias = torch.zeros((n, 3), device=device)
        self.t = torch.zeros(n, device=device)
        self.done = torch.zeros(n, dtype=torch.bool, device=device)
        self.cls = torch.zeros(n, dtype=torch.long, device=device)
        self.valid = torch.zeros(n, device=device)
        self.meas = torch.zeros((n, 3), device=device)    # 깊이 z, 옆, 위 (옆·위 = 줄기가 집게에서 어디)
        self.blocked = torch.zeros(n, device=device)
        self.last = torch.zeros((n, 3), device=device)
        self.steps = torch.zeros(n, device=device)
        self.hist = torch.zeros((n, HIST, FRAME), device=device)
        self.limlo = torch.tensor([v[0] for v in LIM], device=device) * DEG
        self.limhi = torch.tensor([v[1] for v in LIM], device=device) * DEG
        self.reset(torch.ones(n, dtype=torch.bool, device=device))

    def _u(self, n, lo=-1.0, hi=1.0):
        return lo + (hi - lo) * torch.rand(n, device=self.d, generator=self.gen)

    def _n(self, n):
        return torch.randn(n, device=self.d, generator=self.gen)

    def reset(self, m):
        n = int(m.sum())
        if not n:
            return
        for k in NAMES:
            lo, hi = RANGES[k]
            v = CENTER[k] + self._u(n) * (hi - lo) / 2 * self.spread
            self.p[k][m] = v.clamp(min=0.0) if lo >= 0 else v
            if k in self.fix:
                self.p[k][m] = self.fix[k]
        self.lens[m] = torch.tensor([L1, L2, L3], device=self.d) * (1 + self.p["len"][m])[:, None]
        q0 = torch.tensor(START, device=self.d) * DEG
        self.q[m] = q0
        self.g[m] = q0
        self._press(m, q0.expand(self.n, 4))              # 시작 자세에 도착해 있다
        # 표적: 규약(좌우 ±5·앞뒤 ±3cm·회전 ±20°)의 1.5배, 높이 ±3cm, 시작 깊이 100~160mm
        x, y, z, r = fk(self.q, self.lens)
        dz0, lat, up = (self._u(self.n) * w for w in self.hw)
        far = torch.maximum(torch.maximum(dz0.abs() / 50, lat.abs() / 100), up.abs() / 40)   # 시차표는 잰 자리에서 멀수록 어긋난다
        z0 = self.z_mid + dz0
        fwd = (z0 - CAM_BACK) / self.p["ratio"]
        h = self._axes()
        s = torch.stack([x + h[0] * fwd + h[2] * lat, y + h[1] * fwd + h[3] * lat, z + up], 1)
        self.stem[m] = s[m]
        for i, k in enumerate(("bias_d", "bias_l", "bias_u")):
            self.bias[m, i] = self._u(n) * self.p[k][m] * (0.3 + 0.7 * far[m])
        self.lost[m] = self._u(n, 0, 1) < self.p["p_lost"][m]
        self.seen[m] = 0.0
        self.t[m] = 5.0                                   # 클릭·첫 맞춤
        self.done[m] = False
        self.cls[m] = RUN
        self.last[m] = 0.0
        self.steps[m] = 0.0
        self.blocked[m] = 0.0
        self.hist[m] = 0.0
        self.meas[m] = 0.0
        self._observe(m, fresh=True)

    def _axes(self):
        """h(수평 접근)와 left(tool_jog --left와 같은 정의)."""
        hx, hy = -torch.cos(self.q[:, 0]), -torch.sin(self.q[:, 0])   # 피치 ≈179°: 접근축 수평 성분은 −r 방향
        return hx, hy, hy, -hx

    def _err(self):
        """참값: 줄기가 TCP에서 앞·옆·위로 몇 mm."""
        x, y, z, r = fk(self.q, self.lens)
        z = z - self.p["flex"] * r.abs() / 100.0
        vx, vy, vz = self.stem[:, 0] - x, self.stem[:, 1] - y, self.stem[:, 2] - z
        hx, hy, lx, ly = self._axes()
        return vx * hx + vy * hy, vx * lx + vy * ly, vz

    def _settle(self, m):
        k, f = self.p["K"], self.p["f"] * DEG
        for _ in range(3):                                # 토크가 자세에 달려 고정점을 찾는다
            taus = gravity(self.q)
            for i, tau in zip((1, 2, 3), taus):
                eq = self.g[:, i] - tau / k * DEG
                self.q[:, i] = torch.where(m, torch.minimum(torch.maximum(self.q[:, i], eq - f), eq + f), self.q[:, i])
        self.q[:, 0] = torch.where(m, self.g[:, 0], self.q[:, 0])  # 어깨 회전축은 중력이 안 건다

    def _press(self, m, target):
        """tool_jog.press_to: 도착할 때까지 밀어 넣는다(5번, 허용 0.6°, 상한 8°). → 시도 횟수."""
        cmd = target.clone()
        if self.seed_comp:                                # 처짐 보정을 버리지 않고 이어 쓴다(실측 K의 효과 비교용)
            lift, elbow, wrist = gravity(target)
            k = CENTER["K"]
            for i, tau in zip((1, 2, 3), (lift, elbow, wrist)):
                cmd[:, i] = target[:, i] + tau / k * DEG
        tries = torch.zeros(self.n, device=self.d)
        live = m.clone()
        tol, cap = 0.6 * DEG, 8.0 * DEG
        for _ in range(5):
            self.g[live] = (cmd[live] / TICK).round() * TICK
            self._settle(live)
            tries = tries + live.float()
            miss = target - self.q
            live = live & (miss.abs().amax(1) > tol)
            if not live.any():
                break
            cmd = torch.where(live[:, None], torch.minimum(torch.maximum(cmd + miss, target - cap), target + cap), cmd)
        return tries

    def _observe(self, m, fresh=False):
        n = self.n
        ef, el, eu = self._err()
        ztrue = CAM_BACK + self.p["ratio"] * ef
        near = torch.where(ztrue < LOCK_Z, self.p["near"], torch.ones(n, device=self.d))
        ed = self.p["sd_depth"] * near * self._n(n) + self.bias[:, 0]
        out = self._u(n, 0, 1) < self.p["p_out"]
        ed = ed + out.float() * 30.0 * torch.sign(self._n(n))
        m_new = torch.stack([ztrue + ed,
                             el + self.bias[:, 1] + self.p["sd_lat"] * near * self._n(n),
                             eu + self.bias[:, 2] + self.p["sd_up"] * near * self._n(n) + self.p["couple"] * ed], 1)
        stale = (self._u(n, 0, 1) < self.p["p_stale"]) & (not fresh)
        miss = ((self._u(n, 0, 1) < self.p["p_miss"] * near) & (not fresh)) | self.lost
        upd = m & ~stale & ~miss
        self.meas[upd] = m_new[upd]
        self.valid[m] = (~miss[m]).float()
        self.seen[m] += self.valid[m]
        sag = torch.stack([(self.g[:, i] - self.q[:, i]) / DEG for i in (1, 2, 3)], 1)
        _, _, z, r = fk(self.q, torch.tensor([[L1, L2, L3]], device=self.d).expand(n, 3))
        frame = torch.cat([self.valid[:, None],
                           (self.meas[:, 0:1] - 100.0) / 50.0 * self.valid[:, None],
                           self.meas[:, 1:3] / 30.0 * self.valid[:, None],
                           self.last,
                           sag / 3.0,
                           self.blocked[:, None],
                           (self.t[:, None] - 30.0) / 30.0,
                           (-r[:, None] - 330.0) / 50.0,
                           (z[:, None] - 170.0) / 50.0,
                           (self.q[:, 0:1] / 0.3)], 1)
        self.hist[m] = torch.cat([frame[m][:, None], self.hist[m][:, :-1]], 1)

    def obs(self):
        return self.hist.reshape(self.n, -1)

    def privileged(self):
        """비평가만 본다 — 실기에선 못 얻는 참값."""
        ef, el, eu = self._err()
        ps = torch.stack([(self.p[k] - CENTER[k]) / max(1e-6, (RANGES[k][1] - RANGES[k][0])) for k in NAMES], 1)
        return torch.cat([torch.stack([ef / 100, el / 50, eu / 50], 1), self.bias / 10, ps], 1)

    def step(self, fwd, left, up, close):
        """mm 지령 + 닫기. done인 환경은 건드리지 않는다. → 보상."""
        live = ~self.done
        blind = self.seen == 0                            # 안전 규칙(정책 밖): 한 번도 못 봤으면 안 움직이고 안 닫는다
        close = close & ~blind
        fwd, left, up = (torch.where(blind, torch.zeros_like(v), v) for v in (fwd, left, up))
        d0 = self._dist()
        rew = torch.zeros(self.n, device=self.d)
        self.steps[live] += 1
        self.t[live] += 1.0                               # 카메라 안정·판단

        c = live & close
        ef, el, eu = self._err()
        el = el + self.p["play"] * self._n(self.n)
        eu = eu + self.p["play"] * self._n(self.n)
        along = -ef                                       # + = 줄기 지나침
        fit_d = (along >= self.p["d_lo"]) & (along <= self.p["d_hi"])
        fit_l = el.abs() <= self.p["gap"]
        fit_u = eu.abs() <= self.p["band"]
        self.cls[c & fit_d & fit_l & fit_u] = OK
        self.cls[c & fit_l & ~fit_u] = WRONG
        self.cls[c & (self.cls == RUN)] = EMPTY
        self.t[c] += 1.5
        self.done[c] = True

        go = live & ~close
        move = torch.stack([fwd, left, up], 1)
        mv = go & (move.norm(dim=1) >= 0.5)              # tool_jog: 0.5mm 아래는 안 간다
        hx, hy, lx, ly = self._axes()
        nominal = torch.tensor([[L1, L2, L3]], device=self.d).expand(self.n, 3)
        x, y, z, _ = fk(self.q, nominal)                  # 읽은 자리에서 출발(도구 호출마다)
        tx = x + hx * fwd + lx * left
        ty = y + hy * fwd + ly * left
        tz = z + up
        tgt, ok = ik(tx, ty, tz)
        r0, r1 = torch.sqrt(x * x + y * y), torch.sqrt(tx * tx + ty * ty)
        ok = ok & ~((r1 > MAX_REACH) & (r1 > r0))
        ok = ok & ((tgt >= self.limlo + 0.03 * (self.limhi - self.limlo)) & (tgt <= self.limhi - 0.03 * (self.limhi - self.limlo))).all(1)
        run = mv & ok
        self.blocked[go] = (mv & ~ok)[go].float()
        if run.any():
            target = torch.where(run[:, None], tgt, self.q)
            tries = self._press(run, target)
            self.t[run] += 1.6 * tries[run]               # 0.9초 쓰기 + 0.7초 안정
        # 줄기 흔들림·순간 낙하(팔이 내려간 것은 줄기가 올라간 것과 같다)
        self.stem[live] += self.p["sway"][live][:, None] * torch.randn(int(live.sum()), 3, device=self.d, generator=self.gen)
        ev = live & (self._u(self.n, 0, 1) < self.p["p_event"])
        self.stem[ev, 2] += self._u(int(ev.sum()), 2.0, 8.0)

        ef, el, eu = self._err()
        crash = go & ((-ef > PUSH) | (eu > POT_BELOW))
        self.cls[crash] = CRASH
        self.done[crash] = True
        over = live & ~self.done & (self.t > T_LIMIT)
        self.cls[over] = TIMEOUT
        self.cls[over & self.lost] = LOST
        self.done[over] = True
        self.last[go] = (move[go] / ACT.to(self.d)).clamp(-1, 1)
        keep = live & ~self.done
        if keep.any():
            self._observe(keep)

        rew[live] += 0.3 * (d0[live] - self._dist()[live]) / 10.0 - 0.02
        just = live & self.done
        rew[just & (self.cls == OK)] += 10.0 + 5.0 * (1 - self.t[just & (self.cls == OK)] / T_LIMIT)
        rew[just & ((self.cls == EMPTY) | (self.cls == WRONG) | ((self.cls == TIMEOUT) & ~self.lost))] -= 4.0
        rew[just & (self.cls == CRASH)] -= 8.0
        return rew

    def _dist(self):
        ef, el, eu = self._err()
        return torch.sqrt(ef * ef + el * el + eu * eu)


class Recipe:
    """grasp_repeat.close_in + straight_grasp를 옮긴 것 — 정책의 비교 상대."""
    DEFAULT = {"gain": 0.6, "near": 105.0, "stop": 75.0, "tol": 2.5, "lat_tol": 4.5, "iters": 6}

    def __init__(self, gym, **kw):
        self.a = dict(self.DEFAULT, **kw)
        self.gym = gym
        self.i = torch.zeros(gym.n, device=gym.d)
        self.phase = torch.zeros(gym.n, dtype=torch.long, device=gym.d)  # 0 다가가기 1 곧게 2 닫기

    def act(self):
        g, a = self.gym, self.a
        z, l, u = g.meas[:, 0], g.meas[:, 1], g.meas[:, 2]
        v = g.valid > 0
        last = self.i >= a["iters"] - 1
        gain = torch.where(last, torch.ones_like(u), torch.full_like(u, a["gain"]))
        up = torch.where(u.abs() > a["tol"], gain * u, torch.zeros_like(u)).clamp(-30, 30)
        lat = torch.where(l.abs() > a["lat_tol"], l, torch.zeros_like(l)).clamp(-30, 30)
        fw = torch.where(z > a["near"] + 10, (z - a["near"]) / 0.8, torch.zeros_like(z)).clamp(0, 60)
        idle = (up == 0) & (lat == 0) & (fw == 0)
        p0 = self.phase == 0
        to1 = p0 & v & (idle | (self.i >= a["iters"]))
        fwd_cmd = torch.zeros_like(z)
        left, upc = torch.zeros_like(z), torch.zeros_like(z)
        go0 = p0 & v & ~to1
        fwd_cmd[go0], left[go0], upc[go0] = fw[go0], lat[go0], up[go0]
        self.i[go0] += 1
        fwd_cmd[to1] = (z[to1] - a["stop"]).clamp(0, 45)  # 곧게: 기준 깊이로 정한 거리(추적 없음)
        p2 = self.phase == 1
        self.phase[to1] = 1
        close = p2
        return fwd_cmd, left, upc, close

    def reset(self, m):
        self.i[m] = 0
        self.phase[m] = 0


def run_recipe(gym, **kw):
    gym.reset(torch.ones(gym.n, dtype=torch.bool, device=gym.d))
    r = Recipe(gym, **kw)
    while not gym.done.all():
        gym.step(*r.act())
    return summary(gym)


def summary(gym):
    ok = gym.cls == OK
    out = {CLS[c]: float((gym.cls == c).float().mean()) for c in CLS}
    out["secs"] = float(gym.t[ok].mean()) if ok.any() else float("nan")
    out["steps"] = float(gym.steps.mean())
    return out


class Net(nn.Module):
    def __init__(self, n_obs, n_priv, hid=256):
        super().__init__()
        self.mu = nn.Sequential(nn.Linear(n_obs, hid), nn.ELU(), nn.Linear(hid, hid), nn.ELU(), nn.Linear(hid, 128), nn.ELU(), nn.Linear(128, 4))
        self.v = nn.Sequential(nn.Linear(n_obs + n_priv, hid), nn.ELU(), nn.Linear(hid, hid), nn.ELU(), nn.Linear(hid, 1))
        self.log_std = nn.Parameter(torch.full((3,), -0.7))
        nn.init.zeros_(self.mu[-1].weight)
        self.mu[-1].bias.data[3] = -2.0                   # 처음엔 안 닫는 쪽

    def dist(self, o):
        out = self.mu(o)
        return torch.distributions.Normal(out[:, :3], self.log_std.exp()), torch.distributions.Bernoulli(logits=out[:, 3])


class Norm:
    """관측 평균·분산 이동 추정."""

    def __init__(self, n, device):
        self.m, self.s, self.c = torch.zeros(n, device=device), torch.ones(n, device=device), 1e-4

    def update(self, x):
        b, mb, sb = x.shape[0], x.mean(0), x.var(0)
        t = self.c + b
        d = mb - self.m
        self.s = (self.s * self.c + sb * b + d * d * self.c * b / t) / t
        self.m += d * b / t
        self.c = t

    def __call__(self, x):
        return ((x - self.m) / self.s.clamp(min=1e-6).sqrt()).clamp(-8, 8)


def split_action(a, close):
    act = ACT.to(a.device)
    return a[:, 0] * act[0], a[:, 1] * act[1], a[:, 2] * act[2], close.bool()


@torch.no_grad()
def eval_policy(net, norm, gym):
    gym.reset(torch.ones(gym.n, dtype=torch.bool, device=gym.d))
    while not gym.done.all():
        out = net.mu(norm(gym.obs()))
        gym.step(*split_action(out[:, :3].clamp(-1, 1), out[:, 3] > 0))
    return summary(gym)


def eval_sets(device, n=20000):
    """합격 시험 무대 — 학습에 안 쓰는 씨앗. 팔은 학습 범위 전체(실측 전이라 일부러 넓게)."""
    return {"1차 노란 띠": Gym(n, device, scene="규약", seed=101, fix={"near": 1.0, "p_lost": 0.0}),
            "1차 맨 줄기": Gym(n, device, scene="규약", seed=102, fix={"p_lost": 0.0}),
            "3차 넘겨받기": Gym(n, device, scene="3차", seed=103, fix={"p_lost": 0.0}),
            "0차 재현(11/06)": Gym(n, device, spread=0.0, scene="고정", seed=104, fix={"p_lost": 0.65})}


def p_pass(p, n, k):
    """n회 중 k회 이상 성공할 확률."""
    return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(k, n + 1))


def p_need(n, k, conf=0.8):
    """그 확률이 conf가 되는 한 회 성공률."""
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if p_pass(mid, n, k) < conf else (lo, mid)
    return hi


def gate_lines(res):
    """체크포인트 판정으로 옮긴다. ⚠ 시뮬 추정 — 체크포인트 칸은 실기로만 채운다."""
    b1, s1, h3 = (res[k]["성공"] * REAL_GAP for k in ("1차 노란 띠", "1차 맨 줄기", "3차 넘겨받기"))   # 실기 차이만큼 깎은 값
    lost_ok = max(0.0, 1 - p_need(20, 10) / max(s1, 1e-6))
    reach = p_need(10, 5) / max(h3, 1e-6)
    return ["| 목표 | 합격선 | 한 회 성공률(실기 차이 ×%.2f 반영) | 합격 확률 |" % REAL_GAP, "|---|---|---|---|",
            "| 1차 노란 띠 | 20회 ≥10 | %.1f%% | %.1f%% |" % (100 * b1, 100 * p_pass(b1, 20, 10)),
            "| 1차 맨 줄기 | 20회 ≥10 | %.1f%% | %.1f%% |" % (100 * s1, 100 * p_pass(s1, 20, 10)),
            "| 3차 잡기 몫 | 10회 ≥5 | %.1f%% | %.1f%% |" % (100 * h3, 100 * p_pass(h3, 10, 5)),
            "",
            "- 1차 맨 줄기를 80%% 확률로 넘기려면 검출기가 못 찾는 회가 **%.0f%% 이하**여야 한다(11/06 65%%)." % (100 * lost_ok),
            "- 3차를 80%% 확률로 넘기려면 찾아가기 성공률: **%s**(실기 따라가기는 아직 0/3)."
            % ("%.0f%% 이상 필요" % (100 * reach) if reach <= 1 else "100%여도 모자람")]


def score(res):
    return (res["1차 노란 띠"]["성공"] + res["1차 맨 줄기"]["성공"] + res["3차 넘겨받기"]["성공"]) / 3


def tune_recipe(device, trials=150, n=8192):
    """규칙 그대로 값만 찾는다 — 강화학습 전에 이미 얻을 수 있는 만큼."""
    rng = np.random.default_rng(7)
    best = (-1e9, None, None)
    gym = Gym(n, device, seed=55)
    for _ in range(trials):
        kw = {"gain": float(rng.uniform(0.3, 1.0)), "near": float(rng.uniform(85, 125)), "stop": float(rng.uniform(65, 90)),
              "tol": float(rng.uniform(1.0, 6.0)), "lat_tol": float(rng.uniform(1.0, 8.0)), "iters": int(rng.integers(2, 9))}
        s = run_recipe(gym, **kw)
        score = s["성공"] - 0.002 * (s["secs"] if s["secs"] == s["secs"] else T_LIMIT)
        if score > best[0]:
            best = (score, kw, s)
    return best[1], best[2]


def keep_awake():
    """이 프로세스가 도는 동안만 절전을 막는다(설정은 안 바꾼다)."""
    if os.name == "nt":
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000000 | 0x00000001)


def gpu_temp():
    try:
        return int(subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"],
                                  capture_output=True, text=True, timeout=10).stdout.strip())
    except Exception:  # noqa: BLE001
        return 0


def export(net, norm, path):
    """젯슨엔 torch가 있지만 도구에서 numpy로도 돌게 무게만 남긴다."""
    w = {f"w{i}": p.detach().cpu().numpy() for i, p in enumerate(net.mu.parameters())}
    np.savez(path, mean=norm.m.cpu().numpy(), var=norm.s.cpu().numpy(), act=ACT.numpy(), hist=HIST, frame=FRAME, **w)


def train_seed(seed, secs, args, device, evals, log):
    d = os.path.join(OUT, f"seed{seed}")
    os.makedirs(d, exist_ok=True)
    torch.manual_seed(seed)
    gym = Gym(args.envs, device, seed=seed)
    no, np_ = gym.obs().shape[1], gym.privileged().shape[1]
    net = Net(no, np_).to(device)
    opt = torch.optim.Adam(net.parameters(), lr=3e-4)
    norm = Norm(no, device)
    used, frames, best = 0.0, 0, -1.0
    ck = os.path.join(d, "latest.pt")
    if os.path.exists(ck):
        s = torch.load(ck, map_location=device, weights_only=False)
        net.load_state_dict(s["net"]); opt.load_state_dict(s["opt"])
        norm.m, norm.s, norm.c = s["m"], s["s"], s["c"]
        used, frames, best = s["used"], s["frames"], s["best"]
    H = 16
    obs_b = torch.zeros((H, args.envs, no), device=device)
    priv_b = torch.zeros((H, args.envs, np_), device=device)
    a_b = torch.zeros((H, args.envs, 3), device=device)
    c_b = torch.zeros((H, args.envs), device=device)
    lp_b = torch.zeros((H, args.envs), device=device)
    r_b = torch.zeros((H, args.envs), device=device)
    d_b = torch.zeros((H, args.envs), device=device)
    v_b = torch.zeros((H, args.envs), device=device)
    t0, next_eval = time.time(), used + args.eval_every
    while used < secs:
        with torch.no_grad():
            for i in range(H):
                o = gym.obs()
                norm.update(o)
                no_ = norm(o)
                pr = gym.privileged()
                n_d, b_d = net.dist(no_)
                a, c = n_d.sample(), b_d.sample()
                obs_b[i], priv_b[i], a_b[i], c_b[i] = no_, pr, a, c
                lp_b[i] = n_d.log_prob(a).sum(1) + b_d.log_prob(c)
                v_b[i] = net.v(torch.cat([no_, pr], 1))[:, 0]
                r_b[i] = gym.step(*split_action(a.clamp(-1, 1), c))
                d_b[i] = gym.done.float()
                gym.reset(gym.done)
            v_last = net.v(torch.cat([norm(gym.obs()), gym.privileged()], 1))[:, 0]
            adv = torch.zeros_like(r_b)
            g = torch.zeros(args.envs, device=device)
            for i in reversed(range(H)):
                nv = v_last if i == H - 1 else v_b[i + 1]
                delta = r_b[i] + 0.99 * nv * (1 - d_b[i]) - v_b[i]
                g = delta + 0.99 * 0.95 * (1 - d_b[i]) * g
                adv[i] = g
            ret = adv + v_b
        O, P, A, C, L, R, V = (x.reshape(H * args.envs, *x.shape[2:]) for x in (obs_b, priv_b, a_b, c_b, lp_b, ret, adv))
        V = (V - V.mean()) / (V.std() + 1e-8)
        idx = torch.randperm(H * args.envs, device=device)
        for mb in idx.chunk(8):
            n_d, b_d = net.dist(O[mb])
            lp = n_d.log_prob(A[mb]).sum(1) + b_d.log_prob(C[mb])
            ratio = (lp - L[mb]).exp()
            pg = -torch.min(ratio * V[mb], ratio.clamp(0.8, 1.2) * V[mb]).mean()
            vl = (net.v(torch.cat([O[mb], P[mb]], 1))[:, 0] - R[mb]).pow(2).mean()
            ent = n_d.entropy().sum(1).mean() + b_d.entropy().mean()
            loss = pg + 0.5 * vl - 0.003 * ent
            opt.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(net.parameters(), 1.0)
            opt.step()
        frames += H * args.envs
        now = time.time()
        used, t0 = used + now - t0, now
        if used >= next_eval or used >= secs:
            next_eval = used + args.eval_every
            res = {k: eval_policy(net, norm, g_) for k, g_ in evals.items()}
            rate = score(res)
            row = dict(seed=seed, hours=round(used / 3600, 3), frames=frames, temp=gpu_temp(),
                       **{f"{k}_{m}": round(v[m], 4) for k, v in res.items() for m in ("성공", "secs", "충돌")}, score=round(rate, 4))
            log(row)
            if rate > best:
                best = rate
                torch.save({"net": net.state_dict(), "m": norm.m, "s": norm.s, "res": res}, os.path.join(d, "best.pt"))
                export(net, norm, os.path.join(d, "best.npz"))
            torch.save({"net": net.state_dict(), "opt": opt.state_dict(), "m": norm.m, "s": norm.s, "c": norm.c,
                        "used": used, "frames": frames, "best": best}, ck)
            while row["temp"] >= 85:                      # 밤새 돌리는 동안 카드 보호
                time.sleep(60)
                row["temp"] = gpu_temp()
            t0 = time.time()
    return frames, best


def table(rows):
    keys = ("성공", "빈손", "엉뚱한 곳", "충돌", "시간초과", "못 찾음", "secs", "steps")
    lines = ["| 대상 | 무대 | " + " | ".join(keys) + " |", "|" + "---|" * (len(keys) + 2)]
    for who, sets in rows.items():
        for name, s in sets.items():
            lines.append(f"| {who} | {name} | " + " | ".join("%.1f%%" % (100 * s[k]) if k not in ("secs", "steps") else "%.1f" % s[k] for k in keys) + " |")
    return "\n".join(lines)


def cmd_train(args):
    keep_awake()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    os.makedirs(OUT, exist_ok=True)
    start = time.time()
    evals = eval_sets(device)
    tuned_path = os.path.join(OUT, "recipe_tuned.json")
    if os.path.exists(tuned_path):
        tuned = json.load(open(tuned_path, encoding="utf-8"))
    else:
        kw, s = tune_recipe(device)
        tuned = {"kw": kw, "train": s}
        json.dump(tuned, open(tuned_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    base = {"레시피 그대로": {k: run_recipe(g) for k, g in evals.items()},
            "레시피 값 찾기": {k: run_recipe(g, **tuned["kw"]) for k, g in evals.items()}}
    print(table(base), flush=True)
    logf = open(os.path.join(OUT, "log.jsonl"), "a", encoding="utf-8")

    def log(row):
        print(row, flush=True)
        logf.write(json.dumps(row, ensure_ascii=False) + "\n")
        logf.flush()

    total = args.hours * 3600 - (time.time() - start)
    per = total / args.seeds
    outcome = {}
    for s in range(args.seeds):
        outcome[s] = train_seed(s, per, args, device, evals, log)
    bestseed = max(outcome, key=lambda k: outcome[k][1])
    ck = torch.load(os.path.join(OUT, f"seed{bestseed}", "best.pt"), map_location=device, weights_only=False)
    base[f"정책(seed{bestseed})"] = ck["res"]
    who = max(base, key=lambda k: score(base[k]))
    frames = sum(v[0] for v in outcome.values())
    rep = ["# 잡기 수련 결과 (시뮬 — 실기 결과 아님, 범위는 실측 전 초안)", "",
           f"동시 {args.envs}대 · 총 {frames:,} 걸음 · 씨앗 {args.seeds}개 · 씨앗별 최고 {[round(v[1], 3) for v in outcome.values()]}", "",
           table(base), "", f"## 체크포인트로 옮기면 — 가장 나은 쪽: {who}", "", *gate_lines(base[who]), "",
           "⚠ 0차는 실기 측정·보고·주문이라 시뮬로 채울 수 없다. 위는 실기 전에 보는 추정이다.", "",
           "레시피 값 찾기 결과: `" + json.dumps(tuned["kw"]) + "`",
           f"정책 파일: `{os.path.join(OUT, f'seed{bestseed}', 'best.npz')}`"]
    open(os.path.join(OUT, "report.md"), "w", encoding="utf-8").write("\n".join(rep))
    print("\n".join(rep), flush=True)


def cmd_bench(args):
    device = "cuda" if torch.cuda.is_available() else "cpu"
    for n in (2048, 4096, 8192, 16384, 32768, 65536):
        torch.cuda.empty_cache()
        gym = Gym(n, device)
        no, np_ = gym.obs().shape[1], gym.privileged().shape[1]
        net = Net(no, np_).to(device)
        t = time.time()
        with torch.no_grad():
            for _ in range(30):
                n_d, b_d = net.dist(gym.obs())
                gym.step(*split_action(n_d.sample().clamp(-1, 1), b_d.sample()))
                gym.reset(gym.done)
        dt = time.time() - t
        print(f"{n:6d}대  {n * 30 / dt / 1e3:7.1f}k 걸음/초 (학습 전) · 메모리 {torch.cuda.max_memory_allocated() / 1e9:.2f}GB", flush=True)


def cmd_calib(args):
    device = "cuda" if torch.cuda.is_available() else "cpu"

    def show(title, **k):
        print(title, {a: round(v, 3) for a, v in run_recipe(Gym(20000, device, spread=0.0, seed=2, **k)).items()})
    band = {"near": 1.0, "p_lost": 0.0}
    print("범위 가운데 팔, 레시피 그대로 — 실기와 비교")
    show("고정 자리·띠(실기 10/10·8/9·17~44초):", scene="고정", fix=band)
    show("옮긴 자리·띠(실기 9/14·15/20·24~48초):", scene="규약", fix=band)
    show("11/06 맨 줄기(실기 7/20, 닫은 회 7/7, 30~32초):", scene="고정", fix={"p_lost": 0.65})
    show("처짐 보정 이어 쓰기:", scene="규약", fix=band, seed_comp=True)
    show("안 보이는 처짐 없이:", scene="규약", fix=dict(band, flex=0.0, play=0.0))
    show("안 보이는 처짐 최대:", scene="규약", fix=dict(band, flex=4.0, play=1.5))


def cmd_selftest(args):
    sys.path[:0] = [os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src")]
    from tomato_picker.hardware import kinematics as kin
    device = "cuda" if torch.cuda.is_available() else "cpu"
    g = kin.ArmGeometry()
    assert (g.z0, g.d0, g.l1, g.l2, g.l3) == (Z0, D0, L1, L2, L3), "기하가 kinematics와 다르다"
    q = torch.tensor([START], device=device) * DEG
    lens = torch.tensor([[L1, L2, L3]], device=device)
    x, y, z, r = fk(q, lens)
    p = kin.forward(dict(zip(kin.JOINTS, START)), g)
    assert abs(float(x) - p.x) < 0.01 and abs(float(y) - p.y) < 0.01 and abs(float(z) - p.z) < 0.01, "FK가 kinematics와 다르다"
    back, ok = ik(x, y, z)
    assert bool(ok) and (back - q).abs().max() < 1e-3, "IK 왕복"
    # arm_twin 숫자와 같은 중력(K=28, 규약 시작 자세: −1.79, −1.67, −0.52)
    t = gravity(torch.tensor([[1.6, 93.9, 115.7, -34.7]], device=device) * DEG)
    for tau, want in zip(t, (-1.79, -1.67, -0.52)):
        assert abs(float(tau) / 28 - want) < 0.01, "중력이 arm_twin과 다르다"
    # 처짐 방향: 약한 팔일수록 press 전 낮게 선다
    w = Gym(4, device, spread=0.0, seed=4)
    w.p["K"][:] = 14.0
    w.g[:] = q
    w.q[:] = q
    w._settle(torch.ones(4, dtype=torch.bool, device=device))
    assert float(fk(w.q, lens.expand(4, 3))[2][0]) < float(z), "처지면 내려가야 한다"
    # 판정 창: 참값 그대로 닫으면 성공, 멀면 빈손 / 지나치면 충돌
    b = Gym(2, device, spread=0.0, scene="고정", seed=5, fix={"p_lost": 0.0})
    ef, el, eu = b._err()
    assert (ef > 20).all(), "시작은 줄기 앞"
    b.stem[:] = torch.stack(fk(b.q, b.lens)[:3], 1)
    b.step(*(torch.zeros(2, device=device),) * 3, torch.tensor([True, True], device=device))
    assert (b.cls == OK).all(), "손끝이 줄기에 있으면 문다"
    c = Gym(2, device, spread=0.0, scene="고정", seed=6, fix={"p_lost": 0.0})
    c.step(*(torch.zeros(2, device=device),) * 3, torch.tensor([True, True], device=device))
    assert (c.cls == EMPTY).all(), "멀리서 닫으면 빈손"
    d = Gym(2, device, spread=0.0, scene="고정", seed=7, fix={"p_lost": 0.0})
    for _ in range(4):
        d.step(torch.full((2,), 40.0, device=device), *(torch.zeros(2, device=device),) * 2, torch.zeros(2, dtype=torch.bool, device=device))
    assert (d.cls == CRASH).all() or d.blocked.all(), "줄기를 밀고 지나가면 충돌이거나 막혀야 한다"
    # 같은 씨앗 → 같은 결과
    s1, s2 = run_recipe(Gym(512, device, seed=9)), run_recipe(Gym(512, device, seed=9))
    assert s1 == s2, "재현 안 됨"
    # 작은 지령은 물리적으로 0: tool_jog 0.5mm 문턱
    e = Gym(2, device, spread=0.0, scene="고정", seed=8, fix={"p_lost": 0.0})
    before = e.q.clone()
    e.step(torch.full((2,), 0.2, device=device), *(torch.zeros(2, device=device),) * 2, torch.zeros(2, dtype=torch.bool, device=device))
    assert (e.q == before).all(), "0.5mm 아래는 안 움직인다"
    # 안 보이는 처짐: 엔코더 FK는 그대로인데 참 손끝만 내려간다
    h = Gym(2, device, spread=0.0, scene="고정", seed=10, fix={"flex": 0.0, "play": 0.0, "p_lost": 0.0})
    eu0 = h._err()[2].clone()
    h.p["flex"][:] = 4.0
    assert ((h._err()[2] - eu0) > 5).all(), "flex는 참 손끝을 낮춘다"
    assert all(LIM[i][0] < START[i] < LIM[i][1] for i in range(4)), "시작 자세가 관절 범위 안이어야 한다"
    assert abs(p_pass(0.5, 20, 10) - 0.5881) < 1e-3 and abs(p_pass(p_need(10, 5), 10, 5) - 0.8) < 1e-6, "이항 계산"
    lost = Gym(64, device, spread=0.0, scene="고정", seed=11, fix={"p_lost": 1.0})
    assert run_recipe(lost)["못 찾음"] == 1.0, "못 찾으면 닫지 않고 못 찾음으로 끝난다"
    blind_gym = Gym(8, device, spread=0.0, scene="규약", seed=12, fix={"p_lost": 1.0})
    q0 = blind_gym.q.clone()
    blind_gym.step(torch.full((8,), 30.0, device=device), *(torch.zeros(8, device=device),) * 2, torch.ones(8, dtype=torch.bool, device=device))
    assert (blind_gym.q == q0).all() and not blind_gym.done.any(), "못 본 채로는 안 움직이고 안 닫는다"
    print("selftest 통과")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["selftest", "bench", "calib", "train"])
    ap.add_argument("--hours", type=float, default=10.0)
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--envs", type=int, default=8192)
    ap.add_argument("--eval-every", type=float, default=600.0)
    a_ = ap.parse_args()
    {"selftest": cmd_selftest, "bench": cmd_bench, "calib": cmd_calib, "train": cmd_train}[a_.cmd](a_)
