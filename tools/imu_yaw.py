#!/usr/bin/env python3
"""차체 회전량 발행기 — MPU6050 자이로를 적분해 /dev/shm/imu.json에 낸다 (stdlib만).

    python3 tools/imu_yaw.py            # 발행 (systemd imu-yaw)
    python3 tools/imu_yaw.py selftest   # PC, 장치 없이

왜 — 2026-10-22까지 이 로봇엔 자이로가 **하나도** 없었다(D405·Astra·Uno·젯슨 전부).
회전량은 space_map 스캔매칭뿐이었고, 그건 곧은 벽 한 토막이면 모호해진다.
배경 = docs/stm32-자세보드-목표.md §1·§4 0단계.

발행물 (원자적 교체, 50Hz)
    /dev/shm/imu.json  {ts, boot, yaw_deg(감기 누적, 반시계+), rate_dps, still, ready, bias_dps, up, hz, i2c_err}

⚠ **칩이 뒤집혀 붙어 있다**(가속도 z=−1g, 2026-10-22). 부호를 코드에 박지 않는다 —
  정지 때 잰 중력 방향(up)에 자이로를 사영하니 어떻게 달든 "위축 반시계 +"(REP-103)다.
⚠ **영점이 1.1°/s다**(z, 27°C) — 보정 없이 1분에 66°. 멈출 때마다(0.5초 창의 흔들림이
  작으면) 영점과 up을 다시 잰다. 넛지 주행이라 자주 멈춘다 = MPU6050으로 충분한 이유.
⚠ 젯슨 파이썬엔 smbus가 없다 — /dev/i2c-N + ioctl(I2C_RDWR)로 직접 읽는다.
⚠ **깨진 샘플 하나가 1.25°다**(2026-10-22 실측: 정지 중 −1.26°씩 계단, 첫 10분에 −8.6°).
  ±250°/s 꽉 찬 값 × 5ms. 레지스터 지정과 읽기를 한 트랜잭션(반복 시작)으로 묶고,
  DLPF 42Hz로는 5ms에 못 생기는 튐(JUMP)은 버린다 — 3번 연속이면 진짜로 받아들인다.
⚠ yaw는 **이 프로세스가 켜진 뒤의 상대값**이다. 다시 켜지면 0부터 — 읽는 쪽은 `boot`가
  바뀌었으면 옛 기준을 버린다.
"""

from __future__ import annotations

import ctypes
import json
import math
import os
import struct
import sys
import time
from collections import deque

BUS = int(os.environ.get("IMU_BUS", "7"))          # 40핀 3·5번 = i2c-7 (Orin Nano)
ADDR = 0x68
OUT = os.environ.get("IMU_OUT", "/dev/shm/imu.json")
SCALE = float(os.environ.get("IMU_SCALE", "1.0"))  # 자이로 스케일 보정 — 360° 돌려 실측해 덮는다
LSB = 131.0                                        # ±250°/s
WIN = 100                                          # 정지 판정 창 ≈0.5초
STILL_SD = 0.3                                     # 창 안 표준편차(°/s)가 세 축 모두 이 아래면 정지
DEAD = 0.5                                         # 정지 중 이보다 작은 각속도는 0으로(표류 차단)
JUMP = 60.0                                        # 한 샘플(5ms)에 이보다 크게 바뀌면 깨진 샘플(°/s)


class Yaw:
    """자이로·가속도 → 위축 yaw. 장치와 무관한 순수 계산이라 selftest가 그대로 돈다."""

    def __init__(self):
        self.bias = None            # 세 축 영점(°/s) — 첫 정지 창 전엔 None(=ready 아님)
        self.up = (0.0, 0.0, 1.0)
        self.yaw = 0.0              # 도, 감기 누적
        self.rate = 0.0
        self.still = False
        self.k = 0
        self.last, self.rej, self.glitch = None, 0, 0
        self.g, self.a = deque(maxlen=WIN), deque(maxlen=WIN)

    def step(self, gyro, acc, dt: float) -> None:
        if self.last is not None and max(abs(gyro[i] - self.last[i]) for i in range(3)) > JUMP and self.rej < 2:
            self.rej += 1
            self.glitch += 1
            gyro, acc = self.last, self.a[-1]        # 그 5ms는 직전 값으로 메운다
        else:
            self.rej = 0
            self.last = gyro
        self.g.append(gyro)
        self.a.append(acc)
        self.k += 1
        if len(self.g) == WIN and self.k % 10 == 0:     # 창 통계는 10샘플마다(파이썬 부하) — 판정은 그 사이 유지
            self.still = False
            m = [sum(v[i] for v in self.g) / WIN for i in range(3)]
            sd = [math.sqrt(sum((v[i] - m[i]) ** 2 for v in self.g) / WIN) for i in range(3)]
            # 천천히 도는 꼬리를 영점으로 삼키지 않게 — 첫 영점 뒤엔 1°/s 넘게 다르면 정지가 아니다
            if max(sd) < STILL_SD and (self.bias is None or max(abs(m[i] - self.bias[i]) for i in range(3)) < 1.0):
                self.still = True
                self.bias = m if self.bias is None else [b + 0.2 * (x - b) for b, x in zip(self.bias, m)]
                am = [sum(v[i] for v in self.a) / WIN for i in range(3)]
                n = math.sqrt(sum(x * x for x in am)) or 1.0
                self.up = tuple(x / n for x in am)
        if self.bias is None:
            return
        r = SCALE * sum((gyro[i] - self.bias[i]) * self.up[i] for i in range(3))
        self.rate = r
        if self.still and abs(r) < DEAD:
            return
        self.yaw += r * dt


class Msg(ctypes.Structure):                       # linux/i2c.h struct i2c_msg
    _fields_ = [("addr", ctypes.c_uint16), ("flags", ctypes.c_uint16),
                ("len", ctypes.c_uint16), ("buf", ctypes.c_void_p)]


class RdWr(ctypes.Structure):                      # struct i2c_rdwr_ioctl_data
    _fields_ = [("msgs", ctypes.POINTER(Msg)), ("nmsgs", ctypes.c_uint32)]


class Mpu:
    def __init__(self, bus: int = BUS):
        import fcntl
        self.fd = os.open("/dev/i2c-%d" % bus, os.O_RDWR)
        fcntl.ioctl(self.fd, 0x0703, ADDR)          # I2C_SLAVE (쓰기용)
        self.ioctl = fcntl.ioctl
        self.err = 0
        if self.rd(0x75, 1)[0] not in (0x68, 0x70, 0x71, 0x72, 0x73, 0x98):
            raise RuntimeError("0x%02x는 MPU 계열이 아니다" % self.rd(0x75, 1)[0])
        for r, v in ((0x6B, 0x01), (0x19, 0x04), (0x1A, 0x03), (0x1B, 0x00), (0x1C, 0x00)):
            self.wr(r, v)                           # 깨움·200Hz·DLPF 42Hz·±250°/s·±2g
        time.sleep(0.1)

    def wr(self, r: int, v: int) -> None:
        os.write(self.fd, bytes([r, v]))

    def rd(self, r: int, n: int) -> bytes:
        """레지스터 지정 + 읽기를 **한 트랜잭션**(I2C_RDWR, 반복 시작)으로 — 사이에 끼어들 틈이 없다."""
        reg, out = ctypes.create_string_buffer(bytes([r]), 1), ctypes.create_string_buffer(n)
        msgs = (Msg * 2)(Msg(ADDR, 0, 1, ctypes.addressof(reg)), Msg(ADDR, 1, n, ctypes.addressof(out)))  # 1 = I2C_M_RD
        req = RdWr(msgs, 2)
        for _ in range(5):                          # 175Hz에 1회꼴 Remote I/O — 재시도로 흡수(실측)
            try:
                self.ioctl(self.fd, 0x0707, req)    # I2C_RDWR
                return out.raw
            except OSError:
                self.err += 1
                time.sleep(0.002)
        raise OSError("I2C 5회 연속 실패 — 배선·전원")

    def read(self):
        v = struct.unpack(">hhhhhhh", self.rd(0x3B, 14))
        return [v[4 + i] / LSB for i in range(3)], [v[i] / 16384.0 for i in range(3)]


def publish(st: dict) -> None:
    """⚠ /dev/shm 파일이 쓰는 도중 사라질 수 있다(2026-10-22 19:48 실측: tmp→replace 사이 FileNotFoundError로
    발행기가 죽고 yaw가 0부터 다시 — 스캔 중이면 그 스캔이 버려진다). server 사용자의 마지막 SSH가 끊기면
    logind RemoveIPC가 그 사용자의 /dev/shm 파일을 지우는 것으로 본다(Linger=no). 한 번 거르고 다음 20ms에 다시 쓴다."""
    tmp = OUT + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(st, f)
        os.replace(tmp, OUT)
    except FileNotFoundError:
        pass


def run() -> int:
    mpu, y = Mpu(), Yaw()
    boot = time.time()
    t_prev = time.monotonic()
    t_pub, n = 0.0, 0
    while True:
        g, a = mpu.read()
        t = time.monotonic()
        # ponytail: 폴링 + 실제 dt 적분 — 파이썬이 멈칫하면 그 구간은 직선 근사. 샘플을 놓치면 FIFO로.
        y.step(g, a, min(t - t_prev, 0.1))
        t_prev, n = t, n + 1
        if t - t_pub >= 0.02:
            publish({"ts": time.time(), "boot": boot, "yaw_deg": round(y.yaw, 3),
                     "rate_dps": round(y.rate, 2), "still": y.still, "ready": y.bias is not None,
                     "bias_dps": [round(b, 3) for b in (y.bias or [0, 0, 0])],
                     "up": [round(u, 3) for u in y.up], "hz": round(n / max(t - t_pub, 1e-3)) if t_pub else 0,
                     "i2c_err": mpu.err, "glitch": y.glitch})
            t_pub, n = t, 0
        time.sleep(0.004)


def selftest() -> int:
    # 뒤집혀 달린 칩(up=−z), 영점 (−0.5, 0.6, −1.1)°/s, 잡음 0.1°/s — 2026-10-22 실측과 같은 꼴
    import random
    random.seed(1)
    bias, dt = (-0.5, 0.6, -1.1), 0.005
    y = Yaw()

    def feed(secs, wz):
        for _ in range(int(secs / dt)):
            g = [bias[i] + random.gauss(0, 0.1) for i in range(3)]
            g[2] += -wz                             # 위축 반시계 wz = 칩 z축으로는 −wz
            y.step(g, [0.0, 0.0, -1.0], dt)

    feed(1.0, 0)
    assert y.bias is not None and abs(y.bias[2] - bias[2]) < 0.05, y.bias
    assert y.up[2] < -0.99, "뒤집힌 칩의 위는 −z"
    feed(30.0, 0)
    assert abs(y.yaw) < 0.2, ("멈춰 있는데 돈다", y.yaw)
    feed(2.0, 45.0)                                 # 반시계 90°
    feed(1.0, 0)
    assert abs(y.yaw - 90.0) < 1.0, y.yaw
    for _ in range(3):                              # 깨진 샘플(꽉 찬 −250°/s) — 버린다
        y.step([0.0, 0.0, -250.0], [0.0, 0.0, -1.0], dt)
        feed(0.05, 0)
    assert abs(y.yaw - 90.0) < 1.0 and y.glitch == 3, (y.yaw, y.glitch)
    feed(1.0, -30.0)                                # 시계 30°
    feed(60.0, 0)
    assert abs(y.yaw - 60.0) < 1.0, ("1분 정지 뒤", y.yaw)
    print("imu_yaw selftest ✅ yaw %.2f° (참 60°)" % y.yaw)
    return 0


if __name__ == "__main__":
    raise SystemExit(selftest() if sys.argv[1:] == ["selftest"] else run())
