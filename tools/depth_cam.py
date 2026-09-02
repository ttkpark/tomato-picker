#!/usr/bin/env python3
"""D405 깊이 카메라 발행기 (독립 실행, vision venv 전용).

D405를 잡아 **컬러에 정렬된 깊이**를 계속 /dev/shm에 쓴다. 팔·음성 스택은
다른 venv(~/lerobot/.venv)라 pyrealsense2를 못 쓰므로, 이 프로세스가 유일하게
카메라를 만지고 나머지는 공유 파일만 읽는다 — tomato_vision.py·line_follow.py와
같은 구조다.

발행물 (전부 원자적 교체, os.replace)
    /dev/shm/d405_color.jpg   컬러 프레임 (대시보드 표시 + 색검출용)
    /dev/shm/d405_depth.npy   컬러 격자에 정렬된 uint16 깊이 (원시 단위)
    /dev/shm/d405_depth.jpg   깊이 컬러맵 (사람이 눈으로 확인하는 용도)
    /dev/shm/d405_meta.json   내부파라미터·깊이단위·시각·유효율

────────────────────────────────────────────────────────────────────────
현장에서 실제로 확인한 것들 (2026-08-28, 시리얼 260322272920, FW 5.15.1.55)

⚠ **depth_scale이 0.0001 m/단위다** — 흔한 0.001(1mm)이 아니라 **0.1mm**다.
   이걸 1mm로 가정하면 모든 거리가 10배로 나오고, 손-눈 보정은 배율 0.1을
   보고하며 팔은 열매 열 배 뒤를 집으러 간다. 그래서 이 값을 **카메라에서
   읽어 meta.json에 적어 넘긴다** — 코드 어디에도 상수로 박지 않는다.

⚠ **정렬 방향은 depth→color여야 한다.** 반대로 하면(align to depth) 컬러가
   깊이 격자로 워핑되면서 **깊이가 무효인 픽셀마다 컬러에 검은 구멍**이 뚫린다
   (실측 유효율 0.65 → 화면의 3분의 1이 검은 얼룩). 색검출이 그 구멍을
   경계로 오인한다. 이쪽 방향은 컬러가 온전하고, 비용도 공짜였다(29.9fps).

⚠ **컬러와 깊이의 내부파라미터가 다르다.** depth는 정류돼 왜곡계수가 전부 0인데
   color는 inverse_brown_conrady에 계수가 실제로 있다(중심에서 먼 픽셀은
   50cm에서 8mm까지 틀어진다). 정렬을 컬러에 맞췄으므로 역투영도 **컬러
   내부파라미터**를 써야 한다 — meta.json에 그쪽을 적는다.
   (참고: depth→color 외부파라미터는 회전≈단위행렬, 평행이동≈0.01mm였다.
    D405는 같은 스테레오 모듈에서 컬러와 깊이가 나오기 때문이다.)

⚠ **D405는 근거리 전용이다** — 이상 동작범위 7~50cm, 1m를 넘으면 급격히
   나빠진다. 2026-08-28 삼각대 위치에서는 장면의 **0%**가 50cm 안에 없었고
   전부 1~2m였다. 그 상태로는 보정을 아무리 잘해도 열매를 못 집는다.
   그래서 meta.json에 거리 분포를 함께 실어, 대시보드가 "너무 멀다"를
   말할 수 있게 한다.
"""

from __future__ import annotations

import json
import os
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

WIDTH = int(os.environ.get("D405_WIDTH", "848"))
HEIGHT = int(os.environ.get("D405_HEIGHT", "480"))
FPS = int(os.environ.get("D405_FPS", "30"))
# 발행 주기. 카메라는 30fps로 돌지만 파일로 내보내는 건 이 정도면 충분하다
# (열매는 안 도망간다). 848x480 uint16 = 814KB라 8fps면 6.5MB/s.
PUBLISH_FPS = float(os.environ.get("D405_PUBLISH_FPS", "8"))
JPEG_QUALITY = int(os.environ.get("D405_JPEG_QUALITY", "80"))
# 깊이 필터. 정지 장면의 깊이 잡음을 줄여 보정 표본의 질을 크게 올린다.
USE_FILTERS = os.environ.get("D405_FILTERS", "1") not in ("0", "false", "no")
COLOR_PATH = os.environ.get("D405_COLOR", "/dev/shm/d405_color.jpg")
DEPTH_NPY = os.environ.get("D405_DEPTH_NPY", "/dev/shm/d405_depth.npy")
DEPTH_JPG = os.environ.get("D405_DEPTH_JPG", "/dev/shm/d405_depth.jpg")
META_PATH = os.environ.get("D405_META", "/dev/shm/d405_meta.json")
# 깊이 컬러맵의 표시 범위(mm) — D405 유효범위에 맞춘다. 이 밖은 잘려 보인다.
VIEW_MIN_MM = float(os.environ.get("D405_VIEW_MIN_MM", "60"))
VIEW_MAX_MM = float(os.environ.get("D405_VIEW_MAX_MM", "600"))
# 읽는 쪽의 거절 기준. 2026-09-01 추가 — 카메라가 둘이 되면서, 유효 거리를
# 읽는 코드(hardware/depth_camera.py)에 카메라별 상수로 두면 언젠가 한쪽이
# 틀린다. 깊이 단위가 그랬듯 **카메라가 스스로 말하게** 한다.
MIN_MM = float(os.environ.get("D405_MIN_MM", "70"))
MAX_MM = float(os.environ.get("D405_MAX_MM", "900"))


def _atomic(path: str, data: bytes) -> None:
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def _atomic_npy(path: str, arr: np.ndarray) -> None:
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        np.save(f, arr)
    os.replace(tmp, path)


def _intrinsics_dict(i) -> dict:
    """librealsense 왜곡모델 이름을 handeye.Intrinsics가 아는 문자열로."""
    name = str(i.model).rsplit(".", 1)[-1]
    known = {"none": "none", "brown_conrady": "brown_conrady",
             "modified_brown_conrady": "brown_conrady",
             "inverse_brown_conrady": "inverse_brown_conrady"}
    return {"width": i.width, "height": i.height,
            "fx": i.fx, "fy": i.fy, "ppx": i.ppx, "ppy": i.ppy,
            "model": known.get(name, "none"),
            "coeffs": [float(c) for c in i.coeffs]}


def _range_profile(z_mm: np.ndarray) -> dict:
    """거리 분포 — "지금 무대가 D405가 볼 수 있는 거리인가"를 화면이 말하게."""
    if z_mm.size == 0:
        return {"near_frac": 0.0, "median_mm": None}
    return {
        # D405가 정말 잘 보는 구간(70~500mm)의 비율. 이게 0이면 보정해도 소용없다.
        "near_frac": round(float(((z_mm >= 70) & (z_mm <= 500)).mean()), 3),
        "median_mm": round(float(np.median(z_mm)), 1),
        "p05_mm": round(float(np.percentile(z_mm, 5)), 1),
        "p95_mm": round(float(np.percentile(z_mm, 95)), 1),
    }


def _colormap(depth: np.ndarray, scale_mm: float) -> np.ndarray:
    z = depth.astype(np.float32) * scale_mm
    norm = np.clip((z - VIEW_MIN_MM) / max(1.0, VIEW_MAX_MM - VIEW_MIN_MM), 0, 1)
    img = cv2.applyColorMap((255 * (1.0 - norm)).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[depth == 0] = (0, 0, 0)   # 깊이 없음은 검정 — 구멍을 색으로 속이지 않는다
    return img


def main() -> None:
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    profile = pipe.start(cfg)

    dev = profile.get_device()
    scale_m = dev.first_depth_sensor().get_depth_scale()
    scale_mm = scale_m * 1000.0            # 원시 단위 → mm (D405는 0.1)
    align = rs.align(rs.stream.color)      # ⚠ 방향 주의 — 위 주석 참고
    color_intr = _intrinsics_dict(
        profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics())

    serial = dev.get_info(rs.camera_info.serial_number)
    print(f"D405 {serial} fw={dev.get_info(rs.camera_info.firmware_version)} "
          f"{WIDTH}x{HEIGHT}@{FPS} depth_scale={scale_m} ({scale_mm}mm/단위)",
          flush=True)

    filters = []
    if USE_FILTERS:
        # spatial = 가장자리를 지키며 구멍 메우기, temporal = 정지 장면 잡음 억제.
        # 보정 표본을 찍을 때 이 둘이 있고 없고가 잔차를 눈에 띄게 가른다.
        sp = rs.spatial_filter()
        sp.set_option(rs.option.holes_fill, 2)
        filters = [sp, rs.temporal_filter()]

    interval = 1.0 / max(0.5, PUBLISH_FPS)
    seq = 0
    next_pub = 0.0
    while True:
        try:
            frames = align.process(pipe.wait_for_frames(timeout_ms=5000))
        except RuntimeError as e:
            print(f"프레임 대기 실패: {e}", file=sys.stderr, flush=True)
            time.sleep(0.5)
            continue

        now = time.time()
        if now < next_pub:
            continue
        next_pub = now + interval

        dframe = frames.get_depth_frame()
        cframe = frames.get_color_frame()
        if not dframe or not cframe:
            continue
        for f in filters:
            dframe = f.process(dframe)

        depth = np.asanyarray(dframe.get_data())
        color = np.asanyarray(cframe.get_data())
        seq += 1

        valid = depth > 0
        z_mm = depth[valid].astype(np.float32) * scale_mm

        # ⚠ 순서가 중요하다 — 깊이를 먼저 쓰고 meta를 마지막에 쓴다. 읽는 쪽이
        #   meta를 먼저 보므로, meta가 가리키는 프레임은 항상 이미 디스크에 있다.
        _atomic_npy(DEPTH_NPY, depth)
        ok, buf = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            _atomic(COLOR_PATH, buf.tobytes())
        ok, buf = cv2.imencode(".jpg", _colormap(depth, scale_mm),
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            _atomic(DEPTH_JPG, buf.tobytes())

        meta = {
            "seq": seq,
            "ts": now,
            "serial": serial,
            "width": int(depth.shape[1]),
            "height": int(depth.shape[0]),
            # ⚠ 역투영은 이 값을 쓴다 — 깊이를 컬러에 정렬했으므로 컬러 쪽이다.
            "intrinsics": color_intr,
            "depth_scale_mm": scale_mm,
            "valid_frac": round(float(valid.mean()), 3),
            "filters": bool(filters),
            "camera": "d405",
            "min_mm": MIN_MM,
            "max_mm": MAX_MM,
            "near_mm": [70.0, 500.0],
            # 깊이를 컬러 격자에 정렬해 넘긴다(위 ⚠ 참고) — 컬러 화면의 픽셀이
            # 곧 깊이 격자의 같은 자리다. Astra는 여기가 false다.
            "color_aligned": True,
            "has_color": True,
            **_range_profile(z_mm),
        }
        _atomic(META_PATH, json.dumps(meta, ensure_ascii=False).encode("utf-8"))


if __name__ == "__main__":
    main()
