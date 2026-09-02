#!/usr/bin/env python3
"""Orbbec Astra Pro 깊이 발행기 (독립 실행, vision venv 전용).

`tools/depth_cam.py`(D405)와 **같은 계약**으로 /dev/shm에 쓴다 — 읽는 쪽
(`hardware/depth_camera.py`)이 카메라를 골라 같은 코드로 읽게 하기 위해서다.
파일 이름과 meta의 몇 값만 다르고 구조는 같다.

발행물 (전부 원자적 교체, os.replace)
    /dev/shm/astra_depth.npy   uint16 깊이 (밀리미터, **깊이 격자**)
    /dev/shm/astra_depth.jpg   깊이 컬러맵 (사람이 눈으로 확인하는 용도)
    /dev/shm/astra_color.jpg   컬러 프레임 (ASTRA_COLOR=1 일 때만)
    /dev/shm/astra_meta.json   내부파라미터·깊이단위·시각·거리분포

────────────────────────────────────────────────────────────────────────
왜 D405가 있는데 이걸 또 다나 — **거리 대역이 안 겹친다.**

    D405   7~50cm   집게가 열매를 무는 순간의 거리
    Astra  60~400cm 무대 전체를 보는 거리

2026-08-28 D405를 처음 달았을 때 삼각대가 무대에서 2m 떨어져 있었고 장면의
**0%**가 D405 유효범위 안이었다(`docs/depth-camera.md` §0). 그 구멍이 이
카메라의 자리다. 반대로 Astra는 60cm 아래를 아예 못 본다 — 둘 중 하나가
"더 좋은" 게 아니라 **쓰는 자리가 다르다.**

────────────────────────────────────────────────────────────────────────
현장에서 실제로 확인한 것들 (2026-09-01, 시리얼 17121112300, FW 5.8.22)

⚠ **드라이버는 커널에도 apt에도 없다.** 우분투 `libopenni2-0`이 들고 있는
   PS1080 드라이버는 PrimeSense VID(0x1d27)만 찾는다 — Orbbec은 0x2bc5라
   장치가 **하나도 안 잡힌다**(에러가 아니라 "장치 0개"로 조용히 끝난다).
   Orbbec이 배포하는 `liborbbec.so`를 따로 깔아야 한다. 설치는
   `deploy/astra-install.sh` 한 방 · 근거는 `docs/depth-camera.md` §9.

⚠ **깊이 단위가 1mm다** — D405(0.1mm)와 다르다. 그래서 이 값도 코드에 박지
   않고 meta.json으로 넘긴다. 두 카메라가 같은 코드로 읽히는 이유가 이거다.

⚠ **ini의 기본 해상도(640x400)로 열면 프레임이 통째로 깨진다.** 매 프레임
   `Depth buffer is corrupt. Size is 511856 (!= 512000)` — 144바이트가 모자란다.
   640x480(Resolution=1)으로 바꾸면 멀쩡하다. USB 대역이나 usbfs 문제가
   아니었다(usbfs_memory_mb를 16→1000으로 올려도 그대로였다). 설치 스크립트가
   `orbbec.ini`를 640x480@30으로 고쳐 놓는다.

⚠ **컬러는 깊이와 정렬돼 있지 않다.** Astra Pro의 RGB는 **별개의 USB 장치**
   (UVC, `/dev/video0`)라 깊이와 하드웨어 동기도 정렬도 없다. 게다가 이 개체는
   공장 D2C 보정값(OBCameraParams)이 **전부 NaN**이다 — 즉 소프트웨어로 맞출
   근거도 없다. 그래서 meta에 `color_aligned: false`를 싣고, 읽는 쪽이 컬러
   픽셀 클릭을 **거절**한다. 클릭은 깊이 화면에서 한다(그쪽은 정의상 정렬돼 있다).
   D405는 반대다(align to color가 공짜였다) — 두 카메라의 진짜 차이가 여기다.

⚠ **IR 스트림을 만들면 드라이버가 힙을 깨고 죽는다**(`malloc(): invalid size`).
   `oniDeviceGetProperty(IMAGE_REGISTRATION)`도 같다. 그래서 이 발행기는
   **검증된 호출만** 쓴다 — 깊이 스트림 생성·모드 설정·읽기, 그리고 시리얼/
   장치이름 읽기. 편의를 위해 호출을 하나 더 넣고 싶어지면, 그 호출이 30분 뒤
   서비스를 죽인다는 걸 기억하라.

⚠ **너무 가까우면 전부 0이다.** 최소거리 60cm이고, 그보다 가까운 물체가 있으면
   LDP(레이저 보호)가 아예 투광기를 끈다. 첫 연결 때 벽 20cm 앞에 놓여 있어
   유효 픽셀이 **0%**였다 — 카메라가 고장난 것처럼 보이지만 정상이다.
   meta의 거리 분포가 이걸 말해준다.
"""

from __future__ import annotations

import ctypes
import json
import os
import sys
import time

import cv2
import numpy as np

# 드라이버 위치. deploy/astra-install.sh 가 여기에 푼다.
ONI_HOME = os.environ.get("ASTRA_ONI_HOME", os.path.expanduser("~/openni2"))
WIDTH = int(os.environ.get("ASTRA_WIDTH", "640"))
HEIGHT = int(os.environ.get("ASTRA_HEIGHT", "480"))
FPS = int(os.environ.get("ASTRA_FPS", "30"))
PUBLISH_FPS = float(os.environ.get("ASTRA_PUBLISH_FPS", "8"))
JPEG_QUALITY = int(os.environ.get("ASTRA_JPEG_QUALITY", "80"))
# ⚠ 컬러는 **기본 꺼짐**이다 — 같은 UVC 노드를 tomato-vision이 잡고 있다.
#   장치는 한 프로세스만 연다(이 저장소의 오래된 규칙). 켜려면 tomato-vision을
#   먼저 내리거나 TV_CAMERA_DEV로 다른 카메라를 물려라.
USE_COLOR = os.environ.get("ASTRA_COLOR", "0") not in ("0", "false", "no", "")
COLOR_DEV = os.environ.get("ASTRA_COLOR_DEV",
                           "/dev/v4l/by-id/usb-Astra_Pro_HD_Camera_Astra_Pro_HD_Camera-video-index0")
DEPTH_NPY = os.environ.get("ASTRA_DEPTH_NPY", "/dev/shm/astra_depth.npy")
DEPTH_JPG = os.environ.get("ASTRA_DEPTH_JPG", "/dev/shm/astra_depth.jpg")
COLOR_PATH = os.environ.get("ASTRA_COLOR_JPG", "/dev/shm/astra_color.jpg")
META_PATH = os.environ.get("ASTRA_META", "/dev/shm/astra_meta.json")

# 이 카메라가 **믿을 만한** 거리(mm). 스펙은 0.6~8m지만 8m에서는 오차가
# 10cm를 넘어 열매 크기와 비교가 안 된다. 읽는 쪽은 meta의 이 값을 그대로
# 거절 기준으로 쓴다(코드에 카메라별 상수를 박지 않기 위해).
MIN_MM = float(os.environ.get("ASTRA_MIN_MM", "600"))
MAX_MM = float(os.environ.get("ASTRA_MAX_MM", "4000"))
# "무대가 지금 좋은 거리에 있나"를 재는 띠. 이 안이 near_frac이다.
NEAR_MM = (float(os.environ.get("ASTRA_NEAR_MIN_MM", "600")),
           float(os.environ.get("ASTRA_NEAR_MAX_MM", "2500")))
VIEW_MIN_MM = float(os.environ.get("ASTRA_VIEW_MIN_MM", "500"))
VIEW_MAX_MM = float(os.environ.get("ASTRA_VIEW_MAX_MM", "3000"))
# ⚠ **깊이가 통째로 0인 채 굳는 병을 스스로 푼다** (실측 2026-09-01/02).
#   카메라가 60cm보다 가까운 것을 보고 있으면 LDP(레이저 보호)가 투광기를 끄는데,
#   **그 상태가 스트림 수명 내내 유지된다.** 카메라를 좋은 거리로 돌려놔도
#   0%가 그대로였고, 프로세스를 새로 띄우니 즉시 79%가 나왔다.
#   증상은 "카메라 고장"처럼 보이고 원인은 "아까 벽에 붙여 놨던 것"이라 —
#   사람이 이 둘을 잇기가 거의 불가능하다. 그래서 코드가 잇는다.
#   프레임은 계속 오는데 유효 픽셀이 이 시간 동안 하나도 없으면 **장치를 다시 연다**
#   (프로세스를 끝내고 systemd가 새로 띄운다). 0이면 이 기능을 끈다.
ZERO_REOPEN_SEC = float(os.environ.get("ASTRA_ZERO_REOPEN_SEC", "30"))

# ---------------------------------------------------------------------------
# OpenNI2 C API — ctypes로 직접 부른다
#
# pip의 `openni` 바인딩을 쓰지 않는 이유: 그 쪽 고수준 래퍼로 스트림을 만들면
# 이 드라이버에서 힙이 깨져 죽는다(실측). 어차피 쓰는 함수가 여덟 개뿐이라,
# 검증된 호출만 직접 부르는 편이 의존성도 사고 표면도 작다.
# ---------------------------------------------------------------------------

ONI_STATUS_OK = 0
ONI_SENSOR_DEPTH = 3
ONI_PIXEL_FORMAT_DEPTH_1_MM = 100
STREAM_PROPERTY_HORIZONTAL_FOV = 1
STREAM_PROPERTY_VERTICAL_FOV = 2
STREAM_PROPERTY_VIDEO_MODE = 3
OBEXTENSION_ID_SERIALNUMBER = 16
OBEXTENSION_ID_DEVICETYPE = 17


class OniVideoMode(ctypes.Structure):
    _fields_ = [("pixelFormat", ctypes.c_int), ("resolutionX", ctypes.c_int),
                ("resolutionY", ctypes.c_int), ("fps", ctypes.c_int)]


class OniFrame(ctypes.Structure):
    _fields_ = [("dataSize", ctypes.c_int), ("data", ctypes.c_void_p),
                ("sensorType", ctypes.c_int), ("timestamp", ctypes.c_ulonglong),
                ("frameIndex", ctypes.c_int), ("width", ctypes.c_int),
                ("height", ctypes.c_int), ("videoMode", OniVideoMode),
                ("croppingEnabled", ctypes.c_int), ("cropOriginX", ctypes.c_int),
                ("cropOriginY", ctypes.c_int), ("stride", ctypes.c_int)]


class OpenNIError(RuntimeError):
    pass


class Astra:
    """깊이 스트림 하나. 여는 데 실패하면 **왜 못 열었는지**를 들고 죽는다."""

    def __init__(self, home: str = ONI_HOME) -> None:
        so = os.path.join(home, "libOpenNI2.so")
        if not os.path.exists(so):
            raise OpenNIError(
                f"OpenNI2 라이브러리가 없다: {so}\n"
                "Orbbec 드라이버를 안 깐 것이다 — `bash deploy/astra-install.sh`. "
                "우분투 apt의 libopenni2는 Orbbec 장치를 못 본다(VID가 다르다).")
        # 드라이버 저장소는 .so 위치 기준 'OpenNI2/Drivers'로 찾는다 — 그래서
        # 어디서 실행하든(서비스의 WorkingDirectory가 무엇이든) 같게 동작한다.
        os.environ.setdefault("OPENNI2_DRIVERS_PATH", os.path.join(home, "OpenNI2", "Drivers"))
        self._lib = ctypes.CDLL(so)
        self._dev = ctypes.c_void_p()
        self._stream = ctypes.c_void_p()
        self._init()
        self._open()

    # --- 수명 ---

    def _check(self, rc: int, what: str) -> None:
        if rc != ONI_STATUS_OK:
            raise OpenNIError(f"{what} 실패 (OniStatus={rc})")

    def _init(self) -> None:
        # API 버전은 헤더 상수라 라이브러리 빌드마다 다르다. 맞을 때까지
        # 해 보는 편이, 버전 하나를 코드에 박고 "장치 0개"로 헤매는 것보다 낫다.
        last = -1
        for ver in (2003, 2002, 2001, 2000):
            last = self._lib.oniInitialize(ver)
            if last == ONI_STATUS_OK:
                return
        raise OpenNIError(
            f"oniInitialize가 모든 API 버전에서 실패했다 (마지막 OniStatus={last})")

    def _open(self) -> None:
        self._check(self._lib.oniDeviceOpen(None, ctypes.byref(self._dev)),
                    "장치 열기(oniDeviceOpen)")
        self._check(self._lib.oniDeviceCreateStream(
            self._dev, ONI_SENSOR_DEPTH, ctypes.byref(self._stream)), "깊이 스트림 생성")
        vm = OniVideoMode(pixelFormat=ONI_PIXEL_FORMAT_DEPTH_1_MM,
                          resolutionX=WIDTH, resolutionY=HEIGHT, fps=FPS)
        self._check(self._lib.oniStreamSetProperty(
            self._stream, STREAM_PROPERTY_VIDEO_MODE, ctypes.byref(vm), ctypes.sizeof(vm)),
            f"깊이 모드 설정({WIDTH}x{HEIGHT}@{FPS})")
        self._check(self._lib.oniStreamStart(self._stream), "깊이 스트림 시작")

    def close(self) -> None:
        try:
            self._lib.oniStreamStop(self._stream)
        except Exception:  # noqa: BLE001 - 종료 경로에서 더 시끄러워질 이유가 없다
            pass

    # --- 값 ---

    def _text_property(self, prop: int, size: int = 64) -> str:
        buf = (ctypes.c_char * size)()
        n = ctypes.c_int(size)
        rc = self._lib.oniDeviceGetProperty(self._dev, prop, buf, ctypes.byref(n))
        return buf.value.decode("utf-8", "replace") if rc == ONI_STATUS_OK else ""

    @property
    def serial(self) -> str:
        return self._text_property(OBEXTENSION_ID_SERIALNUMBER)

    @property
    def model(self) -> str:
        return self._text_property(OBEXTENSION_ID_DEVICETYPE) or "Orbbec Astra"

    def fov(self) -> tuple[float, float]:
        """(수평, 수직) 화각(라디안). 내부파라미터를 여기서 만든다."""
        out = []
        for prop in (STREAM_PROPERTY_HORIZONTAL_FOV, STREAM_PROPERTY_VERTICAL_FOV):
            f = ctypes.c_float(0.0)
            n = ctypes.c_int(ctypes.sizeof(f))
            self._check(self._lib.oniStreamGetProperty(
                self._stream, prop, ctypes.byref(f), ctypes.byref(n)), "화각 읽기")
            out.append(float(f.value))
        return out[0], out[1]

    def read(self, timeout_ms: int = 2000) -> np.ndarray:
        fp = ctypes.POINTER(OniFrame)()
        self._check(self._lib.oniStreamReadFrame(self._stream, ctypes.byref(fp)),
                    "프레임 읽기")
        f = fp.contents
        arr = np.ctypeslib.as_array(
            ctypes.cast(f.data, ctypes.POINTER(ctypes.c_uint16)),
            (f.height, f.width)).copy()          # copy: 아래에서 프레임을 놓는다
        self._lib.oniFrameRelease(fp)
        return arr


# ---------------------------------------------------------------------------


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


def _intrinsics(width: int, height: int, hfov: float, vfov: float) -> dict:
    """화각 → 핀홀 내부파라미터.

    ⚠ D405는 카메라가 fx·fy·주점을 직접 알려주지만 Astra는 **화각만** 준다.
      그래서 주점을 화면 정중앙으로 **가정**한다. 실제 주점은 몇 픽셀 어긋나
      있을 수 있고, 그 오차는 손-눈 보정 잔차로 나타난다 — 잔차가 유난히
      크면 이 가정을 먼저 의심하라(자로 잰 값이 아니라 가정이다).
    """
    return {"width": width, "height": height,
            "fx": (width / 2.0) / np.tan(hfov / 2.0),
            "fy": (height / 2.0) / np.tan(vfov / 2.0),
            "ppx": (width - 1) / 2.0, "ppy": (height - 1) / 2.0,
            "model": "none", "coeffs": [0.0] * 5}


def _range_profile(z_mm: np.ndarray) -> dict:
    """거리 분포 — "지금 무대가 이 카메라가 볼 수 있는 거리인가"를 화면이 말하게."""
    if z_mm.size == 0:
        return {"near_frac": 0.0, "median_mm": None}
    lo, hi = NEAR_MM
    return {
        "near_frac": round(float(((z_mm >= lo) & (z_mm <= hi)).mean()), 3),
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


def _open_color():
    """UVC 컬러. 못 열면 None — 컬러가 없어도 깊이는 계속 나가야 한다."""
    cap = cv2.VideoCapture(COLOR_DEV, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"컬러({COLOR_DEV})를 못 열었다 — 다른 프로세스(tomato-vision)가 "
              "잡고 있을 수 있다. 깊이만 발행한다.", file=sys.stderr, flush=True)
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    for _ in range(8):
        cap.read()
    return cap


def main() -> None:
    cam = Astra()
    hfov, vfov = cam.fov()
    intr = _intrinsics(WIDTH, HEIGHT, hfov, vfov)
    scale_mm = 1.0                      # ⚠ D405(0.1)와 다르다 — 아래 meta로 넘긴다
    serial, model = cam.serial, cam.model
    print(f"{model} {serial} {WIDTH}x{HEIGHT}@{FPS} "
          f"hfov={np.degrees(hfov):.1f}° vfov={np.degrees(vfov):.1f}° "
          f"fx={intr['fx']:.1f} fy={intr['fy']:.1f} depth_scale={scale_mm}mm/단위",
          flush=True)

    color = _open_color() if USE_COLOR else None
    interval = 1.0 / max(0.5, PUBLISH_FPS)
    seq = 0
    next_pub = 0.0
    last_valid = time.time()
    while True:
        try:
            depth = cam.read()
        except OpenNIError as e:
            # 여기서 되살리려 들지 않는다 — USB가 빠졌으면 장치 핸들이 통째로
            # 죽은 것이고, systemd가 새 프로세스로 다시 여는 편이 확실하다.
            print(f"깊이 프레임 실패: {e}", file=sys.stderr, flush=True)
            time.sleep(0.5)
            raise SystemExit(1)

        now = time.time()
        if now < next_pub:
            continue
        next_pub = now + interval
        seq += 1

        valid = depth > 0
        z_mm = depth[valid].astype(np.float32) * scale_mm

        # 굳은 투광기 풀기 — 위 ZERO_REOPEN_SEC 주석 참고.
        if z_mm.size:
            last_valid = now
        elif ZERO_REOPEN_SEC > 0 and now - last_valid > ZERO_REOPEN_SEC:
            print(f"깊이가 {now - last_valid:.0f}초째 전부 0이다 — 프레임은 오는데 "
                  "유효 픽셀이 하나도 없다. 투광기가 꺼진 채 굳은 것으로 보고 "
                  "장치를 다시 연다(systemd가 새로 띄운다).\n"
                  "  ⚠ 렌즈가 막혔거나 60cm보다 가까운 것만 보고 있으면 다시 열어도 "
                  "같으니, 이 줄이 계속 반복되면 카메라가 보는 곳을 확인하라.",
                  file=sys.stderr, flush=True)
            raise SystemExit(1)

        # ⚠ 순서가 중요하다 — 깊이를 먼저 쓰고 meta를 마지막에 쓴다. 읽는 쪽이
        #   meta를 먼저 보므로, meta가 가리키는 프레임은 항상 이미 디스크에 있다.
        _atomic_npy(DEPTH_NPY, depth)
        ok, buf = cv2.imencode(".jpg", _colormap(depth, scale_mm),
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            _atomic(DEPTH_JPG, buf.tobytes())
        if color is not None:
            got, img = color.read()
            if got:
                ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    _atomic(COLOR_PATH, buf.tobytes())

        meta = {
            "seq": seq,
            "ts": now,
            "camera": "astra",
            "model": model,
            "serial": serial,
            "width": int(depth.shape[1]),
            "height": int(depth.shape[0]),
            "intrinsics": intr,
            "depth_scale_mm": scale_mm,
            "valid_frac": round(float(valid.mean()), 3),
            "filters": False,
            # 읽는 쪽의 거절 기준. 카메라가 스스로 말한다 — 그래야 읽는 코드에
            # 카메라별 상수가 안 생긴다.
            "min_mm": MIN_MM,
            "max_mm": MAX_MM,
            "near_mm": list(NEAR_MM),
            # ⚠ 컬러는 깊이 격자가 아니다(§ 파일 첫머리). 이 한 줄이 읽는 쪽에서
            #   "컬러 픽셀 클릭"을 막는다.
            "color_aligned": False,
            "has_color": color is not None,
            **_range_profile(z_mm),
        }
        _atomic(META_PATH, json.dumps(meta, ensure_ascii=False).encode("utf-8"))


if __name__ == "__main__":
    main()
