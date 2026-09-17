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

⚠ **센서가 하나다 — 컬러와 깊이가 노출·게인을 공유한다** (2026-09-18 실측).
   `query_devices()[0].query_sensors()`가 `Stereo Module` **하나**를 내놓고 그것이
   color·depth·infrared를 전부 낸다. 그래서 "컬러만 밝게"가 없다 — 게인을 올리면
   깊이 잡음도 같이 오른다. 공짜가 아니니 **손잡이를 기본값으로 켜 두지 않는다**.
   같은 날 잰 사다리(어두운 무대, 848x480@30):

       설정                       컬러 평균  최댓값  깊이 유효율
       자동노출(기본)                  7.7     47      0.043
       exp= 33000us gain= 16           0.3      3      0.009
       exp= 33000us gain=248           7.8     52      0.040
       exp=100000us gain=128          10.6     61      0.184
       exp=165000us gain=248 (최대)   33.9    113      0.326

   두 가지가 여기서 갈렸다. ① **컬러는 손잡이를 끝까지 올려도 평균 34가 한계다**
   — 7.2가 나오는 것은 발행기 탓이 아니라 **무대가 정말 어둡기 때문**이다(조명을
   켜야 한다). ② 그런데 **깊이 유효율은 0.043 → 0.326으로 7.6배**가 된다. 컬러를
   포기하고 깊이만 쓰는 일(손-눈 표본 채집)에는 이 손잡이가 실제로 값을 한다.

⚠ **D405는 근거리 전용이다** — 이상 동작범위 7~50cm, 1m를 넘으면 급격히
   나빠진다. 2026-08-28 삼각대 위치에서는 장면의 **0%**가 50cm 안에 없었고
   전부 1~2m였다. 그 상태로는 보정을 아무리 잘해도 열매를 못 집는다.
   그래서 meta.json에 거리 분포를 함께 실어, 대시보드가 "너무 멀다"를
   말할 수 있게 한다.
"""

from __future__ import annotations

import collections
import json
import os
import sys
import time

import cv2
import numpy as np

# ⚠ pyrealsense2는 **함수 안에서** 불러온다 — 젯슨의 vision venv에만 있어서
#   모듈 최상단에 두면 PC 자체검증(tools/eye_check.py)이 이 파일을 열지도 못한다.
#   이 저장소의 규칙은 "숫자는 젯슨에 올리기 전에 PC에서 확인한다"이므로,
#   순수 계산부(color_settings·clamp_option)는 카메라 없이 import돼야 한다.

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
# 노출·게인 손잡이 (2026-09-18 추가). 기본은 **건드리지 않음** — 지금까지의
# 동작(자동노출)을 그대로 둔다. 어두운 무대에서 깊이 표본을 채집할 때만 켠다.
#   D405_AUTO_EXPOSURE=0 D405_EXPOSURE_US=165000 D405_GAIN=248
# ⚠ 이 손잡이는 깊이에도 걸린다(센서가 하나다 — 위 ⚠ 참고). 그리고 165ms 노출은
#   스트림을 ~6fps로 떨어뜨리고 움직이는 것을 흐리게 만든다. 팔이 멎어 있는
#   채집에는 괜찮지만 주행 중에는 쓰지 마라.
AUTO_EXPOSURE = os.environ.get("D405_AUTO_EXPOSURE")
EXPOSURE_US = os.environ.get("D405_EXPOSURE_US")
GAIN = os.environ.get("D405_GAIN")

# 실제로 나오는 발행 주기를 재는 창 (2026-09-18 추가). PUBLISH_FPS는 **바라는**
# 값이지 나오는 값이 아니다 — 위 ⚠대로 165ms 노출을 걸면 센서가 6fps밖에 못
# 내는데 발행기는 8을 적어 놓고, 읽는 쪽은 mtime만 보므로 "신선하다"로 읽힌다.
# 이 저장소의 1번 병(지령은 나가는데 아무 일도 안 일어난다)이 프레임 주기에서
# 재발하는 자리다. 그러니 **깊이 단위·유효거리와 같은 규칙** — 카메라가 스스로
# 말하게 하고, 읽는 코드에 fps를 박지 않는다.
# 창을 회수가 아니라 **초**로 주는 이유: 회수로 주면 fps가 떨어질수록 창이
# 길어져서, 정작 느려진 것을 늦게 본다.
FPS_WINDOW_SEC = float(os.environ.get("D405_FPS_WINDOW_SEC", "4"))


def fps_measure(stamps, now: float, window_sec: float = FPS_WINDOW_SEC) -> float | None:
    """발행 시각 창을 갱신하고 **실측** fps를 낸다. 표본이 모자라면 None.

    왜 순수 함수인가: 이 숫자가 틀리면 읽는 쪽이 "느리다"를 거꾸로 말한다.
    카메라 없이 PC에서 자른다(`tools/eye_check.py`).

    간격 수 ÷ 걸린 시간이다(표본 수가 아니다) — 표본 2개는 간격 1개다.
    None은 "아직 모른다"이지 "0이다"가 아니므로 meta에도 그대로 실어 보낸다.
    """
    stamps.append(now)
    while len(stamps) > 2 and now - stamps[0] > window_sec:
        stamps.popleft()
    span = stamps[-1] - stamps[0]
    if len(stamps) < 2 or span <= 0:
        return None
    return round((len(stamps) - 1) / span, 2)


def color_settings(env: dict | None = None) -> dict:
    """환경변수 → 스테레오 모듈에 넣을 노출 설정. 아무것도 안 주면 빈 dict.

    왜 순수 함수인가: 카메라가 있어야만 확인할 수 있는 코드는 젯슨에 올려 봐야
    틀린 걸 안다. 값 해석은 PC에서 자른다(`tools/eye_check.py`).

    규칙 하나만 기억하면 된다 — **노출이나 게인을 손으로 주면 자동노출은 꺼진다.**
    켜 둔 채 값을 넣으면 자동노출이 곧바로 덮어써서 "지령은 나갔는데 아무 일도
    안 일어나는" 이 저장소의 1번 병이 그대로 재현된다. `D405_AUTO_EXPOSURE=1`을
    함께 주면 그건 모순이므로 **거절**한다(조용히 한쪽을 이기게 두지 않는다).
    """
    env = os.environ if env is None else env

    def num(key: str) -> float | None:
        raw = env.get(key)
        if raw is None or raw == "":
            return None
        try:
            return float(raw)
        except ValueError:
            raise ValueError(f"{key}={raw!r} — 숫자여야 한다") from None

    out: dict = {}
    exposure, gain = num("D405_EXPOSURE_US"), num("D405_GAIN")
    raw_auto = env.get("D405_AUTO_EXPOSURE")
    auto = None if raw_auto in (None, "") else raw_auto not in ("0", "false", "no")

    if (exposure is not None or gain is not None) and auto:
        raise ValueError("D405_AUTO_EXPOSURE=1과 수동 노출/게인은 같이 못 준다 — "
                         "자동노출이 덮어써서 수동값이 무시된다")
    if exposure is not None or gain is not None:
        out["auto_exposure"] = False        # 수동값을 주면 자동은 꺼진다
    elif auto is not None:
        out["auto_exposure"] = auto
    if exposure is not None:
        out["exposure"] = exposure
    if gain is not None:
        out["gain"] = gain
    return out


def clamp_option(name: str, value: float, lo: float, hi: float) -> float:
    """센서가 말한 범위로 자른다. 범위 밖을 그대로 넣으면 librealsense가 던진다.

    자르되 **조용히 자르지 않는다** — 넣은 값과 실제 값이 다르면 그 자리에서
    말해야, 나중에 "왜 안 밝지"로 반나절을 쓰지 않는다.
    """
    cut = min(hi, max(lo, value))
    if cut != value:
        print(f"⚠ {name}={value:g}가 범위 [{lo:g},{hi:g}] 밖이라 {cut:g}로 잘랐다",
              file=sys.stderr, flush=True)
    return cut


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


def apply_color_settings(sensor, settings: dict, rs_mod) -> dict:
    """설정을 센서에 넣고 **되읽은 값**을 돌려준다(넣은 값이 아니라).

    되읽는 이유: 자동노출이 켜져 있으면 exposure는 카메라가 정하고, 범위 밖 값은
    잘린다. meta.json에는 **실제로 걸린 값**이 가야 읽는 쪽이 속지 않는다.
    """
    opts = {"auto_exposure": rs_mod.option.enable_auto_exposure,
            "exposure": rs_mod.option.exposure, "gain": rs_mod.option.gain}
    # ⚠ 자동노출을 먼저 끈다 — 켜진 채 exposure를 넣으면 다음 프레임에 덮인다.
    for key in ("auto_exposure", "exposure", "gain"):
        if key not in settings:
            continue
        opt, value = opts[key], float(settings[key])
        if not sensor.supports(opt):
            print(f"⚠ 이 카메라는 {key}를 지원하지 않는다 — 건너뛴다",
                  file=sys.stderr, flush=True)
            continue
        rng = sensor.get_option_range(opt)
        sensor.set_option(opt, clamp_option(key, value, rng.min, rng.max))

    out = {}
    for key, opt in opts.items():
        if sensor.supports(opt):
            out[key] = sensor.get_option(opt)
    out["auto_exposure"] = bool(out.get("auto_exposure", 0))
    return out


def color_stats(color: np.ndarray) -> dict:
    """화면이 얼마나 밝은가 — meta에 실어 읽는 쪽이 "어둡다"를 말할 수 있게.

    2026-09-18에 이걸 안 실어서 하루를 썼다: `target_check.find_dots`의 contrast
    게이트가 0개를 내놓는 것을 "마커판이 시야 밖"으로 읽었는데, 실제로는 화면
    평균이 7.2/255라 마커가 눈앞에 있어도 못 넘는 상태였다. 밝기를 숫자로 내보내면
    그 둘이 갈린다. 임계는 여기에 박지 않는다 — 숫자만 주고 판단은 읽는 쪽이 한다.

    ⚠ 최댓값만으로는 못 가른다 — 어두운 프레임에도 반짝이는 화소 하나는 있다
      (2026-09-18 실측: 평균 7.2인 화면의 최댓값이 42였다). 그래서 **99백분위**를
      함께 싣는다. 점 검출은 고리(밝은 종이)가 알맹이보다 30만큼 밝기를 요구하니,
      화면의 1%도 30을 못 넘으면 마커 크기의 밝은 고리는 존재할 수 없다.
    """
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    return {"color_mean": round(float(gray.mean()), 1),
            "color_p99": int(np.percentile(gray, 99)),
            "color_max": int(gray.max())}


def main() -> None:
    import pyrealsense2 as rs        # 젯슨 vision venv에만 있다(위 ⚠ 참고)

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

    # ⚠ D405는 센서가 하나다 — 이 'depth sensor'가 컬러도 낸다(위 ⚠ 참고).
    color_opts = apply_color_settings(dev.first_depth_sensor(), color_settings(), rs)

    serial = dev.get_info(rs.camera_info.serial_number)
    print(f"D405 {serial} fw={dev.get_info(rs.camera_info.firmware_version)} "
          f"{WIDTH}x{HEIGHT}@{FPS} depth_scale={scale_m} ({scale_mm}mm/단위) "
          f"auto_exposure={color_opts.get('auto_exposure')} "
          f"exposure={color_opts.get('exposure')} gain={color_opts.get('gain')}",
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
    stamps = collections.deque()
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
        measured_fps = fps_measure(stamps, now)

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
            # 바라는 주기와 **나오는** 주기를 나란히 싣는다(위 FPS_WINDOW_SEC 참고).
            # 판정("느리다")은 읽는 쪽이 한다 — 발행기는 임계를 안 박는다.
            "capture_fps": float(FPS),
            "publish_fps": PUBLISH_FPS,
            "measured_fps": measured_fps,
            "min_mm": MIN_MM,
            "max_mm": MAX_MM,
            "near_mm": [70.0, 500.0],
            # 깊이를 컬러 격자에 정렬해 넘긴다(위 ⚠ 참고) — 컬러 화면의 픽셀이
            # 곧 깊이 격자의 같은 자리다. Astra는 여기가 false다.
            "color_aligned": True,
            "has_color": True,
            # 실제로 걸린 노출 설정과 그 결과 밝기. 둘을 같이 실어야 "어둡다"의
            # 원인이 무대인지 설정인지 읽는 쪽에서 갈린다.
            **color_opts,
            **color_stats(color),
            **_range_profile(z_mm),
        }
        _atomic(META_PATH, json.dumps(meta, ensure_ascii=False).encode("utf-8"))


if __name__ == "__main__":
    main()
