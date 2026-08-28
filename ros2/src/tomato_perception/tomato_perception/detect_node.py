"""D405 컬러+깊이 → `/fruits` (열매 3D). 좌표계는 **컬러 광학 프레임**이다.

왜 팔 좌표로 바꿔서 내보내지 않는가 — 검출은 "카메라가 무엇을 봤는가"만 말해야
하고, "그게 팔 기준 어디인가"는 TF가 답할 일이다. 여기서 미리 바꿔 버리면
보정 전에는 발행이 멈추고, 보정을 다시 하면 과거 메시지의 뜻이 달라진다.
소비자(arm_node)가 `tf2`로 그 순간의 변환을 조회한다.

⚠ **깊이는 컬러에 정렬(align to color)돼 있어야 한다.** realsense2_camera의
   `align_depth.enable:=true`. 정렬을 안 하면 같은 (u,v)가 두 센서에서 다른 곳을
   가리켜, 열매 중심의 깊이가 옆 잎의 깊이가 된다 — 그리고 **아무 에러도 안 난다.**

색 임계는 기존 [`vision/color_detect.py`](../../../../src/tomato_picker/vision/color_detect.py)의
값을 그대로 파라미터 기본값으로 가져왔다. 검출기를 YOLO로 바꿀 때 갈아끼우는
지점도 여기 하나다 — 아래 `_blobs()`만 교체하면 나머지는 안 바뀐다.
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from tomato_msgs.msg import Fruit3D, FruitArray

from .fruit3d import Blob, read_all

try:
    from tomato_picker.hardware.handeye import Intrinsics
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        f"tomato_picker.hardware.handeye를 import하지 못했다: {exc}\n"
        "저장소 src/를 PYTHONPATH에 넣어라 (내부 파라미터·역투영을 거기서 쓴다)."
    ) from exc

# color_detect.py의 RED_HSV_RANGES / GREEN_HSV_RANGE와 같은 값.
RED_RANGES = [[0, 120, 70, 10, 255, 255], [170, 120, 70, 180, 255, 255]]
GREEN_RANGE = [35, 80, 40, 85, 255, 255]


class DetectNode(Node):

    def __init__(self) -> None:
        super().__init__("tomato_detect")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic",
                               "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("min_pixels", 400)      # MIN_FRUIT_AREA_PX와 같은 뜻
        self.declare_parameter("detect_unripe", False)  # 초록 열매도 낼 것인가
        self.declare_parameter("red_ranges", [float(v) for r in RED_RANGES for v in r])
        self.declare_parameter("green_range", [float(v) for v in GREEN_RANGE])
        self.declare_parameter("annotate", True)       # 진단 이미지 발행

        self._bridge = CvBridge()
        self._intr: Intrinsics | None = None
        self._pub = self.create_publisher(FruitArray, "fruits", 10)
        self._debug = self.create_publisher(Image, "~/annotated", 1)

        self.create_subscription(
            CameraInfo, self.get_parameter("info_topic").value, self._on_info, 1)

        sync = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, self.get_parameter("color_topic").value),
             Subscriber(self, Image, self.get_parameter("depth_topic").value)],
            queue_size=5, slop=0.05)
        sync.registerCallback(self._on_frame)
        self._sync = sync  # 참조를 놓으면 콜백이 조용히 안 온다

        self._warned_info = False

    # ------------------------------------------------------------------

    def _on_info(self, msg: CameraInfo) -> None:
        """내부 파라미터. 매 프레임 다시 만들 필요가 없어 한 번만 잡는다."""
        if self._intr is not None:
            return
        k = msg.k
        self._intr = Intrinsics(
            width=msg.width, height=msg.height,
            fx=float(k[0]), fy=float(k[4]), ppx=float(k[2]), ppy=float(k[5]),
            # 컬러에 정렬된 깊이를 쓰므로 왜곡은 드라이버가 이미 처리한 상태다.
            # (D405의 depth 스트림은 정류돼 있어 계수가 전부 0이다.)
            model="none")
        self.get_logger().info(
            f"내부 파라미터 수신: {msg.width}x{msg.height} "
            f"fx={k[0]:.1f} fy={k[4]:.1f} ppx={k[2]:.1f} ppy={k[5]:.1f}")

    def _on_frame(self, color_msg: Image, depth_msg: Image) -> None:
        if self._intr is None:
            if not self._warned_info:
                self._warned_info = True
                self.get_logger().warning(
                    "camera_info를 아직 못 받았다 — 내부 파라미터 없이는 픽셀을 "
                    "3D로 못 푼다. info_topic 파라미터를 확인하라.")
            return

        color = self._bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth = self._bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        if depth.shape[:2] != color.shape[:2]:
            self.get_logger().error(
                f"깊이 {depth.shape[:2]}와 컬러 {color.shape[:2]}의 크기가 다르다 — "
                "align_depth.enable:=true 로 켜야 한다. 이대로는 좌표가 전부 틀린다.")
            return
        # D405의 depth는 uint16 밀리미터. float으로 오면 m일 수 있으니 확인한다.
        depth_mm = depth.astype(np.float64)
        if depth.dtype in (np.float32, np.float64) and float(np.nanmax(depth_mm)) < 20.0:
            depth_mm = depth_mm * 1000.0

        blobs, masks = self._blobs(color)
        readings = read_all(self._intr, depth_mm, blobs, masks)

        out = FruitArray()
        # **컬러 프레임의 시각**을 그대로 쓴다 — 이 값이 TF 조회의 기준이 된다.
        out.header = color_msg.header
        dropped = []
        for r in readings:
            if not r.ok:
                dropped.append(r)
                continue
            f = Fruit3D()
            f.position.x = r.point_mm[0] / 1000.0
            f.position.y = r.point_mm[1] / 1000.0
            f.position.z = r.point_mm[2] / 1000.0
            f.radius = float(r.radius_mm / 1000.0)
            f.ripe = r.blob.ripe
            f.confidence = float(r.blob.confidence)
            f.u = int(r.blob.u)
            f.v = int(r.blob.v)
            f.pixels = int(r.blob.pixels)
            f.depth_pixels = int(r.depth_pixels)
            f.depth_spread = float(r.spread_mm / 1000.0)
            out.fruits.append(f)
        self._pub.publish(out)

        if dropped:
            # 버려진 이유를 **한 줄로 모아** 남긴다. "6개 중 2개만 잡힌다"의 원인이
            # 검출이 아니라 깊이였던 경우를 몇 초 만에 구분할 수 있어야 한다.
            self.get_logger().info(
                f"{len(out.fruits)}개 발행 / {len(dropped)}개 버림 — "
                + " | ".join(d.reason for d in dropped[:3]))

        if self.get_parameter("annotate").value:
            self._publish_annotated(color, color_msg.header, readings)

    # ------------------------------------------------------------------

    def _blobs(self, bgr: np.ndarray) -> tuple[list[Blob], list[np.ndarray]]:
        """HSV 마스킹 → 덩이. **여기만 갈아끼우면 YOLO가 된다.**"""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        min_px = int(self.get_parameter("min_pixels").value)

        flat = list(self.get_parameter("red_ranges").value)
        ranges = [(flat[i:i + 3], flat[i + 3:i + 6]) for i in range(0, len(flat), 6)]
        red = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            red |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))

        found: list[Blob] = []
        masks: list[np.ndarray] = []
        groups = [(red, True)]
        if self.get_parameter("detect_unripe").value:
            g = list(self.get_parameter("green_range").value)
            groups.append((cv2.inRange(hsv, np.array(g[:3], np.uint8),
                                       np.array(g[3:], np.uint8)), False))

        for mask, ripe in groups:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = int(cv2.contourArea(c))
                if area < min_px:
                    continue
                m = cv2.moments(c)
                if m["m00"] == 0:
                    continue
                (_, _), radius = cv2.minEnclosingCircle(c)
                found.append(Blob(u=m["m10"] / m["m00"], v=m["m01"] / m["m00"],
                                  radius_px=float(radius), pixels=area, ripe=ripe))
                one = np.zeros(mask.shape, dtype=np.uint8)
                cv2.drawContours(one, [c], -1, 255, cv2.FILLED)
                masks.append(one.astype(bool))
        return found, masks

    def _publish_annotated(self, bgr, header, readings) -> None:
        img = bgr.copy()
        for r in readings:
            center = (int(r.blob.u), int(r.blob.v))
            color = (0, 200, 0) if r.ok else (0, 0, 255)
            cv2.circle(img, center, int(r.blob.radius_px), color, 2)
            label = (f"{r.depth_mm:.0f}mm" if r.ok else "X")
            cv2.putText(img, label, (center[0] + 6, center[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        msg = self._bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header = header
        self._debug.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
