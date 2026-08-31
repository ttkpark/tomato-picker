#!/usr/bin/env python3
"""5~6단 · 카메라와 검출 — **팔 없이** 돈다(arm:=false).

    python3 /ws/tools/cam_check.py

확인하는 것:
  ⑤ realsense2_camera가 D405를 연다 — 컬러·깊이·내부파라미터가 실제로 온다
  ⑤' **깊이가 컬러에 정렬돼 있다** — 크기가 다르면 열매 중심의 깊이가 옆 잎의
      깊이가 되는데, 그때 아무 에러도 안 난다(그래서 여기서 명시적으로 본다)
  ⑥ detect_node가 살아서 /fruits를 낸다 — 열매가 0개여도 발행은 돼야 한다
     (발행이 멈추는 것과 "안 보인다"는 전혀 다른 상태다)

⚠ `depth-cam.service`를 먼저 내려야 한다. D405도 한 프로세스만 연다.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from tomato_msgs.msg import FruitArray

COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"
INFO = "/camera/camera/color/camera_info"


class CamCheck(Node):

    def __init__(self) -> None:
        super().__init__("tomato_cam_check")
        self.color: Image | None = None
        self.depth: Image | None = None
        self.info: CameraInfo | None = None
        self.fruits: FruitArray | None = None
        self.counts = {"color": 0, "depth": 0, "fruits": 0}
        self.create_subscription(Image, COLOR, self._color, 5)
        self.create_subscription(Image, DEPTH, self._depth, 5)
        self.create_subscription(CameraInfo, INFO, self._info, 5)
        self.create_subscription(FruitArray, "/fruits", self._fruit, 5)
        self.failed: list[str] = []
        self.passed = 0

    def _color(self, m):
        self.color = m
        self.counts["color"] += 1

    def _depth(self, m):
        self.depth = m
        self.counts["depth"] += 1

    def _info(self, m):
        self.info = m

    def _fruit(self, m):
        self.fruits = m
        self.counts["fruits"] += 1

    def check(self, name: str, ok: bool, detail: str = "") -> None:
        if ok:
            self.passed += 1
            print(f"  ok   {name}" + (f"  ({detail})" if detail else ""))
        else:
            self.failed.append(name)
            print(f"  FAIL {name}  {detail}")

    def gather(self, secs: float) -> None:
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main() -> int:
    rclpy.init()
    node = CamCheck()
    window = 6.0
    print(f"[5단] D405 스트림 {window:.0f}초 수집")
    node.gather(window)

    node.check("컬러 프레임", node.color is not None,
               f"{node.counts['color']}장 · {node.counts['color']/window:.1f}fps")
    node.check("깊이 프레임", node.depth is not None,
               f"{node.counts['depth']}장 · {node.counts['depth']/window:.1f}fps")
    node.check("camera_info", node.info is not None,
               (f"{node.info.width}x{node.info.height} fx={node.info.k[0]:.1f} "
                f"ppx={node.info.k[2]:.1f}") if node.info else "안 옴")

    if node.color is not None and node.depth is not None:
        same = (node.color.width == node.depth.width
                and node.color.height == node.depth.height)
        node.check("깊이가 컬러에 정렬됐다", same,
                   f"컬러 {node.color.width}x{node.color.height} / "
                   f"깊이 {node.depth.width}x{node.depth.height} — "
                   "다르면 align_depth.enable을 켜라. 이대로면 좌표가 전부 틀린다")
        node.check("깊이 인코딩이 16UC1", node.depth.encoding == "16UC1",
                   f"{node.depth.encoding}")

    print("\n[6단] detect_node")
    node.check("/fruits 발행", node.counts["fruits"] > 0,
               f"{node.counts['fruits']}번 · {node.counts['fruits']/window:.1f}Hz")
    if node.fruits is not None:
        n = len(node.fruits.fruits)
        node.check("메시지가 읽힌다", True,
                   f"열매 {n}개 · frame_id={node.fruits.header.frame_id!r}")
        node.check("좌표계가 카메라 광학 프레임이다",
                   "optical" in (node.fruits.header.frame_id or ""),
                   f"{node.fruits.header.frame_id!r} — TF로 팔 좌표로 옮길 기준이다")
        for f in node.fruits.fruits[:5]:
            print(f"       ({f.position.x*1000:.0f}, {f.position.y*1000:.0f}, "
                  f"{f.position.z*1000:.0f})mm r={f.radius*1000:.0f}mm "
                  f"픽셀({f.u},{f.v}) 깊이화소={f.depth_pixels} "
                  f"퍼짐={f.depth_spread*1000:.0f}mm")
        if n == 0:
            print("       (열매 0개 — 무대에 빨간 것이 없으면 정상이다. "
                  "발행이 도는 것이 이 단의 관심사다)")

    print()
    if node.failed:
        print(f"❌ {len(node.failed)}개 실패 / {node.passed + len(node.failed)}개 중")
        for name in node.failed:
            print(f"   - {name}")
        rclpy.shutdown()
        return 1
    print(f"✅ 전부 통과 ({node.passed}개)")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
