"""손-눈 보정 — 표본을 찍고, 풀고, static TF로 내보낸다.

계산은 [`handeye.py`](../../../../src/tomato_picker/hardware/handeye.py)가 한다
(Kabsch / 선형 최소자승 + 퇴화 표본 거절 + 잔차). 이 노드가 하는 일은 넷이다:

  ① 마커를 카메라 좌표로 읽기 (ArUco 중심 + 그 화소의 깊이)
  ② **그 프레임을 찍은 시각의** 팔 자세를 TF에서 읽기
  ③ 표본을 모아 풀고, 잔차를 사람에게 문장으로 돌려주기
  ④ 결과를 파일과 static TF로 남기기

────────────────────────────────────────────────────────────────────────
찍는 법 (부스에서 5분)

  1. 마커를 집게에 붙인다(`fixed` 모드) 또는 책상에 둔다(`on_arm` 모드).
     집게에 붙였다면 `marker_offset_tool_mm`으로 **TCP에서 마커 중심까지**를
     넣어라. 안 넣으면 그 오프셋이 통째로 보정 오차가 된다.
  2. 팔을 옮긴다 → 멈춘다 → `capture_sample`. 이걸 8번 이상,
     **좌우·상하·앞뒤로 서로 멀리 떨어뜨려서.** 한 직선 위의 표본은 아무리 많아도
     회전을 결정하지 못한다(그리고 잔차는 작게 나온다 — 가장 위험한 경우다).
  3. `solve`. 잔차 RMS가 15mm를 넘으면 **저장하지 말고** worst_index가 가리키는
     표본을 `drop_worst`로 지우고 다시 풀어라.

⚠ `fixed` 모드에서 **차체가 움직이면 보정은 그 순간 거짓이 된다** — 카메라가
   차체에 붙어 있으면 괜찮지만(같이 움직이니까), 삼각대에 있으면 아니다.
   그래서 `parent_frame`을 무엇으로 두느냐가 곧 그 선언이다:
   `arm_base`(차체에 고정) / `map`·`odom`(지면에 고정, 주행 중 무효).
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from tomato_msgs.srv import CaptureSample, SolveHandEye
from tomato_perception.fruit3d import Blob, read_blob

from . import store

try:
    from tomato_picker.hardware.handeye import (
        CalibrationError, Intrinsics, Rigid, solve_fixed, solve_on_arm,
    )
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        f"tomato_picker.hardware.handeye를 import하지 못했다: {exc}\n"
        "저장소 src/를 PYTHONPATH에 넣어라 — 보정 계산은 그 모듈이 한다."
    ) from exc

ARUCO_DICTS = {
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "5x5_50": cv2.aruco.DICT_5X5_50,
    "6x6_250": cv2.aruco.DICT_6X6_250,
    "apriltag_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


class HandeyeNode(Node):

    def __init__(self) -> None:
        super().__init__("tomato_handeye")

        self.declare_parameter("mount", "fixed")          # fixed | on_arm
        self.declare_parameter("parent_frame", "arm_base")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("camera_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("camera_link_frame", "camera_link")
        self.declare_parameter("calib_path", store.DEFAULT_PATH)
        self.declare_parameter("aruco_dict", "4x4_50")
        self.declare_parameter("marker_id", -1)           # -1 = 아무 마커나 (하나만 보일 때)
        # TCP에서 마커 중심까지 (도구 좌표 mm). fixed 모드에서만 의미가 있다.
        self.declare_parameter("marker_offset_tool_mm", [0.0, 0.0, 0.0])
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic",
                               "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("info_topic", "/camera/camera/color/camera_info")

        self._mount = str(self.get_parameter("mount").value)
        if self._mount not in ("fixed", "on_arm"):
            raise SystemExit(f"mount는 'fixed' 또는 'on_arm'이다 (받은 값: {self._mount!r})")

        self._bridge = CvBridge()
        self._intr: Intrinsics | None = None
        self._frame: tuple[np.ndarray, np.ndarray, object] | None = None

        self._cam_pts: list[tuple[float, float, float]] = []
        self._base_pts: list[tuple[float, float, float]] = []
        self._tool_frames: list[Rigid] = []
        self._labels: list[str] = []

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._static = StaticTransformBroadcaster(self)

        self.create_subscription(
            CameraInfo, self.get_parameter("info_topic").value, self._on_info, 1)
        sync = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, self.get_parameter("color_topic").value),
             Subscriber(self, Image, self.get_parameter("depth_topic").value)],
            queue_size=5, slop=0.05)
        sync.registerCallback(self._on_frame)
        self._sync = sync

        self.create_service(CaptureSample, "handeye/capture_sample", self._on_capture)
        self.create_service(SolveHandEye, "handeye/solve", self._on_solve)
        self.create_service(Trigger, "handeye/clear_samples", self._on_clear)
        self.create_service(Trigger, "handeye/drop_worst", self._on_drop_worst)

        self._last_fit = None
        self._restore()

    # ------------------------------------------------------------------
    # 입력
    # ------------------------------------------------------------------

    def _on_info(self, msg: CameraInfo) -> None:
        if self._intr is None:
            k = msg.k
            self._intr = Intrinsics(width=msg.width, height=msg.height,
                                    fx=float(k[0]), fy=float(k[4]),
                                    ppx=float(k[2]), ppy=float(k[5]), model="none")

    def _on_frame(self, color_msg: Image, depth_msg: Image) -> None:
        color = self._bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth = self._bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
        self._frame = (color, depth, color_msg.header)

    # ------------------------------------------------------------------
    # 표본 찍기
    # ------------------------------------------------------------------

    def _on_capture(self, req: CaptureSample.Request,
                    res: CaptureSample.Response) -> CaptureSample.Response:
        res.samples = len(self._cam_pts)
        if self._frame is None or self._intr is None:
            res.ok = False
            res.detail = "아직 프레임이나 camera_info가 없다 — 카메라 드라이버를 확인하라."
            return res

        color, depth, header = self._frame
        try:
            cam_pt = self._marker_in_camera(color, depth)
        except LookupError as exc:
            res.ok = False
            res.detail = str(exc)
            return res

        try:
            tool = self._tool_rigid(header.stamp)
        except Exception as exc:  # noqa: BLE001 - TF 실패 이유를 그대로 올린다
            res.ok = False
            res.detail = (f"그 프레임 시각의 팔 자세를 TF에서 못 읽었다: {exc}. "
                          "arm_node가 /joint_states를 내고 있는지, "
                          "robot_state_publisher가 떠 있는지 확인하라.")
            return res

        self._cam_pts.append(cam_pt)
        self._tool_frames.append(tool)
        offset = np.array(list(self.get_parameter("marker_offset_tool_mm").value),
                          dtype=float)
        self._base_pts.append(tuple(tool.apply(offset)))
        self._labels.append(req.label or f"#{len(self._cam_pts)}")

        res.ok = True
        res.samples = len(self._cam_pts)
        res.detail = (f"표본 {res.samples}개 — 카메라 "
                      f"({cam_pt[0]:.0f}, {cam_pt[1]:.0f}, {cam_pt[2]:.0f})mm / "
                      f"팔 ({self._base_pts[-1][0]:.0f}, {self._base_pts[-1][1]:.0f}, "
                      f"{self._base_pts[-1][2]:.0f})mm")
        return res

    def _marker_in_camera(self, color, depth) -> tuple[float, float, float]:
        """ArUco 중심 + 그 자리의 깊이 → 카메라 좌표(mm).

        코너 4점의 PnP 자세를 쓰지 않고 **중심 + 깊이**를 쓰는 이유 — 작은 태그의
        PnP 자세는 거리 방향으로 잘 흔들리는데, D405는 그 거리를 직접 잰다.
        우리에게 필요한 것도 자세가 아니라 **점 하나**다.
        """
        name = str(self.get_parameter("aruco_dict").value)
        if name not in ARUCO_DICTS:
            raise LookupError(f"모르는 aruco_dict={name!r}. 쓸 수 있는 것: "
                              + ", ".join(ARUCO_DICTS))
        dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[name])
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(color)
        if ids is None or len(ids) == 0:
            raise LookupError("마커가 안 보인다 — 카메라 화각 안에 들어왔는지, "
                              "너무 기울지 않았는지, 초점이 맞는지 확인하라.")

        want = int(self.get_parameter("marker_id").value)
        index = 0
        if want >= 0:
            matches = [i for i, v in enumerate(ids.flatten()) if int(v) == want]
            if not matches:
                raise LookupError(f"마커 {want}번이 안 보인다 (보이는 것: "
                                  + ", ".join(str(int(v)) for v in ids.flatten()) + ")")
            index = matches[0]
        elif len(ids) > 1:
            raise LookupError(
                f"마커가 {len(ids)}개 보인다 — 어느 것인지 알 수 없다. "
                "marker_id 파라미터로 지정하거나 하나만 보이게 하라.")

        pts = corners[index].reshape(-1, 2)
        u, v = float(pts[:, 0].mean()), float(pts[:, 1].mean())
        # 태그 안쪽만 본다 — 반지름을 코너까지 거리의 절반으로 잡아 가장자리를 피한다.
        radius = float(np.linalg.norm(pts - np.array([u, v]), axis=1).mean()) * 0.5
        reading = read_blob(self._intr, depth,
                            Blob(u=u, v=v, radius_px=max(3.0, radius),
                                 pixels=int(math.pi * radius * radius), ripe=False))
        if not reading.ok:
            raise LookupError(f"마커는 보이는데 깊이를 못 믿는다 — {reading.reason}")
        return reading.point_mm

    def _tool_rigid(self, stamp) -> Rigid:
        """TF: parent_frame → tool_frame 를 **그 시각으로** 조회해 Rigid(mm)로.

        `stamp`를 쓰는 것이 핵심이다. "지금"으로 조회하면 표본을 찍는 동안 팔이
        미세하게 흔들린 만큼이 그대로 잔차가 된다.
        """
        parent = str(self.get_parameter("parent_frame").value)
        tool = str(self.get_parameter("tool_frame").value)
        tf = self._tf.lookup_transform(parent, tool, Time.from_msg(stamp))
        t = tf.transform.translation
        q = tf.transform.rotation
        return Rigid(store.rotation((q.x, q.y, q.z, q.w)),
                     np.array([t.x, t.y, t.z]) * 1000.0)

    # ------------------------------------------------------------------
    # 풀기
    # ------------------------------------------------------------------

    def _on_solve(self, req: SolveHandEye.Request,
                  res: SolveHandEye.Response) -> SolveHandEye.Response:
        res.samples = len(self._cam_pts)
        res.worst_index = -1
        try:
            if self._mount == "fixed":
                fit = solve_fixed(self._cam_pts, self._base_pts)
            else:
                fit = solve_on_arm(self._cam_pts, self._tool_frames)
        except CalibrationError as exc:
            res.ok = False
            res.detail = str(exc)
            return res

        self._last_fit = fit
        res.ok = True
        res.good = bool(fit.good)
        res.rms_mm = float(fit.rms_mm)
        res.max_mm = float(fit.max_mm)
        res.worst_index = int(fit.worst_index())
        res.detail = fit.summary()

        if not fit.good:
            res.detail += (f" · 가장 어긋난 표본은 {res.worst_index}번"
                           f"({self._labels[res.worst_index]}) — drop_worst로 지우고 "
                           "다시 풀어 보라. 저장하지 않았다.")
            return res

        self._broadcast(fit.transform)
        if req.save:
            path = store.save(fit, self._mount,
                              str(self.get_parameter("parent_frame").value),
                              str(self.get_parameter("camera_optical_frame").value),
                              str(self.get_parameter("calib_path").value))
            res.detail += f" · 저장: {path}"
        else:
            res.detail += " · TF만 갱신(저장 안 함, save=true로 저장)"
        return res

    def _on_clear(self, _req, res: Trigger.Response) -> Trigger.Response:
        n = len(self._cam_pts)
        self._cam_pts.clear()
        self._base_pts.clear()
        self._tool_frames.clear()
        self._labels.clear()
        res.success = True
        res.message = f"표본 {n}개 지움"
        return res

    def _on_drop_worst(self, _req, res: Trigger.Response) -> Trigger.Response:
        if self._last_fit is None:
            res.success = False
            res.message = "먼저 solve를 해야 어느 표본이 나쁜지 안다"
            return res
        i = int(self._last_fit.worst_index())
        if not 0 <= i < len(self._cam_pts):
            res.success = False
            res.message = "지울 표본이 없다"
            return res
        label = self._labels[i]
        for seq in (self._cam_pts, self._base_pts, self._tool_frames, self._labels):
            del seq[i]
        self._last_fit = None
        res.success = True
        res.message = f"{i}번({label}) 지움 — 남은 표본 {len(self._cam_pts)}개. 다시 solve하라"
        return res

    # ------------------------------------------------------------------
    # TF로 내보내기
    # ------------------------------------------------------------------

    def _broadcast(self, transform: Rigid) -> None:
        """보정 결과를 static TF로. **camera_link에 붙인다**(store.retarget 참고)."""
        parent = str(self.get_parameter("parent_frame").value)
        optical = str(self.get_parameter("camera_optical_frame").value)
        link = str(self.get_parameter("camera_link_frame").value)

        target_frame, final = optical, transform
        if link and link != optical:
            try:
                tf = self._tf.lookup_transform(link, optical, Time())
                t, q = tf.transform.translation, tf.transform.rotation
                link_to_optical = Rigid(store.rotation((q.x, q.y, q.z, q.w)),
                                        np.array([t.x, t.y, t.z]) * 1000.0)
                final = store.retarget(transform, link_to_optical)
                target_frame = link
            except Exception as exc:  # noqa: BLE001
                # 드라이버가 그 TF를 안 내면 광학 프레임에 직접 붙인다. 이때
                # 드라이버도 같은 프레임을 발행하고 있으면 부모가 둘이 되니
                # **경고를 크게 남긴다** — 조용히 넘어가면 조회가 오락가락한다.
                self.get_logger().warning(
                    f"{link} → {optical} TF를 못 읽었다({exc}). {optical}에 직접 붙인다 — "
                    "카메라 드라이버가 같은 프레임을 발행 중이면 부모가 둘이 되어 "
                    "TF 조회가 불안정해진다.")

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = target_frame
        msg.transform.translation.x = float(final.t[0]) / 1000.0
        msg.transform.translation.y = float(final.t[1]) / 1000.0
        msg.transform.translation.z = float(final.t[2]) / 1000.0
        qx, qy, qz, qw = store.quaternion(final.R)
        msg.transform.rotation.x, msg.transform.rotation.y = qx, qy
        msg.transform.rotation.z, msg.transform.rotation.w = qz, qw
        self._static.sendTransform(msg)

        roll, pitch, yaw = final.rpy_deg
        self.get_logger().info(
            f"static TF 발행: {parent} → {target_frame} · "
            f"t=({final.t[0]:.0f}, {final.t[1]:.0f}, {final.t[2]:.0f})mm · "
            f"rpy=({roll:.1f}, {pitch:.1f}, {yaw:.1f})°")

    def _restore(self) -> None:
        """저장된 보정이 있으면 켜자마자 쏜다. 없으면 **아무것도 안 쏜다.**"""
        data = store.load(str(self.get_parameter("calib_path").value))
        if not data:
            self.get_logger().warning(
                "저장된 손-눈 보정이 없다 — 카메라 TF를 발행하지 않는다. "
                "이 상태에서 열매 좌표를 팔 좌표로 옮기려 하면 TF 조회가 실패한다"
                "(그게 옳은 동작이다: 보정 안 된 값으로 팔을 움직이지 않는다).")
            return
        if data.get("mount") != self._mount:
            self.get_logger().error(
                f"저장된 보정은 mount={data.get('mount')!r}인데 지금 파라미터는 "
                f"{self._mount!r}다 — 다른 장착 방식의 값을 쓰면 팔이 엉뚱한 데로 간다. "
                "발행하지 않는다.")
            return
        self._broadcast(Rigid.from_dict(data["transform"]))
        self.get_logger().info(
            f"저장된 보정 복원 — 표본 {data.get('samples')}개, "
            f"잔차 RMS {data.get('rms_mm')}mm ({data.get('saved_at')})")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandeyeNode()
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
