"""
pick_node.py
============
실행 노드. 골프공을 검출 -> 캘리브레이션으로 로봇좌표 변환 -> Dobot이 집기.

흐름:
  1) 카메라 이미지에서 골프공(또는 색 마커) 검출 -> (u,v)
  2) Affine 행렬로 (u,v) -> (x,y) mm
  3) 안전높이로 접근 -> pick_z까지 하강 -> 흡착 ON -> 들어올림
  4) (옵션) place 좌표로 이동 -> 흡착 OFF

실행:
  ros2 run dobot_realsense_calib pick_node \\
    --ros-args -p calib_path:=./calibration.yaml \\
               -p target:=ball \\
               -p auto:=false      # true면 검출 즉시 자동 집기, false면 [p]키로 집기
"""

import yaml
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .robot_interface import DobotInterface
from . import vision


class PickNode(Node):
    def __init__(self):
        super().__init__('pick_node')
        self.declare_parameter('calib_path', './calibration.yaml')
        self.declare_parameter('target', 'ball')      # 'ball' 또는 HSV_PRESETS 키
        self.declare_parameter('auto', False)
        self.declare_parameter('safe_z', 60.0)        # 접근 안전 높이 (mm)
        self.declare_parameter('lift_z', 60.0)        # 집은 뒤 들어올릴 높이
        self.declare_parameter('place_xy', [200.0, -120.0])  # 내려놓을 위치
        self.declare_parameter('pose_topic', 'dobot_pose_raw')

        with open(self.get_parameter('calib_path').value) as f:
            calib = yaml.safe_load(f)
        self.M = np.array(calib['matrix'], dtype=np.float64)
        self.pick_z = float(calib.get('pick_z', 0.0))
        self.image_topic = calib.get('image_topic') or '/camera/camera1/color/image_raw'

        self.target = self.get_parameter('target').value
        self.auto = self.get_parameter('auto').value
        self.safe_z = self.get_parameter('safe_z').value
        self.lift_z = self.get_parameter('lift_z').value
        self.place_xy = self.get_parameter('place_xy').value

        self.bridge = CvBridge()
        self.latest = None
        self.busy = False

        self.robot = DobotInterface(
            self, pose_topic=self.get_parameter('pose_topic').value)
        self.create_subscription(Image, self.image_topic, self._img_cb, 10)
        self.create_timer(1.0 / 30.0, self._loop)
        self.get_logger().info(
            f"pick_node 시작. target='{self.target}', auto={self.auto}\n"
            f"  pick_z={self.pick_z:.1f}mm. [p] 집기 실행  [q] 종료")

    def _img_cb(self, msg):
        self.latest = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def pixel_to_robot(self, u, v):
        xy = self.M @ np.array([u, v, 1.0])
        return float(xy[0]), float(xy[1])

    def _detect(self, bgr):
        if self.target == 'ball':
            return vision.detect_golf_ball(bgr)
        if self.target == 'red':
            u, v, _ = vision.detect_color_marker(bgr, *vision.HSV_PRESETS['red_low'])
            if u is None:
                return vision.detect_color_marker(bgr, *vision.HSV_PRESETS['red_high'])
            return u, v, bgr.copy()
        lo, hi = vision.HSV_PRESETS.get(self.target, vision.HSV_PRESETS['blue'])
        return vision.detect_color_marker(bgr, lo, hi)

    def _loop(self):
        if self.latest is None or self.busy:
            return
        u, v, vis = self._detect(self.latest)
        if u is not None:
            rx, ry = self.pixel_to_robot(u, v)
            cv2.putText(vis, f"robot=({rx:.0f},{ry:.0f})mm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)
        else:
            cv2.putText(vis, "target not found", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow('pick_node', vis)
        key = cv2.waitKey(1) & 0xFF

        do_pick = (self.auto and u is not None) or key == ord('p')
        if do_pick and u is not None:
            rx, ry = self.pixel_to_robot(u, v)
            self._pick_sequence(rx, ry)
        if key == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def _pick_sequence(self, x, y):
        self.busy = True
        try:
            self.get_logger().info(f"집기 시작 -> ({x:.1f},{y:.1f})")
            # 1) 대상 위 안전높이로 이동
            self.robot.move_to(x, y, self.pick_z + self.safe_z)
            # 2) 집는 높이로 하강
            self.robot.move_to(x, y, self.pick_z)
            # 3) 흡착 ON
            self.robot.set_suction(True)
            # 4) 들어올림
            self.robot.move_to(x, y, self.pick_z + self.lift_z)
            # 5) 내려놓을 위치로 이동
            px, py = self.place_xy
            self.robot.move_to(px, py, self.pick_z + self.lift_z)
            self.robot.move_to(px, py, self.pick_z)
            # 6) 흡착 OFF
            self.robot.set_suction(False)
            self.robot.move_to(px, py, self.pick_z + self.lift_z)
            self.get_logger().info("집기 완료")
        finally:
            self.busy = False


def main():
    rclpy.init()
    node = PickNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
