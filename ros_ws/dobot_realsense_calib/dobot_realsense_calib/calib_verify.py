"""
calib_verify.py
===============
캘리브레이션 검증용. 카메라 창에서 마우스로 클릭하면
변환된 로봇 좌표로 실제로 이동합니다.

실행:
  ros2 run dobot_realsense_calib calib_verify \\
    --ros-args -p calib_path:=./calibration.yaml \\
               -p safe_z:=50.0
"""

import yaml
import numpy as np
import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .robot_interface import DobotInterface


class CalibVerify(Node):
    def __init__(self):
        super().__init__('calib_verify')
        self.declare_parameter('calib_path', './calibration.yaml')
        self.declare_parameter('pose_topic', 'dobot_pose_raw')
        self.declare_parameter('safe_z', 50.0)

        calib_path = self.get_parameter('calib_path').value
        pose_topic = self.get_parameter('pose_topic').value
        self.safe_z = self.get_parameter('safe_z').value

        with open(calib_path) as f:
            calib = yaml.safe_load(f)
        self.M = np.array(calib['matrix'], dtype=np.float64)
        self.pick_z = float(calib.get('pick_z', 0.0))
        self.image_topic = calib.get('image_topic') or '/camera/camera1/color/image_raw'

        self.bridge = CvBridge()
        self.latest = None
        self.click  = None
        self.busy   = False

        self.robot = DobotInterface(self, pose_topic=pose_topic)
        self.create_subscription(Image, self.image_topic, self._img_cb, 10)
        cv2.namedWindow('calib_verify')
        cv2.setMouseCallback('calib_verify', self._on_mouse)
        self.create_timer(1.0 / 30.0, self._loop)
        self.get_logger().info(
            f"클릭하면 로봇이 그 좌표로 이동합니다. safe_z={self.safe_z}mm\n"
            f"  [q] 종료")

    def _img_cb(self, msg):
        self.latest = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def pixel_to_robot(self, u, v):
        xy = self.M @ np.array([u, v, 1.0])
        return float(xy[0]), float(xy[1])

    def _on_mouse(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and not self.busy:
            rx, ry = self.pixel_to_robot(x, y)
            self.click = (x, y, rx, ry)
            self.get_logger().info(
                f"클릭 px=({x},{y}) -> robot=({rx:.1f},{ry:.1f})mm  이동 중...")
            threading.Thread(target=self._move, args=(rx, ry), daemon=True).start()

    def _move(self, rx, ry):
        self.busy = True
        try:
            self.robot.move_to(rx, ry, self.safe_z)
        finally:
            self.busy = False

    def _loop(self):
        if self.latest is None:
            return
        vis = self.latest.copy()

        if self.click:
            x, y, rx, ry = self.click
            cv2.drawMarker(vis, (x, y), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(vis, f"robot=({rx:.1f},{ry:.1f})mm",
                        (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        status = "이동 중..." if self.busy else "클릭 → 로봇 이동"
        cv2.putText(vis, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 100, 255) if self.busy else (200, 200, 200), 2)

        cv2.imshow('calib_verify', vis)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CalibVerify()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
