"""
calib_collect.py
================
캘리브레이션 데이터 수집 노드.

사용 흐름:
  1) 카메라 영상에서 원하는 지점을 마우스로 클릭 → 초록 십자 표시
  2) 로봇 엔드이펙터를 그 실제 위치에 직접 갖다 댄다 (수동 jog)
  3) [s] 키 → 현재 로봇 XY + 클릭한 픽셀 UV 저장
  4) 6~12 지점 반복 (작업판 전체에 골고루)
  5) [q] 키 → pairs.yaml 저장 후 종료

실행:
  ros2 run dobot_realsense_calib calib_collect \\
    --ros-args -p image_topic:=/camera/camera1/color/image_raw \\
               -p out_path:=./pairs.yaml
"""

import os
import yaml
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .robot_interface import DobotInterface

HELP = "[s] save  [u] undo  [q] quit  |  click: set pixel"


class CalibCollect(Node):
    def __init__(self):
        super().__init__('calib_collect')
        self.declare_parameter('image_topic', '/camera/camera1/color/image_raw')
        self.declare_parameter('out_path', './pairs.yaml')
        self.declare_parameter('pose_topic', 'dobot_pose_raw')

        self.image_topic = self.get_parameter('image_topic').value
        self.out_path    = self.get_parameter('out_path').value
        pose_topic       = self.get_parameter('pose_topic').value

        self.bridge = CvBridge()
        self.latest = None
        self.pairs  = []
        self._clicked_uv = None   # 마우스로 찍은 픽셀 좌표

        self.robot = DobotInterface(self, pose_topic=pose_topic)
        self.create_subscription(Image, self.image_topic, self._img_cb, 10)

        cv2.namedWindow('calib_collect', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('calib_collect', self._mouse_cb)

        self.get_logger().info(
            f"수집 시작. 토픽='{self.image_topic}'\n"
            f"  1) 영상에서 원하는 지점 클릭\n"
            f"  2) 로봇 엔드이펙터를 그 위치로 이동\n"
            f"  3) [s] 저장   [u] 취소   [q] 종료")
        self.create_timer(1.0 / 30.0, self._loop)

    def _img_cb(self, msg):
        self.latest = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _mouse_cb(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._clicked_uv = (float(x), float(y))
            self.get_logger().info(f"픽셀 선택: ({x}, {y})")

    def _loop(self):
        if self.latest is None:
            return

        vis = self.latest.copy()

        # 클릭 위치 표시
        if self._clicked_uv is not None:
            cx, cy = int(self._clicked_uv[0]), int(self._clicked_uv[1])
            cv2.drawMarker(vis, (cx, cy), (255, 100, 0),
                           cv2.MARKER_CROSS, 24, 2)

        # 이미 저장된 점 표시
        for p in self.pairs:
            cv2.circle(vis, (int(p['pixel'][0]), int(p['pixel'][1])),
                       5, (255, 0, 255), -1)

        # 상태 텍스트
        if self._clicked_uv is not None:
            status = f"px=({self._clicked_uv[0]:.0f},{self._clicked_uv[1]:.0f})  → 로봇 이동 후 [s]"
            color  = (0, 255, 0)
        else:
            status = "클릭으로 픽셀 선택"
            color  = (0, 180, 255)

        cv2.putText(vis, f"pairs:{len(self.pairs)}  {status}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        cv2.putText(vis, HELP, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        cv2.imshow('calib_collect', vis)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            self._capture()
        elif key == ord('u'):
            if self.pairs:
                self.pairs.pop()
                self.get_logger().info(f"마지막 취소. 남은 {len(self.pairs)}개")
        elif key == ord('q'):
            self._save()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def _capture(self):
        if self._clicked_uv is None:
            self.get_logger().warn("픽셀을 먼저 클릭하세요.")
            return
        pose = self.robot.get_pose()
        if pose is None:
            self.get_logger().warn("로봇 좌표를 못 받음. pose_topic 확인 필요.")
            return
        x, y, z, r = pose
        u, v = self._clicked_uv
        self.pairs.append({'robot': [float(x), float(y)],
                           'pixel': [float(u), float(v)],
                           'z':     float(z)})
        self._clicked_uv = None   # 다음 지점을 위해 초기화
        self.get_logger().info(
            f"저장 #{len(self.pairs)}: robot=({x:.1f},{y:.1f}) px=({u:.0f},{v:.0f})")

    def _save(self):
        zs = [p['z'] for p in self.pairs]
        data = {
            'image_topic': self.image_topic,
            'pick_z': float(np.mean(zs)) if zs else 0.0,
            'pairs': [{'robot': p['robot'], 'pixel': p['pixel']} for p in self.pairs],
        }
        os.makedirs(os.path.dirname(os.path.abspath(self.out_path)), exist_ok=True)
        with open(self.out_path, 'w') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        self.get_logger().info(
            f"총 {len(self.pairs)}쌍을 '{self.out_path}'에 저장했습니다.")


def main():
    rclpy.init()
    node = CalibCollect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._save()
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
