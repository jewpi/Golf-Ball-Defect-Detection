"""
robot_interface.py
==================
magician_ros2_control_system_ws 패키지에 맞춘 구현:
  - 현재좌표: dobot_pose_raw (Float64MultiArray, [x/1000, y/1000, z/1000, r])
  - PTP 이동:  PTP_action (PointToPoint action, target_pose=[x, y, z, r] mm 단위)
  - 흡착컵:    dobot_suction_cup_service (SuctionCupControl srv)
"""

import threading
import time
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl


class DobotInterface:
    def __init__(self, node: Node,
                 pose_topic: str = 'dobot_pose_raw',
                 ptp_action: str = 'PTP_action',
                 suction_service: str = 'dobot_suction_cup_service'):
        self.node = node
        self._lock = threading.Lock()
        self._pose = None   # (x, y, z, r) mm 단위

        # 현재좌표 구독 (dobot_pose_raw: [x/1000, y/1000, z/1000, r])
        self._pose_sub = node.create_subscription(
            Float64MultiArray, pose_topic, self._pose_cb, 10)

        # PTP 이동 액션 클라이언트
        self._ptp_cli = ActionClient(node, PointToPoint, ptp_action)

        # 흡착컵 서비스 클라이언트
        self._suction_cli = node.create_client(SuctionCupControl, suction_service)

        node.get_logger().info('DobotInterface 준비됨. 현재좌표 토픽 대기 중...')

    # ---------------- 현재 좌표 ----------------
    def _pose_cb(self, msg: Float64MultiArray):
        # dobot_pose_raw 는 [x/1000, y/1000, z/1000, r] 형태로 퍼블리시됨
        # mm 로 변환해 저장
        x = msg.data[0] * 1000.0
        y = msg.data[1] * 1000.0
        z = msg.data[2] * 1000.0
        r = msg.data[3]
        with self._lock:
            self._pose = (x, y, z, r)

    def get_pose(self, timeout=2.0):
        """현재 (x, y, z, r) mm 반환. 아직 못 받았으면 잠깐 기다림."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                if self._pose is not None:
                    return self._pose
            time.sleep(0.05)
        return None

    # ---------------- 이동 ----------------
    def move_to(self, x, y, z, r=0.0, wait=True):
        """(x, y, z, r) mm/deg 로 PTP 이동. wait=True 면 완료까지 블로킹."""
        if not self._ptp_cli.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('[move_to] PTP action 서버에 연결할 수 없음')
            return False

        goal = PointToPoint.Goal()
        goal.motion_type = PointToPoint.Goal.MOTION_TYPE_MOVJ_XYZ  # 1
        goal.target_pose = [float(x), float(y), float(z), float(r)]

        if not wait:
            self._ptp_cli.send_goal_async(goal)
            return True

        done_event = threading.Event()
        result_holder = [None]

        def _result_cb(future):
            result_holder[0] = future.result().result
            done_event.set()

        def _goal_response_cb(future):
            gh = future.result()
            if not gh.accepted:
                self.node.get_logger().warn('[move_to] 목표가 거부됨')
                done_event.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        send_future = self._ptp_cli.send_goal_async(goal)
        send_future.add_done_callback(_goal_response_cb)

        reached = done_event.wait(timeout=30.0)
        if not reached:
            self.node.get_logger().warn('[move_to] 30초 내에 완료되지 않음')
        return reached

    # ---------------- 흡착 ----------------
    def set_suction(self, on: bool):
        """흡착컵 on/off."""
        if not self._suction_cli.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('[set_suction] 흡착 서비스에 연결할 수 없음')
            return False

        req = SuctionCupControl.Request()
        req.enable_suction = on

        done_event = threading.Event()
        result_holder = [None]

        def _cb(future):
            result_holder[0] = future.result()
            done_event.set()

        self._suction_cli.call_async(req).add_done_callback(_cb)
        done_event.wait(timeout=5.0)

        resp = result_holder[0]
        if resp is None:
            self.node.get_logger().warn('[set_suction] 서비스 응답 없음')
            return False
        return resp.success
