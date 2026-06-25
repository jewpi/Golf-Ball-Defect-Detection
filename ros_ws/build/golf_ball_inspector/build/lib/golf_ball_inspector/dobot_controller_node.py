#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dobot_controller_node — Dobot 로봇 암 Pick & Place 제어 노드

구독: /inspection/cycle_complete (std_msgs/String)
     "INIT"  : inspection_node 초기화 완료 → 첫 번째 Pick 시작 가능
     "GOOD"  : 현재 사이클 양품 판정 완료
     "BAD"   : 현재 사이클 불량 판정 완료 → 불량 공 제거 루틴 실행

발행: /inspection/start_cmd (std_msgs/Empty)
     Place 완료 후 검사 시작을 inspection_node에 알림
"""

import threading
import time

import pydobot
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Empty, String

# ─── 좌표 설정 ────────────────────────────────────────────────────────────────

LIFT_Z  = 50.0    # 이동 시 들어올릴 Z 높이 (mm)
PRESS_Z = -35.0   # Pick 시 공을 누르는 Z 깊이 (mm)

PICK_POSITIONS = [
    ( 81.73, 205.31, -23.38,  68.29),   # Pick 1
    (111.75, 170.47, -23.58,  56.75),   # Pick 2
]

PLACE_X, PLACE_Y, PLACE_Z, PLACE_R = 149.63, 79.85, 28.76, 28.09

BAD_PICK_X, BAD_PICK_Y, BAD_PICK_Z, BAD_PICK_R = 180.33, -130.25, -8.76, -35.84
BAD_DROP_X, BAD_DROP_Y, BAD_DROP_Z, BAD_DROP_R =  15.06, -165.80, 97.09, -84.81
BAD_PRESS_Z = -20.0   # 불량 공 집을 때 press 깊이 (mm)

DOBOT_PORT = '/dev/ttyACM0'


class DobotControllerNode(Node):

    def __init__(self):
        super().__init__('dobot_controller')

        self._cycle_result: str | None = None
        self._cycle_event = threading.Event()

        # inspection_node의 latched QoS와 일치 — 늦게 연결해도 INIT 메시지 수신 가능
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._start_pub = self.create_publisher(Empty, '/inspection/start_cmd', 10)
        self.create_subscription(
            String, '/inspection/cycle_complete', self._on_cycle_complete, latched_qos
        )

        self.get_logger().info(f'Dobot 연결 중 ({DOBOT_PORT})...')
        self._device = pydobot.Dobot(port=DOBOT_PORT, verbose=False)
        hx, hy, hz, hr, *_ = self._device.pose()
        self._home = (hx, hy, hz, hr)
        self.get_logger().info(f'Dobot 연결 완료 | 홈: ({hx:.1f}, {hy:.1f}, {hz:.1f})')

        threading.Thread(target=self._run_sequence, daemon=True).start()

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _on_cycle_complete(self, msg: String):
        self._cycle_result = msg.data.upper()
        self._cycle_event.set()

    # ── 동기화 헬퍼 ───────────────────────────────────────────────────────────

    def _wait_cycle(self, timeout: float = 60.0) -> str:
        """inspection_node의 사이클 완료 신호를 블로킹 대기."""
        if not self._cycle_event.wait(timeout):
            raise TimeoutError('검사 완료 신호 타임아웃')
        self._cycle_event.clear()
        result = self._cycle_result
        self._cycle_result = None
        return result  # type: ignore[return-value]

    def _send_start(self):
        self._start_pub.publish(Empty())
        self.get_logger().info('[Dobot] 검사 시작 신호 발행')

    # ── 불량 공 처리 ──────────────────────────────────────────────────────────

    def _handle_bad_ball(self):
        self.get_logger().info('[BAD] 불량 공 처리 루틴 시작')
        d = self._device
        hx, hy, hz, hr = self._home

        d.move_to(BAD_PICK_X, BAD_PICK_Y, LIFT_Z,     BAD_PICK_R, wait=True)
        d.move_to(BAD_PICK_X, BAD_PICK_Y, BAD_PICK_Z, BAD_PICK_R, wait=True)
        d.move_to(BAD_PICK_X, BAD_PICK_Y, BAD_PRESS_Z, BAD_PICK_R, wait=True)
        time.sleep(2)
        d.suck(True)
        time.sleep(2)
        d.move_to(BAD_PICK_X, BAD_PICK_Y, BAD_DROP_Z, BAD_PICK_R, wait=True)
        d.move_to(BAD_DROP_X, BAD_DROP_Y, BAD_DROP_Z, BAD_DROP_R, wait=True)
        d.suck(False)
        # 특이점 회피: 왔던 경로로 역추적 후 홈
        d.move_to(BAD_PICK_X, BAD_PICK_Y, BAD_DROP_Z, BAD_PICK_R, wait=True)
        d.move_to(hx, hy, hz, hr, wait=True)
        self.get_logger().info('[BAD] 처리 완료, 홈 복귀')

    # ── 메인 시퀀스 (별도 스레드) ─────────────────────────────────────────────

    def _run_sequence(self):
        d = self._device
        hx, hy, hz, hr = self._home

        try:
            self.get_logger().info('inspection_node 초기화 대기 중...')
            self._wait_cycle(timeout=120.0)   # "INIT" 메시지 대기
            self.get_logger().info('시스템 준비 완료. Pick & Place 시작')

            total = len(PICK_POSITIONS)
            for i, (px, py, pz, pr) in enumerate(PICK_POSITIONS, 1):
                self.get_logger().info(f'[{i}/{total}] Pick 시작')

                # ── Pick ──
                d.move_to(px, py, pz,     pr, wait=True)
                d.move_to(px, py, PRESS_Z, pr, wait=True)
                time.sleep(2)
                d.suck(True)
                time.sleep(2)
                d.move_to(px, py, LIFT_Z,  pr, wait=True)

                # ── Place ──
                d.move_to(PLACE_X, PLACE_Y, LIFT_Z,  PLACE_R, wait=True)
                d.move_to(PLACE_X, PLACE_Y, PLACE_Z, PLACE_R, wait=True)
                d.suck(False)
                d.move_to(PLACE_X, PLACE_Y, LIFT_Z,  PLACE_R, wait=True)
                d.move_to(PLACE_X, PLACE_Y, hz,      PLACE_R, wait=True)
                d.move_to(hx, hy, hz, hr, wait=True)
                self.get_logger().info(f'[{i}/{total}] 홈 복귀 완료')

                # ── 검사 시작 ──
                self._send_start()

                # ── 결과 대기 ──
                result = self._wait_cycle(timeout=60.0)
                self.get_logger().info(f'[{i}/{total}] 결과: {result}')
                if result == 'BAD':
                    self._handle_bad_ball()

            self.get_logger().info('모든 Pick & Place 완료')

        except Exception as e:
            self.get_logger().error(f'시퀀스 오류: {e}')
        finally:
            d.close()


def main(args=None):
    rclpy.init(args=args)
    node = DobotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
