#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
conveyor_node — 컨베이어(스텝모터) + 서보모터 제어 노드 (라즈베리파이 실행)

구독: /conveyor/cmd (std_msgs/String)
     "START" : 2초 후 컨베이어 구동 + 15초 타임아웃
     "BAD"   : 서보 95°→175°, SORT_HOLD_TIME 유지 후 95° 복귀 → 컨베이어 정지
     "STOP"  : 즉시 전체 정지 → IDLE

GPIO 핀 배선:
  DIR    → GPIO 17   (스텝모터 드라이버)
  STEP   → GPIO 27
  ENABLE → GPIO 22
  SERVO  → GPIO 18
"""

import threading
import time

import gpiod
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ─── GPIO 설정값 ──────────────────────────────────────────────────────────────

DIR_PIN    = 17
STEP_PIN   = 27
ENABLE_PIN = 22
SERVO_PIN  = 18

STEP_DELAY   = 0.0004
CONVEYOR_DIR = 0        # 0=CW(정방향), 1=CCW

ANGLE_STANDBY  = 95
ANGLE_BAD      = 175
SORT_HOLD_TIME = 10.0   # 서보 분류 위치 유지 시간 (초)
SERVO_PULSES   = 20     # 소프트웨어 PWM 펄스 반복 횟수

START_DELAY    = 2.0    # START 수신 후 컨베이어 구동 대기 (초)
START_TIMEOUT  = 15.0   # START 후 GOOD/BAD 미수신 시 자동 초기화 (초)


class ConveyorNode(Node):

    def __init__(self):
        super().__init__('conveyor_node')

        self._chip         = gpiod.Chip('gpiochip0')
        self._dir_line     = self._chip.get_line(DIR_PIN)
        self._step_line    = self._chip.get_line(STEP_PIN)
        self._enable_line  = self._chip.get_line(ENABLE_PIN)
        self._servo_line   = self._chip.get_line(SERVO_PIN)

        self._dir_line.request(   consumer='dir',    type=gpiod.LINE_REQ_DIR_OUT)
        self._step_line.request(  consumer='step',   type=gpiod.LINE_REQ_DIR_OUT)
        self._enable_line.request(consumer='enable', type=gpiod.LINE_REQ_DIR_OUT)
        self._servo_line.request( consumer='servo',  type=gpiod.LINE_REQ_DIR_OUT)

        self._enable_line.set_value(1)         # 초기: 드라이버 비활성화
        self._dir_line.set_value(CONVEYOR_DIR)
        self._step_line.set_value(0)
        self._servo_move(ANGLE_STANDBY)

        self._motor_running = False
        self._motor_lock    = threading.Lock()
        self._servo_lock    = threading.Lock()
        self._state         = 'IDLE'
        self._run_gen       = 0

        self.create_subscription(String, '/conveyor/cmd', self._on_cmd, 10)
        self.get_logger().info('[컨베이어] 초기화 완료 — 명령 대기 중')

    # ── GPIO 제어 ─────────────────────────────────────────────────────────────

    def _servo_move(self, angle: float):
        angle = max(0.0, min(270.0, angle))
        pulse_width = (angle / 270.0) * (0.0025 - 0.0005) + 0.0005
        for _ in range(SERVO_PULSES):
            self._servo_line.set_value(1)
            time.sleep(pulse_width)
            self._servo_line.set_value(0)
            time.sleep(0.02 - pulse_width)

    def _motor_pulse_loop(self):
        self._enable_line.set_value(0)          # 드라이버 활성화
        while self._motor_running:
            self._step_line.set_value(1)
            time.sleep(STEP_DELAY)
            self._step_line.set_value(0)
            time.sleep(STEP_DELAY)
        self._step_line.set_value(0)
        self._enable_line.set_value(1)          # 드라이버 비활성화

    def _start_motor(self):
        with self._motor_lock:
            if self._motor_running:
                return
            self._motor_running = True
            threading.Thread(target=self._motor_pulse_loop, daemon=True).start()
        self.get_logger().info('[모터] 구동 시작')

    def _stop_motor(self):
        with self._motor_lock:
            self._motor_running = False
        self.get_logger().info('[모터] 정지')

    # ── 명령 콜백 ─────────────────────────────────────────────────────────────

    def _on_cmd(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'[컨베이어] 명령 수신: {cmd}')

        if cmd == 'START':
            if self._state != 'IDLE':
                self.get_logger().warn(f'START 무시 (현재: {self._state})')
                return
            self._state   = 'RUNNING'
            self._run_gen += 1
            gen = self._run_gen

            def delayed_start(gen=gen):
                time.sleep(START_DELAY)
                self._start_motor()

                def timeout_reset(gen=gen):
                    time.sleep(START_TIMEOUT)
                    if self._run_gen != gen:
                        return
                    self._stop_motor()
                    with self._servo_lock:
                        self._servo_move(ANGLE_STANDBY)
                    self._state = 'IDLE'
                    self.get_logger().info(f'[컨베이어] {START_TIMEOUT}s 타임아웃 → IDLE')

                threading.Thread(target=timeout_reset, daemon=True).start()

            threading.Thread(target=delayed_start, daemon=True).start()

        elif cmd == 'BAD':
            if self._state != 'RUNNING':
                self.get_logger().warn(f'BAD 무시 (현재: {self._state})')
                return
            self._state = 'SORTING'

            def do_sort():
                with self._servo_lock:
                    self.get_logger().info(f'[서보] {ANGLE_BAD}° 이동')
                    self._servo_move(ANGLE_BAD)
                    time.sleep(SORT_HOLD_TIME)
                    self._servo_move(ANGLE_STANDBY)
                    self.get_logger().info(f'[서보] {ANGLE_STANDBY}° 복귀')
                self._stop_motor()
                self._state = 'IDLE'
                self.get_logger().info('[컨베이어] 분류 완료 → IDLE')

            threading.Thread(target=do_sort, daemon=True).start()

        elif cmd == 'STOP':
            self._stop_motor()
            with self._servo_lock:
                self._servo_move(ANGLE_STANDBY)
            self._state = 'IDLE'
            self.get_logger().info('[컨베이어] 긴급 정지 → IDLE')

        else:
            self.get_logger().warn(f'알 수 없는 명령: {cmd}')

    # ── 정리 ──────────────────────────────────────────────────────────────────

    def cleanup(self):
        self._stop_motor()
        time.sleep(0.05)
        self._dir_line.release()
        self._step_line.release()
        self._enable_line.release()
        self._servo_line.release()
        self._chip.close()
        self.get_logger().info('[컨베이어] GPIO 해제 완료')


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
