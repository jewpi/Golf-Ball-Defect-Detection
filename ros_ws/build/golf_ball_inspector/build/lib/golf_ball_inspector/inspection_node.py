#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
inspection_node — YOLO 검사 + 이중 카메라 표시 노드 (노트북 실행)

구독: /inspection/start_cmd (std_msgs/Empty)
     dobot_controller가 Place 완료 후 검사 시작을 알릴 때 수신

발행: /inspection/cycle_complete (std_msgs/String)
     "INIT" : 카메라·YOLO 초기화 완료 (Dobot 첫 Pick 허용 신호)
     "GOOD" : 17초 사이클 종료 — 불량 없음
     "BAD"  : 17초 사이클 종료 — 불량 감지됨

발행: /conveyor/cmd (std_msgs/String)
     "START" / "BAD" / "STOP" → conveyor_node(RPi)로 전달
"""

import base64
import os
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Empty, String
from ultralytics import YOLO

# ─── 설정값 ───────────────────────────────────────────────────────────────────

CAMERA_SERIAL   = '244222077012'   # 검사용 카메라
OVERVIEW_SERIAL = '243522072229'   # 전체뷰 카메라 (overview)

BALL_CONF_THRESHOLD  = 0.25
CRACK_CONF_THRESHOLD = 0.20
IOU_THRESHOLD        = 0.45
IMGSZ                = 640

API_URL = 'http://192.168.110.113:8000/api/broken_ball'
CYCLE_TIMEOUT = 17.0   # start 후 자동 사이클 종료까지 대기 시간 (초)


class InspectionNode(Node):

    def __init__(self):
        super().__init__('inspection_node')

        self.declare_parameter('model_path', '/home/ssafy/pene_pjt/best.pt')

        self._state      = 'IDLE'
        self._state_lock = threading.Lock()
        self._last_result = ''
        self._cycle_bad  = False
        self._run_gen    = 0

        # transient_local: 마지막 메시지를 보존 → dobot이 늦게 연결돼도 수신 가능
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._cycle_pub   = self.create_publisher(String, '/inspection/cycle_complete', latched_qos)
        self._conveyor_pub = self.create_publisher(String, '/conveyor/cmd', 10)
        self.create_subscription(Empty, '/inspection/start_cmd', self._on_start_cmd, 10)

    # ── 검사 시작 콜백 ────────────────────────────────────────────────────────

    def _on_start_cmd(self, _msg: Empty):
        with self._state_lock:
            if self._state != 'IDLE':
                self.get_logger().warn('start_cmd 수신 — IDLE 아님, 무시')
                return
            self._state     = 'RUNNING'
            self._cycle_bad = False
            self._run_gen  += 1
            gen = self._run_gen

        self._conveyor_pub.publish(String(data='START'))
        self.get_logger().info(f'[검사] 시작 (gen={gen}) — {CYCLE_TIMEOUT}초 후 자동 종료')

        def auto_reset(gen=gen):
            time.sleep(CYCLE_TIMEOUT)
            with self._state_lock:
                if self._run_gen != gen:
                    return                        # 더 최신 사이클이 있으면 무시
                bad       = self._cycle_bad
                self._state = 'IDLE'

            result = 'BAD' if bad else 'GOOD'
            self._conveyor_pub.publish(String(data='STOP'))
            self._cycle_pub.publish(String(data=result))
            self.get_logger().info(f'[검사] 사이클 완료 → {result}')

        threading.Thread(target=auto_reset, daemon=True).start()

    # ── BAD 트리거 ────────────────────────────────────────────────────────────

    def _trigger_bad(self, frame=None):
        with self._state_lock:
            if self._state != 'RUNNING':
                return
            self._state     = 'SORTING'
            self._cycle_bad = True
        self._last_result = 'BAD'

        self._conveyor_pub.publish(String(data='BAD'))
        self.get_logger().info('[검사] BAD 감지 — 컨베이어 분류 명령 전송')

        if frame is not None:
            threading.Thread(
                target=self._send_api_report, args=(frame,), daemon=True
            ).start()

    def _send_api_report(self, frame):
        try:
            img_320 = cv2.resize(frame, (320, 320))
            _, buf  = cv2.imencode('.jpg', img_320)
            b64     = base64.b64encode(buf).decode('utf-8')
            payload = {
                'location': {'room': 1},
                'image': f'data:image/jpeg;base64,{b64}',
            }
            resp = requests.post(API_URL, json=payload, timeout=5)
            self.get_logger().info(f'[API] 파손 이미지 전송 완료 (status={resp.status_code})')
        except Exception as e:
            self.get_logger().error(f'[API] 전송 실패: {e}')

    # ── 카메라 루프 (메인 스레드에서 실행) ───────────────────────────────────

    def run_camera_loop(self):
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f'[YOLO] 모델 로드: {model_path}')
        model = YOLO(model_path)
        self.get_logger().info(f'[YOLO] 로드 완료 | 클래스: {model.names}')

        # 검사 카메라 파이프라인
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(CAMERA_SERIAL)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # 전체뷰 카메라 파이프라인
        pipeline2 = rs.pipeline()
        cfg2 = rs.config()
        cfg2.enable_device(OVERVIEW_SERIAL)
        cfg2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        pipeline.start(cfg)
        pipeline2.start(cfg2)
        self.get_logger().info('[카메라] 스트림 시작 완료')

        # 초기화 완료 → Dobot에게 첫 Pick 허용 신호
        self._cycle_pub.publish(String(data='INIT'))
        self.get_logger().info('[검사] INIT 신호 발행 → Dobot 준비')

        WIN_NAME = 'Golf Ball Inspection'
        cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_NAME, 640, 960)

        overview_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        pipeline.wait_for_frames()
        pipeline2.wait_for_frames()

        try:
            while rclpy.ok():
                # 검사 카메라: 버퍼 드레인 → 최신 프레임만 사용
                frames = None
                while True:
                    f = pipeline.poll_for_frames()
                    if f and f.get_color_frame():
                        frames = f
                    else:
                        break
                if frames is None:
                    frames = pipeline.wait_for_frames()

                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                frame   = np.asanyarray(color_frame.get_data())
                display = frame.copy()

                # 전체뷰 카메라: 폴링 (없으면 이전 프레임 유지)
                ov = pipeline2.poll_for_frames()
                if ov and ov.get_color_frame():
                    overview_frame = np.asanyarray(ov.get_color_frame().get_data()).copy()

                with self._state_lock:
                    state = self._state

                # YOLO 추론 (RUNNING 상태에서만)
                if state == 'RUNNING':
                    results = model(
                        frame, verbose=False,
                        imgsz=IMGSZ, conf=CRACK_CONF_THRESHOLD, iou=IOU_THRESHOLD
                    )[0]

                    has_crack = has_ball = False
                    for box in results.boxes:
                        cls_id = int(box.cls[0])
                        conf   = float(box.conf[0])
                        x1, y1, x2, y2 = map(int, box.xyxy[0])

                        is_crack = (cls_id == 0)
                        if conf < (CRACK_CONF_THRESHOLD if is_crack else BALL_CONF_THRESHOLD):
                            continue

                        box_color = (0, 0, 255) if is_crack else (0, 255, 0)
                        cv2.rectangle(display, (x1, y1), (x2, y2), box_color, 2)
                        cv2.putText(
                            display, f'{model.names[cls_id]} {conf:.2f}',
                            (x1, max(y1 - 8, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2,
                        )
                        if is_crack:
                            has_crack = True
                        else:
                            has_ball = True

                    # 공이 프레임 안에 있을 때만 crack 판정 (가장자리 오감지 방지)
                    if has_crack and has_ball:
                        threading.Thread(
                            target=self._trigger_bad, args=(frame.copy(),), daemon=True
                        ).start()

                # SORTING 상태: 판별 결과 크게 표시
                if state == 'SORTING' and self._last_result:
                    text      = self._last_result
                    txt_color = (0, 220, 0) if text == 'GOOD' else (0, 0, 255)
                    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 3.0, 5)
                    tx = (640 - tw) // 2
                    ty = (480 + th) // 2
                    cv2.putText(display, text, (tx, ty),
                                cv2.FONT_HERSHEY_DUPLEX, 3.0, (30, 30, 30), 8)
                    cv2.putText(display, text, (tx, ty),
                                cv2.FONT_HERSHEY_DUPLEX, 3.0, txt_color, 5)

                # 두 화면 합치기 (흰색 구분선)
                separator = np.full((10, 640, 3), 255, dtype=np.uint8)
                combined  = np.vstack([display, separator, overview_frame])
                cv2.imshow(WIN_NAME, combined)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            pipeline.stop()
            pipeline2.stop()
            cv2.destroyAllWindows()
            self.get_logger().info('[카메라] 종료')


def main(args=None):
    rclpy.init(args=args)
    node = InspectionNode()

    # ROS spin을 백그라운드 스레드에서 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_camera_loop()   # OpenCV imshow는 메인 스레드에서
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
