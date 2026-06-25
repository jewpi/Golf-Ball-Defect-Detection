"""
ROS2 RealSense camera1 시연 영상 녹화 스크립트
================================================
카메라:  serial_no=_243522072229  (camera_name=camera1)
토픽:    /camera1/color/image_raw

사용법:
  python ros2_record_camera1.py

단축키 (미리보기 창):
  R  →  녹화 시작 / 중지
  Q  →  종료

출력 파일: demo_YYYYMMDD_HHMMSS.mp4
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
from datetime import datetime
import threading


# ──────────────────────────────────────────
# 설정값
# ──────────────────────────────────────────
CAMERA_TOPIC = "/camera1/color/image_raw"
OUTPUT_FPS   = 30          # 저장 FPS
OUTPUT_SIZE  = None        # None → 카메라 해상도 그대로 사용
SHOW_OVERLAY = True        # 미리보기에 상태 오버레이 표시
# ──────────────────────────────────────────


def make_filename():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"demo_{ts}.mp4"


def draw_overlay(frame, recording, elapsed_sec, live_fps):
    h, w = frame.shape[:2]

    if recording:
        cv2.circle(frame, (30, 30), 12, (0, 0, 220), -1)
        mins, secs = divmod(int(elapsed_sec), 60)
        cv2.putText(frame, f"REC  {mins:02d}:{secs:02d}",
                    (52, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 220), 2)
    else:
        cv2.putText(frame, "STANDBY  [R] to record",
                    (15, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2)

    now_str = datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
    (tw, _), _ = cv2.getTextSize(now_str, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    cv2.putText(frame, now_str, (w - tw - 15, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)

    fps_str = f"{live_fps:.1f} fps"
    (fw, _), _ = cv2.getTextSize(fps_str, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    cv2.putText(frame, fps_str, (w - fw - 15, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 220, 180), 1)

    # 카메라 이름 표시
    cv2.putText(frame, "camera1  |  serial: 243522072229",
                (15, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    return frame


class Camera1Recorder(Node):
    def __init__(self):
        super().__init__("camera1_recorder")
        self.bridge     = CvBridge()
        self.latest_frame = None
        self.frame_lock  = threading.Lock()

        # FPS 측정
        self.fps_counter = 0
        self.fps_timer   = datetime.now()
        self.live_fps    = 0.0

        # 녹화 상태
        self.recording  = False
        self.writer     = None
        self.rec_start  = None
        self.out_path   = None
        self.out_size   = OUTPUT_SIZE   # (w, h) or None

        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10
        )
        self.get_logger().info(f"Subscribed to  {CAMERA_TOPIC}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # 출력 해상도 초기화 (첫 프레임 기준)
        if self.out_size is None:
            h, w = frame.shape[:2]
            self.out_size = (w, h)

        # FPS 계산
        self.fps_counter += 1
        elapsed = (datetime.now() - self.fps_timer).total_seconds()
        if elapsed >= 1.0:
            self.live_fps    = self.fps_counter / elapsed
            self.fps_counter = 0
            self.fps_timer   = datetime.now()

        with self.frame_lock:
            self.latest_frame = frame.copy()

        # 녹화 중이면 원본 그대로 저장
        if self.recording and self.writer is not None:
            out_frame = cv2.resize(frame, self.out_size) if frame.shape[:2][::-1] != self.out_size else frame
            self.writer.write(out_frame)

    def start_recording(self):
        if self.recording:
            return
        if self.out_size is None:
            print("[경고] 아직 프레임을 수신하지 못했습니다. 잠시 후 다시 시도하세요.")
            return
        self.out_path  = make_filename()
        fourcc         = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer    = cv2.VideoWriter(self.out_path, fourcc, OUTPUT_FPS, self.out_size)
        self.rec_start = datetime.now()
        self.recording = True
        print(f"[녹화 시작]  →  {self.out_path}")

    def stop_recording(self):
        if not self.recording:
            return
        self.recording = False
        self.writer.release()
        self.writer = None
        print(f"[녹화 완료]  →  {self.out_path}")
        self.out_path = None

    def run_preview(self):
        """메인 스레드에서 OpenCV 미리보기 루프 실행"""
        print("\n미리보기 창에서  R = 녹화 시작/중지,  Q = 종료\n")
        while rclpy.ok():
            with self.frame_lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None

            if frame is None:
                key = cv2.waitKey(30) & 0xFF
            else:
                display = frame.copy()
                if SHOW_OVERLAY:
                    rec_elapsed = (datetime.now() - self.rec_start).total_seconds() if self.recording else 0
                    display = draw_overlay(display, self.recording, rec_elapsed, self.live_fps)

                cv2.imshow("camera1 Demo Recorder", display)
                key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == ord("Q"):
                break
            elif key == ord("r") or key == ord("R"):
                if not self.recording:
                    self.start_recording()
                else:
                    self.stop_recording()

        if self.recording:
            self.stop_recording()
        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = Camera1Recorder()

    # ROS2 spin을 별도 스레드에서 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_preview()   # 메인 스레드: OpenCV 창
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("종료되었습니다.")


if __name__ == "__main__":
    main()
