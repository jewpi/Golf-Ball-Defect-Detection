# -*- coding: utf-8 -*-
"""
[노트북 실행 코드] pc_client.py
역할: AI(YOLOv8)로 ROI 내 골프공/결함 즉시 판별 → 라즈베리파이 제어 신호 전송

의존성 설치:
    pip install pyrealsense2 opencv-python numpy ultralytics

사용법:
    python pc_client.py --host 192.168.x.x --port 9999

터미널 명령어:
    start  : 컨베이어 구동 → ROI에 골프공 감지 시 즉시 good/bad 판별
    stop   : 긴급 정지
    q      : 프로그램 종료
"""

import cv2
import numpy as np
import socket
import time
import argparse
import threading
import base64
import requests
import pyrealsense2 as rs
from ultralytics import YOLO
import os


# ═══════════════════════════════════════════
# 설정값
# ═══════════════════════════════════════════

RASPBERRY_PI_HOST = "192.168.110.143"
RASPBERRY_PI_PORT = 9999

READY_FLAG = '/tmp/dobot_pc_ready'
BAD_FLAG   = '/tmp/dobot_bad_ball'

def _set_ready():
    """도봇에게 start 명령 수신 가능 상태임을 알리는 플래그 파일 생성."""
    open(READY_FLAG, 'w').close()

MODEL_PATH           = os.path.join(os.path.dirname(__file__), "best.pt")
BALL_CONF_THRESHOLD  = 0.25  # golf ball 감지 신뢰도
CRACK_CONF_THRESHOLD = 0.20  # crack 감지 신뢰도 (낮게 → 민감하게)
IOU_THRESHOLD        = 0.45  # NMS IoU 임계값
IMGSZ                = 640   # 추론 해상도 (학습과 동일)


# ═══════════════════════════════════════════
# 전역 상태
# ═══════════════════════════════════════════

# IDLE    : 대기 중
# RUNNING : 컨베이어 구동 중, ROI 감지 중
# SORTING : 서보 동작 중
system_state = "IDLE"
state_lock   = threading.Lock()

sock: socket.socket | None = None
sock_lock = threading.Lock()

last_result = ""   # 마지막 판별 결과 ("GOOD" / "BAD")
_run_gen    = 0    # start 호출 세대 번호 (오래된 타이머 무효화용)


# ═══════════════════════════════════════════
# 소켓 전송
# ═══════════════════════════════════════════

def send_cmd(cmd: str) -> bool:
    """라즈베리파이로 명령 전송. 실패 시 재연결 시도."""
    global sock
    message = cmd.strip().upper() + "\n"
    with sock_lock:
        for attempt in range(2):
            try:
                sock.sendall(message.encode("utf-8"))
                print(f"[전송] → {cmd.strip().upper()}")
                return True
            except (BrokenPipeError, OSError):
                print(f"[소켓] 연결 끊김, 재연결 중... ({attempt+1}/2)")
                try:
                    sock.close()
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.connect((RASPBERRY_PI_HOST, RASPBERRY_PI_PORT))
                    print("[소켓] 재연결 완료")
                except OSError as e:
                    print(f"[소켓] 재연결 실패: {e}")
                    return False
    return False


# ═══════════════════════════════════════════
# 판별 트리거
# ═══════════════════════════════════════════

def send_broken_ball_report(frame):
    """파손 감지 이미지를 320x320으로 리사이즈 후 Base64 인코딩하여 API 전송."""
    try:
        img_320 = cv2.resize(frame, (320, 320))
        _, buffer = cv2.imencode('.jpg', img_320)
        b64 = base64.b64encode(buffer).decode('utf-8')
        payload = {
            "location": {"room": 1},
            "image": f"data:image/jpeg;base64,{b64}"
        }
        resp = requests.post(
            "http://192.168.110.113:8000/api/broken_ball",
            json=payload,
            timeout=5
        )
        print(f"[API] 파손 이미지 전송 완료 (status: {resp.status_code})")
    except Exception as e:
        print(f"[API] 전송 실패: {e}")


def trigger_sort(result: str, frame=None):
    """RUNNING 상태에서 bad 결정 시 호출. SORTING으로 전이 후 명령 전송."""
    global system_state, last_result
    with state_lock:
        if system_state != "RUNNING":
            return
        system_state = "SORTING"

    last_result = result.upper()
    send_cmd(result.upper())
    print(f"[판별] → {result.upper()}")

    if result.upper() == "BAD":
        open(BAD_FLAG, 'w').close()  # 도봇에게 불량 공 처리 요청 신호
        if frame is not None:
            threading.Thread(target=send_broken_ball_report, args=(frame,), daemon=True).start()


# ═══════════════════════════════════════════
# 입력 스레드
# ═══════════════════════════════════════════

def input_thread():
    global system_state, _run_gen

    print("\n" + "="*50)
    print("  명령어: start | bad(수동) | stop | q")
    print("  crack 감지 시 자동으로 BAD 판별합니다.")
    print("="*50 + "\n")

    while True:
        try:
            cmd = input(">>> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        with state_lock:
            state = system_state

        if cmd == "start":
            with state_lock:
                system_state = "RUNNING"
                _run_gen += 1
                gen = _run_gen
            send_cmd("START")
            print("[안내] 컨베이어 구동 중... 17초 후 자동 초기화됩니다.")

            def auto_reset(gen=gen):
                global system_state, _run_gen
                time.sleep(17.0)
                with state_lock:
                    if _run_gen != gen:
                        return
                    system_state = "IDLE"
                send_cmd("STOP")
                print("[안내] 17초 경과 → 초기화. start를 입력하세요.")
                _set_ready()

            threading.Thread(target=auto_reset, daemon=True).start()

        elif cmd in ("good", "bad"):
            if state != "RUNNING":
                print(f"[입력] {cmd} 불가 (현재 상태: {state})")
                continue
            trigger_sort(cmd)

        elif cmd == "stop":
            with state_lock:
                system_state = "IDLE"
            send_cmd("STOP")
            print("[안내] 긴급 정지. start를 입력하면 재시작됩니다.")

        elif cmd == "q":
            send_cmd("STOP")
            print("[입력] 종료합니다.")
            break

        else:
            print("[입력] 알 수 없는 명령입니다. (start / good / bad / stop / q)")


# ═══════════════════════════════════════════
# 카메라 루프
# ═══════════════════════════════════════════

CAMERA_SERIAL   = "244222077012"   # 검사용 카메라
OVERVIEW_SERIAL = "243522072229"   # 전체 뷰 카메라 (camera1)

def camera_loop():
    """ROI 내 골프공 감지 시 즉시 crack 여부로 good/bad 판별."""

    print("[모델] YOLOv26 로드 중...")
    model = YOLO(MODEL_PATH)
    print(f"[모델] 로드 완료 | 클래스: {model.names}\n")

    # 검사 카메라
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_device(CAMERA_SERIAL)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 전체 뷰 카메라 (camera1)
    pipeline2 = rs.pipeline()
    config2   = rs.config()
    config2.enable_device(OVERVIEW_SERIAL)
    config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    print("[카메라] 시작 중...")
    pipeline.start(config)
    pipeline2.start(config2)
    print("[카메라] 시작 완료\n")
    _set_ready()  # 초기화 완료 → 도봇에게 첫 번째 pick 시작 가능 신호

    WIN_NAME = "Golf Ball Inspection"
    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)   # 창 크기 조절 가능
    cv2.resizeWindow(WIN_NAME, 640, 960)            # 초기 크기 (640 × 480×2)

    overview_frame = np.zeros((480, 640, 3), dtype=np.uint8)  # 초기 빈 화면

    # 버퍼 안정화
    pipeline.wait_for_frames()
    pipeline2.wait_for_frames()

    try:
        while True:
            # 검사 카메라: 버퍼를 비우고 최신 프레임 획득
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

            # 전체 뷰 카메라: 최신 프레임 (없으면 이전 유지)
            ov_frames = pipeline2.poll_for_frames()
            if ov_frames and ov_frames.get_color_frame():
                overview_frame = np.asanyarray(ov_frames.get_color_frame().get_data()).copy()

            with state_lock:
                state = system_state

            if state == "RUNNING":
                # 모델은 가장 낮은 임계값으로 호출 → 루프에서 클래스별 필터링
                results = model(frame, verbose=False,
                                imgsz=IMGSZ, conf=CRACK_CONF_THRESHOLD, iou=IOU_THRESHOLD)[0]

                has_crack = False
                has_ball  = False

                for box in results.boxes:
                    cls_id = int(box.cls[0])
                    conf   = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    is_crack = (cls_id == 0)
                    if conf < (CRACK_CONF_THRESHOLD if is_crack else BALL_CONF_THRESHOLD):
                        continue

                    box_color = (0, 0, 255) if is_crack else (0, 255, 0)
                    cv2.rectangle(display, (x1, y1), (x2, y2), box_color, 2)
                    cv2.putText(display, f"{model.names[cls_id]} {conf:.2f}",
                                (x1, max(y1 - 8, 0)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2)

                    if is_crack:
                        has_crack = True
                    else:
                        has_ball = True

                # 골프공이 프레임 안에 있을 때만 crack 판정 (가장자리 오감지 방지)
                if has_crack and has_ball:
                    print(f"[디버그] crack 감지 → BAD")
                    threading.Thread(target=trigger_sort, args=("bad", frame.copy()), daemon=True).start()

            # SORTING 중 판별 결과 크게 표시
            if state == "SORTING" and last_result:
                text      = last_result            # "GOOD" or "BAD"
                txt_color = (0, 220, 0) if text == "GOOD" else (0, 0, 255)
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 3.0, 5)
                tx = (640 - tw) // 2
                ty = (480 + th) // 2
                cv2.putText(display, text, (tx, ty),
                            cv2.FONT_HERSHEY_DUPLEX, 3.0, (30, 30, 30), 8)   # 그림자
                cv2.putText(display, text, (tx, ty),
                            cv2.FONT_HERSHEY_DUPLEX, 3.0, txt_color, 5)


            # 두 화면 사이 구분선 추가 후 합치기
            separator = np.full((10, 640, 3), 255, dtype=np.uint8)
            combined = np.vstack([display, separator, overview_frame])
            cv2.imshow(WIN_NAME, combined)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                send_cmd("STOP")
                break

    finally:
        pipeline.stop()
        pipeline2.stop()
        cv2.destroyAllWindows()
        print("[카메라] 종료")


# ═══════════════════════════════════════════
# 진입점
# ═══════════════════════════════════════════

def main(host: str, port: int):
    global sock, RASPBERRY_PI_HOST, RASPBERRY_PI_PORT
    RASPBERRY_PI_HOST = host
    RASPBERRY_PI_PORT = port

    print(f"[소켓] {host}:{port} 연결 중...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("[소켓] 연결 완료")

    t = threading.Thread(target=input_thread, daemon=True)
    t.start()

    try:
        camera_loop()
    finally:
        sock.close()
        print("[소켓] 연결 종료")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="골프공 판별 클라이언트")
    parser.add_argument("--host", default=RASPBERRY_PI_HOST,
                        help=f"라즈베리파이 IP (기본값: {RASPBERRY_PI_HOST})")
    parser.add_argument("--port", type=int, default=RASPBERRY_PI_PORT,
                        help=f"포트 번호 (기본값: {RASPBERRY_PI_PORT})")
    args = parser.parse_args()
    main(args.host, args.port)
