# -*- coding: utf-8 -*-
"""
[라즈베리파이 실행 코드] rpi_server.py
역할: 소켓 수신 → 컨베이어(스텝모터) + 서보모터 통합 제어

의존성:
    pip3 install gpiod

사용법:
    python3 rpi_server.py --port 9999

────────────────────────────────────────────
GPIO 핀 배선
────────────────────────────────────────────
  스텝모터 드라이버 (A4988 / DRV8825 / TB6600)
    DIR    → GPIO 17
    STEP   → GPIO 27
    ENABLE → GPIO 22

  서보모터 (소프트웨어 PWM)
    SIGNAL → GPIO 18

────────────────────────────────────────────
소켓 명령어 (노트북 → 라즈베리파이)
────────────────────────────────────────────
  "START\n"  : 2초 후 컨베이어 구동 시작 + 15초 타임아웃 시작
  "BAD\n"    : 불량 → 서보 95°→175° 이동, 유지 후 95° 복귀
  "STOP\n"   : 즉시 전체 정지 → IDLE

────────────────────────────────────────────
동작 흐름
────────────────────────────────────────────
  1. "START" 수신 → 2초 대기 → 컨베이어 CW 구동 + 15초 타이머 시작
  2. "BAD" 수신 → 서보 95°→175° (컨베이어 계속 구동)
     SORT_HOLD_TIME 유지 → 서보 95° 복귀 → 컨베이어 정지 → IDLE
  3. START 후 15초 경과 → 상태 무관하게 컨베이어·서보 강제 초기화 → IDLE
────────────────────────────────────────────
"""

import gpiod
import socket
import threading
import time
import argparse


# ═══════════════════════════════════════════
# 설정값
# ═══════════════════════════════════════════

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 9999

# ── 스텝모터 핀 ──
DIR_PIN    = 17
STEP_PIN   = 27
ENABLE_PIN = 22

STEP_DELAY   = 0.0004   # 스텝 간격(초)
CONVEYOR_DIR = 0        # 0=CW(정방향), 1=CCW

# ── 서보모터 핀 ──
SERVO_PIN    = 18

ANGLE_STANDBY = 95           # 상시 대기 위치
ANGLE_BAD     = 175          # 불량 → 175°

SORT_HOLD_TIME    = 10.0      # 분류 위치 유지 시간(초)
AFTER_SORT_DELAY  = 0.0      # 서보 복귀 후 컨베이어 추가 구동 시간(초)
SERVO_PULSES      = 20       # 소프트웨어 PWM 펄스 반복 횟수


# ═══════════════════════════════════════════
# gpiod 칩 (전역 공유)
# ═══════════════════════════════════════════

chip = gpiod.Chip('gpiochip0')


# ═══════════════════════════════════════════
# 스텝모터 컨트롤러
# ═══════════════════════════════════════════

class StepMotorController:
    """gpiod 기반 스텝모터 제어 (conv.py 로직 기반)."""

    def __init__(self):
        self._dir_line    = chip.get_line(DIR_PIN)
        self._step_line   = chip.get_line(STEP_PIN)
        self._enable_line = chip.get_line(ENABLE_PIN)

        self._dir_line.request(consumer="dir",     type=gpiod.LINE_REQ_DIR_OUT)
        self._step_line.request(consumer="step",   type=gpiod.LINE_REQ_DIR_OUT)
        self._enable_line.request(consumer="enable", type=gpiod.LINE_REQ_DIR_OUT)

        self._enable_line.set_value(1)          # 초기: 비활성화
        self._dir_line.set_value(CONVEYOR_DIR)
        self._step_line.set_value(0)

        self._running = False
        self._lock    = threading.Lock()

        print(f"[스텝모터] 초기화 완료 (DIR={DIR_PIN}, STEP={STEP_PIN}, EN={ENABLE_PIN})")

    def _pulse_loop(self):
        """conv.py의 step_motor()와 동일한 펄스 루프."""
        self._enable_line.set_value(0)           # 드라이버 활성화
        while self._running:
            self._step_line.set_value(1)
            time.sleep(STEP_DELAY)
            self._step_line.set_value(0)
            time.sleep(STEP_DELAY)
        self._step_line.set_value(0)             # STEP LOW 확정
        self._enable_line.set_value(1)           # 드라이버 비활성화

    def start(self):
        with self._lock:
            if self._running:
                print("[스텝모터] 이미 구동 중")
                return
            self._dir_line.set_value(CONVEYOR_DIR)
            self._running = True
            threading.Thread(target=self._pulse_loop, daemon=True).start()
        print("[스텝모터] 구동 시작")

    def stop(self):
        with self._lock:
            if not self._running:
                return
            self._running = False
        print("[스텝모터] 정지 신호 전달")

    def cleanup(self):
        self.stop()
        time.sleep(0.05)
        self._dir_line.release()
        self._step_line.release()
        self._enable_line.release()
        print("[스텝모터] 라인 해제 완료")


# ═══════════════════════════════════════════
# 서보모터 컨트롤러
# ═══════════════════════════════════════════

class ServoController:
    """gpiod 소프트웨어 PWM 기반 서보 제어 (servo.py 로직 기반)."""

    def __init__(self):
        self._servo_line = chip.get_line(SERVO_PIN)
        self._servo_line.request(consumer="servo", type=gpiod.LINE_REQ_DIR_OUT)
        self._lock = threading.Lock()

        self._move_to(ANGLE_STANDBY)
        print(f"[서보] 초기화 완료 → 대기 위치 {ANGLE_STANDBY}°")

    def _move_to(self, angle: float):
        """servo.py의 set_servo()와 동일한 소프트웨어 PWM."""
        angle = max(0, min(270, angle))
        pulse_width = (angle / 270) * (0.0025 - 0.0005) + 0.0005
        for _ in range(SERVO_PULSES):
            self._servo_line.set_value(1)
            time.sleep(pulse_width)
            self._servo_line.set_value(0)
            time.sleep(0.02 - pulse_width)

    def move_to(self, angle: float):
        with self._lock:
            self._move_to(angle)
        print(f"[서보] {angle}° 이동 완료")

    def sort_bad(self):
        """불량 판별 시 서보 동작: 95° → 175° → 95° 복귀."""
        print(f"[분류] 불량 → {ANGLE_BAD}°")
        self.move_to(ANGLE_BAD)

        print(f"[분류] {SORT_HOLD_TIME}초 유지 중...")
        time.sleep(SORT_HOLD_TIME)

        print(f"[분류] 복귀 → {ANGLE_STANDBY}°")
        self.move_to(ANGLE_STANDBY)

    def cleanup(self):
        self.move_to(ANGLE_STANDBY)
        self._servo_line.release()
        print("[서보] 라인 해제 완료")


# ═══════════════════════════════════════════
# 시스템 상태 머신
# ═══════════════════════════════════════════

class ConveyorSystem:
    """
    상태 전이:
        IDLE    → RUNNING  : "START" 수신 → 컨베이어 구동
        RUNNING → SORTING  : "GOOD"/"BAD" 수신 → 서보 동작 (컨베이어 계속 구동)
        SORTING → IDLE     : 서보 5초 유지 후 복귀 → 컨베이어 정지
        ANY     → IDLE     : "STOP" 수신 → 즉시 전체 정지
    """

    IDLE    = "IDLE"
    RUNNING = "RUNNING"
    SORTING = "SORTING"

    START_TIMEOUT = 15.0    # START 후 GOOD/BAD 미수신 시 자동 정지(초)

    def __init__(self, motor: StepMotorController, servo: ServoController):
        self.motor = motor
        self.servo = servo
        self.state = self.IDLE
        self._lock  = threading.Lock()
        self._timer: threading.Timer | None = None
        print(f"[시스템] 초기 상태: {self.state}")

    # ── 이벤트 핸들러 ──

    START_DELAY = 2.0       # START 수신 후 컨베이어 구동 대기(초)

    def on_start(self):
        with self._lock:
            if self.state != self.IDLE:
                print(f"[시스템] START 무시 (현재: {self.state})")
                return
            self._transition(self.RUNNING)

        print(f"[시스템] {self.START_DELAY}초 후 컨베이어 구동...")
        time.sleep(self.START_DELAY)
        self.motor.start()
        self._reset_timer()   # 15초 타임아웃 시작
        print(f"[시스템] 컨베이어 구동 중 → {self.START_TIMEOUT}초 내 GOOD/BAD 없으면 자동 정지")

    def on_bad_received(self):
        with self._lock:
            if self.state != self.RUNNING:
                print(f"[시스템] BAD 무시 (현재: {self.state})")
                return
            self._transition(self.SORTING)

        # 타이머 유지 — START 후 15초가 되면 무조건 강제 초기화
        threading.Thread(target=self._do_sort, daemon=True).start()

    def on_stop(self):
        self._cancel_timer()
        with self._lock:
            self._transition(self.IDLE)
        self.motor.stop()
        print("[시스템] 긴급 정지 → IDLE")

    # ── 내부 동작 ──

    def _do_sort(self):
        """
        서보 분류 수행 (BAD 전용).
        타임아웃이 먼저 발생하면 IDLE로 전환되므로 각 단계에서 상태를 확인한다.
        """
        self.servo.sort_bad()        # 95° → 175° → SORT_HOLD_TIME 유지 → 95° 복귀 (blocking)

        with self._lock:
            if self.state != self.SORTING:
                return   # 타임아웃이 이미 초기화함 → 중복 동작 방지

        print(f"[시스템] 서보 복귀 완료 → 컨베이어 {AFTER_SORT_DELAY}초 추가 구동")
        time.sleep(AFTER_SORT_DELAY)

        self.motor.stop()
        with self._lock:
            if self.state == self.SORTING:
                self._transition(self.IDLE)
        print("[시스템] 분류 완료 → 컨베이어 정지 → IDLE")

    def _reset_timer(self):
        """타임아웃 타이머를 (재)시작한다."""
        self._cancel_timer()
        self._timer = threading.Timer(self.START_TIMEOUT, self._on_timeout)
        self._timer.daemon = True
        self._timer.start()

    def _cancel_timer(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _on_timeout(self):
        """START 후 START_TIMEOUT초 경과 — 상태 무관하게 강제 초기화."""
        with self._lock:
            if self.state == self.IDLE:
                return          # 이미 정지 상태면 무시
            self._transition(self.IDLE)
        self.motor.stop()
        self.servo.move_to(ANGLE_STANDBY)   # 서보가 분류 중이었어도 95°로 복귀
        print(f"[시스템] {self.START_TIMEOUT}초 경과 → 강제 초기화 → IDLE")

    def _transition(self, new_state: str):
        """상태 전이 로그 (반드시 _lock 보유 상태에서 호출)."""
        print(f"[시스템] {self.state} → {new_state}")
        self.state = new_state


# ═══════════════════════════════════════════
# 소켓 서버
# ═══════════════════════════════════════════

def handle_client(conn: socket.socket, addr: tuple, system: ConveyorSystem):
    print(f"[소켓] {addr} 접속")
    buffer = ""

    try:
        while True:
            data = conn.recv(1024)
            if not data:
                print(f"[소켓] {addr} 연결 종료")
                break

            buffer += data.decode("utf-8")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                cmd = line.strip().upper()

                if cmd == "START":
                    print("[소켓] START 수신")
                    system.on_start()

                elif cmd == "BAD":
                    print("[소켓] BAD 수신")
                    system.on_bad_received()

                elif cmd == "STOP":
                    print("[소켓] STOP 수신")
                    system.on_stop()

                elif cmd:
                    print(f"[소켓] 알 수 없는 명령: '{cmd}' (무시)")

    except (ConnectionResetError, OSError) as e:
        print(f"[소켓] {addr} 오류: {e}")
    finally:
        conn.close()


def run_server(port: int, system: ConveyorSystem):
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((SERVER_HOST, port))
    server_sock.listen(5)
    print(f"[소켓] 포트 {port}에서 대기 중...")

    try:
        while True:
            conn, addr = server_sock.accept()
            threading.Thread(
                target=handle_client,
                args=(conn, addr, system),
                daemon=True
            ).start()
    finally:
        server_sock.close()


# ═══════════════════════════════════════════
# 진입점
# ═══════════════════════════════════════════

def main(port: int):
    motor  = StepMotorController()
    servo  = ServoController()
    system = ConveyorSystem(motor, servo)

    print("\n[시스템] 준비 완료. 소켓 명령 대기 중...\n")

    try:
        run_server(port, system)
    except KeyboardInterrupt:
        print("\n[종료] 사용자 요청으로 종료합니다.")
    finally:
        motor.cleanup()
        servo.cleanup()
        chip.close()
        print("[종료] 완료")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="골프공 분류 시스템 서버 (라즈베리파이)")
    parser.add_argument("--port", type=int, default=SERVER_PORT,
                        help=f"수신 포트 (기본값: {SERVER_PORT})")
    args = parser.parse_args()
    main(args.port)
