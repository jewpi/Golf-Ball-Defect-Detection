import pydobot
import time
import subprocess
import sys
import os
from serial.tools import list_ports

# 연결
port = list_ports.comports()[0].device
device = pydobot.Dobot(port='/dev/ttyACM0', verbose=False)

# 설정
LIFT_Z     = 50.0    # 들어올릴 Z 높이 (mm) ← 환경에 맞게 조정
PRESS_Z    = -35.0   # Pick 위치에서 더 누를 Z 높이 (mm)
PICK_X,  PICK_Y,  PICK_Z,  PICK_R  = 82.14,  205.79,  -23.99,  68.24
PLACE_X, PLACE_Y, PLACE_Z, PLACE_R = 170.90,  67.85,  26.18,  21.65

# 1. 현재 위치 확인
x, y, z, r, *_ = device.pose()
print(f"현재 위치 - X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}, R: {r:.2f}")

# 2. Pick 위치로 이동 (MoveJ)
device.move_to(PICK_X, PICK_Y, PICK_Z, PICK_R, wait=True, mode=pydobot.enums.PTPMode.MOVJ_XYZ)
print(f"Pick 위치 도착 - X: {PICK_X}, Y: {PICK_Y}, Z: {PICK_Z}, R: {PICK_R}")

# 3. Z 방향으로 press (더 눌러서 밀착)
device.move_to(PICK_X, PICK_Y, PRESS_Z, PICK_R, wait=True)
print(f"Press 완료 → Z: {PRESS_Z}")
print("2초 대기 중...")
time.sleep(2)

# 4. Suction ON
device.suck(True)
print("Suction ON")
time.sleep(2)

# 5. 위로 들어올리기
device.move_to(PICK_X, PICK_Y, LIFT_Z, PICK_R, wait=True)
print(f"Z 상승 완료 → Z: {LIFT_Z}")

# 6. 목표 XY 위치로 이동 (높은 상태 유지)
device.move_to(PLACE_X, PLACE_Y, LIFT_Z, PLACE_R, wait=True)
print("XY 이동 완료")

# 7. 목표 Z로 하강
device.move_to(PLACE_X, PLACE_Y, PLACE_Z, PLACE_R, wait=True)
print("Z 하강 완료")

# 8. Suction OFF
device.suck(False)
print("Suction OFF")

# 9. 초기 위치로 복귀
device.move_to(x, y, LIFT_Z, r, wait=True)
device.move_to(x, y, z, r, wait=True)
print(f"초기 위치 복귀 완료 - X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}, R: {r:.2f}")

device.close()

# 10. pc_client2 실행 후 start 전송
PC_CLIENT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pc_client2(final).py")
print(f"\npc_client2 실행 중... ({PC_CLIENT})")

pc_proc = subprocess.Popen(
    [sys.executable, PC_CLIENT],
    stdin=subprocess.PIPE,
    text=True,
    bufsize=1
)

print("카메라·소켓 초기화 대기 중 (5초)...")
time.sleep(5)

pc_proc.stdin.write("start\n")
pc_proc.stdin.flush()
print("[전송] start → pc_client2")

pc_proc.wait()