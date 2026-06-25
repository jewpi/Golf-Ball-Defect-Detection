import pydobot
import time
from serial.tools import list_ports

# 연결
port = list_ports.comports()[0].device
device = pydobot.Dobot(port='/dev/ttyACM0', verbose=False)

LIFT_Z  = 50.0   # 들어올릴 Z 높이 (mm)
PRESS_Z = -35.0  # 누를 Z 높이 (mm)

# 1. 현재 위치 확인
x, y, z, r, *_ = device.pose()
print(f"현재 위치 - X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}, R: {r:.2f}")

# 2. 살짝 누르기
device.move_to(x, y, PRESS_Z, r, wait=True)
print(f"누르기 완료 → Z: {PRESS_Z}")

# 3. 2초 대기
print("2초 대기 중...")
time.sleep(2)

# 4. Suction ON
device.suck(True)
print("Suction ON")
time.sleep(2)

# 5. 위로 들어올리기
device.move_to(x, y, LIFT_Z, r, wait=True)
print(f"Z 상승 완료 → Z: {LIFT_Z}")

# 6. 원래 Z로 하강
device.move_to(x, y, z, r, wait=True)
print(f"Z 하강 완료 → Z: {z:.2f}")

# 7. Suction OFF
device.suck(False)
print("Suction OFF")

device.close()