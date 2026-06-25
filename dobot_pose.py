import pydobot
from serial.tools import list_ports

# 연결
port = list_ports.comports()[0].device
device = pydobot.Dobot(port='/dev/ttyACM0', verbose=False)

# 현재 좌표 확인
x, y, z, r, *_ = device.pose()
print(f"X: {x:.2f} mm")
print(f"Y: {y:.2f} mm")
print(f"Z: {z:.2f} mm")
print(f"R: {r:.2f} °")