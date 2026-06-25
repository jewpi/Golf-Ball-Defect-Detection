import pydobot
import time
import subprocess
import sys
import os
from serial.tools import list_ports

READY_FLAG = '/tmp/dobot_pc_ready'
BAD_FLAG   = '/tmp/dobot_bad_ball'

# 불량 공 처리 좌표
BAD_PICK_X, BAD_PICK_Y, BAD_PICK_Z, BAD_PICK_R = 180.33, -130.25,  -8.76, -35.84
BAD_DROP_X, BAD_DROP_Y, BAD_DROP_Z, BAD_DROP_R =  15.06, -165.80,  97.09, -84.81
BAD_PRESS_Z = -20.0  # 불량 공 집을 때 press 깊이

def handle_bad_ball(home_x, home_y, home_z, home_r):
    """불량 공을 집어서 폐기 위치로 이동 후 초기 위치 복귀."""
    print("\n[BAD] 불량 공 제거 루틴 시작")

    # 1. 불량 공 위치로 이동 (XY 먼저, Z 마지막)
    device.move_to(BAD_PICK_X, BAD_PICK_Y, LIFT_Z, BAD_PICK_R, wait=True)
    device.move_to(BAD_PICK_X, BAD_PICK_Y, BAD_PICK_Z, BAD_PICK_R, wait=True)
    print("불량 공 위치 도착")

    # 2. Z 방향으로 press (더 눌러서 밀착)
    device.move_to(BAD_PICK_X, BAD_PICK_Y, BAD_PRESS_Z, BAD_PICK_R, wait=True)
    print(f"Press 완료 → Z: {BAD_PRESS_Z}")
    time.sleep(2)

    # 3. Suction ON
    device.suck(True)
    print("Suction ON")
    time.sleep(2)

    # 3. 폐기 위치로 이동 (Z 먼저, XY 나중)
    device.move_to(BAD_PICK_X, BAD_PICK_Y, BAD_DROP_Z, BAD_PICK_R, wait=True)
    device.move_to(BAD_DROP_X, BAD_DROP_Y, BAD_DROP_Z, BAD_DROP_R, wait=True)
    print("폐기 위치 도착")

    # 4. Suction OFF
    device.suck(False)
    print("Suction OFF")

    # 5. 초기 위치 복귀 (XY를 pick 위치로 역경로 후 홈으로)
    device.move_to(BAD_PICK_X, BAD_PICK_Y, BAD_DROP_Z, BAD_PICK_R, wait=True)
    device.move_to(home_x, home_y, home_z, home_r, wait=True)
    print("[BAD] 불량 공 처리 완료, 초기 위치 복귀\n")


def wait_for_ready(timeout=60):
    """pc_client2가 start 명령 대기 상태가 될 때까지 폴링."""
    print("pc_client2 준비 대기 중...", end='', flush=True)
    elapsed = 0
    while not os.path.exists(READY_FLAG):
        if pc_proc.poll() is not None:
            raise RuntimeError(f"pc_client2 프로세스가 종료됨 (exit code: {pc_proc.returncode})")
        time.sleep(0.5)
        elapsed += 0.5
        if elapsed % 5 == 0:
            print('.', end='', flush=True)
        if elapsed >= timeout:
            raise TimeoutError("pc_client2 준비 타임아웃 (60초 초과)")
    os.remove(READY_FLAG)
    print(" 준비 완료!")


# 이전 세션의 잔여 플래그 파일 제거
for _flag in (READY_FLAG, BAD_FLAG):
    if os.path.exists(_flag):
        os.remove(_flag)

# pc_client2 먼저 실행 (카메라·소켓 초기화 포함)
PC_CLIENT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pc_client2(final).py")
print(f"pc_client2 실행 중...")
pc_proc = subprocess.Popen(
    [sys.executable, PC_CLIENT],
    stdin=subprocess.PIPE,
    text=True,
    bufsize=1
)

# 도봇 연결
port = list_ports.comports()[0].device
device = pydobot.Dobot(port='/dev/ttyACM0', verbose=False)

# 설정
LIFT_Z  = 50.0   # 들어올릴 Z 높이 (mm)
PRESS_Z = -35.0  # Pick 위치에서 더 누를 Z 높이 (mm)

PICK_POSITIONS = [
    ( 81.73,  205.31, -23.38,  68.29),  # Pick 1
    (111.75,  170.47, -23.58,  56.75),  # Pick 2
]

PLACE_X, PLACE_Y, PLACE_Z, PLACE_R = 149.63, 79.85, 28.76, 28.09

# 초기 위치 저장
x, y, z, r, *_ = device.pose()
print(f"현재 위치 - X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}, R: {r:.2f}")

total = len(PICK_POSITIONS)

for i, (px, py, pz, pr) in enumerate(PICK_POSITIONS, 1):
    print(f"\n{'='*40}")
    print(f"  [{i}/{total}] Pick & Place 시작")
    print(f"{'='*40}")

    # pc_client2가 start 대기 상태일 때만 pick 시작
    wait_for_ready()

    # 이전 검사에서 BAD 판정 시 불량 공 제거 후 진행
    if os.path.exists(BAD_FLAG):
        os.remove(BAD_FLAG)
        handle_bad_ball(x, y, z, r)

    # 1. Pick 위치로 이동 (MoveJ)
    device.move_to(px, py, pz, pr, wait=True, mode=pydobot.enums.PTPMode.MOVJ_XYZ)
    print(f"Pick 위치 도착 - X: {px}, Y: {py}, Z: {pz}, R: {pr}")

    # 2. Z 방향으로 press
    device.move_to(px, py, PRESS_Z, pr, wait=True)
    print(f"Press 완료 → Z: {PRESS_Z}")
    time.sleep(2)

    # 3. Suction ON
    device.suck(True)
    print("Suction ON")
    time.sleep(2)

    # 4. 들어올리기
    device.move_to(px, py, LIFT_Z, pr, wait=True)
    print(f"Z 상승 완료 → Z: {LIFT_Z}")

    # 5. Place XY로 이동
    device.move_to(PLACE_X, PLACE_Y, LIFT_Z, PLACE_R, wait=True)
    print("XY 이동 완료")

    # 6. Place Z로 하강
    device.move_to(PLACE_X, PLACE_Y, PLACE_Z, PLACE_R, wait=True)
    print("Z 하강 완료")

    # 7. Suction OFF
    device.suck(False)
    print("Suction OFF")

    # 8. 하강한만큼 Z 상승 (Place Z → Lift Z)
    device.move_to(PLACE_X, PLACE_Y, LIFT_Z, PLACE_R, wait=True)
    print(f"Z 상승 완료 → Z: {LIFT_Z}")

    # 9. 초기 위치 복귀 (Z 먼저, XY 나중)
    device.move_to(PLACE_X, PLACE_Y, z, PLACE_R, wait=True)
    device.move_to(x, y, z, r, wait=True)
    print(f"초기 위치 복귀 완료 [{i}/{total}]")

    # 9. 컨베이어 + 검사 시작 (공이 올려진 직후)
    if pc_proc.poll() is not None:
        raise RuntimeError(f"pc_client2 프로세스가 종료됨 (exit code: {pc_proc.returncode})")
    pc_proc.stdin.write("start\n")
    pc_proc.stdin.flush()
    print(f"[전송] start → pc_client2  [{i}/{total}]")

print(f"\n{'='*40}")
print("  모든 Pick & Place 완료")
print(f"{'='*40}")

# 마지막 검사 결과 대기 후 BAD면 불량 공 처리
wait_for_ready(timeout=60)
if os.path.exists(BAD_FLAG):
    os.remove(BAD_FLAG)
    handle_bad_ball(x, y, z, r)

device.close()

# pc_client2는 마지막 검사를 계속 진행하도록 유지
pc_proc.wait()
