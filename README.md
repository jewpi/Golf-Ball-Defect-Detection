# 골프공 불량 감지 및 분류 시스템

Dobot 로봇 암, RealSense 카메라, YOLOv8 모델, 라즈베리파이 컨베이어를 연동하여 골프공 표면 결함을 자동으로 검사하고 분류하는 시스템입니다.

---

## 시스템 구성

```
[노트북]
  ├── dobot_suctionmove.py   ← Dobot 제어 (메인 진입점)
  │       │ subprocess 실행
  │       ↓
  └── pc_client2(final).py  ← YOLO 검사 + 카메라 표시
              │ TCP 소켓
              ↓
         [라즈베리파이]
          rpi_server.py      ← 컨베이어 + 서보 제어
```

---

## 동작 흐름

1. `dobot_suctionmove.py` 실행 → `pc_client2(final).py`를 서브프로세스로 자동 시작
2. `pc_client2`가 카메라·소켓 초기화 완료 → `/tmp/dobot_pc_ready` 플래그 생성
3. Dobot이 플래그를 감지하면 Pick 위치로 이동 → 공을 집어 Place 위치에 올려놓음
4. Dobot이 초기 위치 복귀 후 `pc_client2`에 `start` 명령 전송
5. `pc_client2`가 라즈베리파이로 `START` → 컨베이어 구동 + YOLO 검사 시작
6. **GOOD**: 17초 후 자동으로 다음 Pick 준비 신호 발생
7. **BAD**: `BAD_FLAG` 파일 생성 + 라즈베리파이로 `BAD` 전송 → Dobot이 불량 공 제거
8. Pick 위치 2개를 모두 처리한 뒤 마지막 검사 결과까지 대기 후 종료

---

## 파일별 설명

### 1. `dobot_suctionmove.py` (노트북)

Dobot 로봇 암의 전체 동작을 제어하는 메인 스크립트입니다.

**실행 방법**
```bash
python dobot_suctionmove.py
```

**주요 설정값**

| 변수 | 설명 |
|------|------|
| `LIFT_Z = 50.0` | 이동 시 들어올릴 Z 높이 (mm) |
| `PRESS_Z = -35.0` | Pick 시 공을 누르는 Z 깊이 (mm) |
| `PICK_POSITIONS` | Pick 위치 좌표 목록 (기본 2개) |
| `PLACE_X/Y/Z/R` | Place 위치 좌표 |
| `BAD_PICK_X/Y/Z/R` | 불량 공 수거 위치 좌표 |
| `BAD_DROP_X/Y/Z/R` | 불량 공 폐기 위치 좌표 |
| `BAD_PRESS_Z = -20.0` | 불량 공 수거 시 press 깊이 (mm) |

**Pick & Place 순서**
1. Pick 위치로 이동 (XY → Z 순)
2. `PRESS_Z`까지 하강 후 2초 대기
3. Suction ON → 2초 대기
4. `LIFT_Z`로 상승 → Place 위치 XY 이동 → Place Z 하강
5. Suction OFF → 상승 → 초기 위치 복귀
6. `pc_client2`에 `start` 전송

**불량 공 처리 순서 (`handle_bad_ball`)**
1. `BAD_PICK` 위치로 이동 (XY → Z 순)
2. `BAD_PRESS_Z`까지 하강 → 2초 대기
3. Suction ON → 2초 대기
4. `BAD_DROP_Z`로 상승 → `BAD_DROP` 위치 XY 이동
5. Suction OFF
6. 왔던 경로로 역추적 후 초기 위치 복귀 (특이점 회피)

**IPC 플래그 파일**

| 파일 | 역할 |
|------|------|
| `/tmp/dobot_pc_ready` | `pc_client2`가 다음 명령 수신 준비 완료를 알림 |
| `/tmp/dobot_bad_ball` | `pc_client2`가 불량 판정 시 Dobot에게 알림 |

---

### 2. `pc_client2(final).py` (노트북)

YOLOv8로 골프공 결함을 감지하고 라즈베리파이 컨베이어를 제어하는 스크립트입니다. `dobot_suctionmove.py`에 의해 서브프로세스로 자동 실행됩니다.

**주요 설정값**

| 변수 | 설명 |
|------|------|
| `RASPBERRY_PI_HOST` | 라즈베리파이 IP (`192.168.110.143`) |
| `RASPBERRY_PI_PORT` | 라즈베리파이 포트 (`9999`) |
| `CAMERA_SERIAL` | 검사용 RealSense 카메라 시리얼 번호 |
| `OVERVIEW_SERIAL` | 전체뷰 RealSense 카메라 시리얼 번호 |
| `BALL_CONF_THRESHOLD = 0.25` | 골프공 감지 신뢰도 임계값 |
| `CRACK_CONF_THRESHOLD = 0.20` | 균열 감지 신뢰도 임계값 |

**카메라 화면**
- 검사 카메라(위)와 전체뷰 카메라(아래)를 흰색 구분선으로 나눠 하나의 창에 표시
- 창 크기 조절 가능 (`cv2.WINDOW_NORMAL`)
- 판별 결과 `GOOD` / `BAD` 텍스트 오버레이 표시

**판별 로직**
- YOLO가 `crack(균열)` AND `ball(골프공)` 모두 감지한 경우에만 BAD 판정
- 공이 없는 상태에서 균열 오감지 방지
- 프레임 버퍼를 매 루프마다 비워 최신 프레임만 YOLO에 입력

**BAD 판정 시 동작**
1. 라즈베리파이로 `BAD` 전송
2. `/tmp/dobot_bad_ball` 플래그 생성
3. 판정 당시 프레임을 320×320 JPEG로 인코딩 → 파손 이력 API 전송 (비동기)

**파손 이력 API**
```
POST http://192.168.110.113:8000/api/broken_ball
Content-Type: application/json

{
  "location": {"room": 1},
  "image": "data:image/jpeg;base64,<base64 인코딩 이미지>"
}
```

**자동 초기화 (17초 타이머)**
- `start` 명령 수신 후 17초 경과 시 자동으로 `IDLE` 상태 전환
- 라즈베리파이로 `STOP` 전송 후 Ready 플래그 생성

---

### 3. `rpi_server.py` (라즈베리파이)

스텝모터(컨베이어)와 서보모터(분류기)를 소켓 명령으로 제어하는 서버입니다.

**실행 방법**
```bash
python3 rpi_server.py --port 9999
```

**GPIO 핀 배선**

| 핀 | GPIO | 연결 대상 |
|----|------|----------|
| DIR | 17 | 스텝모터 드라이버 DIR |
| STEP | 27 | 스텝모터 드라이버 STEP |
| ENABLE | 22 | 스텝모터 드라이버 ENABLE |
| SERVO | 18 | 서보모터 신호선 |

**소켓 명령어**

| 명령 | 동작 |
|------|------|
| `START` | 2초 대기 후 컨베이어 구동 시작 + 15초 타임아웃 시작 |
| `BAD` | 서보 95° → 175° 이동, 10초 유지 후 95° 복귀 → 컨베이어 정지 |
| `STOP` | 즉시 전체 정지 → IDLE |

**상태 전이**
```
IDLE ──[START]──→ RUNNING ──[BAD]──→ SORTING ──(완료)──→ IDLE
 ↑                  │                    │
 └──────[STOP]──────┘                    │
 └──────────────(15초 타임아웃)───────────┘
```

**주요 설정값**

| 변수 | 설명 |
|------|------|
| `STEP_DELAY = 0.0004` | 스텝모터 펄스 간격 (초) |
| `ANGLE_STANDBY = 95` | 서보 대기 위치 (°) |
| `ANGLE_BAD = 175` | 불량 분류 위치 (°) |
| `SORT_HOLD_TIME = 10.0` | 서보 분류 위치 유지 시간 (초) |
| `START_TIMEOUT = 15.0` | START 후 자동 강제 초기화 시간 (초) |

---

## 의존성

**노트북**
```bash
pip install pydobot pyrealsense2 opencv-python numpy ultralytics requests
```

**라즈베리파이**
```bash
pip3 install gpiod
```

---

## 실행 순서

1. **라즈베리파이**에서 `rpi_server.py` 먼저 실행
2. **노트북**에서 `dobot_suctionmove.py` 실행 (`pc_client2(final).py`는 자동 시작됨)
