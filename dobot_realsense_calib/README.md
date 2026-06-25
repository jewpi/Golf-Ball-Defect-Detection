# dobot_realsense_calib

Dobot Magician + RealSense **Eye-to-Hand 평면 Affine 캘리브레이션** (ROS2 Humble).
카메라가 작업판을 위에서 내려다보는 고정형 구성에서, 픽셀 좌표(u,v)를
로봇 좌표(x,y)로 바꾸는 2x3 Affine 행렬을 구합니다.

---

## 0. 빌드

```bash
# 워크스페이스로 복사
cp -r dobot_realsense_calib ~/ros2_ws/src/

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # cv_bridge 등 의존성
colcon build --packages-select dobot_realsense_calib
source install/setup.bash
```

---

## 1. ⚠ 먼저 robot_interface.py 수정 (필수)

`dobot_realsense_calib/robot_interface.py`가 당신의 Dobot ROS2 패키지와
대화하는 유일한 파일입니다. 아래 3개 함수만 채우면 됩니다.

```bash
# 당신 패키지 인터페이스 확인
ros2 topic list      # 현재좌표 토픽 (예: /dobot_pose)
ros2 service list    # 이동/흡착 서비스
ros2 interface show <타입>
```

- `get_pose()`        : 현재 (x,y,z,r) mm 반환  ← 구독 콜백 파싱만 맞추면 됨
- `move_to(x,y,z,r)`  : PTP 이동 (블로킹)
- `set_suction(on)`   : 흡착 on/off

파일 안에 흔한 형태(토픽 구독 + 서비스 호출) 예시가 주석으로 들어 있습니다.

---

## 2. 캘리브레이션 데이터 수집

마커는 **그리퍼 끝에 붙인 색 스티커**를 추천합니다 (흰 골프공은 흰 배경에서
검출이 까다로움). 마커 중심이 흡착컵 중심과 일치하도록 붙이세요.

```bash
ros2 run dobot_realsense_calib calib_collect \
  --ros-args -p image_topic:=/camera/camera1/color/image_raw \
             -p marker:=blue \
             -p pose_topic:=/dobot_pose \
             -p out_path:=$HOME/pairs.yaml
```

화면 조작:
- 로봇을 작업판 위 한 지점으로 jog → 초록 원으로 마커가 잡히면 `s` 저장
- **6~12지점**을 작업판 전체에 골고루 (네 모서리 + 가운데 포함하면 정확도↑)
- `u` 마지막 취소, `q` 저장 후 종료

> 검출이 안 잡히면 marker 색을 red/green/orange로 바꾸거나
> `vision.py`의 `HSV_PRESETS` 범위를 현장 조명에 맞게 조정하세요.

---

## 3. Affine 행렬 계산

```bash
ros2 run dobot_realsense_calib calib_compute \
  --ros-args -p pairs_path:=$HOME/pairs.yaml \
             -p out_path:=$HOME/calibration.yaml
```

출력되는 **RMS 오차(mm)** 를 확인하세요. 5mm 이하면 픽앤플레이스에 충분합니다.
크면 점을 더 넓게/많이 찍으세요.

---

## 4. 검증

```bash
ros2 run dobot_realsense_calib calib_verify \
  --ros-args -p calib_path:=$HOME/calibration.yaml
```

이미지를 클릭 → 변환된 로봇 좌표 출력. 그 좌표로 로봇을 보내
끝점이 클릭 지점에 오는지 눈으로 확인합니다.

---

## 5. 실행 (골프공 집기)

```bash
ros2 run dobot_realsense_calib pick_node \
  --ros-args -p calib_path:=$HOME/calibration.yaml \
             -p target:=ball \
             -p auto:=false \
             -p place_xy:='[200.0, -120.0]'
```

`p` 키로 집기 실행 (auto:=true면 검출 즉시 자동). `q` 종료.

---

## z(높이)에 대해

작업판이 평평하므로 z는 수집 시 평균값(`pick_z`)을 고정으로 씁니다.
물체 높이가 제각각이면 RealSense **depth 토픽**(`/camera/.../aligned_depth_to_color/image_raw`)
에서 해당 픽셀 깊이를 읽어 z를 보정하면 됩니다. 필요하면 확장해 드릴게요.

---

## 정확도 팁

- 마커 지점은 한 줄로 몰지 말고 **2D로 넓게** 분포 (직선상이면 Affine 불안정)
- 마커 중심 = 흡착 지점이 되도록 정렬
- 카메라/작업판을 캘리브레이션 후 **움직이지 말 것** (움직이면 재캘리브레이션)
- 조명이 바뀌면 HSV 범위 재조정
