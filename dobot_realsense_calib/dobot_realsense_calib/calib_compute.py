"""
calib_compute.py
================
calib_collect가 만든 pairs.yaml을 읽어 Affine 변환(픽셀 uv -> 로봇 xy)을 계산하고
calibration.yaml로 저장합니다. 재투영 오차(RMS, mm)도 출력합니다.

실행:
  ros2 run dobot_realsense_calib calib_compute \\
    --ros-args -p pairs_path:=./pairs.yaml \\
               -p out_path:=./calibration.yaml

ROS 없이 순수 파이썬으로도 돌릴 수 있게 argv도 지원:
  python3 calib_compute.py ./pairs.yaml ./calibration.yaml
"""

import sys
import yaml
import numpy as np
import cv2


def compute(pairs_path, out_path):
    with open(pairs_path) as f:
        data = yaml.safe_load(f)

    pairs = data['pairs']
    if len(pairs) < 3:
        raise ValueError(f"점이 {len(pairs)}개뿐입니다. Affine은 최소 3점(권장 6점+) 필요.")

    pixel = np.array([p['pixel'] for p in pairs], dtype=np.float32)
    robot = np.array([p['robot'] for p in pairs], dtype=np.float32)

    # 픽셀 -> 로봇 Affine (2x3). RANSAC으로 이상치 자동 제거.
    M, inliers = cv2.estimateAffine2D(pixel, robot,
                                      method=cv2.RANSAC,
                                      ransacReprojThreshold=3.0)
    if M is None:
        raise RuntimeError("Affine 추정 실패. 점 분포가 한 줄로 몰렸는지 확인하세요.")

    # 재투영 오차 (mm)
    ones = np.ones((len(pixel), 1), dtype=np.float32)
    px_h = np.hstack([pixel, ones])          # Nx3
    pred = (M @ px_h.T).T                     # Nx2
    err = np.linalg.norm(pred - robot, axis=1)
    rms = float(np.sqrt(np.mean(err ** 2)))
    n_in = int(inliers.sum()) if inliers is not None else len(pairs)

    out = {
        'transform_type': 'affine_2x3_pixel_to_robot',
        'image_topic': data.get('image_topic', ''),
        'pick_z': float(data.get('pick_z', 0.0)),
        'matrix': M.astype(float).tolist(),   # [[a,b,c],[d,e,f]]
        'rms_error_mm': rms,
        'max_error_mm': float(err.max()),
        'num_points': len(pairs),
        'num_inliers': n_in,
    }
    with open(out_path, 'w') as f:
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)

    print(f"[OK] '{out_path}' 저장 완료")
    print(f"  점 {len(pairs)}개 (inlier {n_in})")
    print(f"  RMS 오차 = {rms:.2f} mm,  최대 오차 = {err.max():.2f} mm")
    if rms > 5.0:
        print("  ⚠ RMS가 5mm보다 큽니다. 점을 더 넓게/많이 찍거나 마커 중심을 점검하세요.")
    else:
        print("  ✔ 픽앤플레이스에 충분한 정확도입니다.")
    return out


def main():
    # ROS2 파라미터 경로
    pairs_path, out_path = './pairs.yaml', './calibration.yaml'
    try:
        import rclpy
        from rclpy.node import Node
        rclpy.init()
        node = Node('calib_compute')
        node.declare_parameter('pairs_path', pairs_path)
        node.declare_parameter('out_path', out_path)
        pairs_path = node.get_parameter('pairs_path').value
        out_path = node.get_parameter('out_path').value
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        # 순수 파이썬 호출
        if len(sys.argv) >= 3:
            pairs_path, out_path = sys.argv[1], sys.argv[2]

    compute(pairs_path, out_path)


if __name__ == '__main__':
    main()
