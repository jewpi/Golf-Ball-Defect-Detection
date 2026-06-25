"""
vision.py
=========
픽셀 좌표 검출 함수 모음.
 - detect_color_marker : 캘리브레이션용 색 마커 검출 (가장 안정적)
 - detect_golf_ball    : 흰 골프공 검출 (흰색 마스크 + Hough, 폴백: 원형도)
둘 다 (u, v, annotated_image) 반환. 검출 실패 시 u,v = None.
"""

import cv2
import numpy as np


def detect_color_marker(bgr, hsv_lower, hsv_upper, min_area=80):
    """
    지정한 HSV 범위의 색 마커(예: 그리퍼에 붙인 빨간/파란 스티커) 중심 검출.
    hsv_lower, hsv_upper : (H,S,V) 튜플. H는 0~179, S/V는 0~255.
    빨강처럼 H가 0 근처에서 갈라지는 색은 pick_node/collect에서 두 범위를 OR 하세요.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lower), np.array(hsv_upper))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    out = bgr.copy()
    if not cnts:
        return None, None, out
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None, None, out
    M = cv2.moments(c)
    if M['m00'] == 0:
        return None, None, out
    u = M['m10'] / M['m00']
    v = M['m01'] / M['m00']
    cv2.circle(out, (int(u), int(v)), 6, (0, 0, 255), 2)
    cv2.drawContours(out, [c], -1, (0, 255, 0), 1)
    return float(u), float(v), out


def detect_golf_ball(bgr, min_radius=15, max_radius=80,
                     white_s_max=60, white_v_min=170):
    """
    흰 골프공 검출.

    Step 1: HSV 흰색 마스크로 배경 제거 → CLAHE + Hough Circle
    Step 2: Hough 실패 시 컨투어 원형도(circularity ≥ 0.75) 폴백

    흰 배경일 때: 골프공 아래에 어두운 천/종이를 깔면 검출률이 크게 오릅니다.

    파라미터:
      white_s_max  : 흰색으로 볼 채도 상한 (낮을수록 더 하얀 것만)
      white_v_min  : 흰색으로 볼 밝기 하한
    """
    out = bgr.copy()
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # ── Step 1: 흰색 마스크 ──────────────────────────────────────────
    white_mask = cv2.inRange(
        hsv,
        np.array([0,   0,          white_v_min]),
        np.array([179, white_s_max, 255])
    )
    white_mask = cv2.morphologyEx(
        white_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    white_mask = cv2.morphologyEx(
        white_mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)
    # 마스크 외 영역을 0으로 → Hough가 흰 공에만 집중
    gray_masked = cv2.bitwise_and(gray, gray, mask=white_mask)
    gray_blurred = cv2.GaussianBlur(gray_masked, (9, 9), 2)

    circles = cv2.HoughCircles(
        gray_blurred, cv2.HOUGH_GRADIENT,
        dp=1.2, minDist=100,
        param1=50, param2=20,
        minRadius=min_radius, maxRadius=max_radius)

    if circles is not None:
        circles = np.uint16(np.around(circles[0]))
        best = max(circles, key=lambda c: c[2])
        u, v, r = float(best[0]), float(best[1]), int(best[2])
        cv2.circle(out, (int(u), int(v)), r, (0, 255, 0), 2)
        cv2.circle(out, (int(u), int(v)), 3,  (0, 0, 255), 3)
        cv2.putText(out, "Hough", (int(u) + r + 4, int(v)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return u, v, out

    # ── Step 2: 컨투어 원형도 폴백 ──────────────────────────────────
    cnts, _ = cv2.findContours(
        white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt, best_score = None, 0.0
    for c in cnts:
        area = cv2.contourArea(c)
        if area < np.pi * min_radius ** 2:
            continue
        perim = cv2.arcLength(c, True)
        if perim == 0:
            continue
        circularity = 4 * np.pi * area / (perim ** 2)
        if circularity < 0.75:
            continue
        (_, _), radius = cv2.minEnclosingCircle(c)
        if not (min_radius <= radius <= max_radius):
            continue
        score = circularity * area
        if score > best_score:
            best_score = score
            best_cnt = c

    if best_cnt is not None:
        M = cv2.moments(best_cnt)
        if M['m00'] > 0:
            u = M['m10'] / M['m00']
            v = M['m01'] / M['m00']
            (_, _), radius = cv2.minEnclosingCircle(best_cnt)
            cv2.circle(out, (int(u), int(v)), int(radius), (0, 200, 255), 2)
            cv2.circle(out, (int(u), int(v)), 3, (0, 0, 255), 3)
            cv2.putText(out, "Contour", (int(u) + int(radius) + 4, int(v)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            return float(u), float(v), out

    return None, None, out


# 흔히 쓰는 HSV 범위 (현장 조명에 맞게 조정)
HSV_PRESETS = {
    'red_low':  ((0, 120, 80),   (10, 255, 255)),
    'red_high': ((170, 120, 80), (179, 255, 255)),
    'blue':     ((100, 120, 60), (130, 255, 255)),
    'green':    ((45, 80, 60),   (85, 255, 255)),
    'orange':   ((10, 130, 100), (25, 255, 255)),
}
