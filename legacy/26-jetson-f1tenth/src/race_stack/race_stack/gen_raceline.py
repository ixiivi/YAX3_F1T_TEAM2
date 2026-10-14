#!/usr/bin/env python3
"""
gen_raceline.py — TUMFTM 최소 곡률 레이싱라인 생성기 + GUI 드로잉 모드

알고리즘: Heilmeier et al. "Minimum Curvature Trajectory Planning and Control
for an Autonomous Racecar" (2019), DOI: 10.1080/00423114.2019.1631455

의존 패키지: numpy, scipy, osqp, opencv-python, Pillow
  pip install numpy scipy osqp opencv-python Pillow

사용법:
  python3 gen_raceline.py                            # 대화형 맵 선택 → TUMFTM
  python3 gen_raceline.py --map halfhot_clean        # 맵 지정 → TUMFTM
  python3 gen_raceline.py --map halfhot_clean --gui  # GUI 드로잉 모드
  python3 gen_raceline.py --map halfhot_clean --gui --out halfhot_clean_late_wp
  python3 gen_raceline.py --map halfhot_clean --vmax 8.0 --mu 0.4 --rmin 1.0
"""

import os, sys, csv, argparse, math
import numpy as np
import cv2
from PIL import Image
from scipy.interpolate import CubicSpline, interp1d
from scipy.spatial import cKDTree
import scipy.ndimage as ndimage
import osqp
import scipy.sparse as sp

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ★ 차량 최소회전반경 [m] — 라인이 아무리 빨라도 못 도는 건 소용없다.
#   R = wheelbase / tan((servo_min − offset) / (−gain))
#
#   ★ 손으로 적지 않는다. config/vehicle_<차>.yaml 에서 읽어 계산한다 —
#     예전엔 여기 0.856 이 박혀 있었는데 컨트롤러는 wheelbase 0.33 을 써서
#     0.872 로 보고 있었다. 같은 값을 두 군데 적으면 반드시 갈라진다.
#     차량 파일이 없으면(다른 기기에서 분석만 할 때) 아래 폴백을 쓴다.
def _car_min_radius(fallback=0.872):
    import math
    for name in ('.vehicle_overlay.yaml',):
        p = os.path.join(_PKG_DIR, '..', '..', 'config', name)
        p = os.path.abspath(p)
        if not os.path.exists(p):
            continue
        try:
            import yaml as _y
            o = _y.safe_load(open(p))
            ms = float(o['pure_pursuit']['max_steer'])
            wb = float(o['pure_pursuit']['wheelbase'])
            return wb / math.tan(ms)
        except Exception:
            pass
    return fallback


R_CAR_MIN = _car_min_radius()
MAPS_DIR = os.path.join(_PKG_DIR, 'maps')
WP_DIR   = os.path.join(_PKG_DIR, 'waypoints')

# ══════════════════════════════════════════════════════════════════════════════
# 1. 맵 → 트랙 경계 추출
# ══════════════════════════════════════════════════════════════════════════════
def extract_reftrack(map_name: str, safety_m: float, n_cl: int = 300,
                     return_bounds: bool = False, gap_fill: float = 0.05):
    """
    pgm 맵에서 centerline + 트랙 폭을 추출한다.
    반환: [N, 4] = [x, y, w_right, w_left] (미터)
    """
    yaml_path = os.path.join(MAPS_DIR, map_name + '.yaml')
    img_path  = None
    for ext in ['.pgm', '.png']:
        p = os.path.join(MAPS_DIR, map_name + ext)
        if os.path.exists(p):
            img_path = p; break
    if img_path is None:
        sys.exit(f'맵 이미지 없음: {MAPS_DIR}/{map_name}.pgm')

    import yaml
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    res    = float(cfg['resolution'])
    origin = np.array(cfg['origin'][:2], dtype=float)

    img = np.array(Image.open(img_path).convert('L'))
    h, w = img.shape

    def px2world(px, py):
        x = np.asarray(px, float) * res + origin[0]
        y = (h - np.asarray(py, float)) * res + origin[1]
        return x, y

    free_b = img > 128

    # ── 벽의 틈 메우기 ────────────────────────────────────────────────
    #
    # ★ 트랙 경계에 작은 틈이 있으면 라이다가 그리로 새어 나가 맵에 통로가
    #   생긴다. 그러면 "가장 큰 자유공간 성분" 이 **샌 통로까지 포함**하고,
    #   윤곽이 그 통로를 따라 들어갔다 나오면서 중심선이 벽에 붙는다.
    #
    #   2026-08-14 실측 (0815_clean) — 이걸 안 하면:
    #       복도 최소폭(능선)  0.070m   ← 실제 벽이 아니라 **샌 통로의 능선**
    #       중심선 최소 여유   0.098m   (차 반폭 0.155m 미달 → 라인 생성 실패)
    #     벽을 5cm(1px) 메우면:
    #       복도 최소폭        0.750m   자유 픽셀 변화는 −24px 뿐
    #     즉 침식이 아니라 **틈만 정확히 막는다.**
    #
    #   ★ border_value=1 이 필수다. 기본값(0)이면 맵 밖을 자유공간으로 보고
    #     **테두리의 벽 채움을 침식한다** — 실제로 자유 픽셀이 20817→22853 으로
    #     늘어 트랙이 통째로 넓어졌다. 맵 밖은 벽이다.
    if gap_fill > 0.0:
        r_px = max(1, int(round(gap_fill / res)))
        kern = np.zeros((2 * r_px + 1, 2 * r_px + 1), bool)
        yy, xx = np.ogrid[-r_px:r_px + 1, -r_px:r_px + 1]
        kern[(yy * yy + xx * xx) <= r_px * r_px] = True
        wall = ndimage.binary_closing(~free_b, structure=kern, border_value=1)
        n_filled = int(free_b.sum()) - int((~wall).sum())   # + = 자유가 줄었다(=틈이 막혔다)
        free_b = ~wall
        print(f'  벽 틈 메움: {gap_fill:.2f}m({r_px}px) — 자유 {-n_filled:+d}px '
              f'{"(틈만 막힘)" if abs(n_filled) < 500 else "⚠ 변화가 크다 — 침식 의심"}')

    free = free_b.astype(np.uint8) * 255
    n_lbl, labels, stats, _ = cv2.connectedComponentsWithStats(free)
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    track_mask = (labels == largest).astype(np.uint8) * 255

    contours, hierarchy = cv2.findContours(
        track_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    outer_c = inner_c = None
    for c, hi in zip(contours, hierarchy[0]):
        area = cv2.contourArea(c)
        if hi[3] < 0:
            if outer_c is None or area > cv2.contourArea(outer_c): outer_c = c
        elif area > 500:
            if inner_c is None or area > cv2.contourArea(inner_c): inner_c = c

    if outer_c is None or inner_c is None:
        sys.exit('경계 추출 실패 — 맵을 clean_map.py로 정제하세요')

    def contour_world(c):
        pts = c.reshape(-1, 2)
        x, y = px2world(pts[:, 0], pts[:, 1])
        return np.column_stack([x, y])

    outer_w = contour_world(outer_c)
    inner_w = contour_world(inner_c)

    # 균일 리샘플
    def resample_closed(pts, n):
        closed = np.vstack([pts, pts[0]])
        ds = np.linalg.norm(np.diff(closed, axis=0), axis=1)
        s  = np.concatenate([[0], np.cumsum(ds)])
        s_new = np.linspace(0, s[-1], n, endpoint=False)
        xi = np.interp(s_new, s, closed[:, 0])
        yi = np.interp(s_new, s, closed[:, 1])
        return np.column_stack([xi, yi])

    outer_rs = resample_closed(outer_w, n_cl)

    # 중심선: outer 각 점 → 가장 가까운 inner 점의 중점
    tree_inner = cKDTree(inner_w)
    tree_outer = cKDTree(outer_w)
    _, idx_i = tree_inner.query(outer_rs)
    midpoints = (outer_rs + inner_w[idx_i]) / 2.0

    # 방향 보정 (CCW)
    cx_, cy_ = midpoints[:, 0], midpoints[:, 1]
    area = 0.5 * np.sum(cx_ * np.roll(cy_, -1) - np.roll(cx_, -1) * cy_)
    if area < 0:
        midpoints = midpoints[::-1]

    # 중심선 스플라인 스무딩
    from scipy.interpolate import splprep, splev
    tck, _ = splprep([midpoints[:, 0], midpoints[:, 1]],
                     s=len(midpoints) * 0.01, per=True, k=3)
    u = np.linspace(0, 1, n_cl, endpoint=False)
    cx_s, cy_s = splev(u, tck)
    cl = np.column_stack([cx_s, cy_s])

    # ── 중심선 재중심화 ───────────────────────────────────────────────
    #
    # 위 중심선은 "바깥 점마다 가장 가까운 안쪽 점과의 중점" 이다. 두 경계가
    # 나란하면 맞지만, 코너에서 안쪽 경계가 휘어 달아나면 중점이 바깥벽 쪽으로
    # 밀린다. fun_clean 실측: s=2.7m 에서 우 0.159m / 좌 0.903m (치우침 70%).
    #
    # ★★ 2026-08-14 재작성 — 예전 방식은 **거리 기준이 틀렸다.**
    #   `shift = (d_inner − d_outer)/2` 로 밀었는데, d_inner 는 "인필드 윤곽점
    #   까지의 거리" 다. 인필드가 트랙을 고르게 감싸면 맞지만, 한쪽으로 치우쳐
    #   있으면 그 구간에서 d_inner 가 엉뚱하게 커진다(0815_clean 실측 5.68m).
    #   그러면 shift 가 2.5m 씩 튀고, 5점 스무딩이 그걸 도로 끌어당겨
    #   **중심선이 바깥벽에서 2cm 인 채로 굳는다.**
    #     0815_clean: 중심선 최소 여유 0.019m ← 차 반폭 0.155m 미달 → 라인 실패
    #     0814-1_clean 이 되던 건 인필드가 고르게 감싸 우연히 맞아떨어져서다.
    #
    #   이제 **거리변환**을 쓴다 — "어느 벽이든 가장 가까운 벽까지" 라
    #   윤곽을 안팎으로 나눌 필요가 없다. 법선 방향으로만 훑어 가장 먼 점을
    #   고르므로 점 순서도 보존된다 (자유 이동은 능선을 따라 뭉친다).
    dist_m = cv2.distanceTransform(track_mask, cv2.DIST_L2, 5) * res

    def dist_at(pts):
        cxp = (pts[:, 0] - origin[0]) / res
        ryp = h - (pts[:, 1] - origin[1]) / res
        jj = np.clip(np.round(cxp).astype(int), 0, w - 1)
        ii = np.clip(np.round(ryp).astype(int), 0, h - 1)
        return dist_m[ii, jj]

    span = np.linspace(-0.30, 0.30, 25)      # 한 번에 ±30cm 안에서만 찾는다
    for _ in range(20):
        tx = np.roll(cl[:, 0], -1) - np.roll(cl[:, 0], 1)
        ty = np.roll(cl[:, 1], -1) - np.roll(cl[:, 1], 1)
        tn = np.maximum(np.hypot(tx, ty), 1e-9)
        nx, ny = -ty / tn, tx / tn
        # 법선 위 여러 지점의 벽거리를 재서 가장 먼 곳으로
        best_t = np.zeros(len(cl))
        best_d = dist_at(cl)
        for t in span:
            cand = cl + np.column_stack([nx, ny]) * t
            dcand = dist_at(cand)
            upd = dcand > best_d
            best_d = np.where(upd, dcand, best_d)
            best_t = np.where(upd, t, best_t)
        if np.max(np.abs(best_t)) < 1e-3:
            break
        cl = cl + np.column_stack([nx, ny]) * best_t[:, None]
        # 매 스텝 가볍게 다듬어 지그재그를 막는다
        k = np.ones(5) / 5
        for c in (0, 1):
            ext = np.r_[cl[-2:, c], cl[:, c], cl[:2, c]]
            cl[:, c] = np.convolve(ext, k, 'valid')

    # 트랙 폭 계산 (중심선 → outer/inner 거리)
    d_o, _ = tree_outer.query(cl)
    d_i, _ = tree_inner.query(cl)

    # 안전 마진 적용 후 최소 폭 보장
    # ★ 이 하한은 작아야 한다 (2026-08-09). 0.20 이면 실제로 0.12m 밖에 없는
    #   병목에서 QP 가 0.20m 를 쓸 수 있다고 착각해 벽 쪽으로 8cm 더 나간다.
    #   없는 폭을 만들어내면 안 된다 — 좁으면 좁은 대로 보여야 α 가 0 이 된다.
    min_w = 0.02
    w_right = np.maximum(d_o - safety_m, min_w)
    w_left  = np.maximum(d_i - safety_m, min_w)

    reftrack = np.column_stack([cl, w_right, w_left])
    print(f'  트랙 추출: {n_cl}pts  길이≈{np.sum(np.linalg.norm(np.diff(cl, axis=0), axis=1)):.1f}m')
    print(f'  폭: right {d_o.min():.2f}~{d_o.max():.2f}m  left {d_i.min():.2f}~{d_i.max():.2f}m')
    if return_bounds:
        # 반복 최적화가 매 스텝 폭을 **경계에서 다시 재려면** 이게 필요하다.
        # α 를 더해 빼는 근사(w_right−α)만 쓰면 반복하며 오차가 누적돼
        # 라인이 조용히 트랙 밖으로 샌다.
        return reftrack, outer_w, inner_w
    return reftrack


# ══════════════════════════════════════════════════════════════════════════════
# 2. 균일 호 길이 리샘플
# ══════════════════════════════════════════════════════════════════════════════
def interp_track(reftrack: np.ndarray, stepsize: float) -> np.ndarray:
    """호 길이 기준으로 reftrack을 균일 간격 리샘플."""
    pts = reftrack[:, :2]
    closed = np.vstack([pts, pts[0]])
    ds = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s  = np.concatenate([[0], np.cumsum(ds)])
    n_new = max(3, int(s[-1] / stepsize))
    s_new = np.linspace(0, s[-1], n_new, endpoint=False)

    out = np.zeros((n_new, 4))
    for col in range(4):
        vals_cl = np.append(reftrack[:, col], reftrack[0, col])
        out[:, col] = np.interp(s_new, s, vals_cl)
    return out


# ══════════════════════════════════════════════════════════════════════════════
# 3. 3차 스플라인 시스템 (TUMFTM calc_splines 재구현)
# ══════════════════════════════════════════════════════════════════════════════
def calc_splines(path: np.ndarray) -> tuple:
    """
    닫힌 경로에 곡률 연속 3차 스플라인 피팅.
    path: [N+1, 2] — 마지막 행이 첫 행과 동일한 닫힌 경로

    반환:
      coeffs_x, coeffs_y: [N, 4] 스플라인 계수 [a0, a1, a2, a3]
      M: [4N, 4N] 선형 시스템 행렬
      normvec: [N, 2] 정규화된 법선벡터 (오른쪽 방향)
    """
    assert np.allclose(path[0], path[-1]), "닫힌 경로여야 합니다 (첫점 = 마지막점)"
    N = path.shape[0] - 1  # 스플라인 구간 수 = 제어점 수

    # 스케일링: 인접 구간 거리 비
    el = np.linalg.norm(np.diff(path, axis=0), axis=1)  # [N]
    el_ext = np.append(el, el[0])
    scaling = el_ext[:-1] / el_ext[1:]  # [N]

    # 4N × 4N 선형 시스템
    M  = np.zeros((4 * N, 4 * N))
    bx = np.zeros(4 * N)
    by = np.zeros(4 * N)

    # 각 스플라인 구간 i (0 ~ N-2): 위치 연속 + C1/C2 연속
    for i in range(N - 1):
        j = i * 4
        # P_i(0) = p_i
        M[j, j] = 1.0
        bx[j] = path[i, 0]; by[j] = path[i, 1]
        # P_i(1) = p_{i+1}
        M[j+1, j:j+4] = [1, 1, 1, 1]
        bx[j+1] = path[i+1, 0]; by[j+1] = path[i+1, 1]
        # C1: P'_i(1) = s * P'_{i+1}(0)
        M[j+2, j+1] = 1; M[j+2, j+2] = 2; M[j+2, j+3] = 3
        M[j+2, j+5] = -scaling[i]
        # C2: P''_i(1) = s^2 * P''_{i+1}(0)
        M[j+3, j+2] = 2; M[j+3, j+3] = 6
        M[j+3, j+6] = -2 * scaling[i]**2

    # 마지막 구간 N-1: 위치만 + 닫힘 조건
    j = (N - 1) * 4
    M[j,   j:j+4] = [1, 0, 0, 0]
    bx[j] = path[N-1, 0]; by[j] = path[N-1, 1]
    M[j+1, j:j+4] = [1, 1, 1, 1]
    bx[j+1] = path[0, 0]; by[j+1] = path[0, 1]
    # 닫힘 C1
    M[j+2, j+1] = scaling[N-1]
    M[j+2, j+2] = 2 * scaling[N-1]
    M[j+2, j+3] = 3 * scaling[N-1]
    M[j+2, 1] = -1
    # 닫힘 C2
    M[j+3, j+2] = 2 * scaling[N-1]**2
    M[j+3, j+3] = 6 * scaling[N-1]**2
    M[j+3, 2] = -2

    sol_x = np.linalg.solve(M, bx)
    sol_y = np.linalg.solve(M, by)

    coeffs_x = sol_x.reshape(N, 4)  # [a0, a1, a2, a3] per segment
    coeffs_y = sol_y.reshape(N, 4)

    # 법선벡터: 헤딩(a1)의 수직 방향 (오른쪽)
    # tangent = [a1_x, a1_y], normal = [a1_y, -a1_x] (right-hand)
    normvec = np.stack([coeffs_y[:, 1], -coeffs_x[:, 1]], axis=1)
    norms   = np.linalg.norm(normvec, axis=1, keepdims=True)
    norms   = np.maximum(norms, 1e-9)
    normvec /= norms

    return coeffs_x, coeffs_y, M, normvec


# ══════════════════════════════════════════════════════════════════════════════
# 4. 최소 곡률 QP (TUMFTM opt_min_curv 재구현)
# ══════════════════════════════════════════════════════════════════════════════
def opt_min_curv(reftrack: np.ndarray,
                 normvec:  np.ndarray,
                 M_spl:   np.ndarray,
                 w_veh:   float,
                 r_min:   float = 0.9,
                 alpha_max: float = 0.0,
                 quiet:   bool = False) -> np.ndarray:
    """
    최소 곡률 경로 최적화 (OSQP QP 풀이).

    수식 (Heilmeier et al. 2019):
      경로: p_i = p_ref_i + α_i * n_i
      목적: min Σ κ_i²(α)
      → QP: min ½ α' H α + f' α
      제약: -w_left[i]+W/2 ≤ α[i] ≤ w_right[i]-W/2

    H = E'E,  f = 2 E' κ_ref
    E = diag(cx) T_ny − diag(cy) T_nx
    cx_i = x'_i / ||(x',y')||^(3/2),  cy_i = y'_i / ||(x',y')||^(3/2)
    T_nx = (A_ex_c @ A_inv) @ M_x_norm
    T_ny = (A_ex_c @ A_inv) @ M_y_norm
    """
    N = reftrack.shape[0]

    # ── 추출 행렬 ──
    # A_ex_c: 각 스플라인 a2 계수(= x''/2) 추출 → x''_i = 2*a2_i
    A_ex_c = np.zeros((N, 4 * N))
    for i in range(N):
        A_ex_c[i, i * 4 + 2] = 2.0  # 2*a2_i

    # A_ex_b: a1 계수(= x'_i at t=0) 추출
    A_ex_b = np.zeros((N, 4 * N))
    for i in range(N):
        A_ex_b[i, i * 4 + 1] = 1.0

    A_inv = np.linalg.inv(M_spl)
    T_c   = A_ex_c @ A_inv   # [N, 4N]: maps q → x''

    # ── 법선벡터를 RHS에 투영하는 행렬 M_x, M_y ──
    # alpha[i]가 q_x[j=4i]와 q_x[j=4i+1]을 normvec[i,0]만큼 이동
    M_x = np.zeros((4 * N, N))
    M_y = np.zeros((4 * N, N))
    for i in range(N):
        j = i * 4
        i_next = (i + 1) % N
        M_x[j,      i]      = normvec[i,      0]
        M_x[j + 1,  i_next] = normvec[i_next, 0]
        M_y[j,      i]      = normvec[i,      1]
        M_y[j + 1,  i_next] = normvec[i_next, 1]

    # ── 참조선 좌표를 스플라인 RHS 벡터 [4N]로 구성 ──
    # 스플라인 시스템의 position 행(4i, 4i+1)에 좌표 배치
    q_x = np.zeros(4 * N)
    q_y = np.zeros(4 * N)
    for i in range(N):
        j = i * 4
        i_next = (i + 1) % N
        q_x[j]   = reftrack[i,      0]
        q_x[j+1] = reftrack[i_next, 0]
        q_y[j]   = reftrack[i,      1]
        q_y[j+1] = reftrack[i_next, 1]

    # 스플라인 RHS → x'_i = a1_i at t=0
    x_prime = (A_ex_b @ A_inv @ q_x)  # [N]
    y_prime = (A_ex_b @ A_inv @ q_y)  # [N]

    # ── 곡률 분모 (= ||(x',y')||^(3/2)) ──
    denom = np.maximum((x_prime**2 + y_prime**2) ** 1.5, 1e-9)  # [N]
    cx = x_prime / denom   # [N]
    cy = y_prime / denom   # [N]

    # ── T_nx, T_ny: α → Δx''(제2도함수 변화량) ──
    # T_c: [N, 4N],  M_x: [4N, N]  → T_nx: [N, N]
    T_nx = T_c @ M_x   # [N, N]
    T_ny = T_c @ M_y   # [N, N]

    # ── E 행렬: α → Δκ ──
    # Δκ_i = cx_i * (T_ny @ α)_i - cy_i * (T_nx @ α)_i
    E = np.diag(cx) @ T_ny - np.diag(cy) @ T_nx   # [N, N]

    # ── 참조선 곡률 ──
    x2_ref = T_c @ q_x   # x''_ref [N] (q_x는 [4N] 벡터)
    y2_ref = T_c @ q_y   # y''_ref [N]
    kappa_ref = cx * y2_ref - cy * x2_ref   # [N]

    # ── QP 행렬 ──
    H = E.T @ E                        # [N, N] PSD
    H = (H + H.T) / 2.0               # 수치 대칭 보장
    f = 2.0 * E.T @ kappa_ref          # [N]

    # ── 제약: -w_left + W/2 ≤ α ≤ w_right - W/2 ──
    dev_right = reftrack[:, 2] - w_veh / 2   # [N]
    dev_left  = reftrack[:, 3] - w_veh / 2   # [N]
    dev_right = np.maximum(dev_right, 0.01)
    dev_left  = np.maximum(dev_left,  0.01)

    # ★ 신뢰영역 — E 는 '노드를 α 만큼 밀어도 호길이는 그대로'라는 1차 전개다.
    #   |α| 가 격자 간격에 근접하면 그 가정이 무너져 **모델이 실제 곡률과
    #   완전히 어긋난다** (2026-08-09 실측: QP 는 R=0.90 이라는데 그 노드를
    #   정확히 통과시키면 R=0.12 였다). 한 번에 조금만 움직이고 재선형화한다.
    if alpha_max and alpha_max > 0:
        dev_right = np.minimum(dev_right, alpha_max)
        dev_left  = np.minimum(dev_left,  alpha_max)

    # OSQP: min ½ x'Px + q'x  s.t. l ≤ Ax ≤ u
    # 제약 1: 트랙 폭  -dev_left ≤ α ≤ dev_right
    # 제약 2: R_min   |κ_ref + E·α| ≤ 1/r_min
    #          → -1/r_min - κ_ref ≤ E·α ≤ 1/r_min - κ_ref
    kappa_max = 1.0 / max(r_min, 1e-3)
    A_full = sp.vstack([sp.eye(N, format='csc'),
                        sp.csc_matrix(E)], format='csc')
    l_full = np.concatenate([-dev_left,  -kappa_max - kappa_ref])
    u_full = np.concatenate([ dev_right,  kappa_max - kappa_ref])

    prob = osqp.OSQP()
    prob.setup(
        P=sp.csc_matrix(H),
        q=f,
        A=A_full,
        l=l_full,
        u=u_full,
        verbose=False,
        eps_abs=1e-9,
        eps_rel=1e-9,
        max_iter=50000,
        polish=True,
        polish_refine_iter=10,
    )
    result = prob.solve()
    if result.info.status not in ('solved', 'solved_inaccurate'):
        print(f'  경고: OSQP 상태 = {result.info.status}')

    alpha = np.clip(result.x, -dev_left, dev_right)

    # 곡률 개선 확인
    kappa_new = kappa_ref + E @ alpha
    if not quiet:
        print(f'  참조선 |κ|_max={np.abs(kappa_ref).max():.4f}  '
              f'레이싱라인 |κ|_max={np.abs(kappa_new).max():.4f} 1/m')
    return alpha


# ══════════════════════════════════════════════════════════════════════════════
# 5. 레이싱라인 생성 + 균일 리샘플
# ══════════════════════════════════════════════════════════════════════════════
def build_raceline(reftrack: np.ndarray,
                   normvec:  np.ndarray,
                   alpha:    np.ndarray,
                   stepsize: float = 0.1) -> tuple:
    """⚠ 폐기됨 (2026-08-09). eval_spline_dense 를 쓸 것.

    splprep 로 노드를 **근사**하는 별개의 곡선을 만들기 때문에, QP 가 최적화한
    곡선과 다르다. r_min 을 0.9/1.0/1.1 로 바꿔도 출력 R 이 0.803/0.813/0.813 로
    거의 변하지 않았다 — 최종 라인을 QP 가 아니라 스무더가 정하고 있었다.
    호출부는 남아 있지 않지만, 같은 실수를 다시 하지 않도록 이유와 함께 남긴다.
    """
    N = len(reftrack)
    nodes = reftrack[:, :2] + alpha[:, None] * normvec   # [N, 2]

    # 노드 간 호 길이
    ds_n  = np.linalg.norm(np.diff(np.vstack([nodes, nodes[0]]), axis=0), axis=1)
    total = ds_n.sum()

    # splprep (periodic, 가벼운 스무딩) — 진동 방지
    from scipy.interpolate import splprep, splev
    tck, _ = splprep([nodes[:, 0], nodes[:, 1]],
                     s=N * 0.002, per=True, k=3)

    M   = max(3, int(total / stepsize))
    u_u = np.linspace(0, 1, M, endpoint=False)

    xi, yi = splev(u_u, tck)
    rl = np.column_stack([xi, yi])

    el = np.linalg.norm(np.diff(np.vstack([rl, rl[0]]), axis=0), axis=1)

    dx, dy   = splev(u_u, tck, der=1)
    ddx, ddy = splev(u_u, tck, der=2)
    kappa = np.abs(dx * ddy - dy * ddx) / np.maximum((dx**2 + dy**2) ** 1.5, 1e-9)

    print(f'  레이싱라인: {M}pts  길이={total:.1f}m  |κ|_max={kappa.max():.4f} 1/m')
    return rl, el, kappa



def eval_spline_dense(nodes: np.ndarray, stepsize: float, n_sub: int = 40):
    """노드를 **정확히 통과하는** 닫힌 3차 스플라인을 조밀 평가해 균일 리샘플.

    ★ 왜 splprep 스무딩을 안 쓰나 (2026-08-09) ────────────────────────────
    예전 build_raceline 은 `splprep(..., s=N*0.002)` 로 노드를 **근사**하는 별개의
    곡선을 새로 만들었다. QP 가 최적화한 곡선(calc_splines 의 보간 스플라인)과
    다른 곡선이라, r_min 을 아무리 조여도 출력 곡률이 따라오지 않았다:

        r_min 0.9 → 출력 R 0.803 / r_min 1.0 → 0.813 / r_min 1.1 → 0.813

    즉 최종 라인은 QP 가 아니라 **스무더가** 정하고 있었다. 최적화한 것과
    주행하는 것이 달랐다. 여기서는 calc_splines 를 그대로 써서 둘을 일치시킨다.
    """
    closed = np.vstack([nodes, nodes[0]])
    cx, cy, _, _ = calc_splines(closed)
    N = len(nodes)

    t = np.linspace(0.0, 1.0, n_sub, endpoint=False)
    P  = np.array([np.ones_like(t), t, t**2, t**3])          # [4, n_sub]
    D1 = np.array([np.zeros_like(t), np.ones_like(t), 2*t, 3*t**2])
    D2 = np.array([np.zeros_like(t), np.zeros_like(t), 2*np.ones_like(t), 6*t])

    x   = (cx @ P ).ravel();  y   = (cy @ P ).ravel()
    dx  = (cx @ D1).ravel();  dy  = (cy @ D1).ravel()
    ddx = (cx @ D2).ravel();  ddy = (cy @ D2).ravel()

    # 곡률은 매개변수화에 무관한 식이라 t 로 미분해도 그대로 쓸 수 있다
    kap = np.abs(dx * ddy - dy * ddx) / np.maximum((dx**2 + dy**2) ** 1.5, 1e-9)

    pts = np.column_stack([x, y])
    seg = np.linalg.norm(np.diff(np.vstack([pts, pts[0]]), axis=0), axis=1)
    s_c = np.concatenate([[0.0], np.cumsum(seg)])            # [n+1]
    total = float(s_c[-1])

    M = max(3, int(total / stepsize))
    s_new = np.linspace(0.0, total, M, endpoint=False)
    xi = np.interp(s_new, s_c, np.append(x, x[0]))
    yi = np.interp(s_new, s_c, np.append(y, y[0]))
    ki = np.interp(s_new, s_c, np.append(kap, kap[0]))

    rl = np.column_stack([xi, yi])
    el = np.linalg.norm(np.diff(np.vstack([rl, rl[0]]), axis=0), axis=1)
    return rl, el, ki, total


def signed_curvature(line: np.ndarray, smooth_m: float = 0.5) -> np.ndarray:
    """닫힌 폴리라인의 **부호 있는** 곡률 [1/m]. 세 점 원(Menger)으로 잰다.

    부호 규약을 calc_splines 의 법선(오른쪽 방향)에 맞춘다:
        +  오른쪽으로 돈다  → 코너 안쪽이 **오른쪽**(바깥 경계 쪽)
        −  왼쪽으로 돈다    → 코너 안쪽이 **왼쪽**(안쪽 경계 쪽)

    세 점 곡률은 점 간격이 촘촘하면 잡음이 크므로 호길이 smooth_m 만큼
    이동평균한다. 안 하면 직선 구간에서도 곡률이 튀어 여유가 들쭉날쭉해진다.
    """
    p = np.asarray(line, float)[:, :2]
    a, b, c = np.roll(p, 1, axis=0), p, np.roll(p, -1, axis=0)
    v1, v2 = b - a, c - b
    cross = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]      # + = 좌회전(CCW)
    l1 = np.linalg.norm(v1, axis=1)
    l2 = np.linalg.norm(v2, axis=1)
    l3 = np.linalg.norm(c - a, axis=1)
    k = 2.0 * cross / np.maximum(l1 * l2 * l3, 1e-12)
    k = -k                                                  # 법선(오른쪽)에 맞춤
    ds = float(np.median(l1)) or 0.1
    n = max(1, int(round(smooth_m / max(ds, 1e-6))) | 1)     # 홀수
    if n > 1:                                               # 순환 이동평균
        pad = np.r_[k[-(n // 2):], k, k[:n // 2]]
        k = np.convolve(pad, np.ones(n) / n, 'valid')[:len(p)]
    return k


def apex_safety(line: np.ndarray, safety_m: float,
                apex_extra: float, apex_kappa: float):
    """코너 **안쪽에만** 여유를 더 준다 → (오른쪽 여유, 왼쪽 여유).

    ★ 왜 균일한 --safety 로는 부족한가 (2026-08-14)
      --safety 는 전 구간을 똑같이 좁힌다. 그런데 **추종 오차는 코너에 몰린다** —
      실측(08/13~14, 5.3㎧): 직선 0.16m / 완만 0.23m / 코너 0.39~0.55m.
      직선까지 같이 좁히면 랩타임만 잃고 정작 필요한 곳은 여전히 모자란다.

    ★ 왜 안쪽만인가
      코너에서 벽에 붙는 곳은 **안쪽 정점(apex)** 이다. 바깥은 그대로 두어야
      아웃-인-아웃 진입이 살아 있어 반경을 크게 쓸 수 있다. 양쪽을 다 좁히면
      라인이 코너 한복판에 갇혀 R 이 작아지고, 그러면 더 느려진다.

    여유는 곡률에 비례해 매끄럽게 는다 (계단이면 그 지점에서 라인이 꺾인다):
        extra = apex_extra · clip(|κ| / apex_kappa, 0, 1)
    apex_kappa 는 "이 곡률이면 여유를 다 준다" 는 기준이고, 기본은 1/r_min —
    즉 라인이 낼 수 있는 가장 조인 코너에서 최대 여유가 된다.

    ★ 기본 0.40m 의 근거 (0814-1_clean, mu 0.42, safety 0.30 고정)

        apex   코너여유   −차반폭   R_min    랩(η=1.0)
        0      0.360     0.190    1.125    7.61s
        0.20   0.500     0.330    1.180    7.77s
        0.40   0.550     0.380    1.079    7.77s     ← 기본

      코너 여유가 0.360 → 0.550m (+53%) 인데 랩은 0.16s 만 든다.
      비교: 균일한 --safety 를 0.30→0.45 로 올리면 0.400m 에 0.17s —
      **같은 비용에 훨씬 적게 번다.** 손해가 코너에만 국한되기 때문이다.

    ※ 그래도 아직 부족하다 — 실측 코너 추종오차가 0.39~0.55m 라 여유 0.38m 와
      거의 같다. 추종 오차를 0.2m 대로 줄인 뒤에야 이 라인으로 갈아탈 수 있다.
      그전까지는 gen_waypoints_kappa 라인(코너 여유 0.579m)이 안전하다.
    """
    if apex_extra <= 0.0:
        return safety_m, safety_m
    k = signed_curvature(line)
    f = np.clip(np.abs(k) / max(apex_kappa, 1e-6), 0.0, 1.0)
    extra = apex_extra * f
    #   k > 0  오른쪽으로 돎 → 안쪽이 오른쪽 → 오른쪽 여유를 늘린다
    return (safety_m + np.where(k > 0, extra, 0.0),
            safety_m + np.where(k < 0, extra, 0.0))


def track_widths(line: np.ndarray, outer: np.ndarray, inner: np.ndarray,
                 safety_m, min_w: float = 0.05, safety_l=None):
    """임의의 라인에서 좌우 트랙 폭을 **경계에서 다시 잰다**.

    반복 최적화가 매 스텝 라인을 옮기므로, 폭을 α 로 더하고 빼는 근사만 쓰면
    오차가 누적돼 라인이 조용히 트랙 밖으로 샌다. 매번 실측하면 그 경로가 없다.
    right = outer(바깥 경계), left = inner — 중심선이 CCW 라 바깥이 오른쪽이다.

    safety_m 은 스칼라도 배열도 된다. 배열이면 점마다 다른 여유 (apex_safety).
    safety_l 을 주면 좌우를 따로 쓴다.
    """
    d_o, _ = cKDTree(outer).query(line)
    d_i, _ = cKDTree(inner).query(line)
    s_r = safety_m
    s_l = safety_m if safety_l is None else safety_l
    return (np.maximum(d_o - s_r, min_w),
            np.maximum(d_i - s_l, min_w))


def optimize_raceline(reftrack, outer, inner, *, w_veh, r_min, stepsize,
                      out_step, safety_m, iters=12, step_limit=0.15,
                      wall_clear=0.06, tol=0.995,
                      apex_extra=0.0, apex_kappa=None):
    """재선형화 반복 최소곡률 최적화 (TUMFTM iqp 에 해당).

    ★ 한 번만 푸는 게 왜 안 되나 ─────────────────────────────────────────
    E 는 "노드를 α 만큼 밀어도 **호길이는 그대로**"라는 1차 전개다. 한 방에 풀면
    |α| 가 격자 간격(0.5m)을 넘어(실측 0.815m) 그 가정이 완전히 깨진다.
    그때 QP 는 자기 모델에서 R=0.900 을 만족한다고 보고하지만, 그 노드를 정확히
    통과시킨 실제 곡선은 R=0.117 이었다 — 8배 차이. 모델과 기하가 따로 논다.

    그래서 한 번에 step_limit 만큼만 움직이고, 옮긴 라인을 **새 기준선으로 삼아**
    스플라인·법선·폭을 다시 계산한 뒤 또 푼다. 매 스텝 α 가 작으니 선형화가 유효하다.
    """
    ref = reftrack.copy()
    best = None
    for it in range(iters):
        ru = interp_track(ref, stepsize)
        path_cl = np.vstack([ru[:, :2], ru[0, :2]])
        _, _, M_spl, nv = calc_splines(path_cl)

        alpha = opt_min_curv(ru, nv, M_spl, w_veh=w_veh, r_min=r_min,
                             alpha_max=step_limit, quiet=True)
        if not np.all(np.isfinite(alpha)):
            print(f'  iter {it:2d}: QP 해가 유효하지 않다 — 중단')
            break

        nodes = ru[:, :2] + alpha[:, None] * nv
        rl, el, kap, total = eval_spline_dense(nodes, out_step)
        r_act = 1.0 / float(np.abs(kap).max())

        # 트랙 이탈 검사 — 경계까지의 **생거리**로 잰다.
        # ★ safety_m 을 빼면 안 된다: QP 의 dev 는 이미 (폭 − safety − w_veh/2) 라
        #   여기서 또 빼면 이중 차감이 된다. 실제로 그렇게 했더니 중심선조차
        #   탈락해 "해를 못 찾았다"가 나왔다 (이 트랙은 최협 반폭이 0.27m 다).
        #   합격 조건은 물리 조건 하나 — 차가 들어가는가.
        d_o, d_i = track_widths(rl, outer, inner, safety_m=0.0, min_w=0.0)
        clear = float(min(d_o.min(), d_i.min())) - w_veh / 2.0

        print(f'  iter {it:2d}: |α|max={np.abs(alpha).max():.3f}  '
              f'R_act={r_act:.3f} m  길이={total:.2f}m  차폭여유={clear:+.3f}m')

        # ★ 곡률만 보고 고르면 안 된다. 반복이 진행될수록 라인은 코너를 더
        #   파고들어 R 은 좋아지지만 **벽에 붙는다.** 이 트랙은 중심선조차 차폭
        #   여유가 0.112m 뿐이라 여유가 곧 사라진다. 실제로 완주한 라인
        #   (halfhot_forza_opt_wp)의 여유가 0.060m 였으므로 그 이상을 요구한다.
        if clear >= wall_clear and (best is None or r_act > best[0]):
            best = (r_act, nodes.copy(), rl, el, kap, total, clear)

        # 다음 반복의 기준선 = 방금 옮긴 라인 (폭은 경계에서 다시 잰다)
        #   ★ 여유는 **방금 나온 라인의 곡률**로 다시 계산한다. 반복이 진행되며
        #     코너가 어디인지가 바뀌므로, 처음 중심선 곡률로 고정하면 어긋난다.
        s_r, s_l = apex_safety(nodes, safety_m, apex_extra,
                               apex_kappa if apex_kappa else 1.0 / max(r_min, 1e-6))
        w_r2, w_l2 = track_widths(nodes, outer, inner, s_r, safety_l=s_l)
        ref = np.column_stack([nodes, w_r2, w_l2])

        if np.abs(alpha).max() < 1e-3:
            print(f'  iter {it:2d}: α 수렴 — 종료')
            break

    if best is None:
        # ★ 안내를 나누는 이유 (2026-08-14)
        #   원인이 둘인데 처방이 정반대다. 예전엔 무조건 "--safety 를 올려라" 라고
        #   해서 헛다리를 짚게 만들었다 — 0815_clean 에서 safety 0.15/0.20/0.25/0.35
        #   를 다 시도해도 전부 실패했다. 중심선 자체가 차폭보다 좁은 곳을 지나면
        #   QP 가 할 수 있는 일이 없기 때문이다.
        cl_room = float(min(track_widths(reftrack[:, :2], outer, inner,
                                         safety_m=0.0, min_w=0.0)[0].min(),
                            track_widths(reftrack[:, :2], outer, inner,
                                         safety_m=0.0, min_w=0.0)[1].min()))
        if cl_room < w_veh / 2.0 + wall_clear:
            raise RuntimeError(
                f'optimize_raceline: 벽 여유 {wall_clear:.3f}m 를 만족하는 해가 없다.\n'
                f'  ★ 원인은 QP 가 아니라 **중심선**이다 — 중심선의 최소 벽 여유가\n'
                f'    {cl_room:.3f}m 로, 필요한 {w_veh/2.0 + wall_clear:.3f}m'
                f' (차 반폭 {w_veh/2.0:.3f} + 여유 {wall_clear:.3f}) 에 못 미친다.\n'
                f'    중심선조차 못 지나는 목이 있으면 --safety 를 어떻게 줘도 안 된다.\n'
                f'  → 맵을 확인할 것. 흔한 원인 두 가지:\n'
                f'     · 벽에 틈이 있어 라이다가 새어 나갔다 → --gapfill 을 올린다\n'
                f'       (기본 0.05m. 틈이 크면 0.10~0.15)\n'
                f'     · 그 자리에 실제로 뭔가 놓여 있었다 → 치우고 다시 맵핑\n'
                f'  → 급하면 --wallclear 를 낮춰 통과시킬 수는 있으나, 그만큼 차가\n'
                f'    벽에 가까이 붙는다는 뜻이다.')
        raise RuntimeError(
            f'optimize_raceline: 벽 여유 {wall_clear:.3f}m 를 만족하는 해가 없다.\n'
            f'  중심선은 {cl_room:.3f}m 로 충분하니 **QP 가 코너를 너무 파고든 것**이다.\n'
            f'  → **--safety 를 올릴 것** (예: 0.35). safety 는 QP 가 쓸 수 있는\n'
            f'     폭을 좁혀 라인을 중앙에 붙인다 = 벽 여유가 늘어난다.\n'
            f'     직관과 반대라 2026-08-10 에 이 메시지가 "줄이라" 고 잘못\n'
            f'     안내한 적이 있다. fun_clean 실측: safety 0.15 → 0.35 으로\n'
            f'     여유 0.074→0.208m, R_min 1.150→1.180m, 랩 11.5→11.4s (전부 개선).\n'
            f'  → 그래도 안 되면 트랙이 정말 좁은 것이다. --wallclear 를 낮출 것.')
    return best


# ══════════════════════════════════════════════════════════════════════════════
# 6. 속도 프로파일 (전방/역방향 스윕)
# ══════════════════════════════════════════════════════════════════════════════
def calc_vel_profile(kappa:    np.ndarray,
                     el:       np.ndarray,
                     mu:       float,
                     v_max:    float,
                     v_min:    float,
                     a_lon_acc: float,
                     a_lon_brk: float) -> np.ndarray:
    """
    횡방향 마찰 한계 + 전/후방 종방향 가속도로 속도 프로파일 계산.
    """
    n = len(kappa)
    g = 9.81
    a_lat_max = mu * g

    # 횡가속도 한계: v = sqrt(a_lat_max / |κ|)
    v_lat = np.where(kappa > 1e-4,
                     np.sqrt(a_lat_max / kappa),
                     v_max)
    v = np.minimum(np.full(n, v_max), v_lat)

    # 역방향 스윕 (제동 한계)
    for _ in range(3):
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v_lim = math.sqrt(v[j]**2 + 2.0 * a_lon_brk * el[i])
            if v[i] > v_lim:
                v[i] = v_lim

    # 전방향 스윕 (가속 한계)
    for _ in range(3):
        for i in range(n):
            j = (i + 1) % n
            v_lim = math.sqrt(v[i]**2 + 2.0 * a_lon_acc * el[i])
            if v[j] > v_lim:
                v[j] = v_lim

    v = np.clip(v, v_min, v_max)
    lap_t = float(np.sum(el / np.maximum(v, 0.01)))
    print(f'  속도: {v.min():.2f}~{v.max():.2f} m/s  예상 랩타임={lap_t:.1f}s')
    above = (v > 7.319).mean() * 100
    if above > 0:
        print(f'  동역학 모델(v>7.319m/s) 구간: {above:.1f}%  → 슬립 발생 가능')
    return v


# ══════════════════════════════════════════════════════════════════════════════
# 7. 오버레이 PNG 저장
# ══════════════════════════════════════════════════════════════════════════════
def _overtake_zone_for(map_name, raceline, args, gap_fill):
    """오버레이에 그릴 추월 가능 구간. 실패하면 None (그림만 빠진다).

    ★ 주행(frenet_planner)과 **같은 함수**를 쓴다 (obstacle_avoid.overtake_zone).
      두 벌로 두면 그림과 실제가 갈리고, 그건 "그림은 되는데 차는 안 피한다"
      로 나타나 디버깅이 샌다.
    """
    try:
        from race_stack.obstacle_avoid import overtake_zone
        import yaml as _y
        cfg = _y.safe_load(open(os.path.join(MAPS_DIR, map_name + '.yaml')))
        res = float(cfg['resolution']); ox, oy = cfg['origin'][:2]
        img = None
        for ext in ('.pgm', '.png'):
            q = os.path.join(MAPS_DIR, map_name + ext)
            if os.path.exists(q):
                img = np.array(Image.open(q).convert('L')); break
        if img is None:
            return None
        h, w = img.shape
        free_b = img > 128
        if gap_fill > 0.0:                       # 주행 쪽과 같은 전처리
            r_px = max(1, int(round(gap_fill / res)))
            yy, xx = np.ogrid[-r_px:r_px + 1, -r_px:r_px + 1]
            kern = (yy * yy + xx * xx) <= r_px * r_px
            free_b = ~ndimage.binary_closing(~free_b, structure=kern, border_value=1)
        dist = cv2.distanceTransform(free_b.astype(np.uint8), cv2.DIST_L2, 5) * res
        jj = np.clip(((raceline[:, 0] - ox) / res).astype(int), 0, w - 1)
        ii = np.clip((h - (raceline[:, 1] - oy) / res).astype(int), 0, h - 1)
        return overtake_zone(raceline, dist[ii, jj],
                             args.ot_width, args.ot_exit_pad, args.ot_min_len)
    except Exception as e:
        print(f'  추월구간 계산 생략: {e}')
        return None


def save_overlay(map_name, raceline, v, wp_path, ot=None):
    try:
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.colors import Normalize
        from matplotlib.collections import LineCollection

        yaml_path = os.path.join(MAPS_DIR, map_name + '.yaml')
        import yaml
        with open(yaml_path) as f:
            cfg = yaml.safe_load(f)
        res    = float(cfg['resolution'])
        origin = np.array(cfg['origin'][:2])

        img_path = None
        for ext in ['.pgm', '.png']:
            p = os.path.join(MAPS_DIR, map_name + ext)
            if os.path.exists(p):
                img_path = p; break
        img = np.array(Image.open(img_path).convert('L'))
        h_img = img.shape[0]

        px = ((raceline[:, 0] - origin[0]) / res).astype(int)
        py = (h_img - (raceline[:, 1] - origin[1]) / res).astype(int)
        px = np.append(px, px[0]); py = np.append(py, py[0])
        v_ext = np.append(v, v[0])

        fig = Figure(figsize=(9, 9))
        FigureCanvasAgg(fig)
        ax = fig.add_subplot(111)
        ax.imshow(img, cmap='gray', origin='upper')
        ax.set_aspect('equal'); ax.axis('off')

        pts  = np.array([px, py]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc   = LineCollection(segs, cmap='RdYlGn',
                              norm=Normalize(v.min(), v.max()),
                              linewidth=2.5, zorder=5)
        lc.set_array(v_ext)
        ax.add_collection(lc)
        ax.plot(px[0], py[0], 'b*', ms=15, zorder=6)

        # ── 추월 가능 구간 (1대1) ────────────────────────────────────
        #   주행 중 로그에만 뜨면 "트랙 어디가 추월 구간인지" 를 눈으로 못 본다.
        #   라인을 만들 때 같이 그려두면 그 자리에서 검수할 수 있다.
        #   계산은 obstacle_avoid.overtake_zone — **주행과 같은 함수**다.
        ot_txt = ''
        if ot is not None and ot.any():
            oz = np.append(ot, ot[0])
            seg_ot = segs[oz[:-1] & oz[1:]]
            if len(seg_ot):
                ax.add_collection(LineCollection(
                    seg_ot, colors='#00bfff', linewidth=7.0, zorder=4, alpha=0.55))
                ot_txt = f'   |  overtake zone {100*ot.mean():.0f}%'
        elif ot is not None:
            ot_txt = '   |  overtake zone 0%'

        fig.colorbar(lc, ax=ax, label='speed (m/s)', fraction=0.03, pad=0.02)
        ax.set_title(f'{map_name}  —  {len(raceline)}pts '
                     f'({v.min():.1f}~{v.max():.1f} m/s){ot_txt}', fontsize=10)

        out = os.path.splitext(wp_path)[0] + '.png'
        fig.tight_layout()
        fig.savefig(out, dpi=130, bbox_inches='tight')
        print(f'  오버레이 PNG: {out}')
    except Exception as e:
        print(f'  오버레이 저장 실패: {e}')


# ══════════════════════════════════════════════════════════════════════════════
# 8. GUI 드로잉 모드
# ══════════════════════════════════════════════════════════════════════════════
def gui_draw_raceline(map_name: str, args) -> tuple | None:
    """
    맵 위에서 클릭으로 경로를 그리는 대화형 GUI.
    좌클릭: 점 추가  |  우클릭/u: 언두  |  d: 초기화  |  Enter: 확정  |  q: 취소
    반환: (raceline [M,2], el [M], kappa [M], v [M])  또는  None (취소)
    """
    import matplotlib
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from scipy.interpolate import splprep, splev
    import yaml

    # ── 맵 로드 ──
    yaml_path = os.path.join(MAPS_DIR, map_name + '.yaml')
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    res    = float(cfg['resolution'])
    origin = np.array(cfg['origin'][:2])

    img_path = None
    for ext in ['.pgm', '.png']:
        p = os.path.join(MAPS_DIR, map_name + ext)
        if os.path.exists(p):
            img_path = p; break
    img = np.array(Image.open(img_path).convert('L'))
    h_img, w_img = img.shape
    xmin, xmax = origin[0], origin[0] + w_img * res
    ymin, ymax = origin[1], origin[1] + h_img * res

    # ── 트랙 경계 ──
    reftrack = extract_reftrack(map_name, safety_m=args.safety, gap_fill=args.gapfill)
    cl = reftrack[:, :2]
    path_cl = np.vstack([cl, cl[0]])
    _, _, _, nv = calc_splines(path_cl)
    bnd_r = np.vstack([cl + reftrack[:, 2:3] * nv, cl[0] + reftrack[0, 2] * nv[0]])
    bnd_l = np.vstack([cl - reftrack[:, 3:4] * nv, cl[0] - reftrack[0, 3] * nv[0]])

    # ── 기존 웨이포인트 후보 로드 ──
    candidates = {}
    if os.path.isdir(WP_DIR):
        for fname in sorted(os.listdir(WP_DIR)):
            if fname.startswith(map_name) and fname.endswith('.csv'):
                try:
                    pts = []
                    with open(os.path.join(WP_DIR, fname)) as f:
                        for row in csv.DictReader(f):
                            pts.append([float(row['x']), float(row['y'])])
                    candidates[fname[:-4]] = np.array(pts)
                except Exception:
                    pass

    # ── 그림 ──
    fig, ax = plt.subplots(figsize=(13, 10))
    ax.imshow(img, origin='lower', extent=[xmin, xmax, ymin, ymax],
              cmap='gray', alpha=0.75)
    ax.set_aspect('equal')

    ax.plot(bnd_r[:, 0], bnd_r[:, 1], color='orange', lw=1.2, alpha=0.7, label='right bound')
    ax.plot(bnd_l[:, 0], bnd_l[:, 1], color='cyan',   lw=1.2, alpha=0.7, label='left bound')
    cl_cl = np.vstack([cl, cl[0]])
    ax.plot(cl_cl[:, 0], cl_cl[:, 1], 'b--', lw=0.8, alpha=0.35, label='centerline')

    pal = ['red', 'lime', 'magenta', 'yellow', 'deepskyblue']
    for i, (cname, cpts) in enumerate(candidates.items()):
        short = cname.replace(map_name + '_', '')
        ax.plot(cpts[:, 0], cpts[:, 1], '-', color=pal[i % len(pal)],
                lw=2.0, alpha=0.8, label=short)

    ax.legend(loc='upper right', fontsize=7, framealpha=0.6)
    ax.set_title(
        f'{map_name}  —  L-click: add  |  R-click/u: undo  |  '
        f'd: clear  |  Enter: confirm  |  q: cancel',
        fontsize=9)

    # 현재 그린 경로 표시
    state = dict(pts=[], artists=[])

    def _redraw():
        for a in state['artists']:
            try: a.remove()
            except Exception: pass
        state['artists'].clear()
        pts = state['pts']
        if not pts:
            fig.canvas.draw_idle(); return
        arr = np.array(pts)
        state['artists'].append(
            ax.scatter(arr[:, 0], arr[:, 1], c='white', edgecolors='red',
                       s=28, zorder=12, linewidths=1.2))
        if len(pts) >= 2:
            ln, = ax.plot(arr[:, 0], arr[:, 1], 'r--', lw=1, alpha=0.45, zorder=10)
            state['artists'].append(ln)
        if len(pts) >= 4:
            try:
                tck, _ = splprep([arr[:, 0], arr[:, 1]], s=0, per=False, k=3)
                u_f = np.linspace(0, 1, max(300, len(pts) * 12))
                sx, sy = splev(u_f, tck)
                prev, = ax.plot(sx, sy, 'r-', lw=2.5, zorder=11)
                state['artists'].append(prev)
            except Exception:
                pass
        fig.canvas.draw_idle()

    def on_click(ev):
        if ev.inaxes != ax: return
        if ev.button == 1:
            state['pts'].append([ev.xdata, ev.ydata])
        elif ev.button == 3 and state['pts']:
            state['pts'].pop()
        _redraw()

    def on_key(ev):
        if ev.key == 'u' and state['pts']:
            state['pts'].pop()
        elif ev.key == 'd':
            state['pts'].clear()
        elif ev.key in ('enter', 'return'):
            plt.close(fig)
        elif ev.key == 'q':
            state['pts'].clear()
            plt.close(fig)
        _redraw()

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.tight_layout()
    plt.show()

    pts = state['pts']
    if len(pts) < 4:
        print('  not enough points (need at least 4). cancelled.')
        return None

    # ── 스플라인 피팅 ──
    print(f'\n  {len(pts)}점 → 주기 스플라인 피팅...')
    arr = np.array(pts)
    tck, _ = splprep([arr[:, 0], arr[:, 1]], s=0, per=True, k=3)

    # 총 길이 추정
    u_est = np.linspace(0, 1, 600, endpoint=False)
    xe, ye = splev(u_est, tck)
    pts_est = np.column_stack([xe, ye])
    total = float(np.linalg.norm(
        np.diff(np.vstack([pts_est, pts_est[0]]), axis=0), axis=1).sum())

    M   = max(3, int(total / args.step))
    u_u = np.linspace(0, 1, M, endpoint=False)
    xi, yi = splev(u_u, tck)
    raceline = np.column_stack([xi, yi])

    el = np.linalg.norm(np.diff(np.vstack([raceline, raceline[0]]), axis=0), axis=1)
    dx, dy   = splev(u_u, tck, der=1)
    ddx, ddy = splev(u_u, tck, der=2)
    kappa = np.abs(dx * ddy - dy * ddx) / np.maximum((dx**2 + dy**2) ** 1.5, 1e-9)

    r_act = 1.0 / float(kappa.max())
    print(f'  레이싱라인: {M}pts  길이={total:.1f}m  |κ|_max={kappa.max():.4f}  R_min={r_act:.3f}m')
    if r_act < args.rmin * 0.995:
        print(f'  ⚠ R_min {r_act:.3f}m < 목표 {args.rmin}m — 헤어핀을 더 넓게 그려 보세요')

    v = calc_vel_profile(kappa, el,
                         mu=args.mu, v_max=args.vmax, v_min=args.vmin,
                         a_lon_acc=args.acc, a_lon_brk=args.brk)
    return raceline, el, kappa, v


# ══════════════════════════════════════════════════════════════════════════════
# 9. main
# ══════════════════════════════════════════════════════════════════════════════
def main():
    p = argparse.ArgumentParser(description='TUMFTM 최소 곡률 레이싱라인 생성기')
    p.add_argument('--map',    default=None,  help='맵 이름 (생략 시 대화형 선택)')
    # ★ CSV 속도 열은 solve_profile 에서 **상한으로만** 쓰인다. 실제 속도는
    #   limits.grip_util × μ_eff 가 정한다. 그래서 여기를 차량 물리 한계까지
    #   열어두고 grip_util 로 낮춰가며 올리는 게 맞다 — 낮게 잡아두면 CSV 가
    #   먼저 잘라서 grip_util 을 올려도 안 빨라진다.
    #   12.0 은 VESC 소프트 상한(speed_max 50000 erpm ÷ 4203 = 11.9㎧)에 맞춘 값.
    p.add_argument('--vmax',   type=float, default=12.0, help='최대 속도 (m/s)')
    p.add_argument('--vmin',   type=float, default=1.0,  help='최소 속도 (m/s)')
    # 2026-08-09 실측 추정 0.40 (구 1.0489). 과대평가하면 레이싱라인 속도가
# 실제 그립을 넘어 코너에서 밀려난다 — 안전 방향으로 낮춘다.
    # ★ 이 값은 **CSV 속도 열**만 정한다. 라인 모양(QP)과는 무관하다.
    #   그 속도는 주행 중 solve_profile 에서 **상한으로만** 쓰이고, 실제 속도는
    #   limits.grip_util × (map.mu × pac_d) 가 정한다.
    #   → 그러므로 넉넉히 잡는 게 맞다. 낮게 잡으면 나중에 그립을 올려도
    #     CSV 가 먼저 잘라 안 빨라진다 (fun_clean 실측: μ_eff 0.56 상당에서
    #     --mu 0.4 로 만든 CSV 는 랩 12.2s, 0.6 이상은 11.1s — 1.1초 손해).
    #   0.9 는 우레탄 노면까지 열어둔 값. 안전은 grip_util 로 잡는다.
    p.add_argument('--mu',     type=float, default=0.90,
                   help='CSV 속도 열 계산용 마찰계수. 라인 모양과 무관하고 주행 중엔 '
                        '상한으로만 쓰인다 — 넉넉히 줄 것 (실제 속도는 grip_util 이 정함)')
    p.add_argument('--wveh',   type=float, default=0.31,  help='차량 폭 (m)')
    p.add_argument('--safety', type=float, default=0.35,
                   help='벽 안전 마진 (m). QP 가 쓸 수 있는 트랙 폭을 이만큼 좁힌다 '
                        '— 올리면 라인이 중앙에 붙어 벽 여유가 늘어난다')
    p.add_argument('--acc',    type=float, default=5.0,  help='가속도 (m/s²)')
    p.add_argument('--brk',    type=float, default=8.0,  help='제동 (m/s²)')
    p.add_argument('--step',   type=float, default=0.1,  help='웨이포인트 간격 (m)')
    p.add_argument('--rmin',   type=float, default=1.0,
                   help='최소 회전반경 (m). 차량 한계 0.856m 보다 크게 —'
                        ' 여유가 없으면 AMCL 오차·조향 지연을 흡수 못 한다')
    p.add_argument('--gridstep', type=float, default=0.5,
                   help='QP 격자 간격 (m). |α| 가 이 값에 근접하면 선형화가 깨진다')
    p.add_argument('--steplimit', type=float, default=0.15,
                   help='반복 1회당 최대 횡이동 (m) — 신뢰영역')
    p.add_argument('--iters',  type=int, default=12, help='재선형화 반복 횟수')
    p.add_argument('--gapfill', type=float, default=0.05,
                   help='트랙 벽의 틈을 이만큼까지 메운다 [m] (기본 0.05). '
                        '틈으로 라이다가 새면 "가장 큰 자유공간"이 샌 통로까지 '
                        '포함해 중심선이 벽에 붙는다. 0=끔')
    p.add_argument('--ot-width', type=float, default=1.10, dest='ot_width',
                   help='추월 가능 구간 판정 폭 [m] — 오버레이 표시용. '
                        '주행 값은 race_script.yaml frenet.ot_width 다')
    p.add_argument('--ot-exit-pad', type=float, default=2.50, dest='ot_exit_pad')
    p.add_argument('--ot-min-len',  type=float, default=2.00, dest='ot_min_len')
    p.add_argument('--apex-extra', type=float, default=0.40, dest='apex_extra',
                   help='코너 안쪽에만 더 줄 여유 [m]. 0=끔 (기본 0.40). '
                        '추종 오차가 코너에 몰리므로(실측 직선 0.16 vs 코너 0.39~0.55m) '
                        '균일한 --safety 보다 여기에 주는 편이 훨씬 싸다 — '
                        '0814-1_clean 실측: 코너 여유 0.360→0.550m 에 랩 +0.16s')
    p.add_argument('--apex-kappa', type=float, default=0.0, dest='apex_kappa',
                   help='이 곡률[1/m]에서 --apex-extra 를 다 준다 (0=자동: 1/rmin)')
    p.add_argument('--wallclear', type=float, default=0.18,
                   help='벽까지 최소 여유 (m, 차폭 제외). ★ 실측 추종오차보다 커야 '
                        '한다 — 2026-08-09 CTE p95 가 η0.5 에서 0.217m, η1.0 에서 '
                        '0.320m 였다. 옛 기본값 0.06 은 코너 정점에서 확실히 걸린다')
    p.add_argument('--gui',   action='store_true',       help='클릭 GUI 드로잉 모드')
    p.add_argument('--out',   default=None,              help='출력 웨이포인트 이름 (기본: {map}_map_wp 또는 {map}_draw_wp)')
    args = p.parse_args()

    # ── 맵 선택 ──
    if args.map:
        map_name = args.map
    else:
        # ★ .pgm 이 있는 것만 맵이다. maps/ 에는 미리보기 PNG 도 같이 사는데
        #   (clean_map_auto 의 <맵>_clean_preview.png), 그걸 목록에 올리면
        #   고를 수 있는 것처럼 보이고 고르면 죽는다.
        maps = sorted(f[:-4] for f in os.listdir(MAPS_DIR) if f.endswith('.pgm'))
        print('\n사용 가능한 맵:')
        for i, m in enumerate(maps, 1):
            print(f'  {i}. {m}')
        val = input('맵 이름 또는 번호: ').strip()
        map_name = maps[int(val) - 1] if val.isdigit() else val

    os.makedirs(WP_DIR, exist_ok=True)

    # ══════════════════════════════════════════════════════════════════════
    # GUI 드로잉 모드
    # ══════════════════════════════════════════════════════════════════════
    if args.gui:
        out_name = args.out or f'{map_name}_draw_wp'
        wp_path  = os.path.join(WP_DIR, out_name + '.csv')

        print(f'\n[GUI] 맵: {map_name}  출력: {out_name}')
        result = gui_draw_raceline(map_name, args)
        if result is None:
            return
        raceline, el, kappa, v = result

        # ★ 손으로 그린 경로도 차량 기하를 만족해야 한다. TUMFTM 경로와 같은 검사다 —
        #   여기 빠뜨리면 "그렸으니 되겠지" 하고 못 도는 라인이 그대로 나간다.
        r_act = 1.0 / float(np.abs(kappa).max())
        print(f'\nR_min = {r_act:.3f} m  (차량 한계 {R_CAR_MIN:.3f} m)', end='')
        n_bad = int(np.sum(kappa > 1.0 / R_CAR_MIN))
        if n_bad:
            print(f'\n  ★★ 차량이 못 도는 점 {n_bad}/{len(kappa)}개 — 헤어핀을 더 크게 그릴 것')
        else:
            print(f'  여유 {r_act - R_CAR_MIN:+.3f} m ✓')

        print(f'속도: {v.min():.2f}~{v.max():.2f} m/s')

        # 저장
        with open(wp_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y', 'speed'])
            for (x, y), vi in zip(raceline, v):
                w.writerow([f'{x:.4f}', f'{y:.4f}', f'{vi:.4f}'])
        print(f'  저장: {len(raceline)}pts → {wp_path}')
        save_overlay(map_name, raceline, v, wp_path,
                     _overtake_zone_for(map_name, raceline, args, args.gapfill))
        print(f'\n완료!  race_script.yaml → waypoints: {out_name}')
        return

    # ══════════════════════════════════════════════════════════════════════
    # TUMFTM 최적화 모드
    # ══════════════════════════════════════════════════════════════════════
    out_name = args.out or f'{map_name}_map_wp'
    wp_path  = os.path.join(WP_DIR, out_name + '.csv')

    print(f'\n맵: {map_name}  vmax={args.vmax}  wveh={args.wveh}  '
          f'safety={args.safety}  apex_extra={args.apex_extra}  mu={args.mu}')

    print('\n[1] 트랙 경계 추출...')
    reftrack, outer_w, inner_w = extract_reftrack(
        map_name, safety_m=args.safety, return_bounds=True, gap_fill=args.gapfill)

    print(f'\n[2~4] 재선형화 반복 최소곡률 최적화 '
          f'(격자 {args.gridstep}m, 스텝한계 {args.steplimit}m, 최대 {args.iters}회)...')
    r_act, nodes, raceline, el, kappa, total, wclear = optimize_raceline(
        reftrack, outer_w, inner_w,
        w_veh=args.wveh, r_min=args.rmin,
        stepsize=args.gridstep, out_step=args.step,
        safety_m=args.safety, iters=args.iters, step_limit=args.steplimit,
        apex_extra=args.apex_extra,
        apex_kappa=(args.apex_kappa if args.apex_kappa > 0 else None),
        wall_clear=args.wallclear)

    print(f'\n[5] 결과: {len(raceline)}pts  길이={total:.2f}m  '
          f'|κ|_max={1/r_act:.4f} 1/m')
    print(f'  최종 R_min = {r_act:.3f} m  (목표 {args.rmin} m)', end='')
    if r_act < args.rmin * 0.995:
        print('  ⚠ 목표 미달 — 트랙 여유 부족 또는 반복 한계')
    else:
        print('  ✓')
    print(f'  벽 여유 (차폭 제외) {wclear:+.3f} m  '
          f'(요구 {args.wallclear:.3f} / 실주행 검증된 라인 0.060)')

    # ★ 차량 기하와 직접 대조한다. 라인이 아무리 빨라도 못 도는 건 소용없다.
    #   R_min(차량) = wheelbase / tan((servo_min − offset)/(−gain))  ← config/vesc.yaml
    #   2026-08-09 기준 좌회전 0.856m (servo_min 0.12, offset 0.3954, gain −0.761).
    #   실제로 08/09 주행에서 R 0.91m 라인의 헤어핀에서 라인 밖으로 0.39m 밀렸다.
    R_CAR = R_CAR_MIN
    n_bad = int(np.sum(kappa > 1.0 / R_CAR))
    if n_bad:
        print(f'  ★★ 차량 최소회전반경({R_CAR:.3f}m)을 넘는 점이 {n_bad}/{len(kappa)}개 '
              f'— 이 라인은 못 돈다. --rmin 을 올려 다시 생성할 것')
    else:
        print(f'  차량 최소회전반경 {R_CAR:.3f}m 대비 여유 {r_act - R_CAR:+.3f}m ✓')

    print('\n[6] 속도 프로파일...')
    v = calc_vel_profile(kappa, el,
                         mu=args.mu, v_max=args.vmax, v_min=args.vmin,
                         a_lon_acc=args.acc, a_lon_brk=args.brk)

    print('\n[7] 저장...')
    with open(wp_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'speed'])
        for (x, y), vi in zip(raceline, v):
            w.writerow([f'{x:.4f}', f'{y:.4f}', f'{vi:.4f}'])
    print(f'  웨이포인트: {len(raceline)}pts → {wp_path}')

    ot = _overtake_zone_for(map_name, raceline, args, args.gapfill)
    if ot is not None:
        print(f'  추월 가능 구간: {100*ot.mean():.0f}% '
              f'(폭≥{args.ot_width:.2f}m, 끝에서 {args.ot_exit_pad:.1f}m 잘라냄)')
    save_overlay(map_name, raceline, v, wp_path, ot)

    print('\n완료! 다음 단계:')
    print(f'  race_script.yaml → waypoints: {out_name}')
    print(f'  SPEED_SCALE 1.0 으로 시작 (웨이포인트가 이미 고속 최적화됨)')


if __name__ == '__main__':
    main()
