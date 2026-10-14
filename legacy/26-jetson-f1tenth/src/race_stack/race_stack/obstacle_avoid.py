#!/usr/bin/env python3
"""obstacle_avoid — Frenet 장애물 회피 로직 (컨트롤러 공용).

stanley_controller 에서 추출한 공용 회피 로직. Frenet 후보 비용에는
통과속도(v_max) 항을 넣을 수 있다 (fr_w_spd, 기본 0 이면 예전과 동일).

무엇을 하나
──────────
레이싱라인을 기준선으로 삼아 **점별 법선 방향으로 d 만큼 민 후보들**을 만들고,
장애물 여유·트랙 이탈·복귀력·변화량으로 비용을 매겨 최적 d 하나를 고른다.
경로를 새로 만드는 게 아니라 **기준선의 횡오프셋 스칼라 d** 를 정하는 방식이라,
경로 추종 컨트롤러면 무엇이든 소비할 수 있다 (Stanley·MAP 모두).

★ 컨트롤러마다 d 에 수렴하는 속도가 다르다 — 이게 이식 시 유일한 함정이다.
  Stanley 는 δ = ψ_e + atan(k·e/v) 로 전방주시 지연이 없어 즉시 반응한다.
  MAP/L1 은 2차계(ζ=0.707, ωn=√2·v/L1)라 오버슈트는 4.3% 로 작지만
  **상승거리가 1.8·L1/√2 ≈ 1.15m (L1=0.9m)** 나 된다.
  그래서 회피 호라이즌은 **2 × L1 이상**이어야 한다. Stanley 기준으로 맞춘
  obs_horizon_min(0.8m)을 MAP 에 그대로 쓰면 다 피하기 전에 장애물에 닿는다.
  → race_script.yaml frenet.per_controller 에서 컨트롤러별로 따로 준다.

쓰는 법
───────
이 믹스인을 상속하고 아래 속성을 채워두면 된다. (Stanley 가 이미 채우는 것들이고,
frenet_planner 노드도 같은 이름으로 채운다.)
"""

import math
import os

import cv2
import numpy as np
import scipy.spatial.distance
import yaml
from PIL import Image

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from race_stack.speed_profile import GRAVITY, solve_profile


def _load_map(map_yaml_path: str):
    """Load map image + YAML → (map_array [H,W,3], (h,w,res,ox,oy))."""
    cfg  = yaml.safe_load(open(map_yaml_path))
    stem = os.path.splitext(map_yaml_path)[0]

    # ★ YAML 의 image: 를 그대로 따른다 (map_server 와 동일한 규약).
    #   예전엔 확장자를 ('.png', '.pgm') 순으로 추측했는데, maps/ 에는 같은 이름의
    #   미리보기 PNG 가 함께 있다. track_2nd_clean.png 는 1300x900 인데 실제 맵
    #   track_2nd_clean.pgm 은 170x154 다. PNG 를 집으면 map_meta 의 h/w 가
    #   6.7배 커져 _world_to_img 좌표가 통째로 어긋나고, 경계 폴리곤(픽셀 좌표)
    #   밖으로 떨어져 장애물이 단 하나도 감지되지 않는다. (2026-07-21 실차 확인)
    img_p = None
    img_name = cfg.get('image')
    if img_name:
        cand = os.path.join(os.path.dirname(os.path.abspath(map_yaml_path)),
                            img_name)
        if os.path.exists(cand):
            img_p = cand
    if img_p is None:                      # image: 키가 없는 옛 맵 폴백
        for ext in ('.pgm', '.png'):       # 순서도 pgm 우선으로
            if os.path.exists(stem + ext):
                img_p = stem + ext
                break
    if img_p is None:
        raise FileNotFoundError(f'no image found for {stem}')

    img = Image.open(img_p).convert('L').transpose(Image.FLIP_TOP_BOTTOM)
    arr = np.asarray(img, dtype=np.float64)
    arr = np.where(arr > 128, 255.0, 0.0)
    h, w = arr.shape
    res  = float(cfg['resolution'])
    ox   = float(cfg['origin'][0])
    oy   = float(cfg['origin'][1])

    xs, ys = np.meshgrid(np.arange(w) * res + ox,
                         np.arange(h) * res + oy)
    vs = np.where(arr > 0, 0.0, 1.0)   # 1 = occupied wall, 0 = free
    return np.dstack((vs, xs, ys)), (h, w, res, ox, oy)
def _world_to_img(pts: np.ndarray, h, res, ox, oy) -> np.ndarray:
    px = (pts[:, 0] - ox) / res
    py = h - (pts[:, 1] - oy) / res
    return np.vstack((px, py)).T.astype(np.int32)


def overtake_zone(xy, room, width, exit_pad, min_len):
    """레이싱라인 각 점에서 **추월을 시작해도 되는가** (bool 배열).

    ★ 한 곳에만 둔다. gen_raceline(오버레이 PNG)과 frenet_planner(주행)가
      같은 함수를 쓴다. 두 벌로 두면 그림과 실제가 갈리는데, 그건 "그림은
      되는데 차는 안 피한다" 로 나타나 디버깅이 샌다.

    xy       (N,2) 라인 좌표
    room     (N,)  각 점에서 벽까지 거리 [m] (거리변환)
    width    나란히 가려면 필요한 복도 폭 [m] (= 2×room 과 비교)
    exit_pad 좁아지기 전에 미리 구간을 끝낼 거리 [m]
    min_len  이보다 짧은 구간은 버린다 [m]

    ★ 왜 끝을 잘라내나 — 회피는 시작보다 **끝내는 것**이 어렵다. 넓은 구간
      끝에서 옆으로 나가면 다 지나가기 전에 좁아져 상대와 벽 사이에 낀다.
      그때는 옆으로 피할 곳이 없다.
    """
    xy = np.asarray(xy, float)[:, :2]
    n = len(xy)
    ds = np.hypot(np.roll(xy[:, 0], -1) - xy[:, 0],
                  np.roll(xy[:, 1], -1) - xy[:, 1])
    s_arr = np.concatenate(([0.0], np.cumsum(ds[:-1])))
    L = float(ds.sum())

    wide = (2.0 * np.asarray(room, float)) >= float(width)
    ok = wide.copy()
    if not wide.any():
        return np.zeros(n, bool)
    if wide.all():
        return ok

    # ② 각 넓은 구간의 **끝**에서 exit_pad 만큼 잘라낸다.
    #    순환이므로 "다음 좁은 점까지 남은 거리" 를 s 로 재서 자른다.
    s_nar = s_arr[~wide]
    for i in np.where(wide)[0]:
        if float(((s_nar - s_arr[i]) % L).min()) < float(exit_pad):
            ok[i] = False

    # ③ 너무 짧은 구간은 버린다 — 들어가자마자 나오면 의미가 없다
    i = 0
    while i < n:
        if not ok[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and ok[j + 1]:
            j += 1
        if float((s_arr[j] - s_arr[i]) % L) < float(min_len):
            ok[i:j + 1] = False
        i = j + 1
    # 0 을 넘어 이어지는 구간은 위 루프가 둘로 나눠 본다. 한 번 더 이어 붙여
    # 판정하면 정확하지만, 짧게 잘리는 쪽이 안전한 방향이라 그대로 둔다.
    return ok


# 플래너 모드. /frenet/status 와 driven_path.csv planner_status.
TRACK, AVOID_STATIC, BLOCKED, FOLLOW, RECOVER, OVERTAKE = 0, 1, 2, 3, 4, 5
NORMAL, AVOID, RETURN = TRACK, AVOID_STATIC, RECOVER  # 옛 이름
PLANNER_MODE_NAME = {
    TRACK: 'TRACK', AVOID_STATIC: 'AVOID_STATIC', BLOCKED: 'BLOCKED',
    FOLLOW: 'FOLLOW', RECOVER: 'RECOVER', OVERTAKE: 'OVERTAKE',
}

# /frenet/diag (Float64MultiArray) · driven_path.csv 공통 열. 순서를 바꾸지 말 것.
FRENET_DIAG_KEYS = (
    'selected_d', 'speed_scale', 'planner_status', 'ot_allowed',
    'n_candidates', 'n_valid', 'n_obs', 'min_ttc', 'planning_ms',
    'opp_id', 'opp_s_dot', 'opp_d_lat', 'opp_vd',
)


def pack_frenet_diag(d: dict) -> list:
    return [float(d.get(k, float('nan'))) for k in FRENET_DIAG_KEYS]


def unpack_frenet_diag(data) -> dict:
    out = {k: float('nan') for k in FRENET_DIAG_KEYS}
    for k, v in zip(FRENET_DIAG_KEYS, data or ()):
        try:
            out[k] = float(v)
        except (TypeError, ValueError):
            out[k] = float('nan')
    return out


# ── 회피 로직 (믹스인) ──────────────────────────────────────────────────────


class ObstacleAvoidMixin:
    """
    필수 속성 (사용하는 쪽에서 준비할 것)
    ─────────────────────────────────────
    상태:   pos, yaw, yaw_rate, speed, wps, _wp_ds, mode, latest_scan
    맵:     map_arr, map_meta, track_mask, ray_occ, track_margin,
            use_bounds, inner_bound, outer_bound
    라이다: laser_dx, laser_dy
    호라이즌: obs_h_sec, obs_h_min, obs_h_max, obs_corr_hw, obs_release_scale
    Frenet: fr_grid, fr_clear, fr_clr_cap, fr_dmax, fr_bounds,
            fr_w_off, fr_w_chg, fr_w_clr, fr_w_spd (없으면 0=속도항 끔),
            grip_util/mu/pac_d 또는 fr_a_lat,
            _shift_cur, _fr_costs, _fr_best_d, _fr_vmax, _fr_v_ref
    검출결과: on_track_pts, blocking_pts, free_votes, lane_free
    발행:   fr_pub, obs_pub   (없으면 None)
    """

    def _frenet_paths(self, px: float, py: float):
        """레이싱라인을 기준선으로 한 후보 경로들을 만든다.

        반환: (cands (N, M, 2) map 좌표, seg (M,2), left (M,2))
              후보가 만들어지지 않으면 (None, None, None)

        기준선은 "현재 위치에서 앞으로 horizon 만큼의 레이싱라인" 이고, 후보는
        그것을 각 점의 **법선 방향으로** d 만큼 민 것이다. 직선 평행이동이 아니라
        점마다 법선을 쓰기 때문에 코너에서도 라인의 곡률을 그대로 따라간다.
        """
        wps = self.wps
        if wps is None or len(wps) < 5:
            return None, None, None
        n = len(wps)
        near = int(np.argmin(np.hypot(wps[:, 0] - px, wps[:, 1] - py)))
        horizon = self._obs_horizon()
        m = max(3, int(round(horizon / max(self._wp_ds, 1e-3))))
        m = min(m, n - 1)
        idx = (near + np.arange(m + 1)) % n
        seg = wps[idx, :2]

        # 점별 접선 → 법선 (중심차분). 끝점은 한쪽 차분으로 대체된다.
        fwd = wps[(idx + 1) % n, :2]
        bwd = wps[(idx - 1) % n, :2]
        tang = fwd - bwd
        tn = np.hypot(tang[:, 0], tang[:, 1])
        ok = tn > 1e-9
        tang[ok] /= tn[ok, None]
        left = np.stack([-tang[:, 1], tang[:, 0]], axis=1)

        # (N, M, 2) — 후보 × 경로점 × 좌표
        cands = seg[None, :, :] + self.fr_grid[:, None, None] * left[None, :, :]
        return cands, seg, left
    # ── 소비자가 안 정해도 동작하도록 하는 기본값 ─────────────────────
    #   frenet_planner 와 stanley_controller 가 각자 파라미터로 덮어쓴다.
    #   여기 기본값이 없으면 한쪽만 갱신했을 때 AttributeError 로 죽는다.
    spd_span  = 1.20        # 속도 판정에 쓸 전방 거리 [m]. 0 = 경로 전체(옛 동작)
    skew_gain = 1.0         # 자세 어긋남을 문턱에 얼마나 반영할지 (0=끔)
    skew_max  = 0.35        # 그 보정의 상한 [m]
    fr_min_room = 0.22      # 통과 여유 하한 [m] = 차 반폭 0.17 + 여유 0.05
    fr_gap_hyst = 0.10      # 반대쪽 통로로 갈아탈 때 요구하는 우세폭 [m]

    def _wall_room(self, flat: np.ndarray) -> np.ndarray:
        """각 점에서 맵 벽까지의 거리 [m]. 맵이 없으면 +inf.

        거리변환을 기동 시 한 번 만들어 두므로 조회는 배열 인덱싱 한 번이다.
        맵 밖으로 나간 점은 0 (= 갈 수 없는 곳) 으로 본다.
        """
        wd = getattr(self, 'wall_dist_m', None)
        if wd is None or self.map_meta is None:
            return np.full(len(flat), np.inf)
        h, w = self.map_meta[0], self.map_meta[1]
        res, ox, oy = self.map_meta[2], self.map_meta[3], self.map_meta[4]
        img = _world_to_img(flat, h, res, ox, oy)
        ix, iy = img[:, 0], img[:, 1]
        ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        out = np.zeros(len(flat))
        out[ok] = wd[iy[ok], ix[ok]]
        return out

    @staticmethod
    def _runs(mask: np.ndarray):
        """연속된 True 구간을 [(시작, 끝+1), ...] 로. 빈 배열이면 []."""
        out = []
        i = 0
        n = len(mask)
        while i < n:
            if mask[i]:
                j = i
                while j + 1 < n and mask[j + 1]:
                    j += 1
                out.append((i, j + 1))
                i = j + 1
            else:
                i += 1
        return out

    def _frenet_select(self, px: float, py: float, yaw: float) -> float:
        """후보 오프셋 중 하나를 고른다 (m, 좌측 +).

        ★★ 2026-08-11 재작성. 그 전 판정이 **레이싱라인 자신을 27% 의 프레임에서
            "트랙 밖" 으로 탈락시키고 있었다.** ★★

          옛 방식은 `inside = 마스크안(모든 점).all()` 이었다. 호라이즌이
          2.9~5.0m 인데 그 구간 전체가 벽에서 track_margin(0.28m) 밖이어야
          통과다. cool_clean 은 라인 점의 0.7% 가 0.28m 보다 벽에 가까운데,
          5m 창에 `all()` 을 걸면 그 0.7% 가 **27% 프레임 탈락**으로 증폭된다.
          17개 후보 중 중앙 5개만 살아남고 전부 살아남는 프레임은 0% 였다.

          결과가 로그에 그대로 있었다 — **장애물이 없을 때 더 많이 움직였다**:
              q1(장애물 없음)  |d| 중앙 0.075m  p90 0.375m  |d|>0.1 이 31%
              q2(장애물 있음)  |d| 중앙 0.075m  p90 0.225m           17%
          "회피가 안 된다" 의 절반은 회피 로직이 아니라 이 판정이었다.

        이제 예/아니오 대신 **통과 여유(room)** 를 잰다:

            room[i] = min over 경로점 ( 벽까지 거리 , 장애물까지 거리 )
                    = 그 후보 라인으로 갈 때 차가 확보하는 최소 반경

          · 벽 거리는 거리변환 조회 한 번 (마스크와 같은 비용)
          · 실행가능 = 벽여유 ≥ fr_min_room **그리고** 장애물여유 ≥ fr_clear
            두 임계를 따로 두는 이유는 아래 본문 주석 참고 — 합치면 장애물
            요구치가 조용히 내려간다.
            fr_min_room 0.22m 면 두 트랙 모두 레이싱라인 탈락 0% 다
            (실측 라인→벽 최소: cool_clean 0.280m, fun_clean 0.360m)

        그리고 **"더 넓은 쪽" 을 실제로 고른다** (2026-08-11 요구사항):

          실행가능한 후보들을 연속 구간(run)으로 묶으면 각 run 이 "그쪽으로
          지나갈 수 있는 통로" 다. run 의 점수는 그 안의 **최대 room** —
          곧 그쪽으로 갔을 때 낼 수 있는 가장 넓은 여유다. 점수가 큰 run 을
          고른다. 장애물이 왼쪽에 치우쳐 있으면 오른쪽 run 의 room 이 크므로
          자동으로 오른쪽을 고른다.

          ★ 좌우 뒤집힘 방지: 지금 쓰는 오프셋이 든 run 을 계속 쓴다. 다른
            run 이 fr_gap_hyst 이상 더 넓을 때만 갈아탄다. 이게 없으면
            장애물 정면에서 좌우가 프레임마다 바뀌어 조향이 진동한다.

        고른 run 안에서는 예전 비용으로 한 점을 정한다 — 라인 복귀력(w_offset)이
        살아 있으므로 **장애물이 지나가면 저절로 d=0 으로 돌아온다.**

            비용 = w_offset·|d| + w_change·|d-현재| - w_clear·min(room, cap)

        ★ 비용 벡터를 투표 창만큼 이동평균한다. 라이다 한 프레임의 노이즈로
          목표 오프셋이 튀면 그게 곧 조향 킥이 된다.
        """
        cands, seg, left = self._frenet_paths(px, py)
        if cands is None:
            return 0.0
        N, M, _ = cands.shape
        flat = cands.reshape(N * M, 2)

        # ── 장애물까지의 여유 ──────────────────────────────────────────
        obs = self.on_track_pts
        if len(obs) == 0:
            obs = np.asarray(self.blocking_pts) if self.blocking_pts else obs
        if len(obs):
            # 관련 없는 먼 점은 버린다 — cdist 크기를 실제로 지배하는 건 이쪽이다
            near_m = (np.hypot(obs[:, 0] - px, obs[:, 1] - py)
                      < self._obs_horizon() + self.fr_dmax + 1.0)
            obs = obs[near_m]
        if len(obs):
            d_obs = scipy.spatial.distance.cdist(flat, obs).min(axis=1)
        else:
            d_obs = np.full(N * M, np.inf)

        # ── 벽까지의 여유 ─────────────────────────────────────────────
        d_wall = (self._wall_room(flat) if self.fr_bounds
                  else np.full(N * M, np.inf))

        # ── 통과 여유 ─────────────────────────────────────────────────
        #   ★ 벽과 장애물은 **요구치가 다르다.** 둘을 min 으로 합쳐 임계 하나를
        #     걸면 안 된다 (2026-08-11 첫 시도에서 실제로 그랬다):
        #       · 벽    — 맵에 있고 안 움직인다. 차 반폭 + 여유(0.22m)면 지나간다
        #       · 장애물 — 위치가 덜 정확하고 움직일 수 있다. 더 크게(0.30m) 잡는다
        #     합치면 장애물 요구치가 0.30 → 0.22 로 조용히 내려가, 검증에서
        #     q1 은 좋아졌는데 **q2 의 회피량까지 같이 줄었다** (|d|>0.1 이 16%→5%).
        #     "여유가 늘었다" 가 아니라 "덜 피하게 됐다" 는 뜻이라 정반대다.
        obs_room  = d_obs.reshape(N, M).min(axis=1)
        wall_room = d_wall.reshape(N, M).min(axis=1)
        # 넓이 순위를 매길 때는 실제로 차가 쓸 수 있는 폭이므로 둘 중 좁은 쪽
        room = np.minimum(obs_room, wall_room)

        d = self.fr_grid
        feasible = (wall_room >= self.fr_min_room) & (obs_room >= self.fr_clear)

        # ── 추월 금지 구간이면 옆으로 안 나간다 ───────────────────────
        #   구간 밖에서 옆으로 나가면 다 지나가기 전에 통로가 좁아져 **상대와
        #   벽 사이에 낀다.** 그때는 손쓸 방법이 없다. 그래서 여기서는 후보를
        #   d=0(레이싱라인) 하나로 묶고, 대신 상대 뒤를 따라간다 —
        #   횡으로 못 풀면 종으로 푸는 수밖에 없다 (_follow_scale).
        #   ※ 이미 옆으로 나가 있으면(|shift|>0.05) 라인으로 **돌아오는 것**은
        #     허용한다. 구간 경계를 넘는 순간 오프셋을 계단으로 끊으면 조향이
        #     튀고, 무엇보다 지금 상대 옆에 있을 수 있다.
        self._ot_allowed = True
        self._ot_static = False
        if self.ot_zone_on:
            zone = self._overtake_zone()
            lut = self._s_lut()
            if zone is not None and lut is not None:
                i_ego = int(np.argmin(np.hypot(lut['xy'][:, 0] - px,
                                               lut['xy'][:, 1] - py)))
                self._ot_allowed = bool(zone[i_ego])
            # ★★ 2026-08-15 — 앞을 막은 것이 **정지 장애물**이면 구간을 안 따진다.
            #   08/15 실차: "정지한 장애물조차 q2 에선 피했는데 추월 모드에서는
            #   못 피하고 박았다." 당연하다 — 구간 밖이면 d=0 으로 묶여 있었다.
            #
            #   구간을 두는 이유는 **상대차와 나란히 달리는 시간이 길기 때문**이다.
            #   그 사이 통로가 좁아지면 낀다. 콘은 길이 20cm 라 스쳐 지나가면
            #   끝이고, 낄 일이 없다. 정지 장애물에 구간을 걸면 얻는 것 없이
            #   **회피 능력만 잃는다** (q2 에서 ot_zone 을 끈 것과 같은 논리).
            #
            #   "정지" 판정은 추적된 클러스터의 ṡ 로 한다. 아직 추적이 덜 됐으면
            #   (n < opp_min_obs) 보수적으로 **움직인다고 본다** — 상대차를
            #   정지로 오판해 좁은 데서 나가는 쪽이 훨씬 위험하다.
            if not self._ot_allowed:
                trs = [t for t in (getattr(self, '_opp_tracks', []) or [])
                       if t.get('n', 0) >= self.opp_min_obs]
                if trs and all(abs(float(t.get('s_dot', 0.0))) < self.opp_min_speed
                               for t in trs):
                    self._ot_allowed = True
                    self._ot_static = True
        #   ※ feasible 을 여기서 깎지 않는다. 깎으면 후보가 전멸해 아래
        #     "전부 막혔다" 폴백(argmax(room))으로 떨어지고, 하필 그게 **가장
        #     크게 옆으로 나가는 후보**다 — 금지하려던 것을 정확히 하게 된다.
        #     (실측: 금지 구간에서 d 가 −0.075 → −0.150 으로 오히려 커졌다)
        #     그래서 고른 뒤에 라인으로 되돌린다.

        # ── 곡률 실행가능 + 후보별 v_max ─────────────────────────────
        # 법선 오프셋 d 의 평행곡선: κ' = κ / (1 − d·κ)  (좌회전 +, d 좌측 +).
        # 안쪽으로 밀면 반경이 차 R_min = L/tan(max_steer) 아래로 떨어질 수 있다.
        # 한계는 차량 기하에서 온다. d=0 은 항상 살린다 — 라인 자체가 κ_max 에
        # 붙어 있어도 회피가 라인에서 쫓겨나면 안 된다.
        #
        # v_curve = √(a_lat / |κ'|). 조향 뒤에 깎는 steer_speed_limit 과 같은
        # 물리지만, **고르기 전**에 써서 빠르게 통과할 후보를 고른다.
        wb = float(getattr(self, 'wheelbase', 0.33) or 0.33)
        ms = float(getattr(self, 'max_steer', 0.36) or 0.36)
        kappa_max = math.tan(ms) / max(wb, 1e-6)
        a_lat = self._frenet_a_lat()
        v_ref = self._line_speed(px, py)
        dx, dy = np.gradient(seg[:, 0]), np.gradient(seg[:, 1])
        ddx, ddy = np.gradient(dx), np.gradient(dy)
        kap = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)
        kappa_ok = np.ones(N, dtype=bool)
        v_curve = np.zeros(N)
        kmax_arr = np.zeros(N)
        den_eps = 0.05
        kap_eps = 1e-4
        # ★★ 2026-08-15 — 속도는 **가까운 구간의 곡률**로만 정한다.
        #
        #   예전엔 후보 경로 **전체**(호라이즌 2.9~5.0m)의 최대 곡률로 v_max 를
        #   구했다. 그러면 5m 앞이 조인 것 때문에 **지금** 느려진다.
        #   그런데 컨트롤러의 속도 프로파일이 이미 앞을 보고 감속한다
        #   (역방향 스윕 + speed_lag). **이중 감속**이다.
        #
        #   실차 로그가 그대로 보여줬다 (08/15 race, 장애물까지 거리별 배율):
        #       3~4m  0.98      ← 아직 멀다
        #       2~3m  0.85      ← 오프셋은 0.1m 뿐인데 15% 깎임
        #       1~1.5m 0.87
        #       0~1m  0.63      ← 붙을수록 느려진다
        #   "너무 일찍 속도를 낮춘다. 바짝 따라붙어야 추월을 하지" 는 정확한 지적이다.
        #
        #   실행가능(kappa_ok)은 **전체 경로**로 판정한다 — 못 도는 길을 고르면
        #   안 되니까. 속도만 가까운 구간을 본다.
        n_near = len(kap)
        if self.spd_span > 0.0:
            n_near = max(2, int(round(self.spd_span / max(self._wp_ds, 1e-3))))
            n_near = min(n_near, len(kap))
        for i, di in enumerate(d):
            denom = 1.0 - float(di) * kap
            # 곡률 중심을 지나거나 분모≈0 이면 κ' 가 폭주한다 → 탈락, v=0.
            if np.any(~np.isfinite(denom)) or np.any(denom <= den_eps):
                kappa_ok[i] = False
                continue
            kabs = np.abs(kap / denom)
            kmax = float(np.max(kabs))                 # 실행가능 판정: 전체
            if (not np.isfinite(kmax)) or kmax > kappa_max * 1.05:
                kappa_ok[i] = False
                continue
            kmax_arr[i] = kmax
            k_spd = float(np.max(kabs[:n_near]))       # 속도: 가까운 구간만
            v_curve[i] = math.sqrt(a_lat / max(k_spd, kap_eps))
        zero_i = int(np.argmin(np.abs(d)))
        kappa_ok[zero_i] = True
        if kmax_arr[zero_i] <= 0.0:
            kmax_arr[zero_i] = float(np.max(np.abs(kap)))
            v_curve[zero_i] = math.sqrt(a_lat / max(kmax_arr[zero_i], kap_eps))
        feasible_wo = feasible.copy()
        feasible = feasible & kappa_ok

        v_cand = np.minimum(v_curve, v_ref)
        speed_score = np.clip(v_cand / max(v_ref, 1e-3), 0.0, 1.0)
        w_spd = float(getattr(self, 'fr_w_spd', 0.0) or 0.0)
        w_ttc = float(getattr(self, 'fr_w_ttc', 0.0) or 0.0)
        ttc_sec, ttc_score = self._candidate_ttc(d, px, py)

        trav = None
        n_obs_now = self._n_on_track()
        if (bool(getattr(self, 'candidate_fb', False))
                and (n_obs_now > 0 or abs(float(self._shift_cur)) > 0.05)):
            trav = self._candidate_fb_times(cands, feasible)
            self._fr_trav = trav

        # candidate_fb 켜면 통과시간 + 전환 벌점. w_speed 휴리스틱은 버린다.
        if trav is not None and np.any(np.isfinite(trav)):
            t_line = trav[zero_i]
            if not np.isfinite(t_line) or t_line < 1e-6:
                t_line = float(np.nanmin(trav))
            t_line = max(t_line, 1e-6)
            cost = (trav / t_line
                    + self.fr_w_chg * np.abs(d - self._shift_cur))
            cost = np.where(np.isfinite(trav), cost, 1e6)
        else:
            cost = (self.fr_w_off * np.abs(d)
                    + self.fr_w_chg * np.abs(d - self._shift_cur)
                    - self.fr_w_clr * np.minimum(room, self.fr_clr_cap))
            if w_spd > 0.0:
                cost = cost - w_spd * speed_score
        if w_ttc > 0.0:
            cost = cost + w_ttc * ttc_score
        self._fr_costs.append(cost)
        cost_avg = np.mean(np.stack(self._fr_costs), axis=0)

        runs = self._runs(feasible)
        self._fr_blocked = not bool(runs)
        if runs:
            # ── 어느 쪽이 더 넓은가 ────────────────────────────────────
            scores = [float(room[a:b].max()) for a, b in runs]
            pick = int(np.argmax(scores))
            # 지금 쓰는 오프셋이 든 run 은 웬만하면 유지한다 (좌우 진동 방지)
            cur_i = int(np.argmin(np.abs(d - self._shift_cur)))
            for k, (a, b) in enumerate(runs):
                if a <= cur_i < b:
                    if scores[pick] < scores[k] + self.fr_gap_hyst:
                        pick = k
                    break
            a, b = runs[pick]
            best = a + int(np.argmin(cost_avg[a:b]))
            self._fr_gap_room = scores[pick]
        else:
            # 전부 막혔다. 벽·장애물로는 되는 후보가 곡률만 안 되면 라인(d=0)
            # — 안쪽으로 더 파고들면 조향이 안 따라간다. 둘 다 막혔으면 예전처럼
            # 제일 넓은 쪽 (그건 AEB/감속의 영역이고, 조향은 최선을 다한다).
            if np.any(feasible_wo):
                best = zero_i
            else:
                best = int(np.argmax(room))
            self._fr_gap_room = float(room[best])
            self.get_logger().warn(
                f'[frenet] 통과 가능한 후보 없음 — 최대 여유 {room[best]:.2f}m '
                f'(요구 {self.fr_min_room:.2f}m), d={d[best]:+.2f}m. '
                f'감속해야 하는 상황이다',
                throttle_duration_sec=1.0)

        # ── 추월 금지 구간: 라인으로 되돌린다 ────────────────────────
        #   이미 옆에 나가 있으면(|shift|>0.05) 그대로 둔다 — 구간 경계에서
        #   오프셋을 계단으로 끊으면 조향이 튀고, 지금 상대 옆일 수도 있다.
        #   대신 종방향으로 푼다 (_follow_scale 이 속도를 상대에 맞춘다).
        if self.ot_zone_on and not self._ot_allowed and abs(self._shift_cur) <= 0.05:
            best = zero_i
            self._fr_gap_room = float(room[best])

        # ★ 다 지나갈 때까지 붙잡는다 (상대가 시야에서 사라져도).
        #   d 를 정한 **뒤**에 건다 — 후보 평가를 건드리면 "지금 어디가 비었나"
        #   판단까지 왜곡된다. static_lock 은 콘처럼 안 움직이는 대상의 d 를
        #   프레임마다 다시 고르지 않는다 (좌우 뒤집힘 방지).
        d_raw = float(d[best])
        d_raw, best = self._apply_static_lock(d, feasible, d_raw, best)
        self._fr_best_d = self._update_hold(d_raw)
        self._fr_n_cand = int(N)
        self._fr_n_valid = int(np.count_nonzero(feasible))
        self._fr_n_obs = self._n_on_track()
        self._set_planner_mode(d_raw, float(self._fr_best_d),
                               self._fr_n_valid)
        self._fr_v_ref = float(v_ref)
        self._fr_vmax = float(v_cand[best])
        self._fr_kappa = float(kmax_arr[best])
        self._fr_speed_score = float(speed_score[best])
        self._fr_cost = float(cost_avg[best])
        self._fr_ttc = float(ttc_sec[best])
        self._fr_ttc_score = float(ttc_score[best])
        if self.fr_pub is not None:
            self._pub_frenet(cands, feasible, best)
        return self._fr_best_d

    def _frenet_a_lat(self) -> float:
        """허용 횡가속. fr_a_lat 이 있으면 그걸 쓰고, 없으면 η·μ·D·g."""
        a = float(getattr(self, 'fr_a_lat', 0.0) or 0.0)
        if a > 0.0:
            return a
        eta = float(getattr(self, 'grip_util', 0.6) or 0.6)
        mu = float(getattr(self, 'mu', 0.348) or 0.348)
        pac_d = float(getattr(self, 'pac_d', 1.15) or 1.15)
        return max(float(np.clip(eta, 0.05, 1.0)) * mu * pac_d * GRAVITY, 1e-3)

    def _profile_speed_arr(self):
        """컨트롤러가 실제로 쓰는 속도 프로파일. wps 가 그대로면 캐시.

        ★★ 2026-08-15 — 예전엔 여기서 **CSV 속도열을 그대로** 썼다.
          그런데 CSV 는 gen_raceline 이 --mu 0.9 로 계산한 값이고(넉넉히 잡는 게
          규약), 주행 중 실제 속도는 solve_profile 이 μ_eff(0.483)로 다시 푼다.
          그래서 speed_scale = v_max/v_ref 의 **분모만 1.5배 큰** 상태가 됐고,
          장애물이 없어도 배율이 항상 0.5 근처로 깎였다.

          실차 로그가 그대로 말해줬다 (08/15 race):
              status 0(평상, 막힌 것 없음) 에서도 speed_scale 중앙 0.52
              지령 속도 중앙 1.50㎧ (프로파일은 2.47~5.20)
          "q2 가 잘 피하는데 계속 느리다" 의 원인이 이것이다. 회피와 무관하다.
        """
        wps = getattr(self, 'wps', None)
        if wps is None or len(wps) < 3 or wps.shape[1] < 3:
            return None
        cached = getattr(self, '_prof_cache', None)
        if cached is not None and cached[0] is wps:
            return cached[1]
        from race_stack.speed_profile import solve_profile
        try:
            v, _ = solve_profile(
                wps[:, 0], wps[:, 1], wps[:, 2],
                mu_eff=float(getattr(self, 'mu', 0.348))
                       * float(getattr(self, 'pac_d', 1.15)),
                grip_util=float(getattr(self, 'grip_util', 0.6)),
                lf=float(getattr(self, 'lf', 0.1757)),
                lr=float(getattr(self, 'lr', 0.1441)),
                v_min=float(getattr(self, 'min_speed', 0.3)),
                a_lon_acc=float(getattr(self, 'a_lon_acc', 0.0)),
                a_lon_brk=float(getattr(self, 'a_lon_brk', 0.0)),
                hard_cap=float(getattr(self, 'hard_cap', 0.0)))
        except Exception:
            return None
        self._prof_cache = (wps, v)
        return v

    def _line_speed(self, px: float, py: float) -> float:
        """현재 위치에서 **컨트롤러가 낼 속도**. speed_scale 의 분모다.

        ★ CSV 열이 아니라 solve_profile 결과를 써야 한다 — 컨트롤러가 그걸
          쓰기 때문이다. 분모가 다르면 배율이 통째로 어긋난다 (위 주석 참고).
        """
        wps = getattr(self, 'wps', None)
        if wps is not None and len(wps) >= 2:
            i = int(np.argmin(np.hypot(wps[:, 0] - px, wps[:, 1] - py)))
            prof = self._profile_speed_arr()
            if prof is not None and float(prof[i]) > 1e-3:
                return float(prof[i])
            if wps.shape[1] >= 3 and float(wps[i, 2]) > 1e-3:
                return float(wps[i, 2])
        return max(float(getattr(self, 'speed', 0.0) or 0.0), 1e-3)

    def _candidate_ttc(self, d_grid, px, py):
        """후보별 (ttc_sec, ttc_score).

        `_opp_tracks['s_dot']` 는 **레이싱라인 전역 속도 ṡ** 이다 (ego-relative
        가 아님). 상대속도는 v_rel = v_ego_s − s_dot. 정적(ṡ≈0)이면 TTC≈Δs/v_ego.

        score = clip((ttc_ref − ttc) / (ttc_ref − ttc_min), 0, 1)
        장애물이 없거나 멀어지면 0. w_ttc=0 이면 계산을 건너뛴다.
        """
        n = len(d_grid)
        inf = 1.0e9
        ttc = np.full(n, inf)
        zeros = np.zeros(n)
        if float(getattr(self, 'fr_w_ttc', 0.0) or 0.0) <= 0.0:
            return ttc, zeros
        lut = self._s_lut()
        if lut is None:
            return ttc, zeros
        s_ego, _ = self._project_sd(np.array([px, py], float), lut)
        v_ego = abs(float(getattr(self, 'speed', 0.0) or 0.0))
        half = float(getattr(self, 'obs_corr_hw', 0.22) or 0.22)
        ttc_ref = float(getattr(self, 'fr_ttc_ref', 2.0) or 2.0)
        ttc_min = float(getattr(self, 'fr_ttc_min', 0.5) or 0.5)
        den = max(ttc_ref - ttc_min, 1e-3)
        eps = 0.1

        obs = []
        for tr in getattr(self, '_opp_tracks', []) or []:
            obs.append((float(tr.get('s', 0.0)),
                        float(tr.get('d_lat', 0.0)),
                        float(tr.get('s_dot', 0.0))))
        if not obs:
            pts = getattr(self, 'on_track_pts', None)
            if pts is not None and len(pts):
                # 정적 점. 너무 많으면 가까운 것만.
                rel = np.hypot(pts[:, 0] - px, pts[:, 1] - py)
                keep = np.argsort(rel)[:40]
                for p in pts[keep]:
                    s, dlat = self._project_sd(p, lut)
                    obs.append((s, dlat, 0.0))
        if not obs:
            return ttc, zeros

        for s_o, d_o, s_dot in obs:
            ds = self._wrap_ds(s_o - s_ego, lut['L'])
            if ds <= 0.05:
                continue
            v_rel = v_ego - s_dot
            dt = ds / v_rel if v_rel > eps else inf
            for i, di in enumerate(d_grid):
                if abs(d_o - float(di)) > half:
                    continue
                if dt < ttc[i]:
                    ttc[i] = dt
        score = np.clip((ttc_ref - ttc) / den, 0.0, 1.0)
        return ttc, score

    def _pub_frenet(self, cands, feasible, best):
        """후보 경로를 RViz 에 그린다 — 초록=선택, 파랑=가능, 빨강=불가."""
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i in range(len(cands)):
            mk = Marker()
            mk.header.frame_id = 'map'
            mk.header.stamp = now
            mk.ns = 'frenet'
            mk.id = i
            mk.type = Marker.LINE_STRIP
            mk.action = Marker.ADD
            mk.scale.x = 0.03 if i == best else 0.015
            if i == best:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.0, 1.0, 0.0, 1.0
            elif feasible[i]:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.2, 0.5, 1.0, 0.5
            else:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.2, 0.2, 0.4
            for x, y in cands[i]:
                pt = Point(); pt.x = float(x); pt.y = float(y); pt.z = 0.02
                mk.points.append(pt)
            arr.markers.append(mk)
        self.fr_pub.publish(arr)
    def _build_track_mask(self, ign: float):
        """경계 안쪽 판정을 픽셀 마스크로 사전계산 (기동 시 1회).

        ★ 왜 필요한가 — 측정으로 드러난 병목
          예전엔 스캔마다 점유 셀 하나하나에 cv2.pointPolygonTest 를 2번씩
          파이썬 루프로 돌렸다. 실제 트랙(벽 포함) 스캔에서 점유 셀이 2505개라
          호출이 5010회/스캔, _scan_cb 가 44.5ms(22.5Hz) 걸렸다. 라이다는 40Hz
          이므로 따라가지 못하고, 10프레임 투표에 445ms = 0.80m 주행이 걸렸다.
          감시거리가 1.04m 인데 판정에 0.80m 를 쓰면 피할 거리가 남지 않는다.

          거리변환으로 마스크를 한 번 만들어 두면 판정이 배열 인덱싱 한 번이 된다.
          (io > ign) ⟺ 외곽 폴리곤 내부에서 경계까지 거리 > ign
          (ii < -ign) ⟺ 내곽 폴리곤 외부에서 경계까지 거리 > ign
        """
        self.track_mask = None
        if self.map_meta is None:
            return
        h, w = self.map_meta[0], self.map_meta[1]
        res = self.map_meta[2]

        # ★ 맵에서 직접 만든다 (2026-08-05). 예전엔 inner/outer 폴리곤을 썼는데
        #   그건 사람이 클릭해 만든 저해상도 도형이다. half_clean 은 33m 트랙에
        #   inner 10점 / outer 28점뿐이라 실제 벽선을 전혀 못 따라갔다:
        #     · 마스크가 **실제로는 벽인 181px 를 트랙 안쪽으로 포함** →
        #       그 벽에 찍힌 라이다 점이 장애물로 오인됐다
        #     · 반대로 진짜 주행 가능한 456px 는 누락
        #   실차에서 장애물 없이도 벽을 피하려 들며 트랙 좌우로 요동쳤다.
        #   맵의 자유공간을 margin 만큼 침식하면 5cm 해상도로 실제 벽을 따라간다.
        #
        #   margin 은 "이보다 벽에 가까운 반사는 벽으로 본다" 는 뜻이다.
        #   라인 위 최소 여유(이 트랙 36cm)보다 작아야 라인이 마스크 안에 남는다.
        margin = float(self.track_margin)
        # map_arr 은 상하 반전(아래가 0행)이라 _world_to_img 규약(위가 0행)에 맞춘다
        occ_top = np.flipud(self.map_arr[:, :, 0]) > 0.5
        free = (~occ_top).astype(np.uint8)
        dist_m = cv2.distanceTransform(free, cv2.DIST_L2, 5) * res
        self.track_mask = dist_m > margin
        # ★ 거리장 자체를 보관한다 (2026-08-11). 마스크는 "여기 들어가도 되나" 라는
        #   예/아니오뿐인데, 회피는 "여기가 **얼마나 넓나**" 를 알아야 한다.
        #   _frenet_select 가 후보별 통과 여유(room)를 이걸로 잰다.
        self.wall_dist_m = dist_m

        src = '맵 자유공간'
        # 경계 폴리곤이 있으면 "트랙 밖 자유공간"(피트·관중석 등)을 덜어내는
        # 용도로만 겹쳐 쓴다. 폴리곤이 엉성해도 벽을 장애물로 만들지는 않는다.
        if (self.use_bounds and self.inner_bound is not None
                and self.outer_bound is not None):
            outer_m = np.zeros((h, w), np.uint8)
            inner_m = np.zeros((h, w), np.uint8)
            cv2.fillPoly(outer_m, [self.outer_bound.astype(np.int32)], 1)
            cv2.fillPoly(inner_m, [self.inner_bound.astype(np.int32)], 1)
            self.track_mask &= (outer_m > 0) & (inner_m == 0)
            src += ' ∩ 경계 폴리곤'

        self.get_logger().info(
            f'track mask built: {int(self.track_mask.sum())} / {h*w} px '
            f'({src}, margin {margin:.2f}m)')

    # ── 빔별 기대거리 비교 (레이캐스팅) ────────────────────────────────────
    def _build_raycast_map(self):
        """레이캐스팅용 점유격자를 기동 시 1회 만든다.

        ★ 왜 맵을 부풀리나 (dilate)
          기대거리를 "정확한 위치" 기준으로 계산하면, AMCL 이 20cm 틀린 순간
          벽 반사가 기대보다 20cm 짧게 보여 장애물이 된다. 마스크 방식이 실패한
          것과 똑같은 이유다.

          그래서 맵의 벽을 위치추정 불확실성 반경만큼 부풀린 뒤 레이캐스팅한다.
          그러면 기대거리는 "불확실성 안의 어떤 위치에서든 가능한 **가장 짧은**
          거리" 가 된다. 실측이 그보다도 더 짧다면, 그 반사는 어떤 위치를
          가정해도 맵 벽으로 설명할 수 없다 — 즉 진짜 장애물이다.

          이 방향의 대가는 "벽에 바짝 붙은 장애물을 놓친다" 는 것이다. 반대
          방향의 오류(벽을 장애물로 봄)는 직선 9m 를 42cm 밀려 달리게 만들었으니,
          이쪽 대가가 훨씬 싸다.
        """
        self.ray_occ = None
        if self.map_meta is None or not bool(self.get_parameter('raycast_obs').value):
            return
        res = self.map_meta[2]
        unc = float(self.get_parameter('raycast_pose_unc').value)
        occ_top = (np.flipud(self.map_arr[:, :, 0]) > 0.5).astype(np.uint8)
        r_px = max(1, int(round(unc / res)))
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_px + 1, 2 * r_px + 1))
        self.ray_occ = cv2.dilate(occ_top, k) > 0
        self.get_logger().info(
            f'raycast map built: 벽을 {unc:.2f}m({r_px}px) 부풀림, '
            f'점유 {int(occ_top.sum())} → {int(self.ray_occ.sum())} px')
    def _raycast_expected(self, lx0: float, ly0: float,
                          angs: np.ndarray, rmax: float) -> np.ndarray:
        """(lx0,ly0) 에서 angs 방향으로 부풀린 맵을 쏴 기대거리를 구한다.

        벽에 닿지 않으면 rmax 를 돌려준다. 전 빔을 한 번에 행진시켜(B×S 배열)
        파이썬 루프를 없앴다 — _scan_cb 는 40Hz 안에 끝나야 한다.
        """
        h, w, res, ox, oy = self.map_meta
        step = res * 0.5
        t = np.arange(0.0, rmax + step, step)                 # (S,)
        xs = lx0 + np.cos(angs)[:, None] * t[None, :]
        ys = ly0 + np.sin(angs)[:, None] * t[None, :]
        ix = ((xs - ox) / res).astype(np.int32)
        iy = (h - (ys - oy) / res).astype(np.int32)
        inb = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        hit = np.zeros(ix.shape, bool)
        hit[inb] = self.ray_occ[iy[inb], ix[inb]]
        # 맵 밖으로 나간 지점은 더 볼 것이 없으므로 벽으로 간주해 행진을 멈춘다
        hit |= ~inb

        # ★ 출발점이 부풀린 벽 안에 있는 경우 (차가 벽에서 pose_unc 이내)
        #   그대로 두면 t=0 에서 즉시 "명중" 이라 기대거리가 전부 0 이 되고,
        #   장애물 탐지가 통째로 멎는다. 트랙 여유가 36cm 인데 부풀림이 25cm 라
        #   실제로 자주 일어난다. 광선이 일단 자유공간으로 빠져나온 뒤부터
        #   명중을 센다.
        free = ~hit
        escaped = free.any(axis=1)
        first_free = np.argmax(free, axis=1)
        after = np.arange(hit.shape[1])[None, :] > first_free[:, None]
        hit = hit & after

        any_hit = hit.any(axis=1)
        first = np.argmax(hit, axis=1)
        exp = np.where(any_hit, t[first], rmax)
        # 끝까지 자유공간을 못 만난 빔은 판단 근거가 없다 — 0 을 돌려주면
        # (meas < 0 - deficit) 이 항상 거짓이 되어 장애물로 오인하지 않는다.
        return np.where(escaped, exp, 0.0)
    def _raycast_obstacle_pts(self, msg: LaserScan) -> np.ndarray:
        """장애물로 볼 빔만 골라 map 좌표 점으로 돌려준다.

        검사 두 가지의 **합집합**이다. 하나로는 못 잡는 영역이 서로 다르다.

        (A) 기대보다 뚜렷하게 짧게 끊긴 빔        meas < 기대 − deficit
            벽 **근처**의 장애물을 잡는다. 기대거리는 부풀린 맵에서 오므로
            AMCL 오차에 강하다 (_build_raycast_map 참고).
            약점: 기대거리가 짧은 곳(좁은 통로·코너 정면)에서는 deficit 만큼의
            여유가 안 나와 아무것도 못 잡는다.

        (B) 빔 끝점이 **열린 공간 한가운데** 찍혔다  벽까지 거리 > free_gap
            트랙 한가운데 놓인 장애물을 잡는다. "거리 차이" 가 아니라 "그 점이
            벽에서 얼마나 떨어졌나" 를 보므로 좁은 통로에서도 그대로 동작하고,
            AMCL 오차에도 강하다 (벽 반사는 정의상 벽 근처에 찍힌다).

        ★ 왜 (B) 를 넣었나 — 2026-08-09 q2 bag 6개를 재생해 실측했다.
          가려진 장애물(벽 뒤)을 제외한 진짜 검출률:

              거리        0.3~1m   1~2m   2~3m   3~4m   4~5m
              (A)만         32%    25%    14%    17%     2%
              (A)|(B)       37%    46%    41%    35%    21%

          호라이즌은 2.9~5.0m 로 맞춰 뒀는데 **검출기가 그 거리에서 안 잡고
          있었다.** 감지했을 땐 이미 1m 앞이고, MAP 은 오프셋을 옮기는 데
          2.2~3.7m 를 달려야 한다 — 구조적으로 못 피한다.
          (B) 를 더하면 2~3m 에서 3배, 4~5m 에서 10배가 된다.

          대안으로 raycast_pose_unc 를 0.25→0.12 로 낮추면 비슷한 재현율이
          나오지만 그건 **벽을 장애물로 보게 만드는 방향**이다 (2026-08-05 에
          직선 9.4m 를 42cm 밀려 달린 그 실패). 우리 AMCL 종방향 오차가 실측
          10~20cm 이므로 0.12 는 쓸 수 없다. (B) 는 unc 를 건드리지 않는다.

          free_gap 은 0.30m 가 무릎점이다 (0.25 로 더 낮춰도 이득이 없다).
          트랙 반폭이 0.5~0.6m 이므로 한가운데 장애물은 0.30 을 넘고, AMCL
          오차 0.2m 를 빼도 여유가 남는다.
        """
        deficit = float(self.get_parameter('raycast_deficit').value)
        fov     = math.radians(float(self.get_parameter('raycast_fov_deg').value))
        bstep   = math.radians(float(self.get_parameter('raycast_beam_deg').value))

        n = len(msg.ranges)
        stride = max(1, int(round(bstep / max(msg.angle_increment, 1e-6))))
        idx = np.arange(0, n, stride)
        # ★ 낮은/얇은 물체(콘, 상대 차 옆면)는 각폭이 1° 보다 작다.
        #   옛 기본값 beam_deg=1.0 은 UST-10LX(0.25°) 를 4개에 하나 건너뛰어
        #   2~4m 앞 10cm 콘을 통째로 놓쳤다. 높은 박스·차는 각폭이 커서 살아남고
        #   낮은 콘만 안 보이던 증상의 소프트웨어 쪽 원인. (스캔 평면 11cm 보다
        #   완전히 낮은 물체는 하드웨어로도 안 보인다.)
        a_car = msg.angle_min + idx * msg.angle_increment       # 차량 기준 빔각
        keep = np.abs(a_car) <= fov * 0.5
        idx, a_car = idx[keep], a_car[keep]
        if len(idx) == 0:
            return np.zeros((0, 2))

        meas = np.asarray(msg.ranges, dtype=float)[idx]
        valid = np.isfinite(meas) & (meas > msg.range_min) & (meas < msg.range_max)

        horizon = self._obs_horizon()
        rmax = horizon + deficit + 0.5

        # 라이다 원점은 base_link 보다 앞에 있다 — 기대거리도 거기서 재야 한다
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        lx0 = float(self.pos[0]) + cy * self.laser_dx - sy * self.laser_dy
        ly0 = float(self.pos[1]) + sy * self.laser_dx + cy * self.laser_dy
        exp = self._raycast_expected(lx0, ly0, self.yaw + a_car, rmax)

        ahead = valid & (meas < horizon)

        # ── 자세 어긋남 자기보정 ──────────────────────────────────────
        #
        # ★★ 2026-08-15 실차: "AMCL 이 맵과 완전 일치하지 않는 순간마다 트랙
        #    벽들을 장애물로 인식했다." 맞다. 로그로 확인했다:
        #
        #        정합도(호라이즌 안 빔의 벽거리 중앙)  프레임   (A) 검출점
        #          <5cm  좋음                        1095        0
        #          5~10cm                            210       24   ← 벽이다
        #          10~20cm                            20       16
        #
        #    회피가 트랙 전 구간에 퍼졌다 (s=12~20m 는 100%, |d| 최대 0.6m).
        #    실제 장애물은 둘뿐이었다.
        #
        #   pose_unc(0.25m) 로 벽을 부풀리는 것은 기대거리를 **짧게** 만드는
        #   방향이라, "측정이 기대보다 짧다" 판정에서 **한쪽으로만** 보호된다.
        #   차가 벽에 가까워지는 쪽으로 틀어지면 실측이 짧아져 그대로 걸린다.
        #
        #   자세가 얼마나 어긋났는지는 **매 스캔 공짜로 잴 수 있다** —
        #   빔 끝점이 맵 벽에서 떨어진 거리의 중앙값이다. 자세가 맞으면 0 이고
        #   틀어진 만큼 커진다. 중앙값이라 장애물 몇 개에는 안 흔들린다.
        #   그만큼 두 검사의 문턱을 같이 올린다 = 자기보정.
        skew = 0.0
        wd_all = None
        if getattr(self, 'wall_dist_m', None) is not None and ahead.sum() >= 20:
            ang_a = self.yaw + a_car
            ends_a = np.vstack((lx0 + meas * np.cos(ang_a),
                                ly0 + meas * np.sin(ang_a))).T
            wd_all = self._wall_room(ends_a)
            skew = float(np.median(wd_all[ahead]))
            skew = float(np.clip(skew * self.skew_gain, 0.0, self.skew_max))
        self._pose_skew = skew

        obs = ahead & (meas < exp - (deficit + skew))            # (A)

        free_gap = float(self.get_parameter('raycast_free_gap').value)
        obs_b = np.zeros_like(obs)
        if free_gap > 0.0 and wd_all is not None:
            # (B) 도 같은 값으로 올린다. 자세가 틀어지면 **모든** 벽 반사가
            # 그만큼 떨어져 보이므로, 안 올리면 벽 전체가 장애물이 된다.
            obs_b = ahead & (wd_all > free_gap + skew)
            obs = obs | obs_b
        # ★ AMCL 마스킹은 **두 검사를 다 통과한 빔만** 쓴다 (아래 _amcl_mask 참고).
        #   회피는 하나만 걸려도 피하는 게 맞지만, AMCL 입력에서 빔을 지우는 건
        #   훨씬 위험하다 — 진짜 벽을 지우면 자세가 더 틀어진다.
        self._obs_beam_strict = idx[obs & obs_b]   # (B)

        # ★ AMCL 용 마스킹에 쓸 수 있도록 "어느 빔이 장애물인가" 를 남긴다.
        #   (idx 는 원본 ranges 의 인덱스. stride 로 솎은 것만 들어 있다)
        self._obs_beam_idx = idx[obs]
        self._obs_beam_n = int(len(idx))

        if not obs.any():
            return np.zeros((0, 2))
        r, a = meas[obs], (self.yaw + a_car)[obs]
        return np.vstack((lx0 + r * np.cos(a), ly0 + r * np.sin(a))).T
    def _predicted_track_path(self, horizon: float) -> np.ndarray:
        """앞으로 지나갈 경로를 "레이싱라인 + 현재 횡오차"로 예측 (map 좌표).

        ★ 왜 원호만으로는 부족한가
          원호 예측은 "지금 조향을 유지한다" 가정이라 코너 전후에서 실제 경로와
          어긋난다. 진입 직전엔 yaw rate 가 아직 안 붙어 직선에 가깝고, 탈출
          직후엔 아직 커서 과하게 안쪽으로 휜다. 그 사이 코너 안쪽 장애물을 놓친다.

          이 차는 Stanley 로 라인을 추종하므로 미래 경로는 "라인을 현재 횡오차만큼
          평행이동한 것" 에 가깝다. 라인의 곡률을 그대로 따르니 코너 기하가 자동으로
          반영된다. 라인을 판정 기준으로 쓰는 게 아니라 내 진로를 예측하는 데 쓴다.
        """
        wps = self.wps
        if wps is None or len(wps) < 3:
            return np.zeros((0, 2))
        n = len(wps)
        px, py = float(self.pos[0]), float(self.pos[1])
        near = int(np.argmin(np.hypot(wps[:, 0] - px, wps[:, 1] - py)))

        m = max(1, int(round(horizon / max(self._wp_ds, 1e-3))))
        m = min(m, n - 1)
        idx = (near + np.arange(m + 1)) % n
        seg = wps[idx, :2]

        # 현재 횡오차(부호 포함)를 라인 법선 방향으로 잰다
        j_f = (near + 1) % n
        tang = wps[j_f, :2] - wps[near, :2]
        nt = float(np.hypot(*tang))
        if nt < 1e-6:
            return seg
        tang = tang / nt
        left = np.array([-tang[1], tang[0]])
        cte = float((np.array([px, py]) - wps[near, :2]) @ left)

        return seg + left * cte           # 라인을 내 오차만큼 평행이동
    def _in_predicted_corridor(self, pts_world: np.ndarray) -> np.ndarray:
        """차가 실제로 지나갈 통로 안에 있는 점인가 (bool 배열).

        두 예측의 합집합이다 — 어느 하나만으로는 구간에 따라 빗나간다:
          (1) 순간 원호  : /odom yaw rate 로 R=v/w. 지금 조향 기준.
          (2) 라인+횡오차: 코너 기하를 따르는 예측 (_predicted_track_path).
        """
        if len(pts_world) == 0:
            return np.zeros(0, bool)

        d = pts_world - self.pos
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        fx = c * d[:, 0] - s * d[:, 1]        # 전방(+)
        fy = s * d[:, 0] + c * d[:, 1]        # 좌측(+)

        # ★ 슈미트 트리거 — 회피 중에는 통로를 넓혀서 본다.
        #   좁은 통로만 보면 회피 조향을 시작하는 순간 장애물이 통로 밖으로
        #   나가 감지가 풀리고, 라인으로 복귀하다 다시 감지되는 진동이 생긴다.
        #   해제 조건을 진입보다 느슨하게 두어 장애물을 지나칠 때까지 붙잡는다.
        scale = 1.0 if self.lane_free else self.obs_release_scale
        hw = self.obs_corr_hw * scale
        horizon = self._obs_horizon() * scale
        v, w = abs(self.speed), self.yaw_rate

        R = None
        if abs(w) > 1e-3 and v > 1e-3:
            R = v / w if self.speed >= 0 else -v / w
            if abs(R) > 1e3:
                R = None

        # (1) 순간 원호 통로
        if R is None:                          # 직진
            arc_in = (fx > 0.0) & (fx < horizon) & (np.abs(fy) < hw)
        else:
            dist = np.hypot(fx, fy - R)
            lateral = np.abs(dist - abs(R)) < hw
            sr = math.copysign(1.0, R)
            phi = np.mod(np.arctan2(sr * fx, sr * (R - fy)), 2.0 * math.pi)
            arc_in = lateral & (abs(R) * phi < horizon)

        # (2) 라인+횡오차 예측 경로 통로 — 코너 기하를 따라간다
        path = self._predicted_track_path(horizon)
        if len(path) == 0:
            return arc_in
        # 뒤쪽 점은 제외 (이미 지나친 장애물에 반응하지 않도록)
        ahead = fx > -0.05
        path_in = np.zeros(len(pts_world), bool)
        if ahead.any():
            dmin = scipy.spatial.distance.cdist(
                path, pts_world[ahead]).min(axis=0)
            path_in[ahead] = dmin < hw

        return arc_in | path_in
    # ── 동적 장애물(상대차) 추적 ────────────────────────────────────────
    #   1대1 에서는 장애물이 **움직인다.** 지금 자리를 피하면, 내가 거기 도달할
    #   때쯤 상대는 이미 다른 곳에 있다. 4㎧ 로 3m 앞의 상대에게 다가가면
    #   도달까지 0.75s 인데, 상대가 3㎧ 로 달리면 그동안 2.25m 를 간다 —
    #   **현재 위치를 피하는 것은 거의 의미가 없다.**
    #
    #   검출점을 클러스터로 묶어 프레임 간 대응시키고, 도달 시각까지
    #   **쓸고 지나갈 자리 전체**를 장애물로 넘긴다.
    #
    #   ★ 예측은 유클리드 등속(p+v t)이 아니라 **레이싱라인 s** 다.
    #     접선으로 밀면 현(chord)이라 곡률 구간에서 라인(과 벽) 밖으로 나간다.
    #     우리 차 진로는 이미 `_predicted_track_path` 가 라인+횡오차로 코너를
    #     따라가는데, 상대만 직선이면 Frenet 이 가짜 점을 피해 안쪽으로 들어간다.
    #     같은 기하를 상대에 쓴다: ṡ 로 라인을 따라 옮기고 d 는 유지.
    #     웨이포인트가 없으면 예전 유클리드로 폴백한다.
    opp_track     = True    # 동적 예측 사용
    opp_eps       = 0.30    # 클러스터 묶는 거리 [m]
    opp_gate      = 0.80    # 프레임 간 대응 게이트 [m]
    opp_alpha     = 0.6     # 속도 저역통과 (0=즉답, 1=안 움직임)
    opp_min_obs   = 3       # 이만큼 연속 관측돼야 속도를 믿는다
    opp_min_speed = 0.5     # 이보다 느리면 정적 장애물로 본다 [m/s]
    opp_pred_max  = 1.2     # 예측 시간 상한 [s]
    opp_lost_sec  = 0.5     # 이만큼 못 보면 트랙 폐기
    _next_tid     = 1       # persistent ID. 매칭이 끊기면 새 번호

    def _cluster_pts(self, pts, eps):
        """격자 해싱 클러스터링. 이웃 격자까지 합쳐 경계에서 갈라지지 않게 한다.

        DBSCAN 을 안 쓰는 이유는 40Hz 안에 끝나야 하기 때문이다 — 점이
        수백 개라 O(n log n) 라이브러리 호출보다 이쪽이 빠르고 의존성도 없다.
        """
        key = np.floor(pts / eps).astype(np.int64)
        lab = -np.ones(len(pts), np.int64)
        table = {}
        for i, k in enumerate(map(tuple, key)):
            table.setdefault(k, []).append(i)
        nxt = 0
        for k, idxs in table.items():
            if lab[idxs[0]] >= 0:
                continue
            # 이웃 격자를 너비우선으로 합친다
            stack, comp = [k], []
            seen = {k}
            while stack:
                c = stack.pop()
                if c not in table:
                    continue
                comp.extend(table[c])
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n = (c[0] + dx, c[1] + dy)
                        if n not in seen and n in table:
                            seen.add(n); stack.append(n)
            for i in comp:
                lab[i] = nxt
            nxt += 1
        return lab, nxt

    # ── 추월 가능 구간 ────────────────────────────────────────────────
    #   기본값. frenet_planner 가 파라미터로 덮어쓴다.
    ot_zone_on   = False   # 끄면 예전과 동일 (어디서든 회피)
    ot_width     = 1.10    # 나란히 지나가려면 필요한 복도 폭 [m]
    ot_exit_pad  = 2.50    # 좁아지기 전에 이만큼 미리 구간을 끝낸다 [m]
    ot_min_len   = 2.00    # 이보다 짧은 구간은 버린다 [m]

    # 추종(follow) 파라미터 — frenet_planner 가 덮어쓴다
    follow_gap   = 0.80    # 상대 뒤 이 간격을 유지한다 [m] (범퍼 사이가 아니라 s 차)
    follow_k     = 1.2     # 간격 오차 → 속도 보정 이득 [1/s]
    follow_min   = 0.25    # 속도 배율 하한

    # ── 회피 오프셋 유지 (pass-through hold) ──────────────────────────
    hold_on      = False   # 켜면 회피를 시작한 뒤 다 지나갈 때까지 유지한다
    hold_car_len = 0.50    # 내 차 길이 [m]
    hold_obj_len = 0.50    # 피할 대상 길이 [m] (상대차 기준. 콘이면 훨씬 짧다)
    hold_margin  = 0.30    # 여유 [m]
    hold_max_sec = 3.0     # 상한 [s] — 잘못 물리면 영영 안 풀리는 것을 막는다

    # 테스트용 하이브리드. yaml 로 끈다 (static_lock / candidate_fb).
    static_lock  = False   # 08/15 기준. 켜면 정지 장애물 d 고정
    candidate_fb = False   # 08/15 기준. 켜면 짧은 FB 통과시간으로 고른다
    fb_horizon   = 5.0     # FB 에 넣을 전방 거리 [m]
    fb_iters     = 2       # solve_profile 반복. 2면 50Hz 에 충분

    def _horizon_xy(self, xy, horizon):
        """후보 경로를 전방 horizon [m] 까지만 자른다."""
        xy = np.asarray(xy, float)
        if len(xy) < 3:
            return xy
        ds = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
        s = np.concatenate([[0.0], np.cumsum(ds)])
        m = int(np.searchsorted(s, float(horizon), side='left'))
        m = int(np.clip(m, 2, len(xy) - 1))
        return xy[:m + 1]

    def _candidate_fb_times(self, cands, feasible):
        """실행가능 후보의 짧은 열린 경로 FB 통과시간 [s]. 불가면 inf."""
        n = len(cands)
        T = np.full(n, np.inf)
        horizon = float(getattr(self, 'fb_horizon', 5.0) or 5.0)
        iters = max(int(getattr(self, 'fb_iters', 2) or 2), 1)
        mu = float(getattr(self, 'mu', 0.348) or 0.348)
        pac_d = float(getattr(self, 'pac_d', 1.15) or 1.15)
        kwargs = dict(
            mu_eff=mu * pac_d,
            grip_util=float(getattr(self, 'grip_util', 0.6) or 0.6),
            lf=float(getattr(self, 'lf', 0.1757) or 0.1757),
            lr=float(getattr(self, 'lr', 0.1441) or 0.1441),
            v_min=float(getattr(self, 'min_speed', 0.3) or 0.3),
            a_lon_acc=float(getattr(self, 'a_lon_acc', 0.0) or 0.0),
            a_lon_brk=float(getattr(self, 'a_lon_brk', 0.0) or 0.0),
            hard_cap=float(getattr(self, 'hard_cap', 0.0) or 0.0),
            closed=False,
            iters=iters,
        )
        for i in range(n):
            if not bool(feasible[i]):
                continue
            xy = self._horizon_xy(cands[i], horizon)
            if xy is None or len(xy) < 3:
                continue
            try:
                _, info = solve_profile(xy[:, 0], xy[:, 1], None, **kwargs)
                t = float(info.get('lap_time', np.inf))
            except Exception:
                continue
            if np.isfinite(t) and t > 0.0:
                T[i] = t
        return T

    def _is_static_scene(self):
        """앞에 있는 것이 전부 정지면 True. 성숙 트랙이 없으면 콘으로 본다."""
        if getattr(self, '_ot_static', False):
            return True
        trs = [t for t in (getattr(self, '_opp_tracks', []) or [])
               if t.get('n', 0) >= int(getattr(self, 'opp_min_obs', 3) or 3)]
        n_pts = self._n_on_track()
        if not trs:
            return n_pts > 0
        vmin = float(getattr(self, 'opp_min_speed', 0.5) or 0.5)
        return all(abs(float(t.get('s_dot', 0.0))) < vmin for t in trs)

    def _n_on_track(self) -> int:
        """트랙 안에서 검출된 점 개수.

        ★ `getattr(self, 'on_track_pts', ()) or ()` 로 쓰면 안 된다 (2026-08-16).
          on_track_pts 는 **numpy 배열**이라(np.zeros((0,2)) 또는 np.asarray)
          `or` 가 bool(배열) 을 부르고, 원소가 2개 이상이면 numpy 가
          ValueError: truth value of an array ... is ambiguous 를 던진다.
          shape (1,2) 도 원소 2개라 **장애물 하나만 잡혀도** 터진다.

          2026-08-17 실차에서 이걸로 죽었다: q2 주행 255초 내내 /frenet/* 가
          한 건도 안 나갔다. 노드가 뜨자마자 첫 장애물에서 예외 → 타이머
          콜백이 죽어 executor 종료. 차는 d=0 으로 멀쩡히 달리므로
          **로그만 봐서는 정상으로 보인다** — 이 저장소가 반복해 맞는 모양이다
          (08/09 bench, 08/13 race 미검출, 08/15 Marker 미import).
        """
        pts = getattr(self, 'on_track_pts', None)
        return 0 if pts is None else int(len(pts))

    def _apply_static_lock(self, d, feasible, d_raw, best):
        """정지 장애물에서 고른 d 를 고정한다. (d_cmd, best_index).

        17후보를 매 프레임 다시 고르면 콘 정면에서 좌우가 뒤집힌다. hold 는
        FOV 손실용이고, 이쪽은 **아직 보이는 동안** 도 같은 d 를 유지한다.
        잠긴 후보가 더 이상 실행가능이 아니거나 장애물이 사라지면 푼다.
        """
        n_obs = self._n_on_track()
        lock = getattr(self, '_lock_d', None)
        lost = int(getattr(self, '_lock_lost', 0) or 0)
        if not bool(getattr(self, 'static_lock', False)):
            self._lock_d = None
            self._lock_lost = 0
            return float(d_raw), int(best)
        if n_obs <= 0:
            lost += 1
            self._lock_lost = lost
            if lost >= 3:
                self._lock_d = None
                self._lock_lost = 0
            elif lock is not None:
                i = int(np.argmin(np.abs(d - float(lock))))
                if bool(feasible[i]):
                    return float(d[i]), i
            return float(d_raw), int(best)
        self._lock_lost = 0
        if not self._is_static_scene():
            self._lock_d = None
            return float(d_raw), int(best)
        if lock is not None:
            # ★★ 2026-08-17 — **부호만** 잠근다. 예전엔 값을 통째로 잠갔다.
            #   실차 q2 (20260817_005738 / _005431): 오프셋이 **첫 검출 순간의
            #   값에 얼어붙었다** — 접근 에피소드 9/14, 13/14 가 첫 값 그대로.
            #     첫 검출 2.5m 에서 |d|=0.100  →  최근접 0.026m 에서도 0.100
            #   2.5m 밖에서는 콘이 빔 몇 개만 스쳐 작은 오프셋이 충분해 보인다.
            #   가까워지며 해소돼도 다시 안 골라서 그 값이 끝까지 간다.
            #   콘을 비키려면 차 반폭 0.155 + 콘 반경 ~0.1 + 여유 = 0.30m 은
            #   있어야 하는데 gap<0.4m 에서 |d| 중앙이 0.200 이었고,
            #   |d|<0.15 인 틱이 35% / 86% 였다. 최소 gap 0.009m — 박았다.
            #   팀 보고 "감지가 잘 안 되기 시작하면 그냥 냅다 박아버리네".
            #
            #   잠금의 원래 목적은 **콘 정면에서 좌우가 뒤집히는 것**을 막는
            #   것이지 크기를 얼리는 게 아니다. 그래서:
            #     · 부호는 유지한다 (뒤집힘 방지 — 원래 목적)
            #     · 크기는 줄지 않게만 하고 **커지는 건 허용한다** (래칫)
            #   해소되며 작은 후보가 실행불가가 되면 선택이 저절로 커진다.
            #   ⚠ 2026-08-17 두 번째 시도 — **래칫은 틀렸다.** 크기를 줄지
            #     않게 하니 오프셋이 커지기만 하고 안 돌아왔다. 벽 오검출로
            #     n_obs 가 계속 >0 이라 잠금이 풀리지도 않는다.
            #       실차 20260817_013016 vs 앞 판 005738:
            #         |d|≥0.55 인 시간   0% → 23%
            #         TRACK(깨끗)      6.6% → 0.5%
            #         CTE p90        0.738 → 1.746m  (최대 2.003m)
            #     팀 보고 "그냥 박고 맵대로 안 움직였다". 10초 만에 중단.
            #
            #   목적은 **좌우 뒤집힘을 막는 것** 하나다. 그러니 부호가 뒤집힌
            #   선택만 거부하고, 크기와 라인 복귀(d=0)는 손대지 않는다.
            #   가까워지며 작은 후보가 실행불가가 되면 선택이 저절로 커진다 —
            #   그게 원래 동작이고 08/15 에는 0.60m 까지 잘 나갔다.
            sgn = 1.0 if float(lock) >= 0.0 else -1.0
            if bool(feasible[best]) and d[best] * sgn >= -1e-9:
                i = int(best)                      # 같은 쪽이거나 0 — 그대로
            else:
                cand = np.where(feasible & (d * sgn >= -1e-9))[0]
                if len(cand) == 0:
                    self._lock_d = None
                    return float(d_raw), int(best)
                i = int(cand[np.argmin(np.abs(d[cand] - d[best]))])
            # 라인으로 돌아왔으면 잠금을 푼다 (안 그러면 영영 못 돌아온다)
            self._lock_d = float(d[i]) if abs(float(d[i])) > 0.05 else None
            return float(d[i]), i
        if abs(float(d_raw)) > 0.05:
            self._lock_d = float(d_raw)
        return float(d_raw), int(best)

    def _update_hold(self, d_cmd):
        """옆으로 나간 뒤 **다 지나갈 때까지** 오프셋을 유지한다.

        ★ 왜 필요한가 — 나란히 달리는 동안 상대가 **시야에서 사라진다.**
          라이다는 base_link 앞 0.27m 에 있고 화각이 ±70° 다. 상대가 옆
          0.4m 에 있을 때:
              종방향 1.00m →  29°  보임
              종방향 0.60m →  50°  보임
              종방향 0.40m →  72°  ★ 안 보임
              종방향 0.00m → 124°  ★ 안 보임
          즉 **추월의 마지막 구간에서 상대를 전혀 못 본다.** 검출이 끊기면
          비용의 w_offset 이 라인으로 되돌리므로, 아직 나란히 있는데 옆구리를
          들이받는 쪽으로 조향한다.

        ★ 왜 콘에서는 문제가 안 됐나 — 지나가는 데 걸리는 시간이 다르다.
          필요한 **상대 종방향 변위** = 내 길이 + 대상 길이 + 여유
              콘  (0.2m) : 1.0m / 접근속도 4㎧  → 0.25s
              상대차(0.5m): 1.3m / 접근속도 1㎧  → 1.30s   ← 5배
          접근속도는 v_ego − v_opp 라 상대가 빠를수록 더 오래 걸린다.
          검출이 끊기는 시간(≈0.15s)이 콘에서는 짧아 티가 안 났다.

        유지 시간은 매 틱 **남은 변위 ÷ 접근속도**로 다시 잡는다. 고정 시간으로
        두면 접근속도가 변할 때 모자라거나 남는다.
        """
        if not self.hold_on:
            return d_cmd
        now = float(getattr(self, '_now_sec', 0.0) or 0.0)
        need = self.hold_car_len + self.hold_obj_len + self.hold_margin

        # 지금 막고 있는 것 중 가장 가까운 것의 접근속도
        closing, seen = None, False
        for tr in getattr(self, '_opp_tracks', []) or []:
            if tr.get('n', 0) < self.opp_min_obs:
                continue
            seen = True
            c = abs(float(getattr(self, 'speed', 0.0) or 0.0)) - float(tr.get('s_dot', 0.0))
            closing = c if closing is None else min(closing, c)

        if abs(d_cmd) > 1e-3 and seen:
            # 회피 중이고 아직 보인다 — 만료 시각을 계속 갱신한다
            cl = max(closing if closing is not None else 1e-3, 0.2)
            self._hold_until = now + min(need / cl, self.hold_max_sec)
            self._hold_d = float(d_cmd)
            return d_cmd

        # 안 보이는데 유지 기간이 남았다 → 방금 옆에 붙은 것이다. 붙잡는다.
        until = float(getattr(self, '_hold_until', 0.0) or 0.0)
        if now < until and abs(float(getattr(self, '_hold_d', 0.0))) > 1e-3:
            return float(self._hold_d)
        return d_cmd

    def _set_planner_mode(self, d_raw, d_out, n_valid):
        """ot_zone / hold / 후보 유무를 한곳에서 모드로 접는다.

        숫자가 흩어져 있으면 조합 버그가 난다 — 예전엔 status>=2 가
        FOLLOW(3)까지 최저속도로 깎았다.
        """
        hold_on = False
        now = float(getattr(self, '_now_sec', 0.0) or 0.0)
        until = float(getattr(self, '_hold_until', 0.0) or 0.0)
        if now < until and abs(float(getattr(self, '_hold_d', 0.0))) > 1e-3:
            hold_on = True
        static = self._is_static_scene()
        locked = getattr(self, '_lock_d', None) is not None
        if getattr(self, '_fr_blocked', False) or int(n_valid) <= 0:
            mode = BLOCKED
        elif self.ot_zone_on and not getattr(self, '_ot_allowed', True):
            mode = FOLLOW
        elif hold_on and abs(float(d_raw)) < 0.05 and abs(float(d_out)) >= 0.05:
            mode = RECOVER
        elif (static or locked) and abs(float(d_out)) >= 0.05:
            mode = AVOID_STATIC
        elif abs(float(d_out)) >= 0.05:
            mode = OVERTAKE
        else:
            mode = TRACK
        self._fr_status = float(mode)
        self._fr_mode = int(mode)

    def _lead_opp(self):
        """전방 가장 가까운 트랙. 없으면 None."""
        lut = self._s_lut()
        pos = getattr(self, 'pos', None)
        if lut is None or pos is None:
            return None
        s_ego, _ = self._project_sd(np.asarray(pos, float)[:2], lut)
        best = None
        for tr in getattr(self, '_opp_tracks', []) or []:
            if tr.get('n', 0) < self.opp_min_obs:
                continue
            gap = (float(tr.get('s', 0.0)) - s_ego) % lut['L']
            if gap > 0.5 * lut['L']:
                continue
            if best is None or gap < best[0]:
                best = (gap, tr)
        return None if best is None else best[1]

    def frenet_diag_dict(self, scale, planning_ms):
        """로그·/frenet/diag 용 스냅샷. loc_residual 은 컨트롤러가 붙인다."""
        lead = self._lead_opp()
        ttc = float(getattr(self, '_fr_ttc', 1e9) or 1e9)
        if lead is not None:
            lut = self._s_lut()
            if lut is not None and self.pos is not None:
                s_ego, _ = self._project_sd(np.asarray(self.pos, float)[:2], lut)
                gap = (float(lead.get('s', 0.0)) - s_ego) % lut['L']
                closing = max(abs(float(getattr(self, 'speed', 0.0) or 0.0))
                              - float(lead.get('s_dot', 0.0)), 0.1)
                ttc = min(ttc, gap / closing)
        nan = float('nan')
        return {
            'selected_d':     float(getattr(self, '_fr_best_d', 0.0) or 0.0),
            'speed_scale':    float(scale),
            'planner_status': float(getattr(self, '_fr_status', 0.0) or 0.0),
            'ot_allowed':     1.0 if getattr(self, '_ot_allowed', True) else 0.0,
            'n_candidates':   float(getattr(self, '_fr_n_cand', 0) or 0),
            'n_valid':        float(getattr(self, '_fr_n_valid', 0) or 0),
            'n_obs':          float(getattr(self, '_fr_n_obs', 0) or 0),
            'min_ttc':        float(ttc),
            'planning_ms':    float(planning_ms),
            'opp_id':         float(lead.get('id', 0) if lead else nan),
            'opp_s_dot':      float(lead.get('s_dot', nan) if lead else nan),
            'opp_d_lat':      float(lead.get('d_lat', nan) if lead else nan),
            'opp_vd':         float(lead.get('d_dot', nan) if lead else nan),
        }

    # ── 회피 완료 가능성 속도 상한 ────────────────────────────────────
    shift_cap_on   = True    # 끄면 예전 동작
    shift_l1_min   = 1.32    # L1 = max(l1_min, l1_k·v). 컨트롤러와 같은 값이어야 한다
    shift_l1_k     = 0.75
    shift_rate_ref = 2.0     # 기준선 램프 속도 [m/s] (frenet.shift_rate 와 같게)
    shift_safety   = 1.25    # 여유 배수
    shift_min_v    = 0.8     # 이 아래로는 안 깎는다 [m/s]
    # 남은 이동량이 이보다 작으면 "다 옮겼다" 로 보고 상한을 푼다 [m] (2026-08-17)
    shift_done     = 0.05
    # ★ 움직이는 상대에게는 gap 을 접근속도로 환산한다 (2026-08-16). 아래 참조.
    shift_close_min = 0.30   # 접근속도 하한 [m/s] — 증폭 상한 = v/이 값
    shift_close_max = 6.0    # gap 증폭 배수의 절대 상한
    shift_close_r   = 0.60   # gap 을 정한 점에서 이 반경 안의 추적만 본다 [m]
    shift_det_m    = 2.70    # 장애물이 **보이기 시작하는** 거리 [m] (실측)
    shift_brake    = 0.0     # 제동 능력 [m/s²]. 0 = μ_eff·g 로 자동
    shift_preempt  = True    # 보이기 전에도 접근속도를 제한할지

    def _approach_speed_cap(self):
        """장애물이 **보이기 전에** 걸어두는 접근속도 상한 [m/s].

        ★ 왜 필요한가 (2026-08-15 실차: "지금이 딱 마지노선으로, 부딪히지 않을
          마지막 순간에 회피한다. 대회장에서 마찰이 좋아져 속도가 오르면 못 피할
          수도 있다" — 정확한 예측이다.)

          검출 거리는 **하드웨어가 정한다.** 스캔 평면이 base_link 위 11cm 라
          낮은 콘은 가까이 와야 보인다. 실측 회피 시작 거리 중앙 2.76m 로
          속도와 무관하게 일정했다. 그런데 남는 **시간**은 속도에 반비례한다:

              회피 시작 시 남는 시간 (오늘 실측)
                2.81m @ 3.82㎧ → 0.74s
                0.42m @ 4.04㎧ → 0.10s   ★
                0.81m @ 4.57㎧ → 0.18s   ★
              중앙 1.01s

          _shift_feasible_speed 는 **보인 뒤에** 속도를 깎는데, 그때는 이미
          제동거리가 모자란다:

              접근속도  옆으로 옮기기  제동거리  합계   가용 2.76m
                2.0㎧     2.21m      0.22m   2.43m    ✓
                3.0㎧     3.10m      0.62m   3.72m    ★ 모자람
                5.0㎧     5.01m      1.80m   6.81m    ★ 모자람

          그래서 **미리** 건다. "지금 이 속도에서 2.7m 앞에 콘이 나타나도
          피할 수 있는가" 를 항상 만족시킨다. 못 하면 그 속도로 달리지 않는다.

        ※ q2·race 처럼 장애물이 있을 수 있는 모드에서만 의미가 있다.
          장애물이 없으면 이 상한이 랩타임을 그대로 깎아먹으므로,
          shift_preempt 를 꺼서 예전 동작으로 돌릴 수 있다.
        """
        if not (self.shift_cap_on and self.shift_preempt):
            return float('inf')
        det = float(self.shift_det_m)
        if det <= 0.0:
            return float('inf')
        brk = float(self.shift_brake)
        if brk <= 0.0:
            brk = (float(getattr(self, 'grip_util', 1.0))
                   * float(getattr(self, 'mu', 0.348))
                   * float(getattr(self, 'pac_d', 1.15)) * GRAVITY)
        brk = max(brk, 0.5)
        room = det / max(self.shift_safety, 1e-3)

        # v 를 위에서부터 훑어 "제동거리 + 옆으로 옮기는 거리 ≤ room" 인 최대값.
        #   닫힌 해가 없다 (L1 이 v 에 걸리고 목표 속도도 v 에 걸린다).
        #   격자 탐색이 가장 단순하고, 20개면 0.25㎧ 해상도라 충분하다.
        best = self.shift_min_v
        for v in np.linspace(self.shift_min_v, 8.0, 32):
            L1 = max(self.shift_l1_min, self.shift_l1_k * v)
            rise = 1.8 * L1 / math.sqrt(2.0)
            # 그 속도에서 옆으로 다 옮기는 데 드는 거리
            need = rise + v * self.fr_dmax / max(self.shift_rate_ref, 1e-3)
            if need <= room:
                best = v
            else:
                break                      # v 에 단조증가라 더 볼 것 없다
        return best

    def _closing_stretch(self, v_now, blk_xy):
        """gap 을 **접근속도 기준**으로 늘리는 배수. 정지 물체면 1.0.

        blk_xy 는 gap 을 정한 **그 점**이다. 트랙 전체가 아니라 그 점 근처의
        추적만 본다 — 뒤에 남은 클러스터나 반대편의 콘 하나가 배수를 1.0 으로
        눌러버리면 이 수정이 조용히 아무 일도 안 하게 된다.

        내가 앞의 물체에 실제로 닿기까지 달리는 거리는

            거리 = gap × v_ego / (v_ego − ṡ_opp)

        이다. ṡ_opp = 0 (콘) 이면 배수 1.0 으로 옛 동작 그대로고, 상대가
        내 속도에 가까우면 배수가 커진다 — 그만큼 옆으로 옮길 시간이 있다.

        ★ 안전 쪽으로만 보수적으로 잡는다:
          · 앞을 막은 것들 중 **가장 빠른** 게 아니라 **가장 느린** ṡ 를 쓴다.
            여럿이 섞여 있으면 제일 안 비켜주는 놈이 기준이다.
          · 추적이 덜 된 것(n < opp_min_obs)이 하나라도 있으면 **1.0** 을
            준다. 상대차를 정지로 오판하는 것보다 그 반대가 훨씬 싸다.
          · ṡ 가 opp_min_speed 미만이면 정지로 보고 1.0 (기존 판정과 같은 선).
          · 접근속도에 하한을 두고(shift_close_min) 배수도 상한
            (shift_close_max)으로 묶는다. 0 근처에서 나누면 상한이 무한대로
            튀어 이 안전장치가 통째로 사라진다.
        """
        trs = [t for t in (getattr(self, '_opp_tracks', []) or [])
               if np.hypot(*(np.asarray(t['p'], float) - blk_xy))
               <= self.shift_close_r]
        if not trs:
            return 1.0                       # 막은 게 뭔지 모른다 — 정지로 취급
        if any(t.get('n', 0) < self.opp_min_obs for t in trs):
            return 1.0                       # 아직 못 믿는다 — 정지로 취급
        s_dot = min(float(t.get('s_dot', 0.0)) for t in trs)
        if s_dot < self.opp_min_speed:
            return 1.0                       # 정지 장애물 — 옛 동작
        closing = max(v_now - s_dot, self.shift_close_min)
        return float(min(v_now / closing, self.shift_close_max))

    def _shift_feasible_speed(self, px, py, d_cmd):
        """**옆으로 다 옮길 수 있는** 속도 상한 [m/s]. 제한이 없으면 inf.

        ★ 왜 필요한가 (2026-08-15 실차: "q2 가 너무 과감해져서 계속 박았다")
          콘은 **2m 안에서야 검출된다.** 스캔 평면이 base_link 위 11cm 라
          낮은 콘은 멀리서 스캔을 스치지 못한다 (하드웨어 한계).
          실측: 라인(d=0)의 장애물 여유가
              장애물까지 2~4m → 1.1~1.2m (아직 안 막힘. 옆 물체다)
              장애물까지 0~2m → 0.15m   (여기서야 막힌다)

          그런데 MAP 이 오프셋을 다 옮기려면 **1.8·L1/√2 + v·|d|/shift_rate**
          를 달려야 한다. 3.2㎧ 면 L1=2.4m → 필요 4.0m. **2m 로는 절반이다.**

          예전에는 통과속도를 경로 **전체** 곡률로 구해서(이중 감속) 우연히
          느려졌고 그게 시간을 벌어줬다. 그걸 spd_span 으로 고치자 q2 가
          제 속도로 달리며 들이받게 됐다 — 원인 하나를 고치니 그것에 기대던
          다른 것이 드러난 것이다.

          그래서 **명시적으로** 건다:
              gap ≥ 1.8·L1/√2 + v·|d|/shift_rate   (× 여유배수)
          를 v 에 대해 푼다. L1 은 현재 속도로 한 번 평가한다(고정점 1스텝).

        ※ 감속이 아니라 **상한**이다. 회피가 필요 없으면 inf 라 아무 영향이 없다.

        ★★ 2026-08-16 — gap 을 **접근속도로 환산**한다 (1대1 실차에서 드러남).
          위 식의 gap 은 "내가 그 지점에 닿기까지 달릴 거리" 라는 뜻이다.
          정지한 콘이면 맞다. **상대차에는 완전히 틀린다.**

          car2 를 앞세우고 돌린 두 판(20260816_004150 / _004418):
            · 오프셋은 0.600m 까지 잘 나갔다 (교전 16회 중 7회)
            · 그런데 |d| ≥ 0.45 인 동안 속도 배율이 **중앙 0.45,
              0.5 미만인 시간이 100%** — 옆으로 나가는 순간 바닥을 친다
            · 그래서 접근속도가 −0.14 ~ +0.16㎧. 나란히 8~10초를 달리고 끝
          팀 보고: "상대가 느리면 잘 하는데, 상대속도가 크지 않으면 그냥
          뒤에서 속도만 줄이면서 추월을 못했다." 그 고리가 여기다.

          계산해 보면 걸릴 수밖에 없다:
            v 1.8㎧ → L1 1.35m → rise 1.72m
            gap 1.5m / 1.25 = 1.20m  →  room = −0.52  →  shift_min_v 강제
          **상대가 2.15m 안에 들어오면 무조건 바닥이다.**

          실제로 닿기까지 달리는 거리는 gap 이 아니라 gap·v/접근속도 다:
            gap 1.5m, v 1.8㎧, 접근 0.16㎧  →  16.9m. rise 1.72m 의 열 배다.

          그리고 이건 **자기강화**였다 — 감속하면 접근속도가 더 줄고, 그러면
          실제 여유는 더 늘어나는데 식이 그걸 모르니 계속 바닥에 머문다.
          영원히 나란히 달린다.

          ※ 접근속도에 하한(shift_close_min)을 두고 증폭도 상한
            (shift_close_max)으로 묶는다. s_dot 은 추적 잡음이 섞이는데,
            0 근처에서 나누면 상한이 무한대로 튀어 **이 안전장치가 통째로
            무력화된다.** 정지 장애물(추적 없음/ṡ≈0)은 옛 동작 그대로다.
        """
        # ★★ 2026-08-17 — **남은** 이동량으로 건다 (실차 q2 에서 드러남).
        #   이 상한의 뜻은 "옆으로 다 옮길 시간을 벌어라" 다. 그런데 판정에
        #   목표 오프셋 |d_cmd| 를 통째로 써서, **이미 다 옮긴 뒤에도** 계속
        #   걸려 있었다. 옮기고 나면 그 차선은 이미 비어 있으니 늦출 이유가 없다.
        #
        #   실측(20260817_005738 / _005431, q2):
        #     배율이 정확히 바닥(0.45)인 시간 = 주행 중 55% / 65%
        #     |d|>0.05 인 시간 91% / 89%,  TRACK(깨끗)은 7% / 3% 뿐
        #     모드별  AVOID_STATIC 60% 배율 0.45  ↔  OVERTAKE 24% 배율 1.00
        #   팀 보고 "장애물이 있을 때 확 멈췄다가 잘 가고" — 그 진동이 이것이다.
        #
        #   왜 상시로 걸렸나: v 1.8㎧ 면 rise = 1.8·L1/√2 = 1.72m 라
        #   gap < 1.25×1.72 = 2.15m 이면 무조건 shift_min_v 다. 그런데 **콘은
        #   2.7m 안에서야 보인다**(스캔 평면 11cm 한계). 즉 콘을 보는 거의
        #   모든 순간이 곧 바닥이었다. 여기에 static_lock(4d45353)이 d 를
        #   잠가 두니 바닥이 풀리지도 않았다.
        #
        #   → 남은 이동량 rem = |d_cmd − _shift_cur| 로 판정한다.
        #     옮기는 중이면 08/15 설계 그대로 걸리고, 다 옮겼으면 풀린다.
        #     목표가 바뀌면 rem 이 다시 커져 저절로 다시 걸린다.
        rem = abs(float(d_cmd) - float(getattr(self, '_shift_cur', 0.0) or 0.0))
        if not self.shift_cap_on or abs(d_cmd) < 1e-3 or rem < self.shift_done:
            return float('inf')
        pts = getattr(self, 'on_track_pts', None)
        if pts is None or not len(pts):
            return float('inf')
        dd = np.hypot(pts[:, 0] - px, pts[:, 1] - py)
        i_blk = int(np.argmin(dd))
        gap = float(dd[i_blk])
        v_now = max(abs(float(getattr(self, 'speed', 0.0) or 0.0)), 0.5)
        # 막은 것이 **움직이는 상대**면 실제로 닿기까지 달리는 거리는 gap 보다 길다
        gap *= self._closing_stretch(v_now, np.asarray(pts[i_blk][:2], float))
        L1 = max(self.shift_l1_min, self.shift_l1_k * v_now)
        rise = 1.8 * L1 / math.sqrt(2.0)
        room = gap / max(self.shift_safety, 1e-3) - rise
        if room <= 0.0:
            return self.shift_min_v          # 이미 늦었다 — 최대한 줄인다
        v_cap = room * self.shift_rate_ref / rem
        return max(v_cap, self.shift_min_v)

    def _follow_scale(self, px, py):
        """추월 금지 구간에서 **상대 뒤를 따라가는** 속도 배율.

        내가 더 빠르니 그냥 두면 들이받는다. 옆으로 못 피하는 구간이므로
        종방향으로 푸는 수밖에 없다.

            v_target = ṡ_opp + follow_k · (gap − follow_gap)
            scale    = clip(v_target / v_profile, follow_min, 1.0)

        · gap 이 목표보다 크면 상대보다 빨리 가서 좁힌다 (추월 구간까지 붙어간다)
        · gap 이 목표보다 작으면 상대보다 느려져 벌린다
        · 상대가 없으면 1.0 (프로파일 그대로)

        ★ 위로는 절대 안 올린다 (clip 상한 1.0). 프로파일은 마찰 한계이고,
          상대를 쫓는다고 그걸 넘으면 미끄러진다.
        """
        lut = self._s_lut()
        if lut is None:
            return 1.0
        best = None
        s_ego, _ = self._project_sd(np.array([px, py], float), lut)
        for tr in getattr(self, '_opp_tracks', []) or []:
            if tr.get('n', 0) < self.opp_min_obs:
                continue
            gap = (float(tr.get('s', 0.0)) - s_ego) % lut['L']
            if gap > 0.5 * lut['L']:
                continue                       # 뒤에 있다 — 신경 쓸 것 없다
            if best is None or gap < best[0]:
                best = (gap, float(tr.get('s_dot', 0.0)))
        if best is None:
            return 1.0
        gap, s_dot = best
        if gap > self._obs_horizon():
            return 1.0                          # 아직 멀다
        v_prof = max(float(getattr(self, '_fr_v_ref', 0.0) or 0.0), 1e-3)
        v_tgt = max(s_dot + self.follow_k * (gap - self.follow_gap), 0.0)
        self._follow_gap = gap

        # ★★ 2026-08-15 — 하한(follow_min)을 **움직이는 상대에게만** 건다.
        #   08/15 race 주행에서 차가 정지 장애물을 **25% 속도로 들이받았다.**
        #     · 추월 금지 구간이라 d=0 (옆으로 안 나감)
        #     · 정지 장애물이라 ṡ=0 → v_target = 1.2·(gap−0.8) → gap 0.8 에서 0
        #     · 그런데 clip 하한이 0.25 라 **0 이 되지 못하고 계속 전진**
        #   하한의 뜻은 "앞차를 쫓다가 너무 느려지면 영영 못 잡는다" 였다.
        #   상대가 **멈춰 있으면 쫓을 이유가 없다** — 서야 한다.
        #   서고 나면 회피가 필요한데 금지 구간이면 그것도 못 하므로, 그때는
        #   status 2 로 올려 사람이 알 수 있게 한다 (조용히 서 있지 않도록).
        lo = self.follow_min if abs(s_dot) >= self.opp_min_speed else 0.0
        if lo <= 0.0 and v_tgt <= 1e-3:
            self._fr_status = 2.0               # 정지 장애물 앞 — 막힘
            self.get_logger().warn(
                f'[frenet] 정지 장애물 {gap:.2f}m 앞 — 추월 금지 구간이라 '
                f'옆으로 못 나간다. 정지한다',
                throttle_duration_sec=1.0)
        return float(np.clip(v_tgt / v_prof, lo, 1.0))

    def _overtake_zone(self):
        """레이싱라인 각 점에서 **추월을 시작해도 되는가** (bool 배열).

        ★ 왜 필요한가 (2026-08-15)
          회피는 시작보다 **끝내는 것**이 어렵다. 넓은 구간 끝에서 옆으로
          나가기 시작하면, 다 지나가기 전에 좁은 구간을 만나 **상대와 벽
          사이에 끼인다.** 옆으로 피할 곳이 없으니 그때는 손쓸 방법이 없다.

          그래서 맵에서 미리 정한다:
            ① 복도 폭 ≥ ot_width 인 구간을 찾는다 (두 대가 나란히 갈 수 있는 곳)
            ② 그 구간의 **끝에서 ot_exit_pad 만큼 잘라낸다** — 추월을 시작해
               끝내는 데 드는 거리다. 여기서 시작하면 좁은 데서 끝난다
            ③ 남은 길이가 ot_min_len 보다 짧으면 통째로 버린다

          구간 밖에서는 옆으로 안 나가고 **상대 뒤를 따라간다** (속도를 맞춘다).
          내가 더 빠르니 그냥 두면 들이받는다 — 횡회피가 금지된 곳에서는
          종방향으로 푸는 수밖에 없다.

        복도 폭은 **맵 거리변환**으로 잰다 (라인에서 벽까지 × 2). 경계 윤곽이
        아니라 거리장을 쓰는 이유는 중심선 추출과 같다 — 인필드가 치우쳐도
        영향받지 않는다.
        """
        wps = getattr(self, 'wps', None)
        lut = self._s_lut()
        if wps is None or lut is None:
            return None
        cached = getattr(self, '_ot_cache', None)
        if cached is not None and cached[0] is wps:
            return cached[1]

        xy = np.asarray(wps[:, :2], float)
        ok = overtake_zone(xy, self._wall_room(xy),
                           self.ot_width, self.ot_exit_pad, self.ot_min_len)
        self._ot_cache = (wps, ok)
        return ok

    def _s_lut(self):
        """닫힌 레이싱라인의 s·법선. wps 가 그대로면 캐시."""
        wps = getattr(self, 'wps', None)
        cached = getattr(self, '_s_lut_cache', None)
        if wps is None or len(wps) < 3:
            return None
        if cached is not None and cached[0] is wps:
            return cached[1]
        xy = np.asarray(wps[:, :2], float)
        n = len(xy)
        ds = np.hypot(np.roll(xy[:, 0], -1) - xy[:, 0],
                      np.roll(xy[:, 1], -1) - xy[:, 1])
        ds = np.maximum(ds, 1e-6)
        s = np.concatenate(([0.0], np.cumsum(ds[:-1])))
        fwd = np.roll(xy, -1, axis=0) - xy
        tn = np.maximum(np.hypot(fwd[:, 0], fwd[:, 1]), 1e-9)
        tang = fwd / tn[:, None]
        left = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
        lut = dict(xy=xy, ds=ds, s=s, L=float(ds.sum()), left=left, n=n)
        self._s_lut_cache = (wps, lut)
        return lut

    @staticmethod
    def _wrap_ds(ds, L):
        """(-L/2, L/2] 로 접은 종방향 변위."""
        L = max(float(L), 1e-6)
        ds = (float(ds) + 0.5 * L) % L - 0.5 * L
        return ds

    def _project_sd(self, p, lut):
        """맵 좌표 → (s, d_lat). s 는 라인 원점부터의 호장 [m], d 좌측 +."""
        xy = lut['xy']
        i = int(np.argmin((xy[:, 0] - p[0]) ** 2 + (xy[:, 1] - p[1]) ** 2))
        rel = np.asarray(p, float) - xy[i]
        j = (i + 1) % lut['n']
        fwd = xy[j] - xy[i]
        u2 = float(fwd @ fwd)
        t = float(rel @ fwd) / u2 if u2 > 1e-12 else 0.0
        t = float(np.clip(t, 0.0, 1.0))
        s = float(lut['s'][i] + t * lut['ds'][i])
        d_lat = float(rel @ lut['left'][i])
        return s, d_lat

    def _pose_at_s(self, s, d_lat, lut):
        """라인 위 s 에서 법선으로 d_lat 만큼 민 맵 좌표."""
        L = lut['L']
        s = float(s) % L
        i = int(np.searchsorted(lut['s'], s, side='right') - 1)
        i = int(np.clip(i, 0, lut['n'] - 1))
        t = (s - lut['s'][i]) / lut['ds'][i]
        t = float(np.clip(t, 0.0, 1.0))
        j = (i + 1) % lut['n']
        p = lut['xy'][i] + t * (lut['xy'][j] - lut['xy'][i])
        return p + lut['left'][i] * float(d_lat)

    def _predict_obstacles(self, pts, msg):
        """검출점 + **도달 시각까지 상대가 쓸고 갈 자리**를 함께 돌려준다.

        정적 장애물(속도 < opp_min_speed)이면 원래 점을 그대로 낸다 — q2 의
        동작이 바뀌지 않는다. 움직이는 점은 레이싱라인 s 를 따라 옮긴다.
        """
        if not self.opp_track or len(pts) == 0:
            if len(pts) == 0:
                self._opp_tracks = []
            return pts
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            return pts
        lab, n = self._cluster_pts(pts, self.opp_eps)
        cents = [pts[lab == i].mean(axis=0) for i in range(n)]
        lut = self._s_lut()

        old = getattr(self, '_opp_tracks', [])
        old = [tr for tr in old if t - tr['t'] < self.opp_lost_sec]
        used, new = set(), []
        for ci, c in enumerate(cents):
            best, bd = None, self.opp_gate
            for k, tr in enumerate(old):
                if k in used:
                    continue
                dd = float(np.hypot(*(c - tr['p'])))
                if dd < bd:
                    bd, best = dd, k
            s_now = d_lat = s_dot = 0.0
            if lut is not None:
                s_now, d_lat = self._project_sd(c, lut)
            if best is None:
                tid = int(getattr(self, '_next_tid', 1) or 1)
                self._next_tid = tid + 1
                new.append(dict(p=c, v=np.zeros(2), t=t, n=1, ci=ci, id=tid,
                                s=s_now, s_dot=0.0, d_lat=d_lat, d_dot=0.0))
                continue
            tr = old[best]; used.add(best)
            dt = t - tr['t']
            v = tr['v'] if dt <= 1e-3 else (
                self.opp_alpha * tr['v'] + (1.0 - self.opp_alpha) * (c - tr['p']) / dt)
            d_dot = float(tr.get('d_dot', 0.0))
            if lut is not None and dt > 1e-3:
                ds = self._wrap_ds(s_now - tr['s'], lut['L'])
                s_dot = (self.opp_alpha * tr.get('s_dot', 0.0)
                         + (1.0 - self.opp_alpha) * (ds / dt))
                d_dot = (self.opp_alpha * tr.get('d_dot', 0.0)
                         + (1.0 - self.opp_alpha)
                         * ((d_lat - float(tr.get('d_lat', d_lat))) / dt))
            else:
                s_dot = float(tr.get('s_dot', 0.0))
            new.append(dict(p=c, v=v, t=t, n=tr['n'] + 1, ci=ci,
                            id=int(tr.get('id', 0) or 0),
                            s=s_now, s_dot=s_dot, d_lat=d_lat, d_dot=d_dot))
        self._opp_tracks = new

        out = [pts]
        self._opp_moving = 0
        s_ego = None
        if lut is not None and getattr(self, 'pos', None) is not None:
            s_ego, _ = self._project_sd(np.asarray(self.pos, float)[:2], lut)
        for tr in new:
            sp = (abs(float(tr.get('s_dot', 0.0))) if lut is not None
                  else float(np.hypot(*tr['v'])))
            if tr['n'] < self.opp_min_obs or sp < self.opp_min_speed:
                continue                       # 정적 — 원래 점으로 충분하다
            self._opp_moving += 1
            mem = pts[lab == tr['ci']]
            tau = 0.0
            if lut is None or s_ego is None:
                rel = tr['p'] - self.pos
                d = float(np.hypot(*rel))
                closing = max(abs(self.speed) - float(rel @ tr['v']) / max(d, 1e-3), 0.5)
                tau = float(np.clip(d / closing, 0.0, self.opp_pred_max))
                for f in (0.35, 0.7, 1.0):
                    out.append(mem + tr['v'] * (tau * f))
                continue
            gap = (tr['s'] - s_ego) % lut['L']
            if gap > 0.5 * lut['L']:
                continue                       # 뒤쪽 — 전방 쓸기를 만들지 않는다
            closing = max(abs(self.speed) - float(tr['s_dot']), 0.5)
            tau = float(np.clip(gap / closing, 0.0, self.opp_pred_max))
            for f in (0.35, 0.7, 1.0):
                xy_f = self._pose_at_s(tr['s'] + tr['s_dot'] * tau * f,
                                       tr['d_lat'], lut)
                out.append(mem + (xy_f - tr['p']))
        return np.vstack(out)

    # ── AMCL 용 스캔 마스킹 ───────────────────────────────────────────
    amcl_mask_on   = False   # 켜면 /scan_amcl 을 낸다
    amcl_mask_pad  = 2       # 지우는 빔 좌우로 이만큼 더 (모서리 여유)
    amcl_mask_max  = 0.30    # 이 비율을 넘게 지워야 하면 **아예 안 지운다**

    def _amcl_mask(self, msg):
        """앞차에 맞은 빔을 지운 스캔. 못 지울 상황이면 None.

        ★ 왜 필요한가 (2026-08-15 실측)
          앞차 뒤에 붙으면 라이다가 벽 대신 앞차를 본다. AMCL 은 그 빔을
          "여기 벽이 있다" 로 읽어 자세를 그쪽으로 당긴다.
          race bag 실측 — 맵으로 설명 안 되는 전방 빔의 비율:
              가장 가까운 장애물 0.6m 안   18~43%
              0.6~1.0m                    6~46%
              2m 밖/없음                   0~20%

        ★★ 피드백 루프를 조심해야 한다
          기대거리는 **AMCL 자세로** 계산한다. 그 결과로 스캔을 걸러 다시
          AMCL 에 먹이면, 자세가 틀어졌을 때 진짜 벽을 지우고 더 틀어진다.
          2026-08-10 deskew A/B 에서 똑같은 함정에 빠졌다 (원시 스캔에서 나온
          자세를 기준으로 그 스캔을 판정했다).

          그래서 세 겹으로 막는다:
            ① **두 검사를 다 통과한 빔만** 지운다 (기대보다 짧다 AND 벽에서 멀다).
               회피는 하나만 걸려도 피하지만, 빔을 지우는 건 훨씬 위험하다.
            ② 지울 빔이 전체의 amcl_mask_max(30%) 를 넘으면 **아예 안 지운다.**
               그만큼 설명이 안 된다는 건 장애물이 아니라 **자세가 틀렸다**는
               뜻이다. 그때 지우면 정확히 잘못된 방향으로 굳는다.
            ③ 원본 /scan 은 그대로 두고 /scan_amcl 을 따로 낸다. RViz 에서
               둘을 겹쳐 보면 무엇을 지웠는지 눈으로 확인된다.
        """
        idx = getattr(self, '_obs_beam_strict', None)
        n_all = int(getattr(self, '_obs_beam_n', 0) or 0)
        if idx is None or n_all <= 0:
            return None
        if len(idx) > self.amcl_mask_max * n_all:
            self._amcl_masked = -1                 # 너무 많다 — 손대지 않는다
            return None

        rr = np.asarray(msg.ranges, dtype=float).copy()
        if not len(rr):
            return None
        kill = np.zeros(len(rr), bool)
        pad = int(self.amcl_mask_pad)
        for i in idx:
            kill[max(int(i) - pad, 0):int(i) + pad + 1] = True
        rr[kill] = float('inf')
        self._amcl_masked = int(kill.sum())

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = rr.tolist()
        out.intensities = list(msg.intensities)
        return out

    def _publish_amcl_scan(self, msg):
        """마스킹한 스캔을 발행. 못 지울 상황이면 **원본을 그대로** 낸다.

        ★ 어떤 경우에도 발행을 거르면 AMCL 이 굶는다. 마스킹은 개선이지
          필수가 아니므로, 실패하면 원본으로 폴백하는 것이 맞다.
        """
        pub = getattr(self, 'amcl_scan_pub', None)
        if pub is None or not self.amcl_mask_on:
            return
        try:
            out = self._amcl_mask(msg)
        except Exception:
            out = None
        pub.publish(out if out is not None else msg)

    def _obs_horizon(self) -> float:
        """속도에 비례하는 전방 감시 거리(m).

        회피에 필요한 거리는 속도에 비례한다. 고정값(구 3m)은 1.8m/s 에서
        1.7초 앞까지 보는 셈이라 아직 한참 남은 장애물에 미리 반응했다.
        저속에서도 최소 obs_horizon_min 은 확보한다 (0 이면 아무것도 못 본다).
        """
        return float(np.clip(self.obs_h_min + self.obs_h_sec * abs(self.speed),
                             self.obs_h_min, self.obs_h_max))

    @property
    def _avoid_active(self) -> bool:
        """Q2 정적 장애물과 race 1v1 상대 모두 같은 검출기를 쓴다."""
        return self.mode in ('q2', 'race')

    def _update_lane_free(self):
        """차단 투표 창이 과반이면 회피 유지 (통로를 넓혀 놓치지 않음).

        ★ frenet_planner 는 예전엔 이 값을 영원히 True 로 뒀다. 회피를
          시작하는 순간 장애물이 좁은 통로 밖으로 나가 감지가 풀렸다.
        """
        ml = self.free_votes.maxlen or len(self.free_votes)
        if not ml:
            self.lane_free = True
            return
        self.lane_free = sum(self.free_votes) * 2 >= ml

    def _scan_cb(self, msg: LaserScan):
        # 벽 회피는 pose 없이도 (차량 좌표계만으로) 동작해야 한다 —
        # AMCL 수렴 전에도 벽에 스치는 건 막아야 하므로 pose 검사보다 먼저 저장.
        self.latest_scan = msg

        if self.pos is None:
            return
        # ★ 2026-08-13 이전에는 `mode != 'q2'` 라 **race 에서 검출이 통째로 꺼져
        #   있었다.** frenet_planner 는 q2·race 둘 다 뜨므로(race.launch.py) 노드는
        #   살아서 d=0 만 계속 발행한다 — 로그로는 완전히 정상으로 보이는,
        #   조용히 아무 일도 안 하는 종류의 버그였다.
        if not self._avoid_active:
            return          # q1/q3 는 빈 트랙. q2·race 만 검출한다.

        p = self.get_parameter

        # ── 빔별 기대거리 비교 (기본 경로) ─────────────────────────────
        # 격자+마스크 방식은 "벽 근처인가" 만 보므로 AMCL 오차가 track_margin 을
        # 넘으면 벽이 장애물이 된다 (2026-08-05 q2: 직선 9.4m 를 42cm 밀려 주행).
        # 여기서는 "맵 벽보다 앞에서 끊긴 빔" 만 장애물로 본다.
        if getattr(self, 'ray_occ', None) is not None:
            pts_world = self._raycast_obstacle_pts(msg)
            if len(pts_world) == 0:
                self.on_track_pts = np.zeros((0, 2))
                self.blocking_pts = []
                self.free_votes.append(True)
                self._update_lane_free()
                # ★ 장애물이 없어도 **반드시 발행한다.** 안 내면 AMCL 이
                #   /scan_amcl 을 굶어 위치추정이 통째로 멎는다.
                self._publish_amcl_scan(msg)
                return
            # 마스크를 다시 걸지 않는다 — 이미 "맵으로 설명되지 않는 점" 만 남았다.
            # ★ 상대차는 움직인다 — 지금 자리가 아니라 **내가 도달할 때 있을 자리**를
            #   피해야 한다 (_predict_obstacles).
            pts_world = self._predict_obstacles(pts_world, msg)
            self._classify_obstacles(pts_world, 0.0, masked=False)
            self._publish_amcl_scan(msg)
            return

        xmin = p('grid_xmin').value;   xmax = p('grid_xmax').value
        ymin = p('grid_ymin').value;   ymax = p('grid_ymax').value
        res  = p('grid_res').value;    safe = p('grid_safe_dist').value
        ign  = p('bound_ignore_px').value

        ranges = np.clip(msg.ranges, msg.range_min, msg.range_max)

        # build occupied grid in vehicle frame
        nx = int((xmax - xmin) / res) + 1
        ny = int((ymax - ymin) / res) + 1
        gx = np.linspace(xmin, xmax, nx)
        gy = np.linspace(ymin, ymax, ny)
        gy, gx = np.meshgrid(gy, gx)

        # ★ 격자는 base_link 기준인데 ranges 는 laser 원점에서 잰 값이다.
        #   라이다가 base_link 보다 0.27m 앞에 있으므로(bringup_launch.py 의
        #   static_baselink_to_laser) 그 차이만큼 빼고 비교해야 한다.
        #   빼지 않으면 |rho - range| 에 0.27m 의 계통 오차가 생긴다. safe 가
        #   0.3 이라 정면에선 간신히 걸쳐 셀 1~2 개만 살아남고, 옆으로 갈수록
        #   오차가 커져 아예 안 잡힌다 — 실차에서 "점 1개"로 나타난 원인.
        lx = gx - self.laser_dx
        ly = gy - self.laser_dy
        rho = np.sqrt(lx**2 + ly**2)
        phi = np.arctan2(ly, lx)
        idx = np.clip(
            ((phi - msg.angle_min) / msg.angle_increment).astype(int),
            0, len(ranges) - 1)
        occ = (np.abs(rho - ranges[idx]) < safe).ravel()

        gxf = gx.ravel()[occ]
        gyf = gy.ravel()[occ]

        if len(gxf) == 0:
            self.blocking_pts = []
            self.free_votes.append(True)
            self._update_lane_free()
            return

        # rotate + translate to world frame
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        R  = np.array([[cy, -sy], [sy, cy]])
        pts_world = (R @ np.vstack((gxf, gyf))).T + self.pos

        self._classify_obstacles(pts_world, ign)
    def _classify_obstacles(self, pts_world: np.ndarray, ign: float,
                            masked: bool = True):
        # without bound files → treat all grid hits as candidates (벽도 섞인다)
        if not masked or getattr(self, 'track_mask', None) is None:
            on_track = list(pts_world)
        else:
            # 사전계산된 마스크 조회 — 예전의 점당 cv2.pointPolygonTest 2회
            # 파이썬 루프를 대체한다 (_build_track_mask 주석 참조).
            h, w = self.map_meta[0], self.map_meta[1]
            res, ox, oy = self.map_meta[2], self.map_meta[3], self.map_meta[4]
            pts_img = _world_to_img(pts_world, h, res, ox, oy)
            ix, iy = pts_img[:, 0], pts_img[:, 1]
            inb = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            keep = np.zeros(len(pts_world), bool)
            keep[inb] = self.track_mask[iy[inb], ix[inb]]
            on_track = list(pts_world[keep])

        # Frenet 후보 평가는 "지금 진로" 밖의 점도 봐야 한다 (옆으로 피한 자리에
        # 뭐가 있는지). blocking 만으로는 그 정보가 없어서 따로 보관한다.
        self.on_track_pts = (np.asarray(on_track) if on_track
                             else np.zeros((0, 2)))

        if not on_track:
            self.blocking_pts = []
            self.free_votes.append(True)
            self._update_lane_free()
            return

        #   그리고 "막고 있는 점"만 남겨 회피 방향·마커에 쓴다 — 예전엔 트랙 안의
        #   모든 점 평균으로 방향을 정해서 엉뚱한 쪽으로 틀 수 있었다.
        # ★ 판정 기준은 "내가 실제로 지나갈 자리를 막았는가" 하나다.
        #   레이싱라인은 우리가 임의로 넣은 계획일 뿐이고, 충돌은 실제 진로에서만
        #   일어난다. 라인 기준으로 보면 추종 오차만큼 벌어진 구간에서 실제로
        #   부딪힐 장애물을 통째로 놓친다 (2026-07-21 실차: 두 번째 장애물).
        #   라인은 "어디로 피할지"(_obstacle_shift 의 목표점)에만 쓴다.
        blocking = [p for p, k in
                    zip(on_track, self._in_predicted_corridor(np.asarray(on_track)))
                    if k]

        self.blocking_pts = blocking
        self.free_votes.append(not blocking)
        self._update_lane_free()
        if blocking:
            self._pub_obstacle_markers(blocking)

    # ── publishers / helpers ───────────────────────────────────────────────
    def _pub_obstacle_markers(self, pts: list):
        arr = MarkerArray()
        for i, pt in enumerate(pts[:30]):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'obstacle'; m.id = i
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = float(pt[0])
            m.pose.position.y = float(pt[1])
            m.color.r = 1.0; m.color.a = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.lifetime.nanosec = int(1e8)
            arr.markers.append(m)
        self.obs_pub.publish(arr)
