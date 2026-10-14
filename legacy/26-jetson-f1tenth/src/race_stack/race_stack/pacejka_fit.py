"""pacejka_fit.py — 실차 replay CSV → Pacejka 타이어 파라미터 식별

사용법:
  python3 pacejka_fit.py [CSV [CSV ...]] [--plot] [--out YAML]

  기본 입력: ~/Downloads/bag_20260807_*_replay.csv
  출력: 전/후 타이어 Pacejka B, C, D, E (mu*D_eff 포함)

방법:
  준정상상태(quasi-steady-state) 구간만 필터링한 뒤,
  자전거 모델 힘 방정식으로 Fy 역산 → Pacejka 곡선 피팅.

  Fy_r = m·vx·ω·lf/L        (횡 + 요 모멘트 연립, Iz·ω_dot ≈ 0)
  Fy_f = m·vx·ω·lr/(L·cosδ)

  슬립각:
    α_r = -atan2(vy - lr·ω, vx)
    α_f = δ - atan2(vy + lf·ω, vx)

  피팅 목표:
    Fy / Fz = D · sin(C · atan(B·α − E·(B·α − atan(B·α))))
"""

from __future__ import annotations
import argparse
import csv
import math
from pathlib import Path
from typing import List

import numpy as np
from scipy.optimize import curve_fit

# ── 차량 파라미터 (실측 2026-08-09) ─────────────────────────────────────────
M  = 3.354    # kg
LF = 0.1757   # m  CoG → 앞 차축
LR = 0.1441   # m  CoG → 뒤 차축
L  = LF + LR  # 휠베이스 0.3198 m
G  = 9.81

FZ_R = M * G * LF / L   # 정적 뒤 수직 하중 [N]
FZ_F = M * G * LR / L   # 정적 앞 수직 하중 [N]

EPS = 0.05  # vx 최소값

# ── 필터 임계값 ──────────────────────────────────────────────────────────────
MIN_VX       = 0.5    # m/s   — 정지 구간 제외
MAX_OD       = 2.0    # rad/s² — 준정상상태: ω_dot 제한
MAX_ALPHA_R  = 0.25   # rad (~14°) — 슬립각 포화 영역 미만
MAX_ALPHA_F  = 0.30   # rad (~17°)
MAX_FYN      = 1.8    # Fy/Fz 상한 (이상치 제거)


# ── Pacejka magic formula ────────────────────────────────────────────────────

def pacejka(alpha: np.ndarray, B: float, C: float, D: float, E: float) -> np.ndarray:
    phi = B * alpha - E * (B * alpha - np.arctan(B * alpha))
    return D * np.sin(C * np.arctan(phi))


def fit_tire(alpha: np.ndarray, fyn: np.ndarray, label: str):
    """alpha [rad], fyn = Fy/Fz 배열 → (B, C, D, E) 피팅 결과 반환."""
    # 초기값: MAP 논문 기본값
    p0     = [3.629, 1.3, 1.2084, -0.5]
    bounds = ([0.1, 0.5, 0.1, -5.0], [30.0, 3.0, 3.0, 1.0])

    try:
        popt, pcov = curve_fit(
            pacejka, alpha, fyn,
            p0=p0, bounds=bounds,
            maxfev=20000, method='trf',
        )
        perr = np.sqrt(np.diag(pcov))
        B, C, D, E = popt
        rmse = np.sqrt(np.mean((pacejka(alpha, *popt) - fyn) ** 2))
        print(f"\n[{label}]  n={len(alpha)}")
        print(f"  B={B:.4f}±{perr[0]:.4f}  C={C:.4f}±{perr[1]:.4f}"
              f"  D={D:.4f}±{perr[2]:.4f}  E={E:.4f}±{perr[3]:.4f}")
        print(f"  RMSE(Fy/Fz)={rmse:.4f}   peak α≈{1/B:.3f} rad ({math.degrees(1/B):.1f}°)")
        return popt, perr, rmse
    except RuntimeError as e:
        print(f"[{label}] 피팅 실패: {e}")
        return None, None, None


# ── 데이터 로드 ──────────────────────────────────────────────────────────────

def load_csvs(paths: List[Path]) -> dict:
    rows = []
    for p in paths:
        with open(p) as fh:
            for r in csv.DictReader(fh):
                rows.append({k: float(v) for k, v in r.items()})
    if not rows:
        raise ValueError("CSV 데이터 없음")
    cols = list(rows[0].keys())
    return {c: np.array([r[c] for r in rows]) for c in cols}


# ── 메인 ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csvs', nargs='*',
                    default=sorted(Path.home().glob('Downloads/bag_*_replay.csv')))
    ap.add_argument('--plot', action='store_true')
    ap.add_argument('--out',  type=str, default=None,
                    help='결과를 YAML 파일로 저장')
    args = ap.parse_args()

    paths = [Path(p) for p in args.csvs]
    for p in paths:
        print(f"로드: {p}  ({p.stat().st_size//1024} KB)")

    d = load_csvs(paths)
    n_total = len(d['t'])
    print(f"\n전체 {n_total}행 로드 완료")

    vx    = d['v_wheel']
    vy    = d['vy_base']
    omega = d['wz_imu']
    delta = d['delta']

    # ω_dot 수치 미분
    dt = np.diff(d['t'], prepend=d['t'][0])
    dt = np.clip(dt, 1e-3, 0.5)
    omega_dot = np.gradient(omega, d['t'])

    # 슬립각 — 운동학적 근사 (vy=0 가정)
    # vy_base (LiDAR ICP) 노이즈 ~0.37 m/s 가 corr 0.17 로 망가뜨림 (2026-08-09 확인)
    # 운동학 근사 시 corr(alpha_r, Fyn_r)=0.90 으로 회복
    alpha_r =  np.arctan2(LR * omega, np.maximum(vx, EPS))
    alpha_f =  delta - np.arctan2(-LF * omega, np.maximum(vx, EPS))

    # 준정상상태 횡력 역산
    Fy_r = M * vx * omega * LF / L
    cos_d = np.cos(delta)
    cos_d = np.where(np.abs(cos_d) > 0.1, cos_d, 0.1)
    Fy_f = M * vx * omega * LR / (L * cos_d)

    Fyn_r = Fy_r / FZ_R
    Fyn_f = Fy_f / FZ_F

    # 필터
    mask_r = (
        (vx > MIN_VX) &
        (np.abs(omega_dot) < MAX_OD) &
        (np.abs(alpha_r) > 0.005) &          # 직진 구간 제외 (슬립각 거의 0)
        (np.abs(alpha_r) < MAX_ALPHA_R) &
        (np.abs(Fyn_r) < MAX_FYN)
    )
    mask_f = (
        (vx > MIN_VX) &
        (np.abs(omega_dot) < MAX_OD) &
        (np.abs(alpha_f) > 0.005) &
        (np.abs(alpha_f) < MAX_ALPHA_F) &
        (np.abs(Fyn_f) < MAX_FYN)
    )

    print(f"\n필터 후  뒤: {mask_r.sum()}행   앞: {mask_f.sum()}행")

    if mask_r.sum() < 50:
        print("경고: 뒤 타이어 피팅 데이터 부족 (준정상상태 구간이 짧거나 없음)")
    if mask_f.sum() < 50:
        print("경고: 앞 타이어 피팅 데이터 부족")

    popt_r, perr_r, rmse_r = fit_tire(alpha_r[mask_r], Fyn_r[mask_r], "REAR")
    popt_f, perr_f, rmse_f = fit_tire(alpha_f[mask_f], Fyn_f[mask_f], "FRONT")

    # 결과 요약
    print("\n" + "="*60)
    print("vehicle_sim.py 에 넣을 값 (뒤 타이어 기준, 앞뒤 평균 가능):")
    if popt_r is not None:
        B, C, D, E = popt_r
        print(f"  pac_B = {B:.4f}")
        print(f"  pac_C = {C:.4f}")
        print(f"  pac_D = {D:.4f}   # ≈ mu × D_shape,  D/D_ref 가 μ 지표")
        print(f"  pac_E = {E:.4f}")
        mu_est = D * FZ_R / (M * G)  # 대략적인 μ 추정
        print(f"  mu 추정 (D·Fz_r/mg) ≈ {mu_est:.4f}")

    # YAML 저장
    if args.out and popt_r is not None:
        out = Path(args.out)
        B, C, D, E = popt_r
        with open(out, 'w') as fh:
            fh.write(f"# Pacejka parameters identified from real vehicle logs (2026-08-09)\n")
            fh.write(f"# RMSE(Fy/Fz): rear={rmse_r:.4f}")
            if rmse_f:
                fh.write(f"  front={rmse_f:.4f}")
            fh.write("\n")
            fh.write(f"pac_B: {B:.6f}\n")
            fh.write(f"pac_C: {C:.6f}\n")
            fh.write(f"pac_D: {D:.6f}\n")
            fh.write(f"pac_E: {E:.6f}\n")
            if popt_f is not None:
                Bf, Cf, Df, Ef = popt_f
                fh.write(f"pac_B_front: {Bf:.6f}\n")
                fh.write(f"pac_C_front: {Cf:.6f}\n")
                fh.write(f"pac_D_front: {Df:.6f}\n")
                fh.write(f"pac_E_front: {Ef:.6f}\n")
        print(f"\n결과 저장: {out}")

    # 플롯
    if args.plot:
        try:
            import matplotlib.pyplot as plt

            fig, axes = plt.subplots(1, 2, figsize=(12, 5))
            fig.suptitle('Pacejka Tire Identification — Real Vehicle Logs (2026-08-07)')

            for ax, alpha, fyn, mask, popt, label in [
                (axes[0], alpha_r, Fyn_r, mask_r, popt_r, 'Rear'),
                (axes[1], alpha_f, Fyn_f, mask_f, popt_f, 'Front'),
            ]:
                ax.scatter(np.degrees(alpha[mask]), fyn[mask],
                           s=4, alpha=0.4, color='steelblue', label='data')
                if popt is not None:
                    a_plot = np.linspace(-MAX_ALPHA_R, MAX_ALPHA_R, 300)
                    ax.plot(np.degrees(a_plot), pacejka(a_plot, *popt),
                            'r-', lw=2, label=f'fit B={popt[0]:.2f} C={popt[1]:.2f} D={popt[2]:.2f}')
                ax.set_xlabel('Slip angle α [°]')
                ax.set_ylabel('Fy / Fz')
                ax.set_title(f'{label} tire')
                ax.legend(fontsize=8)
                ax.grid(True, alpha=0.3)
                ax.axhline(0, color='k', lw=0.5)
                ax.axvline(0, color='k', lw=0.5)

            plt.tight_layout()
            plt.savefig('/tmp/pacejka_fit.png', dpi=150)
            print("플롯 저장: /tmp/pacejka_fit.png")
            plt.show()
        except ImportError:
            print("matplotlib 없음 — --plot 불가")


if __name__ == '__main__':
    main()
