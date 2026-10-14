"""vehicle_sim.py — Single-track (bicycle) dynamic model with Pacejka tire model.

Pacejka convention consistent with map_controller.py:
  Fy = mu * Fz * D * sin(C * atan(B*α - E*(B*α - atan(B*α))))
  where D is the normalized shape peak and mu is surface friction.

ISO 8855 body frame:
  x: forward  y: left  z: up
  r: yaw rate, positive = CCW (left turn)
  delta: steering, positive = left

Slip angle sign:
  α = -arctan2(vy_wheel_frame, vx_wheel_frame)
  Left turn (r > 0): α_r > 0 → Fy_r > 0 (centripetal) ✓

Static vertical loads (no load transfer — bicycle model assumption):
  Fz_f = m * g * lr / L
  Fz_r = m * g * lf / L
"""

from __future__ import annotations
import csv
from copy import copy
from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import fsolve

G = 9.81
EPS = 0.05   # minimum |vx| to avoid divide-by-zero in arctan2


# ── Vehicle Parameters ────────────────────────────────────────────────────────

@dataclass
class VehicleParams:
    """F1TENTH vehicle parameters (from map_controller.py defaults)."""
    m:  float = 3.354      # kg  (실측 2026-08-09)
    lf: float = 0.1757     # m  CG to front axle  (실측: W_r/m*L, L=0.32m)
    lr: float = 0.1441     # m  CG to rear axle   (실측: W_f/m*L)
    Iz: float = 0.050      # kg·m²  (estimated: m/12*(L²+tw²), tw≈0.27m)

    # Surface friction coefficient
    # ★ 이 mu 는 마찰계수가 **아니다**. 실효 최대 그립 = mu × pac_D 다
    #   (MAP 논문 III-B: "peak value D̃ is in fact μ·Fz,i·Di").
    #   대조: 논문값 1.0489 × 1.2084 = 1.267g — 본문의 "1.25 g" 와 일치한다.
    #
    # ★ 불변식: mu × pac_D = 0.40 (실측 실효 그립). 하나를 바꾸면 둘 다 바꿀 것.
    #   0.348 × 1.15 = 0.40 ✔
    # ※ 아래 pac_* 는 race_script.yaml [map] 과 **같은 값이어야 한다** —
    #   여기서 LUT 를 만들고 저기서 컨트롤러가 쓴다. 갈라지면 시뮬과 실차가
    #   다른 타이어를 쓰게 된다 (2026-08-09 에 실제로 갈라져 있었다:
    #   컨트롤러 B=3.8/D=1.15 vs 생성기 B=3.629/D=1.2084).
    mu: float = 0.348

    # Pacejka magic formula — normalized by Fz
    # Consistent with map_controller: Fy = mu * Fz * D * pac_shape(alpha)
    pac_B: float = 3.8      # ↓ race_script.yaml [map] 과 동기 유지
    pac_C: float = 1.3
    pac_D: float = 1.15     # shape peak (dimensionless)
    pac_E: float = -0.5

    @property
    def L(self) -> float:
        return self.lf + self.lr

    @property
    def Fz_f(self) -> float:
        return self.m * G * self.lr / self.L

    @property
    def Fz_r(self) -> float:
        return self.m * G * self.lf / self.L

    def with_mu(self, mu: float) -> VehicleParams:
        p = copy(self); p.mu = mu; return p


# ── Pacejka Tire Force ────────────────────────────────────────────────────────

def _pac(slip: float, B: float, C: float, D: float, E: float) -> float:
    x = B * slip
    return D * np.sin(C * np.arctan(x - E * (x - np.arctan(x))))


def tire_Fy(alpha: float, Fz: float, p: VehicleParams,
            mu: float | None = None) -> float:
    """Lateral tire force [N]. alpha > 0 → Fy > 0 (leftward)."""
    mu_eff = mu if mu is not None else p.mu
    return mu_eff * Fz * _pac(alpha, p.pac_B, p.pac_C, p.pac_D, p.pac_E)


def tire_Fx(kappa: float, Fz: float, p: VehicleParams,
            mu: float | None = None) -> float:
    """Longitudinal tire force [N]. kappa > 0 = drive."""
    mu_eff = mu if mu is not None else p.mu
    return mu_eff * Fz * _pac(kappa, p.pac_B, p.pac_C, p.pac_D, p.pac_E)


# ── Slip Angle ────────────────────────────────────────────────────────────────

def slip_alpha(vx: float, vy_axle: float, steer: float) -> float:
    """Slip angle [rad] at an axle.

    vx:      longitudinal velocity at axle (body frame) [m/s]
    vy_axle: lateral velocity at axle centre (body frame) [m/s]
             front: vy + lf*r,  rear: vy - lr*r
    steer:   steering angle [rad] (0 for rear)
    """
    cd, sd = np.cos(steer), np.sin(steer)
    vx_w = vx * cd + vy_axle * sd
    vy_w = -vx * sd + vy_axle * cd
    return -np.arctan2(vy_w, max(abs(vx_w), EPS))


# ── Single-Track ODE ──────────────────────────────────────────────────────────

def bicycle_ode(t, state, delta, Fx_rear, p: VehicleParams,
                mu: float | None = None) -> list:
    """Single-track (bicycle) dynamic model.

    state: [vx, vy, r, x, y, psi]
    delta:   steering [rad], positive = left
    Fx_rear: rear longitudinal force [N], positive = drive
    """
    vx, vy, r, x, y, psi = state
    vx = max(vx, EPS)

    # Axle velocities
    vy_f = vy + p.lf * r
    vy_r = vy - p.lr * r

    # Slip angles
    alpha_f = slip_alpha(vx, vy_f, delta)
    alpha_r = slip_alpha(vx, vy_r, 0.0)

    # Tire forces
    Fy_f = tire_Fy(alpha_f, p.Fz_f, p, mu)
    Fy_r = tire_Fy(alpha_r, p.Fz_r, p, mu)

    # Front forces in body frame (front wheel is steered)
    cd, sd = np.cos(delta), np.sin(delta)
    Fx_f_b = -Fy_f * sd          # backward component from steer
    Fy_f_b =  Fy_f * cd          # lateral component

    # Equations of motion (body frame, Coriolis terms)
    dvx = (Fx_f_b + Fx_rear) / p.m + vy * r
    dvy = (Fy_f_b + Fy_r)   / p.m - vx * r
    dr  = (p.lf * Fy_f_b - p.lr * Fy_r) / p.Iz

    # Global kinematics
    dx   = vx * np.cos(psi) - vy * np.sin(psi)
    dy   = vx * np.sin(psi) + vy * np.cos(psi)
    dpsi = r

    return [dvx, dvy, dr, dx, dy, dpsi]


# ── Simulation ────────────────────────────────────────────────────────────────

def simulate(t_end: float, state0: list,
             delta_fn, Fx_fn,
             p: VehicleParams, mu: float | None = None,
             dt: float = 0.005):
    """Integrate bicycle model.

    delta_fn(t) -> steering [rad]
    Fx_fn(t)    -> rear drive force [N]
    Returns (t_arr, states) where states.shape = (N, 6).
    """
    sol = solve_ivp(
        lambda t, y: bicycle_ode(t, y, delta_fn(t), Fx_fn(t), p, mu),
        [0.0, t_end], state0,
        t_eval=np.arange(0.0, t_end, dt),
        method='RK45', rtol=1e-6, atol=1e-8,
    )
    return sol.t, sol.y.T


# ── Steady-State Solver ───────────────────────────────────────────────────────

def steady_state(v: float, delta: float, p: VehicleParams,
                 mu: float | None = None) -> dict | None:
    """Find steady-state (vy, r) for constant-speed circular driving.

    Solves dvy/dt = 0, dr/dt = 0 at given speed v and steering delta.
    Returns result dict or None if no convergence.
    """
    def residual(x):
        vy, r = x
        d = bicycle_ode(0.0, [v, vy, r, 0, 0, 0], delta, 0.0, p, mu)
        return [d[1], d[2]]   # dvy, dr → zero

    # Kinematic bicycle initial guess
    r0  = v * np.tan(delta) / p.L if abs(delta) > 1e-4 else 0.0
    vy0 = -r0 * p.lr * 0.5

    try:
        sol, _, ier, _ = fsolve(residual, [vy0, r0], full_output=True)
        vy, r = sol
        if ier != 1 or np.hypot(*residual([vy, r])) > 0.5:
            return None
    except Exception:
        return None

    beta = float(np.degrees(np.arctan2(vy, v)))
    ay   = float(v * r)
    R    = float(v / r) if abs(r) > 1e-4 else np.inf

    alpha_f = slip_alpha(v, vy + p.lf * r, delta)
    alpha_r = slip_alpha(v, vy - p.lr * r, 0.0)
    Fy_f = tire_Fy(alpha_f, p.Fz_f, p, mu)
    Fy_r = tire_Fy(alpha_r, p.Fz_r, p, mu)

    return {
        'v':           v,
        'delta_rad':   delta,
        'delta_deg':   float(np.degrees(delta)),
        'vy':          float(vy),
        'r':           float(r),
        'beta_deg':    beta,
        'ay':          ay,
        'R':           R,
        'alpha_f_deg': float(np.degrees(alpha_f)),
        'alpha_r_deg': float(np.degrees(alpha_r)),
        'Fy_f':        float(Fy_f),
        'Fy_r':        float(Fy_r),
    }


# ── LUT Generation ────────────────────────────────────────────────────────────

def generate_lut(v_range, delta_range,
                 p: VehicleParams, mu: float | None = None,
                 csv_path: str | None = None) -> list[dict]:
    """Build steady-state LUT over (v, delta) grid.

    CSV columns match steady_state() output keys.
    Failed grid points are skipped silently.
    """
    rows = []
    for v in v_range:
        for delta in delta_range:
            ss = steady_state(v, delta, p, mu)
            if ss is not None:
                rows.append(ss)

    if csv_path and rows:
        with open(csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=rows[0].keys())
            w.writeheader()
            w.writerows(rows)

    return rows


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    import argparse
    ap = argparse.ArgumentParser(
        description='Generate MAP steady-state LUT (single-track bicycle model)')
    ap.add_argument('--mu',      type=float, default=None,
                    help='friction override (default: VehicleParams.mu = 0.40)')
    ap.add_argument('--v-min',   type=float, default=0.5)
    ap.add_argument('--v-max',   type=float, default=5.0)
    ap.add_argument('--v-steps', type=int,   default=20)
    ap.add_argument('--d-max',   type=float, default=0.35,
                    help='max |delta| [rad]')
    ap.add_argument('--d-steps', type=int,   default=31,
                    help='number of delta points (odd = includes 0)')
    ap.add_argument('--out',     type=str,   default='lut_bicycle.csv')
    args = ap.parse_args()

    p = VehicleParams()
    v_arr = np.linspace(args.v_min, args.v_max, args.v_steps)
    d_arr = np.linspace(-args.d_max, args.d_max, args.d_steps)

    print(f'Bicycle LUT: {len(v_arr)} speeds × {len(d_arr)} deltas '
          f'(mu={args.mu or p.mu:.4f}) ...')
    rows = generate_lut(v_arr, d_arr, p, mu=args.mu, csv_path=args.out)
    print(f'{len(rows)} / {len(v_arr)*len(d_arr)} converged → {args.out}')


if __name__ == '__main__':
    main()
