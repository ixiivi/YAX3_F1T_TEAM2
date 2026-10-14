"""sysid_nn.py — 온-트랙 Pacejka SysID (arXiv 2411.17508, Dikici et al.)

파이프라인
----------
오프라인 (노트북에서 한 번):
  python3 sysid_nn.py train [--iters N] [--log CSV]
    --log 없으면: vehicle_sim으로 varied-μ 시뮬 학습 데이터 자동 생성
    --log 있으면: slip_friction CSV 로그로 학습

온라인 (결과 확인):
  python3 sysid_nn.py run --log logs/slip_tests/xxx_friction.csv

MLP 구조: 4→8→2, LeakyReLU, 58 params
  입력 [xk, uk] = [vx, vy, ω, δ]
  출력  ek      = [e_vy, e_ω]   (명목 모델 1-스텝 잔차, 논문 eq.6)

논문과 차이점:
  - 학습 데이터: 실차 대신 varied-μ 시뮬 (slip_friction 로그로 대체 가능)
  - Pacejka 식별: 전체 Φp 대신 D_f, D_r만 선형 LS (μ 추정이 목적이므로)
  - 반복 학습: --iters N 으로 논문 Figure 4 방식 지원
"""

from __future__ import annotations
import argparse
import math
import csv
from pathlib import Path
from copy import copy

import numpy as np
import torch
import torch.nn as nn

# vehicle_sim 임포트 (없으면 인라인 파라미터 사용)
try:
    from race_stack.vehicle_sim import VehicleParams
except ImportError:
    try:
        from vehicle_sim import VehicleParams
    except ImportError:
        from dataclasses import dataclass
        @dataclass
        class VehicleParams:
            m: float = 3.354;   lf: float = 0.1757;  lr: float = 0.1441
            # mu 는 마찰계수가 아니다 — 실효 그립 = mu × pac_D (MAP 논문 III-B).
            # 0.348 × 1.15 = 0.40g (2026-08-09 실측 추정)
            Iz: float = 0.050;  mu: float = 0.348
            pac_B: float = 3.8; pac_C: float = 1.3
            pac_D: float = 1.15; pac_E: float = -0.5
            @property
            def L(self): return self.lf + self.lr
            @property
            def Fz_f(self): return self.m * 9.81 * self.lr / self.L
            @property
            def Fz_r(self): return self.m * 9.81 * self.lf / self.L

TS   = 0.02   # 샘플링 주기 [s]  (논문: 50 Hz)
G    = 9.81
EPS  = 0.05   # vx 최소값 (arctan2 안정성)

WEIGHTS = Path(__file__).parent / 'sysid_weights.pt'


# ── 명목 모델 (Euler 적분, 논문 eq.4) ─────────────────────────────────────────

def _pac(alpha: float, B: float, C: float, D: float, E: float, Fz: float) -> float:
    x = B * alpha
    return D * Fz * np.sin(C * np.arctan(x - E * (x - np.arctan(x))))


def nominal_step(vy: float, omega: float, vx: float, delta: float,
                 p: VehicleParams,
                 Df: float | None = None,
                 Dr: float | None = None) -> tuple[float, float]:
    """1-스텝 Euler 적분. Df·Dr 으로 D만 오버라이드 가능.

    Returns: (vy_next, omega_next)
    """
    Df = Df if Df is not None else p.mu * p.pac_D
    Dr = Dr if Dr is not None else p.mu * p.pac_D
    B, C, E = p.pac_B, p.pac_C, p.pac_E

    # 슬립각 (논문 eq.3)
    alpha_f = delta - math.atan2(vy + p.lf * omega, max(abs(vx), EPS))
    alpha_r = -math.atan2(vy - p.lr * omega, max(abs(vx), EPS))

    Fyf = _pac(alpha_f, B, C, Df, E, p.Fz_f)
    Fyr = _pac(alpha_r, B, C, Dr, E, p.Fz_r)

    vy_n  = vy    + (Fyr + Fyf * math.cos(delta) - p.m * vx * omega) / p.m  * TS
    om_n  = omega + (Fyf * p.lf * math.cos(delta) - Fyr * p.lr)       / p.Iz * TS

    return vy_n, om_n


# ── MLP (논문 §IV.A) ──────────────────────────────────────────────────────────

class ResidualMLP(nn.Module):
    """4→8→2, LeakyReLU.  파라미터 수 = 4·8+8 + 8·2+2 = 58."""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(4, 8),
            nn.LeakyReLU(),
            nn.Linear(8, 2),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)

    def predict(self, inp: np.ndarray) -> np.ndarray:
        """numpy (4,) → numpy (2,) 잔차 예측."""
        with torch.no_grad():
            return self.net(torch.tensor(inp, dtype=torch.float32)).numpy()

    def save(self, path: Path = WEIGHTS):
        torch.save(self.state_dict(), path)
        print(f'[sysid] weights → {path}')

    @classmethod
    def load(cls, path: Path = WEIGHTS) -> ResidualMLP:
        m = cls(); m.load_state_dict(torch.load(path, weights_only=True)); m.eval()
        return m


# ── 학습 데이터 생성 ──────────────────────────────────────────────────────────

def gen_sim_data(p: VehicleParams,
                 n_traj: int = 300,
                 t_each: float = 8.0,
                 Df_nom: float | None = None,
                 Dr_nom: float | None = None,
                 mu_fixed: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    """varied-μ 시뮬로 (X, Y) 생성.

    mu_fixed가 None이면 μ를 0.5~1.5로 변화, 지정하면 단일 μ만 사용.
    X: (N, 4) = [vx, vy, ω, δ]
    Y: (N, 2) = [e_vy, e_ω]
    """
    mu_arr = [mu_fixed] if mu_fixed is not None else np.linspace(0.5, 1.5, 11)
    n_per  = max(1, n_traj // len(mu_arr))
    steps  = int(t_each / TS)
    rng    = np.random.default_rng(42)

    Df_nom = Df_nom if Df_nom is not None else p.mu * p.pac_D
    Dr_nom = Dr_nom if Dr_nom is not None else p.mu * p.pac_D

    Xs, Ys = [], []
    for mu_true in mu_arr:
        Df_true = mu_true * p.pac_D
        Dr_true = mu_true * p.pac_D
        for _ in range(n_per):
            vx = rng.uniform(0.5, 4.5)
            # 정현파 + 랜덤 혼합 조향 입력
            t_arr = np.arange(steps) * TS
            d_seq = np.clip(
                0.20 * np.sin(2 * np.pi * rng.uniform(0.2, 1.5) * t_arr)
                + rng.normal(0, 0.05, steps),
                -0.38, 0.38
            )
            vy, om = 0.0, 0.0
            for k in range(steps - 1):
                delta = float(d_seq[k])
                inp = np.array([vx, vy, om, delta], dtype=np.float32)

                # 명목 예측 (공칭 μ)
                vy_hat, om_hat = nominal_step(vy, om, vx, delta, p, Df_nom, Dr_nom)
                # 실제 다음 상태 (μ_true)
                vy_next, om_next = nominal_step(vy, om, vx, delta, p, Df_true, Dr_true)

                Xs.append(inp)
                Ys.append(np.array([vy_next - vy_hat, om_next - om_hat], dtype=np.float32))
                vy, om = vy_next, om_next

    return np.vstack(Xs), np.vstack(Ys)


def load_log_data(csv_path: str | Path,
                  p: VehicleParams,
                  window_sec: float = 30.0) -> tuple[np.ndarray, np.ndarray]:
    """slip_friction CSV 로그에서 (X, Y) 추출.

    columns 필요: t, vx_lidar, vy_lidar, wz_lidar, delta
    마지막 window_sec 초만 사용.
    """
    rows = []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            try:
                rows.append({
                    't':     float(row['t']),
                    'vx':    float(row['vx_lidar']),
                    'vy':    float(row['vy_lidar']),
                    'om':    float(row['wz_lidar']),
                    'delta': float(row['delta']),
                })
            except (KeyError, ValueError):
                continue

    if not rows:
        raise ValueError(f'유효 데이터 없음: {csv_path}')

    t_end   = rows[-1]['t']
    rows    = [r for r in rows if r['t'] >= t_end - window_sec]

    # 일정 간격으로 리샘플링 (TS 기준)
    t0      = rows[0]['t']
    t_grid  = np.arange(0, rows[-1]['t'] - t0, TS)
    t_raw   = np.array([r['t'] - t0 for r in rows])
    vx_raw  = np.array([r['vx']    for r in rows])
    vy_raw  = np.array([r['vy']    for r in rows])
    om_raw  = np.array([r['om']    for r in rows])
    d_raw   = np.array([r['delta'] for r in rows])

    vx_s  = np.interp(t_grid, t_raw, vx_raw)
    vy_s  = np.interp(t_grid, t_raw, vy_raw)
    om_s  = np.interp(t_grid, t_raw, om_raw)
    d_s   = np.interp(t_grid, t_raw, d_raw)

    Xs, Ys = [], []
    for k in range(len(t_grid) - 1):
        inp = np.array([vx_s[k], vy_s[k], om_s[k], d_s[k]], dtype=np.float32)
        vy_hat, om_hat = nominal_step(vy_s[k], om_s[k], vx_s[k], d_s[k], p)
        Xs.append(inp)
        Ys.append(np.array([vy_s[k+1] - vy_hat, om_s[k+1] - om_hat], dtype=np.float32))

    return np.vstack(Xs), np.vstack(Ys)


# ── 노이즈 보강 (논문 §III.C.2, eq.10) ──────────────────────────────────────

def augment_noise(X: np.ndarray, Y: np.ndarray,
                  eta_max: float = 1.4, eta_step: float = 0.2):
    """η = 0.2, 0.4, ..., eta_max 로 노이즈 복제 보강."""
    d_avg = np.abs(X).mean(axis=0) + 1e-8
    aug_X, aug_Y = [X], [Y]
    for eta in np.arange(eta_step, eta_max + 1e-9, eta_step):
        noise = np.random.normal(0, d_avg * eta, X.shape).astype(np.float32)
        aug_X.append(X + noise)
        aug_Y.append(Y)
    return np.vstack(aug_X), np.vstack(aug_Y)


# ── 학습 (논문 §III.D) ───────────────────────────────────────────────────────

def train_mlp(X: np.ndarray, Y: np.ndarray,
              epochs: int = 500,
              lr: float = 5e-4) -> ResidualMLP:
    """Adam + MSE, 전체 배치 (논문과 동일)."""
    model = ResidualMLP()
    opt   = torch.optim.Adam(model.parameters(), lr=lr)
    Xt    = torch.tensor(X, dtype=torch.float32)
    Yt    = torch.tensor(Y, dtype=torch.float32)

    for ep in range(1, epochs + 1):
        model.train()
        pred = model(Xt)
        loss = nn.functional.mse_loss(pred, Yt)
        opt.zero_grad(); loss.backward(); opt.step()
        if ep % 100 == 0:
            print(f'  epoch {ep:4d}/{epochs}  MSE={loss.item():.2e}')

    model.eval()
    return model


# ── 가상 정상상태 데이터 생성 (논문 §III.C.2) ─────────────────────────────────

def generate_virtual_ss(mlp: ResidualMLP, p: VehicleParams,
                        vx_ref: float,
                        delta_max: float = 0.4,
                        t_sim: float = 10.0,
                        Df: float | None = None,
                        Dr: float | None = None) -> tuple[np.ndarray, np.ndarray,
                                                          np.ndarray, np.ndarray]:
    """보정 모델(Nominal + NN)을 정속 시뮬하여 가상 SS 데이터 생성.

    δ를 0 → delta_max 까지 t_sim 초에 걸쳐 선형 증가.
    횡력은 eq.8 force balance로, 슬립각은 시뮬 상태(vy_next, om_next)를 사용.

    Returns: alpha_f, Fy_f_norm, alpha_r, Fy_r_norm  (각 (N,) array)
    """
    steps   = int(t_sim / TS)
    delta_s = np.linspace(0.0, delta_max, steps)
    L       = p.L

    vy, om = 0.0, 0.0
    a_f_list, Ff_list = [], []
    a_r_list, Fr_list = [], []

    for k in range(steps - 1):
        delta = float(delta_s[k])
        inp   = np.array([vx_ref, vy, om, delta], dtype=np.float32)

        # 명목 예측
        vy_hat, om_hat = nominal_step(vy, om, vx_ref, delta, p, Df, Dr)
        # NN 잔차 보정 (논문 eq.7)
        e  = mlp.predict(inp)
        vy_next = vy_hat + e[0]
        om_next = om_hat + e[1]

        # 횡력: force balance에서 직접 (논문 eq.8, vy와 무관)
        om_k = float(om_next)
        if abs(om_k) < 1e-4:
            vy, om = vy_next, om_next
            continue

        cos_d = math.cos(delta)
        Fyr = p.m * p.lf * vx_ref * om_k / L
        Fyf = p.m * p.lr * vx_ref * om_k / (L * cos_d) if abs(cos_d) > 1e-3 else 0.0

        # 슬립각: 시뮬 상태 사용 (vy=0 가정 제거)
        vy_k = float(vy_next)
        alpha_r = -math.atan2(vy_k - p.lr * om_k, max(abs(vx_ref), EPS))
        alpha_f = delta - math.atan2(vy_k + p.lf * om_k, max(abs(vx_ref), EPS))

        # Fz로 정규화
        a_f_list.append(alpha_f);  Ff_list.append(Fyf / max(p.Fz_f, 1e-3))
        a_r_list.append(alpha_r);  Fr_list.append(Fyr / max(p.Fz_r, 1e-3))

        vy, om = vy_next, om_next

    return (np.array(a_f_list), np.array(Ff_list),
            np.array(a_r_list), np.array(Fr_list))


# ── Pacejka D 선형 LS ─────────────────────────────────────────────────────────

def fit_D(alphas: np.ndarray, Fy_norm: np.ndarray,
          p: VehicleParams) -> float:
    """B, C, E 고정, D만 선형 LS로 식별.

    Fy_norm ≈ D * shape(α; B,C,E)
    D = Σ(Fy_i * s_i) / Σ(s_i²)
    """
    B, C, E = p.pac_B, p.pac_C, p.pac_E

    def shape(a):
        x = B * a
        return np.sin(C * np.arctan(x - E * (x - np.arctan(x))))

    s = shape(alphas)
    denom = np.dot(s, s)
    if denom < 1e-8:
        return p.mu * p.pac_D   # fallback: 공칭값
    return float(np.dot(Fy_norm, s) / denom)


# ── 전체 온라인 파이프라인 ────────────────────────────────────────────────────

def run_sysid(log_path: str | Path,
              mlp: ResidualMLP,
              p: VehicleParams,
              n_iters: int = 3,
              window_sec: float = 30.0) -> dict:
    """로그 CSV → (D_f, D_r, μ̂).

    논문 Figure 4의 반복 학습 방식:
      1. MLP 잔차 보정으로 가상 SS 생성
      2. LS로 D_f, D_r 식별
      3. 식별된 D를 새 명목값으로 MLP 재학습 (--iters > 1)
    """
    X, Y = load_log_data(log_path, p, window_sec)
    vx_ref = float(X[:, 0].mean())
    print(f'[sysid] 로그: {len(X)} 스텝  vx_ref={vx_ref:.2f} m/s')

    Df = Dr = None   # 첫 반복: 공칭값 사용
    mlp_cur = mlp

    for it in range(n_iters):
        # 가상 SS 생성
        af, Ff, ar, Fr = generate_virtual_ss(mlp_cur, p, vx_ref, Df=Df, Dr=Dr)
        if len(af) == 0:
            print('[sysid] 경고: 가상 SS 데이터 없음'); break

        # D 식별
        Df_new = fit_D(af, Ff, p)
        Dr_new = fit_D(ar, Fr, p)
        print(f'[sysid] iter {it+1}: D_f={Df_new:.4f}  D_r={Dr_new:.4f}')

        if n_iters > 1 and it < n_iters - 1:
            # 새 D로 MLP 재학습 (반복 학습 §III.F)
            X_aug, Y_aug = augment_noise(X, Y)
            # Y 재계산: 새 D 기준 잔차
            Y_new = np.zeros_like(Y)
            for k in range(len(X)):
                vx_k, vy_k, om_k, d_k = X[k]
                vy_h, om_h = nominal_step(vy_k, om_k, vx_k, d_k, p, Df_new, Dr_new)
                Y_new[k] = Y[k]   # 실제-새명목 잔차 (여기선 근사, 실차엔 실측값 사용)
            mlp_cur = train_mlp(X_aug, Y_aug, epochs=200, lr=5e-4)

        Df, Dr = Df_new, Dr_new

    # μ 추정
    D_nom  = p.mu * p.pac_D
    mu_f   = (Df / D_nom) * p.mu if D_nom > 1e-8 else p.mu
    mu_r   = (Dr / D_nom) * p.mu if D_nom > 1e-8 else p.mu
    mu_hat = float(np.mean([mu_f, mu_r]))

    print(f'[sysid] μ̂ = {mu_hat:.4f}  (front={mu_f:.4f}, rear={mu_r:.4f})')
    return {'Df': Df, 'Dr': Dr, 'mu_f': mu_f, 'mu_r': mu_r, 'mu_hat': mu_hat}


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description='On-Track Pacejka SysID (2411.17508)')
    sub = ap.add_subparsers(dest='cmd', required=True)

    # train
    tr = sub.add_parser('train', help='MLP 학습')
    tr.add_argument('--log',    type=str, default=None,
                    help='학습용 slip_friction CSV (없으면 시뮬 자동 생성)')
    tr.add_argument('--iters',  type=int, default=1,  help='반복 학습 횟수')
    tr.add_argument('--epochs', type=int, default=500)
    tr.add_argument('--out',    type=str, default=str(WEIGHTS))

    # run
    ru = sub.add_parser('run', help='로그에서 μ 추정')
    ru.add_argument('--log',    type=str, required=True)
    ru.add_argument('--iters',  type=int, default=3)
    ru.add_argument('--window', type=float, default=30.0, help='데이터 윈도우 [s]')
    ru.add_argument('--weights',type=str, default=str(WEIGHTS))

    args = ap.parse_args()
    p    = VehicleParams()

    if args.cmd == 'train':
        print('[sysid] 학습 데이터 생성 중...')
        if args.log:
            X, Y = load_log_data(args.log, p)
        else:
            X, Y = gen_sim_data(p)
        print(f'[sysid] 원시 데이터: {len(X)} 스텝')

        X_aug, Y_aug = augment_noise(X, Y)
        print(f'[sysid] 노이즈 보강 후: {len(X_aug)} 스텝  → 학습 시작')

        Df = Dr = None
        model = None
        for it in range(args.iters):
            print(f'[sysid] === 반복 {it+1}/{args.iters} ===')
            model = train_mlp(X_aug, Y_aug, epochs=args.epochs)

            if args.iters > 1:
                vx_ref = float(X[:, 0].mean())
                af, Ff, ar, Fr = generate_virtual_ss(model, p, vx_ref, Df=Df, Dr=Dr)
                if len(af):
                    Df = fit_D(af, Ff, p)
                    Dr = fit_D(ar, Fr, p)
                    print(f'[sysid] 식별: D_f={Df:.4f}  D_r={Dr:.4f}')

        model.save(Path(args.out))

    elif args.cmd == 'run':
        mlp = ResidualMLP.load(Path(args.weights))
        run_sysid(args.log, mlp, p, n_iters=args.iters, window_sec=args.window)


if __name__ == '__main__':
    main()
