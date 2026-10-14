#!/usr/bin/env python3
"""ontrack_sysid — 주행 로그에서 Pacejka 타이어 파라미터를 식별한다.

  Dikici·Ghignone et al., "Learning-Based On-Track System Identification for
  Scaled Autonomous Racing in Under a Minute" (arXiv:2411.17508) 구현.

왜 이 방식인가
─────────────
정상상태 점별 역산(Voser/MAP 방식)은 샘플마다 α 를 계산하므로 v_y 노이즈가
그대로 α 로 들어간다. 2026-08-07 실측에서 v_y 의 표준편차가 0.238㎧ 였고,
v=2.3㎧ 에서 이는 α 에 ±6.0° — Pacejka 관심영역(0~8.6°)과 맞먹는다. 그래서
점별 피팅은 파라미터가 경계에 붙고 곡선 모양이 안 나왔다.

이 방법은 점별 역산을 하지 않는다. 명목 모델을 **전진 적분**해 한 스텝 예측을
만들고, NN 이 그 잔차를 학습한 뒤, 보정된 모델로 **가상 정상상태 데이터**를
생성해 거기서 Pacejka 를 뽑는다. 노이즈가 데이터셋 전체에 평균화되고, 최종
피팅은 노이즈 없는 가상 데이터 위에서 이뤄진다.

의존성
──────
numpy 만 쓴다. NN 이 58개 파라미터(4-8-2)뿐이라 torch 가 필요 없고, 젯슨의
numpy 1.26.4 고정 환경을 건드리지 않는다.

사용
────
  ros2 run race_stack ontrack_sysid -- logs/slip_tests/*_friction.csv \\
      --mass 3.354 --lf 0.1757 --lr 0.1441 --izz 0.050
  ※ 2026-08-09 이전에 녹화한 bag/CSV 는 --speed-scale 0.911 을 줄 것

  ★ 제원(mass/lf/lr/izz)은 반드시 실측값으로 바꿀 것. 기본값은 자리표시자다.
    --sweep-izz 로 I_z 민감도를 먼저 볼 수 있다.
"""
import argparse, csv, glob, math, os, sys
import numpy as np

TS_DEFAULT = 0.02


# ── 데이터 ────────────────────────────────────────────────────────────
def load_csvs(paths, speed_scale, omega_src='wz_lidar'):
    """slip_friction CSV 들을 읽어 (t, v_x, v_y, ω, δ) 로 만든다.

    파일 경계에서 t 가 리셋되므로 연속 구간(segment)으로 잘라 돌려준다 —
    한 스텝 예측은 연속된 두 샘플에서만 의미가 있다.
    """
    segs = []
    for p in sorted(paths):
        rows = []
        with open(p) as f:
            r = csv.reader(f)
            head = next(r)
            if not head or not head[0].startswith('#'):
                f.seek(0);  r = csv.reader(f)
            hdr = next(r)
            for x in r:
                try:
                    rows.append([float(v) for v in x])
                except ValueError:
                    continue
        if not rows:
            continue
        d = np.array(rows);  C = {n: i for i, n in enumerate(hdr)}
        need = ('t', 'v_wheel', 'delta', omega_src)
        if any(k not in C for k in need):
            print(f'  건너뜀 {os.path.basename(p)} — 컬럼 부족 {need}')
            continue
        vy_col = 'vy_base' if 'vy_base' in C else 'vy_lidar'
        if vy_col == 'vy_lidar':
            print(f'  ⚠ {os.path.basename(p)}: vy_base 없음 → vy_lidar 사용 '
                  f'(레버암 미보정 데이터)')
        t  = d[:, C['t']]
        vx = d[:, C['v_wheel']] / speed_scale        # 줄자 보정
        vy = d[:, C[vy_col]]
        w  = d[:, C[omega_src]]
        dl = d[:, C['delta']]
        brk = np.where(np.diff(t) <= 0)[0]
        for s in np.split(np.arange(len(t)), brk + 1):
            if len(s) > 50:
                segs.append(dict(t=t[s], vx=vx[s], vy=vy[s], w=w[s], d=dl[s],
                                 src=os.path.basename(p)))
    return segs


def resample(seg, ts):
    """등간격 ts 로 다시 샘플링한다. Euler 적분이 균일 dt 를 전제한다."""
    t = seg['t'];  tn = np.arange(t[0], t[-1], ts)
    out = {k: np.interp(tn, t, seg[k]) for k in ('vx', 'vy', 'w', 'd')}
    out['t'] = tn;  out['src'] = seg['src']
    return out


def lowpass_nc(x, ts, fc):
    """비인과 저역통과 (전진+후진 1차 IIR). 위상 지연을 안 만든다 — 논문 III-C-2.

    scipy.filtfilt 대신 손으로 두 번 도는 이유: 의존성을 numpy 로 묶기 위해서다.
    """
    a = ts / (ts + 1.0 / (2 * math.pi * fc))
    y = np.empty_like(x);  acc = x[0]
    for i in range(len(x)):
        acc += a * (x[i] - acc);  y[i] = acc
    z = np.empty_like(y);  acc = y[-1]
    for i in range(len(y) - 1, -1, -1):
        acc += a * (y[i] - acc);  z[i] = acc
    return z


def mirror(X, Y):
    """좌우 대칭 증강 — v_y, ω, δ 부호 반전, v_x 유지 (논문 III-C-2).

    한 방향으로만 돈 데이터의 편향을 없앤다. 2026-08-07 스윕은 좌회전 61% /
    우회전 27% 로 치우쳐 있었으므로 이게 특히 중요하다.
    """
    Xm = X.copy();  Xm[:, 1:] *= -1.0        # v_x 만 그대로
    return np.vstack([X, Xm]), np.vstack([Y, -Y])


# ── 차량 모델 ─────────────────────────────────────────────────────────
class Vehicle:
    def __init__(self, m, lf, lr, izz):
        self.m, self.lf, self.lr, self.izz = m, lf, lr, izz

    def slip(self, vx, vy, w, d):
        vx = np.maximum(vx, 0.3)
        af = d - np.arctan((vy + self.lf * w) / vx)
        ar =   - np.arctan((vy - self.lr * w) / vx)
        return af, ar

    def step(self, vx, vy, w, d, phi, ts, sub=1):
        """명목 모델 한 스텝 (논문 eq. 4).

        ★ sub: 내부 서브스텝 수 (2026-08-07 추가). 전진 오일러는
          |1 + ts·λ| < 1 이어야 안정한데, 요 방정식의 고유값이
          λ ≈ −(C_αf·l_f² + C_αr·l_r²)/(I_z·v_x) 로 스케일 차량에서는
          −150/s 를 넘기 쉽다. 그러면 ts=0.02 에서 발산한다 — 실제로
          가상 정상상태 생성이 진동해 (α, F) 가 참 곡선에서 6N 벗어났다.
          한 스텝 예측(학습용)은 매번 측정값에서 출발하므로 누적되지 않아
          sub=1 로 충분하지만, 500스텝을 굴리는 시뮬레이션은 서브스텝이 필요하다.
        """
        h = ts / sub
        for _ in range(sub):
            af, ar = self.slip(vx, vy, w, d)
            Fyf = pacejka(phi[:4], af)
            Fyr = pacejka(phi[4:], ar)
            vy = vy + (Fyr + Fyf * np.cos(d) - self.m * vx * w) / self.m * h
            w  = w  + (Fyf * self.lf * np.cos(d) - Fyr * self.lr) / self.izz * h
        return vy, w

    def euler_substeps(self, phi, vx, ts, margin=4.0):
        """안정성에서 필요한 서브스텝 수를 정한다 (선형화 고유값 기준)."""
        caf = abs(phi[0] * phi[1] * phi[2])        # B·C·D ≈ 코너링 강성
        car = abs(phi[4] * phi[5] * phi[6])
        vx = max(float(vx), 0.5)
        lam_w  = (caf * self.lf ** 2 + car * self.lr ** 2) / (self.izz * vx)
        lam_vy = (caf + car) / (self.m * vx)
        lam = max(lam_w, lam_vy, 1e-6)
        return int(max(1, math.ceil(margin * lam * ts / 2.0)))


def pacejka(p, a):
    B, C, D, E = p
    return D * np.sin(C * np.arctan(B * a - E * (B * a - np.arctan(B * a))))


# ── NN (4-8-2 MLP, 58 파라미터) ───────────────────────────────────────
class MLP:
    """논문 IV-A: 입력 4, 은닉 8, 출력 2. 입력·은닉 LeakyReLU, 출력 선형.
    Adam lr 5e-4, MSE, 전체 배치."""

    def __init__(self, nin=4, nh=8, nout=2, seed=0):
        r = np.random.default_rng(seed)
        s = lambda a, b: r.normal(0, math.sqrt(2.0 / a), (a, b))
        self.W = [s(nin, nh), s(nh, nout)]
        self.b = [np.zeros(nh), np.zeros(nout)]
        self.mW = [np.zeros_like(w) for w in self.W]
        self.vW = [np.zeros_like(w) for w in self.W]
        self.mb = [np.zeros_like(x) for x in self.b]
        self.vb = [np.zeros_like(x) for x in self.b]
        self.k = 0

    @staticmethod
    def _lr(x):  return np.where(x > 0, x, 0.01 * x)
    @staticmethod
    def _dlr(x): return np.where(x > 0, 1.0, 0.01)

    def forward(self, X):
        self.z0 = self._lr(X)                    # 입력층 LeakyReLU
        self.z1 = self.z0 @ self.W[0] + self.b[0]
        self.a1 = self._lr(self.z1)
        return self.a1 @ self.W[1] + self.b[1]

    def train(self, X, Y, epochs=1500, lr=5e-4, b1=.9, b2=.999, eps=1e-8):
        n = len(X)
        for _ in range(epochs):
            out = self.forward(X)
            e = (out - Y) / n
            gW1 = self.a1.T @ e;              gb1 = e.sum(0)
            da1 = (e @ self.W[1].T) * self._dlr(self.z1)
            gW0 = self.z0.T @ da1;            gb0 = da1.sum(0)
            self.k += 1
            for i, (gW, gb) in enumerate(((gW0, gb0), (gW1, gb1))):
                self.mW[i] = b1 * self.mW[i] + (1 - b1) * gW
                self.vW[i] = b2 * self.vW[i] + (1 - b2) * gW ** 2
                self.mb[i] = b1 * self.mb[i] + (1 - b1) * gb
                self.vb[i] = b2 * self.vb[i] + (1 - b2) * gb ** 2
                mw = self.mW[i] / (1 - b1 ** self.k);  vw = self.vW[i] / (1 - b2 ** self.k)
                mb = self.mb[i] / (1 - b1 ** self.k);  vb = self.vb[i] / (1 - b2 ** self.k)
                self.W[i] -= lr * mw / (np.sqrt(vw) + eps)
                self.b[i] -= lr * mb / (np.sqrt(vb) + eps)
        return float(np.mean((self.forward(X) - Y) ** 2))


# ── Pacejka 최소자승 ──────────────────────────────────────────────────
#  ★ 물리 경계 (2026-08-07). MAP 논문 III-B 가 "C 를 1.5, E 를 1.1 로 제한해
#    높은 슬립각에서 곡선이 0 으로 되꺾이는 것을 막았다" 고 한 것과 같은 취지다.
#    느슨하게 두면 C=2.0, E=−3.0 같은 값이 나오는데, E 가 크게 음수면 Pacejka
#    내부 항이 초선형으로 커져 모델이 수치적으로 폭발한다 — 실제로 그 값이
#    다음 반복의 가상 정상상태 시뮬을 발산시켰다.
B_LIM = (1.0, 40.0)
C_LIM = (1.0, 1.70)
E_LIM = (-1.0, 1.10)


def fit_pacejka(alpha, Fy, p0):
    """가우스-뉴턴 대신 좌표하강 + 격자 (scipy 의존을 피한다)."""
    from itertools import product
    clip = lambda q: (B_LIM[0] <= q[0] <= B_LIM[1] and C_LIM[0] <= q[1] <= C_LIM[1]
                      and q[2] > 0 and E_LIM[0] <= q[3] <= E_LIM[1])
    best, bp = np.inf, np.array(p0, float)
    for B, C, D, E in product(np.linspace(*B_LIM, 10), np.linspace(*C_LIM, 5),
                              [np.percentile(np.abs(Fy), 98) * k for k in (0.9, 1.0, 1.15)],
                              np.linspace(*E_LIM, 5)):
        r = np.mean((pacejka((B, C, D, E), alpha) - Fy) ** 2)
        if r < best:  best, bp = r, np.array([B, C, D, E], float)
    step = np.array([2.0, 0.15, max(bp[2] * .1, .1), 0.2])
    for _ in range(200):
        improved = False
        for i in range(4):
            for s in (+1, -1):
                q = bp.copy();  q[i] += s * step[i]
                if not clip(q):
                    continue
                r = np.mean((pacejka(q, alpha) - Fy) ** 2)
                if r < best:  best, bp, improved = r, q, True
        if not improved:
            step *= 0.5
            if step[1] < 1e-4:  break
    return bp, math.sqrt(best)


# ── 본체 ──────────────────────────────────────────────────────────────
def virtual_steady_state(veh, phi, nn, vx, ts=0.02, dur=10.0, dmax=0.4, sub=None):
    """보정 모델을 정속·조향 램프로 굴려 가상 정상상태 데이터를 만든다.
    논문 III-E-2. 여기서 나온 데이터는 노이즈가 없다 — 그게 핵심이다."""
    n = int(dur / ts)
    d = np.linspace(0, dmax, n)
    vy = w = 0.0
    VY, W = np.empty(n), np.empty(n)
    # ★ sub 는 반드시 학습 때와 같아야 한다. NN 은 "sub 스텝짜리 명목 예측의
    #   잔차"를 배운 것이므로, 다른 sub 로 굴린 예측에 그 잔차를 더하면
    #   기준이 어긋나 생성 데이터가 곡선에서 벗어난다 (2026-08-07 디버깅).
    if sub is None:
        sub = veh.euler_substeps(phi, vx, ts)

    # ★ NN 보정 가드 (2026-08-07). 논문은 잔차를 매 스텝 그대로 더하는데,
    #   MLP 는 유계가 아니라 학습 분포를 벗어나면 임의로 큰 값을 낸다. 그게
    #   500스텝 누적되면 시뮬이 발산한다 — 실제로 ω 가 0~2.3 에서 −3.3~+2.4 로,
    #   α_f 가 +47° 까지 튀어 생성 데이터가 통째로 못 쓰게 됐다.
    #   그래서 (a) 보정량을 학습 잔차 범위로 자르고 (b) 상태가 학습 범위를
    #   벗어나면 발산으로 보고 알린다. 자르는 건 물리를 왜곡하지 않는다 —
    #   애초에 그 범위 밖에서는 NN 이 아무 근거가 없기 때문이다.
    lim = getattr(nn, 'elim', None) if nn is not None else None
    diverged = False
    for k in range(n):
        vy_n, w_n = veh.step(np.array(vx), np.array(vy), np.array(w),
                             np.array(d[k]), phi, ts, sub=sub)
        if nn is not None:
            e = nn.forward(np.array([[vx, vy, w, d[k]]]))[0]
            if lim is not None:
                e = np.clip(e, lim[0], lim[1])
            vy_n = vy_n + e[0];  w_n = w_n + e[1]
        vy, w = float(vy_n), float(w_n)
        if not (abs(vy) < 5 * max(vx, 1.0) and abs(w) < 50.0):
            diverged = True;  break
        VY[k], W[k] = vy, w
    if diverged:
        return None, None, None
    return d, VY, W


def identify(segs, veh, phi0, ts, iters, epochs, lpf_hz, seed, quiet=False):
    # 전처리 — 비인과 LPF 후 등간격 리샘플
    X, Y = [], []
    for s in segs:
        s = resample(s, ts)
        for k in ('vx', 'vy', 'w', 'd'):
            s[k] = lowpass_nc(s[k], ts, lpf_hz)
        X.append(np.c_[s['vx'][:-1], s['vy'][:-1], s['w'][:-1], s['d'][:-1]])
        Y.append(np.c_[s['vy'][1:],  s['w'][1:]])
    X = np.vstack(X);  Y = np.vstack(Y)
    X, Y = mirror(X, Y)
    vx_mean = float(np.mean(X[:, 0]))
    if not quiet:
        print(f'학습 샘플 {len(X)}개 (좌우 증강 포함)  평균 v_x {vx_mean:.2f}㎧')

    phi = np.array(phi0, float)
    hist = []
    for it in range(1, iters + 1):
        # 학습과 시뮬레이션이 같은 적분기를 써야 한다 (아래 ② 주석 참고)
        sub = veh.euler_substeps(phi, vx_mean, ts)

        # ① 명목 모델 한 스텝 예측의 잔차를 NN 이 학습
        vy_h, w_h = veh.step(X[:, 0], X[:, 1], X[:, 2], X[:, 3], phi, ts, sub=sub)
        E = np.c_[Y[:, 0] - vy_h, Y[:, 1] - w_h]
        nn = MLP(seed=seed + it)
        mse = nn.train(X, E, epochs=epochs)
        # 보정량을 학습 잔차의 실제 범위로 제한한다 (위 가드 설명 참고)
        nn.elim = (np.percentile(E, 1, axis=0), np.percentile(E, 99, axis=0))

        # ② 보정 모델로 가상 정상상태 데이터 생성 (같은 sub 로)
        d, VY, W = virtual_steady_state(veh, phi, nn, vx_mean, ts, sub=sub)
        if d is None:
            # NN 보정이 발산했다 — 이번 반복은 명목 모델만으로 생성한다.
            # 조용히 넘어가면 "수렴한 것처럼" 보이므로 반드시 알린다.
            if not quiet:
                print(f'  반복 {it}: ⚠ NN 보정 발산 → 명목 모델로 대체 생성')
            d, VY, W = virtual_steady_state(veh, phi, None, vx_mean, ts, sub=sub)
            if d is None:
                if not quiet:
                    print(f'  반복 {it}: ★ 명목 모델도 발산 — 중단')
                break

        # ③ 정상상태 가정 → 축별 횡력과 슬립각 → Pacejka 최소자승 (eq. 8, 3)
        L = veh.lf + veh.lr
        Fyr = veh.m * veh.lf / L * vx_mean * W
        Fyf = veh.m * veh.lr / L * vx_mean * W / np.cos(d)
        af, ar = veh.slip(np.full_like(W, vx_mean), VY, W, d)
        pf, rf = fit_pacejka(af, Fyf, phi[:4])
        pr, rr = fit_pacejka(ar, Fyr, phi[4:])
        phi = np.concatenate([pf, pr])
        hist.append(dict(it=it, mse=mse, phi=phi.copy(), rf=rf, rr=rr))
        if not quiet:
            print(f'  반복 {it}: NN MSE {mse:.3e} | '
                  f'전륜 B={pf[0]:5.2f} C={pf[1]:.2f} D={pf[2]:6.2f} E={pf[3]:+.2f} | '
                  f'후륜 B={pr[0]:5.2f} C={pr[1]:.2f} D={pr[2]:6.2f} E={pr[3]:+.2f}')
    return phi, hist, X, Y


def one_step_rmse(X, Y, veh, phi, ts):
    vy_h, w_h = veh.step(X[:, 0], X[:, 1], X[:, 2], X[:, 3], phi, ts)
    return (float(np.sqrt(np.mean((Y[:, 0] - vy_h) ** 2))),
            float(np.sqrt(np.mean((Y[:, 1] - w_h) ** 2))))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('csvs', nargs='+', help='slip_friction CSV (glob 가능)')
    ap.add_argument('--mass', type=float, default=3.5,  help='★ 실측할 것 [kg]')
    ap.add_argument('--lf',   type=float, default=0.165, help='★ 무게중심→전축 [m]')
    ap.add_argument('--lr',   type=float, default=0.165, help='★ 무게중심→후축 [m]')
    ap.add_argument('--izz',  type=float, default=0.04, help='★ 요 관성모멘트 [kg·m²]')
    # 2026-08-09 vesc.yaml 수정 이후 1.0. 그 이전 bag 은 0.911 을 줄 것.
    ap.add_argument('--speed-scale', type=float, default=1.0,
                    help='표시속도/실제속도. odom_scale 로 잰 값')
    ap.add_argument('--omega', default='wz_lidar', choices=['wz_lidar', 'wz_imu'])
    ap.add_argument('--ts',     type=float, default=TS_DEFAULT)
    ap.add_argument('--iters',  type=int,   default=6)
    ap.add_argument('--epochs', type=int,   default=1500)
    ap.add_argument('--lpf-hz', type=float, default=5.0)
    ap.add_argument('--seed',   type=int,   default=0)
    ap.add_argument('--sweep-izz', action='store_true',
                    help='I_z 를 훑어 민감도만 본다 (실측 전 판단용)')
    a = ap.parse_args([x for x in (argv or sys.argv[1:]) if x != '--'])

    paths = [p for g in a.csvs for p in (glob.glob(g) or [g])]
    segs = load_csvs(paths, a.speed_scale, a.omega)
    if not segs:
        print('★ 읽을 데이터가 없다');  return 1
    print(f'{len(paths)}개 파일 → 연속구간 {len(segs)}개, '
          f'총 {sum(len(s["t"]) for s in segs)}행\n')

    phi0 = [10.0, 1.5, 12.0, 0.9, 10.0, 1.5, 12.0, 0.9]   # 명목 초기값

    if a.sweep_izz:
        print('I_z 민감도 (다른 제원 고정)')
        for izz in (0.02, 0.03, 0.04, 0.06, 0.08):
            veh = Vehicle(a.mass, a.lf, a.lr, izz)
            phi, hist, X, Y = identify(segs, veh, phi0, a.ts, 3, a.epochs,
                                       a.lpf_hz, a.seed, quiet=True)
            r = one_step_rmse(X, Y, veh, phi, a.ts)
            print(f'  I_z={izz:.3f}: 전륜 C={phi[1]:.2f} D={phi[2]:6.2f} | '
                  f'후륜 C={phi[5]:.2f} D={phi[6]:6.2f} | '
                  f'1스텝 RMSE v_y {r[0]:.4f} ω {r[1]:.4f}')
        return 0

    veh = Vehicle(a.mass, a.lf, a.lr, a.izz)
    print(f'제원: m={a.mass}kg  l_f={a.lf}m  l_r={a.lr}m  I_z={a.izz}kg·m²')
    print(f'      (기본값을 그대로 썼다면 결과의 D 는 믿지 말 것)\n')
    phi, hist, X, Y = identify(segs, veh, phi0, a.ts, a.iters, a.epochs,
                               a.lpf_hz, a.seed)

    r0 = one_step_rmse(X, Y, veh, np.array(phi0), a.ts)
    r1 = one_step_rmse(X, Y, veh, phi, a.ts)
    print(f'\n한 스텝 예측 RMSE   v_y: {r0[0]:.4f} → {r1[0]:.4f} ㎧'
          f'    ω: {r0[1]:.4f} → {r1[1]:.4f} rad/s')
    print(f'\n식별 결과')
    for nm, p in (('전륜', phi[:4]), ('후륜', phi[4:])):
        print(f'  {nm}  B={p[0]:6.3f}  C={p[1]:.3f}  D={p[2]:7.3f} N  E={p[3]:+.3f}'
              + ('   ← C 가 물리범위(1.3~1.7) 밖' if not 1.2 < p[1] < 1.8 else ''))
    out = 'pacejka_identified.yaml'
    with open(out, 'w') as f:
        f.write('# ontrack_sysid (arXiv:2411.17508) 식별 결과\n')
        f.write(f'# 제원 m={a.mass} lf={a.lf} lr={a.lr} izz={a.izz} '
                f'speed_scale={a.speed_scale}\n')
        for nm, p in (('front', phi[:4]), ('rear', phi[4:])):
            f.write(f'{nm}: {{B: {p[0]:.4f}, C: {p[1]:.4f}, '
                    f'D: {p[2]:.4f}, E: {p[3]:.4f}}}\n')
    print(f'\n저장: {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
