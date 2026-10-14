# SysID NN 구현 — arXiv 2411.17508 (Dikici et al.)

**날짜:** 2026-08-07  
**관련 파일:** `src/race_stack/race_stack/sysid_nn.py`, `vehicle_sim.py`, `setup.py`

---

## 배경

HyperMAP의 1단계: 실차 주행 로그 30초로 Pacejka D 파라미터를 실시간 식별해서 현재 노면 마찰계수 μ를 추정한다.

논문 2411.17508의 파이프라인을 따른다:

```
실차 로그 (30s)
    ↓
Residual MLP 학습 (4→8→2, 58 params)
    ↓
보정 모델(Nominal + NN)로 가상 정상상태 생성
    ↓
선형 LS: Pacejka D 식별
    ↓
μ̂ = D_fit / D_nom × μ_nom
```

---

## 구현한 내용

### 파일: `sysid_nn.py`

| 함수 | 역할 |
|------|------|
| `nominal_step()` | Euler 1-스텝 적분 (Pacejka 횡력, eq.4) |
| `ResidualMLP` | 4→8→2 MLP, LeakyReLU, 58 params |
| `gen_sim_data()` | 시뮬로 학습 데이터 생성 (`mu_fixed` 인자로 단일/복수 μ 선택) |
| `load_log_data()` | slip_friction CSV 로그 → (X, Y) |
| `augment_noise()` | η=0.2~1.4 노이즈 보강 (논문 §III.C.2) |
| `train_mlp()` | Adam + MSE 전체배치 학습 |
| `generate_virtual_ss()` | 보정 모델로 가상 SS 생성 → (α, Fy_norm) |
| `fit_D()` | 선형 LS: D = Σ(Fy·s) / Σ(s²) |
| `run_sysid()` | 로그 CSV → μ̂ (반복 학습 지원) |

**CLI:**
```bash
# 학습 (시뮬 데이터 자동 생성)
python3 sysid_nn.py train --epochs 500

# 학습 (실차 로그 사용)
python3 sysid_nn.py train --log logs/slip_tests/xxx_friction.csv

# μ 추정
python3 sysid_nn.py run --log logs/slip_tests/xxx_friction.csv
```

### 파일: `vehicle_sim.py`

단일-트랙(자전거) 동역학 모델 + Pacejka 타이어 모델.  
MAP LUT 생성 CLI: `ros2 run race_stack gen_lut --out lut.csv`

---

## 버그 수정 — `generate_virtual_ss()` 슬립각 오류

### 원인

가상 정상상태 생성 루프에서 슬립각을 **vy=0 가정**으로 계산하고 있었다.

```python
# 이전 (틀림) — vy=0 가정
alpha_r = math.atan2(p.lr * om_k, vx_ref)
alpha_f = delta - math.atan2(p.lf * om_k, vx_ref)
```

정상상태에서 vy는 0이 아니다. vx=2 m/s, δ=0.2 rad 조건에서:
- 실제 vy ≈ 0.125 m/s
- vy=0 가정 시 α_r = **5.94°** (과대)
- 실제 vy 사용 시 α_r = **2.40°** (정확)

Pacejka 곡선에서 5.94°는 포화 영역에, 2.40°는 선형 영역에 있어서 D 피팅 결과가 완전히 틀어진다.

### 수치 영향

| | vy=0 가정 (이전) | 실제 vy 사용 (수정 후) |
|---|---|---|
| α_r | 5.94° | 2.40° |
| D_fit | **0.54** | **1.27** |
| D_nom | 1.2675 | 1.2675 |
| μ̂ | ~0.45 (완전히 틀림) | ~0.70 (정확) |

### 수정

```python
# 수정 후 — 시뮬에서 적분된 vy_next 사용
vy_k = float(vy_next)
alpha_r = -math.atan2(vy_k - p.lr * om_k, max(abs(vx_ref), EPS))
alpha_f = delta - math.atan2(vy_k + p.lf * om_k, max(abs(vx_ref), EPS))
```

횡력(Fyr, Fyf)은 force balance에서 직접 구하므로 vy와 무관하다 — 수정 대상 아님.

```python
# 횡력 (eq.8) — vy 무관, 수정 없음
Fyr = p.m * p.lf * vx_ref * om_k / L
Fyf = p.m * p.lr * vx_ref * om_k / (L * cos_d)
```

### 검증 결과

단일 μ=0.7로 학습 → 가상 SS → D 피팅:

```
D_nom = 1.2675
D_f   = 0.8460  D_r   = 0.8650
μ̂    = 0.7080   (정답: 0.70)
오차  = 0.008
```

---

## 슬립각 부호 규칙

`nominal_step()`과 `generate_virtual_ss()` 모두 동일 규칙 사용:

```
alpha_r = -arctan2(vy - lr·ω,  vx)     # 후축
alpha_f =  delta - arctan2(vy + lf·ω, vx)  # 전축
```

- α > 0 → Fy > 0 (좌회전 시 구심력 방향) ✓
- `vehicle_sim.py`의 `slip_alpha()`와 동일 (소각 근사)

---

## setup.py 변경

```python
'gen_lut   = race_stack.vehicle_sim:main',
'sysid_nn  = race_stack.sysid_nn:main',   # 추가
```

재빌드 필요: `colcon build --packages-select race_stack --symlink-install`

---

## 차량 파라미터

ETH ForzaETH 팀 값 그대로 사용 (시뮬 전용 연구이므로 허용):

| 파라미터 | 값 | 비고 |
|----------|-----|------|
| m | 3.74 kg | |
| lf | 0.15875 m | |
| lr | 0.17145 m | |
| Iz | 0.050 kg·m² | |
| μ | 1.0489 | 공칭 |
| B, C, D, E | 3.629, 1.3, 1.2084, -0.5 | Pacejka |
| D_nom = μ·D | **1.2675** | D 피팅 기준값 |

실차 실험 시 m, lf, lr은 저울로 측정 필요.

---

## 다음 단계

- **Phase 2 검증**: 실차 로그(`slip_friction` CSV) 로 `train → run` 파이프라인 실측 확인
- **Phase 3**: Residual NN 테이블 재생성기 (`LUT_new = LUT_ref × D_new/D_ref + NN 보정`)
