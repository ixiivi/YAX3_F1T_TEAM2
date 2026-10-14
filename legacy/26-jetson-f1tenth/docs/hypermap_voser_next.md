# HyperMAP Phase 1 — Voser μ 측정 핸드오프 (2026-08-09)

## 지금까지 완료한 것

### 1. 실차 파라미터 실측 (완료)

| 파라미터 | 기존 코드값 | 실측값 | 측정 방법 |
|---------|-----------|--------|---------|
| m | 3.74 kg | **3.354 kg** | 저울 직접 측정 |
| 앞 하중 W_f | — | 1510 g | 앞바퀴만 저울에 올리기 |
| 뒤 하중 W_r | — | 1840 g | 뒷바퀴만 저울에 올리기 |
| wheelbase L | 0.33 m | **0.32 m** | 전/후 차축 간 줄자 측정 |
| lf (CoG→앞 차축) | 0.15875 m | **0.1757 m** | W_r/m × L |
| lr (CoG→뒤 차축) | 0.17145 m | **0.1441 m** | W_f/m × L |

CoG가 뒤쪽에 치우침 (W_r > W_f). 기존 코드보다 lf +11%, lr -16% 차이.

### 2. 코드 반영 완료

- `src/race_stack/race_stack/vehicle_sim.py` — m, lf, lr 갱신
- `src/race_stack/race_stack/sysid_nn.py` — m, lf, lr 갱신 (fallback dataclass)
- `src/race_stack/race_stack/slip_friction.py` — wheelbase 0.33→0.32
- `src/race_stack/race_stack/pacejka_fit.py` — Pacejka 피팅 도구 (신규)

### 3. Pacejka ID 시도 결과 (bag 3개, 총 ~3600행)

일반 주행 bag(`bag_20260807_*`)으로 B, C, E 피팅 시도 → **불가능** 판정.

이유 3가지:
1. **vy 측정 불가**: LiDAR ICP vy 노이즈 0.37 m/s (corr 0.17), IMU 적분 위상 오류
2. **포화 미달**: 관측 최대 횡가속도 99th = 0.80g, 타이어 한계(μ>1g) 미달
3. **비정상상태**: 레이싱 주행 = 가속/슬라럼 많아 준정상 구간 부족

### 4. 핵심 발견: MAP 컨트롤러에서 B,C,E보다 μ가 중요

`map_controller.py:106` 주석 확인:
```python
# mass/lf/lr 은 조향 결과에 영향을 주지 않는다 — a_lat/(mu·g) 만 남는다.
# 실제로 조향을 바꾸는 건 mu 하나다.
```

MAP LUT = Pacejka 곡선의 역함수 테이블.
- B, C, E: 곡선 모양 (MAP 논문 F1TENTH 동일 하드웨어값 그대로 사용 OK)
- **μ**: 전체 횡력 스케일 → **이게 우리 차에 맞게 캘리브레이션해야 하는 값**

### 5. IMU 축 매핑 확인 (중요)

```python
# slip_friction.py 기준
car_ay = +msg.linear_acceleration.x * G   # 차체 횡가속도 (g → m/s²)
car_ax = -msg.linear_acceleration.y * G   # 차체 종가속도
car_wz = msg.angular_velocity.z * DEG2RAD # 요레이트
```

`IMU.ax` (= 차체 횡가속도)와 원심가속도 `vx·ω`의 상관계수 = 0.93. 신뢰 가능.

---

## 다음 할 일: Voser 정속 원형 주행으로 μ 측정

### 목표

μ (노면 마찰계수) 식별. 공식:
```
μ = max(|ay_car|) / g    (슬립 직전 속도에서)
```

### 실험 절차

1. **바닥에 원 마킹**: 반지름 R = 1.0~1.5 m 권장 (테이프로 표시)

2. **race.sh로 bag 녹화 시작**:
   ```bash
   ./race.sh log voser_mu
   ```

3. **원형 정속 주행 (조이스틱 수동)**:
   - 조향 고정 (동일 반지름 유지)
   - 속도 0.5 m/s → 1.0 → 1.5 → 2.0 → 2.5 m/s 단계별로 올리기
   - 각 속도에서 **최소 5바퀴** 돌기 (준정상상태)
   - 바퀴가 미끄러지면 직전 속도 기록

4. **bag 분석**: 아래 스크립트로 μ 추출

### 분석 스크립트

```bash
# 젯슨에서 실행 (rosbag2_py + numpy 사용)
python3 ~/yax/src/race_stack/race_stack/pacejka_fit.py \
    ~/bags/voser_mu_*.db3 --plot --out ~/yax/docs/voser_result.yaml
```

또는 직접 분석:
```python
# bag에서 IMU.ax (차체 ay) 뽑기
# car_ay = +msg.linear_acceleration.x * 9.81  (g → m/s²)
# μ_lower = max(|car_ay|) / 9.81  (준정상 구간, vx > 0.5 필터)
```

### 분석 후 반영할 파일

`~/yax/src/race_stack/race_stack/vehicle_sim.py`:
```python
mu: float = X.XXXX   # Voser 실측 (2026-08-09)
```

`~/yax/src/race_stack/race_stack/map_controller.py` (파라미터):
```
mu: X.XXXX
```

---

## 현재 vehicle_sim.py 파라미터 상태

```python
m  = 3.354    # kg  실측 ✓
lf = 0.1757   # m   실측 ✓
lr = 0.1441   # m   실측 ✓
Iz = 0.050    # kg·m²  추정값 (미측정)
mu = 1.0489   # MAP 논문값 → Voser 실험 후 교체 예정
pac_B = 3.629  # MAP 논문값 (유지)
pac_C = 1.3    # MAP 논문값 (유지)
pac_D = 1.2084 # MAP 논문값 (유지)
pac_E = -0.5   # MAP 논문값 (유지)
```

---

## 참고: HyperMAP 전체 구조

```
[1] SysID NN (2411.17508)
    → D_new 추정 (μ 변화 감지)

[2] LUT 재생성
    LUT_new = LUT_ref × (D_new / D_ref)
    → ref는 Voser로 캘리브레이션한 μ 기준

[3] 갱신 규칙
    → rate limit + low-pass 블렌딩

[4] MAP 컨트롤러
    → 갱신된 LUT로 슬립각 보정 조향
```
