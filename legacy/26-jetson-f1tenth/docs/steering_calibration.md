# 조향 캘리브레이션 (2026-07-13, 1호차)

> **2026-08-07**: 아래 3번 항목의 "직진 → 눈으로 확인 → offset 수정 → 반복" 루프를
> `./race.sh steertrim` (`src/race_stack/race_stack/steer_trim.py`)로 자동화했다 —
> IMU 자이로 적분으로 곡률을 재서 사람 눈보다 정량적으로 트림을 찾고, 수렴하면
> `config/vesc.yaml`의 offset을 직접 계산해 반영한다(스택 재시작은 여전히 필요).
> 6번 항목(조향 게인 실측)은 여전히 수동 — 그건 안 건드림.
>
> ★ 실차 미검증 (2026-08-07 기준) — 문법 체크만 통과한 상태. 아래 사용법대로
> 돌려보고 이상 있으면 이 노트에 추가할 것.

<details>
<summary><b>0. steer_trim.py 사용법</b></summary>

**사전 준비** (코드가 바뀌었을 때만, 최초 1회)
```bash
cd ~/f1tenth_ws   # 또는 RACE_WS
colcon build --packages-select race_stack --symlink-install
```

**1. 공간 확보** — 차 앞뒤로 각 `test_dist`(기본 2m) + 여유. 전진·후진을
반복하니 최소 4~5m 트인 직선 필요.

**2. 실행**
```bash
cd ~/yax   # repo 루트
./race.sh steertrim
```
브링업 뜨고(5초 대기) 노드 시작. 시작 직후 2초간 정지 상태 유지(자이로
바이어스 측정 — 이때 차 건드리지 말 것).

**3. 화면**
```
회차  0/20   trim +0.0000 rad
v +0.00 m/s   leg_dist 0.00/2.0m
마지막[fwd] κ=+0.xxx 1/m  (허용 ±0.01)
전방최소 …m   후방최소 …m (또는 "라이다 FOV 밖")
```
알아서 전진 2m → 정지 → 보정된 trim으로 후진 2m → 다시 전진... 반복. 사람이
할 일은 없음, 지켜보다 이상하면 개입.

**4. 종료 방식**
| 상황 | 결과 |
|---|---|
| 전진·후진 둘 다 κ 오차 작아짐 (수렴) | 자동 종료 + `config/vesc.yaml` offset 갱신 |
| 최대 20회 돌아도 미수렴 | 마지막 trim으로 종료, 콘솔에 "수렴 못함" 표시, 그래도 파일엔 반영됨 |
| `q` 키 | 즉시 중단, **아무것도 저장 안 함** |
| `s` 키 | 지금 하던 다리 끝나면 그 시점 trim으로 저장 후 종료 |
| 전방 장애물 감지(라이다 0.4m 미만) | 자동 E-stop, 저장 안 함 — 다시 `./race.sh steertrim` |

**5. 끝난 뒤** — 콘솔에 `offset X → Y`가 새 값. **바로 반영 안 됨**:
`ackermann_to_vesc`는 시작할 때만 파라미터를 읽으므로, 떠 있는 스택이 있으면
껐다가 `./race.sh q1`(원하는 모드)로 다시 켜야 새 offset이 적용된다.

</details>

## 오늘 한 것

- **직진 시 한쪽으로 휘는 문제 보정** — VESC 조향 중립값 `steering_angle_to_servo_offset` 을 `0.5304` → `0.40` 으로 튜닝
- **휠베이스 실측(33cm)** — `vesc.yaml` 에 `0.25` 로 잘못 들어가 있어 odom이 요 각속도를 32% 과대평가하고 있었음. `0.33` 으로 수정
- **검증 도구 추가** — `yaw_check.py` (odom 누적 회전량을 도(°)로 출력)
- **미해결** — 조향 게인 `-1.2135` 는 아직 실측 안 됨 / 좌우 최대 조향각이 1.75배 비대칭해짐

수정 파일: `~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml` (빌드 완료)

---

<details>
<summary><b>1. 어디를 고쳐야 하나 — 조향 신호 경로</b></summary>

```
조이스틱 → /joy → joystick_teleop → /teleop → ackermann_mux → /ackermann_cmd
  → ackermann_to_vesc   δ[rad] → 서보값[0~1]   ★ 보정 지점
  → vesc_driver         servo_min/max 클리핑 후 서보 출력
```

자율주행(pure_pursuit / gap_follower)도 `/ackermann_cmd` 아래는 **같은 경로**를 탄다.
→ `ackermann_to_vesc` 단계에서 보정해야 수동·자율 모두 적용됨.

⚠️ `joystick_teleop.py` 에 트림을 넣으면 수동 주행만 고쳐지고 자율주행은 그대로 휜다.

</details>

<details>
<summary><b>2. 파라미터 — 무엇이 무엇인가</b></summary>

`~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml`

```yaml
steering_angle_to_servo_gain:   -1.2135   # 기울기
steering_angle_to_servo_offset:  0.40     # 절편 = 중립 위치
servo_min: 0.15
servo_max: 0.85

vesc_to_odom_node:
  ros__parameters:
    wheelbase: 0.33
```

변환식 (`ackermann_to_vesc.cpp:75`):
```cpp
servo = gain × steering_angle + offset
```

| 파라미터 | 의미 |
|---|---|
| `offset` | **조향각 0일 때 서보에 나가는 절대값** = 중립 위치. 이걸 조정하는 게 트림 |
| `gain` | 바퀴를 1 rad 꺾는 데 필요한 서보값 변화량. 서보 PWM → 혼 각도 → 링키지비 → 바퀴 각도 체인 전체의 기울기 |
| `servo_min/max` | 서보 물리 한계. `vesc_driver` 가 여기서 클리핑 |
| `wheelbase` | `vesc_to_odom.cpp:110` 의 각속도 계산 `ω = v·tan(δ)/L` 에 사용 |

`gain` 이 음수인 이유: ROS 관례상 조향각 양수 = 좌회전인데 이 차는 서보값을 올리면 우회전. 링키지 조립 방향 문제.

</details>

<details>
<summary><b>3. 튜닝 절차 — 함정 3가지 먼저</b></summary>

### 함정

1. **`offset` 은 트림 "증분"이 아니라 중립의 "절대값"이다.**
   `0` 을 넣으면 직진 명령 → 서보값 0 → `servo_min: 0.15` 에 걸려 **바퀴가 풀락으로 꺾인 채 주행한다.** (실제로 한 번 밟은 지뢰)
   조정 범위는 **0.50~0.56 안쪽**, 한 번에 **±0.01씩**.

2. **`ros2 param set` 은 안 먹는다.** `ackermann_to_vesc` 가 생성자에서 파라미터를 한 번만 읽고 동적 콜백이 없음 → **노드 재시작 필수**.

3. **`install` 은 심링크가 아니라 실제 복사본이다.**
   - `src` 만 고치면 → 실행에 반영 안 됨
   - `install` 만 고치면 → 다음 빌드 때 날아감
   - **튜닝 중에는 `colcon build` 금지**

### 경로

```bash
# 정식 소스 (최종값 → 빌드 필요)
~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml
# 설치본 (튜닝 중 임시 수정 → 빌드 불필요, 즉시 반영)
~/f1tenth_ws/install/f1tenth_stack/share/f1tenth_stack/config/vesc.yaml
```

### 루프

```
1. ./race.sh joy       → 직진 명령, 휘는 방향/정도 확인
2. Ctrl+C
3. install 쪽 vesc.yaml 의 offset 수정
4. 반복
```

### 보정량 계산 (휠베이스 0.33)

직진 명령으로 거리 **L(m)** 갔을 때 횡방향 이탈 **y(m)** 발생 시:
```
δ = 0.33 × 2y / L²   [rad]
offset 변화량 = δ × 1.2135
```
- 예: 5m 에 50cm 휨 → δ ≈ 0.0132 rad → offset **0.016** 이동
- 감각: **offset 0.01 ≈ 조향 0.47°**
- 부호는 링키지에 따라 다르니 **실험으로 한 번만 확정**

### 최종 반영

```bash
cd ~/f1tenth_ws
colcon build --packages-select f1tenth_stack
grep -n "servo_offset\|wheelbase" install/f1tenth_stack/share/f1tenth_stack/config/vesc.yaml
```

</details>

<details>
<summary><b>4. ⚠️ 좌우 조향각이 비대칭해졌다 (1호차)</b></summary>

offset을 0.5304 → 0.40 으로 크게 옮긴 대가로 클리핑이 한쪽에만 걸린다.

| offset | 양수 방향 최대 | 음수 방향 최대 | 비대칭 |
|---|---|---|---|
| 0.5304 (원래) | 0.313 rad (18.0°) | 0.263 rad (15.1°) | 1.19배 |
| **0.40 (현재)** | **0.206 rad (11.8°)** ← 서보 한계 | **0.360 rad (20.6°)** ← `max_steer` 한계 | **1.75배** |

한쪽은 11.8°에서 서보가 벽에 부딪혀 더 안 꺾인다. 수동 저속에선 체감이 어렵지만, 자율주행은 `max_steer: 0.36` 까지 명령하므로 **한쪽 코너에서만 조향이 모자라 아웃되는 형태**로 나타난다.

**원인**: 6.1°나 트림이 필요했다는 건 타이로드/링키지가 물리적으로 어긋나 있다는 신호.
**정석**: 타이로드로 기계적 중립을 맞추고 → offset은 0.53 근처로 복원 → 남은 잔차(±0.01~0.02)만 소프트웨어로. 좌우 대칭도 회복된다.
**2호차는 처음부터 이 순서로 할 것.**

</details>

<details>
<summary><b>5. 검증 — yaw_check.py 한 바퀴 테스트</b></summary>

`/odom` 의 yaw를 랩(wrap) 풀어서 **누적 회전량**으로 출력한다. 단순 heading은 ±180°에서 뒤집혀 한 바퀴를 셀 수 없기 때문.

```bash
# 터미널 A
cd ~/yax && ./race.sh joy

# 터미널 B
cd ~/yax
source /opt/ros/humble/setup.bash
python3 yaw_check.py
```

바닥에 출발 방향 표시 → 한쪽 풀조향 + 천천히 제자리 한 바퀴 → 돌아오면 `Ctrl+C`.

| 결과 | 해석 |
|---|---|
| 360° ± 5% | 휠베이스 + 게인 OK |
| 360°보다 큼 | odom 과대평가 → 게인이 실제보다 큼 |
| 360°보다 작음 | odom 과소평가 → 게인이 실제보다 작음 |

</details>

<details>
<summary><b>6. 조향 게인 실측 — 각도기 없이 (미완료)</b></summary>

`-1.2135` 는 근거가 없는 값이다. offset이 0.13이나 틀어져 있던 차라 게인도 안 맞을 수 있다.
게인이 틀리면 컨트롤러는 과/부족 조향을 하고, odom은 각속도를 잘못 계산한다 (SLAM/AMCL 정확도 저하).

**원리**: 자전거 모델에서 `δ = atan(0.33 / R)` — 원의 반지름만 재면 조향각이 역산된다.

> ⚠️ `yaw_check.py` 의 odom yaw를 이 용도로 쓰면 안 됨. odom 각속도 자체가 검증 대상인 게인으로 계산되므로 순환논리. **바닥에 그려진 실제 원**을 자로 재야 한다.

**모터는 안 돌린다.** 서보만 고정하고 차를 손으로 밀면 원을 그린다. 타이어 슬립이 없어 더 정확하고 안전하다.

```bash
# 준비: race.sh 가 떠 있으면 반드시 Ctrl+C
#       (joy가 20Hz autorepeat로 서보 명령을 계속 쏴서 수동 발행을 덮어씀)

# 터미널 A — VESC 드라이버만 단독 기동
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/setup.bash
ros2 run vesc_driver vesc_driver_node --ros-args \
  --params-file ~/f1tenth_ws/install/f1tenth_stack/share/f1tenth_stack/config/vesc.yaml

# 터미널 B — 서보를 한쪽 끝에 고정 (Ctrl+C 전까지 유지)
source /opt/ros/humble/setup.bash
ros2 topic pub -r 20 /commands/servo/position std_msgs/msg/Float64 "{data: 0.15}"
# 차를 손으로 천천히 밀어 원 한 바퀴 → 지름 측정
# Ctrl+C 후 반대쪽 반복
ros2 topic pub -r 20 /commands/servo/position std_msgs/msg/Float64 "{data: 0.85}"
```

**R의 기준점**: 자전거 모델의 R은 **뒷바퀴 축 중심**까지의 거리.
- (정확) 뒤축 한가운데에 분필/매직을 붙여 바닥에 선이 그려지게 함
- (간편) 뒤축 중심을 바닥에 표시 → 정확히 반 바퀴(차 머리가 정반대) 민 뒤 다시 표시 → **두 점 사이 거리 = 지름**

**계산** (좌회전 δ 를 양수로):
```
δ₁ = atan(0.33 / R₁)   (servo 0.15)
δ₂ = atan(0.33 / R₂)   (servo 0.85)

gain   = (0.85 − 0.15) / (δ₂ − δ₁)
offset = 0.15 − gain × δ₁      ← 진짜 중립값. 주행으로 찾은 0.40 과 교차 검증됨
```

**공간**: 예상 지름 약 3.1m / 1.7m → 4×4m 필요. 천천히 밀 것 (빨리 밀면 슬립으로 원이 커짐).

</details>

<details>
<summary><b>7. 그 외 발견 — max_steer, 휠베이스 불일치</b></summary>

### `max_steer: 0.36` 은 근거 없는 관례값

물리 한계가 아니라 **컨트롤러가 스스로 거는 소프트 클립**이다. 물리 한계는 `servo_min/max` + 링키지 스토퍼가 정한다.
업스트림 `joy_teleop.yaml` 은 0.34, f1tenth_gym 시뮬레이터 물리 한계는 0.4189 rad(24°), `q2_gap_follow` 만 0.4. 코드 어디에도 유도 근거가 없다.

⚠️ **무심코 올리지 말 것** — `gap_follower.py:99` 에서 감속 로직의 정규화 분모로도 쓰인다:
```python
speed *= max(0.4, 1.0 - abs(steer) / self.p('max_steer') * 0.6)
```
올리면 같은 조향각에서 감속이 **덜** 걸려, 코너 진입 속도가 올라가 오히려 아웃된다. (분리 필요 — TODO)

### 휠베이스가 세 군데에서 달랐다

| 위치 | 전 | 후 |
|---|---|---|
| 컨트롤러 전부 (`race_modes/*.yaml`, `race_script.yaml`, pure_pursuit, stanley, roboracer_1v1) | 0.33 | 0.33 (맞았음) |
| `vesc.yaml` (vesc_to_odom) | **0.25** ❌ | **0.33** ✅ |
| `vesc_to_odom_node.launch.xml` | 0.2 | 0.2 (미사용 — bringup이 yaml을 직접 로드) |

`.25` 는 업스트림 주석 `# car wheelbase is about 25cm` 이 그대로 따라온 값.
**2호차에도 그대로 있을 것 — 반드시 확인.**

</details>

---

## 2호차 체크리스트

- [ ] **휠베이스 실측** (앞뒤 **축 중심** 거리) → `vesc.yaml` + `race_script.yaml` + `config/race_modes/*.yaml`(5개) + 각 노드 `declare_parameter` 기본값에 동일하게 반영
- [ ] **타이로드/링키지 기계적 중립 먼저** (소프트웨어 트림으로 때우면 좌우 조향각이 비대칭해짐 — 4번 토글)
- [ ] **조향 게인 실측** (6번 토글) → `steering_angle_to_servo_gain` 갱신
- [ ] **offset 튜닝** (3번 토글) → 링키지가 맞았다면 0.53 ± 0.02 안에서 끝나야 정상
- [ ] `colcon build --packages-select f1tenth_stack` 후 install 반영 확인
- [ ] `yaw_check.py` 로 360° ± 5% 검증
- [ ] 좌우 풀조향 걸어보고 한쪽만 덜 꺾이지 않는지 확인

## 남은 TODO

- `steering_angle_to_servo_gain` 실측 (1·2호차 모두)
- 1호차 타이로드 기계적 정렬 → offset 0.53 근처 복원 → 좌우 대칭 회복
- `max_steer` 를 실측 물리 한계 기반으로 재산정
- `gap_follower.py` 감속 로직에서 `max_steer` 의존 분리
