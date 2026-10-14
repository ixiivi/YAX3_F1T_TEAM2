# 2호차 — main 병합 안내 (차량별 설정 분리)

**작성 2026-08-10 (1호차 젯슨). 대상: 2호차 담당자.**

`car2` 브랜치에 main 커밋 **23건**이 안 들어가 있습니다. 그중 3건이
차량별 설정 분리이고, 나머지는 어제 실차에서 잡은 것들입니다.

---

## 1. 왜 바꿨나

`config/vesc.yaml` 하나에 두 차의 값이 들어갈 수 없습니다. 2호차는 모터 결선이
반대라 `speed_to_erpm_gain` 이 음수인데, 그 값이 1호차로 넘어가면 **1호차가
그대로 후진**합니다.

브랜치로 나누는 방법도 있지만 그러면 **코드까지 갈라져 같은 버그를 두 번**
고치게 됩니다. 실제로 그랬습니다:

```
main  d4b65f3  fix(safety): bench(box) 모드가 무력화돼 있었다
car2  f307810  fix(safety): bench(box) 모드가 무력화돼 있었다   ← 각자 디버깅
```

그래서 **코드는 합치고 값만 분리**합니다.

---

## 2. 새 구조

```
config/vesc_base.yaml       배관(포트·스무더·리밋). 두 차 공용, git
config/vehicle_car1.yaml    1호차가 재서 나온 값,      git
config/vehicle_car2.yaml    2호차,                     git
.car                        "car2" 한 줄               .gitignore
        ↓ config/apply_vehicle.py  (race.sh 가 매 실행 호출)
config/vesc.yaml                생성물  .gitignore
config/.vehicle_overlay.yaml    race_script.yaml 위에 덮을 값  .gitignore
```

**`config/vesc.yaml` 이 git 에서 빠졌습니다.** 이제 차량 캘리브레이션을 잘못
커밋할 방법 자체가 없습니다.

### 무엇이 차량별인가

| 절 | 키 |
|---|---|
| `vesc` | `speed_to_erpm_gain`, 조향 `gain`/`offset`, `servo_min`/`max` |
| `geometry` | `wheelbase`, `mass`, `lf`, `lr` |
| `tire` | `mu`, `pac_b/c/d/e` |
| `tuning` | `l1_min`, `l1_k`, `steer_alpha`, `slip_gain`, `speed_lag`, `a_lon_*`, `grip_util` |
| `joystick` | `steer_invert`, `speed_invert`, `device`, `axis_*`, `btn_*` |

`race_script.yaml` 의 나머지 ~130개(알고리즘·안전·회피·계측)는 **그대로 공용**
입니다. 이게 핵심입니다 — 한쪽에서 고친 알고리즘이 자동으로 양쪽에 흐릅니다.

### ★ `max_steer` 는 손으로 적지 않습니다

```
δ_좌 = (servo_min − offset) / (−gain)     →  max_steer, R_min
```

2026-08-09 에 `steer_trim` 이 offset 을 0.425 → 0.3954 로 옮겼는데 `max_steer`
0.36 은 그대로여서 **좌 조향 지령의 25.8% 가 vesc_driver 에서 조용히 잘려
나갔습니다.** 이제 `apply_vehicle.py` 가 계산해 내려보냅니다.
`gen_raceline` 의 `R_CAR_MIN` 도 같은 값에서 유도합니다.

---

## 3. 병합 절차

```bash
git fetch origin
git checkout car2
git merge origin/main
```

충돌 **10개 파일**. 파일마다 해결이 정해져 있습니다.

| 파일 | 해결 | 이유 |
|---|---|---|
| `config/vesc.yaml` | **삭제 수락** | 이제 생성물. 값은 `vehicle_car2.yaml` 에 있음 |
| `race_script.yaml` | main 것 | 차량값은 `vehicle_car2.yaml` 이 덮는다 |
| `gap_follower.py` `pure_pursuit.py` `stanley_controller.py` `map_controller.py` | main 것 | bench 수정 중복. 내용 동일 |
| `race.launch.py` | main 것 | `--bag`·차량 overlay·bench·AMCL 슬루 제한 포함 |
| `gen_raceline.py` | main 것 | 재선형화 반복으로 재작성됨 |
| `install_on_jetson.sh` | main 것 | vesc.yaml 생성 단계 추가됨 |
| `race.sh` | **손으로 합칠 것** | main 의 `--bag`·차량생성 + car2 의 `waypoints-tumftm`·조이 정리 |

```bash
git rm config/vesc.yaml
git checkout --theirs race_script.yaml src/race_stack/launch/race.launch.py \
  src/race_stack/race_stack/gap_follower.py \
  src/race_stack/race_stack/pure_pursuit.py \
  src/race_stack/race_stack/stanley_controller.py \
  src/race_stack/race_stack/map_controller.py \
  src/race_stack/race_stack/gen_raceline.py \
  install_on_jetson.sh
# race.sh 는 편집기로 열어 양쪽 다 살릴 것
git add -A && git commit

echo car2 > .car
```

---

## 4. 첫 실행 — 순서를 지킬 것

### ① `./race.sh box` — 세 줄을 눈으로 확인

```
[vehicle] car2  (config/vehicle_car2.yaml)     ← car1 이면 즉시 중단
[vehicle]   vesc: erpm_gain -4203.0            ← 부호가 음수인지
★ BENCH 모드 — 속도를 0 으로 강제한다           ← 없으면 즉시 Ctrl+C
```

세 번째 줄이 없으면 **바퀴가 돕니다.** 2026-08-09 에 `box` 가 시작 버튼도 없이
풀속도로 주행한 사고가 있었습니다.

### ② 저속 전진 방향 확인

`box` 는 바퀴가 안 돌아 **방향을 못 봅니다.** 조이스틱으로 아주 천천히
전진시켜 **앞으로 가는지** 확인하세요.

뒤로 가면 `speed_to_erpm_gain` 부호가 아직 틀린 것입니다. 그 상태로 맵핑하면
**SLAM 맵이 방사형으로 깨집니다** (`vesc_to_odom` 이 같은 게인을 쓰기 때문).

### ③ 캘리브레이션 — 순서가 중요합니다

기동 로그가 `⚠ 다른 차에서 빌려온 값 5개` 로 세어줍니다. 재고 나면
`vehicle_car2.yaml` 의 `(1호차 값)` 표시를 지우세요.

| 순서 | 항목 | 명령 |
|---|---|---|
| ① | 모터 방향 | ✅ 완료 (부호 −) |
| ② | **오도메트리 스케일** | `ros2 run race_stack odom_scale` (줄자, 6회) |
| ③ | 조향 중립·게인 | `./race.sh steertrim` |
| ④ | 기구 스토퍼 | `vesc_steer_test.yaml` + `steer_limit_check` |
| ⑤ | 무게·하중분포 | 저울 |

**②가 틀리면 그 위의 전부가 틀립니다** — 속도 프로파일과 μ 추정이 v² 에
비례합니다. 1호차는 8.9% 틀려 있었습니다.

### ④ 라인 재생성

`R_min` 은 ④의 결과에서 나옵니다. 조향 한계가 확정된 뒤에 만드세요.

```bash
cd src/race_stack
python3 -m race_stack.gen_raceline --map <맵> --mu 0.4 --rmin 1.0
```

출력 마지막에 `차량 최소회전반경 ... 여유 +0.xxx ✓` 가 떠야 합니다.
`★★ 못 도는 점 N개` 가 뜨면 그 라인은 못 씁니다.

---

## 5. 같이 들어오는 것들 (main 23건 중)

2호차가 속도를 올리면 **똑같이 겪을** 것들입니다. 1호차가 어제 실차 로그로
원인까지 확인했습니다.

| 수정 | 증상 | 원인 |
|---|---|---|
| `steer_alpha` 0.555 | 고속 직선 요 진동 | 이 값이 **루프 주기에 걸려 있다.** 실차 50Hz 에서 170ms, 시뮬 250Hz 에서 34ms — 같은 숫자가 5배 무겁다 |
| `l1_k` 0.75 | 〃 | α_f 가 `a_lat ∝ v` 라 L1 의 속도무관 설계를 깬다 |
| `slip_gain` 0.5 | 코너에서 안쪽으로 파고듦 | 슬립보정이 기구학항만큼 커짐 (주행 곡률이 기준의 1.2~1.9배) |
| `speed_lag` 0.40 | 코너 진입 과속 | 구동계 지연 실측 406~440ms |
| AMCL 슬루 제한 | 주행 중 위치 점프 | 보정을 즉시 전부 반영하고 있었다 (4.5m 점프 → 즉시 풀조향) |
| `auto_localize` 양보 | 2D Pose Estimate 가 덮임 | 기동 2초 뒤 자동정렬이 사람 입력을 덮어썼다 |
| `--bag` | — | 주행 중 원시 토픽 녹화. 어제 진단 대부분이 이걸로 풀렸다 |
| `gen_raceline` 재작성 | R_min 제약이 안 먹음 | 최적화한 곡선과 출력 곡선이 달랐다 |

### μ 는 확정됐습니다

η 스윕(0.5 → 0.7 → 1.0)에서 **η=1.0 으로 2.95랩 완주**했습니다.
`μ_eff ≥ 0.400` 은 이제 추정이 아니라 확정된 하한입니다. 2호차도 같은 방법으로
확인하세요 — **η 스윕이 곧 μ 측정**입니다.

---

## 6. 아직 안 나눈 것 — 트랙 설정

`map_name` / `waypoints` / `mission.start_*` / `localization.init_*` 는
`race_script.yaml` 에 있고 **두 차가 공유**합니다.

두 차가 다른 트랙을 돌면 부딪힙니다. 당장은 **커밋하지 말고 로컬 수정으로**
두세요. 계속 문제가 되면 `.car` 처럼 트랙도 분리하겠습니다.

---

## 7. 규칙

- **`config/vehicle_car1.yaml` 은 건드리지 마세요.** 1호차 실측값입니다
- 차량값이 아닌 **코드·알고리즘 수정은 main 에 올려주세요** — 그래야 양쪽에 흐릅니다
- 기동 로그의 `[vehicle]` 줄을 매번 확인하세요. `⚠` 가 뜨면 이유가 있습니다
- 막히면 충돌 상태 그대로 물어보세요
