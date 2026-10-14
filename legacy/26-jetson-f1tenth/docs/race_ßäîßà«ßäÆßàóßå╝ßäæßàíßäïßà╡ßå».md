# race.sh 주행 파일 & 주행 방식 정리

> 정리일: 2026-07-17 / 기준 설정: `race_script.yaml` (mode=q2, controller=stanley)

## 1. 실행 체인 (어떤 파일이 실제로 도는가)

`./race.sh` 를 인자 없이 실행하면 `race_script.yaml` 의 `mode` 값대로 돈다.
현재 설정은 **mode: q2 / controller: stanley** 이므로 실행 경로는 다음과 같다.

| 단계 | 파일 (경로) | 역할 |
|------|------------|------|
| 1. 딸깍 스크립트 | [race.sh](../race.sh) | 환경 소싱·VESC 심링크·모드 결정 후 launch 호출 |
| 2. 설정 파일 | [race_script.yaml](../race_script.yaml) | 팀원이 수정하는 유일한 설정 (mode/controller/맵/속도/게인) |
| 3. Launch | [src/race_stack/launch/race.launch.py](../src/race_stack/launch/race.launch.py) | 브링업+맵서버+AMCL+미션매니저+안전+컨트롤러 기동 |
| 4. **실제 주행 노드** | **[src/race_stack/race_stack/stanley_controller.py](../src/race_stack/race_stack/stanley_controller.py)** | **차를 실제로 조향/가감속시키는 코드** |

- `race.sh` 는 최종적으로 `ros2 launch race_stack race.launch.py mode:=q2 controller:=stanley` 를 실행한다 ([race.sh:187](../race.sh#L187)).
- `controller: auto` 였다면 q2 는 기본값이 `gap_follower` 였겠지만, 현재 yaml 이 `stanley` 로 강제하고 있어 **Stanley 컨트롤러**가 돈다 ([race_script.yaml:34](../race_script.yaml#L34)).
- 컨트롤러 노드는 AMCL 수렴 시간 확보를 위해 **3초 지연** 후 기동된다 ([race.launch.py:187](../src/race_stack/launch/race.launch.py#L187)).

### 함께 뜨는 노드 (race.launch.py)
- `f1tenth_stack` 브링업 — LiDAR + VESC + odom + mux
- `map_server` + `amcl` + `lifecycle_manager` — 저장된 맵 위에서 위치 추정
- `mission_manager` — 랩 카운트 / 자동 정지 (현재 laps=3)
- `safety_node` (AEB) — **현재 `safety.enabled: False` 라 안 뜸**

## 2. 어떤 웨이포인트를 따라가나

- **웨이포인트 파일:** [src/race_stack/waypoints/track_2nd_clean_wp.csv](../src/race_stack/waypoints/track_2nd_clean_wp.csv)
  - `race_script.yaml` 의 `waypoints: track_2nd_clean_wp` 로 지정됨.
  - 포맷: `x, y, speed` (m, m, m/s), 헤더 1줄 + **400개 점**.
  - 각 점마다 목표 속도가 박혀 있음 (예: `2.4520, 3.7801, 2.6821`) → 레이싱라인 + 구간별 속도.
- **맵:** [src/race_stack/maps/track_2nd_clean.yaml](../src/race_stack/maps/track_2nd_clean.yaml) (+ `.png`/`.pgm`)
- **Q2 경계 파일 (장애물 판정용):** [src/race_stack/maps/track_2nd_clean/](../src/race_stack/maps/track_2nd_clean/)
  - `inner_bound.npy`, `outer_bound.npy` (픽셀 좌표) — 스캔 점이 트랙 안(경계 사이)에 있을 때만 장애물로 인정.

## 3. 어떻게 주행하나 (Stanley + Q2 로직)

실제 제어는 [stanley_controller.py](../src/race_stack/race_stack/stanley_controller.py) `_compute_cmd()` 에서 **50Hz 고정 주기**로 이뤄진다.

**위치 추정 (pose 융합)**
- `/amcl_pose` (느리고 정확) + `/odom` (빠르지만 드리프트) 를 융합해 매 20ms "지금" 위치를 추정 ([stanley_controller.py:251](../src/race_stack/race_stack/stanley_controller.py#L251)).
- 정지 시에도 0속도를 계속 발행 → mux timeout(0.2s) 데드락 방지 + odom kick.

**조향 (Stanley)**
1. **Adaptive lookahead** — 속도에 비례해 전방주시거리 L 을 `lookahead_min(0.5)`~`lookahead_max(2.0)` 사이에서 조절.
2. 최근접 웨이포인트에서 L 만큼 전방의 목표점을 보간으로 계산.
3. **경로 접선(tangent)** 기준으로 횡오차(CTE)·헤딩오차 계산 → Stanley 조향식. (현(chord) 대신 중심차분 접선을 써서 코너 안쪽 파고듦 제거.)
4. **곡률 피드포워드(ff_gain=1.0)** — 로드 시 미리 계산한 경로 곡률로 코너의 "정답 조향각"을 선행 적용, `ff_preview_sec`(0.1s)로 서보 지연 보상.
5. 조향각을 `max_steer`(0.36 rad)로 클립, `steer_lpf`(현재 1.0=끔)로 필터.

**속도**
- 웨이포인트 CSV 의 speed × `vel_scale`(1.0) 을 목표속도로, `min_speed`(0.5)~`q2_max`(2.0) 로 클립.

**Q2 장애물 회피** ([stanley_controller.py:405](../src/race_stack/race_stack/stanley_controller.py#L405))
- `/scan` 을 차량 좌표계 격자로 만들어 장애물 점유 격자를 잡고, inner/outer 경계 사이(트랙 위)에 있는 점만 장애물로 인정.
- 웨이포인트~장애물 최소거리가 `obs2lane_dist`(0.25m) 이하이면 레인 차단으로 판정 (최근 10표 중 5표 기준 투표).
- 차단 시: 목표점을 장애물 반대쪽으로 **0.2m 옆으로 시프트**해서 우회하고, 속도를 `obstacle_decel`(0.85=15% 감속).

**정지**
- `mission_manager` 가 `/race/enabled`(Bool) 로 게이트. laps=3 완주하면 자동 정지.

## 4. 요약 한 줄

> `./race.sh` → **q2 모드, Stanley 컨트롤러**([stanley_controller.py](../src/race_stack/race_stack/stanley_controller.py))가
> **track_2nd_clean_wp.csv**(400점 레이싱라인)를 50Hz Stanley+곡률FF로 추종하며,
> LiDAR 로 트랙 위 장애물을 만나면 목표점을 0.2m 옆으로 밀어 우회·감속하고 3바퀴 후 정지한다.

## 5. 설정을 바꾸려면
- 모드/컨트롤러/맵/웨이포인트/속도/게인 → 전부 [race_script.yaml](../race_script.yaml) 한 파일에서.
- 컨트롤러 임시 변경 → `./race.sh q2 pure_pursuit` 처럼 두 번째 인자로 오버라이드 (yaml 안 건드림).
