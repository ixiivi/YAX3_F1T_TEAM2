# RoboRacer 2026 — race_stack (딸깍 주행 뼈대)

**팀원은 `race_script.yaml` 하나만 수정하면 됩니다. 코드/빌드 건드릴 필요 없음.**

> 📌 **지금 어디까지 왔나** → [docs/STATUS.md](docs/STATUS.md)
> (확정된 값, 아직 열려 있는 문제, 다음 작업)
> **무엇이 바뀌었나** → [docs/UPDATES.md](docs/UPDATES.md) (날짜별)

```
race_stack_skeleton/
├── race.sh              ← 딸깍 스크립트 (이거 하나 실행)
├── race_script.yaml     ← ★ 팀원이 수정하는 유일한 파일 ★
├── install_on_jetson.sh ← 최초 1회 설치
├── systemd/             ← 부팅 자동실행(FA용) + VESC udev rule
└── src/race_stack/      ← ROS2 패키지 (수정 금지 구역)
    ├── launch/  mapping.launch.py, race.launch.py
    ├── config/  amcl.yaml
    ├── maps/        ← 맵 .yaml + .png/.pgm 넣는 곳
    ├── waypoints/   ← 레이싱라인 .csv 넣는 곳
    └── race_stack/  ← 노드들 (gap_follower, pure_pursuit, ...)
```

## 아키텍처 (기존 f1tenth_stack 위에 얹음)

```
[f1tenth_stack bringup]  LiDAR(/scan) + VESC + wheel odom(/odom) + ackermann_mux
        │
[map_server + AMCL]      저장된 맵 위에서 위치 추정 → /amcl_pose, map→odom tf
        │
[mission_manager]        출발선 통과로 랩 카운트, 목표 랩/제한시간 도달 시
        │                /race/enabled=false → 자동 정지 (FA 대응)
[controller]             q1/q3: pure_pursuit(레이싱라인) | q2/race: gap_follower(회피)
        │                → /drive (mux priority 10)
[safety_node]            TTC < 임계값이면 /teleop(priority 100)으로 급정지 (AEB)
```

mux 우선순위: `/teleop`(100) > `/drive`(10). 그래서 키보드 텔레옵과 AEB는
항상 자율주행 명령을 덮어씁니다.

## 최초 설치 (젯슨에서 1회)

```bash
# 이 폴더를 젯슨 홈에 복사한 뒤
cd ~/race_stack_skeleton
chmod +x race.sh install_on_jetson.sh
./install_on_jetson.sh
```

## 사용 순서 (대회 Day 1 워크플로우)

### 1. 맵 만들기 (주최측 맵 받으면 이 단계 스킵)
```bash
./race.sh mapping
# → 키보드(w/a/s/d)로 트랙 1~2바퀴 천천히 주행
# → 다른 노트북에서 Foxglove(ws://젯슨IP:8765)로 맵 확인 가능*
# → Ctrl+C 후:
./race.sh savemap bexco_track
```
*Foxglove 쓰려면 별도 터미널에서 `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`

**주최측이 .png/.yaml 배포하면** 그냥 `src/race_stack/maps/`에 넣고
`cd ~/f1tenth_ws && colcon build --packages-select race_stack` 한 번.

### 2. race_script.yaml 설정
- `map_name`: 방금 저장한 맵 이름
- `mission.start_x/start_y`: RViz/Foxglove에서 출발선 좌표 확인해서 입력
- `mode`: q1 / q2 / q3 / race

### 3. (q1/q3용) 레이싱라인 기록
```bash
# 터미널 1: 로컬라이제이션 켜진 상태로 스택 실행 (gap_follower여도 됨)
./race.sh q2
# 터미널 2: 키보드로 좋은 라인 주행하며 기록
./race.sh teleop         # 키보드가 자율주행을 덮어씀
# 터미널 3:
./race.sh log bexco_wp
# 한 바퀴 돌고 Ctrl+C → race_script.yaml의 waypoints를 bexco_wp로
```

### 4. 딸깍
```bash
./race.sh          # race_script.yaml의 mode대로
./race.sh q1       # 모드만 임시 변경
```

## Fully Autonomous (부팅 → 자동 주행)
```bash
sudo cp systemd/roboracer.service /etc/systemd/system/
sudo nano /etc/systemd/system/roboracer.service   # User=, 경로 수정
sudo systemctl daemon-reload && sudo systemctl enable roboracer
```
전원만 켜면 race_script.yaml 설정대로 스택이 뜹니다.
연습 때는 `sudo systemctl disable roboracer` 해두세요.

## 대회 규정 관련 메모
- 모든 연산 온보드(Jetson): 이 스택은 외부 통신 없이 동작 (Foxglove/SSH는 디버깅 전용, **주행 중 금지**)
- Q3는 time_limit이 자동으로 120초 설정됨
- 하드웨어 비상 킬스위치(토글식)는 SW와 별개로 반드시 장착 (검사 항목)
- AEB(safety_node)는 온보드 자율 로직이므로 규정 위반 아님. 단 예선 중
  오작동으로 멈추면 손해이므로 `safety.ttc_threshold`를 실전에서는 낮추거나
  `enabled: false` 고려

## 튜닝 가이드 (전부 race_script.yaml에서)

값은 `./tune.sh` 로 바꾸면 오타 가드와 변경 이력이 붙는다 — 사용법은 `docs/tune_사용법.md`.
`./tune.sh status` 로 현재 상태·조절 우선순위를 본다. (Stanley 게인 설명: `docs/stanley_gain_tuning.md`)

```bash
./tune.sh status                  # 지금 상태
./tune.sh stanley.ff_gain+=0.3    # 조정 → 다음 ./race.sh 부터 적용 (재빌드 불필요)
```

| 증상 | 조치 |
|---|---|
| 코너에서 벽에 붙음 | `gap_follower.car_width` ↑, `steer_gain` ↑ |
| 지그재그 진동 | `pure_pursuit.lookahead` ↑ |
| 코너 이탈(언더스티어) | `curvature_slowdown` ↑, `qX_max` ↓ |
| 장애물에 너무 늦게 반응 | `disparity_threshold` ↓, `max_range` ↑ |
| 출발선 랩 카운트 안 됨 | `mission.start_radius` ↑, start_x/y 재확인 |
| AMCL 위치 튐 | 초기포즈(init_x/y/yaw) 정확히, 출발 전 제자리에서 살짝 전후진 |

## 트러블슈팅
- `/dev/sensors/vesc 없음` → `ls -l /dev/ttyACM*` 확인, udev rule의 idVendor/idProduct를 `lsusb` 값으로 수정
- 맵은 뜨는데 차가 안 움직임 → `ros2 topic echo /amcl_pose` 나오는지, `/race/enabled`가 true인지 확인
- 웨이포인트 없다고 에러 → q1/q3는 waypoints CSV 필수. 없으면 `controller: gap_follower`로 임시 주행 가능
