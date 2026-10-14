# yax — RoboRacer 2026 F1TENTH 저장소

---

## ⚠️ 두 대의 차가 이 저장소를 함께 씁니다 — 작업 전 반드시 읽을 것

### ★ 작업 브랜치 — `main` 이 아닙니다

```bash
git checkout car1      # 1호차 젯슨
git checkout car2      # 2호차 젯슨
```

**각 차는 자기 브랜치에서 작업하고 자기 브랜치로 푸시합니다.**
`main` 은 양쪽이 검증한 것을 합치는 통합 브랜치이고, **직접 커밋하지 않습니다.**

> ⚠️ 2026-08-10 에 "브랜치는 main 하나" 로 적었다가 2026-08-13 에 되돌렸습니다.
> 그날 문서를 안 고쳐 둔 탓에 **1호차 작업이 통째로 main 으로 올라갔고, 같은
> 버그(빔 1° 건너뜀·race 모드 미검출)를 car1 과 main 에서 각각 고치는 일이
> 실제로 일어났습니다.** 문서가 실제와 어긋나면 반드시 이 값을 치릅니다.

값은 여전히 파일로 분리합니다 (`.car` + `vehicle_car*.yaml`). 브랜치가 갈려도
**차량 캘리브레이션이 섞이는 사고는 구조적으로 막혀 있습니다.**

### 코드가 갈리는 것은 여전히 비용입니다

브랜치를 나누면 **같은 버그를 두 번** 고치게 됩니다. 실제로 두 번 겪었습니다:

```
2026-08-10  bench(box) 모드 무력화       main d4b65f3 / car2 f307810
2026-08-13  race 모드 장애물 미검출       car1 decb799 / main 4090eab
```

그러니 **알고리즘 수정은 되도록 빨리 main 으로 올려 상대 차에 흘려보내세요.**
브랜치는 검증 전 작업을 격리하는 용도지, 영구히 갈라두라는 뜻이 아닙니다.

### 첫 실행 전에 한 번 — 이 젯슨이 몇 호차인지 알려주기

```bash
echo car2 > .car        # 1호차면 car1
```

`.car` 는 `.gitignore` 라 커밋되지 않습니다. 기기마다 한 번만 하면 됩니다.

### 구조

```
config/vesc_base.yaml       배관(포트·스무더·리밋). 두 차 공용, git
config/vehicle_car1.yaml    1호차가 재서 나온 값,      git
config/vehicle_car2.yaml    2호차,                     git
.car                        "car2" 한 줄               .gitignore
        ↓ config/apply_vehicle.py  (race.sh 가 매 실행 호출)
config/vesc.yaml                생성물  .gitignore
config/.vehicle_overlay.yaml    race_script.yaml 위에 덮을 값  .gitignore
```

**`config/vesc.yaml` 은 git 에 없습니다.** 차량 캘리브레이션을 잘못 커밋할
방법 자체가 없어졌습니다.

### 무엇이 차량별인가

| 절 | 키 |
|---|---|
| `vesc` | `speed_to_erpm_gain`, 조향 `gain`/`offset`, `servo_min`/`max` |
| `geometry` | `wheelbase`, `mass`, `lf`, `lr` |
| `tire` | `mu`, `pac_b/c/d/e` |
| `tuning` | `l1_min`, `l1_k`, `steer_alpha`, `slip_gain`, `speed_lag`, `a_lon_*`, `grip_util` |
| `joystick` | `steer_invert`, `speed_invert`, `device`, `axis_*`, `btn_*` |
| `network` | `domain_id` (1호차 `57` / 2호차 `77`), `peers` (젯슨 IP) |

`race_script.yaml` 의 나머지(알고리즘·안전·회피·계측)는 **그대로 공용**입니다.
이게 핵심입니다 — 한쪽에서 고친 알고리즘이 자동으로 양쪽에 흐릅니다.

`max_steer` 는 **손으로 적지 않습니다.** `(servo_min − offset) / (−gain)` 으로
`apply_vehicle.py` 가 계산해 내려보냅니다. 따로 적으면 반드시 어긋납니다.

### 규칙

- **남의 차 파일(`vehicle_car1.yaml` ↔ `vehicle_car2.yaml`)은 건드리지 마세요.**
  각자 실측값입니다
- **자기 차 브랜치(`car1`/`car2`)에 푸시하세요.** `main` 에 직접 커밋하지 마세요
- 실차로 검증한 **알고리즘 수정은 되도록 빨리 main 에 합치세요** — 안 합치면
  상대 차가 같은 버그를 다시 고칩니다 (2026-08-13 에 실제로 그랬습니다)
- 기동 로그의 `[vehicle]` 줄을 매번 확인하세요. `⚠` 가 뜨면 이유가 있습니다
- 자세한 절차와 캘리브레이션 순서: `docs/car2_merge_guide.md`

### ⚠ 아직 안 나눈 것 — 트랙 설정

`map_name` / `waypoints` / `mission.start_*` / `localization.init_*` 는
`race_script.yaml` 에 있고 **두 차가 공유**합니다. 두 차가 다른 트랙을 돌면
부딪히니, 당장은 **커밋하지 말고 로컬 수정으로** 두세요.

---

## ⚠️ 네트워크를 바꿨으면 스택을 반드시 재시작할 것

이 젯슨은 두 와이파이를 오갑니다:

| 상황 | 붙는 곳 | 상태 |
|---|---|---|
| **주행 중** | 팀 공유기 `192.168.50.x` | 노트북 RViz 연동 가능 |
| **Claude 에게 질문할 때** | 인터넷 되는 공용 와이파이 | **주행은 멈춰 있음** |

Claude 와 대화하는 동안은 차가 공유기에 붙어 있지 않고 주행도 안 하는 상태입니다.
그러니 `[dds] ⚠ CycloneDDS 폴백` 경고나 `network.subnet 로 시작하는 내 IP 가 없습니다`
는 **이때는 정상**입니다 — 설정 오류로 보고할 필요가 없습니다.

### ★ 그런데 스택을 켠 뒤에 네트워크를 바꾸면 조용히 망가집니다

`dds_env.sh` 는 **기동 시점의 IP 로 CycloneDDS 인터페이스를 고정**합니다
(`/tmp/cyclonedds-<내IP>.xml` 의 `<NetworkInterface address=...>`).
그 뒤 와이파이를 바꾸면 그 인터페이스가 사라지는데, **CycloneDDS 는 재바인딩하지
못합니다.** 노드는 살아 있는데 서로 못 듣는 상태가 됩니다:

```
launch.log 가 이걸로 도배된다:
  ddsi_udp_conn_write to udp/192.168.50.77:7480 failed with retcode -1
```

2026-08-10 실제로 겪었습니다 — 공유기에서 `./race.sh joy` 를 띄운 뒤 공용
와이파이로 옮기자 **조향이 갑자기 먹통**이 됐습니다. `joystick_teleop` 의 명령이
`ackermann_to_vesc` 에 도달하지 못한 것이고, 서보나 VESC 고장처럼 보여 디버깅이 샙니다.

**규칙: 와이파이를 바꿨으면 `./race.sh stop` 후 다시 띄울 것.**

---

## 두 차를 같은 공유기에서 동시에 돌릴 때

차마다 **`ROS_DOMAIN_ID` 가 다릅니다** (1호차 `57` / 2호차 `77` — 젯슨 IP 끝자리와 같게).
도메인이 다르면 멀티캐스트 그룹과 포트 대역 자체가 달라 `/scan`·`/map`·`/tf` 가
절대 안 섞이고 노드 이름 충돌도 없습니다. 원본은
`config/vehicle_car*.yaml` 의 `network.domain_id` 이고, 젯슨에서는 `race.sh` 가
`.car` 를 보고 자동으로 export 합니다.

**노트북에서는 붙을 차를 골라야 합니다:**

```bash
./rviz.sh 77      # 2호차 (젯슨 IP 끝자리 — 권장)
./rviz.sh 57      # 1호차
./rviz.sh car2    # 차 이름도 됨
```

노트북 두 대가 각각 다른 차에 붙어도 됩니다.

> ⚠️ **2026-08-10 이전에는 두 차 모두 기본값 `0` 이었습니다.** 젯슨과 노트북이
> **양쪽 다** `git pull` 해야 서로 보입니다 — 한쪽만 갱신하면 통신이 0 이 됩니다.
> `~/.bashrc` 에 `ROS_DOMAIN_ID` 를 박아뒀다면 지우세요. 이제 스크립트가 정합니다.

---

## 저장소 정보

- **원격:** `git@github.com:lloes098/26-jetson-f1tenth.git`
- **푸시 브랜치:** 자기 차 브랜치 — 1호차 `car1` / 2호차 `car2`.
  `main` 은 검증된 것을 합치는 통합 브랜치입니다 (직접 커밋 금지).
  (구 저장소 `naykwak/f1TENTH`의 `hj_v2_f1tenth_ws`는 더 이상 사용 안 함)
- **젯슨 내 경로:** `~/yax` — 이 git repo가 **유일한 소스**

## 두 개의 스택

| 스택 | 위치 | 용도 | 문서 |
|------|------|------|------|
| 실차(젯슨) race_stack | `src/race_stack` | 실제 차량 자율주행 (대회 본선) | `JETSON_COMMANDS.md`, `README.md` |
| 시뮬 번들 | `f1TENTH-contest_full_stack/` | f1tenth_gym_ros 시뮬 + 컨트롤러 개발 | 해당 폴더의 `CLAUDE.md`, `README.md` |

## 심링크 구조 (중요)

실행 워크스페이스 `~/f1tenth_ws/src/race_stack`은
이 repo의 `src/race_stack`으로 향하는 **심링크**다 (복사 아님, `install_on_jetson.sh`가 생성).

- repo에서 파이썬 코드 수정 → 재빌드 없이 즉시 반영 (`--symlink-install`)
- `~/f1tenth_ws` 쪽에서 수정해도 그게 곧 repo → 커밋/푸시로 보존
- 재빌드가 필요한 경우: 새 `.py`/launch 파일 추가, `setup.py`/`package.xml` 변경
  → `cd ~/f1tenth_ws && colcon build --packages-select race_stack --symlink-install`
- **맵/웨이포인트 신규 추가는 재빌드 불필요** (2026-07-17~): `race.sh`가 `RACE_WS`를
  export하고, `gen_waypoints`·`race.launch.py`가 이 값 기준으로 repo `src/race_stack`의
  `maps/`·`waypoints/`를 직접 읽는다. 예전엔 build/install 스냅샷을 읽어 새 맵이 안 보였음.
  (그래도 새 맵 파일은 커밋/푸시로 repo에 보존할 것)
- `race_script.yaml`은 repo(`~/yax`)에만 존재 — `race.sh`가 자기 디렉토리에서 읽음
- `config/vesc.yaml`(조향/모터 캘리브레이션)도 repo가 원본 — `~/f1tenth_ws`의
  f1tenth_stack src·install 양쪽이 여기로 심링크됨. f1tenth_stack을 colcon build
  하면 install 링크가 복사본으로 덮이니 `install_on_jetson.sh` 재실행으로 재링크

## 주요 스크립트

- `race.sh` — 실차 딸깍 실행 (mapping/q1/q2/q3/race/savemap/log/waypoints/joy/teleop)
- `jetson.sh` — 노트북에서 SSH+tmux로 젯슨의 race.sh 원격 실행
- `install_on_jetson.sh` — 최초 1회 설치 (심링크 + 빌드 + udev)
- `race_script.yaml` — ★ 팀원이 수정하는 유일한 설정 파일

## 환경 주의사항

- **numpy는 1.26.4 고정** — 2.x로 올리면 cv2/matplotlib/colcon이 깨짐 (`~/.local`에 pyparsing도 설치 금지)
- ROS 2 Humble / Ubuntu 22.04 (Jetson, aarch64)
- 조이스틱 트리거: `trigger_invert`는 `/joy` 토픽 기준 — RT 중립값 부호가 js0 원시값과 반대
