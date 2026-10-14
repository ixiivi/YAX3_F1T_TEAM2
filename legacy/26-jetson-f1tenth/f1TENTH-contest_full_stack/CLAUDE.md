# F1TENTH 자율주행 프로젝트 (시뮬레이터 스택)

## 프로젝트 개요

F1TENTH 시뮬레이터 기반 자율주행 프로젝트.  
Pure Pursuit / 하이브리드(APP·Stanley·LQR) 컨트롤러로 웨이포인트를 추종하며 충돌 없이 완주하는 것을 목표로 한다.

- **OS:** Ubuntu 22.04 / **ROS 2:** Humble / **Python:** 3.10
- **시뮬레이터:** f1tenth_gym_ros
- **저장소:** `lloes098/26-jetson-f1tenth`의 `main` 브랜치, 이 폴더는 그 안의 시뮬 번들
- **실차(젯슨) 스택은 별개**: `~/yax/src/race_stack` 등 — 실차 사용법은 repo 루트의 `JETSON_COMMANDS.md` 참조

---

## 워크스페이스 구조

```
~/yax/f1TENTH-contest_full_stack/        ← 이 번들의 루트 (구 ~/f1tenth/f1TENTH/)
├── contest.sh                           # 빌드·맵핑·트랙추출·주행 통합 스크립트 (실차용)
├── track_data/maps|waypoints/           # contest.sh 가 만드는 트랙 데이터
├── sim_ws/src/f1tenth_gym_ros/
│   ├── config/sim.yaml              # 맵 경로 · 스폰 위치 설정
│   ├── launch/gym_bridge_launch.py  # 시뮬레이터 launch
│   └── maps/
│       ├── {맵}.png / {맵}.yaml
│       ├── {맵}_waypoints.csv          # 웨이포인트 데이터
│       ├── {맵}_waypoints.jpg          # 웨이포인트 시각화 (generate_waypoints.py)
│       ├── {맵}_center.jpg             # 중심선 그래프 시각화 (generate_waypoints_center.py)
│       ├── {맵}_route{n}.csv           # 루트 경로 (generate_waypoints_center.py)
│       ├── {맵}_ga_route{n}.csv        # GA 최적화 결과 (ga_racing_line.py)
│       └── {맵}_ga.jpg                 # GA 시각화 (ga_racing_line.py)
│
└── f1tenth_ws/src/
    ├── contest_autonomy/                 # 실차 맵핑·주행 패키지 (contest.sh 가 사용)
    ├── f1tenth_system/ ackermann_mux/ vesc/  # 실차 드라이버 스택
    └── f1tenth_racing/f1tenth_racing/
        ├── racing_node.py                    # 주행 방식 선택 런처 (컨트롤러 고르기)
        ├── pure_pursuit_node.py              # ★ Pure Pursuit 주행 노드
        ├── hybrid_controller_node.py         # APP·Stanley·LQR 하이브리드 컨트롤러
        ├── lap_timer_node.py                 # 랩타임 측정 + CSV 기록
        ├── generate_waypoints.py             # 웨이포인트 생성 (레이싱 라인 최적화)
        ├── generate_waypoints_center.py      # 중심선 추출 + 분기점 그래프 시각화
        ├── ga_racing_line.py                 # GA 레이싱 라인 최적화
        ├── generate_controller_map.py        # 곡률 기준 APP/Stanley/LQR 구간 분류 시각화
        ├── make_waypoints.py                 # 웨이포인트 생성 통합 스크립트
        ├── make_yaml.py                      # 맵 yaml 자동 생성
        ├── setup_sim.py                      # 시뮬 설정 (스폰 위치 · 방향)
        └── optimize_pp_params.py             # Pure Pursuit 파라미터 자동 최적화
```

---

## 프로그램 파일 설명 (`f1tenth_racing/`)

**실행 방식 구분**
- ROS 노드: `ros2 run f1tenth_racing <노드명>` (빌드 필요)
- 독립 스크립트: `python3 <파일명>.py` (빌드 불필요)

---

### `racing_node.py` — 주행 방식 선택 런처 (ROS 노드)
| | |
|---|---|
| **실행** | `ros2 run f1tenth_racing racing_node` |
| **동작** | 컨트롤러(pure_pursuit / hybrid) 선택 실행, 선택 결과를 `~/.f1tenth_active_controller`에 기록 (lap_timer가 읽음) |

### `pure_pursuit_node.py` — ★ 메인 주행 노드 (ROS 노드)
| | |
|---|---|
| **입력** | `/ego_racecar/odom` (위치·속도), `{맵}_waypoints.csv` |
| **출력** | `/drive` (AckermannDriveStamped 조향·속도 명령) |
| **설정** | 파일 상단 `WAYPOINTS_MAP`, `SPEED_SCALE`, `LA_BASE` 등 파라미터 |

### `hybrid_controller_node.py` — 하이브리드 컨트롤러 (ROS 노드)
| | |
|---|---|
| **입력** | `/ego_racecar/odom`, 웨이포인트 CSV |
| **출력** | `/drive` — 곡률에 따라 APP(타이트 코너) / Stanley(완만한 커브) / LQR(직선) 전환 |

### `lap_timer_node.py` — 랩타임 측정 (ROS 노드)
| | |
|---|---|
| **입력** | `/ego_racecar/odom` (위치), `~/.f1tenth_active_controller` (활성 컨트롤러 이름) |
| **출력** | 터미널 랩타임 출력, `~/f1TENTH/lap_results.csv` |

### `generate_waypoints.py` — 레이싱 라인 생성 (독립 스크립트)
| | |
|---|---|
| **입력** | `{맵}.png`, `{맵}.yaml` (맵 선택 후 스폰 위치 클릭) |
| **출력** | `{맵}_waypoints.csv`, `{맵}_waypoints.jpg` |
| **알고리즘** | 최소 곡률 최적화 + 마찰원 속도 프로파일 |

### `generate_waypoints_center.py` — 중심선 추출 및 루트 분리 (독립 스크립트)
| | |
|---|---|
| **입력** | `{맵}.png`, `{맵}.yaml` (맵 선택 후 시작점·방향 클릭) |
| **출력** | `{맵}_route{n}.csv` (주행 가능 루트), `{맵}_center.jpg` (시각화) |
| **알고리즘** | 거리 변환 → 스켈레톤 → 분기점 그래프 → 단순 사이클 탐색 |

### `ga_racing_line.py` — 유전 알고리즘 레이싱 라인 최적화 (독립 스크립트)
| | |
|---|---|
| **입력** | `{맵}.png`, `{맵}.yaml`, `{맵}_route{n}.csv` (없으면 `{맵}_waypoints.csv`) |
| **출력** | `{맵}_ga_route{n}.csv`, `{맵}_ga.jpg` |
| **알고리즘** | 경로별 서브 집단 독립 진화, 횡방향 오프셋 최적화, 물리 기반 랩타임 추정 |

### `optimize_pp_params.py` — Pure Pursuit 파라미터 최적화 (독립 스크립트)
| | |
|---|---|
| **입력** | 시뮬레이터 실행 상태 (`/ego_racecar/odom` 토픽) |
| **출력** | `pure_pursuit_node.py` 파라미터 직접 수정 |

### `make_waypoints.py` — 웨이포인트 생성 통합 런처 (독립 스크립트)
| | |
|---|---|
| **입력** | `{맵}.png`, `{맵}.yaml` (맵 선택 후 클릭) |
| **출력** | `{맵}_waypoints.csv` (`generate_waypoints.py` 래퍼) |

### `make_yaml.py` — 맵 yaml 자동 생성 (독립 스크립트)
| | |
|---|---|
| **입력** | `{맵}.png` (maps 폴더 내 yaml 없는 맵 자동 탐색) |
| **출력** | `{맵}.yaml` |

### `setup_sim.py` — 시뮬레이터 설정 (독립 스크립트)
| | |
|---|---|
| **입력** | `{맵}.png` (맵 선택 후 스폰 위치·방향 클릭) |
| **출력** | `sim.yaml` (스폰 좌표 업데이트), `pure_pursuit_node.py` (`WAYPOINTS_MAP` 변수 업데이트) |

---

## 빌드 명령어

```bash
# f1tenth_racing 패키지만 빌드 (일반적인 경우)
cd ~/yax/f1TENTH-contest_full_stack/f1tenth_ws
colcon build --packages-select f1tenth_racing
source install/setup.bash

# sim_ws 빌드 (시뮬레이터 변경 시)
cd ~/yax/f1TENTH-contest_full_stack/sim_ws
colcon build
source install/setup.bash
```

---

## 실행 명령어

```bash
# 터미널 1 — 시뮬레이터
cd ~/yax/f1TENTH-contest_full_stack/sim_ws && source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# 터미널 2 — 드라이버 (ros2 run 사용, launch 아님 — input() 프롬프트 때문)
cd ~/yax/f1TENTH-contest_full_stack/f1tenth_ws && source install/setup.bash
ros2 run f1tenth_racing pure_pursuit_node
# 또는 컨트롤러 선택 런처:
ros2 run f1tenth_racing racing_node
```

---

## 새 맵 추가 순서

1. `maps/`에 `.png` 복사
2. `make_yaml.py` 실행 → `.yaml` 생성
3. `make_waypoints.py` 실행 → 스폰 클릭 → 안전 여유 입력 (0.4~0.8m)
4. `setup_sim.py` 실행 → 스폰 위치·방향 클릭
5. `colcon build --packages-select f1tenth_racing && source install/setup.bash`

---

## Pure Pursuit 파라미터 (`pure_pursuit_node.py` 상단)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `WHEELBASE` | 0.33 | 차량 축간거리 (m) |
| `SPEED_SCALE` | 1.1057 | 웨이포인트 속도 배율 |
| `LA_BASE` | 0.5931 | 전방주시거리 기본값 (m) |
| `LA_K` | 0.1234 | 속도 비례 전방주시거리 증가율 |
| `PREVIEW_N` | 79 | 전방 감속 예견 포인트 수 |
| `A_BRAKE` | 9.4675 | 최대 제동 가속도 (m/s²) |
| `STEER_ALPHA` | 0.40 | 조향 저역통과 필터 계수 |

파라미터 자동 최적화: `python3 optimize_pp_params.py`

---

## 코드 설계 주의사항

### 경로 처리
- `ros2 run`으로 실행되는 노드는 install 경로가 가변적이므로 `_find_repo_root()`로 루트 탐색
  (`sim_ws/`와 `f1tenth_ws/`를 모두 포함하는 디렉토리 = 이 번들 루트 `~/yax/f1TENTH-contest_full_stack/`)
- 직접 실행 스크립트는 `__file__` 기준 4단계 상위로 루트 확정
- 절대경로(`/home/sssamsssong/...`) 하드코딩 금지 — 팀원 환경 호환성 깨짐

```python
# pure_pursuit_node.py 패턴
def _find_repo_root():
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(12):
        if os.path.isdir(os.path.join(d, 'sim_ws')) and os.path.isdir(os.path.join(d, 'f1tenth_ws')):
            return d
        d = os.path.dirname(d)
    raise RuntimeError('repo root를 찾을 수 없습니다')
```

### 웨이포인트
- 웨이포인트 CSV는 `sim_ws/.../maps/`에 저장 — `f1tenth_ws/` 내부가 아님
- **파일명 규칙: `{맵이름}_{종류}.{확장자}`** — 맵이름이 항상 앞에 와서 맵별로 정렬됨
  - `{맵}_waypoints.csv` / `{맵}_waypoints.jpg` — 기본 웨이포인트
  - `{맵}_center.jpg` — 중심선 시각화
  - `{맵}_route{n}.csv` — 루트 경로 (n=1,2,...)
  - `{맵}_ga_route{n}.csv` / `{맵}_ga.jpg` — GA 최적화 결과
- `setup_sim.py`는 `pure_pursuit_node.py`의 `WAYPOINTS_MAP` 변수(맵 이름만) 교체 — 전체 경로 교체 아님
- 웨이포인트 순서가 차량 헤딩과 역방향이면 `_load_waypoints`에서 `pts[::-1]` 적용

### 주행 노드 교체 시 수정 대상
1. `setup.py` — `console_scripts` 엔트리포인트 등록
2. `sim_launch.py` — 실행할 노드 이름
3. `setup_sim.py` — `PP_NODE` 경로 변수

---

## 앞으로 추가할 기능

- [ ] 충돌 감지 후 자동 제동 · 후진 · 복구 (벽 충돌 시 역주행 방지)
- [ ] 실제 차량(Jetson) 환경 대응 — 경로 및 파라미터 검증
- [ ] Gap Follow / FTG 알고리즘 추가 검토
- [ ] 웨이포인트 생성 결과 자동 검증 (벽 관통 여부 수치 리포트)
- [ ] generate_waypoints_center.py: 추출된 루프를 순서대로 정렬 → 웨이포인트 CSV 저장
- [ ] 분기 맵 분리 재설계: 교차로 입구에 명시적 벽을 그리는 방식

---

## 프로젝트 마무리 루틴

"오늘 프로젝트를 마무리하자"라고 하면 다음 순서로 진행:

1. **대화록 저장** — `forme/대화록/대화록_MMDD.md`에 오늘 대화 내용 요약 저장  
   (`forme/`는 이 폴더의 `.gitignore`에 등록된 개인 폴더 — 커밋/푸시 대상 아님)
2. **문서 업데이트**
   - `README.md`: 새로 완성된 프로그램, 다른 사람이 알아야 할 사용법 추가
   - `개발현황.md`: 날짜 헤더 + 번호 목록으로 의도·작업 내용·시행착오 기록
3. **커밋 & 푸시** — 파일마다 어울리는 커밋 메시지로 커밋 후 `git push`  
   (push 대상: `lloes098/26-jetson-f1tenth`의 `main` 브랜치 — 구 `naykwak/f1TENTH`의 `hj_v2_f1tenth_ws` 아님)
