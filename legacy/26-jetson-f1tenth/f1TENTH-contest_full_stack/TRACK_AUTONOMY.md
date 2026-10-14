# 새 트랙 매핑부터 자율주행까지

모든 명령은 저장소 루트의 `contest.sh` 하나로 실행한다. 원본 ZIP에는 빌드 산출물이
포함되어 있었지만 다른 Jetson에서 재사용할 수 없으므로, 스크립트가 현재 장치에서
필요 패키지를 다시 빌드한다.

## 생성물의 고정 경로

트랙 이름이 `final_track`이면 다음 파일만 사용한다.

- 맵 YAML: `track_data/maps/final_track.yaml`
- 맵 이미지: `track_data/maps/final_track.pgm`
- 경로 CSV: `track_data/waypoints/final_track.csv`
- 검사용 이미지: `track_data/waypoints/final_track_preview.png`
- 최근 선택 트랙: `track_data/active_track`

경로 CSV 형식은 `x,y,speed`이며 좌표 단위는 m, 속도 단위는 m/s다.

## 1. 최초 준비

```bash
cd ~/f1TENTH-contest_full_stack
chmod +x contest.sh
./contest.sh build
```

필수 ROS 2 패키지는 `slam_toolbox`, `nav2_map_server`, `nav2_amcl`,
`nav2_lifecycle_manager`, `joy`, `joy_teleop`이다. Python에는 `numpy`, `scipy`,
`Pillow`, `scikit-image`, `PyYAML`이 필요하다.

## 2. 조이스틱으로 새 트랙 학습

```bash
./contest.sh learn final_track
```

LB를 누른 상태로 주행한다. 매핑 모드는 안전을 위해 조이스틱 최고속도를 1.0 m/s로
제한한다. 같은 출발점에서 2~3바퀴 돌고, 벽 양쪽을 LiDAR가
보도록 중앙 부근을 달린다. 한 바퀴만 급히 저장하면 SLAM loop closure가 불완전해
경로 생성기가 의도적으로 실패한다. 맵이 안정된 뒤 차를 완전히 세우고 터미널에서
Enter를 누르면, SLAM이 살아 있는 동안 맵을 저장하고 폐곡선·벽 여유를 검사하여
경로와 속도 프로파일을 자동 생성한다.

`*_preview.png`에서 빨간 경로가 벽을 가로지르지 않는지 반드시 확인한다. 자동 검사를
통과했다는 것은 점유격자 기준 안전하다는 뜻이지, 실제 차량의 조향·제동 오차까지
보증한다는 뜻은 아니다.

## 3. 자율주행

차량을 매핑 시작 위치와 같은 위치·방향에 둔다. 맵 좌표의 초기 위치가 원점이면:

```bash
./contest.sh race final_track 0.0 0.0 0.0
```

최근 학습 트랙과 원점 초기 자세를 그대로 쓰려면 `./contest.sh race`만 실행해도 된다.
초기 자세가 다르면 RViz의 좌표를 확인해 `X Y YAW`를 정확히 넣는다. AMCL 공분산이
설정 한계를 넘거나 전방 비상영역에 물체가 들어오면 명령 속도는 0이 된다.

## 현장 튜닝

`f1tenth_ws/src/contest_autonomy/config/race.yaml`에서 먼저 저속으로 검증한다.

- 벽 여유: `car_width`, `wall_margin`
- 속도: `max_speed`, `min_speed`
- 경로 추종: `lookahead_base`, `lookahead_gain`
- 장애물 대응: `obstacle_distance`, `emergency_distance`
- 미션 종료: `laps`, `time_limit_sec`

파라미터를 바꾼 뒤에도 `contest.sh race`가 패키지를 다시 빌드한다. 처음에는
`max_speed: 1.0`으로 빈 트랙 3바퀴, 다음은 정적 장애물 하나로 검증하고 단계적으로
속도를 올린다. 소프트웨어만으로 “모든 장애물에 절대 충돌하지 않음”을 보장할 수는
없으므로 하드웨어 킬스위치와 사람이 접근 가능한 안전 구역은 필수다.
