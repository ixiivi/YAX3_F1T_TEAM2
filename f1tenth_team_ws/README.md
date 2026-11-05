# F1TENTH 팀 워크스페이스

이 차로 처음부터 시작하기 위한 폴더다. 드라이버와 차량 설정만 들어 있고, 자율주행 코드는 없다. 맵 만들기, 위치 추정, 경로 추종은 여기서 직접 만든다.

## 들어 있는 것

| 경로 | 내용 |
|---|---|
| `src/f1tenth_system` | 공식 드라이버 스택(humble-devel)에 아래 패치를 적용한 소스. 서브모듈 없이 코드가 직접 들어 있다 |
| `patches/` | 이 차에 맞춘 수정 사항. 이미 적용돼 있다 |
| `scripts/setup_sudo.sh` | 새 Jetson용 초기 설정. **지금 차에서는 실행할 필요 없다** |
| `maps/` | 저장한 맵을 두는 곳 (비어 있음) |

## 차량 정보

- **컴퓨터:** Jetson Orin Nano Super, JetPack R36.4.7, ROS 2 Humble
- **라이다:** Hokuyo UST-10LX, 이더넷 연결. Jetson 192.168.0.15, 라이다 192.168.0.10
- **모터 컨트롤러:** VESC 60_MK6 FW 7.0, 센서리스 모터, `/dev/sensors/vesc`

`base_link`는 뒤 차축 중심이다.

| 실측값 (2026-11-03) | 값 |
|---|---|
| 휠베이스 | 0.33 m |
| 앞 범퍼 / 뒤 범퍼 (base_link 기준) | 앞 0.42 m / 뒤 0.08 m |
| 차폭 | 0.42 m |
| 라이다 위치 | 앞 0.267 m, 위 0.13 m |

**아직 캘리브레이션하지 않은 값** (`f1tenth_stack/config/vesc.yaml`):
- `speed_to_erpm_gain` 4614: 이전 팀의 차에서는 4203으로 측정됐다.
- `steering_angle_to_servo_gain` -1.2135: 이전 팀의 차는 이 기본값에서 1.6배 과조향했다.

이 두 값이 틀리면 odom이 틀어지므로, 위치 추정을 시작하기 전에 맞추는 것을 권장한다.

## 시작하기

### 1. 받기와 빌드

```bash
git clone git@github.com:ixiivi/YAX3_F1T_TEAM2.git ~/YAX3_F1T_TEAM2   # 이미 있으면 git pull
cd ~/YAX3_F1T_TEAM2/f1tenth_team_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

새 터미널을 열 때마다 마지막 줄의 `source`를 다시 실행한다. 경로는 `~/YAX3_F1T_TEAM2/f1tenth_team_ws/install/setup.bash`다.

### 2. 연결 확인

```bash
ls -l /dev/sensors/vesc
ping -c 2 192.168.0.10
```

### 3. 드라이버 실행

**다른 사람이 차를 쓰고 있지 않은지 먼저 확인한다.** VESC와 라이다는 한 번에 드라이버 하나만 쓸 수 있다.

```bash
ros2 node list      # 비어 있어야 한다
ros2 launch f1tenth_stack bringup_launch.py
```

다른 터미널에서 센서를 확인한다.

```bash
ros2 topic hz /scan
ros2 topic echo /sensors/core --field state.voltage_input
ros2 topic echo /odom --field pose.pose.position
```

### 4. 움직여 보기

**처음에는 차를 받침대에 올려 바퀴를 띄운다.**

전진 (0.6 m/s, Ctrl+C로 명령을 멈춘다):
```bash
ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.6, steering_angle: 0.0}}"
```

정지 (브레이크):
```bash
ros2 topic pub --once /drive ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0, steering_angle: 0.0}}"
```

전진 중에 `/odom`의 x 값이 늘어나면 정상이다.

**조이패드(Logitech F710)를 쓰는 경우:** LB(버튼 4)를 누르고 있는 동안만 조종된다. 스틱 끝까지 밀면 5 m/s이므로 조금씩 민다.

### 5. 맵 만들기

드라이버를 켠 상태에서 실행하고, 차를 천천히 몰아 트랙을 한 바퀴 돈다.

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(ros2 pkg prefix f1tenth_stack)/share/f1tenth_stack/config/f1tenth_online_async.yaml
rviz2      # 다른 터미널에서. Fixed Frame을 map으로, /map과 /scan 표시 추가
```

맵 저장:
```bash
cd ~/YAX3_F1T_TEAM2/f1tenth_team_ws/maps
ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p map_subscribe_transient_local:=true
```

## 주행 명령 구조

`ackermann_mux`가 입력 두 개 중 하나를 골라 VESC로 보낸다.

| 토픽 | 용도 | 우선순위 | 타임아웃 |
|---|---|---|---|
| `/teleop` | 조이패드 등 수동 조종 | 100 | 0.2 s |
| `/drive` | **자율주행 코드는 여기로 발행** | 10 | 0.2 s |

메시지 타입은 `ackermann_msgs/msg/AckermannDriveStamped`다. `/teleop`이 0.2초 동안 안 들어오면 `/drive`가 다시 적용되므로, 정지 명령을 `/teleop`으로 한 번만 보내면 멈춘 상태가 유지되지 않는다.

같은 이유로 `/drive`도 타임아웃 안에 계속 발행해야 한다. 20 Hz 이상을 권장한다.

## 이 차의 특성

- **저속 한계:** 0.5 m/s 아래에서는 센서리스 모터가 덜컥거린다. 조향할수록 더 높은 속도가 필요하다(조향 1 rad당 약 1 m/s 추가).
- **정지:** |속도| < 0.3 m/s 명령은 브레이크 전류(20 A)로 처리된다. 저속 속도 제어를 쓰면 바퀴가 뒤로 튀기 때문이다.
- **후방 사각지대:** 라이다는 차 뒤쪽을 보지 못한다. 후진할 때 주의한다.
- **라이다에 차체가 보인다:** 스캔에 바퀴와 차체 점이 섞이므로 장애물 판단 전에 걸러내야 한다.
- **조향 한계:** ±0.34 rad

## 패치 내용

`patches/`의 수정 사항이다. 전체 내용은 파일에서 볼 수 있다.

| 파일 | 수정 | 이유 |
|---|---|---|
| `0001-car-config.patch` | 조향 중심 0.500 | 직진 주행으로 맞춘 값 (기본 0.5304는 오른쪽으로 쏠림) |
| | `brake_below_speed` 0.3, `stop_brake_current` 20 A | 저속 정지 처리. 10 A는 차가 밀렸다 |
| | odom 휠베이스 0.33, 라이다 TF | 실측값 |
| `0002-vesc-fixes.patch` | `ackermann_to_vesc` 브레이크 처리 | 위 브레이크 파라미터를 실제로 적용 |
| | `vesc_to_odom` 속도 부호 반전 | FW 7에서는 전진이 양수 ERPM |
| | 첫 서보 명령 전에도 odom 발행 | 명령 전에도 odom TF가 나와야 AMCL이 동작 |

패치는 `src/f1tenth_system`의 코드에 이미 반영돼 있어서 따로 적용할 필요가 없다. 공식 저장소의 새 버전으로 교체할 때만 다시 적용한다. `0001`은 `f1tenth_system` 루트에서, `0002`는 `vesc` 폴더에서 `git apply`로 적용한다.

가져온 공식 저장소 버전:

| 저장소 | 커밋 |
|---|---|
| f1tenth/f1tenth_system (humble-devel) | `94cb8d7` |
| f1tenth/ackermann_mux | `b3c0b08` |
| f1tenth/teleop_tools | `163827a` |
| f1tenth/vesc | `153998d` |

## 참고 자료

- 이전 팀 코드: 이 저장소의 `legacy/26-jetson-f1tenth` (race_stack)
