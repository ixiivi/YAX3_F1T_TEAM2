# 실시간 RViz 관찰 — 노트북 셋업 가이드

> 최초 작성: 2026-07-17 / CycloneDDS 전환 반영: 2026-07-21

젯슨 주행을 노트북에서 실시간(맵 + 레이싱라인 + 차 위치/궤적 + LiDAR + 목표점)으로 보기 위한 설정.

---

## 왜 CycloneDDS 인가 (기본 FastDDS 대신)

WiFi 환경에서 `/map`(67 KB)과 `/tf_static`은 **TRANSIENT_LOCAL** QoS를 쓴다.
늦게 접속한 구독자(노트북 RViz)에게 과거 발행분을 다시 보내줘야 하는데,
기본 FastDDS + WiFi 조합에서 이 재전달이 자주 실패했다.
→ `ros2 topic list`에는 토픽이 뜨는데 RViz 화면은 간헐적으로만 나오는 증상.

**CycloneDDS + 유니캐스트 설정**으로 전환한 뒤 안정화됨.

> ★ 젯슨(`race.sh`)과 노트북(`rviz.sh`) 양쪽이 같은 RMW를 써야 한다.
> RMW가 한쪽이라도 다르면 토픽이 **하나도** 안 보인다.

---

## 노트북 최초 1회 설정

### 1. ROS 2 Humble + rviz2 설치

```bash
sudo apt install ros-humble-desktop   # 또는 ros-humble-rviz2
```

### 2. CycloneDDS RMW 설치

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### 3. repo clone

```bash
git clone git@github.com:lloes098/26-jetson-f1tenth.git ~/yax
```

> `rviz.sh`가 repo 안의 `config/dds_env.sh`와 `race_script.yaml`을 읽어
> CycloneDDS 환경변수를 자동으로 설정한다. repo 없이 수동으로 맞추려면
> 아래 [수동 설정](#수동-설정-repo-없이) 참조.

---

## 매번 주행할 때

**터미널 ①** — 젯슨 SSH:
```bash
ssh jetson@192.168.45.246
cd ~/yax && ./race.sh q2      # 평소대로 주행
```

**터미널 ②** — 노트북 로컬 (SSH 아님):
```bash
cd ~/yax
./rviz.sh                     # CycloneDDS 설정 + RViz 실행
```

기동 시 아래 메시지가 뜨면 정상:
```
[dds] CycloneDDS  |  인터페이스 192.168.45.xxx 고정  |  피어 1개
```

젯슨 주행이 돌고 있으면 RViz에 맵·레이싱라인·차 위치가 바로 나온다.
주행 전에 먼저 띄워도 됨 — 주행 시작하면 자동으로 채워진다.

---

## 수동 설정 (repo 없이)

`rviz.sh` 대신 직접 명령어를 입력하는 경우, **반드시** 아래 환경변수를 먼저 설정한다:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2 -d ~/race.rviz          # race.rviz 는 젯슨에서 scp 로 받아둔 파일
```

race.rviz를 받는 법:
```bash
scp jetson@192.168.45.246:~/yax/config/race.rviz ~/race.rviz
```

> 단, 이 방법은 인터페이스 고정·유니캐스트 피어 설정이 빠진 상태라
> 같은 네트워크에서도 discovery 가 느리거나 불안정할 수 있다.
> 가능하면 repo clone + `rviz.sh` 사용을 권장한다.

---

## 문제 해결

### RViz는 뜨는데 아무것도 안 보임

**원인 1 — RMW 불일치 (가장 흔함)**

젯슨은 CycloneDDS, 노트북은 FastDDS 로 뜨면 토픽이 완전히 0이 된다.
노트북 터미널에서 확인:
```bash
echo $RMW_IMPLEMENTATION    # rmw_cyclonedds_cpp 가 나와야 함
ros2 topic list             # /waypoints_path 등이 보여야 함
```
안 보이면 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 후 재시도.

**원인 2 — 네트워크 / ROS_DOMAIN_ID 불일치**

노트북과 젯슨이 같은 네트워크(`192.168.45.x`)인지, `ROS_DOMAIN_ID`가 양쪽 동일한지 확인.
`~/.bashrc`에 `ROS_DOMAIN_ID`를 다른 값으로 설정해둔 경우 주의.

**원인 3 — 주행이 안 돌고 있음**

젯슨에서 `./race.sh q2` 가 실제로 살아 있는지 확인.

---

### `[dds] ⚠ CycloneDDS 미설치` 메시지

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

노트북·젯슨 **양쪽** 다 설치해야 한다.

---

### `[dds] CycloneDDS (자동 탐색)` 메시지 (인터페이스 고정 없음)

주행 내부망(`192.168.45.x`) 에 노트북이 연결되지 않은 상태.
→ 주행망 WiFi 에 먼저 접속한 뒤 `rviz.sh` 를 다시 실행.

---

### `/map` 이나 레이싱라인만 안 나옴

TRANSIENT_LOCAL 래치 데이터를 못 받은 것.
주행 시작 직후 RViz 를 열면 이미 발행된 래치 메시지를 못 받을 수 있다.
RViz 를 재시작하거나 잠깐 기다리면 재전송된다.

---

## 젯슨에서 발행하는 시각화 토픽

| 토픽 | 내용 |
|------|------|
| `/waypoints_path` | 전체 레이싱라인 (속도 컬러, 1 Hz 재발행) |
| `/car_path` | 차가 지나온 궤적 |
| `/waypoint` | 현재 목표점 |
| `/obstacle` | Q2 장애물 위치 |
| `/map` | 점유 격자 지도 |
| `/scan` | LiDAR 포인트 클라우드 |
| `/amcl_pose` + TF | 차 위치 |

viz 설정값과 무관하게 항상 발행되므로, **노트북에서 뷰어만 켜면 된다.**

---

## 참고

- `race_script.yaml` → `network:` 섹션에서 DDS 종류·대역·피어 IP 관리
- 젯슨 IP 가 바뀌면 `race_script.yaml` 의 `network.peers` 값만 수정
- `config/dds_env.sh` — 실행 시 자기 IP를 찾아 CycloneDDS XML 생성 후 환경변수 export
- `config/cyclonedds.xml.in` — CycloneDDS 설정 템플릿 (유니캐스트·버퍼·단편화 설정)
