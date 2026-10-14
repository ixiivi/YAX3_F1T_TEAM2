#!/usr/bin/env bash
# ============================================================
#  race.sh — 딸깍 스크립트
#
#  사용법:
#    ./race.sh                 race_script.yaml의 mode대로 실행
#    ./race.sh mapping         모드 임시 오버라이드 (yaml 안 바꿔도 됨)
#    ./race.sh mapping-joy     조이스틱으로 SLAM 맵핑
#    ./race.sh q1|q2|q3|race [컨트롤러]
#                              컨트롤러 생략 시 yaml의 controller 값 사용
#                              (auto | stanley | pure_pursuit | gap_follower | map)
#                              예: ./race.sh q1 pure_pursuit
#    ./race.sh box [모드]      ★ 박스 위 벤치 테스트 — 바퀴를 돌리지 않는다.
#                              실주행과 동일한 스택(맵/AMCL/LiDAR/컨트롤러)을 띄우되
#                              속도만 0. 토픽 통신·RViz 표시·조향 서보 확인용.
#    ./race.sh <모드> --bag    ★ 주행 중 원시 토픽을 bag 으로 녹화 (분당 약 9MB)
#                              /scan · odom · IMU · pose · drive. 사후 분석용.
#                              logs/bags/<시각>_<모드>_<컨트롤러>/ 에 쌓이고,
#                              Ctrl+C 후 CSV·bag 을 car1/car2 에 커밋·푸시한다.
#    ./race.sh push-logs       아직 안 올라간 logs/ 를 커밋·푸시 (팀 공유기에서
#                              푸시가 실패한 뒤, 인터넷 되는 와이파이에서)
#    ./race.sh check           토픽이 실제로 발행 중인지 표로 확인 (주행 중 별도 터미널)
#    ./race.sh stop            돌고 있는 스택 전부 정리
#    ./race.sh savemap <이름>  맵핑 끝난 뒤 맵 저장 (맵핑 창을 끄기 전에)
#    ./race.sh teleop          키보드 텔레옵만 단독 실행
#    ./race.sh joy             조이스틱 수동 조종 (브링업 포함)
#    ./race.sh log <이름>      웨이포인트 기록 (수동주행하며 레이싱라인 녹화)
#    ./race.sh waypoints [맵]  SLAM 맵에서 레이싱라인 생성 (GUI, 디스플레이 필요)
#    ./race.sh clean [맵]      SLAM 맵 정제 (빛살 제거). 화면 없이 대화형
#    ./race.sh line [맵] [옵션]  ★ 레이싱라인 생성 (최소곡률 QP, 자동)
#                              예) ./race.sh line 0815_clean --apex-extra 0.5
#    ./race.sh waypoints-kappa [맵]  위와 동일 + 조향각 물리한계(곡률) 제약
#                              → <맵>_wp_kappa.csv 로 따로 저장, 랩타임 비교용
#    ./race.sh waypoints-tumftm [맵]  TUMFTM 최소곡률(IQP) QP 최적화 (비-GUI, 자동)
#                              → <맵>_wp_tumftm.csv 로 따로 저장, 랩타임 비교용
#    ./race.sh curvprobe       곡률-속도 캘리브레이션 리그 (slip과 별개 도구)
#    ./race.sh steertrim       조향 중립값 자동 캘리브레이션 (IMU 기반, 전/후진 반복)
#    ./race.sh savepose        ★ 지금 AMCL 위치를 출발점으로 저장 (init_* + start_*)
#                              savemap 이 맵 원점으로 이미 넣어두므로,
#                              **출발 자리를 옮길 때만** 쓰면 됩니다
#                              맵핑 중이 아니라 ./race.sh box (주행 스택) 위에서.
#                              새 맵이면 RViz 2D Pose Estimate 로 맞춘 뒤 1회.
#
#  게인 튜닝은 ./tune.sh — yaml 을 열지 않고 값만 바꾼다. 설명: ./tune.sh -h
#    ./tune.sh i                     속도 대화형. η q1/q2/race 는 모드마다 따로
#                                    (./race.sh q1 → q1 만, ./race.sh race → race 만)
#    ./tune.sh cap=3                 전 모드 공통 천장
#    ./tune.sh q1=0.6                ./race.sh q1 전용. q2= / race= 도 됨
#    ./tune.sh stanley.gain=1.2      다음 ./race.sh 부터 적용 (재빌드 불필요)
# ============================================================
set -e

WS=${RACE_WS:-$HOME/f1tenth_ws}
# gen_waypoints 등 파이썬 노드는 RACE_WS 로 맵/웨이포인트 폴더를 찾는다. export 안 하면
# __file__ 기준으로 build/race_stack/maps(빌드 시점 스냅샷 복사본)를 읽어 새 맵이 안 보인다.
# WS/src/race_stack 는 repo(~/yax)로 심링크돼 있으므로 여기를 가리키게 export.
export RACE_WS="$WS"
# ~/f1tenth_ws/race.sh 는 이 repo로 가는 심링크 — readlink -f 로 실제 위치(~/yax)를
# 찾아야 race_script.yaml 을 제대로 읽는다 (심링크 경유 실행 시 launch가 즉사하던 버그)
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
# 주행 로그(logs/)는 실행 워크스페이스가 아니라 **git repo** 에 쌓여야 한다.
# RACE_WS(~/f1tenth_ws)를 쓰던 시절엔 로그가 repo 밖에 떨어져 커밋·공유가 안 됐다
# (2026-08-06 발견). logs/README.md 가 전제하는 "git 으로 공유" 를 성립시킨다.
export RACE_REPO="$SCRIPT_DIR"
RACE_SCRIPT="$SCRIPT_DIR/race_script.yaml"
PKG_SHARE_MAPS="$WS/src/race_stack/maps"
PKG_SHARE_WP="$WS/src/race_stack/waypoints"

# ---------- 0-1. 차량별 캘리브레이션 펼치기 ----------
# 차가 두 대인데 저장소는 하나다. config/vesc.yaml 을 그대로 공유하면 2호차의
# 음수 speed_to_erpm_gain 이 1호차로 넘어가 **차가 후진한다**. 그래서 코드는
# 합치고 값만 분리한다 — .car 가 가리키는 config/vehicle_<차>.yaml 에서
# config/vesc.yaml 과 race_script 덮어쓰기(overlay)를 매 실행마다 새로 만든다.
# vesc.yaml 은 .gitignore 라 잘못 커밋할 방법 자체가 없다.
python3 "$SCRIPT_DIR/config/apply_vehicle.py" || exit 1
export RACE_VEHICLE_OVERLAY="$SCRIPT_DIR/config/.vehicle_overlay.yaml"

# ---------- 0. 환경 소싱 ----------
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# ★ 다른 워크스페이스가 race_stack 을 가로채지 않는지 확인 (2026-08-03 실제로 겪음)
#   colcon 은 빌드 시점에 소싱돼 있던 prefix 를 install/setup.sh 에 부모로 기록한다.
#   ~/yax 에서 한 번 colcon build 하고 그 상태로 f1tenth_ws 를 빌드했더니
#   f1tenth_ws/install/setup.sh 가 ~/yax/install 을 먼저 소싱하게 됐고, 그래서
#   여기서 $WS 를 source 해도 한 달 전 스냅샷이 이겨서 새 노드를 "not found" 로
#   죽었다. 조용히 옛 코드가 도는 쪽이 더 위험하므로 명시적으로 막는다.
ACTUAL_PREFIX="$(ros2 pkg prefix race_stack 2>/dev/null || true)"
if [ -n "$ACTUAL_PREFIX" ] && [ "$ACTUAL_PREFIX" != "$WS/install/race_stack" ]; then
    echo "[race.sh] ✗ race_stack 이 엉뚱한 워크스페이스에서 로드됩니다."
    echo "          기대: $WS/install/race_stack"
    echo "          실제: $ACTUAL_PREFIX"
    echo "          → 그 경로의 install/build 를 치우고, 깨끗한 셸에서 재빌드하세요:"
    echo "             env -i HOME=\$HOME PATH=\$PATH bash -c 'source /opt/ros/humble/setup.bash; \\"
    echo "               cd $WS && colcon build --packages-select race_stack --symlink-install'"
    exit 1
fi

# DDS 설정 (race_script.yaml 의 network: 섹션). 노트북에서 RViz 로 관찰할 때
# 통신 안정성을 좌우한다 — rviz.sh 도 같은 파일을 source 해서 양쪽을 맞춘다.
export RACE_SCRIPT="$SCRIPT_DIR/race_script.yaml"
[ -f "$SCRIPT_DIR/config/dds_env.sh" ] && source "$SCRIPT_DIR/config/dds_env.sh"

# ---------- 1. VESC 심링크 (재부팅마다 필요) ----------
if [ ! -e /dev/sensors/vesc ]; then
    echo "[race.sh] /dev/sensors/vesc 심링크 생성"
    sudo mkdir -p /dev/sensors
    sudo ln -sf /dev/ttyACM0 /dev/sensors/vesc
fi

# ---------- 1-1. loopback 멀티캐스트 (재부팅마다 필요) ----------
# 이 젯슨은 lo 에 MULTICAST 플래그가 없어서, 같은 호스트 두 프로세스도
# 오직 WiFi/이더넷 멀티캐스트로만 서로를 발견한다. WiFi만 연결된 상태(SSH)
# 에서는 discovery 가 불안정해져 다른 터미널의 /map(slam_toolbox)을 못 찾아
# savemap 이 실패한다. lo 멀티캐스트를 켜서 loopback discovery 경로를 열어준다.
if ! ip link show lo | head -1 | grep -q "MULTICAST"; then
    echo "[race.sh] lo 멀티캐스트 활성화 (같은 호스트 DDS discovery 안정화)"
    sudo ip link set dev lo multicast on
fi

# ---------- 1-2. 배터리 잔량 ----------
# 방전으로 주행이 중간에 끊기는 일이 있어 기동 시 한 줄 띄운다.
# --inline: 전압을 못 읽어도 exit 0 — 기동을 막지 않는다.
# ★ bat.sh 는 스택 미실행 시 vesc_driver 를 잠깐 띄웠다 반드시 회수한다.
#   남아 있으면 아래 2-1 중복 실행 가드가 오탐하고 VESC 시리얼도 물고 있다.
[ -x "$SCRIPT_DIR/bat.sh" ] && "$SCRIPT_DIR/bat.sh" --inline || true

# ---------- 2. 모드 결정 ----------
export RACE_SCRIPT   # launch 파일이 이 env로 yaml 경로를 읽음
# ---------- --bag: 주행 중 원시 토픽 녹화 ----------
# 위치 인자($2=컨트롤러)와 섞이지 않게 **먼저** 걸러낸다.
#   ./race.sh q1 map --bag   ./race.sh q1 --bag   둘 다 된다.
# 왜 필요한가: driven_path.csv 에는 컨트롤러가 이미 해석한 값만 남는다.
# /scan 원시 데이터가 없으면 deskew·AMCL·ICP 같은 앞단 문제를 사후에 못 판정한다
# (2026-08-09: 고속 deskew 를 켤지 정하려는데 고속 주행 bag 이 하나도 없었다).
RECORD_BAG=0
_args=()
for _a in "$@"; do
    case "$_a" in
        --bag) RECORD_BAG=1 ;;
        *) _args+=("$_a") ;;
    esac
done
set -- "${_args[@]}"

CMD=${1:-}
# ★ 차량 overlay 를 먼저 본다 — race.launch.py 와 같은 규약.
#   mapping-joy 처럼 race.sh 가 직접 값을 읽어 -p 로 넘기는 경로가 있는데,
#   여기서 overlay 를 안 보면 그 경로만 race_script.yaml 의 공용값으로 돈다
#   (2026-08-10: joystick 절을 차량 파일로 옮겼는데 조이스틱 모드에 안 먹었다).
get_yaml () { python3 -c "
import yaml, os
key = '$1'.split('.')
def dig(path):
    d = yaml.safe_load(open(path))
    for k in key: d = d[k]
    return d
val = None
ovl = os.environ.get('RACE_VEHICLE_OVERLAY', '')
if ovl and os.path.exists(ovl):
    try: val = dig(ovl)
    except Exception: val = None
if val is None: val = dig('$RACE_SCRIPT')
print(val)"; }

# 돌고 있는 스택 프로세스 PID 목록.
# ★ launch 부모만 찾으면 안 된다. 터미널을 pkill 하거나 SSH 가 끊기면 부모(launch)만
#   죽고 urg_node·map_server·컨트롤러가 고아로 살아남는다. 이 상태를 탐지 못 하면
#   다음 실행에서 LiDAR TCP 포트·VESC 시리얼을 고아가 쥐고 있어 그대로 충돌한다.
#   그래서 부모와 자식 노드를 모두 본다.
#   ★ 빠짐없이 적을 것. joy_node/joy_teleop/static_transform_publisher 가 빠져 있어
#     실행할 때마다 누적됐고, 6세대 17개가 RTPS 포트(7400/7410~)를 쥔 채 남아 있었다.
#     FastDDS 는 견뎠지만 CycloneDDS 는 참여자 인덱스를 못 찾아 스택 전체가 죽었다
#     ("Failed to find a free participant index for domain 0").
STACK_PROCS="[r]ace\.launch\.py|[m]apping\.launch\.py|[u]rg_node_driver|[v]esc_driver_node|\
[n]av2_amcl/amcl|[n]av2_map_server/map_server|[l]ifecycle_manager|[s]tanley_controller|\
[m]ission_manager|[p]ure_pursuit|[g]ap_follower|[m]ap_controller|[s]afety_node|[a]ckermann_mux|\
[a]ckermann_to_vesc_node|[v]esc_to_odom_node|[s]lam_toolbox|\
[j]oy_node|[j]oy_teleop|[s]tatic_transform_publisher|[k]eyboard_teleop|[j]oystick_teleop|\
[w]aypoint_logger"
running_pids () { pgrep -f "$STACK_PROCS" 2>/dev/null || true; }

kill_stack () {
    [ -z "$(running_pids)" ] && return 0
    # launch 부모부터 정리해야 자식이 respawn 되지 않는다.
    local parents; parents=$(pgrep -f "[r]ace\.launch\.py|[m]apping\.launch\.py" 2>/dev/null || true)
    if [ -n "$parents" ]; then
        echo "[race.sh] launch 종료 (PID: $(echo $parents | tr '\n' ' '))"
        kill $parents 2>/dev/null || true
        sleep 3
    fi
    local orphans; orphans=$(running_pids)
    if [ -n "$orphans" ]; then
        echo "[race.sh] 남은 노드 정리 (PID: $(echo $orphans | tr '\n' ' '))"
        kill $orphans 2>/dev/null || true
        sleep 2
        orphans=$(running_pids)
        [ -n "$orphans" ] && { kill -9 $orphans 2>/dev/null || true; sleep 1; }
    fi
    if [ -n "$(running_pids)" ]; then
        echo "[race.sh] ⚠ 아직 남은 프로세스가 있습니다. 수동 확인:"
        echo "          ps -ef | grep -E 'race.launch|urg_node|vesc_driver|map_server'"
        return 1
    fi
    echo "[race.sh] 정리 완료"
}

print_mapping_howto () {
    # mapping / mapping-joy 기동 시 터미널에 띄우는 순서. 자세한 설명은
    # JETSON_COMMANDS.md 「권장 절차」.
    # ★ savemap 이 출발점(init_* + start_*)을 맵 원점으로 저장한다 (2026-08-14).
    local how=$1
    echo ""
    echo ">>> ★ 차를 **출발선에 놓고** SLAM 을 시작하세요."
    echo ">>>   맵 원점(0,0,0) = 지금 이 자리 = 출발점입니다. savemap 이 그대로 저장합니다."
    echo ">>>   여기서 시작하지 않으면 나중에 ./race.sh savepose 로 따로 맞춰야 합니다."
    echo ">>>"
    echo ">>> $how로 천천히 트랙 1~2바퀴."
    echo ">>>"
    echo ">>> 순서 — 이 창은 끄지 말 것 (먼저 Ctrl+C 하면 /map 이 사라져 저장 실패)"
    echo ">>>   1) 이 창: 주행"
    echo ">>>   2) 다른 터미널:  ./race.sh savemap <새이름>"
    echo ">>>   3) 저장 확인 후 이 창에서 Ctrl+C"
    echo ">>>   4) ./race.sh clean <새이름>           ← 빛살 제거 (<이름>_clean 생성)"
    echo ">>>   5) race_script.yaml 의 map_name / waypoints 를 _clean 이름으로"
    echo ">>>   6) ./race.sh waypoints-kappa          ← 라인 생성"
    echo ">>>   7) 차를 **지금 이 자리**에 두고  ./race.sh box"
    echo ">>>"
    echo ">>> ★ 출발점은 savemap 이 맵 원점(=지금 차 자리)으로 자동 저장합니다."
    echo ">>>   맵핑을 시작한 자리가 곧 출발선이므로 savepose 를 따로 할 필요가 없습니다."
    echo ">>>   출발 자리를 옮기고 싶을 때만: RViz 2D Pose Estimate 로 맞춘 뒤"
    echo ">>>   ./race.sh savepose  (맵핑 중이 아니라 AMCL 주행 스택 위에서)."
    echo ""
}

# ★ box(bench) 전용 — bringup 의 joy_node/joy_teleop 을 죽인다.
#   각 컨트롤러가 발행 직전에 속도를 0 으로 못박아도(d4b65f3), 조이스틱은
#   /teleop 으로 따로 나가고 mux.yaml 우선순위가 teleop:100 vs drive:10 이라
#   그 방어를 통째로 우회한다. F710 은 SDL 리매핑에서 joy_teleop 이 보는 axis 1 이
#   "놓은 상태 -1" 인 트리거에 걸려 -1 × scale 5.0 = 전속 후진이 된다.
#   (2026-08-09 2호차 실제 발생: ./race.sh box 로 아무것도 안 건드렸는데 차가 후진)
#   ★ 한 번만 쏘면 안 된다 — launch 가 아직 joy_node 를 안 띄운 사이에 지나가면
#     그 뒤로 무방비가 된다. 뜨자마자 잡도록 기동 구간(8초) 내내 반복한다.
bench_kill_joy () {
    for _ in $(seq 1 16); do
        pkill -9 -f '[j]oy_node'   2>/dev/null || true
        pkill -9 -f '[j]oy_teleop' 2>/dev/null || true
        sleep 0.5
    done
    echo "[race.sh] BENCH — joy_node/joy_teleop 정리 완료 (조이스틱 입력 무시)"
}

# logs/ (CSV + bag) 를 현재 브랜치에 커밋하고 푸시한다. 코드 WIP 는 안 건드린다.
commit_logs () {
    cd "$SCRIPT_DIR" || return 0
    git rev-parse --is-inside-work-tree >/dev/null 2>&1 || return 0
    local br; br=$(git rev-parse --abbrev-ref HEAD)
    if [ "$br" = "main" ]; then
        echo "[race.sh] 로그 git 생략 — 브랜치가 main 입니다. car1/car2 에서 올리세요."
        return 0
    fi
    git add logs/ || true
    local need_push=0
    if ! git diff --cached --quiet; then
        echo "[race.sh] 로그 커밋:"
        git diff --cached --name-only | sed 's/^/          /'
        local names
        names=$(git diff --cached --name-only \
            | awk -F/ '/^logs\// && NF>=2 {print $2}' \
            | sort -u | tr '\n' ' ' | sed 's/ *$//')
        git commit -m "$(cat <<EOF
logs: ${names:-주행 기록}

EOF
)" || true
        need_push=1
    fi
    if git rev-parse --abbrev-ref '@{u}' >/dev/null 2>&1; then
        local ahead; ahead=$(git rev-list --count '@{u}..HEAD' 2>/dev/null || echo 0)
        [ "$ahead" != "0" ] && need_push=1
    elif [ "$need_push" = "0" ]; then
        return 0
    fi
    [ "$need_push" = "0" ] && return 0
    echo "[race.sh] 로그 푸시 ($br)…"
    if git push -u origin HEAD; then
        echo "[race.sh] 올렸습니다. 노트북에서 git pull origin $br"
    else
        echo "[race.sh] 푸시 실패 — 팀 공유기는 GitHub 가 안 됩니다."
        echo "          인터넷 되는 와이파이에서  ./race.sh push-logs"
    fi
}

case "$CMD" in
  stop)
    kill_stack; exit $? ;;
  push-logs)
    commit_logs
    exit 0 ;;
  go)
    # 출발 게이트 해제 (mission.start_gate: true 일 때).
    # 같은 터미널에서 Enter 를 못 쓰는 경우(tmux·jetson.sh 원격 등) 용.
    echo "[race.sh] 출발 신호 발행 → /race/go"
    exec ros2 topic pub -1 /race/go std_msgs/msg/Bool '{data: true}' ;;
  check)
    # 주행 중 다른 터미널에서 실행 — 각 토픽이 실제로 발행되고 있는지 확인.
    # ros2 topic list 는 discovery 만 되면 뜨므로 판정 근거가 못 된다.
    # Publisher count 와 실제 수신 여부를 둘 다 본다.
    echo "============================================"
    echo "  토픽 상태 점검  |  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    echo "============================================"
    printf "%-20s %-10s %s\n" "TOPIC" "PUB" "수신"
    printf "%-20s %-10s %s\n" "--------------------" "----------" "------"
    for t in /map /scan /odom /tf /amcl_pose /drive /ackermann_cmd \
             /commands/motor/speed /race/enabled /waypoints_path; do
        n=$(ros2 topic info "$t" 2>/dev/null | sed -n 's/Publisher count: //p')
        n=${n:-0}
        if [ "$n" -gt 0 ] 2>/dev/null; then
            # transient_local(/map, /race/enabled)도 잡히도록 echo 로 실수신 확인.
            # 젯슨에서 ros2 CLI 노드 기동이 느려 5초로는 40Hz 토픽도 오탐이 난다.
            if timeout 10 ros2 topic echo "$t" --once >/dev/null 2>&1; then
                rx="OK"
            else
                rx="✗ 발행자는 있는데 데이터 없음"
            fi
        else
            rx="✗ 발행자 없음"
        fi
        printf "%-20s %-10s %s\n" "$t" "$n" "$rx"
    done
    echo ""
    echo "  ※ 노트북에서 이 스크립트를 돌리면 젯슨↔노트북 DDS 연결까지 검증됩니다."
    exit 0 ;;
  savemap)
    NAME=${2:?맵 이름을 주세요: ./race.sh savemap my_track}

    # /map 발행자가 없으면 map_saver 는 그냥 타임아웃만 내고 끝난다.
    # 원인을 알려주는 게 낫다 — 대부분 맵핑 창을 Ctrl+C 로 먼저 꺼버린 경우다.
    if ! ros2 topic info /map 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        echo "[race.sh] ✗ /map 을 발행하는 노드가 없습니다 (slam_toolbox 미실행)."
        echo ""
        echo "  맵은 slam_toolbox 가 메모리에 들고 있습니다."
        echo "  맵핑 창을 Ctrl+C 로 끄면 맵도 함께 사라져서 저장할 수 없습니다."
        echo ""
        echo "  올바른 순서:"
        echo "    1) 터미널A:  ./race.sh mapping-joy    ← 켜 둔 채로"
        echo "    2) 주행으로 트랙 한 바퀴"
        echo "    3) 터미널B:  ./race.sh savemap $NAME  ← A를 끄기 전에!"
        echo "    4) 저장 확인 후 터미널A 에서 Ctrl+C"
        echo "    5) clean → yaml map_name 바꾼 뒤 waypoints-kappa → box"
        echo "       출발점은 savemap 이 맵 원점으로 자동 저장합니다 (savepose 불필요)."
        exit 1
    fi

    # slam 은 odom TF 가 있어야 맵을 만든다. vesc_to_odom 은 서보 명령을 한 번
    # 받기 전까지 odom 을 발행하지 않으므로, 주행을 전혀 안 했으면 맵이 비어 있다.
    if ! ros2 topic info /odom 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        echo "[race.sh] ⚠ /odom 발행자가 없습니다 — 맵이 비어 있을 수 있습니다."
        echo "          조종기로 조금이라도 주행한 뒤 저장하세요."
    fi

    echo "[race.sh] 맵 저장 → $PKG_SHARE_MAPS/$NAME"
    ros2 run nav2_map_server map_saver_cli -f "$PKG_SHARE_MAPS/$NAME" --ros-args -p map_subscribe_transient_local:=true
    # install 쪽에도 복사 (rebuild 없이 바로 사용)
    INSTALL_MAPS="$WS/install/race_stack/share/race_stack/maps"
    [ -d "$INSTALL_MAPS" ] && cp "$PKG_SHARE_MAPS/$NAME".* "$INSTALL_MAPS/" || true
    # ── 출발점을 맵 원점으로 자동 설정 ────────────────────────────────
    #
    # ★ SLAM 의 map 프레임 원점 (0,0,0) 은 **맵핑을 시작한 순간의 차 자리**다.
    #   slam_toolbox 가 첫 스캔을 원점에 놓고 시작하기 때문이다. 즉 차를 출발선에
    #   놓고 맵핑을 시작했다면 출발점은 이미 (0,0,0) 으로 정해져 있다.
    #   실측으로도 확인했다 — 0813-1_clean 은 (0,0) 이 벽에서 0.51m,
    #   0814-1_clean 은 0.70m 로 둘 다 트랙 한복판이다.
    #
    # ★ 왜 자동으로 넣나 — 2026-08-14 에 맵만 바꾸고 이 좌표를 안 고쳐서
    #   **옛 좌표가 벽 속**이 됐고 AMCL 이 위치를 아예 못 잡았다. 게다가
    #   auto_localize 의 탐색 범위가 ±0.5m 라 원리적으로 못 찾는 상태였다.
    #   출발선(mission.start_*)도 같이 벽 속이라 랩도 안 세어졌다.
    #   맵을 만든 그 자리에서 같이 써 두면 이 사고가 구조적으로 안 난다.
    #
    #   출발 위치를 옮기고 싶을 때만 ./race.sh savepose 를 쓰면 된다.
    echo "[race.sh] 출발점을 맵 원점(=맵핑 시작 시 차 자리)으로 설정합니다"
    if "$SCRIPT_DIR/tune.sh" localization.init_x=0.0 localization.init_y=0.0 \
                             localization.init_yaw=0.0 \
                             mission.start_x=0.0 mission.start_y=0.0 >/dev/null; then
        echo "[race.sh]   localization.init_x/y/yaw = 0, 0, 0"
        echo "[race.sh]   mission.start_x/y         = 0, 0"
    else
        echo "[race.sh] ⚠ 좌표 저장 실패 — 손으로 넣거나 ./race.sh savepose 를 쓰세요."
    fi

    echo "[race.sh] 완료. race_script.yaml 의 map_name 을 '$NAME' 으로 바꾸세요."
    echo "[race.sh] 다음:"
    echo "          1) 맵핑 창에서 Ctrl+C"
    echo "          2) ./race.sh clean $NAME              ← 빛살 제거"
    echo "          3) yaml 의 map_name / waypoints 를 '${NAME}_clean' 으로"
    echo "          4) ./race.sh waypoints-kappa"
    echo "          5) 차를 **맵핑 시작한 그 자리**에 두고 ./race.sh box"
    echo "             → 출발점은 이미 저장됐으니 savepose 가 필요 없습니다."
    echo "          ※ 출발 위치를 옮기고 싶을 때만 RViz 2D Pose Estimate 로 맞춘 뒤"
    echo "            ./race.sh savepose (AMCL 스택이 떠 있어야 합니다)."
    exit 0 ;;
  teleop)
    exec ros2 run race_stack keyboard_teleop --ros-args -p max_speed:=$(get_yaml speed.mapping_max) ;;
  joy)
    exec ros2 launch race_stack joystick.launch.py ;;
  slip)
    # 타이어 슬립 데이터 수집. 조종(조이스틱)까지 한 번에 올린다.
    # ←→ = 시험 조향각(= L스틱 최대값), ↑↓ = 램프 목표 속도, B = 시작/종료
    exec ros2 launch race_stack slip.launch.py ;;
  curvprobe)
    # 곡률-속도 계단 캘리브레이션 리그. slip 과 별개 도구/설정(curv_probe: 섹션).
    # 브링업은 백그라운드, curvature_probe 노드는 키보드 raw 입력이 필요해
    # mapping 과 같은 패턴으로 포그라운드에서 직접 띄운다.
    # 조작: SPACE=다음 속도단계  q=정지+저장+종료   분석: --analyze
    ros2 launch race_stack curvature_probe.launch.py &
    LAUNCH_PID=$!
    sleep 5
    trap "kill $LAUNCH_PID 2>/dev/null" EXIT
    ros2 run race_stack curvature_probe --ros-args \
      -p full_lock_steer:=$(get_yaml curv_probe.full_lock_steer) \
      -p steer_sign:=$(get_yaml curv_probe.steer_sign) \
      -p speed_start:=$(get_yaml curv_probe.speed_start) \
      -p speed_step:=$(get_yaml curv_probe.speed_step) \
      -p speed_max:=$(get_yaml curv_probe.speed_max) \
      -p accel_limit:=$(get_yaml curv_probe.accel_limit) \
      -p min_hold_sec:=$(get_yaml curv_probe.min_hold_sec) \
      -p kappa_max:=$(get_yaml curv_probe.kappa_max) \
      -p mu:=$(get_yaml curv_probe.mu)
    exit 0 ;;
  steertrim)
    # 조향 트림 자동 캘리브레이션. steer_trim: 섹션, curv_probe/slip과 별개.
    # 조작: q=즉시중단(저장안함)  s=이번 다리 끝나면 저장+종료
    # ★ 전진·후진 각 test_dist(기본 2m)만큼 자율 주행하니 앞뒤로 공간 확보할 것.
    ros2 launch race_stack steer_trim.launch.py &
    LAUNCH_PID=$!
    sleep 5
    trap "kill $LAUNCH_PID 2>/dev/null" EXIT
    ros2 run race_stack steer_trim --ros-args \
      -p test_speed:=$(get_yaml steer_trim.test_speed) \
      -p test_dist:=$(get_yaml steer_trim.test_dist) \
      -p accel_limit:=$(get_yaml steer_trim.accel_limit) \
      -p max_trim:=$(get_yaml steer_trim.max_trim) \
      -p kappa_tol:=$(get_yaml steer_trim.kappa_tol) \
      -p correction_damp:=$(get_yaml steer_trim.correction_damp) \
      -p max_iters:=$(get_yaml steer_trim.max_iters) \
      -p front_safety_m:=$(get_yaml steer_trim.front_safety_m) \
      -p rear_safety_m:=$(get_yaml steer_trim.rear_safety_m) \
      -p auto_write:=$(get_yaml steer_trim.auto_write)
    exit 0 ;;
  log)
    NAME=${2:?웨이포인트 이름을 주세요: ./race.sh log my_track_wp}
    exec ros2 run race_stack waypoint_logger --ros-args -p output:="$PKG_SHARE_WP/$NAME.csv" ;;
  clean)
    # SLAM 원본 맵에서 **트랙 경계 틈으로 새어 나간 가는 빛살**을 지운다.
    #   빛살 폭 1~3px   /  트랙 폭 20px 이상  →  opening 반경 하나로 갈린다
    #
    # ★ clean_map.py(GUI 폴리곤)와 같은 일을 하되 **디스플레이가 필요 없다.**
    #   대회장·SSH 에서 쓸 수 있는 쪽이 이것이다. 맵을 목록에서 고르고, 결과
    #   수치를 보고 반경을 바꿔가며 되풀이한 뒤 저장한다.
    #   맞는 반경은 트랙마다 다르므로 <맵>_clean_preview.png(빨강=지워진 영역)를
    #   반드시 눈으로 확인할 것.
    NAME=${2:-}
    if [ -n "$NAME" ]; then
      exec python3 "$SCRIPT_DIR/src/race_stack/maps/clean_map_auto.py" --map "$NAME"
    else
      exec python3 "$SCRIPT_DIR/src/race_stack/maps/clean_map_auto.py"
    fi ;;
  line)
    # ★ 레이싱라인 생성기 (TUMFTM 최소곡률 QP + 재선형화 반복).
    #
    #   waypoints-kappa 와의 차이 — 둘 다 파체이카는 **안 쓴다** (마찰원만 쓴다).
    #     waypoints-kappa : 사람이 GUI 로 경로를 스케치 → 후보 매칭 → 랩타임 최적화
    #     line(gen_raceline): 맵에서 자동으로 최소곡률 QP. 사람이 안 그린다
    #
    #   0814-1_clean 실측 비교 (같은 맵·같은 물리, η=1.0):
    #       kappa      길이 26.60m  R_min 1.021  코너여유 0.749  랩 8.75s
    #       line       길이 24.77m  R_min 1.079  코너여유 0.550  랩 7.52s
    #     라인이 짧고 곡률이 낮아 1.2초 빠르지만 벽에 더 붙는다.
    #     --apex-extra(기본 0.40)가 코너 안쪽에만 여유를 되돌려 준다.
    #
    #   ※ setup.py entry point 대신 python3 -m 으로 부른다 — setup.py 를 건드리면
    #     colcon build 가 필요한데, 이건 맵 도구라 재빌드 없이 바로 써야 한다.
    #     MAPS_DIR/WP_DIR 이 __file__ 기준이라 실행 위치와 무관하다.
    #   ★★ PYTHONPATH 를 repo 로 못박는다. 안 그러면 `python3 -m` 이
    #     **colcon build 디렉토리의 사본**을 집는다:
    #         맵 이미지 없음: ~/f1tenth_ws/build/race_stack/maps/0814-1_clean.pgm
    #     MAPS_DIR 은 __file__ 기준이라, 어느 race_stack 을 import 했느냐가
    #     그대로 맵 경로가 된다. 이 저장소가 이미 겪은 "colcon prefix 그림자"
    #     와 같은 종류다 — 옛 사본이 조용히 실행되는 쪽이 더 위험하다.
    NAME=${2:-}
    export PYTHONPATH="$SCRIPT_DIR/src/race_stack:${PYTHONPATH:-}"
    if [ -n "$NAME" ]; then
      exec python3 -m race_stack.gen_raceline --map "$NAME" "${@:3}"
    else
      exec python3 -m race_stack.gen_raceline "${@:2}"
    fi ;;
  line2)
    # 열린 야외 공간용 경로 생성기 (gen_raceline_2).
    # inner/outer 경계 불필요 — 맵 위에 클릭으로 경유점을 찍으면
    # 장애물에서 밀어내며 부드럽게 정제한 웨이포인트를 만든다.
    # 닫힌 루프도 가능: --closed 또는 GUI 에서 c 키 토글.
    NAME=${2:-}
    export PYTHONPATH="$SCRIPT_DIR/src/race_stack:${PYTHONPATH:-}"
    if [ -n "$NAME" ]; then
      exec python3 -m race_stack.gen_raceline_2 --map "$NAME" "${@:3}"
    else
      exec python3 -m race_stack.gen_raceline_2 "${@:2}"
    fi ;;
  waypoints)
    NAME=${2:-}
    if [ -n "$NAME" ]; then
      exec ros2 run race_stack gen_waypoints -- --map "$NAME"
    else
      exec ros2 run race_stack gen_waypoints
    fi ;;
  waypoints-kappa)
    # gen_waypoints 와 동일하되 조향각 물리 한계(곡률) 제약이 들어간 버전.
    # vehicle overlay 의 wheelbase/max_steer 를 읽고, 한계의 85% 만 써서
    # 라인 R_min 이 차 R_min 과 같아지는 일을 막는다 (추종 오차만큼 안쪽은 κ 증가).
    # 결과는 <맵이름>_wp_kappa.csv 로 따로 저장되어 기존 결과와 랩타임 비교 가능.
    NAME=${2:-}
    if [ -n "$NAME" ]; then
      exec ros2 run race_stack gen_waypoints_kappa -- --map "$NAME"
    else
      exec ros2 run race_stack gen_waypoints_kappa
    fi ;;
  waypoints-tumftm)
    # TUMFTM 최소곡률(IQP) QP 최적화 — GUI 클릭 없이 자동으로 라인을 뽑는다.
    # 결과는 <맵이름>_wp_tumftm.csv 로 따로 저장되어 기존 결과와 랩타임 비교 가능.
    NAME=${2:-}
    if [ -n "$NAME" ]; then
      exec ros2 run race_stack gen_raceline_tumftm -- --map "$NAME"
    else
      exec ros2 run race_stack gen_raceline_tumftm
    fi ;;
  savepose)
    # 지금 AMCL 이 믿고 있는 위치를 race_script.yaml 의 초기 추정으로 굳힌다.
    #
    # 왜 필요한가: amcl.yaml 은 recovery_alpha_slow/fast=0 (전역 복구 꺼짐)이라
    # 초기 파티클은 init_x/y/yaw 주변에만 뿌려진다. 실제 출발 위치가 거기서 멀면
    # 진짜 위치를 영원히 못 찾고 어긋난 채 국소 수렴한다 (RViz 에서 맵과 스캔이
    # 겹치지 않는 증상). 한 번 맞춰서 저장해두면 다음 실행부터 자동 정렬된다.
    #
    # 사용 순서:
    #   1) ./race.sh box   (또는 주행) 으로 스택을 띄운다
    #   2) 노트북 RViz 에서 "2D Pose Estimate" 로 실제 차 위치·방향을 클릭
    #      → 맵과 라이다 스캔이 겹칠 때까지 조정
    #   3) 이 명령 실행
    # ★ /amcl_pose 토픽을 쓰지 않는다.
    #   AMCL 은 파티클 필터가 "갱신"될 때만 amcl_pose 를 발행하는데, 갱신 조건이
    #   update_min_d(2.5cm) / update_min_a(1.7도) 이동이다. box 모드는 속도가 0 이라
    #   차가 안 움직이므로 영원히 발행되지 않고 echo --once 가 무한 대기한다.
    #   TF map→base_link 는 tf_broadcast:true 라 정지 상태에서도 계속 방송되므로
    #   이쪽을 읽는다. (RViz 에서 2D Pose Estimate 로 옮긴 결과도 즉시 반영된다)
    echo "[race.sh] AMCL 위치 조회 중 — TF map→base_link (최대 15초)…"
    python3 - "$SCRIPT_DIR" <<'PYEOF'
import math, subprocess, sys, time

import rclpy
from rclpy.node import Node
import tf2_ros

script_dir = sys.argv[1]

rclpy.init()
node = Node('savepose_probe')
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, node)

tf = None
deadline = time.time() + 15.0
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    try:
        tf = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        break
    except tf2_ros.TransformException:
        pass

if tf is None:
    print('[race.sh] map→base_link TF 를 못 받았습니다.', flush=True)
    print('          스택이 떠 있고 AMCL 이 active 인지 확인하세요:', flush=True)
    print('            ./race.sh check        (토픽·TF 상태 표)', flush=True)
    print('            ros2 topic echo /tf --once', flush=True)
    node.destroy_node(); rclpy.shutdown(); sys.exit(1)

t, q = tf.transform.translation, tf.transform.rotation
yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
print(f'[race.sh] 현재 AMCL 위치: ({t.x:.3f}, {t.y:.3f})  '
      f'{math.degrees(yaw):.1f}deg', flush=True)
node.destroy_node(); rclpy.shutdown()

# tune.py 로 기록 — 주석·정렬이 보존되고 변경 이력도 남는다
# ★ mission.start_x/y 도 같이 쓴다 (2026-08-14 추가).
#   예전엔 init_* 만 써서, 맵을 바꾼 뒤 **출발선이 옛 좌표(벽 속)에 남아**
#   위치는 잡아도 랩이 하나도 안 세어졌다. 둘은 같은 "출발 자리" 를 가리키는
#   값이므로 따로 두면 반드시 어긋난다.
r = subprocess.run([f'{script_dir}/tune.sh',
                    f'localization.init_x={t.x:.4f}',
                    f'localization.init_y={t.y:.4f}',
                    f'localization.init_yaw={yaw:.4f}',
                    f'mission.start_x={t.x:.4f}',
                    f'mission.start_y={t.y:.4f}'])
if r.returncode != 0:
    print('[race.sh] 저장 실패 — 위 [tune] 메시지를 확인하세요.', flush=True)
    sys.exit(1)
PYEOF
    echo "[race.sh] 저장 완료 — init_x/y/yaw 와 mission.start_x/y 를 함께 갱신했습니다"
    echo "[race.sh]   같은 맵이면 매번 2D Pose Estimate 를 찍을 필요 없습니다."
    echo "[race.sh]   ※ 맵을 새로 뜨면 savemap 이 출발점을 맵 원점으로 자동 저장하므로"
    echo "[race.sh]     savepose 는 **출발 자리를 옮길 때만** 쓰면 됩니다."
    exit 0 ;;
  "" ) MODE=$(get_yaml mode); CTRL="" ;;
  mapping|mapping-joy) MODE=$CMD; CTRL="" ;;
  box)
    # 박스 위 벤치 테스트. 모드는 인자 > yaml 순으로 결정 — 실주행과 같은 스택을
    # 그대로 띄워야 검증이 의미가 있다. 차이는 bench:=true (속도 0, 자동정지 없음)뿐.
    #
    # ★ 2026-08-09 사고: bench 가 컨트롤러의 clip(v,min,max) 에 기대고 있었는데
    #   속도 개편에서 그 클램프가 사라져 box 가 **시작 버튼도 없이 풀속도로 주행**했다.
    #   지금은 각 컨트롤러가 발행 직전에 속도를 0 으로 못박고(bench 파라미터),
    #   출발 게이트도 실주행과 동일하게 요구한다.
    MODE=${2:-$(get_yaml mode)}
    CTRL=""
    BENCH=1
    echo "[race.sh] ★ BENCH 모드 — 속도 0 강제. 바퀴는 돌지 않고 조향만 동작합니다."
    echo "[race.sh]   각 컨트롤러 기동 로그에 'BENCH 모드' 경고가 보이는지 확인하세요."
    echo "[race.sh]   ⚠ 그 줄이 없으면 즉시 Ctrl+C — 속도 강제가 안 걸린 것입니다."
    echo "[race.sh]   새 맵 정렬: 노트북 ./rviz.sh → 2D Pose Estimate → ./race.sh savepose"
    ;;
  q1|q2|q3|race)
    MODE=$CMD
    CTRL=${2:-}   # 두 번째 인자로 컨트롤러 오버라이드 (yaml 수정 불필요)
    case "$CTRL" in
      ""|auto|stanley|pure_pursuit|gap_follower|map) ;;
      * ) echo "알 수 없는 컨트롤러: $CTRL  (auto|stanley|pure_pursuit|gap_follower|map)"; exit 1 ;;
    esac ;;
  * ) echo "알 수 없는 명령: $CMD"; exit 1 ;;
esac

# ---------- 2-1. 중복 실행 가드 ----------
# 스택이 두 벌 뜨면 LiDAR TCP 포트(1클라이언트)·VESC 시리얼을 서로 뺏어
# urg_node 연결 실패, vesc_driver 세그폴트, 같은 이름 노드 충돌로 map_server
# lifecycle 활성화 실패가 연쇄로 터진다. 증상이 하드웨어 고장처럼 보여 디버깅이 샌다.
if [ -n "$(running_pids)" ]; then
    echo "[race.sh] ⚠ 이미 실행 중인 스택이 있습니다 (PID: $(running_pids | tr '\n' ' '))"
    echo "          두 벌이 뜨면 LiDAR/VESC 를 서로 뺏어 전부 실패합니다."
    read -r -p "          기존 스택을 정리하고 계속할까요? [Y/n] " ans
    case "$ans" in
        [nN]*) echo "[race.sh] 중단했습니다."; exit 1 ;;
        *) kill_stack || exit 1 ;;
    esac
fi

echo "============================================"
echo "  RoboRacer 2026  |  MODE = $MODE${CTRL:+  |  CONTROLLER = $CTRL}"
echo "  config: $RACE_SCRIPT"
if [ -n "${BENCH:-}" ]; then
echo "  ★ BENCH (박스 위) — 속도 0, 바퀴 안 돎, 자동정지 없음"
echo "     확인용: 다른 터미널에서  ./race.sh check   /  노트북에서  ./rviz.sh"
fi
echo "============================================"

if [ "$MODE" = "mapping" ]; then
    # SLAM 맵핑: 브링업+slam_toolbox는 백그라운드, 키보드 텔레옵은 포그라운드(tty 필요)
    ros2 launch race_stack mapping.launch.py &
    LAUNCH_PID=$!
    sleep 5
    echo ""
    echo ">>> 키보드로 천천히 트랙 1~2바퀴 돌면서 맵을 만드세요."
    echo ">>> ★ 이 창을 끄지 말고, 다른 터미널에서 저장하세요:"
    echo ">>>       ./race.sh savemap <맵이름>"
    echo ">>> ★ 저장이 끝난 뒤에 이 창에서 Ctrl+C  (먼저 끄면 /map이 사라져 저장 실패)"
    echo ""
    trap "kill $LAUNCH_PID 2>/dev/null" EXIT
    ros2 run race_stack keyboard_teleop --ros-args -p max_speed:=$(get_yaml speed.mapping_max)

elif [ "$MODE" = "mapping-joy" ]; then
    # SLAM 맵핑 + 조이스틱: 브링업+slam_toolbox 백그라운드, joy는 별도 실행
    ros2 launch race_stack mapping.launch.py &
    LAUNCH_PID=$!
    sleep 5
    echo ""
    echo ">>> 조이스틱으로 천천히 트랙 1~2바퀴 돌면서 맵을 만드세요."
    echo ">>> ★ 이 창을 끄지 말고, 다른 터미널에서 저장하세요:"
    echo ">>>       ./race.sh savemap <맵이름>"
    echo ">>> ★ 저장이 끝난 뒤에 이 창에서 Ctrl+C  (먼저 끄면 /map이 사라져 저장 실패)"
    echo ""

    # ★ bringup 의 joy_node + joy_teleop 이 /teleop 을 같이 쏘면
    #   (F710 기본 매핑: 축1×scale5) 손을 안 대도 차가 혼자 간다.
    #   커널 RT 휴지≈0 + trigger_invert 도 반쯤 스로틀로 오인될 수 있다.
    MAX_SPD=$(get_yaml speed.mapping_max)
    AX_STEER=$(get_yaml joystick.axis_steer)
    AX_SPEED=$(get_yaml joystick.axis_speed)
    BTN_REV=$(get_yaml joystick.btn_reverse)
    BTN_TOG=$(get_yaml joystick.btn_toggle)
    STEER_INV=$(get_yaml joystick.steer_invert 2>/dev/null || echo "False")
    SPEED_INV=$(get_yaml joystick.speed_invert 2>/dev/null || echo "False")
    TRIG_INV=$(get_yaml joystick.trigger_invert 2>/dev/null || echo "True")
    TRIG_DZ=$(get_yaml joystick.trigger_deadzone 2>/dev/null || echo "0.05")
    TRIG_EXPO=$(get_yaml joystick.trigger_expo 2>/dev/null || echo "1.0")
    STEER_DZ=$(get_yaml joystick.steer_deadzone 2>/dev/null || echo "0.08")
    JOY_DBG=$(get_yaml joystick.debug 2>/dev/null || echo "False")
    # 맵핑에서는 항상 안전 잠금 ON (yaml 의 arm_on_neutral:false 도 무시)
    ARM=True

    sleep 1
    pkill -9 -f '[j]oy_node' 2>/dev/null || true
    pkill -9 -f '[j]oy_teleop' 2>/dev/null || true
    pkill -9 -f '[j]s_joy_bridge' 2>/dev/null || true
    sleep 0.4

    # 리눅스 장치 경로: js1 이 있으면 우선 (F710 이 js1 로 잡히는 환경)
    JS_DEV=/dev/input/js1
    [ -e /dev/input/js1 ] || JS_DEV=/dev/input/js0
    [ -e "$JS_DEV" ] || { echo "[race.sh] 조이 장치 없음 ($JS_DEV)"; wait $LAUNCH_PID; exit 1; }

    JOY_PID=""
    BRIDGE="$SCRIPT_DIR/js_joy_bridge.py"
    if [ -f "$BRIDGE" ]; then
        echo "[race.sh] js_joy_bridge → $JS_DEV  (SDL joy_node 대신, 혼자 주행 방지)"
        python3 "$BRIDGE" "$JS_DEV" &#    ./race.sh savemap <이름>  맵핑 끝난 뒤 맵 저장
#    ./race.sh savepose        ★ 지금 AMCL 위치를 출발점으로 저장
#                              (savemap 이 맵 원점으로 이미 넣어두므로,
#                               출발 자리를 옮길 때만 쓰면 됩니다)
#                              RViz "2D Pose Estimate" 로 맵과 스캔을 맞춘 뒤 실행하면
#                              다음 실행부터 그 위치에서 시작 (맵-스캔 어긋남 해소)
    echo "[race.sh] 완료. race_script.yaml의 map_name을 '$NAME'으로 바꾸세요."
    echo ""
    echo ">>> 키보드로 천천히 트랙 1~2바퀴 돌면서 맵을 만드세요."
    echo ">>> ★ 이 창을 끄지 말고, 다른 터미널에서 저장하세요:"
    echo ">>>       ./race.sh savemap <맵이름>"
    echo ">>> ★ 저장이 끝난 뒤에 이 창에서 Ctrl+C  (먼저 끄면 /map이 사라져 저장 실패)"
    echo ""
    echo ""
    echo ">>> 조이스틱으로 천천히 트랙 1~2바퀴 돌면서 맵을 만드세요."
    echo ">>> ★ 이 창을 끄지 말고, 다른 터미널에서 저장하세요:"
    echo ">>>       ./race.sh savemap <맵이름>"
    echo ">>> ★ 저장이 끝난 뒤에 이 창에서 Ctrl+C  (먼저 끄면 /map이 사라져 저장 실패)"
    echo ""
        JOY_PID=$!
    else
        echo "[race.sh] js_joy_bridge 없음 — joy_node device_id=0"
        ros2 run joy joy_node --ros-args \
            -p device_id:=0 -p deadzone:=0.05 -p autorepeat_rate:=20.0 &
        JOY_PID=$!
    fi
    trap "kill $LAUNCH_PID $JOY_PID 2>/dev/null; pkill -9 -f '[j]oy_teleop' 2>/dev/null" EXIT

    ros2 run race_stack joystick_teleop --ros-args \
        -p max_speed:="$MAX_SPD" \
        -p axis_steer:="$AX_STEER" \
        -p axis_speed:="$AX_SPEED" \
        -p btn_reverse:="$BTN_REV" \
        -p btn_toggle:="$BTN_TOG" \
        -p steer_invert:="$STEER_INV" \
        -p speed_invert:="$SPEED_INV" \
        -p trigger_invert:="$TRIG_INV" \
        -p trigger_deadzone:="$TRIG_DZ" \
        -p trigger_expo:="$TRIG_EXPO" \
        -p steer_deadzone:="$STEER_DZ" \
        -p arm_on_neutral:="$ARM" \
        -p allow_toggle:=false \
        -p debug:="$JOY_DBG"
elif [ "$RECORD_BAG" = "1" ]; then
    # ★ exec 를 쓰면 안 된다 — 셸이 교체돼 trap 이 안 걸리고 bag 이 안 닫힌다
    #   (rosbag2 는 SIGINT 를 받아야 메타데이터를 쓴다. 그냥 죽이면 db3 만 남아
    #    읽을 수 없다).
    # box --bag 조합에서도 bench 안전장치가 빠지면 안 된다 (아래 bench 분기 참고).
    [ -n "${BENCH:-}" ] && bench_kill_joy &
    BAG_DIR="$SCRIPT_DIR/logs/bags/$(date +%Y%m%d_%H%M%S)_${MODE}_${CTRL:-auto}"
    mkdir -p "$(dirname "$BAG_DIR")"
    echo "[race.sh] ★ bag 녹화: $BAG_DIR"
    echo "[race.sh]   /scan 이 1081빔 33.5Hz 라 분당 약 9MB 쌓입니다."
    echo "[race.sh]   회피 판정용 /frenet/{offset,status,speed_scale,diag} 도 함께 녹화합니다."
    # ★ /scan_deskewed 도 반드시 넣는다. deskew 를 켜면 AMCL 이 보는 건 그쪽이라,
    #   /scan 만 녹화하면 **AMCL 의 실제 입력이 bag 에 없다** — 사후에 deskew 가
    #   도움이 됐는지 해가 됐는지 판정할 수가 없다 (2026-08-10 에 실제로 겪었다).
    #   토픽이 없으면 rosbag2 는 그냥 건너뛰므로 꺼져 있어도 무해하다.
    # ★ 회피 결과(/frenet/*)도 넣는다 (2026-08-15).
    #   driven_path.csv 에 selected_d / n_valid / speed_scale 이 남는다.
    #   /frenet/diag 는 bag 재생용.
    #   status: 0=TRACK 1=AVOID_STATIC 2=BLOCKED 3=FOLLOW 4=RECOVER 5=OVERTAKE
    #   토픽이 없으면 rosbag2 는 건너뛰므로 q1/q3 에서도 무해하다.
    ros2 bag record -o "$BAG_DIR" \
        /scan /scan_deskewed /odom /odom_imu /amcl_pose /drive /sensors/imu/raw \
        /commands/servo/position /race/enabled /diagnostics \
        /particle_cloud \
        /frenet/offset /frenet/status /frenet/speed_scale /frenet/diag \
        /loc/residual \
        /obstacle /overtake_zone >/dev/null 2>&1 &
    BAG_PID=$!
    # SIGINT 로 곱게 닫고, 안 죽으면 3초 뒤 강제 종료. 닫힌 bag 을 git 에 올린다.
    trap 'kill -INT $BAG_PID 2>/dev/null; ( sleep 3; kill -9 $BAG_PID 2>/dev/null ) & wait $BAG_PID 2>/dev/null; commit_logs' EXIT INT TERM

    ros2 launch race_stack race.launch.py mode:=$MODE \
        ${CTRL:+controller:=$CTRL} ${BENCH:+bench:=true} || true

    kill -INT $BAG_PID 2>/dev/null || true
    wait $BAG_PID 2>/dev/null || true
    echo "[race.sh] bag 저장 완료: $BAG_DIR  ($(du -sh "$BAG_DIR" 2>/dev/null | cut -f1))"
    commit_logs
elif [ -n "${BENCH:-}" ]; then
    # bench 는 컨트롤러 속도를 0 으로 못박지만(각 컨트롤러의 발행 직전), bringup 이
    # 함께 띄우는 joy_node+joy_teleop 은 /teleop 으로 따로 나가 그 경로를 우회한다.
    # 그래서 조이스틱 노드 자체를 죽인다 — box 는 조이스틱을 쓸 이유가 없다.
    ros2 launch race_stack race.launch.py mode:=$MODE \
        ${CTRL:+controller:=$CTRL} bench:=true &
    LAUNCH_PID=$!
    trap "kill $LAUNCH_PID 2>/dev/null; pkill -9 -f '[j]oy_teleop' 2>/dev/null" EXIT
    bench_kill_joy
    wait $LAUNCH_PID
else
    # exec 하면 셸이 교체돼 Ctrl+C 뒤 로그 커밋을 못 한다.
    trap commit_logs EXIT
    ros2 launch race_stack race.launch.py mode:=$MODE \
        ${CTRL:+controller:=$CTRL} || true
fi
