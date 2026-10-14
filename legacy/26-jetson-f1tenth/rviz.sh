#!/usr/bin/env bash
# ============================================================
#  rviz.sh — ★ 노트북에서 실행 ★  젯슨 주행을 실시간 RViz로 관찰
#
#  젯슨에 SSH로 들어가 ./race.sh 를 돌리는 것과 별개로,
#  노트북 로컬 터미널(SSH 아님)에서 이 스크립트를 실행하면
#  맵 + 레이싱라인 + 차 위치/궤적 + LiDAR + 목표점이 뜬다.
#
#  전제:
#    - 노트북에 ROS 2 + rviz2 설치 (Humble 또는 Jazzy)
#      Humble: sudo apt install ros-humble-desktop  또는  ros-humble-rviz2
#      Jazzy:  sudo apt install ros-jazzy-desktop   또는  ros-jazzy-rviz2
#      ※ Jazzy는 RViz/원격 모니터링 용. 실차·빌드 기준은 Humble.
#    - 노트북과 젯슨이 같은 네트워크(192.168.50.x)
#    - ROS_DOMAIN_ID 는 **이 스크립트가 차량 파일에서 정한다** (아래 인자 참고).
#      ~/.bashrc 에 ROS_DOMAIN_ID 를 박아뒀다면 지우세요 — 여기서 덮어씁니다.
#
#  사용:
#    ./rviz.sh 77       2호차에 붙는다 (젯슨 IP 끝자리 — 권장)
#    ./rviz.sh 57       1호차
#    ./rviz.sh car2     차 이름도 됨
#    ./rviz.sh          .car 가 있으면 그 차(젯슨에서 실행할 때),
#                       없으면 race_script.yaml 공용값
#
#  ★ 두 차가 같은 공유기에서 동시에 돌 수 있다. 차마다 ROS_DOMAIN_ID 가 달라
#    (1호차 57 / 2호차 77) DDS 레벨에서 완전히 격리되므로, 노트북 두 대가
#    각각 다른 차에 붙어도 /scan·/map·/tf 가 섞이지 않는다.
#    도메인 값은 config/vehicle_car*.yaml 의 network.domain_id 가 원본이다.
#
#  ▶ 젯슨에서 직접 띄우기 (모니터 연결 시) — 초기 위치 정렬에 편하다
#      SSH 로 들어와서 그냥 ./rviz.sh  (DISPLAY 는 자동으로 잡는다)
#      터미널1:  ./race.sh box       스택 기동 (바퀴 안 돌아감)
#      터미널2:  ./rviz.sh           젯슨 화면에 RViz → 2D Pose Estimate 로 정렬
#      터미널3:  ./race.sh savepose  맞춘 위치 저장
#      ※ 주행 중 관찰용이면 노트북 로컬 실행이 낫다 (젯슨 CPU 를 안 뺏김)
# ============================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$DIR/config/race.rviz"

# 설치된 ROS 2 배포판 자동 선택 (노트북 Jazzy / 팀원·젯슨 Humble)
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "[ERROR] ROS 2 Humble 또는 Jazzy를 찾을 수 없습니다."
    exit 1
fi

echo "[INFO] ROS_DISTRO=${ROS_DISTRO}"
case "${ROS_DISTRO}" in
    humble)
        echo "[INFO] Humble 환경: RViz 및 원격 모니터링"
        ;;
    jazzy)
        echo "[WARNING] Jazzy 환경: RViz 및 원격 모니터링 용도로만 사용"
        echo "          실차 실행·빌드 기준은 Humble입니다."
        ;;
    *)
        echo "[ERROR] 지원하지 않는 ROS 배포판: ${ROS_DISTRO}"
        exit 1
        ;;
esac

# ---------- 어느 차에 붙을지 ----------
# 두 차가 같은 공유기에서 동시에 돌 수 있으므로, 붙을 대상을 골라야 한다.
# 젯슨은 .car 로 자동이지만 노트북엔 .car 도 overlay 도 없다 → 인자로 받는다.
#   ./rviz.sh 77     젯슨 IP 끝자리 (권장 — 외우기 쉽다)
#   ./rviz.sh car2   차 이름도 됨
#   ./rviz.sh        .car 가 있으면 그걸 쓰고, 없으면 race_script.yaml 공용값
if [ -n "${1:-}" ]; then
    case "$1" in
        car[0-9]*) RACE_CAR="$1" ;;
        *[!0-9]*)  echo "[rviz.sh] ✗ 알 수 없는 인자: $1  (예: ./rviz.sh 77 또는 ./rviz.sh car2)"; exit 1 ;;
        *)
            # 숫자 → 그 IP 끝자리를 peers 에 가진 vehicle 파일을 찾는다.
            # 번호를 두 군데(여기 + yaml) 적어두면 반드시 어긋나므로 yaml 만 본다.
            RACE_CAR=$(python3 - "$DIR/config" "$1" <<'PY'
import os, sys, glob, yaml
cfgdir, octet = sys.argv[1], sys.argv[2]
for p in sorted(glob.glob(os.path.join(cfgdir, 'vehicle_*.yaml'))):
    try:
        net = (yaml.safe_load(open(p)) or {}).get('network', {}) or {}
    except Exception:
        continue
    if any(str(ip).split('.')[-1] == octet for ip in (net.get('peers') or [])):
        print(os.path.basename(p)[len('vehicle_'):-len('.yaml')]); break
PY
)
            if [ -z "$RACE_CAR" ]; then
                echo "[rviz.sh] ✗ 끝자리 $1 을 가진 차량 파일이 없습니다."
                echo "          있는 차: $(ls "$DIR"/config/vehicle_*.yaml 2>/dev/null \
                    | sed 's#.*/vehicle_##; s#\.yaml##' | tr '\n' ' ')"
                exit 1
            fi ;;
    esac
    export RACE_CAR
    echo "[rviz.sh] 대상 차량: $RACE_CAR  (config/vehicle_${RACE_CAR}.yaml)"
elif [ -f "$DIR/.car" ]; then
    RACE_CAR=$(tr -d '[:space:]' < "$DIR/.car"); export RACE_CAR
    echo "[rviz.sh] 대상 차량: $RACE_CAR  (.car)"
fi

# DDS 설정 — 젯슨의 race.sh 와 같은 파일을 읽어 양쪽 미들웨어를 일치시킨다.
# (RMW 가 한쪽만 다르면 토픽이 하나도 안 보인다)
# ★ ROS_DOMAIN_ID 도 여기서 정해진다 — RACE_CAR 가 가리키는 vehicle 파일 기준.
export RACE_SCRIPT="$DIR/race_script.yaml"
[ -f "$DIR/config/dds_env.sh" ] && source "$DIR/config/dds_env.sh"

# SSH 세션이면 RViz 창은 이 명령을 받은 기기(젯슨) 화면에 뜬다.
# 둘 다 정상적인 쓰임새라 막지 않고, 어느 쪽인지만 알려준다:
#   - 노트북 로컬 실행  → 노트북 화면에서 관찰 (주행 중 권장. 젯슨 CPU 안 씀)
#   - 젯슨에 SSH + 모니터 → 젯슨 화면에 뜸. 2D Pose Estimate 후 곧바로
#                           ./race.sh savepose 할 수 있어 초기 정렬에 편하다.
if [ -n "${SSH_CONNECTION:-}" ]; then
    # SSH 세션은 보통 DISPLAY 가 비어 있다. 로컬 X 세션이 있으면 그걸 잡아준다.
    if [ -z "${DISPLAY:-}" ]; then
        for sock in /tmp/.X11-unix/X*; do
            [ -e "$sock" ] || continue
            export DISPLAY=":${sock##*/X}"
            break
        done
    fi
    if [ -z "${DISPLAY:-}" ]; then
        echo "✗ 젯슨에 그래픽 세션이 없습니다 (DISPLAY 미설정, /tmp/.X11-unix 비어 있음)."
        echo "  젯슨에 모니터를 연결했는지 확인하거나, 노트북 로컬에서 ./rviz.sh 를 쓰세요."
        exit 1
    fi
    echo "ℹ SSH 세션 — RViz 창은 젯슨 화면(DISPLAY=$DISPLAY)에 뜹니다."
    echo "  초기 정렬 흐름:  RViz 에서 2D Pose Estimate  →  다른 터미널에서 ./race.sh savepose"
    echo "  ※ 주행 중에는 젯슨 CPU 를 나눠 쓰므로, 관찰용이면 노트북 로컬 실행을 권장합니다."
    echo ""
fi

if ! command -v rviz2 >/dev/null 2>&1; then
    echo "✗ rviz2 가 없습니다.  sudo apt install ros-${ROS_DISTRO}-rviz2"
    exit 1
fi
[ -f "$CFG" ] || { echo "✗ RViz 설정 없음: $CFG (repo 최신화 필요: git pull)"; exit 1; }

echo "============================================"
echo "  RViz 실시간 관찰  |  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "  config: $CFG"
echo "  ※ 젯슨에서 ./race.sh 주행이 돌고 있어야 토픽이 보입니다."
echo "============================================"
# 젯슨 토픽이 잡히는지 살짝 확인 (안 잡혀도 RViz 는 그냥 실행)
# ★ timeout 필수 — `ros2 topic list` 는 데몬이 콜드하거나 도메인을 막 바꿨을 때
#   수십 초 블록된다. 이건 안내 문구를 고르려는 것뿐이라 못 기다린다
#   (2026-08-10: 여기서 멈춰 Ctrl+C 를 눌러야 RViz 가 떴다).
# ★ 매핑(mapping/mapping-joy) 중에는 /waypoints_path 가 원래 없다 — 주행 모드
#   전용이다. 그때는 이 경고가 떠도 정상이니 문구에 같이 적는다.
if ! timeout 5 ros2 topic list 2>/dev/null | grep -q "/waypoints_path"; then
    echo "  ⚠ 아직 /waypoints_path 가 안 보입니다."
    echo "    (매핑 중이라면 정상입니다 — 이 토픽은 주행 모드에서만 나옵니다)"
    echo "    - 젯슨에서 주행(race.sh q1/q2/q3/race)이 실행 중인지"
    echo "    - 노트북·젯슨 네트워크/ROS_DOMAIN_ID 가 같은지 확인하세요."
    echo "    (그래도 RViz 는 띄웁니다 — 주행 시작하면 자동으로 나타남)"
fi

exec rviz2 -d "$CFG"
