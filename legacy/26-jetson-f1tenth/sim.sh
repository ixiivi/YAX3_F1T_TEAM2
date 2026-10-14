#!/usr/bin/env bash
# ============================================================
#  sim.sh — 시뮬레이션 실행
#
#  [f1tenth gym 시뮬 — 노트북 기본 / 명시 지정]
#    ./sim.sh              컨트롤러 선택 메뉴 (race_stack 없으면 자동)
#    ./sim.sh hybrid       MAP+Stanley 하이브리드
#    ./sim.sh pure_pursuit 퓨어퍼슈트
#    ./sim.sh build        f1tenth_racing 빌드
#    ./sim.sh lap          랩타임 측정 (별도 터미널)
#    ./sim.sh stop         gym 시뮬 종료
#
#  [race_stack 시뮬 — 젯슨 전용]
#    ./sim.sh race         race_script.yaml 기준 실행
# ============================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
GYM_SH="$SCRIPT_DIR/f1TENTH-contest_full_stack/sim.sh"
WS=${RACE_WS:-$HOME/f1tenth_ws}

# gym 관련 커맨드는 무조건 gym으로
case "${1:-}" in
  map|hybrid|pure_pursuit|stanley|build|lap|stop|menu|race)
    exec "$GYM_SH" "$@" ;;
esac

# 기본(인자 없음) 또는 명시적 race 커맨드 처리
CMD=${1:-}

# race_stack 실제 인식 여부로 환경 판단
source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

if ! ros2 pkg prefix race_stack &>/dev/null; then
    # race_stack 없음 → gym으로 자동 전환 (노트북/미빌드 환경)
    exec "$GYM_SH" "$@"
fi

if [ -z "$CMD" ]; then
    # race_stack 있어도 인자 없으면 gym 메뉴로 (옵션 5 = Race Stack)
    exec "$GYM_SH"
fi

# ── race_stack 시뮬 (젯슨) ──────────────────────────────────────────────
export RACE_WS="$WS"
export RACE_SCRIPT="$SCRIPT_DIR/race_script.yaml"
SIM_WS="$SCRIPT_DIR/f1TENTH-contest_full_stack/sim_ws"

if [ ! -f "$SIM_WS/install/setup.bash" ]; then
    echo "✗ sim_ws 가 빌드되지 않았습니다."
    echo "  먼저 ./setup_laptop.sh 를 실행하세요."
    exit 1
fi

source "$SIM_WS/install/setup.bash"

MAP=$(python3 -c "import yaml; print(yaml.safe_load(open('$RACE_SCRIPT'))['map_name'])" 2>/dev/null || echo "?")
MODE=$(python3 -c "import yaml; print(yaml.safe_load(open('$RACE_SCRIPT'))['mode'])" 2>/dev/null || echo "?")
CTRL=$(python3 -c "import yaml; print(yaml.safe_load(open('$RACE_SCRIPT'))['controller'])" 2>/dev/null || echo "?")

echo "============================================"
echo "  RoboRacer 2026  |  SIMULATION (race_stack)"
echo "  mode=$MODE  controller=$CTRL  map=$MAP"
echo "  config: $RACE_SCRIPT"
echo "============================================"

ros2 launch race_stack sim_race.launch.py &
_LAUNCH_PID=$!

_cleanup() {
    kill -SIGINT "$_LAUNCH_PID" 2>/dev/null
    sleep 0.8
    # rviz2 는 SIGINT 를 무시하는 경우가 있어 SIGKILL 로 강제 종료
    pkill -SIGKILL -x rviz2 2>/dev/null || true
    wait "$_LAUNCH_PID" 2>/dev/null
}
trap '_cleanup; exit 0' SIGINT SIGTERM SIGHUP

wait "$_LAUNCH_PID" || true
# 정상 종료 시에도 rviz2 잔존 프로세스 정리
pkill -SIGKILL -x rviz2 2>/dev/null || true
