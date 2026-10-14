#!/usr/bin/env bash
# ============================================================
#  sim.sh — F1TENTH 시뮬 딸깍 스크립트
#
#  사용법:
#    ./sim.sh                  기본 (MAP 단독 컨트롤러)
#    ./sim.sh map              MAP 단독 컨트롤러 (L1 + Pacejka, ForzaETH 방식)
#    ./sim.sh hybrid           MAP+Stanley 하이브리드 컨트롤러
#    ./sim.sh pure_pursuit     퓨어퍼슈트
#    ./sim.sh menu             컨트롤러 선택 메뉴
#    ./sim.sh build            f1tenth_racing 빌드
#    ./sim.sh lap              랩타임 측정 (별도 터미널에서 실행)
#    ./sim.sh stop             시뮬 + 컨트롤러 전부 종료
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_WS="$SCRIPT_DIR/sim_ws"
F1WS="$SCRIPT_DIR/f1tenth_ws"

source /opt/ros/humble/setup.bash

CMD=${1:-}

# ── 프로세스 탐지 ──────────────────────────────────────────────────────────
SIM_PROCS="[g]ym_bridge|[r]acing_node|[h]ybrid_controller_node|[p]ure_pursuit_node|[s]tanley_controller_node|[l]ap_timer_node"
running_pids() { pgrep -f "$SIM_PROCS" 2>/dev/null || true; }

kill_stack() {
    local pids; pids=$(running_pids)
    [ -z "$pids" ] && { echo "[sim.sh] 실행 중인 프로세스 없음"; return 0; }
    echo "[sim.sh] 종료 중 (PID: $(echo "$pids" | tr '\n' ' '))"
    kill $pids 2>/dev/null || true
    sleep 2
    pids=$(running_pids)
    [ -n "$pids" ] && { kill -9 $pids 2>/dev/null || true; sleep 1; }
    if [ -n "$(running_pids)" ]; then
        echo "[sim.sh] ⚠ 아직 남은 프로세스 있음 — 수동 확인: ps -ef | grep gym_bridge"
        return 1
    fi
    echo "[sim.sh] 정리 완료"
}

# ── f1tenth_racing 빌드 (symlink-install) ──────────────────────────────────
# --symlink-install: Python 소스를 복사하지 않고 심링크로 연결
# → 소스 파일을 수정해도 재빌드 없이 즉시 반영됨
build_f1ws() {
    echo "[sim.sh] f1tenth_racing 빌드 중 (symlink-install)..."
    cd "$F1WS"
    colcon build --packages-select f1tenth_racing --symlink-install \
        --event-handlers console_cohesion+
    echo "[sim.sh] 빌드 완료 — 이후 Python 파라미터 수정은 재빌드 불필요"
}

# ── 단독 명령 ─────────────────────────────────────────────────────────────
case "$CMD" in
  stop)
    kill_stack; exit $? ;;

  build)
    echo "[sim.sh] sim_ws 빌드 중..."
    cd "$SIM_WS"
    colcon build
    build_f1ws
    exit 0 ;;

  lap)
    source "$F1WS/install/setup.bash"
    echo "[sim.sh] 랩타임 측정 시작 (Ctrl+C 로 종료)"
    exec ros2 run f1tenth_racing lap_timer_node ;;
esac

# ── sim.yaml 맵 경로 자동 패치 ────────────────────────────────────────────
# race_script.yaml의 map_name을 읽어 sim.yaml의 map_path를 업데이트한다.
# 맵 탐색 순서: sim_ws/maps → src/race_stack/maps
patch_map() {
    local race_script="$SCRIPT_DIR/../race_script.yaml"
    local sim_yaml_src="$SIM_WS/src/f1tenth_gym_ros/config/sim.yaml"
    local sim_yaml_install="$SIM_WS/install/f1tenth_gym_ros/share/f1tenth_gym_ros/config/sim.yaml"

    if [ ! -f "$race_script" ]; then
        echo "[sim.sh] ⚠ race_script.yaml 없음 — 기존 sim.yaml 사용"
        return 0
    fi

    python3 - <<PYEOF
import yaml, sys, os, math, re, csv

race_script    = "$race_script"
sim_yaml_src   = "$sim_yaml_src"
sim_yaml_install = "$sim_yaml_install"
sim_maps_dir   = "$SIM_WS/src/f1tenth_gym_ros/maps"
pp_node_path   = "$F1WS/src/f1tenth_racing/f1tenth_racing/pure_pursuit_node.py"

map_search_dirs = [sim_maps_dir, "$SCRIPT_DIR/../src/race_stack/maps"]
wp_search_dirs  = [sim_maps_dir, "$SCRIPT_DIR/../src/race_stack/waypoints"]

map_name = yaml.safe_load(open(race_script))['map_name']

# ── 1. 맵 파일 탐색 ─────────────────────────────────────────────────
map_path, map_ext = None, None
for d in map_search_dirs:
    for ext in ['.png', '.pgm']:
        if os.path.exists(os.path.join(d, map_name) + ext):
            map_path, map_ext = os.path.join(d, map_name), ext
            break
    if map_path:
        break

if not map_path:
    print(f"[sim.sh] ✗ 맵 '{map_name}' 을 찾을 수 없습니다. 탐색: {map_search_dirs}")
    sys.exit(1)

# ── 2. 웨이포인트 파일 탐색 + sim maps에 심링크 ──────────────────────
sim_wp = os.path.join(sim_maps_dir, f'{map_name}_waypoints.csv')
wp_src = None

if not os.path.exists(sim_wp):
    # race_stack waypoints 에서 탐색 (half_clean_wp.csv 등)
    for d in wp_search_dirs:
        for candidate in [f'{map_name}_waypoints.csv', f'{map_name}_wp.csv']:
            p = os.path.join(d, candidate)
            if os.path.exists(p):
                wp_src = p
                break
        if wp_src:
            break
    if wp_src:
        os.symlink(wp_src, sim_wp)
        print(f"[sim.sh] 웨이포인트 링크: {os.path.basename(wp_src)} → sim maps/")
    else:
        print(f"[sim.sh] ⚠ 웨이포인트 파일 없음 — setup_sim.py 로 생성하세요")

# ── 3. 스폰 위치: 웨이포인트 첫 점 + 진행 방향 ──────────────────────
sx, sy, stheta = 0.0, 0.0, 0.0
if os.path.exists(sim_wp):
    rows = list(csv.DictReader(open(sim_wp)))
    if len(rows) >= 2:
        x0, y0 = float(rows[0]['x']), float(rows[0]['y'])
        x1, y1 = float(rows[1]['x']), float(rows[1]['y'])
        sx, sy  = x0, y0
        stheta  = math.atan2(y1 - y0, x1 - x0)

# ── 4. sim.yaml 패치 ─────────────────────────────────────────────────
for path in [sim_yaml_src, sim_yaml_install]:
    if not os.path.exists(path):
        continue
    cfg = yaml.safe_load(open(path))
    p   = cfg['bridge']['ros__parameters']
    p['map_path']    = map_path
    p['map_img_ext'] = map_ext
    p['sx']          = round(sx, 4)
    p['sy']          = round(sy, 4)
    p['stheta']      = round(stheta, 4)
    with open(path, 'w') as f:
        yaml.dump(cfg, f, allow_unicode=True, default_flow_style=False)

# ── 5. pure_pursuit_node.py WAYPOINTS_MAP 업데이트 ──────────────────
if os.path.exists(pp_node_path):
    src = open(pp_node_path).read()
    new = re.sub(r"^WAYPOINTS_MAP\s*=\s*'[^']*'",
                 f"WAYPOINTS_MAP = '{map_name}'", src, flags=re.MULTILINE)
    if new != src:
        open(pp_node_path, 'w').write(new)

print(f"[sim.sh] 맵={map_name}  스폰=({sx:.2f},{sy:.2f}) θ={math.degrees(stheta):.1f}°  ext={map_ext}")
PYEOF
}

patch_map

# ── 중복 실행 가드 ─────────────────────────────────────────────────────────
if [ -n "$(running_pids)" ]; then
    echo "[sim.sh] ⚠ 이미 실행 중인 프로세스가 있습니다."
    read -r -p "          종료 후 재시작할까요? [Y/n] " ans
    case "$ans" in
        [nN]*) echo "[sim.sh] 중단"; exit 1 ;;
        *) kill_stack ;;
    esac
fi

# ── 컨트롤러 선택 메뉴 (인자 없을 때) ────────────────────────────────────
if [ -z "$CMD" ]; then
    echo ""
    echo "  컨트롤러를 선택하세요:"
    echo "  1) MAP        — L1 가이던스 + Pacejka 슬립 보정"
    echo "  2) Hybrid     — MAP (코너) + Stanley (직선)"
    echo "  3) Stanley    — race_script.yaml 파라미터 그대로"
    echo "  4) Pure Pursuit"
    echo "  5) Race Stack — 실차 race_script.yaml 완전 동일 (sim_race.launch.py)"
    echo ""
    read -r -p "  선택 [1/2/3/4/5, 기본=1]: " _sel
    case "${_sel:-1}" in
        2) CMD=hybrid ;;
        3) CMD=stanley ;;
        4) CMD=pure_pursuit ;;
        5) CMD=race ;;
        *) CMD=map ;;
    esac
fi

# ── Race Stack 전체 시뮬 (gym_bridge 포함 sim_race.launch.py 가 모두 관리) ──
if [ "$CMD" = "race" ]; then
    _RACE_WS="${RACE_WS:-$HOME/f1tenth_ws}"
    if [ ! -f "$_RACE_WS/install/setup.bash" ]; then
        echo "[sim.sh] ✗ race_stack 워크스페이스($_RACE_WS) 가 빌드되지 않았습니다"
        echo "          RACE_WS 환경변수를 설정하거나 f1tenth_ws 를 빌드하세요"
        exit 1
    fi
    source "$_RACE_WS/install/setup.bash"
    if ! ros2 pkg prefix race_stack &>/dev/null 2>&1; then
        echo "[sim.sh] ✗ race_stack 패키지를 찾을 수 없습니다 ($_RACE_WS)"
        exit 1
    fi
    export RACE_WS="$_RACE_WS"
    export RACE_SCRIPT="$SCRIPT_DIR/../race_script.yaml"
    echo "============================================"
    echo "  F1TENTH Sim  |  Race Stack"
    echo "  $(python3 -c "import yaml; c=yaml.safe_load(open('$RACE_SCRIPT')); print(f\"map={c['map_name']}  mode={c['mode']}  ctrl={c['controller']}\")" 2>/dev/null || true)"
    echo "  config: $RACE_SCRIPT"
    echo "============================================"
    source "$SIM_WS/install/setup.bash"
    exec ros2 launch race_stack sim_race.launch.py
fi

# ── 시뮬레이터 백그라운드 실행 ────────────────────────────────────────────
echo "============================================"
echo "  F1TENTH Sim  |  $CMD"
echo "============================================"

source "$SIM_WS/install/setup.bash"
echo "[sim.sh] 시뮬레이터 시작..."
ros2 launch f1tenth_gym_ros gym_bridge_launch.py &
SIM_PID=$!

cleanup() {
    echo ""
    echo "[sim.sh] 종료 — 시뮬레이터 정리 중..."
    kill "$SIM_PID" 2>/dev/null || true
    wait "$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 3

# ── 컨트롤러 포그라운드 실행 ──────────────────────────────────────────────
# 미빌드 또는 symlink-install 미적용 시 자동 빌드
# symlink 여부: build/f1tenth_racing/f1tenth_racing 이 src를 가리키는 심링크인지 확인
_SYMLINK_CHECK="$F1WS/build/f1tenth_racing/f1tenth_racing"
if [ ! -f "$F1WS/install/setup.bash" ]; then
    echo "[sim.sh] f1tenth_ws 미빌드 → 자동 빌드 시작..."
    build_f1ws
elif [ ! -L "$_SYMLINK_CHECK" ]; then
    echo "[sim.sh] symlink-install 미적용 → 재빌드 중..."
    build_f1ws
fi
source "$F1WS/install/setup.bash"

case "$CMD" in
  map)
    echo "[sim.sh] MAP 컨트롤러 (L1 + Pacejka) 실행"
    ros2 run f1tenth_racing map_controller_node ;;

  hybrid)
    echo "[sim.sh] 하이브리드 컨트롤러 (MAP + Stanley) 실행"
    ros2 run f1tenth_racing hybrid_controller_node ;;

  stanley)
    echo "[sim.sh] Stanley 컨트롤러 (race_script.yaml 파라미터) 실행"
    ros2 run f1tenth_racing stanley_controller_node ;;

  pure_pursuit)
    echo "[sim.sh] 퓨어퍼슈트 실행"
    ros2 run f1tenth_racing pure_pursuit_node ;;

  menu)
    echo "[sim.sh] 컨트롤러를 선택하세요..."
    ros2 run f1tenth_racing racing_node ;;

  *)
    echo "[sim.sh] 알 수 없는 명령: $CMD"
    echo "         사용법: ./sim.sh [hybrid|pure_pursuit|build|lap|stop]"
    exit 1 ;;
esac
