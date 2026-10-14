#!/usr/bin/env bash
# ============================================================
#  bat.sh — 배터리 잔량 확인 (VESC 텔레메트리 voltage_input)
#
#  사용:
#    ./bat.sh              전압 + 대략 잔량 출력
#                          (스택 미실행 시 vesc_driver 를 잠깐 띄웠다 닫음)
#    ./bat.sh --inline     race.sh 기동 시 한 줄 출력용
#                          (실패해도 exit 0 — race.sh 를 막지 않는다)
#    ./bat.sh --volts      숫자만 출력 (스크립트용, 실패 시 exit 1)
#
#  ★ 잔량 %는 무부하 전압 기준의 근사치다. 주행 중에는 전압이 처져
#    실제보다 낮게 나온다 — 판단은 정지 상태에서 할 것.
# ============================================================
MODE_ARG=${1:-}

WS=${RACE_WS:-$HOME/f1tenth_ws}
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

# 단독 실행(환경 미소싱)도 지원 — race.sh 경유면 이미 소싱돼 있어 무해하다
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f "$WS/install/setup.bash"   ] && source "$WS/install/setup.bash"   2>/dev/null || true

# ※ dds_env.sh 는 일부러 소싱하지 않는다. 단독 실행이면 드라이버도 구독자도
#   같은 셸의 RMW 를 쓰므로 어차피 일치하고, race.sh 경유면 이미 소싱돼 있다.
#   여기서 또 소싱하면 `ros2 daemon stop` 과 Cyclone 탐색이 끼어들어 느려진다.

# VESC 심링크 (재부팅마다 날아간다 — race.sh 와 같은 처리)
if [ ! -e /dev/sensors/vesc ]; then
    sudo mkdir -p /dev/sensors 2>/dev/null || true
    sudo ln -sf /dev/ttyACM0 /dev/sensors/vesc 2>/dev/null || true
fi

# ★ `ros2 topic echo` 를 쓰지 않는다 (2026-08-04).
#   ros2 CLI 는 조회에 **ros2 데몬**을 거치는데, 데몬은 RMW 별로 따로 뜨고 한 번
#   엉키면 발행자가 멀쩡히 살아 있어도 토픽을 못 찾는다. 실제로 드라이버가
#   `Connected to VESC with firmware version 6.6` 까지 찍는데도 echo 가 아무것도
#   못 받아 "전압을 읽지 못했습니다" 가 떴다. 같은 환경에서 아래 파이썬 구독자는
#   즉시 15.50V 를 읽었다 — CLI 경로만의 문제였다.
#   노드로 직접 구독하면 데몬을 안 거치고 CLI 기동 비용도 없다.
read_volt () {
    timeout "${1:-5}" python3 -c '
import sys, time, rclpy
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped
rclpy.init()
n = Node("bat_probe")
got = []
n.create_subscription(VescStateStamped, "/sensors/core",
                      lambda m: got.append(m.state.voltage_input), 10)
t0 = time.time()
while time.time() - t0 < float(sys.argv[1]) and not got:
    rclpy.spin_once(n, timeout_sec=0.2)
if got:
    print(f"{got[-1]:.3f}")
rclpy.shutdown()
' "${1:-5}" 2>/dev/null
}

VOLT=""
VESC_PID=""

# ★ ros2 run 은 래퍼다 — 실제 vesc_driver_node 는 그 자식으로 뜬다.
#   $! (래퍼 PID) 만 kill 하면 노드가 고아로 살아남는다. 실측으로 확인했다.
#   자손까지 훑어서 정리한다.
kill_tree () {
    local pid=$1 sig=$2 kid
    for kid in $(pgrep -P "$pid" 2>/dev/null); do kill_tree "$kid" "$sig"; done
    kill "$sig" "$pid" 2>/dev/null || true
}

# 스택 감지에 `ros2 topic info` 를 쓰면 DDS 탐색에만 1.7초가 든다 (실측).
# 발행자는 곧 vesc_driver_node 프로세스이므로 pgrep 으로 즉시 판별한다.
if pgrep -f '[v]esc_driver_node' >/dev/null 2>&1; then
    # 스택이 이미 돌고 있다 — 토픽을 바로 읽는다 (빠름, 시리얼 충돌 없음)
    VOLT=$(read_volt 5)
else
    # ★ 임시 기동한 드라이버는 반드시 회수해야 한다.
    #   race.sh 의 중복 실행 가드(2-1)가 vesc_driver_node 를 패턴으로 찾기 때문에,
    #   여기서 살아남으면 "이미 실행 중인 스택" 오탐이 나서 기동 때마다 확인
    #   프롬프트가 뜬다. 게다가 VESC 시리얼을 쥔 채로 남으면 진짜 스택이 못 연다.
    # 기동 전 PID 목록 — 회수 후 "우리가 띄운 것만" 골라내는 대조군
    PRE_PIDS=" $(pgrep -f '[v]esc_driver_node' 2>/dev/null | tr '\n' ' ') "

    ros2 run vesc_driver vesc_driver_node --ros-args \
        --params-file "$SCRIPT_DIR/config/vesc.yaml" \
        > /dev/null 2>&1 &
    VESC_PID=$!

    # ★ 여기서 sleep 하지 않는다. `topic echo --once` 는 발행자가 나타날 때까지
    #   기다리므로, 드라이버 기동과 DDS 탐색(~2초)이 병렬로 진행된다.
    #   sleep 2 를 넣었을 땐 직렬로 쌓여 7.3초가 걸렸다 (실측).
    VOLT=$(read_volt 6)
    if [ -z "$VOLT" ] && kill -0 "$VESC_PID" 2>/dev/null; then
        VOLT=$(read_volt 3)                        # 탐색이 늦은 경우 한 번 더
    fi

    if [ -n "$VESC_PID" ]; then
        kill_tree "$VESC_PID" -TERM
        # 실제로 사라질 때까지 확인 — 시리얼 포트를 확실히 놓게 한다
        for _ in $(seq 20); do
            kill -0 "$VESC_PID" 2>/dev/null || break
            sleep 0.1
        done
        kill_tree "$VESC_PID" -KILL
        wait "$VESC_PID" 2>/dev/null || true

        # 최후 보루 — 트리 추적이 놓친 게 있으면 여기서 잡는다.
        # 우리가 띄우기 전부터 있던 PID 는 건드리지 않는다 (남의 스택 보호).
        for pid in $(pgrep -f '[v]esc_driver_node' 2>/dev/null); do
            case "$PRE_PIDS" in
                *" $pid "*) ;;                       # 원래 있던 것 — 그대로 둔다
                *) kill -KILL "$pid" 2>/dev/null || true ;;
            esac
        done
    fi
fi

if [ -z "$VOLT" ]; then
    case "$MODE_ARG" in
        --volts)  exit 1 ;;
        --inline) echo "[bat] ⚠ 전압을 읽지 못했습니다 (VESC 연결 확인 — ./bat.sh)"; exit 0 ;;
        *)        echo "[bat] ⚠ 전압을 읽지 못했습니다."
                  echo "      확인: ls -l /dev/sensors/vesc  |  다른 스택이 시리얼을 쥐고 있는지 ./race.sh check"
                  exit 1 ;;
    esac
fi

if [ "$MODE_ARG" = "--volts" ]; then
    echo "$VOLT"
    exit 0
fi

# ── 잔량 환산 ────────────────────────────────────────────────
# 4S LiPo 무부하 방전곡선의 구간 선형보간. 앵커는 동료가 충전기로 실측한 값:
#   3.78 V/cell (15.12V) = 29%,  3.90 V/cell (15.6V) ≈ 60%,  3.70 V/cell (14.8V) ≈ 20%
# LiPo 는 3.7~3.9V 구간에서 곡선이 가파르다 — 0.1V 차이가 잔량 30%를 가른다.
# 그래서 전압만 보면 "많이 남은 것 같은데 갑자기 죽는" 일이 생긴다.
PCT=$(awk -v v="$VOLT" 'BEGIN{
    c = v / 4.0;                       # 4S 가정 (이 차량 고정)
    n = split("3.30 0,3.40 2,3.50 5,3.60 11,3.65 15,3.70 20,3.75 25,3.78 29,3.80 33,3.85 45,3.90 60,3.95 65,4.00 70,4.05 78,4.10 85,4.15 92,4.20 100", a, ",");
    split(a[1], lo, " ");
    if (c <= lo[1]) { print 0; exit }
    for (i = 2; i <= n; i++) {
        split(a[i-1], p, " "); split(a[i], q, " ");
        if (c <= q[1]) {
            printf "%d", p[2] + (q[2] - p[2]) * (c - p[1]) / (q[1] - p[1]);
            exit
        }
    }
    print 100
}')

V=$(printf "%.1f" "$VOLT" 2>/dev/null || echo "$VOLT")
CELL=$(awk -v v="$VOLT" 'BEGIN{printf "%.2f", v/4.0}')

# ── 경고 단계 (2026-08-04 하향) ───────────────────────────────
#   예전엔 주의선이 15.6V(≈60%)라 절반이나 남았는데도 "충전 권장"이 떴다.
#   실제로 52%에서 경고가 뜨는 건 과하다 — 판단을 흐리기만 한다.
#   위 방전곡선 앵커 기준으로 단계를 잔량 %에 맞춰 다시 잡는다:
#       14.8V = 3.70 V/cell = 20%   ← 주의
#       14.4V = 3.60 V/cell = 11%   ← 위험
#   ※ 무부하 기준이다. 주행 중에는 가속마다 전압이 처지므로, 정지 상태에서
#     14.8V 면 부하 시엔 그보다 한참 아래로 떨어진다. 20%는 "이번 주행은
#     마치되 다음 주행 전에 충전" 정도의 선이지 여유가 많다는 뜻이 아니다.
BAT_WARN=14.8      # 20%
BAT_CRIT=14.4      # 11%
if   awk "BEGIN{exit !($VOLT >= $BAT_WARN)}"; then COLOR="\033[32m"; NOTE="충분"
elif awk "BEGIN{exit !($VOLT >= $BAT_CRIT)}"; then COLOR="\033[33m"; NOTE="충전 권장"
else                                               COLOR="\033[31m"; NOTE="★ 즉시 충전"
fi
RESET="\033[0m"

# 터미널이 아니면(파일 리다이렉트·파이프) 색을 끈다 — 로그에 ESC[32m 이 남지 않도록
if [ ! -t 1 ]; then COLOR=""; RESET=""; fi

echo -e "[bat] 배터리 ${COLOR}${V} V${RESET} (${CELL} V/cell) — 약 ${COLOR}${PCT}%${RESET}, ${NOTE}   [만충 16.8 / 주의 ${BAT_WARN}(20%) / 위험 ${BAT_CRIT}(11%)]"

if [ "$MODE_ARG" != "--inline" ]; then
    echo "      ※ 잔량은 무부하 전압 기준 근사치 — 주행 중에는 처져서 낮게 나온다."
fi
exit 0
