#!/usr/bin/env bash
set -euo pipefail

JETSON_HOST="${JETSON_HOST:-jetson@192.168.50.57}"
JETSON_WS="${JETSON_WS:-/home/jetson/f1tenth_ws}"
JETSON_REPO="${JETSON_REPO:-/home/jetson/yax}"   # race_script.yaml 원본 위치
SESSION="${ROBORACER_SESSION:-roboracer}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # 이 스크립트(=repo) 위치 (노트북)
CMD="${1:-status}"

# 노트북에서 RViz 기동 (백그라운드). viz 플래그 또는 ./jetson.sh rviz 로 호출.
launch_rviz () {
  local cfg="$DIR/config/race.rviz"
  if ! command -v rviz2 >/dev/null 2>&1; then
    echo "[jetson.sh] ⚠ 노트북에 rviz2 가 없어 시각화를 건너뜁니다 (sudo apt install ros-humble-rviz2)"; return
  fi
  [ -f "$cfg" ] || { echo "[jetson.sh] ⚠ RViz 설정 없음: $cfg"; return; }
  echo "[jetson.sh] 노트북에서 RViz 기동 → $cfg"
  ( source /opt/ros/humble/setup.bash 2>/dev/null; rviz2 -d "$cfg" >/dev/null 2>&1 & )
}

case "$CMD" in
  q1|q2|q3|final|mapping)
    CTRL="${2:-}"   # 선택: 컨트롤러 오버라이드 (예: ./jetson.sh q1 pure_pursuit)
    # 주행 모드(mapping 제외)이고 race_script.yaml 의 viz: true 면 노트북에서 RViz 자동 기동
    if [ "$CMD" != "mapping" ]; then
      VIZ=$(ssh "$JETSON_HOST" "grep -E '^[[:space:]]*viz:' $JETSON_REPO/race_script.yaml 2>/dev/null | awk '{print \$2}' | tr -d '[:space:]'" || echo "")
      if [ "$VIZ" = "true" ] || [ "$VIZ" = "True" ]; then launch_rviz; fi
    fi
    ssh -t "$JETSON_HOST" "tmux new-session -d -s $SESSION -c $JETSON_WS './race.sh $CMD $CTRL; bash' 2>/dev/null || tmux new-window -t $SESSION -c $JETSON_WS './race.sh $CMD $CTRL; bash'; tmux attach -t $SESSION"
    ;;
  rviz)
    launch_rviz   # 주행과 별개로 노트북에서 RViz 만 열기
    ;;
  savemap)
    NAME="${2:?usage: ./jetson.sh savemap <map_name>}"
    ssh -t "$JETSON_HOST" "cd $JETSON_WS && ./race.sh savemap '$NAME'"
    ;;
  log)
    NAME="${2:?usage: ./jetson.sh log <waypoint_name>}"
    ssh -t "$JETSON_HOST" "tmux new-session -d -s $SESSION -c $JETSON_WS './race.sh log $NAME; bash' 2>/dev/null || tmux new-window -t $SESSION -c $JETSON_WS './race.sh log $NAME; bash'; tmux attach -t $SESSION"
    ;;
  stop)
    ssh -t "$JETSON_HOST" "cd $JETSON_WS && ./race.sh stop; tmux kill-session -t $SESSION 2>/dev/null || true"
    ;;
  status)
    ssh -t "$JETSON_HOST" "tmux ls 2>/dev/null || true; source /opt/ros/humble/setup.bash; source $JETSON_WS/install/setup.bash 2>/dev/null || true; ros2 node list 2>/dev/null || true"
    ;;
  attach)
    ssh -t "$JETSON_HOST" "tmux attach -t $SESSION"
    ;;
  *)
    echo "usage: ./jetson.sh q1|q2|q3|final|mapping|rviz|savemap <name>|log <name>|stop|status|attach"
    exit 2
    ;;
esac
