#!/usr/bin/env bash
# ============================================================
#  setup_laptop.sh — 노트북 시뮬 환경 구축 (최초 1회)
#
#  실행:  ./setup_laptop.sh
#  이후:  ./sim.sh
# ============================================================
set -e
REPO_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

echo "============================================"
echo "  RoboRacer 2026  |  노트북 시뮬 환경 구축"
echo "============================================"

source /opt/ros/humble/setup.bash

# ---------- 1. ~/f1tenth_ws: race_stack 심링크 ----------
echo ""
echo "[1/2] ~/f1tenth_ws 생성 및 race_stack 연결"
mkdir -p ~/f1tenth_ws/src

if [ -L ~/f1tenth_ws/src/race_stack ]; then
    echo "  이미 링크 있음 → 건너뜀"
elif [ -d ~/f1tenth_ws/src/race_stack ]; then
    echo "  ⚠ ~/f1tenth_ws/src/race_stack 가 심링크가 아닌 디렉토리 — 직접 확인 필요"
else
    ln -s "$REPO_DIR/src/race_stack" ~/f1tenth_ws/src/race_stack
    echo "  심링크 생성: ~/f1tenth_ws/src/race_stack → $REPO_DIR/src/race_stack"
fi

echo "  race_stack 빌드 중..."
cd ~/f1tenth_ws
colcon build --packages-select race_stack --symlink-install
echo "  ✓ race_stack 빌드 완료"

# ---------- 2. sim_ws: f1tenth_gym_ros 빌드 ----------
echo ""
echo "[2/2] sim_ws 빌드 중..."
SIM_WS="$REPO_DIR/f1TENTH-contest_full_stack/sim_ws"
cd "$SIM_WS"
colcon build
echo "  ✓ sim_ws 빌드 완료"

echo ""
echo "============================================"
echo "  셋업 완료!"
echo "  시뮬 실행: cd $REPO_DIR && ./sim.sh"
echo "============================================"
