#!/usr/bin/env bash
# ============================================================
#  tune.sh — yaml 을 에디터로 안 열고 값만 안전하게 수정
#
#  다음 ./race.sh 부터 적용 (재빌드 불필요). 스택이 켜져 있으면 재시작.
#  ./tune.sh -h  하면 이 설명이 그대로 나온다.
#
#
#  ── 속도: q1 / q2 / q3 / race 는 동시에 안 먹는다 ──────────────
#
#    launch 가 ./race.sh 인자로 받은 **그 모드 키 하나만** 고른다.
#
#        ./race.sh q1     →  grip_util.q1     만 사용
#        ./race.sh q2     →  grip_util.q2     만 사용
#        ./race.sh q3     →  grip_util.q3     만 사용
#        ./race.sh race   →  grip_util.race   만 사용
#
#    q1 을 올려도 ./race.sh q2 속도는 안 변한다. 반대도 같다.
#    yaml 의 mode: 줄은 ./race.sh 인자가 있으면 무시된다.
#
#    hard_cap 만 예외 — 모든 모드 공통 천장 (m/s). 0 = 끔.
#    hard_cap 이 2 면 q1 η=1.0 이어도 2㎧ 에서 잘린다.
#
#    젯슨에서는 grip_util 이 config/vehicle_<차>.yaml 로 간다.
#    race_script.yaml 의 limits.grip_util 을 고쳐도 overlay 가 덮어써서
#    주행에 안 먹는다. 이 스크립트가 파일을 골라 준다.
#
#    숫자 하나(예: grip_util: 0.6)면 q1/q2/race 가 **전부 그 값**.
#    모드별로 나누려면 {q1: …, q2: …, q3: …, race: …} 딕셔너리여야 한다.
#
#
#  ── 대화형 (SSH 에서) ────────────────────────────────────────
#
#    ./tune.sh i              번호 고르고 값 입력 → 즉시 저장
#    ./tune.sh speed          위와 같음 (TTY 일 때)
#
#      1 3.0                  hard_cap = 3
#      2 +=0.1                표시된 η 를 0.1 올림
#      cap=4                  별칭으로도 됨
#      q1=0.6                 ./race.sh q1 전용 η
#      u                      직전 변경 되돌리기
#      q                      종료
#
#
#  ── 한 줄 별칭 ───────────────────────────────────────────────
#
#    ./tune.sh cap=3          hard_cap=3          전 모드 천장
#    ./tune.sh q1=0.6         grip_util.q1=0.6    ./race.sh q1 만
#    ./tune.sh q2+=0.1        grip_util.q2 += 0.1 ./race.sh q2 만
#    ./tune.sh q3=0.75                            ./race.sh q3 만
#    ./tune.sh race=0.6                           ./race.sh race 만
#    ./tune.sh eta+=0.1       지금 yaml mode 의 η (인자 없이 ./race.sh 할 때)
#
#
#  ── 그 밖 ────────────────────────────────────────────────────
#
#    ./tune.sh status         현재 값 + 조절 우선순위 + 커밋 대비 변경
#    ./tune.sh                설정값 전체 보기
#    ./tune.sh stanley        한 섹션만
#    ./tune.sh stanley.gain=1.2
#    ./tune.sh stanley.gain+=0.2
#    ./tune.sh --undo         직전 변경 되돌리기
#    ./tune.sh --log / --diff
#    ./tune.sh --force ...    범위·5배 가드 무시
#
#  노트북에서:
#    ssh jetson "cd ~/yax && ./tune.sh cap=3"
# ============================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
exec python3 "$SCRIPT_DIR/tune.py" "$@"
