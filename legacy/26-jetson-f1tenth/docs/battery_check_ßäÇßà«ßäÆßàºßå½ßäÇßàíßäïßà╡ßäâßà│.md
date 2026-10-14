# 배터리 전압 확인 기능 구현 가이드

> 젯슨에서 직접 구현할 것 — 노트북에서 먼저 수정하면 충돌 날 수 있음.

---

## 목표

1. **`bat.sh`** — 배터리 전압을 단독으로 확인하는 스크립트 (새 파일)
2. **`race.sh`** — 기동 시 배터리 전압 한 줄 출력 (기존 파일 수정)

---

## 구현 내용

### 1. `bat.sh` 신규 생성

`~/yax/bat.sh` 로 생성, `chmod +x` 필수.

```bash
#!/usr/bin/env bash
# ============================================================
#  bat.sh — 배터리 전압 확인
#
#  사용:
#    ./bat.sh              전압 출력 (스택 미실행 시 VESC를 잠깐 띄웠다 닫음)
#    ./bat.sh --inline     race.sh 내부 호출용 (에러 시 경고만, exit 안 함)
# ============================================================
INLINE=${1:-}
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS=${RACE_WS:-$HOME/f1tenth_ws}

# 환경이 아직 안 소싱된 경우를 대비
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f "$WS/install/setup.bash"   ] && source "$WS/install/setup.bash"   2>/dev/null || true

# VESC 심링크 (없으면 만들기)
if [ ! -e /dev/sensors/vesc ]; then
    sudo mkdir -p /dev/sensors
    sudo ln -sf /dev/ttyACM0 /dev/sensors/vesc 2>/dev/null || true
fi

VESC_PID=""
# 스택이 이미 돌고 있으면 토픽 바로 구독, 없으면 vesc_driver 잠깐 띄우기
if ros2 topic info /sensors/core 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    : # 이미 발행 중 — 아래에서 바로 읽음
else
    ros2 run vesc_driver vesc_driver_node --ros-args \
        --params-file "$SCRIPT_DIR/config/vesc.yaml" \
        > /dev/null 2>&1 &
    VESC_PID=$!
    sleep 2.5
fi

VOLT=$(timeout 3 ros2 topic echo /sensors/core --once 2>/dev/null \
    | grep "voltage_input" | awk '{print $2}' | head -1)

# 임시로 띄운 vesc_driver 정리
if [ -n "$VESC_PID" ]; then
    kill "$VESC_PID" 2>/dev/null; wait "$VESC_PID" 2>/dev/null || true
fi

if [ -z "$VOLT" ]; then
    echo "[bat] ⚠ 전압을 읽지 못했습니다 (VESC 연결 확인)"
    [ "$INLINE" = "--inline" ] && exit 0 || exit 1
fi

V=$(printf "%.1f" "$VOLT" 2>/dev/null || echo "$VOLT")

# 전압 기준 (4S LiPo, 충전기 실측 기반 보정)
#   실측: 3.78V/cell (15.12V) = 29% — 이 기준으로 임계값 설정
#   ≥ 15.6V  초록 — 60%+ (충분)
#   ≥ 14.8V  노랑 — ~20% (충전 권장)
#    < 14.8V  빨강 — 위험 (~11.8V 에서 젯슨 셧다운)
if   awk "BEGIN{exit !($VOLT >= 15.6)}"; then COLOR="\033[32m"
elif awk "BEGIN{exit !($VOLT >= 14.8)}"; then COLOR="\033[33m"
else                                           COLOR="\033[31m"
fi
RESET="\033[0m"

echo -e "[bat] 배터리  ${COLOR}${V} V${RESET}  (충전: ~16.8V  주의: <15.6V  위험: <14.8V)"
```

---

### 2. `race.sh` 수정

`----------1-1. loopback 멀티캐스트----------` 블록 바로 뒤,
`----------2. 모드 결정----------` 시작 전에 아래를 추가한다.

찾을 위치 (약 69번째 줄):
```bash
fi

# ---------- 2. 모드 결정 ----------
```

삽입할 내용:
```bash
fi

# ---------- 1-2. 배터리 전압 ----------
"$SCRIPT_DIR/bat.sh" --inline || true

# ---------- 2. 모드 결정 ----------
```

> `--inline` 플래그: 전압 읽기 실패해도 `exit 0` 처리해서 race.sh 기동을 막지 않는다.

---

## 예상 출력

**기동 시 (race.sh):**
```
[dds] CycloneDDS  |  인터페이스 192.168.45.246 고정  |  피어 1개
[bat] 배터리  15.3 V  (충전: ~16.8V  주의: <12.8V  셧다운: ~11.8V)
============================================
  RoboRacer 2026  |  MODE = q2
  config: /home/jetson/yax/race_script.yaml
============================================
```

**단독 실행:**
```bash
./bat.sh
[bat] 배터리  13.1 V  (충전: ~16.8V  주의: <12.8V  셧다운: ~11.8V)
```
노랑(~20%)이면 충전 권장, 빨강(<14.8V)이면 즉시 충전.

---

## 주의사항

- **스택이 안 돌고 있을 때** `bat.sh` 는 `vesc_driver_node` 를 2.5초 임시 기동한다.
  `race.sh` 기동 시간이 약 3초 늘어남 — 허용 범위.
- **스택이 이미 돌고 있을 때** (`./race.sh check` 등 별도 터미널) 는 토픽 바로 읽어 빠름.
- `vesc_driver_node` 의 `--params-file` 경로는 `~/yax/config/vesc.yaml` — 이미 있는 파일.
  경로가 다르면 `ros2 param dump /vesc_driver` 로 현재 노드 파라미터 확인.

---

## 구현 후 커밋

```bash
cd ~/yax
chmod +x bat.sh
git add bat.sh race.sh
git commit -m "feat: bat.sh 배터리 전압 확인 + race.sh 기동 시 전압 표시"
git push origin main
```

---

## ✅ 구현 완료 (2026-07-21, 젯슨에서 실행·검증)

가이드대로 구현하되, 젯슨에서 실측하며 아래 4가지를 바꿨다. 이유를 남겨둔다.

### 1. 임시 드라이버가 고아로 남던 문제 (★ 중요)

가이드의 `kill "$VESC_PID"` 는 **동작하지 않는다.** `ros2 run` 은 래퍼라서
실제 `vesc_driver_node` 는 그 자식으로 뜬다. 래퍼만 죽이면 노드가 살아남는다 —
첫 실행에서 그대로 재현됐다.

```
--- 잔여 vesc 프로세스 ---
5942 .../vesc_driver_node --ros-args --params-file /home/jetson/yax/config/vesc.yaml
```

이게 남으면 두 가지가 터진다:
- `race.sh` 의 **중복 실행 가드(2-1)** 가 `vesc_driver_node` 를 패턴으로 찾기 때문에
  기동할 때마다 "이미 실행 중인 스택입니다" 프롬프트가 뜬다
- VESC **시리얼을 쥔 채** 남아 진짜 스택이 포트를 못 연다

→ 자손까지 재귀로 훑는 `kill_tree` + 기동 전 PID 대조군(`PRE_PIDS`)으로 해결.
   대조군을 쓰는 이유는 **남이 띄운 드라이버는 건드리지 않기** 위해서다.

### 2. 기동 지연 3초 → 실제로는 7.3초였다

`sleep 2.5` 후 `topic echo` 를 부르면 지연이 **직렬로** 쌓인다. 실측:

| 구간 | 시간 |
|------|------|
| `ros2 topic info` (스택 감지) | 1.7 s |
| `sleep` (드라이버 대기) | 2.0 s |
| `ros2 topic echo --once` | 2.0 s |

두 군데를 고쳐 **~3.7초**로 줄였다:
- **스택 감지를 `pgrep` 으로** — `ros2 topic info` 는 DDS 탐색에만 1.7초를 쓴다.
  발행자는 곧 `vesc_driver_node` 프로세스이므로 `pgrep` 이면 즉시 판별된다
- **`sleep` 제거** — `topic echo --once` 는 발행자가 나타날 때까지 기다린다.
  드라이버 기동과 DDS 탐색이 **병렬로** 진행되므로 기다릴 필요가 없다

### 3. 전압 대신 **잔량 %** 표시

LiPo 는 3.7~3.9 V/cell 구간에서 곡선이 가팔라 0.1V 가 잔량 30%를 가른다.
전압만 보면 "아직 많이 남은 것 같은데 갑자기 죽는" 일이 생긴다 — 이번 방전이 그랬다.
그래서 구간 선형보간으로 %를 같이 찍는다. 앵커는 **이 문서의 실측값을 그대로** 썼다:

| 전압 | V/cell | 잔량 | 근거 |
|------|--------|------|------|
| 16.8 V | 4.20 | 100% | 만충 |
| 15.6 V | 3.90 | 60% | 이 문서 "초록 = 60%+" |
| 15.12 V | 3.78 | **29%** | 이 문서 충전기 실측 앵커 |
| 14.8 V | 3.70 | 20% | 이 문서 "노랑 = ~20%" |
| 13.2 V | 3.30 | 0% | 방전 하한 |

### 4. 그 외

- `--volts` 모드 추가 (숫자만 출력 — 다른 스크립트에서 쓰기 좋게)
- 터미널이 아니면 색을 끈다 (`./race.sh log` 로 리다이렉트할 때 `ESC[32m` 이 남지 않도록)
- 실제 출력 (충전 직후 상태):

```
[bat] 배터리 15.6 V (3.90 V/cell) — 약 60%, 충분   [만충 16.8 / 주의 15.6 / 위험 14.8]
      ※ 잔량은 무부하 전압 기준 근사치 — 주행 중에는 처져서 낮게 나온다.
```

### 검증한 것 / 안 한 것

- ✅ 단독 실행, `--inline`, `--volts`, 전압 읽기 실패, 파이프 리다이렉트
- ✅ 모든 경로에서 고아 프로세스 0개
- ✅ `set -e` 아래에서 `bat.sh` 가 실패해도 `race.sh` 기동이 안 막히는 것
- ⚠ **`race.sh` 를 실제로 끝까지 기동한 검증은 못 했다** — `sudo ip link`(1-1 블록)가
  비대화형 셸에서 막혀서다. 실제 터미널에서는 정상 동작하며, 배터리 줄은 그 뒤에 있다.
  다음 주행 때 첫 줄에 `[bat]` 이 뜨는지만 확인해 줄 것
