# Handoff: 맥북용 F1TENTH 원격 제어판

작성일 2026-11-03. 이 문서만 읽고 작업을 시작할 수 있게 썼다.

## 1. 목표와 요구사항

지금 쓰는 제어판(`~/MAGP/MAGP/control_panel.py`, PyQt5)은 Jetson 화면에서 직접 돌린다. 그래서 원격으로 쓰려면 RustDesk로 화면을 통째로 보내야 한다.
대회장에서는 Wi-Fi가 혼잡하고 인터넷이 없을 수 있다. 화면 스트리밍은 Jetson의 CPU/GPU도 잡아먹는다.
그래서 **맥북에서 도는 GUI가 SSH로만 차와 통신하는 구조**로 바꾼다.

사용자 요구사항 두 가지:

1. **Jetson에서 기능이 바뀌어도 맥 GUI는 고치지 않아도 되게 한다.** 맥 GUI를 고쳐야 하는 경우는 명령을 주고받는 프로토콜 자체가 바뀔 때뿐이다.
2. **맥 GUI 레이아웃은 지금 `control_panel.py`와 똑같이 나와야 한다.** 버튼 배치, 문구, 색이 같아야 한다.

## 2. 아키텍처 (결정 사항)

```
[MacBook]                                     [Jetson: yunsang@yunsang-desktop]
mac_panel.py  = 범용 렌더러 (PyQt6)            panel_server.py  (헤드리스, rclpy, 127.0.0.1:8765)
  - 서버가 보낸 레이아웃을 그대로 그림           ├─ panel_ui.py : 레이아웃 정의 (= 지금 제어판과 동일)
  - 위젯 이벤트를 action으로 전송               ├─ 액션 핸들러 : 명령, 검증, 경고 문구 전부 여기
  - 서버가 보낸 update를 위젯에 적용            ├─ 자식 프로세스 관리, ROS 발행/구독
  └─ ssh -N -L 8765:127.0.0.1:8765 car ────────┘  └─ 20 Hz 제어 루프, E-STOP, 데드맨
```

- **서버 주도 UI(server-driven UI) 방식이다.** 맥 클라이언트에는 기능 지식이 하나도 없다. 버튼 이름, ros2 명령, 상호 제약, 경고 문구, 색 규칙, 키 매핑은 모두 서버에 있다. 클라이언트가 아는 것은 다음 네 가지뿐이다.
  - 위젯 어휘(5절)
  - 메시지 형식(6절)
  - 키 입력을 서버로 보내는 규칙
  - 하트비트
- **Jetson에서 기능을 추가하거나 바꿀 때는 `panel_ui.py`와 서버 핸들러만 고친다.** 서버를 재시작하면 맥 GUI가 다시 접속해서 새 레이아웃을 받아 그린다.
- **맥 GUI를 고쳐야 하는 경우는 위젯 어휘나 메시지 형식이 바뀔 때뿐이다.** 그럴 때는 `PROTOCOL = 1`을 올린다. 버전이 다르면 클라이언트는 "맥 GUI 업데이트 필요 (서버 vN / 클라이언트 vM)" 화면만 띄운다.
- **서버가 모든 상태와 프로세스를 소유한다.** 맥이 끊기거나 노트북을 닫아도 실행 중인 노드는 죽지 않는다.
- **맥에는 ROS가 필요 없다.** 맥에서는 DDS를 쓰지 않으므로 ROS_DOMAIN_ID 충돌이나 Wi-Fi 멀티캐스트 문제도 생기지 않는다.
- **전송 방식은 줄 단위 JSON over TCP다(NDJSON).** Jetson에는 `websockets`도 `fastapi`도 없고, 대회장에서는 pip를 못 쓸 수 있다. 그래서 서버는 표준 라이브러리(`asyncio` 또는 `socketserver` + 스레드)만 쓴다. 클라이언트는 `QTcpSocket`을 쓴다.
- **보안은 SSH에 맡긴다.** 서버는 `127.0.0.1`에만 bind하고, 맥은 SSH 포트 포워딩으로 접속한다.
- **클라이언트는 PyQt6와 PyQt5를 둘 다 지원한다.** `try: from PyQt6 … except ImportError: from PyQt5 …` 형태의 작은 shim을 둔다. 이러면 같은 `mac_panel.py`를 Jetson 로컬에서도 돌릴 수 있고, 나중에 UI 코드를 한 곳으로 합칠 수 있다.

## 3. 현재 시스템 사실 (코드에서 확인함)

| 항목 | 값 |
|---|---|
| 차 | Jetson Orin Nano Super, JetPack R36.4.7, ROS 2 Humble, Python 3.10 |
| 워크스페이스 | `~/MAGP/MAGP` (`source /opt/ros/humble/setup.bash && source ~/MAGP/MAGP/install/setup.bash`) |
| 네트워크 | Wi-Fi `wlP1p1s0` (현재 아이폰 핫스팟 172.20.10.x), Tailscale 100.93.106.78, 이더넷 192.168.0.15는 **라이다 전용** |
| 맵 | `~/MAGP/MAGP/maps/<name>.{yaml,pgm,posegraph,data}` |
| 웨이포인트 | `~/MAGP/MAGP/waypoints/<map>_wp.csv`, 헤더 `x,y,speed` (map frame), 미리보기 `.png` |
| 미설치 | `tmux`, `websockets`, `fastapi` (설치돼 있음: numpy, PIL, yaml, PyQt5) |

**ackermann_mux** (`src/f1tenth_system/f1tenth_stack/config/mux.yaml`):
- `/drive`: 자율주행 입력, priority 10, timeout 0.2 s
- `/teleop`: 수동/E-STOP 입력, priority 100, timeout 0.2 s

**지금 제어판의 기능 로직은 전부 서버로 옮긴다.** 기준 파일은 `~/MAGP/MAGP/control_panel.py`다.

| 기존 위치 | 내용 | 옮길 곳 |
|---|---|---|
| `Proc` 클래스 | 자식 프로세스를 `setsid`로 띄우고, 프로세스 그룹에 SIGINT → 8초 후 SIGTERM → 12초 후 SIGKILL | 서버. Qt 대신 `subprocess.Popen`과 stdout 읽기 스레드로 구현 |
| `RosLink` | `/sensors/core`, `/scan`, `/pure_pursuit/status`, `/pure_pursuit/event`, TF `map→base_link` 구독. `/teleop`, `/pure_pursuit/command` 발행 | 서버. 그대로 |
| `toggle()`, `save_map()`, `edit_waypoints()`, `relocalize()`, `go()` | ros2 명령과 상호 제약 경고 | 서버 액션 핸들러. 경고는 `dialog` 메시지로 보냄 |
| `tick()`, `eventFilter()` | 20 Hz 수동 주행 램프(`keyboard_teleop`의 `step_speed`, `ramp`, `floor_speed`, `REVERSE_HOLD`), WASD/X/Space/B | 서버. 클라이언트는 키 이름만 보냄 |
| `refresh_status()`, `refresh()` | 상태 칩 색(배터리 14.8/14.0 V 경계, 라이다 30 Hz 경계), 버튼 문구와 색 토글 | 서버. `update` 메시지로 보냄 |
| `reload_lists()`, `map_changed()` | 맵/웨이포인트 목록, 맵을 고르면 같은 이름의 웨이포인트 자동 선택 | 서버 |

## 4. 안전 요구사항 (가장 중요)

mux는 `/teleop`이 0.2초 동안 안 들어오면 **자율주행 `/drive`로 되돌아간다.** 그래서 정지 신호를 맥에서 오는 메시지에 맡기면 안 된다.

1. **E-STOP 상태는 서버가 들고 있는다.** E-STOP이 켜져 있는 동안에는 서버가 맥 연결과 상관없이 20 Hz로 `/teleop 0,0`을 발행한다. 해제는 사용자가 버튼으로 해제할 때만 한다.
2. **클라이언트는 연결되어 있는 동안 항상 10 Hz로 `heartbeat`를 보낸다.** 수동 모드에서 0.3초 넘게 하트비트가 끊기면 서버는 다음을 한다.
   - E-STOP 상태로 전환한다.
   - `/pure_pursuit/command estop`을 보낸다.
   - 사유를 "연결 끊김으로 긴급 정지됨"으로 기록해서 재접속한 클라이언트에 보여준다.
   - 재접속해도 E-STOP은 자동으로 해제하지 않는다.
3. **자율주행 중에 연결이 끊겼을 때의 동작은 서버 설정 `on_disconnect`로 정한다.** `stop`(기본값, 연습용)과 `keep`(레이스용) 중 하나다. 현재 값은 상태 줄에 표시한다. 이 설정도 서버 쪽 레이아웃 변경일 뿐이라 클라이언트는 고치지 않는다.
4. **서버가 종료될 때는** `estop`과 `/teleop 0,0`을 5번 보낸 뒤 자식 프로세스를 정리한다.
5. **클라이언트 창을 닫아도 서버의 프로세스는 그대로 둔다.** 닫을 때 클라이언트는 `close_request`를 보내고, 서버가 돌려주는 `dialog`를 따른다. 예를 들면 "수동 주행 중이에요. 정지하고 닫을까요?" 같은 질문이다. 서버가 `{"op":"close_ok"}`를 보내면 창을 닫는다. 서버에 연결이 안 된 상태면 바로 닫는다.

## 5. 위젯 어휘 (클라이언트 v1이 지원할 전부)

레이아웃은 노드 트리다. 모든 노드는 공통 필드를 가질 수 있다.
- `id`: update 대상
- `stretch`: 부모 박스 안의 비율
- `style`: Qt stylesheet 문자열 그대로
- `enabled`, `visible`, `tooltip`
- `font`: `{"mono":true, "size_delta":6, "bold":true}`
- `min_height`

| type | 필드 | 대응 Qt | 사용자 이벤트 → 서버 |
|---|---|---|---|
| `vbox` / `hbox` | `children` | QVBoxLayout / QHBoxLayout | – |
| `grid` | `children`, 각 child에 `row`, `col`, `rowspan`, `colspan` | QGridLayout | – |
| `group` | `title`, `child` | QGroupBox | – |
| `stretch` | `factor` | addStretch | – |
| `label` | `text`, `wrap` | QLabel | – |
| `button` | `text`, `action` | QPushButton (NoFocus) | `action` |
| `combo` | `items: [[label,data],…]`, `current`(data), `action`(선택) | QComboBox (NoFocus) | 값이 바뀌면 `action` |
| `line_edit` | `text`, `placeholder` | QLineEdit | – (값은 values로 전송) |
| `spin` | `min`, `max`, `step`, `value`, `decimals` | QDoubleSpinBox | – |
| `check` | `text`, `checked` | QCheckBox (NoFocus) | – |
| `log_tabs` | `tabs: [[key,label],…]`, `max_lines` | QTabWidget + QPlainTextEdit(readOnly, mono) | – |
| `map_view` | 빈 노드. 내용은 `map_*` 메시지로 받음 | QGraphicsView | 클릭 → `action` (월드 좌표) |

- **values 규칙:** 클라이언트가 `action`을 보낼 때는 `id`가 있는 모든 입력 위젯(combo, line_edit, spin, check)의 현재 값을 `values`에 같이 담아 보낸다. 덕분에 서버는 어떤 버튼이 어떤 입력을 쓰는지 클라이언트에 알려줄 필요가 없다.
- **포커스 규칙:** 버튼, 콤보, 체크박스, 탭, 로그는 `NoFocus`다. 그래야 WASD나 Space가 버튼을 누르지 않는다. 기존 `nofocus()`와 같은 규칙이다.
- **창 필드:** `ui` 메시지 최상위에 `title`, `size: [1000, 800]`, `root`가 있다. 창은 `windows`에 여러 개 둘 수 있다. 메인 창 외에 웨이포인트 편집기 창 등을 추가할 수 있다.

## 6. 메시지 형식 (PROTOCOL 1)

모든 메시지는 JSON 한 줄이고 `\n`으로 끝난다.

**클라이언트 → 서버** (이 6종이 전부다):
```json
{"op":"hello","protocol":1,"client":"mac_panel"}
{"op":"heartbeat"}
{"op":"action","name":"toggle:bringup","source":"<widget id>","values":{"map_cb":"/home/…/DLAB.yaml","vmax":1.0,…},"point":[x,y]?,"button":"left|right"?}
{"op":"key","name":"speed_up"}
{"op":"dialog_result","id":"d17","answer":"yes|no|ok"}
{"op":"close_request"}
```

**서버 → 클라이언트:**
```json
{"op":"ui","protocol":1,"windows":{"main":{"title":"F1TENTH 제어판","size":[1000,800],"root":{…}}}}
{"op":"update","window":"main","widgets":{"<id>":{"text":"끄기","style":"…","enabled":true,"items":[…],"current":"…","value":1.0,"checked":true}}}
{"op":"keymap","enabled":true,"keys":{"W":"speed_up","S":"speed_down","A":"steer_left","D":"steer_right","X":"center","Space":"soft_stop","B":"hard_stop"}}
{"op":"log","tabs":"<log_tabs id>","tab":"slam","text":"…","select":false}
{"op":"dialog","id":"d17","kind":"warning|question","title":"F1TENTH 제어판","text":"…"}
{"op":"notify","title":"F1TENTH 자율주행","text":"…"}          // macOS: osascript display notification
{"op":"map_image","view":"<id>","png_b64":"…","resolution":0.05,"origin":[x,y,yaw]}
{"op":"map_layers","view":"<id>","pose":[x,y,yaw]|null,"paths":[{"name":"wp","xy":[[x,y],…],"colors":["#rrggbb",…]|null,"closed":true}],"points":[[x,y],…]}
{"op":"close_ok"}
{"op":"error","text":"…"}
```

**클라이언트 동작 규칙:**
- 접속하면 `hello`를 보낸다. 서버는 `ui`, 현재 상태 전체를 담은 `update`, `keymap`, 각 로그 탭의 최근 300줄을 보낸다.
- 연결이 끊기면 창을 지우지 않고 회색으로 덮어 "연결 끊김 — 재접속 중…"을 표시한다. 1초 간격으로 재접속한다.
- 서버가 `ui`를 다시 보내면 창 내용을 통째로 다시 만든다.
- **키 처리:** `keymap.enabled`가 true이고 메인 창이 활성 상태일 때, 키가 `keys`에 있으면 그 키 이름을 보내고 이벤트를 소비한다. auto-repeat은 무시한다. 포커스가 `line_edit`나 `spin`에 있으면 키를 보내지 않는다. 키 이름은 Qt 키 이름 문자열로 비교한다(`QKeySequence(key).toString()`).
- **`map_view` 좌표 변환:** 픽셀 → 월드는 `x = origin_x + px*res`, `y = origin_y + (H - py)*res`이다. 클릭하면 `action`에 `point`(월드 좌표)와 `button`을 담아 보낸다. 액션 이름은 노드의 `action` 필드에서 가져온다.
- 모르는 `op`나 모르는 위젯 필드는 무시한다. 그래야 서버가 필드를 추가해도 클라이언트가 깨지지 않는다. 모르는 위젯 `type`이면 그 자리에 "(지원 안 되는 위젯: X)" 라벨을 표시한다.

**서버 쪽 확장 방법** (맥 수정 없음):
- 새 버튼: `panel_ui.py`에 노드를 추가하고 `@action("이름")` 핸들러를 하나 등록한다.
- 새 상태 칩: label 노드를 추가하고 상태 루프의 `update`에 그 id를 넣는다.
- 새 단축키: `keymap`에 추가하고 `key` 핸들러에서 처리한다.
- 새 창(예: 웨이포인트 편집기): `windows`에 추가한다.

## 7. 레이아웃: `control_panel.py`와 동일하게

`panel_ui.py`는 `Panel.__init__`의 `_status_row`, `_run_box`, `_map_box`, `_auto_box`, `_manual_box`를 **문구, 순서, 위치, 색까지 그대로** 노드 트리로 옮긴다. 구조는 다음과 같다.

```
main window  title "F1TENTH 제어판", size 1000x800
vbox
├─ hbox  상태 칩 5개: battery, vesc, lidar, pose, speed
│        기본 style "padding:4px 8px; border-radius:4px; background:#444; color:white"
│        색: 초록 #2e7d32, 빨강 #c62828, 노랑 #f9a825
├─ hbox
│  ├─ vbox stretch 3
│  │  ├─ group "실행"     grid: 라벨 + [켜기] 버튼 × (드라이버 (라이다+VESC) / SLAM (맵 만들기) / 위치 추정 (AMCL) / RViz)
│  │  ├─ group "맵"       grid: (0,0)"맵" (0,1-2)map_cb (0,3)[새로고침]
│  │  │                         (1,0-1)map_name placeholder "저장 이름 (비우면 map_월일_시분)" (1,2)[맵 저장] (1,3)[웨이포인트 편집]
│  │  └─ group "자율주행" grid: (0,0)"웨이포인트" (0,1-3)wp_cb
│  │                            (1,0)"최고 속도 [m/s]" (1,1)vmax spin 0.5–3.0 step 0.1 =1.0 (1,2)[위치 다시 찾기] (1,3)[자율주행 준비]
│  │                            (2,0-1)[출발] #2e7d32  (2,2-3)[정지 (감속)] #f9a825
│  │                            (3,0-3)pp_label  (4,0-1)☑장애물 우회  (4,2-3)☑위치 틀어지면 지나온 길로 후진
│  │                            (5,0-3)event_label wrap, color:#e65100 bold
│  └─ group "수동 주행 / 긴급 정지" stretch 2
│     vbox: [키보드 조종 켜기], manual_label(mono), 키 안내 라벨 3줄, stretch,
│           [긴급 정지] #c62828, min_height 70, font size_delta +6, bold
└─ log_tabs stretch 1: 드라이버, SLAM, 위치 추정, 자율주행, RViz, 웨이포인트 편집, 맵 저장/위치 찾기 (max 3000줄)
```

**모양을 같게 맞추기 위한 규칙:**
- 클라이언트는 `app.setStyle("Fusion")`을 쓴다. macOS 기본 위젯 스타일은 버튼 모양과 여백이 달라서, Fusion으로 Jetson 쪽과 통일한다.
- 고정폭 글꼴은 `QFontDatabase.systemFont(FixedFont)`를 쓴다.
- 일반 글꼴은 OS 시스템 글꼴을 그대로 쓴다. 맥에서 한글은 Apple SD Gothic Neo로 나온다. 글꼴 차이만 남고 배치는 같다.
- 레이아웃 여백과 간격은 Qt 기본값을 쓰고 임의로 바꾸지 않는다.
- **완료 기준:** 같은 서버에 Jetson(PyQt5)과 맥(PyQt6)에서 클라이언트를 띄워 기존 `control_panel.py`와 스크린샷 세 장을 나란히 비교한다. 위젯 순서, 줄 바꿈, 색, 버튼 활성 상태가 같아야 한다.

**기존 제어판과 다른 점 두 가지** (레이아웃은 그대로 두고 동작만 다르다):
- **RViz 버튼:** 맥에서 Jetson의 rviz2 창은 보이지 않는다. 버튼은 그대로 두되, 서버가 처음 켤 때 "Jetson 화면(모니터/RustDesk)에 뜹니다"라는 경고 `dialog`를 띄운다.
- **웨이포인트 편집 버튼:** 기존 편집기는 Jetson에 matplotlib 창을 띄운다. 3단계에서 서버가 `windows.editor`(`map_view` + 버튼들)를 열고 `path_tools.smooth_path`, `speed_profile`, `write_csv`로 저장하도록 바꾼다. 저장 순서는 `waypoint_editor.py` 140~155행과 같다. 이 변경도 서버 쪽만 고친다.

## 8. 구현 순서

1. **서버 뼈대** (`~/MAGP/MAGP/panel_server.py`, `panel_ui.py`, `panel_server.sh`)
   - TCP 서버, `ui`/`update`/`log`/`dialog` 전송
   - `control_panel.py` 로직 이전(3절 표)
   - 20 Hz 루프, E-STOP, 데드맨
   - `panel_server.sh`는 `panel.sh`처럼 ROS 환경을 source한 뒤 실행한다.
2. **범용 클라이언트** (`mac_panel.py`, 단일 파일): 5절과 6절 전체를 구현한다. `map_view`도 v1에 넣는다. 지금은 쓰지 않더라도 나중에 맥을 다시 고치지 않기 위해서다.
3. **동일성 검증:** 7절 완료 기준.
4. **웨이포인트 편집기 창:** 서버만 수정한다.
5. **운영 편의**
   - 클라이언트가 `ssh -N -o ServerAliveInterval=5 -o ExitOnForwardFailure=yes -L 8765:127.0.0.1:8765 <host>`를 자식 프로세스로 띄우고, 끊기면 재시도한다. 호스트는 인자나 설정 파일로 받고 기본값은 `car`다.
   - 서버를 systemd user service로 등록해서 부팅 시 자동으로 시작하게 한다.
   - (선택) 클라이언트를 `pyinstaller`로 `.app`으로 만든다.

## 9. 맥 쪽 준비

```bash
# ~/.ssh/config
Host car
    HostName 172.20.10.2        # 대회장 네트워크에 맞게 변경. 인터넷이 있으면 Tailscale 100.93.106.78도 가능
    User yunsang
    ServerAliveInterval 5
    ServerAliveCountMax 2
ssh-copy-id car
python3 -m venv ~/f1t-venv && ~/f1t-venv/bin/pip install PyQt6
~/f1t-venv/bin/python mac_panel.py            # 기본 호스트 car
```

## 10. 검증 방법

- **차 없이 서버만:** Jetson에서 `panel_server.sh`를 띄우고 Jetson 로컬에서 `mac_panel.py`(PyQt5)로 접속한다. bringup이 꺼져 있으면 칩이 빨강으로 나와야 한다.
- **기능 변경 테스트(요구사항 1):** `panel_ui.py`에 임시 버튼을 하나 추가하고 서버만 재시작한다. 맥 클라이언트를 고치지 않고도 버튼이 나타나고 동작해야 한다.
- **데드맨:** 차를 들어 바퀴를 띄운다. 수동 모드로 조금 전진시킨 뒤 맥 Wi-Fi를 끈다. `ros2 topic echo /teleop`에 0,0이 계속 나와야 하고, 바퀴는 0.3초 안에 멈춰야 한다. 재접속하면 "연결 끊김으로 긴급 정지됨"이 보여야 한다.
- **E-STOP 유지:** E-STOP을 켠 상태에서 SSH 터널을 끊는다. `/teleop` 발행이 계속돼야 한다.
- **서버 소유권:** bringup을 켠 상태에서 맥 앱을 닫는다. `ros2 node list`에 노드가 그대로 남아 있어야 한다.
- **기존 제어판과의 충돌:** 서버와 `control_panel.py`를 동시에 쓰면 `/teleop` 발행자가 둘이 된다. 서버가 떠 있을 때는 기존 제어판을 쓰지 말라고 README에 적는다.

## 11. 건드리지 말 것

- `src/f1tenth_system/vesc` 서브모듈 안의 로컬 패치: odom 부호 반전, `brake_below_speed` 0.3, `stop_brake_current` 10 A. `git submodule update --remote`를 하면 사라진다.
- `mux.yaml`의 timeout과 priority.
- `control_panel.py`와 `keyboard_teleop.py`. import해서 재사용하는 것은 괜찮다.
- wheelbase(odom 0.25, pure pursuit 0.33), speed gain(`ERPM_GAIN` 4614), steering gain은 아직 캘리브레이션이 안 됐다.

## 12. 열린 질문 (진행하면서 사용자에게 확인)

- 레이스 중 연결이 끊겼을 때 `on_disconnect`의 기본값을 `stop`으로 할지 `keep`으로 할지. 대회 규정이 원격 정지를 요구하는지도 확인이 필요하다.
- 레이저 스캔 점을 `map_layers`에 추가할지. 추가한다면 대역폭 때문에 서버에서 다운샘플해서 2 Hz 정도로 보내는 것을 제안한다.
