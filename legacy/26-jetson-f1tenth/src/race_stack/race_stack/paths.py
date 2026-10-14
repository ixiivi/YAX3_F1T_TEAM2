#!/usr/bin/env python3
"""경로 규칙 한 곳 — 주행 로그를 어디에 쌓을지.

두 군데서 따로 정하면 로그가 갈린다. 실제로 2026-08-06 에
`stanley_controller` 와 `slip_logger` 가 각자 계산하다 갈렸다.

이 모듈은 **의존성이 없어야 한다** (표준 라이브러리만). slip_logger 는 터미널
패널만 그리는 가벼운 노드라, 경로 하나 얻자고 cv2/scipy 를 끌어오는 컨트롤러
모듈을 import 하게 만들면 안 된다.
"""
import os


def repo_root() -> str:
    """주행 로그(logs/)를 저장할 **git repo** 루트.

    ★ 예전엔 RACE_WS 를 썼는데 그 값은 실행 워크스페이스(~/f1tenth_ws)라서
      로그가 repo(~/yax) 밖에 쌓였다. logs/README.md 는 "git 으로 커밋해서
      공유" 를 전제로 하는데 정작 커밋될 수 없는 자리에 있었다
      (2026-08-06: 슬립 CSV·주행 로그가 전부 ~/f1tenth_ws/logs/ 에서 발견).
      __file__ 폴백도 build/race_stack 를 거쳐 같은 곳으로 떨어졌다.

    우선순위:
      1. RACE_REPO        — race.sh 가 자기 위치로 export (가장 확실)
      2. RACE_SCRIPT 의 상위 — race_script.yaml 은 repo 에만 있다
      3. RACE_WS          — 옛 동작 (race.sh 없이 ros2 run 한 경우)
      4. race_script.yaml 을 찾을 때까지 위로 탐색
    """
    repo = os.environ.get('RACE_REPO')
    if repo:
        return os.path.expanduser(repo)

    script = os.environ.get('RACE_SCRIPT')
    if script:
        return os.path.dirname(os.path.abspath(os.path.expanduser(script)))

    ws = os.environ.get('RACE_WS')
    if ws:
        return os.path.expanduser(ws)

    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        if os.path.exists(os.path.join(d, 'race_script.yaml')):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return os.path.expanduser('~/yax')
