#!/usr/bin/env python3
# ============================================================
#  tune.py — race_script.yaml 게인 값만 외과적으로 수정
#
#  ★ 왜 yaml.safe_load → yaml.dump 를 안 쓰는가
#    race_script.yaml 은 주석이 곧 팀 문서다. load/dump 왕복하면 주석·빈줄·
#    섹션 구분선이 전부 사라진다. 그래서 대상 "한 줄"만 정규식으로 치환하고
#    나머지 바이트는 손대지 않는다. 주석 정렬 열도 최대한 유지한다.
#
#  진입점은 tune.sh — 사용법은 그쪽 헤더 참조.
# ============================================================
import os
import re
import sys
import json
import shutil
import difflib
import datetime
import subprocess

import yaml

HERE = os.path.dirname(os.path.abspath(os.path.realpath(__file__)))
RACE_SCRIPT = os.path.join(HERE, 'race_script.yaml')

# ── 차량별 키는 race_script.yaml 이 아니라 차량 파일로 간다 ─────────────
#   race_script.yaml 에 써봐야 config/.vehicle_overlay.yaml 이 덮어써서
#   **주행에 안 먹는다.** 2026-08-10 에 차량별 설정을 분리하면서 생긴 함정이고,
#   tune.sh 가 그걸 모른 채 race_script 만 고치고 있었다.
#   → 대상 파일을 키에 따라 자동으로 고른다. (매핑 원본은 config/apply_vehicle.py)
VEHICLE_KEYS = {
    'pure_pursuit.wheelbase': 'geometry.wheelbase',
    'map.mass':               'geometry.mass',
    'map.lf':                 'geometry.lf',
    'map.lr':                 'geometry.lr',
    'map.mu':                 'tire.mu',
    'map.pac_b':              'tire.pac_b',
    'map.pac_c':              'tire.pac_c',
    'map.pac_d':              'tire.pac_d',
    'map.pac_e':              'tire.pac_e',
    'map.l1_min':             'tuning.l1_min',
    'map.l1_k':               'tuning.l1_k',
    'map.steer_alpha':        'tuning.steer_alpha',
    'map.slip_gain':          'tuning.slip_gain',
    'limits.speed_lag':       'tuning.speed_lag',
    'limits.a_lon_acc':       'tuning.a_lon_acc',
    'limits.launch_acc':      'tuning.launch_acc',
    'limits.launch_time':     'tuning.launch_time',
    'limits.a_lon_brk':       'tuning.a_lon_brk',
    'limits.grip_util':       'tuning.grip_util',
    'network.peers':          'network.peers',
}
for _k in ('steer_invert', 'speed_invert', 'trigger_invert', 'device',
           'axis_steer', 'axis_speed', 'btn_reverse', 'btn_toggle', 'max_speed',
           'trigger_deadzone', 'trigger_expo', 'steer_deadzone', 'debug'):
    VEHICLE_KEYS[f'joystick.{_k}'] = f'joystick.{_k}'
# grip_util.q1 처럼 한 단계 더 들어가는 것도 받는다
VEHICLE_PREFIX = {'limits.grip_util': 'tuning.grip_util'}

# 계산으로 나오는 값 — 손으로 고치면 안 된다
DERIVED_KEYS = {
    'pure_pursuit.max_steer':
        '서보 매핑에서 자동 계산된다: (servo_min − offset) / (−gain).\n'
        '  고치려면 config/vehicle_<차>.yaml 의 vesc.servo_min / '
        'steering_angle_to_servo_offset / _gain 을 고칠 것.\n'
        '  2026-08-09 에 offset 만 바뀌고 max_steer 가 안 따라와 좌 조향 지령의 '
        '25.8% 가 조용히 잘려나간 적이 있다.',
}


def _car_name():
    f = os.path.join(HERE, '.car')
    if not os.path.exists(f):
        return None
    return open(f).read().strip()


def target_file(dotted):
    """이 키를 실제로 고쳐야 하는 파일과 그 안에서의 경로를 돌려준다.

    반환 (경로, 그 파일에서의 키). 차량 파일이 없으면 race_script 로 폴백한다.
    """
    vkey = VEHICLE_KEYS.get(dotted)
    if vkey is None:
        for pre, vpre in VEHICLE_PREFIX.items():
            if dotted.startswith(pre + '.'):
                vkey = vpre + dotted[len(pre):]
                break
    if vkey is None:
        return RACE_SCRIPT, dotted
    car = _car_name()
    if car is None:
        return RACE_SCRIPT, dotted          # .car 가 없으면 옛 동작
    vpath = os.path.join(HERE, 'config', f'vehicle_{car}.yaml')
    if not os.path.exists(vpath):
        return RACE_SCRIPT, dotted
    return vpath, vkey


def overlay_cfg(cfg):
    """race_script + 차량 파일. 젯슨에서 실제로 달리는 값."""
    import copy
    out = copy.deepcopy(cfg)
    car = _car_name()
    if not car:
        return out
    vpath = os.path.join(HERE, 'config', f'vehicle_{car}.yaml')
    if not os.path.exists(vpath):
        return out
    v = yaml.safe_load(open(vpath)) or {}
    for rk, vk in VEHICLE_KEYS.items():
        try:
            val = get_nested(v, vk)
        except KeyError:
            continue
        node = out
        parts = rk.split('.')
        for p in parts[:-1]:
            if not isinstance(node.get(p), dict):
                node[p] = {}
            node = node[p]
        node[parts[-1]] = val
    return out


HISTORY = os.path.join(HERE, '.tune_history')
BACKUP = os.path.join(HERE, '.tune_backup.yaml')
BACKUP_JSON = os.path.join(HERE, '.tune_backup.json')

# SSH 한 줄용 별칭. eta 는 현재 mode 의 grip_util.
SPEED_ALIASES = {
    'hard_cap': 'speed.hard_cap',
    'cap':      'speed.hard_cap',
    'q1':       'limits.grip_util.q1',
    'q2':       'limits.grip_util.q2',
    'q3':       'limits.grip_util.q3',
    'race':     'limits.grip_util.race',
}

# 보기(no-arg) 에 출력할 섹션 — 주행 중 만질 값들만
VIEW_SECTIONS = ['stanley', 'pure_pursuit', 'gap_follower', 'speed', 'safety']
# 맨 위에 요약으로 보여줄 값
VIEW_SCALARS = ['mode', 'controller', 'map_name', 'waypoints']

# 안전 가드: (최솟값, 최댓값). None = 제한 없음.
# 실차가 벽으로 가는 오타(1.5 → 15)를 막는 게 목적이다. --force 로 무시 가능.
RANGES = {
    'stanley.gain':           (0.1, 5.0),
    'stanley.k':              (0.1, 5.0),
    'stanley.ks':             (0.1, 5.0),
    'stanley.ff_gain':        (0.1, 3.0),   # 0 금지 — 아래 FORBID_ZERO 참조
    'stanley.ff_preview_sec': (0.0, 1.5),
    'stanley.steer_lpf':      (0.1, 1.0),
    'stanley.vel_scale':      (0.1, 2.0),
    'stanley.lookahead_min':  (0.1, 5.0),
    'stanley.lookahead_max':  (0.1, 5.0),
    'stanley.obstacle_decel': (0.1, 1.0),
    'pure_pursuit.lookahead': (0.2, 5.0),
    'pure_pursuit.max_steer': (0.05, 0.5),
    'gap_follower.steer_gain': (0.05, 3.0),
    'gap_follower.speed_gain': (0.05, 3.0),
    'gap_follower.car_width':  (0.2, 1.0),
    'speed.hard_cap':         (0.0, 10.0),
    'speed.min_speed':        (0.0, 3.0),
    'limits.grip_util':       (0.05, 1.0),
    'limits.grip_util.q1':    (0.05, 1.0),
    'limits.grip_util.q2':    (0.05, 1.0),
    'limits.grip_util.q3':    (0.05, 1.0),
    'limits.grip_util.race':  (0.05, 1.0),
    'safety.ttc_threshold':   (0.05, 3.0),
    'joystick.max_speed':     (0.0, 6.0),
}

# ./tune.sh status 가 보여줄 게인과 조절 우선순위.
#   출처: docs/stanley_gain_tuning.md — "조절 우선순위" 절.
#   순서를 바꾸거나 항목을 늘릴 때는 그 문서를 같이 고칠 것.
TUNING_GUIDE = [
    ('stanley.ff_gain',        1, '코너에서 바깥으로 밀리면 ↑ (가장 먼저 조절)'),
    ('stanley.k',              2, '밀림이 남으면 ↑ — 라인 복귀 속도'),
    ('stanley.gain',           3, '1·2 이후에도 부족하면 소폭만 ↑ (흔들리면 즉시 되돌릴 것)'),
    ('stanley.ff_preview_sec', 4, '너무 일찍/늦게 꺾는 타이밍 문제일 때만'),
    ('stanley.ks',             5, '저속 떨림이 있을 때만'),
    ('stanley.steer_lpf',      5, '고주파 떨림이 있을 때만 (1.0 = 필터 없음)'),
]
GUIDE_DOC = 'docs/stanley_gain_tuning.md'

# ./tune.sh status 속도 섹션 — 현재 모드 강조
SPEED_GUIDE = [
    ('speed.hard_cap',        '물리 무관 천장 (0=끔). 물리면 grip_util 을 올려도 안 빨라진다'),
    ('limits.grip_util.q1',   './race.sh q1 만. 마찰 예산 η (v ∝ √η)'),
    ('limits.grip_util.q2',   './race.sh q2 만'),
    ('limits.grip_util.q3',   './race.sh q3 만'),
    ('limits.grip_util.race', './race.sh race 만'),
    ('limits.grip_util',      '전 모드 공통 η (숫자 하나일 때)'),
    ('speed.min_speed',       '하드웨어 바닥 (이 밑으로 안 내려감)'),
]

# 0 으로 두면 안 되는 키 — yaml 주석에 "절대 0으로 두지 말 것" 으로 명시돼 있다.
# 중심차분 접선은 무편향이라 곡률 FF 가 커브 조향을 전담한다. 0 이면 코너를 늦게 꺾는다.
FORBID_ZERO = {'stanley.ff_gain'}

# 한 번에 이 배수 이상 바꾸면 오타로 보고 막는다 (--force 로 무시)
MAX_JUMP_RATIO = 5.0

# ★ 배수 가드에서 제외할 키.
#   배수는 "게인" 에나 의미가 있다. 맵 좌표·각도는 부호가 바뀌고 0 근처도 정상값이라
#   3.78 → -0.15 같은 정상적인 갱신이 "0.0배 변경" 으로 걸려버린다.
#   (./race.sh savepose 가 여기 키들을 기록한다)
NO_RATIO_GUARD = {
    'localization.init_x', 'localization.init_y', 'localization.init_yaw',
    'mission.start_x', 'mission.start_y',
    'sim.sx', 'sim.sy', 'sim.stheta',
    'speed.hard_cap',   # 0 = 끔 이라 2→0 이 배수 가드에 걸리면 안 된다
}

C = {
    'dim':   '\033[2m',   'red':  '\033[31m', 'grn': '\033[32m',
    'ylw':   '\033[33m',  'cyn':  '\033[36m', 'bld': '\033[1m',
    'off':   '\033[0m',
}
if not sys.stdout.isatty():
    C = {k: '' for k in C}


def die(msg):
    print(f"{C['red']}[tune] {msg}{C['off']}", file=sys.stderr)
    sys.exit(1)


def load_cfg():
    with open(RACE_SCRIPT) as f:
        return yaml.safe_load(f)


def get_nested(cfg, dotted):
    """'stanley.gain' → 값. 없으면 KeyError."""
    node = cfg
    for part in dotted.split('.'):
        if not isinstance(node, dict) or part not in node:
            raise KeyError(dotted)
        node = node[part]
    return node


def all_keys(cfg):
    """설정 가능한 모든 'a.b' 키 목록 (스칼라 리프만)."""
    out = []
    for k, v in cfg.items():
        if isinstance(v, dict):
            out += [f'{k}.{k2}' for k2, v2 in v.items() if not isinstance(v2, (dict, list))]
        elif not isinstance(v, list):
            out.append(k)
    return out


# ---------- 줄 단위 외과 수술 ----------

def find_line(lines, dotted):
    """대상 키가 있는 줄 인덱스를 찾는다. 섹션 경계를 지켜서 오탐을 막는다.

    'speed.min_speed' 를 찾을 때 stanley 섹션의 동명 키를 잡으면 안 되므로,
    최상위 섹션 블록(들여쓰기 0 인 다음 키가 나오기 전까지) 안에서만 검색한다.
    """
    parts = dotted.split('.')

    if len(parts) == 1:
        pat = re.compile(r'^(' + re.escape(parts[0]) + r'):([ \t]*)(.*)$')
        for i, ln in enumerate(lines):
            m = pat.match(ln)
            if m and m.group(3).strip() and not m.group(3).lstrip().startswith('#'):
                return i, m
        return None, None

    if len(parts) >= 3:
        pidx, pm = find_line(lines, f'{parts[0]}.{parts[1]}')
        if pidx is None:
            return None, None
        val, _, _ = split_value_comment(pm.group(3))
        if val.strip().startswith('{') or val.strip():
            return pidx, pm
        parent_indent = len(pm.group(1)) - len(pm.group(1).lstrip())
        sub_pat = re.compile(r'^([ \t]+' + re.escape(parts[2]) + r'):([ \t]*)(.*)$')
        for j in range(pidx + 1, len(lines)):
            ln = lines[j]
            stripped = ln.strip()
            if not stripped or stripped.startswith('#'):
                continue
            indent = len(ln) - len(ln.lstrip())
            if indent <= parent_indent:
                break
            sm = sub_pat.match(ln)
            if sm:
                return j, sm
        return None, None

    sect, key = parts[0], parts[1]
    sect_pat = re.compile(r'^' + re.escape(sect) + r':[ \t]*(#.*)?$')
    start = None
    for i, ln in enumerate(lines):
        if sect_pat.match(ln):
            start = i + 1
            break
    if start is None:
        return None, None

    key_pat = re.compile(r'^([ \t]+' + re.escape(key) + r'):([ \t]*)(.*)$')
    for i in range(start, len(lines)):
        ln = lines[i]
        stripped = ln.strip()
        # 들여쓰기 0 의 새 최상위 키를 만나면 섹션 종료
        if stripped and not stripped.startswith('#') and not ln[:1].isspace():
            break
        m = key_pat.match(ln)
        if m:
            return i, m
    return None, None


def split_value_comment(rest):
    """'1.5    # 주석' → ('1.5', '# 주석', 7).

    셋째 값은 rest 안에서 '#' 이 시작하는 오프셋 — 주석 정렬 열을 유지하는 데 쓴다.
    따옴표 안의 # 는 주석이 아니다 (bound_dir: '' 같은 값 보호).
    """
    in_s = in_d = False
    for i, ch in enumerate(rest):
        if ch == "'" and not in_d:
            in_s = not in_s
        elif ch == '"' and not in_s:
            in_d = not in_d
        elif ch == '#' and not in_s and not in_d:
            return rest[:i].rstrip(), rest[i:], i
    return rest.rstrip(), '', -1


def format_value(v):
    """파이썬 값 → yaml 스칼라 표기."""
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, float):
        # repr 이 2.0 을 '2.0' 으로 유지한다. '2' 로 줄이면 파일의 다른 값(1.5, 3.0)과
        # 표기가 어긋나고 YAML 상 int 가 돼 버린다.
        return repr(round(v, 6))
    return str(v)


def merge_flow_map(rest, subkey, new_val):
    """인라인 딕셔너리 줄에서 **한 하위 키만** 바꾼 문자열을 돌려준다.

    '{q1: 0.5, q2: 0.5, q3: 0.5}' + ('q2', 0.2) → '{q1: 0.5, q2: 0.2, q3: 0.5}'

    ★ 왜 필요한가: find_line 은 'a.b' 두 단계까지만 줄을 찾는다.
      'limits.grip_util.q2' 처럼 세 단계가 오면 grip_util 줄을 찾아놓고
      **그 줄 전체를 스칼라로 덮어써** 나머지 모드가 통째로 사라졌다
      (2026-08-13: grip_util: {q1..race} → grip_util: 0.2 로 뭉개짐).
      트랜잭션 덕에 원본은 살았지만 KeyError 로 실패했다.
    돌려줄 수 없으면 None (블록 스타일 딕셔너리 등).
    """
    value_txt, _comment, _off = split_value_comment(rest)
    txt = value_txt.strip()
    if not (txt.startswith('{') and txt.endswith('}')):
        return None
    out, found = [], False
    for part in txt[1:-1].split(','):
        if ':' not in part:
            out.append(part.strip())
            continue
        k, v = part.split(':', 1)
        if k.strip() == subkey:
            out.append(f'{k.strip()}: {format_value(new_val)}')
            found = True
        else:
            out.append(f'{k.strip()}: {v.strip()}')
    return '{' + ', '.join(out) + '}' if found else None


def write_value(lines, idx, m, new_val):
    """주석과 정렬 열을 보존하며 값만 교체한 새 줄을 만든다."""
    orig = lines[idx]
    key_part, gap, rest = m.group(1), m.group(2), m.group(3)
    _, comment, hash_off = split_value_comment(rest)
    new_txt = format_value(new_val)
    head = f'{key_part}:{gap}{new_txt}'

    if comment:
        # 원래 '#' 이 있던 열을 그대로 유지 → 파일의 표 정렬이 안 깨진다.
        # 새 값이 더 길어 열을 넘기면 최소 공백 1칸.
        target_col = len(key_part) + 1 + len(gap) + hash_off
        new_line = head + ' ' * max(1, target_col - len(head)) + comment
    else:
        new_line = head

    return new_line + ('\n' if orig.endswith('\n') else '')


# ---------- 검증 ----------

def check_guards(dotted, old, new, force):
    problems = []

    if isinstance(old, bool) != isinstance(new, bool):
        problems.append(f'타입 불일치: 기존 {type(old).__name__} → 새 값 {type(new).__name__}')
    elif isinstance(old, (int, float)) and not isinstance(new, (int, float)):
        problems.append(f'숫자 자리에 숫자가 아닌 값: {new!r}')

    if isinstance(new, (int, float)) and not isinstance(new, bool):
        if dotted in FORBID_ZERO and float(new) == 0.0:
            problems.append(
                f'{dotted} 는 0 으로 두면 안 된다 — 곡률 피드포워드가 꺼져 코너를 늦게 꺾는다 '
                '(race_script.yaml 주석 참조)')
        rng = RANGES.get(dotted)
        if rng and not (rng[0] <= float(new) <= rng[1]):
            problems.append(f'권장 범위 {rng[0]}~{rng[1]} 밖의 값: {new}')
        if (dotted not in NO_RATIO_GUARD
                and isinstance(old, (int, float)) and old and float(old) != 0):
            ratio = abs(float(new) / float(old))
            if ratio > MAX_JUMP_RATIO or ratio < 1.0 / MAX_JUMP_RATIO:
                problems.append(
                    f'한 번에 {ratio:.1f}배 변경 ({old} → {new}) — 오타일 수 있다')

    if problems and not force:
        for p in problems:
            print(f"{C['red']}[tune] {dotted}: {p}{C['off']}", file=sys.stderr)
        return False
    if problems and force:
        for p in problems:
            print(f"{C['ylw']}[tune] (--force) {dotted}: {p}{C['off']}")
    return True


def parse_scalar(s):
    """CLI 문자열 → 파이썬 값 (yaml 규칙과 동일하게)."""
    try:
        v = yaml.safe_load(s)
    except yaml.YAMLError:
        return s
    return s if v is None and s.strip() != 'null' else v


# ---------- 이력 ----------

def log_change(entries):
    stamp = datetime.datetime.now().isoformat(timespec='seconds')
    with open(HISTORY, 'a') as f:
        for e in entries:
            f.write(json.dumps({'t': stamp, **e}, ensure_ascii=False) + '\n')


def show_log():
    if not os.path.exists(HISTORY):
        print(f"{C['dim']}(변경 이력 없음){C['off']}")
        return
    with open(HISTORY) as f:
        rows = [json.loads(l) for l in f if l.strip()]
    for r in rows[-40:]:
        t = r['t'].replace('T', ' ')
        print(f"  {C['dim']}{t}{C['off']}  {r['key']:28s} "
              f"{C['dim']}{r['old']}{C['off']} → {C['grn']}{r['new']}{C['off']}")


def do_undo():
    if os.path.exists(BACKUP_JSON):
        snap = json.load(open(BACKUP_JSON))
        for p, content in snap.items():
            with open(p, 'w') as f:
                f.write(content)
        os.remove(BACKUP_JSON)
        if os.path.exists(BACKUP):
            os.remove(BACKUP)
        if any(os.path.abspath(p) != os.path.abspath(RACE_SCRIPT) for p in snap):
            subprocess.run([sys.executable,
                            os.path.join(HERE, 'config', 'apply_vehicle.py'),
                            '--quiet'], check=False)
        print(f"{C['grn']}[tune] 직전 변경을 되돌렸다.{C['off']}")
        log_change([{'key': '(undo)', 'old': '-', 'new': '-'}])
        return
    if not os.path.exists(BACKUP):
        die('되돌릴 직전 변경이 없다 (.tune_backup.yaml 없음)')
    shutil.copy2(BACKUP, RACE_SCRIPT)
    os.remove(BACKUP)
    print(f"{C['grn']}[tune] 직전 변경을 되돌렸다.{C['off']}")
    log_change([{'key': '(undo)', 'old': '-', 'new': '-'}])


def do_diff():
    os.system(f'cd {HERE} && git --no-pager diff -- race_script.yaml')


# ---------- 보기 ----------

def git_baseline():
    """마지막 커밋 시점의 race_script.yaml. git 이 없거나 실패하면 None."""
    try:
        r = subprocess.run(['git', '-C', HERE, 'show', 'HEAD:race_script.yaml'],
                           capture_output=True, text=True, timeout=5)
        return yaml.safe_load(r.stdout) if r.returncode == 0 else None
    except Exception:
        return None


def recent_history(n=8):
    if not os.path.exists(HISTORY):
        return []
    try:
        with open(HISTORY) as f:
            return [json.loads(l) for l in f if l.strip()][-n:]
    except (OSError, ValueError):
        return []


def show_status(cfg):
    """현재 튜닝 상태 — 값 + 조절 우선순위 + 커밋 대비 변경분 + 최근 이력."""
    base = git_baseline()
    now = datetime.datetime.now().strftime('%m/%d %H:%M')

    print(f"\n  {C['bld']}튜닝 상태{C['off']}  {C['dim']}{now}{C['off']}")
    ctrl = cfg.get('controller')
    line = (f"  {C['dim']}mode{C['off']} {cfg.get('mode')}   "
            f"{C['dim']}controller{C['off']} {ctrl}   "
            f"{C['dim']}map{C['off']} {cfg.get('map_name')}")
    print(line)
    if ctrl not in ('stanley', 'auto'):
        print(f"  {C['ylw']}※ controller 가 '{ctrl}' 라 아래 Stanley 게인은 "
              f"이번 주행에 쓰이지 않는다{C['off']}")
    print()

    changed_any = []
    print(f"  {C['cyn']}{C['bld']}Stanley 조향 게인{C['off']}"
          f"   {C['dim']}({GUIDE_DOC}){C['off']}")
    for key, prio, hint in TUNING_GUIDE:
        try:
            cur = get_nested(cfg, key)
        except KeyError:
            continue
        name = key.split('.', 1)[1]

        mark, plain, val = ' ', str(cur), f'{C["bld"]}{cur}{C["off"]}'
        if base is not None:
            try:
                old = get_nested(base, key)
                if old != cur:
                    mark = '*'
                    plain = f'{cur}  ({old} → {cur})'
                    val += f"  {C['ylw']}({old} → {cur}){C['off']}"
                    changed_any.append(key)
            except KeyError:
                pass

        # 색코드는 길이에 포함되면 안 되므로 plain 기준으로 패딩을 계산한다
        pad = ' ' * max(2, 24 - len(plain))
        print(f"   {C['dim']}{prio}{C['off']} {mark} {name:16s} {val}{pad}"
              f"{C['dim']}{hint}{C['off']}")

    # 속도 섹션
    mode = cfg.get('mode', '')
    eff = overlay_cfg(cfg)
    speed_shown = set()
    print(f"\n  {C['cyn']}{C['bld']}속도{C['off']}"
          f"   {C['dim']}(./tune.sh i  또는  ./tune.sh cap=3){C['off']}")
    for dotted, hint in SPEED_GUIDE:
        try:
            cur = get_nested(eff, dotted)
        except KeyError:
            continue
        if isinstance(cur, dict):
            continue
        name = dotted.split('.', 1)[-1]
        speed_shown.add(dotted)
        is_active = dotted.endswith('.' + mode) or dotted == 'limits.grip_util'

        mark, plain, val = ' ', str(cur), f'{C["bld"]}{cur}{C["off"]}'
        if base is not None and target_file(dotted)[0] == RACE_SCRIPT:
            try:
                old = get_nested(base, dotted)
                if old != cur:
                    mark = '*'
                    plain = f'{cur}  ({old} → {cur})'
                    val += f"  {C['ylw']}({old} → {cur}){C['off']}"
                    changed_any.append(dotted)
            except KeyError:
                pass

        active_tag = f"  {C['grn']}◀ 현재 모드{C['off']}" if is_active else ''
        pad = ' ' * max(2, 22 - len(plain))
        print(f"   {mark} {name:14s} {val}{pad}{C['dim']}{hint}{C['off']}{active_tag}")

    # 가이드에 없는 키가 바뀌었으면 따로 보여준다 (gap_follower 등)
    if base is not None:
        others = []
        shown_keys = {g[0] for g in TUNING_GUIDE} | speed_shown
        for k in all_keys(cfg):
            if k in shown_keys:
                continue
            try:
                if get_nested(base, k) != get_nested(cfg, k):
                    others.append((k, get_nested(base, k), get_nested(cfg, k)))
            except KeyError:
                continue
        if others:
            print(f"\n  {C['cyn']}{C['bld']}그 밖의 변경{C['off']}")
            for k, o, n in others:
                print(f"     * {k:26s} {C['dim']}{o}{C['off']} → "
                      f"{C['ylw']}{n}{C['off']}")
            changed_any += [k for k, _, _ in others]

    print()
    if base is None:
        print(f"  {C['dim']}(git 기준값을 못 읽어 변경 표시를 생략했다){C['off']}")
    elif changed_any:
        print(f"  {C['dim']}* = 마지막 커밋 대비 변경됨 ({len(changed_any)}개). "
              f"되돌리려면  git checkout race_script.yaml{C['off']}")
    else:
        print(f"  {C['grn']}커밋된 값 그대로 — 변경 없음{C['off']}")

    hist = recent_history()
    if hist:
        print(f"\n  {C['cyn']}{C['bld']}최근 변경{C['off']}")
        for r in hist:
            t = r['t'].split('T')[-1][:5]
            print(f"     {C['dim']}{t}{C['off']}  {r['key']:26s} "
                  f"{C['dim']}{r['old']}{C['off']} → {C['grn']}{r['new']}{C['off']}")
    print()


def show(cfg, only=None):
    if only is None:
        head = '  '.join(
            f"{C['dim']}{k}{C['off']}={C['bld']}{cfg.get(k)}{C['off']}"
            for k in VIEW_SCALARS if k in cfg)
        print(f'\n  {head}\n')
        sections = [s for s in VIEW_SECTIONS if isinstance(cfg.get(s), dict)]
    else:
        if only not in cfg or not isinstance(cfg[only], dict):
            die(f'그런 섹션이 없다: {only}')
        sections = [only]

    for s in sections:
        print(f"{C['cyn']}{C['bld']}{s}{C['off']}")
        for k, v in cfg[s].items():
            if isinstance(v, (dict, list)):
                continue
            dotted = f'{s}.{k}'
            rng = RANGES.get(dotted)
            hint = f"{C['dim']}  [{rng[0]}~{rng[1]}]{C['off']}" if rng else ''
            print(f'    {k:22s} {C["bld"]}{v}{C["off"]}{hint}')
        print()


def speed_knobs(cfg):
    mode = cfg.get('mode', '')
    items = []
    try:
        cap = get_nested(cfg, 'speed.hard_cap')
        items.append({
            'key': 'speed.hard_cap', 'name': 'hard_cap', 'value': cap,
            'unit': 'm/s', 'file': os.path.basename(target_file('speed.hard_cap')[0]),
            'hint': '전 모드 공통 천장. 0=끔', 'active': False,
        })
    except KeyError:
        pass
    gu_path = target_file('limits.grip_util')[0]
    try:
        gu = get_nested(cfg, 'limits.grip_util')
    except KeyError:
        gu = None
    if isinstance(gu, dict):
        when = {
            'q1':   './race.sh q1 만',
            'q2':   './race.sh q2 만',
            'q3':   './race.sh q3 만',
            'race': './race.sh race 만',
        }
        for m in ('q1', 'q2', 'q3', 'race'):
            if m not in gu:
                continue
            items.append({
                'key': f'limits.grip_util.{m}', 'name': f'η {m}',
                'value': gu[m], 'unit': '',
                'file': os.path.basename(gu_path),
                'hint': when[m], 'active': m == mode,
            })
    elif gu is not None:
        items.append({
            'key': 'limits.grip_util', 'name': 'grip_util',
            'value': gu, 'unit': '',
            'file': os.path.basename(gu_path),
            'hint': '전 모드가 이 숫자 하나 (딕셔너리 아님)', 'active': True,
        })
    try:
        ms = get_nested(cfg, 'speed.min_speed')
        items.append({
            'key': 'speed.min_speed', 'name': 'min_speed',
            'value': ms, 'unit': 'm/s',
            'file': os.path.basename(target_file('speed.min_speed')[0]),
            'hint': '하드웨어 바닥', 'active': False,
        })
    except KeyError:
        pass
    return items


def print_speed_table(cfg):
    knobs = speed_knobs(cfg)
    car = _car_name() or '시뮬'
    print(f"\n  {C['bld']}속도 노브{C['off']}  {C['dim']}({car})  "
          f"mode={cfg.get('mode')}  controller={cfg.get('controller')}{C['off']}")
    print(f"  {C['dim']}η q1 / q2 / race 는 동시에 안 먹는다. "
          f"./race.sh <모드> 가 고른 키 하나만.{C['off']}")
    try:
        cap = float(get_nested(cfg, 'speed.hard_cap') or 0)
    except KeyError:
        cap = 0
    if cap:
        print(f"  {C['ylw']}★ hard_cap={cap} 가 켜져 있다 — "
              f"어느 모드든 이 속도를 못 넘는다.{C['off']}")
    gu = None
    try:
        gu = get_nested(cfg, 'limits.grip_util')
    except KeyError:
        pass
    if gu is not None and not isinstance(gu, dict):
        print(f"  {C['ylw']}★ grip_util={gu} 숫자 하나라 q1/q2/race 가 전부 같다. "
              f"모드별로 나누려면 차량 파일을 "
              f"{{q1: {gu}, q2: {gu}, q3: {gu}, race: {gu}}} 로.{C['off']}")
    print(f"  {C['dim']}{'─' * 64}{C['off']}")
    for i, k in enumerate(knobs, 1):
        mark = f"  {C['grn']}◀ 지금 yaml mode{C['off']}" if k['active'] else ''
        unit = f" {k['unit']}" if k['unit'] else ''
        print(f"  {C['cyn']}{i}{C['off']}  {k['name']:12s} "
              f"{C['bld']}{k['value']}{C['off']}{unit:6s}  "
              f"{C['dim']}{k['hint']}{C['off']}{mark}")
    print(f"  {C['dim']}예: 1 3.0   2 +=0.1   cap=4   q1=0.6   q2+=0.1   race=0.5{C['off']}")
    print(f"  {C['dim']}q 종료   u 되돌리기   ? 설명   ! 가드 무시{C['off']}\n")
    return knobs


def show_header_help():
    """tune.sh 헤더 주석을 그대로 보여 준다. 줄 번호에 묶지 않는다."""
    path = os.path.join(HERE, 'tune.sh')
    with open(path) as f:
        for ln in f:
            if ln.startswith('set -e'):
                break
            if ln.startswith('#!'):
                continue
            if ln.startswith('# '):
                print(ln[2:], end='')
            elif ln.startswith('#'):
                print(ln[1:], end='')
            else:
                print(ln, end='')


def expand_alias(token, cfg):
    m = re.match(r'^([\w.]+)\s*((?:\+=|-=|=)\s*.*)?$', token)
    if not m:
        return token
    key, rest = m.group(1), m.group(2) or ''
    if key in ('eta', 'grip'):
        path, fkey = target_file('limits.grip_util')
        src = yaml.safe_load(open(path)) if os.path.exists(path) else cfg
        try:
            gu = get_nested(src, fkey if path != RACE_SCRIPT else 'limits.grip_util')
        except Exception:
            gu = None
        if isinstance(gu, dict):
            key = f'limits.grip_util.{cfg.get("mode", "q1")}'
        else:
            key = 'limits.grip_util'
    else:
        key = SPEED_ALIASES.get(key, key)
    return key + rest


def cmd_interactive():
    if not sys.stdin.isatty():
        print_speed_table(overlay_cfg(load_cfg()))
        print(f"{C['dim']}대화형은 SSH 들어가서  ./tune.sh i{C['off']}")
        return
    while True:
        cfg = overlay_cfg(load_cfg())
        knobs = print_speed_table(cfg)
        try:
            line = input(f"  {C['cyn']}>{C['off']} ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if line in ('q', 'quit', 'exit'):
            return
        if not line:
            continue
        if line in ('?', 'h', 'help'):
            show_header_help()
            continue
        if line in ('u', 'undo'):
            try:
                do_undo()
            except SystemExit:
                pass
            continue
        force = line.startswith('!')
        if force:
            line = line[1:].strip()
        args = None
        m = re.match(r'^(\d+)\s*((?:\+=|-=|=)?)\s*(.*)$', line)
        if m and 1 <= int(m.group(1)) <= len(knobs):
            n = int(m.group(1))
            op = m.group(2) or '='
            raw = m.group(3).strip()
            if not raw:
                try:
                    raw = input(f"  {knobs[n-1]['name']} "
                                f"[{knobs[n-1]['value']}] 새 값: ").strip()
                except (EOFError, KeyboardInterrupt):
                    print()
                    continue
                if not raw:
                    continue
                op = '='
            args = [f"{knobs[n-1]['key']}{op}{raw}"]
        else:
            expanded = expand_alias(line, cfg)
            if re.search(r'(\+=|-=|=)', expanded):
                args = [expanded]
        if not args:
            print(f"  {C['dim']}예:  1 3.0   2 +=0.1   cap=4{C['off']}")
            continue
        if force:
            args = ['--force'] + args
        try:
            main(args)
        except SystemExit:
            pass


# ---------- 메인 ----------

def main(argv):
    force = '--force' in argv
    argv = [a for a in argv if a != '--force']

    if any(a in ('status', '-s', '--status') for a in argv):
        return show_status(load_cfg())
    if '--undo' in argv:
        return do_undo()
    if '--log' in argv:
        return show_log()
    if '--diff' in argv:
        return do_diff()
    if any(a in ('-h', '--help') for a in argv):
        return show_header_help()
    if any(a in ('i', '-i', '--interactive') for a in argv):
        return cmd_interactive()

    if not os.path.exists(RACE_SCRIPT):
        die(f'race_script.yaml 을 찾을 수 없다: {RACE_SCRIPT}')

    cfg = load_cfg()
    argv = [expand_alias(a, cfg) for a in argv]
    assigns = [a for a in argv if re.search(r'(\+=|-=|=)', a)]
    queries = [a for a in argv if a not in assigns]

    # 인자 없음 → 전체 보기
    if not assigns and not queries:
        return show(cfg)
    if not assigns and queries == ['speed'] and sys.stdin.isatty():
        return cmd_interactive()

    # 조회
    if not assigns:
        for q in queries:
            if q in cfg and isinstance(cfg[q], dict):
                show(cfg, only=q)
            else:
                path, fkey = target_file(q)
                try:
                    src = cfg if path == RACE_SCRIPT else yaml.safe_load(open(path))
                    print(get_nested(src, fkey))
                except KeyError:
                    suggest(cfg, q)
                    sys.exit(1)
        return

    # ---- 설정 ----
    # ★ 대상 파일이 키마다 다르다. 차량별 값은 config/vehicle_<차>.yaml 로 간다 —
    #   race_script.yaml 에 써봐야 overlay 가 덮어써서 주행에 안 먹는다.
    buffers = {}          # 경로 → 줄 목록
    def buf(path):
        if path not in buffers:
            with open(path) as f:
                buffers[path] = f.readlines()
        return buffers[path]

    pending, entries = [], []
    for a in assigns:
        m = re.match(r'^([\w.]+)\s*(\+=|-=|=)\s*(.*)$', a)
        if not m:
            die(f'형식을 알 수 없다: {a}   (예: stanley.gain=1.2)')
        key, op, raw = m.group(1), m.group(2), m.group(3).strip()

        if key in DERIVED_KEYS:
            die(f'{key} 는 직접 고칠 수 없다.\n  {DERIVED_KEYS[key]}')

        path, fkey = target_file(key)
        if path == RACE_SCRIPT:
            try:
                old = get_nested(cfg, key)
            except KeyError:
                suggest(cfg, key)
                sys.exit(1)
        else:
            vcfg = yaml.safe_load(open(path))
            try:
                old = get_nested(vcfg, fkey)
            except KeyError:
                die(f'{key} 는 차량 파일 소관인데 {os.path.basename(path)} 에 '
                    f'{fkey} 가 없다 — 그 파일에 먼저 추가할 것')

        if op == '=':
            new = parse_scalar(raw)
        else:
            if not isinstance(old, (int, float)) or isinstance(old, bool):
                die(f'{key} 는 숫자가 아니라 {op} 를 쓸 수 없다 (현재 {old!r})')
            try:
                delta = float(raw)
            except ValueError:
                die(f'{op} 뒤에는 숫자가 와야 한다: {raw!r}')
            new = float(old) + (delta if op == '+=' else -delta)
            new = round(new, 6)

        if old == new:
            print(f"{C['dim']}[tune] {key} 는 이미 {old} — 변경 없음{C['off']}")
            continue
        if not check_guards(key, old, new, force):
            die('중단했다. 의도한 값이면 --force 를 붙일 것.')

        lines = buf(path)
        idx, lm = find_line(lines, fkey)
        if idx is None:
            die(f'{fkey} 의 줄을 {os.path.basename(path)} 에서 찾지 못했다 '
                f'— 수동으로 확인할 것')
        # 세 단계 키(예: tuning.grip_util.q2)는 그 줄이 인라인 딕셔너리다.
        # 줄 전체를 덮으면 형제 키가 사라지므로 해당 항목만 바꿔 끼운다.
        line_val = new
        if len(fkey.split('.')) >= 3:
            merged = merge_flow_map(lm.group(3), fkey.split('.')[-1], new)
            if merged is not None:
                line_val = merged
            elif lm.group(1).strip() != fkey.split('.')[-1]:
                die(f'{fkey} 는 모드별 딕셔너리가 아니라 통째로만 바꿀 수 있다 '
                    f'(./tune.sh limits.grip_util={new})')
        pending.append((path, idx, lm, line_val))
        entries.append({'key': key, 'old': old, 'new': new,
                        'file': path, 'fkey': fkey})

    if not pending:
        return

    snap = {p: open(p).read() for p in buffers}
    with open(BACKUP_JSON, 'w') as f:
        json.dump(snap, f)
    shutil.copy2(RACE_SCRIPT, BACKUP)
    for path, idx, lm, new in pending:
        buffers[path][idx] = write_value(buffers[path], idx, lm, new)

    # 파일마다 임시 파일에 쓰고 검증한 뒤 한꺼번에 교체한다.
    # 하나라도 실패하면 전부 원본 유지 — 절반만 적용되는 게 제일 나쁘다.
    tmps = {}
    try:
        for path, lines in buffers.items():
            tmp = path + '.tmp'
            with open(tmp, 'w') as f:
                f.writelines(lines)
            tmps[path] = tmp
            try:
                after = yaml.safe_load(open(tmp))
            except yaml.YAMLError as ex:
                die(f'{os.path.basename(path)} 수정 결과가 유효한 YAML 이 아니다 '
                    f'— 원본 유지.\n{ex}')
            for e in entries:
                if e['file'] != path:
                    continue
                got = get_nested(after, e['fkey'])
                if isinstance(e['new'], float) and isinstance(got, (int, float)):
                    ok = abs(float(got) - e['new']) < 1e-9
                else:
                    ok = got == e['new']
                if not ok:
                    die(f"{e['key']} 가 {e['new']} 로 안 들어갔다 "
                        f"(읽으니 {got!r}) — 원본 유지")
    except SystemExit:
        for t in tmps.values():
            if os.path.exists(t):
                os.remove(t)
        raise
    for path, tmp in tmps.items():
        os.replace(tmp, path)

    # 차량 파일을 고쳤으면 vesc.yaml/overlay 를 다시 만들어야 반영된다
    if any(e['file'] != RACE_SCRIPT for e in entries):
        subprocess.run([sys.executable,
                        os.path.join(HERE, 'config', 'apply_vehicle.py'),
                        '--quiet'], check=False)

    log_change(entries)

    for e in entries:
        where = ('' if e['file'] == RACE_SCRIPT
                 else f"  {C['dim']}→ {os.path.basename(e['file'])}"
                      f" ({e['fkey']}){C['off']}")
        print(f"  {e['key']:28s} {C['dim']}{e['old']}{C['off']} → "
              f"{C['grn']}{C['bld']}{e['new']}{C['off']}{where}")
    if any(e['file'] != RACE_SCRIPT for e in entries):
        print(f"\n{C['dim']}  ★ 차량별 값이라 이 차({_car_name()})의 파일에 썼다 —"
              f" race_script.yaml 에 쓰면 overlay 가 덮어써서 안 먹는다.{C['off']}")
    print(f"\n{C['dim']}  다음 ./race.sh 실행부터 적용된다 (재빌드 불필요){C['off']}")


def suggest(cfg, key):
    keys = all_keys(cfg)
    tail = key.split('.')[-1]
    # 부분일치(gap_follower.steer_gain 류) + 편집거리(gian → gain 류) 둘 다 본다
    near = [k for k in keys if tail and tail in k]
    near += [k for k in difflib.get_close_matches(key, keys, n=5, cutoff=0.5)
             if k not in near]
    print(f"{C['red']}[tune] 그런 키가 없다: {key}{C['off']}", file=sys.stderr)
    if near:
        print(f"{C['dim']}       혹시 이건가: {', '.join(near[:8])}{C['off']}", file=sys.stderr)
    else:
        print(f"{C['dim']}       ./tune.sh 로 전체 목록 확인{C['off']}", file=sys.stderr)


if __name__ == '__main__':
    main(sys.argv[1:])
