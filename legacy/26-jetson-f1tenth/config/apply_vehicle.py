#!/usr/bin/env python3
"""apply_vehicle.py — 차량별 캘리브레이션을 실제 설정 파일로 펼친다.

왜 있나
───────
차가 두 대인데 저장소는 하나다. `config/vesc.yaml` 하나에 두 차의 값이 들어갈
수 없고, 잘못 섞이면 **차가 뒤로 간다** (2호차는 모터 결선이 반대라
speed_to_erpm_gain 이 음수다. 그 값이 1호차로 넘어가면 그대로 후진한다).

브랜치로 나누는 방법도 있지만, 그러면 코드까지 갈라져 같은 버그를 두 번
고치게 된다 — 실제로 2026-08-09 에 bench(box) 안전 버그를 두 사람이 각자
디버깅했다. 그래서 **코드는 합치고 값만 분리**한다.

    config/vesc_base.yaml       배관(포트·스무더·프레임). 두 차 공용
    config/vehicle_car1.yaml    1호차가 재서 나온 값
    config/vehicle_car2.yaml    2호차
    .car                        "car1" 또는 "car2" 한 줄. .gitignore
        ↓ 이 스크립트
    config/vesc.yaml            ★ 자동 생성. .gitignore — git 에 없으니
                                  잘못 커밋할 방법 자체가 없다
    config/.vehicle_overlay.yaml  race_script.yaml 위에 덮을 값 (launch 가 읽음)

★ 매핑 지식은 여기 한 곳에만 둔다.
  "vehicle 파일의 어느 키가 race_script 의 어디로 가는가" 를 launch 와 여기
  양쪽에 두면 반드시 어긋난다. launch 는 overlay 를 통째로 deep-merge 만 한다.

사용:
    python3 config/apply_vehicle.py            # .car 를 읽어 적용
    python3 config/apply_vehicle.py --car car2 # 명시
    python3 config/apply_vehicle.py --check    # 생성만 하고 기존과 비교(쓰지 않음)
"""
import argparse
import os
import re
import sys
import time

import yaml

_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_DIR)

BASE     = os.path.join(_DIR, 'vesc_base.yaml')
OUT_VESC = os.path.join(_DIR, 'vesc.yaml')
OUT_OVL  = os.path.join(_DIR, '.vehicle_overlay.yaml')
CARFILE  = os.path.join(_REPO, '.car')

# vesc_base.yaml 의 자리표시자 ↔ vehicle 파일의 키
VESC_KEYS = [
    ('__SPEED_TO_ERPM_GAIN__',            'vesc.speed_to_erpm_gain'),
    ('__STEERING_ANGLE_TO_SERVO_GAIN__',  'vesc.steering_angle_to_servo_gain'),
    ('__STEERING_ANGLE_TO_SERVO_OFFSET__','vesc.steering_angle_to_servo_offset'),
    ('__SERVO_MIN__',                     'vesc.servo_min'),
    ('__SERVO_MAX__',                     'vesc.servo_max'),
    ('__WHEELBASE__',                     'geometry.wheelbase'),
]

# vehicle 파일 키 → race_script.yaml 의 어느 자리로 가는가
OVERLAY_MAP = [
    ('geometry.wheelbase', 'pure_pursuit.wheelbase'),
    ('geometry.mass',      'map.mass'),
    ('geometry.lf',        'map.lf'),
    ('geometry.lr',        'map.lr'),
    ('tire.mu',            'map.mu'),
    ('tire.pac_b',         'map.pac_b'),
    ('tire.pac_c',         'map.pac_c'),
    ('tire.pac_d',         'map.pac_d'),
    ('tire.pac_e',         'map.pac_e'),
    ('tuning.l1_min',      'map.l1_min'),
    ('tuning.l1_k',        'map.l1_k'),
    ('tuning.steer_alpha', 'map.steer_alpha'),
    ('tuning.slip_gain',   'map.slip_gain'),
    ('tuning.speed_lag',   'limits.speed_lag'),
    ('tuning.a_lon_acc',   'limits.a_lon_acc'),
    ('tuning.a_lon_brk',   'limits.a_lon_brk'),
    # 출발 가속 제한 — 타이어·구동계·하중이동에 걸린 값이라 차마다 다르다
    ('tuning.launch_acc',  'limits.launch_acc'),
    ('tuning.launch_time', 'limits.launch_time'),
    ('tuning.grip_util',   'limits.grip_util'),
    # ★ 젯슨 IP 는 차마다 다르다 (1호차 .57 / 2호차 .77). CycloneDDS 가 유니캐스트로
    #   인사할 고정 피어 목록이라, 남의 차 IP 가 들어가면 노트북 RViz 가 먹통이 된다.
    #   race_script.yaml 에 두면 두 차가 머지할 때마다 이 한 줄이 충돌한다.
    ('network.peers',      'network.peers'),
    # ★ ROS_DOMAIN_ID. 두 차를 같은 공유기에서 동시에 돌릴 때 DDS 레벨 격리.
    #   race.sh 가 dds_env.sh 를 통해 export 한다.
    ('network.domain_id',  'network.domain_id'),
    # ★ /odom 병진 속도 부호. 모터 결선이 차마다 달라(2호차는 반대) 필요하다.
    #   imu_odom 이 이 값으로 위치를 적분해 odom→base_link TF 를 낸다 = SLAM 입력.
    ('localization.speed_sign', 'localization.speed_sign'),
]
# joystick 은 키 이름이 같으므로 통째로 옮긴다
OVERLAY_PASSTHROUGH = ['joystick']


def dig(d, dotted, default=KeyError):
    cur = d
    for k in dotted.split('.'):
        if not isinstance(cur, dict) or k not in cur:
            if default is KeyError:
                raise KeyError(dotted)
            return default
        cur = cur[k]
    return cur


def put(d, dotted, val):
    keys = dotted.split('.')
    for k in keys[:-1]:
        d = d.setdefault(k, {})
    d[keys[-1]] = val


def steer_limits(v):
    """서보 매핑에서 실제 조향 한계를 유도한다.

    ★ max_steer 를 사람이 따로 적으면 반드시 어긋난다. 2026-08-09 에 실제로
      그랬다 — steer_trim 이 offset 을 0.425 → 0.3954 로 옮겼는데 max_steer
      0.36 은 그대로여서, 좌 조향 지령의 **25.8% 가 vesc_driver 에서 조용히
      잘려나갔다.** 그래서 여기서 계산해 내려보낸다.
    """
    import math
    g   = float(dig(v, 'vesc.steering_angle_to_servo_gain'))
    off = float(dig(v, 'vesc.steering_angle_to_servo_offset'))
    lo  = float(dig(v, 'vesc.servo_min'))
    hi  = float(dig(v, 'vesc.servo_max'))
    wb  = float(dig(v, 'geometry.wheelbase'))
    d_lo = (lo - off) / g          # servo_min 쪽
    d_hi = (hi - off) / g          # servo_max 쪽
    left, right = max(d_lo, d_hi), min(d_lo, d_hi)   # 좌(+) / 우(−)
    lim = min(abs(left), abs(right))
    return left, right, lim, wb / math.tan(lim) if lim > 1e-6 else float('inf')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--car', default=None)
    ap.add_argument('--check', action='store_true',
                    help='쓰지 않고 기존 vesc.yaml 과 값이 같은지만 본다')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    car = args.car
    if car is None:
        if not os.path.exists(CARFILE):
            sys.exit(
                f'✗ {CARFILE} 이 없습니다.\n'
                f'  이 젯슨이 몇 호차인지 한 줄로 적어주세요:\n'
                f'      echo car1 > {CARFILE}      (또는 car2)\n'
                f'  ※ 기본값으로 조용히 넘어가지 않습니다 — 값이 틀리면 차가 뒤로 갑니다.')
        car = open(CARFILE).read().strip()
    vpath = os.path.join(_DIR, f'vehicle_{car}.yaml')
    if not os.path.exists(vpath):
        avail = sorted(f[8:-5] for f in os.listdir(_DIR)
                       if f.startswith('vehicle_') and f.endswith('.yaml'))
        sys.exit(f'✗ 차량 파일 없음: {vpath}\n  가능한 값: {", ".join(avail)}')

    v = yaml.safe_load(open(vpath))
    base = open(BASE).read()

    # ── ① vesc.yaml 생성 ──────────────────────────────────────────────
    body = base
    for token, key in VESC_KEYS:
        try:
            val = dig(v, key)
        except KeyError:
            sys.exit(f'✗ {vpath} 에 {key} 가 없습니다 (vesc_base.yaml 이 요구)')
        body = body.replace(token, repr(float(val)))
    left, right, lim, rmin = steer_limits(v)
    import math
    header = (
        '# ' + '═' * 68 + '\n'
        '#  ⚠ 자동 생성 파일 — 직접 고치지 마세요. 다음 실행 때 덮어씁니다.\n'
        f'#     고칠 곳:  config/vehicle_{car}.yaml\n'
        '#\n'
        f'#  차량   : {car}\n'
        f'#  생성   : {time.strftime("%Y-%m-%d %H:%M:%S")}\n'
        f'#  원본   : config/vesc_base.yaml + config/vehicle_{car}.yaml\n'
        f'#  유도값 : 좌 {math.degrees(left):+.2f}° / 우 {math.degrees(right):+.2f}°'
        f'  → max_steer {lim:.4f}rad, R_min {rmin:.3f}m\n'
        '# ' + '═' * 68 + '\n')
    out = header + body

    # ── ② race_script 덮어쓰기 overlay 생성 ───────────────────────────
    ovl = {}
    for src, dst in OVERLAY_MAP:
        try:
            put(ovl, dst, dig(v, src))
        except KeyError:
            pass                       # 없으면 race_script.yaml 값을 그대로 쓴다
    for k in OVERLAY_PASSTHROUGH:
        if k in v:
            ovl.setdefault(k, {}).update(v[k])
    put(ovl, 'pure_pursuit.max_steer', round(lim, 4))    # ★ 계산값이 이긴다

    if args.check:
        if not os.path.exists(OUT_VESC):
            print('기존 vesc.yaml 없음 — 비교 불가'); return 0
        old = yaml.safe_load(open(OUT_VESC))
        new = yaml.safe_load(out)
        same = old == new
        print(('✓ 동일' if same else '✗ 다름') + f'  ({car} 기준)')
        if not same:
            for sec in set(old) | set(new):
                o, n = old.get(sec, {}), new.get(sec, {})
                op, np_ = o.get('ros__parameters', {}), n.get('ros__parameters', {})
                for k in set(op) | set(np_):
                    if op.get(k) != np_.get(k):
                        print(f'   {sec}.{k}: {op.get(k)!r} → {np_.get(k)!r}')
        return 0 if same else 1

    with open(OUT_VESC, 'w') as f:
        f.write(out)
    with open(OUT_OVL, 'w') as f:
        f.write(f'# 자동 생성 ({car}) — race_script.yaml 위에 덮어쓴다. 고치지 말 것.\n')
        yaml.safe_dump(ovl, f, allow_unicode=True, sort_keys=True)

    # ── race_script.yaml 과 값이 갈라졌는지 본다 ──────────────────────
    # 같은 키가 두 군데 있으면 반드시 갈라진다. race_script 쪽은 시뮬(노트북)이
    # 읽는 기본값이라 지울 수 없으므로, **다르면 소리를 낸다.**
    # 이걸 안 하면 "race_script 를 고쳤는데 실차에서 효과가 없다" 가 된다 —
    # 2026-08-09 에만 이 형태의 함정이 다섯 번 나왔다.
    rs_path = os.path.join(_REPO, 'race_script.yaml')
    diverged = []
    if os.path.exists(rs_path):
        rs = yaml.safe_load(open(rs_path)) or {}

        def cmp(o, r, pre=''):
            for k, v in o.items():
                key = f'{pre}.{k}' if pre else k
                rv = r.get(k) if isinstance(r, dict) else None
                if isinstance(v, dict):
                    cmp(v, rv if isinstance(rv, dict) else {}, key)
                elif rv is not None and rv != v:
                    diverged.append((key, rv, v))
        cmp(ovl, rs)
        # ★ max_steer 는 **설계상** 계산값이라 race_script 와 달라야 정상이다.
        #   (servo_min−offset)/(−gain) 에서 유도한다 — 손으로 적은 값과 다른 게
        #   이 구조의 목적이다. 경고로 띄우면 진짜 경고가 묻힌다.
        DERIVED = {'pure_pursuit.max_steer'}
        diverged = [d for d in diverged if d[0] not in DERIVED]

    if not args.quiet:
        n = sum(len(x) if isinstance(x, dict) else 1 for x in ovl.values())
        print(f'[vehicle] {car}  (config/vehicle_{car}.yaml)')
        print(f'[vehicle]   vesc: erpm_gain {dig(v, "vesc.speed_to_erpm_gain")}  '
              f'servo {dig(v, "vesc.servo_min")}~{dig(v, "vesc.servo_max")}  '
              f'offset {dig(v, "vesc.steering_angle_to_servo_offset")}')
        print(f'[vehicle]   조향 한계 좌 {math.degrees(left):+.2f}° / '
              f'우 {math.degrees(right):+.2f}° → max_steer {lim:.4f}, R_min {rmin:.3f}m')
        print(f'[vehicle]   race_script 덮어쓴 키 {n}개')
        # 아직 안 잰 값 경고 — 주석에 '미측정' 이 있으면 센다
        # 표시 두 가지를 구분한다:
        #   미측정 — 이 차에서 아직 안 잰 값 (추정치일 수 있다)
        #   차용   — 다른 차 값을 그대로 가져온 것 (반드시 재야 한다)
        raw = open(vpath).read()
        todo  = len(re.findall(r'미측정', raw))
        borrow = len(re.findall(r'\(1호차 값\)|1호차에서 빌려|차용', raw))
        if borrow:
            print(f'[vehicle]   ⚠ 다른 차에서 빌려온 값 {borrow}개 — 이 차에서 재야 합니다')
        if todo:
            print(f'[vehicle]   · 미측정(추정치) 표시 {todo}개')
        if diverged:
            print(f'[vehicle]   ⚠ race_script.yaml 과 다른 값 {len(diverged)}개 '
                  f'— **차량 파일이 이깁니다.** race_script 쪽을 고쳐도 안 먹습니다:')
            for k, rv, vv in diverged:
                print(f'[vehicle]        {k}: race_script={rv} → 실제 적용 {vv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
