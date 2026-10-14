#!/usr/bin/env python3
"""
clean_map_auto.py — SLAM 맵을 **화면 없이** 정제한다 (clean_map.py 의 비대화형 판).

왜 있나
───────
clean_map.py 는 마우스로 폴리곤을 찍는 GUI 라 디스플레이가 필요하다. 맥북에서
SSH 로만 붙어 작업할 때는 못 쓴다. 그런데 우리 맵에서 지워야 할 것은 대부분
**트랙 경계 틈으로 빔이 새어 나가 만든 가는 빛살**이고, 이건 모양이 뚜렷해서
자동으로 걸러진다:

    빛살   폭 1~3px 의 가는 선   → opening 으로 사라진다
    트랙   폭 1m(=20px) 이상     → opening 후에도 남는다

그래서 opening 으로 가는 것을 지우고, 남은 것 중 **가장 큰 연결 성분**만
트랙으로 채택한다. 트랙은 닫힌 복도라 하나의 성분으로 이어져 있다.

출력 형식
─────────
기존 `_clean` 맵과 같게 맞춘다 — **미지(205)를 남기지 않는다.**
    자유(254)  채택된 트랙 복도
    벽(0)      그 밖 전부
clean_map.py 의 "외부 폴리곤 밖은 전부 벽" 과 같은 규약이고, 이렇게 해야
경계에 틈이 없어 gen_raceline 이 트랙을 추출할 수 있다.

사용법
──────
    ./race.sh clean                 ← 대화형 (맵 고르고, 결과 보고, 반경 조정)
    ./race.sh clean 0814-1          ← 맵만 지정, 나머지는 대화형

    python3 .../clean_map_auto.py --map our_second --radius 5 --yes   ← 비대화형

    --map NAME    생략하면 목록에서 고른다 (clean_map.py 와 같은 방식)
    --radius N    opening 반경 [px]. 클수록 과감히 지운다 (기본 4 = 20cm)
    --pad N       밴드 여유 [px]. 벽을 포함할 만큼 (기본 6 = 30cm)
    --hole-min N  이보다 큰 구멍은 인필드로 보고 남긴다 [px] (기본 400)
    --out NAME    출력 이름 (기본 <맵>_clean)
    --preview     결과 PNG 만 만들고 맵은 안 쓴다 (검수용)
    --yes         묻지 않고 바로 저장 (스크립트용)

★ 대화형이 기본인 이유
  이 정제의 유일한 노브는 --radius 인데, **맞는 값은 트랙마다 다르다.**
  좁으면 트랙이 잘려 나가고 넓으면 빛살이 남는데, 둘 다 그림을 봐야 안다.
  그래서 매번 결과 수치를 보여주고 "저장 / 반경 바꿔 다시 / 취소" 를 묻는다.
  clean_map.py 가 폴리곤을 눈으로 찍는 것과 같은 역할이다 — 다만 디스플레이가
  없어도 되도록 **수치와 PNG 경로**로 판단하게 한다.

동작 (clean_map.py 의 폴리곤 방식을 자동화한 것)
────────────────────────────────────────────────
  ① opening 으로 빛살을 지워 **씨앗** 을 얻는다
     — 이 마스크를 그대로 쓰면 가장자리가 opening 자국으로 울퉁불퉁하다
  ② 씨앗에서 외부/내부 경계를 만들어 **밴드** 를 정의한다
     (clean_map.py 의 외부·내부 폴리곤에 해당)
  ③ 밴드 안에서는 **원본 픽셀** 을 쓴다 → 벽이 선명하다
  ④ 복도 안 점 노이즈만 메우고, 인필드 같은 큰 구멍은 남긴다
     — 인필드가 없으면 gen_raceline 이 내곽 윤곽을 못 찾는다

★ 결과 PNG(<출력>_preview.png)를 **반드시 눈으로 확인할 것.** 트랙이 잘려
  나갔으면 --radius 를 줄이고, 빛살이 남았으면 키운다.
"""
import argparse
import os
import shutil
import sys

import numpy as np
import scipy.ndimage as ndi
from PIL import Image

VAL_FREE = 254
VAL_WALL = 0

_DIR = os.path.dirname(os.path.abspath(__file__))


def disk(r):
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    return (yy * yy + xx * xx) <= r * r


def select_map_interactive():
    """맵을 목록에서 고른다 (clean_map.py 의 같은 이름 함수와 동일한 조작감)."""
    files = os.listdir(_DIR)
    maps = sorted(set(f.rsplit('.', 1)[0] for f in files
                      if f.endswith(('.pgm', '.png')) and not f.startswith('.')))
    # 결과물(_clean)과 미리보기는 후보에서 뺀다
    maps = [m for m in maps
            if not m.endswith('_clean') and not m.endswith('_preview')]
    if not maps:
        sys.exit(f'✗ 맵 없음: {_DIR} 에 .pgm 을 넣어두세요 '
                 f'(./race.sh savemap <이름> 으로 만듭니다)')

    print('\n' + '=' * 48)
    print('  정제할 맵을 고르세요')
    print('=' * 48)
    for i, name in enumerate(maps, 1):
        mark = '*' if os.path.exists(os.path.join(_DIR, name + '_clean.pgm')) else ' '
        print(f'  {i:3d}. [{mark}] {name}')
    print('=' * 48)
    print('  * = 정제 맵(_clean) 이 이미 있음 (다시 만들면 덮어씁니다)\n')

    while True:
        val = input('맵 이름 또는 번호 (q=취소): ').strip()
        if val.lower() in ('q', 'quit', 'exit'):
            sys.exit(0)
        if not val:
            continue
        if val.isdigit():
            i = int(val) - 1
            if 0 <= i < len(maps):
                return maps[i]
            print(f'  범위 초과 (1~{len(maps)})')
            continue
        if val in maps:
            return val
        hit = [m for m in maps if val.lower() in m.lower()]
        if len(hit) == 1:
            print(f'  → {hit[0]}')
            return hit[0]
        print('  못 찾음' if not hit else f'  여러 개 걸림: {", ".join(hit)}')


def ask_next(radius, kept_pct, free_pct):
    """결과를 보고 무엇을 할지 묻는다. 반환: ('save',None) | ('retry',r) | ('quit',None)

    ★ 판단 기준을 같이 띄운다. 이 값들이 곧 "트랙이 잘렸나 / 빛살이 남았나" 다.
    """
    print()
    print('  ┌─ 판단 기준 ─────────────────────────────────────────────')
    if kept_pct < 55:
        print(f'  │ ⚠ 원본 자유공간의 {kept_pct:.0f}% 만 남았다 — 트랙이 잘렸을 수 있다.')
        print(f'  │   반경을 줄여 보세요 (지금 {radius} → {max(radius-1,1)})')
    elif kept_pct > 95:
        print(f'  │ ⚠ {kept_pct:.0f}% 가 그대로 남았다 — 빛살을 거의 못 지웠다.')
        print(f'  │   반경을 키워 보세요 (지금 {radius} → {radius+1})')
    else:
        print(f'  │ ✓ 유지율 {kept_pct:.0f}% — 흔한 범위다 (55~95%)')
    print(f'  │ 그래도 **_preview.png 를 눈으로 보는 것이 최종 판단**이다.')
    print('  └─────────────────────────────────────────────────────────')
    while True:
        v = input('  [Enter]=저장  숫자=그 반경으로 다시  q=취소 : ').strip()
        if v == '':
            return 'save', None
        if v.lower() in ('q', 'quit', 'exit', 'n'):
            return 'quit', None
        if v.isdigit() and 1 <= int(v) <= 30:
            return 'retry', int(v)
        print('  Enter / 1~30 / q 중에서 골라주세요')


def clean_once(a, free, args):
    """①~④ 를 한 번 돌려 (트랙 마스크, 유지율%) 를 돌려준다.

    반복 호출되므로 부작용(파일 쓰기)이 없어야 한다 — 저장은 호출부에서 한다.
    """
    # ① 가는 빛살 제거 → 트랙 복도의 **씨앗**
    #    여기서 나온 마스크를 그대로 쓰면 가장자리가 opening 자국으로 울퉁불퉁하다.
    #    씨앗은 "트랙이 어디냐" 를 정하는 데만 쓰고, 실제 픽셀은 ③에서 원본을 쓴다.
    op = ndi.binary_opening(free, structure=disk(args.radius))
    lbl, n = ndi.label(op)
    if n == 0:
        sys.exit(f'✗ opening(r={args.radius}) 후 남은 자유공간이 없습니다. '
                 f'반경을 줄이세요.')
    sizes = ndi.sum(op, lbl, range(1, n + 1))
    seed = (lbl == (1 + int(np.argmax(sizes))))
    print(f'  ① opening(r={args.radius}) → 성분 {n}개, 최대 성분 채택 {seed.sum()}px')

    # ② clean_map.py 의 외부/내부 폴리곤에 해당하는 밴드를 자동으로 만든다
    #    outer = 트랙 ∪ 인필드,  infield = 그중 안쪽 섬
    outer = ndi.binary_fill_holes(seed)
    infield = outer & ~seed
    band = (ndi.binary_dilation(outer, disk(args.pad))
            & ~ndi.binary_erosion(infield, disk(args.pad)))
    print(f'  ② 밴드 생성 (pad={args.pad}px) — 인필드 {infield.sum()}px')

    # ③ 밴드 안에서는 **원본 픽셀** 을 쓴다 → 벽이 선명하게 남는다
    keep = band & free
    l2, m = ndi.label(keep)
    if m == 0:
        sys.exit('✗ 밴드 안에 자유공간이 없습니다.')
    s2 = ndi.sum(keep, l2, range(1, m + 1))
    keep = (l2 == (1 + int(np.argmax(s2))))
    kept_pct = 100.0 * keep.sum() / max(free.sum(), 1)
    print(f'  ③ 원본 픽셀 채택 {keep.sum()}px ({kept_pct:.1f}% 유지)')

    # ④ 복도 안 점 노이즈만 메운다. 인필드 같은 **큰 구멍은 남긴다** —
    #    그게 없으면 gen_raceline 이 내곽 윤곽을 못 찾아 트랙 추출에 실패한다.
    filled = ndi.binary_fill_holes(keep)
    holes = filled & ~keep
    hl, hn = ndi.label(holes)
    track = filled
    n_big = 0
    if hn:
        hs = ndi.sum(holes, hl, range(1, hn + 1))
        for i, sz in enumerate(hs, start=1):
            if sz >= args.hole_min:
                track = track & ~(hl == i)
                n_big += 1
    print(f'  ④ 구멍 {hn}개 중 {n_big}개 유지(인필드), 나머지 메움')
    print(f'  결과  자유 {100 * track.sum() / track.size:.1f}%  (미지 0%)')
    return track, kept_pct


def write_preview(a, free, track, out_name):
    """검수용 PNG. 빨강 = 지워진 영역. 매 시도마다 덮어쓴다."""
    prev = np.full(a.shape + (3,), 128, np.uint8)
    prev[free & ~track] = (255, 80, 80)
    prev[track] = (255, 255, 255)
    prev[a == VAL_WALL] = (0, 0, 0)
    path = os.path.join(_DIR, out_name + '_preview.png')
    Image.fromarray(prev).save(path)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=None,
                    help='맵 이름 (생략하면 목록에서 고른다)')
    ap.add_argument('--yes', action='store_true',
                    help='묻지 않고 바로 저장 (스크립트용)')
    ap.add_argument('--out', default=None, help='출력 이름 (기본 <맵>_clean)')
    ap.add_argument('--radius', type=int, default=4, help='opening 반경 px (기본 4)')
    ap.add_argument('--pad', type=int, default=6,
                    help='밴드 여유 px — 벽을 포함할 만큼 (기본 6 = 30cm)')
    ap.add_argument('--hole-min', type=int, default=400, dest='hole_min',
                    help='이보다 큰 구멍은 인필드로 보고 남긴다 [px] (기본 400)')
    ap.add_argument('--preview', action='store_true', help='PNG 만 만들고 맵은 안 쓴다')
    args = ap.parse_args()

    interactive = (not args.yes) and sys.stdin.isatty()
    if not args.map:
        if not interactive:
            sys.exit('✗ --map 이 없고 대화형도 아닙니다')
        args.map = select_map_interactive()

    src_pgm = os.path.join(_DIR, args.map + '.pgm')
    src_yaml = os.path.join(_DIR, args.map + '.yaml')
    if not os.path.exists(src_pgm):
        sys.exit(f'✗ 맵 없음: {src_pgm}')
    out_name = args.out or (args.map + '_clean')

    a = np.array(Image.open(src_pgm))
    free = (a == VAL_FREE)
    print(f'원본 {a.shape[1]}x{a.shape[0]}px  자유 {free.sum()}  벽 {(a == VAL_WALL).sum()}')

    # ── 반경을 바꿔가며 되풀이한다 ────────────────────────────────────
    #   맞는 --radius 는 트랙마다 다르고, 잘렸는지 남았는지는 결과를 봐야 안다.
    #   그래서 계산 → 수치·PNG 제시 → 저장/재시도 를 반복한다.
    #   PNG 는 매 시도마다 덮어쓰므로 항상 마지막 시도를 보게 된다.
    while True:
        track, kept_pct = clean_once(a, free, args)
        prev_path = write_preview(a, free, track, out_name)
        print(f'검수용 PNG: {prev_path}   (빨강 = 지워진 영역)')
        if not interactive:
            break
        act, r = ask_next(args.radius, kept_pct, 100 * track.sum() / a.size)
        if act == 'save':
            break
        if act == 'quit':
            print('취소했습니다 (맵은 쓰지 않았습니다).')
            return
        args.radius = r
        print(f'\n─── 반경 {r} 로 다시 ───')

    if args.preview:
        print('--preview 라 맵은 쓰지 않았습니다.')
        return

    # ⑤ 출력: 트랙만 자유, 나머지 전부 벽 (미지 없음 — 기존 _clean 규약)
    out = np.full(a.shape, VAL_WALL, np.uint8)
    out[track] = VAL_FREE
    dst_pgm = os.path.join(_DIR, out_name + '.pgm')
    Image.fromarray(out).save(dst_pgm)
    if os.path.exists(src_yaml):
        dst_yaml = os.path.join(_DIR, out_name + '.yaml')
        shutil.copy(src_yaml, dst_yaml)
        with open(dst_yaml) as f:
            txt = f.read()
        with open(dst_yaml, 'w') as f:
            f.write(txt.replace(args.map + '.pgm', out_name + '.pgm'))
        print(f'저장: {dst_pgm}\n      {dst_yaml}')
    else:
        print(f'저장: {dst_pgm}   (yaml 원본이 없어 복사 못 함)')

    print(f'\n다음 단계 (반경 {args.radius} 로 저장했습니다):')
    print(f'  1) {out_name}_preview.png 를 눈으로 확인 '
          f'(트랙이 잘렸으면 ./race.sh clean {args.map} 로 반경을 줄여 다시)')
    print(f'  2) 웨이포인트:   ./race.sh waypoints-kappa {out_name}')
    print(f'  3) race_script.yaml 의 map_name / waypoints 를 바꾸고')
    print(f'     ★ localization.init_* 와 mission.start_* 도 **반드시 같이** 바꿀 것.')
    print(f'        안 바꾸면 옛 좌표가 벽 속이라 AMCL 이 위치를 못 잡는다')
    print(f'        (2026-08-14 실제로 겪음). ./race.sh savepose 가 한 번에 저장한다.')


if __name__ == '__main__':
    main()
