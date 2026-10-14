#!/usr/bin/env python3
"""
clean_map.py v3 — 외부/내부 폴리곤 기반 맵 정제
=================================================
원본 맵 위에서 두 개의 폴리곤을 그려 트랙 영역을 정의.
  - 외부 폴리곤 (빨강): 트랙 바깥 경계 (이 밖은 전부 벽)
  - 내부 폴리곤 (파랑): 내부 벽 경계  (이 안도 전부 벽)
  - 두 폴리곤 사이: SLAM 원본 픽셀 유지 + 틈 봉합

조작:
  1           — 외부 폴리곤 그리기 모드
  2           — 내부 폴리곤 그리기 모드
  LClick      — 꼭짓점 추가
  RClick / z  — 마지막 꼭짓점 취소
  Enter       — 마스크 적용 (두 폴리곤 모두 확정)
  c           — closing 반복 횟수 토글 (0→2→4)
  s           — 저장
  r           — 전체 리셋
  q / Esc     — 종료
"""

import os
import sys
import yaml
import argparse
import numpy as np
from PIL import Image
import scipy.ndimage as ndimage
from matplotlib.path import Path

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

VAL_FREE = 254
VAL_WALL = 0
VAL_UNK  = 205

_DIR = os.path.dirname(os.path.abspath(__file__))


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--map', default=None, help='맵 이름 또는 경로 (생략 시 대화형 선택)')
    p.add_argument('--out', default=None, help='출력 맵 이름 (생략 시 <맵이름>_clean)')
    return p.parse_args()


def select_map_interactive():
    files = os.listdir(_DIR)
    maps  = sorted(set(
        f.rsplit('.', 1)[0]
        for f in files
        if f.endswith(('.pgm', '.png')) and not f.startswith('.')
    ))
    # _clean 으로 끝나는 결과물은 목록에서 제외
    maps = [m for m in maps if not m.endswith('_clean')]

    if not maps:
        sys.exit(f'맵 없음: {_DIR} 에 .pgm/.png 파일을 넣어두세요.')

    clean_set = {m + '_clean' for m in maps
                 if os.path.exists(os.path.join(_DIR, m + '_clean.pgm'))
                 or os.path.exists(os.path.join(_DIR, m + '_clean.png'))}

    print('\n' + '='*48)
    print('  사용 가능한 맵 목록')
    print('='*48)
    for i, name in enumerate(maps, 1):
        mark = '*' if (name + '_clean') in clean_set else ' '
        print(f'  {i:3d}. [{mark}] {name}')
    print('='*48)
    print('  * = 정제 맵(_clean) 이미 존재\n')

    while True:
        val = input('맵 이름 또는 번호: ').strip()
        if not val:
            continue
        if val.isdigit():
            idx = int(val) - 1
            if 0 <= idx < len(maps):
                return maps[idx]
            print(f'  범위 초과 (1~{len(maps)})')
            continue
        if val in maps:
            return val
        matches = [m for m in maps if val.lower() in m.lower()]
        if len(matches) == 1:
            print(f'  -> {matches[0]}')
            return matches[0]
        if matches:
            print(f'  후보: {matches}')
        else:
            print(f'  맵 없음: {val}')


def load_map(map_base):
    for ext in ('.pgm', '.png'):
        path = map_base + ext
        if os.path.exists(path):
            arr = np.array(Image.open(path).convert('L'))
            print(f'맵 로드: {path}  {arr.shape[1]}x{arr.shape[0]}px')
            return arr
    sys.exit(f'오류: {map_base}.pgm/.png 없음')


def load_yaml(map_base):
    path = map_base + '.yaml'
    if not os.path.exists(path):
        return {'resolution': 0.05, 'origin': [-6.33, -7.88, 0],
                'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}
    with open(path) as f:
        return yaml.safe_load(f)


def poly_mask(shape, pts):
    """pts = [(col, row), ...] 픽셀 좌표 → 내부 bool 마스크"""
    if len(pts) < 3:
        return np.zeros(shape, dtype=bool)
    h, w = shape
    path = Path(pts)
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    points = np.column_stack([cols.ravel(), rows.ravel()])
    return path.contains_points(points).reshape(h, w)


def apply_mask(orig, outer_pts, inner_pts, close_iter):
    """
    outer 안 & inner 밖 = 트랙 코리도
      - 코리도 안 원본 픽셀 유지 + closing
    나머지 (outer 밖 or inner 안) = 벽(0)
    """
    h, w = orig.shape
    outer_mask = poly_mask(orig.shape, outer_pts) if len(outer_pts) >= 3 \
                 else np.ones(orig.shape, dtype=bool)
    inner_mask = poly_mask(orig.shape, inner_pts) if len(inner_pts) >= 3 \
                 else np.zeros(orig.shape, dtype=bool)

    track = outer_mask & ~inner_mask   # 두 폴리곤 사이 = 트랙 코리도

    result = np.zeros((h, w), dtype=np.uint8)        # 전체 벽으로 시작
    result[track & (orig == VAL_FREE)] = VAL_FREE    # 코리도 자유공간 복원

    if close_iter > 0:
        free = (result == VAL_FREE)
        struct = ndimage.generate_binary_structure(2, 2)
        closed = ndimage.binary_closing(free, structure=struct, iterations=close_iter)
        result[closed & track] = VAL_FREE
        result[~(closed & track)] = VAL_WALL

    n_free = (result == VAL_FREE).sum()
    print(f'  적용 완료: 자유={n_free}px  closing={close_iter}회')
    return result


class MapCleaner:
    CLOSE_CYCLE = [0, 2, 4]

    def __init__(self, orig, out_base, cfg):
        self.orig     = orig
        self.out_base = out_base
        self.cfg      = cfg

        # 두 폴리곤
        self.polys    = {1: [], 2: []}   # 1=outer(빨강), 2=inner(파랑)
        self.mode     = 1                # 현재 그리기 모드
        self.result   = None
        self.ci_idx   = 1               # closing index

        self._build_ui()

    def _build_ui(self):
        self.fig, self.ax = plt.subplots(figsize=(13, 9))
        plt.subplots_adjust(bottom=0.08)

        self.im = self.ax.imshow(self.orig, cmap='gray',
                                  vmin=0, vmax=255, interpolation='nearest')
        self.ax.axis('off')

        # 폴리곤 선/점
        self.lines = {
            1: self.ax.plot([], [], 'r-',  lw=2,   alpha=0.9)[0],
            2: self.ax.plot([], [], 'b-',  lw=2,   alpha=0.9)[0],
        }
        self.dots = {
            1: self.ax.plot([], [], 'ro',  ms=5)[0],
            2: self.ax.plot([], [], 'bo',  ms=5)[0],
        }
        self.close_lines = {
            1: self.ax.plot([], [], 'r--', lw=1.2, alpha=0.5)[0],
            2: self.ax.plot([], [], 'b--', lw=1.2, alpha=0.5)[0],
        }

        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('key_press_event',    self._on_key)

        self._update_title()
        plt.show()

    # ── 그리기 ────────────────────────────────────────────────────────────
    def _redraw(self):
        for m in (1, 2):
            pts = self.polys[m]
            if not pts:
                self.lines[m].set_data([], [])
                self.dots[m].set_data([], [])
                self.close_lines[m].set_data([], [])
                continue
            cs = [p[0] for p in pts]
            rs = [p[1] for p in pts]
            self.lines[m].set_data(cs, rs)
            self.dots[m].set_data(cs, rs)
            if len(pts) >= 2:
                self.close_lines[m].set_data([cs[-1], cs[0]], [rs[-1], rs[0]])
            else:
                self.close_lines[m].set_data([], [])
        self.fig.canvas.draw_idle()

    def _update_title(self):
        mode_str = 'OUTER (red)' if self.mode == 1 else 'INNER (blue)'
        ci = self.CLOSE_CYCLE[self.ci_idx]
        n1 = len(self.polys[1])
        n2 = len(self.polys[2])
        self.ax.set_title(
            f'{mode_str}  |  outer={n1}pt  inner={n2}pt  closing={ci}  |  '
            f'1=outer  2=inner  L-click=add  R-click/z=undo  '
            f'Enter=apply  s=save  r=reset  q=quit',
            fontsize=9)
        self.fig.canvas.draw_idle()

    # ── 이벤트 ────────────────────────────────────────────────────────────
    def _on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        col = int(round(event.xdata))
        row = int(round(event.ydata))

        if event.button == 1:
            self.polys[self.mode].append((col, row))
        elif event.button == 3 and self.polys[self.mode]:
            self.polys[self.mode].pop()

        self._redraw()
        self._update_title()

    def _on_key(self, event):
        k = event.key

        if k == '1':
            self.mode = 1
            print('mode: OUTER polygon (red)')
            self._update_title()

        elif k == '2':
            self.mode = 2
            print('mode: INNER polygon (blue)')
            self._update_title()

        elif k == 'z' and self.polys[self.mode]:
            self.polys[self.mode].pop()
            self._redraw()
            self._update_title()

        elif k == 'enter':
            self._apply()

        elif k == 'c':
            self.ci_idx = (self.ci_idx + 1) % len(self.CLOSE_CYCLE)
            ci = self.CLOSE_CYCLE[self.ci_idx]
            print(f'closing={ci}')
            self._update_title()
            if self.result is not None:
                self._apply(silent=True)

        elif k == 'r':
            self.polys  = {1: [], 2: []}
            self.result = None
            self.im.set_data(self.orig)
            self._redraw()
            self._update_title()
            print('reset')

        elif k == 's':
            self._save()

        elif k in ('q', 'escape'):
            plt.close('all')

    def _apply(self, silent=False):
        ci = self.CLOSE_CYCLE[self.ci_idx]
        if not silent:
            print(f'apply mask (outer={len(self.polys[1])}pt '
                  f'inner={len(self.polys[2])}pt closing={ci})')
        self.result = apply_mask(self.orig,
                                  self.polys[1], self.polys[2], ci)
        disp = np.where(self.result == VAL_FREE, 255, 0).astype(np.uint8)
        self.im.set_data(disp)
        self.fig.canvas.draw_idle()

    def _save(self):
        if self.result is None:
            print('Press Enter first to apply the mask')
            return
        out_pgm = self.out_base + '.pgm'
        Image.fromarray(self.result, mode='L').save(out_pgm)
        print(f'saved PGM: {out_pgm}')

        cfg_out = dict(self.cfg)
        cfg_out['image'] = os.path.basename(self.out_base) + '.pgm'
        with open(self.out_base + '.yaml', 'w') as f:
            yaml.dump(cfg_out, f, default_flow_style=False)
        print(f'saved YAML: {self.out_base}.yaml')
        print('done')


def main():
    args = parse_args()
    print('='*55)
    print(' clean_map v3 — 외부/내부 폴리곤 마스크')
    print('='*55)

    # 맵 선택
    if args.map:
        # 절대/상대 경로 또는 이름만 입력한 경우 모두 처리
        map_base = args.map if os.path.dirname(args.map) \
                   else os.path.join(_DIR, args.map)
    else:
        map_name = select_map_interactive()
        map_base = os.path.join(_DIR, map_name)

    # 출력 경로 결정
    if args.out:
        out_base = args.out if os.path.dirname(args.out) \
                   else os.path.join(_DIR, args.out)
    else:
        out_base = map_base + '_clean'

    print(f'\n  input:  {map_base}')
    print(f'  output: {out_base}')
    print('\n  1 = OUTER boundary polygon (red)')
    print('  2 = INNER boundary polygon (blue)')
    print('  L-click: add vertex   R-click/z: undo')
    print('  Enter: apply  c: closing toggle  s: save  r: reset  q: quit\n')

    arr = load_map(map_base)
    cfg = load_yaml(map_base)
    MapCleaner(arr, out_base, cfg)


if __name__ == '__main__':
    main()
