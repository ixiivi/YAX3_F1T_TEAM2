#!/usr/bin/env python3
"""
맵 yaml 생성기
==============
.png는 있지만 .yaml이 없는 맵을 자동으로 전체 생성합니다.

사용법:
  python3 make_map_yaml.py
"""

import os
from PIL import Image

_THIS_DIR       = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT      = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
MAPS_DIR        = os.path.join(_REPO_ROOT, 'sim_ws', 'src', 'f1tenth_gym_ros', 'maps')
DEFAULT_RESOLUTION = 0.05
DEFAULT_ORIGIN     = [0.0, 0.0, 0.0]
DEFAULT_NEGATE     = 0
DEFAULT_OCCUPIED   = 0.65
DEFAULT_FREE       = 0.196


def make_yaml(map_name):
    png_path  = os.path.join(MAPS_DIR, f'{map_name}.png')
    yaml_path = os.path.join(MAPS_DIR, f'{map_name}.yaml')

    if os.path.exists(yaml_path):
        print(f'  ⚠ 스킵 (이미 존재): {map_name}.yaml')
        return

    content = (
        f"image: {map_name}.png\n"
        f"resolution: {DEFAULT_RESOLUTION:.6f}\n"
        f"origin: [{DEFAULT_ORIGIN[0]:.6f}, {DEFAULT_ORIGIN[1]:.6f}, {DEFAULT_ORIGIN[2]:.6f}]\n"
        f"negate: {DEFAULT_NEGATE}\n"
        f"occupied_thresh: {DEFAULT_OCCUPIED}\n"
        f"free_thresh: {DEFAULT_FREE}\n"
    )

    with open(yaml_path, 'w') as f:
        f.write(content)
    print(f'  ✓ 생성: {map_name}.yaml')


# ── 메인 ─────────────────────────────────────────────────────────────
files  = os.listdir(MAPS_DIR)
pngs   = {f[:-4] for f in files if f.endswith('.png')}
yamls  = {f[:-5] for f in files if f.endswith('.yaml')}
missing = sorted(pngs - yamls)

if not missing:
    print('모든 맵에 yaml이 있습니다.')
else:
    print(f'yaml 없는 맵 {len(missing)}개 생성 중...')
    for name in missing:
        make_yaml(name)
    print('완료!')
