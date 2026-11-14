"""The GPU pose search (match_device='cuda') gives the CPU's answers, on the venue maps.

Skipped where torch or CUDA is missing. A mismatch here means the GPU path no longer does what
the CPU path does: do not race with match_device=cuda then.
"""
import math
import os

import numpy as np
import pytest
from conftest import map_path, waypoint_path
from test_scan_match import raycast

from waypoint_nav.pure_pursuit import load_waypoints
from waypoint_nav.scan_match import MapMatcher

LASER_X = 0.267
torch = pytest.importorskip('torch')
pytestmark = pytest.mark.skipif(not torch.cuda.is_available(), reason='no CUDA')
MAPS = [m for m in ('left1043', 'right1047') if os.path.exists(map_path(m))]


@pytest.fixture(scope='module', params=MAPS)
def pair(request):
    name = request.param
    cpu = MapMatcher(map_path(name), LASER_X)
    gpu = MapMatcher(map_path(name), LASER_X, device='cuda')
    assert gpu.device_note.startswith('gpu'), gpu.device_note
    xy, _ = load_waypoints(waypoint_path(name))
    return name, cpu, gpu, xy


def poses(xy, n):
    for k in np.linspace(0, len(xy) - 4, n).astype(int):
        yield xy[k, 0], xy[k, 1], math.atan2(*(xy[k + 3] - xy[k])[::-1])


def rays(m, x, y, yaw, step):
    r, a = raycast(m, x, y, yaw)
    ok = np.isfinite(r)
    return r[ok][::step], a[ok][::step]


def test_global_search_matches_cpu(pair):
    _, cpu, gpu, xy = pair
    for x, y, yaw in poses(xy, 2):
        r, a = rays(cpu, x, y, yaw, 6)
        c, g = cpu.global_search(r, a), gpu.global_search(r, a)
        assert g[0] == pytest.approx(c[0], abs=1e-4)
        assert math.hypot(c[1] - g[1], c[2] - g[2]) < 0.02 and abs(c[3] - g[3]) < 1e-3, (c, g)


def test_wide_local_search_matches_cpu(pair):
    # the reach-limited relocalization searches a few metres around the last trusted pose: big
    # enough for the GPU path
    _, cpu, gpu, xy = pair
    for x, y, yaw in poses(xy, 3):
        r, a = rays(cpu, x, y, yaw, 3)
        c = cpu.local_search(x + 0.4, y - 0.3, yaw + 0.15, r, a, radius=2.5)
        g = gpu.local_search(x + 0.4, y - 0.3, yaw + 0.15, r, a, radius=2.5)
        assert g[0] == pytest.approx(c[0], abs=1e-4)
        assert math.hypot(c[1] - g[1], c[2] - g[2]) < 0.02 and abs(c[3] - g[3]) < 1e-3, (c, g)
        assert math.hypot(g[1] - x, g[2] - y) < 0.15          # and it is the right place


def test_no_gpu_falls_back_to_cpu(tmp_path):
    m = MapMatcher(map_path(MAPS[0]), LASER_X, device='nonsense:9')
    assert m.torch is None and m.device_note.startswith('cpu')
