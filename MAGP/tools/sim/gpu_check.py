"""CPU vs GPU MapMatcher on a real map: same answers? how fast? (run on the Jetson)"""
import csv, math, sys, time
import numpy as np
sys.path.insert(0, '/home/yunsang/MAGP/MAGP/src/waypoint_nav')
from waypoint_nav.scan_match import MapMatcher
MAP, WP = sys.argv[1], sys.argv[2]
N = int(sys.argv[3]) if len(sys.argv) > 3 else 6
LX = 0.267
cpu = MapMatcher(MAP, LX); t0 = time.time(); gpu = MapMatcher(MAP, LX, device='cuda')
print('gpu init %.1f s:' % (time.time() - t0), gpu.device_note)
xy = np.array([[float(r['x']), float(r['y'])] for r in csv.DictReader(open(WP))])
rng = np.random.default_rng(0)

def scan_from(m, x, y, yaw):
    ang = np.radians(np.arange(-135, 135, 0.25)); rr = np.arange(0.06, 8.0, 0.01)
    lx, ly = x + LX * math.cos(yaw), y + LX * math.sin(yaw)
    wx = lx + rr[None] * np.cos(yaw + ang)[:, None]; wy = ly + rr[None] * np.sin(yaw + ang)[:, None]
    u, v, ins = m._cells(wx, wy); hit = np.zeros(wx.shape, bool); hit[ins] = m.img[v[ins], u[ins]] < 50
    ok = hit.any(1); r = rr[hit.argmax(1)][ok] + rng.normal(0, 0.015, ok.sum())
    return r, ang[ok]

def heading_of(xs, ys):
    from scipy.spatial import cKDTree
    t = np.roll(xy, -3, 0) - xy; h = np.arctan2(t[:, 1], t[:, 0])
    return h[cKDTree(xy).query(np.column_stack([xs, ys]))[1]]

rows = []
for k in np.linspace(0, len(xy) - 1, N).astype(int):
    j = (k + 3) % len(xy); yaw = math.atan2(*(xy[j] - xy[k])[::-1])
    x, y = xy[k] + rng.normal(0, 0.05, 2)
    r, a = scan_from(cpu, x, y, yaw); r6, a6 = r[::6], a[::6]
    out = {}
    for name, m in (('cpu', cpu), ('gpu', gpu)):
        for kind in ('global_h', 'local'):
            t = time.time()
            if kind == 'global_h':
                res = m.global_search(r6, a6, heading_of=heading_of)
            else:
                res = m.local_search(x + 0.3, y - 0.2, yaw + 0.1, r[::3], a[::3], radius=1.0)
            out[name, kind] = (time.time() - t, res)
    for kind in ('global_h', 'local'):
        (tc, rc), (tg, rg) = out['cpu', kind], out['gpu', kind]
        dpos = math.hypot(rc[1] - rg[1], rc[2] - rg[2]); dyaw = abs(math.degrees(math.remainder(rc[3] - rg[3], 2 * math.pi)))
        err_c = math.hypot(rc[1] - x, rc[2] - y); err_g = math.hypot(rg[1] - x, rg[2] - y)
        rows.append((kind, tc, tg, dpos, dyaw, abs(rc[0] - rg[0]), err_c, err_g))
        print(f'wp {k:4d} {kind:8s} cpu {tc:6.2f}s gpu {tg:5.2f}s | cpu-gpu {dpos*100:5.1f} cm {dyaw:4.1f} deg score diff {abs(rc[0]-rg[0]):.4f} | err to truth cpu {err_c*100:5.1f} cm gpu {err_g*100:5.1f} cm')
for kind in ('global_h', 'local'):
    s = [r for r in rows if r[0] == kind]
    print(f'{kind}: median cpu {np.median([r[1] for r in s]):.2f}s gpu {np.median([r[2] for r in s]):.2f}s, '
          f'max cpu-gpu {max(r[3] for r in s)*100:.1f} cm, wrong place (>0.5 m from truth) cpu {sum(r[6]>0.5 for r in s)} gpu {sum(r[7]>0.5 for r in s)} of {len(s)}')
