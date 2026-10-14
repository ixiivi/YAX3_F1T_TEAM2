#!/usr/bin/env python3
"""Build a safe closed centerline and speed profile from a ROS occupancy map."""
import argparse
import csv
import os
import sys
import yaml
import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.interpolate import splprep, splev
from skimage.morphology import skeletonize


def _longest_loop(skeleton):
    """Prune branches, then order the largest remaining 8-connected loop."""
    s = skeleton.copy()
    kernel = np.ones((3, 3), dtype=np.uint8); kernel[1, 1] = 0
    for _ in range(10000):
        degree = ndimage.convolve(s.astype(np.uint8), kernel, mode='constant')
        ends = s & (degree <= 1)
        if not ends.any():
            break
        s[ends] = False
    labels, count = ndimage.label(s, structure=np.ones((3, 3)))
    if count == 0:
        raise RuntimeError('closed track was not found; drive a complete loop and save again')
    sizes = np.bincount(labels.ravel()); sizes[0] = 0
    component = labels == sizes.argmax()
    pixels = set(map(tuple, np.argwhere(component)))
    start = min(pixels)
    ordered = [start]; previous = None; current = start
    while True:
        candidates = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                p = (current[0] + dr, current[1] + dc)
                if p in pixels and p != previous and p != current:
                    candidates.append(p)
        if not candidates:
            break
        if len(ordered) > 2 and start in candidates:
            break
        unvisited = [p for p in candidates if p not in ordered]
        if not unvisited:
            break
        if previous is None:
            nxt = unvisited[0]
        else:
            direction = np.subtract(current, previous)
            nxt = max(unvisited, key=lambda p: np.dot(np.subtract(p, current), direction))
        previous, current = current, nxt
        ordered.append(current)
        if len(ordered) > len(pixels) + 1:
            break
    if len(ordered) < max(40, int(0.65 * len(pixels))):
        raise RuntimeError('track skeleton is branched/open; improve SLAM loop closure')
    return np.asarray(ordered)


def _curvature(x, y):
    dx = np.gradient(x); dy = np.gradient(y)
    ddx = np.gradient(dx); ddy = np.gradient(dy)
    return np.abs(dx * ddy - dy * ddx) / np.maximum((dx*dx + dy*dy) ** 1.5, 1e-6)


def generate(map_yaml, output, safety, vmax, vmin, mu):
    with open(map_yaml, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    image_path = cfg['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml), image_path)
    image = np.asarray(Image.open(image_path).convert('L'))
    res = float(cfg['resolution']); ox, oy = map(float, cfg['origin'][:2])
    # ROS maps: white=free, black=wall, gray=unknown. Unknown is never drivable.
    free = image >= 250
    labels, n = ndimage.label(free)
    if n == 0:
        raise RuntimeError('map contains no known free space')
    sizes = np.bincount(labels.ravel()); sizes[0] = 0
    track = labels == sizes.argmax()
    clearance = ndimage.distance_transform_edt(track) * res
    safe = clearance >= safety
    if safe.sum() < 100:
        raise RuntimeError('safety margin removes the whole track; reduce --safety or remap')
    loop = _longest_loop(skeletonize(safe))
    py, px = loop[:, 0], loop[:, 1]
    x = px * res + ox
    y = (image.shape[0] - 1 - py) * res + oy
    # Periodic spline removes pixel stair-steps; uniform samples stabilize control.
    keep = np.r_[True, np.hypot(np.diff(x), np.diff(y)) > 0.02]
    x, y = x[keep], y[keep]
    length = np.hypot(np.diff(np.r_[x, x[0]]), np.diff(np.r_[y, y[0]])).sum()
    count = max(120, int(length / 0.08))
    tck, _ = splprep([x, y], per=True, s=max(0.05, len(x) * 0.01), k=3)
    x, y = map(np.asarray, splev(np.linspace(0, 1, count, endpoint=False), tck))
    # Reject smoothing that crosses the configured wall margin.
    qx = np.clip(np.rint((x - ox) / res).astype(int), 0, image.shape[1]-1)
    qy = np.clip(np.rint(image.shape[0] - 1 - (y - oy) / res).astype(int), 0, image.shape[0]-1)
    if np.min(clearance[qy, qx]) < safety * 0.75:
        raise RuntimeError('smoothed line violates wall margin; increase map quality')
    curvature = _curvature(x, y)
    speed = np.clip(np.sqrt(mu * 9.81 / np.maximum(curvature, 0.02)), vmin, vmax)
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f); writer.writerow(['x', 'y', 'speed'])
        writer.writerows(zip(x, y, speed))
    preview = np.dstack([image, image, image]).copy()
    preview[qy, qx] = (0, 0, 255)
    Image.fromarray(preview).save(os.path.splitext(output)[0] + '_preview.png')
    print(f'[auto_track] {count} points, {length:.1f} m -> {output}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--map', required=True, dest='map_yaml')
    p.add_argument('--output', required=True)
    p.add_argument('--safety', type=float, default=0.28)
    p.add_argument('--vmax', type=float, default=3.0)
    p.add_argument('--vmin', type=float, default=0.45)
    p.add_argument('--mu', type=float, default=0.75)
    a = p.parse_args()
    try:
        generate(a.map_yaml, a.output, a.safety, a.vmax, a.vmin, a.mu)
    except Exception as e:
        print(f'[auto_track] ERROR: {e}', file=sys.stderr); raise SystemExit(2)

if __name__ == '__main__':
    main()
