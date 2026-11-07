"""map_editor painting, speck cleanup and saving, on a copy of a real map."""
import shutil

import numpy as np
import pytest
import yaml
from conftest import map_path
from PIL import Image

from waypoint_nav import map_editor as me
from waypoint_nav.scan_match import MapMatcher


@pytest.fixture
def lobby_copy(tmp_path):
    src = map_path('lobby')
    with open(src) as f:
        meta = yaml.safe_load(f)
    shutil.copy(src, tmp_path / 'lobby.yaml')
    shutil.copy(me.image_path(src), tmp_path / meta['image'])
    return str(tmp_path / 'lobby.yaml')


def test_disc_and_rect_stay_inside_the_image():
    img = np.full((20, 30), me.FREE, np.uint8)
    me.paint_disc(img, 0, 0, 3, me.WALL)
    me.paint_disc(img, 40, 40, 3, me.WALL)        # fully outside: no error, no change
    assert img[0, 0] == me.WALL and img[3, 0] == me.WALL and img[3, 3] == me.FREE
    me.paint_rect(img, 25, 15, 100, 100, me.UNKNOWN)
    assert (img[15:, 25:] == me.UNKNOWN).all() and img[14, 24] == me.FREE


def test_specks_are_isolated_small_blobs_only():
    img = np.full((40, 40), me.FREE, np.uint8)
    img[5, 5] = me.WALL                    # 1 px speck
    img[10:12, 20:22] = me.WALL            # 4 px speck
    img[30, :] = me.WALL                   # a real wall
    img[29, 10] = me.WALL                  # touches the wall diagonally-adjacent: not a speck
    mask = me.wall_specks(img, max_px=4)
    assert mask[5, 5] and mask[10:12, 20:22].all()
    assert not mask[30].any() and not mask[29, 10]
    assert not me.wall_specks(img, max_px=3)[10:12, 20:22].any()


def test_save_keeps_the_original_once(lobby_copy):
    img, _, _ = me.load_map(lobby_copy)
    original = img.copy()
    img[me.wall_specks(img, 4)] = me.FREE
    path, backup = me.save_map(img, lobby_copy)
    img2 = img.copy()
    img2[0, 0] = me.WALL
    me.save_map(img2, lobby_copy)          # second save must not touch the backup
    assert (np.array(Image.open(backup)) == original).all()
    saved = np.array(Image.open(path))
    assert (saved == img2).all() and set(np.unique(saved)) <= {me.WALL, me.UNKNOWN, me.FREE}
    # the edited map still loads for AMCL-side code
    assert MapMatcher(lobby_copy).dist.shape == saved.shape


def test_speck_cleanup_on_the_lobby_map(lobby_copy):
    img, _, _ = me.load_map(lobby_copy)
    before = int((img == me.WALL).sum())
    mask = me.wall_specks(img, 4)
    print(f'\nlobby: {int(mask.sum())} of {before} wall px are specks <= 4 px')
    assert 0 < mask.sum() < 0.2 * before
