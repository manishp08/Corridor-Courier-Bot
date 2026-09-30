import numpy as np
import pytest

from ccsim.costlib import CostModel
from ccsim.costmap import INSCRIBED, LETHAL
from ccsim.drift import run_drift
from ccsim.runner import run_episode
from ccsim.sensors import CLASS_IDS, Lidar
from ccsim.world import Obstacle, World, _rect_walls, make_world, reachable


def test_worlds_are_deterministic_and_feasible():
    for name in ["empty_corridor", "cluttered_room", "glass_wall", "walking_actors", "kidnapped"]:
        a, b = make_world(name, 1234), make_world(name, 1234)
        assert np.allclose(a.start, b.start) and np.allclose(a.goal, b.goal)
        assert [o.x for o in a.obstacles] == [o.x for o in b.obstacles]
        assert reachable(a)


def _box_world(obstacles=(), glass=None):
    w = World("t", (-1, -1, 11, 11), _rect_walls(0, 0, 10, 10))
    w.obstacles = list(obstacles)
    if glass is not None:
        w.glass = np.array([glass], float)
    return w


def test_lidar_cannot_see_pallet_but_sees_shelf_and_cart_legs():
    rng = np.random.default_rng(0)
    lidar = Lidar()
    front = np.argmin(np.abs(lidar.angles))
    pallet = Obstacle("pallet", "low_obstacle", 3.0, 5.0, 0.0, 1.2, 0.8, 0.14)
    _, r = lidar.scan(_box_world([pallet]), np.array([1.0, 5.0, 0.0]), np.zeros((0, 4)), rng)
    assert r[front] > 8.0          # passes over the pallet; far wall at 9 m is beyond max range
    shelf = Obstacle("shelf", "unknown", 3.0, 5.0, 0.0, 0.6, 0.6, 1.2)
    _, r = lidar.scan(_box_world([shelf]), np.array([1.0, 5.0, 0.0]), np.zeros((0, 4)), rng)
    assert abs(r[front] - 1.7) < 0.05
    cart = Obstacle("cart", "cart", 3.0, 5.0, 0.0, 0.9, 0.55, 0.95)
    assert len(cart.lidar_circles()) == 4 and len(cart.lidar_segments()) == 0


def test_glass_returns_only_near_normal_incidence():
    lidar = Lidar()
    front = np.argmin(np.abs(lidar.angles))
    head_on = _box_world(glass=[4.0, 2.0, 4.0, 8.0])
    oblique = _box_world(glass=[3.0, 2.0, 5.0, 8.0])     # ~18 deg off normal
    hits = [np.isclose(lidar.scan(head_on, np.array([1.0, 5.0, 0.0]), np.zeros((0, 4)), np.random.default_rng(s))[1][front], 3.0, atol=0.05) for s in range(200)]
    hits_ob = [lidar.scan(oblique, np.array([1.0, 5.0, 0.0]), np.zeros((0, 4)), np.random.default_rng(s))[1][front] < 5.0 for s in range(200)]
    assert 0.65 < np.mean(hits) < 0.95
    assert np.mean(hits_ob) < 0.1


def test_cpp_cost_model_through_ctypes():
    m = CostModel(0.25)
    m.set_class(CLASS_IDS["person"], True, 1.2, 1.5, 0.5, 1, 0.6)
    m.add(CLASS_IDS["person"], 2.0, 2.0, 0.0, 0.5, 0.5, 0.0)
    grid = np.zeros((80, 80), np.uint8)
    assert m.stamp(grid, (0.0, 0.0), 0.05) > 0
    assert grid[40, 40] == LETHAL
    assert grid[40, 40 + 9] == INSCRIBED          # 0.45 m from centre = 0.2 m from surface
    assert 0 < grid[40, 40 + 25] < INSCRIBED      # 1.25 m from centre, inside the 1.2 m person radius
    assert m.prune(1.0) == 1 and m.count() == 0


def test_tuned_ekf_beats_wheel_odometry_and_untuned_does_not():
    wheel, tuned, untuned = [], [], []
    for s in range(5):
        rows = {r["estimator"]: r["final_pos_error_m"] for r in run_drift(100 + s)}
        wheel.append(rows["wheel_only"])
        tuned.append(rows["ekf_wheel_imu_tuned"])
        untuned.append(rows["ekf_wheel_imu_untuned"])
    assert np.mean(tuned) < 0.5 * np.mean(wheel)
    assert abs(np.mean(untuned) - np.mean(wheel)) < 0.2 * np.mean(wheel)


@pytest.mark.slow
def test_episode_is_reproducible_and_camera_layer_avoids_pallets():
    a = run_episode("cluttered_room", "D", 1001)
    b = run_episode("cluttered_room", "D", 1001)
    keys = ["success", "collisions", "path_length", "recoveries", "ate_rmse"]
    assert {k: a[k] for k in keys} == {k: b[k] for k in keys}
    assert a["success"] == 1 and a["collisions"] == 0
