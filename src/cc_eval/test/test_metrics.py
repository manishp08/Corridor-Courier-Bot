import math

import numpy as np

from cc_eval.metrics import (ate_rmse, bootstrap_ci, count_contacts, mcnemar_exact, min_clearance,
                             path_length, relative_reduction, wilcoxon_paired, wilson_ci)
from cc_eval.results_io import FIELDS, read_rows, write_rows


def test_wilson_matches_reference():
    p, lo, hi = wilson_ci(27, 30)
    assert abs(p - 0.9) < 1e-12
    assert abs(lo - 0.7438) < 1e-3 and abs(hi - 0.9654) < 1e-3   # standard Wilson 95% values
    assert wilson_ci(0, 30)[1] == 0.0 and wilson_ci(30, 30)[2] == 1.0


def test_bootstrap_contains_mean_and_ignores_nan():
    v, lo, hi = bootstrap_ci([1, 2, 3, 4, np.nan])
    assert v == 2.5 and lo <= v <= hi


def test_paired_tests():
    a = [1] * 20 + [0] * 10
    b = [1] * 30
    assert mcnemar_exact(a, b) < 0.01
    assert mcnemar_exact(a, a) == 1.0
    assert wilcoxon_paired([0] * 10, [0] * 10) == 1.0
    assert wilcoxon_paired(np.arange(20) + 3, np.arange(20)) < 0.01


def test_geometry_metrics():
    assert ate_rmse([[0, 0], [1, 1]], [[0, 1], [1, 2]]) == 1.0
    assert path_length([[0, 0], [3, 4], [3, 5]]) == 6.0
    robot = np.array([[0, 0], [1, 0]])
    people = np.array([[[3, 0]], [[2, 0]]])
    assert abs(min_clearance(robot, people, 0.25, 0.25) - 0.5) < 1e-12
    assert math.isnan(min_clearance(robot, np.zeros((2, 0, 2)), 0.25, 0.25))


def test_contacts_count_rising_edges():
    seq = [set(), {"a"}, {"a"}, {"a", "b"}, set(), {"a"}]
    assert count_contacts(seq) == 3


def test_relative_reduction():
    assert relative_reduction(2.0, 0.5) == 75.0
    assert math.isnan(relative_reduction(0.0, 0.0))


def test_csv_roundtrip(tmp_path):
    row = {k: 0 for k in FIELDS}
    row.update(world="w", config="A", controller="dwb", status="reached", ate_rmse=float("nan"))
    write_rows(tmp_path / "x.csv", [row])
    back = read_rows(tmp_path / "x.csv")[0]
    assert back["world"] == "w" and back["seed"] == 0 and math.isnan(back["ate_rmse"])
