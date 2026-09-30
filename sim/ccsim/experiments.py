"""Command-line entry point for every surrogate experiment.

  python -m ccsim.experiments ablation --seeds 30 --controllers dwb,mppi
  python -m ccsim.experiments drift --seeds 30
  python -m ccsim.experiments demo
  python -m ccsim.experiments report
"""
import argparse
import csv
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src" / "cc_eval"))

from cc_eval.results_io import write_rows  # noqa: E402

from .drift import run_drift  # noqa: E402
from .runner import run_episode  # noqa: E402
from .world import WORLD_NAMES  # noqa: E402

RESULTS = REPO / "results"
SEED_BASE = 1000   # seeds are SEED_BASE + i, identical across configs (paired design)


def _job(args):
    world, config, seed, controller = args
    return run_episode(world, config, seed, controller=controller)


def ablation(a):
    worlds = a.worlds.split(",")
    configs = a.configs.split(",")
    controllers = a.controllers.split(",")
    jobs = [(w, c, SEED_BASE + i, ctl) for ctl in controllers for w in worlds for c in configs
            for i in range(a.seeds)]
    RESULTS.mkdir(exist_ok=True)
    out = Path(a.out)
    t0 = time.time()
    rows = []
    with Pool(a.workers) as pool:
        for k, r in enumerate(pool.imap_unordered(_job, jobs, chunksize=1), 1):
            rows.append(r)
            if k % 25 == 0 or k == len(jobs):
                print(f"[{k}/{len(jobs)}] {time.time() - t0:.0f}s", flush=True)
    rows.sort(key=lambda r: (r["controller"], r["world"], r["config"], r["seed"]))
    write_rows(out, rows)
    print(f"wrote {out} ({len(rows)} runs)")


def drift(a):
    RESULTS.mkdir(exist_ok=True)
    rows = []
    for gyro in (False, True):
        for i in range(a.seeds):
            rows += run_drift(SEED_BASE + i, uncalibrated_gyro=gyro)
    fields = [k for k in rows[0] if not k.startswith("_")]
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {a.out} ({len(rows)} rows)")


def demo(a):
    from .demo import make_demo
    make_demo(RESULTS / "demo", a.fps)


def report(a):
    from cc_eval.report import build_report
    build_report(RESULTS, REPO / "docs" / "RESULTS.md")


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("ablation")
    s.add_argument("--seeds", type=int, default=30)
    s.add_argument("--worlds", default=",".join(WORLD_NAMES))
    s.add_argument("--configs", default="A,B,C,D")
    s.add_argument("--controllers", default="dwb")
    s.add_argument("--workers", type=int, default=os.cpu_count())
    s.add_argument("--out", default=str(RESULTS / "ablation.csv"))
    s.set_defaults(fn=ablation)
    s = sub.add_parser("drift")
    s.add_argument("--seeds", type=int, default=30)
    s.add_argument("--out", default=str(RESULTS / "drift.csv"))
    s.set_defaults(fn=drift)
    s = sub.add_parser("demo")
    s.add_argument("--fps", type=int, default=10)
    s.set_defaults(fn=demo)
    s = sub.add_parser("report")
    s.set_defaults(fn=report)
    a = p.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
