"""Drives the full Gazebo ablation: one fresh stack per episode (clean state,
fixed seed), harness appends to the CSV, report at the end.

  ros2 run cc_eval run_ablation --seeds 30 --controllers dwb,mppi --out results/gazebo

Budget: ~1-2 min per episode real time; 5 worlds x 4 configs x 30 seeds x 2
controllers = 1200 episodes, so run it overnight or shard with --worlds.
"""
import argparse
import os
import signal
import subprocess
import time
from pathlib import Path

WORLDS = ["empty_corridor", "cluttered_room", "glass_wall", "walking_actors", "kidnapped"]


def episode(world, config, controller, seed, out, detector, bags, headless):
    launch = subprocess.Popen(
        ["ros2", "launch", "cc_navigation", "bringup.launch.py", f"world:={world}", f"ablation:={config}",
         f"controller:={controller}", f"detector:={detector}", f"seed:={seed}", f"headless:={headless}"],
        start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        time.sleep(15)       # gz + Nav2 bring-up; the harness also waits for Nav2 to be active
        cmd = ["ros2", "run", "cc_eval", "harness", "--ros-args", "-p", "use_sim_time:=true",
               "-p", f"world:={world}", "-p", f"config:={config}", "-p", f"controller:={controller}",
               "-p", f"seed:={seed}", "-p", f"out_csv:={out / 'ablation.csv'}"]
        if bags:
            cmd += ["-p", f"bag_dir:={out / 'bags'}"]
        subprocess.run(cmd, timeout=600, check=False)
    finally:
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        subprocess.run(["pkill", "-f", "gz sim"], check=False)
        time.sleep(3)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seeds", type=int, default=30)
    p.add_argument("--seed-base", type=int, default=1000)
    p.add_argument("--worlds", default=",".join(WORLDS))
    p.add_argument("--configs", default="A,B,C,D")
    p.add_argument("--controllers", default="dwb")
    p.add_argument("--detector", default="oracle")
    p.add_argument("--out", default="results/gazebo")
    p.add_argument("--bags", action="store_true")
    p.add_argument("--headless", default="true")
    a = p.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for ctl in a.controllers.split(","):
        for w in a.worlds.split(","):
            for c in a.configs.split(","):
                for i in range(a.seeds):
                    print(f"== {w} {c} {ctl} seed {a.seed_base + i}", flush=True)
                    episode(w, c, ctl, a.seed_base + i, out, a.detector, a.bags, a.headless)
    from .report import build_report
    build_report(out, out / "RESULTS.md", title_note="> Gazebo Harmonic results (cc_eval/run_ablation.py).\n")


if __name__ == "__main__":
    main()
