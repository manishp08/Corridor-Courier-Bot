"""Builds docs/RESULTS.md and results/figures/*.png from the run CSVs.

Works on CSVs from either the Gazebo harness or the 2D surrogate (same
schema, cc_eval.results_io). Usage: python -m cc_eval.report <results_dir> <out.md>
"""
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

from .metrics import bootstrap_ci, mcnemar_exact, relative_reduction, wilcoxon_paired, wilson_ci
from .results_io import read_rows

CONFIG_LABELS = {
    "A": "A: LiDAR + wheel odom",
    "B": "B: A + EKF (IMU)",
    "C": "C: B + camera layer",
    "D": "D: C + people-aware",
}
WORLD_LABELS = {
    "empty_corridor": "1 Empty corridor",
    "cluttered_room": "2 Cluttered room (low obstacles)",
    "glass_wall": "3 Glass wall",
    "walking_actors": "4 Walking people",
    "kidnapped": "5 Kidnapped robot",
}
# Categorical slots 1-4 of the validated reference palette (light surface).
COLORS = {"A": "#2a78d6", "B": "#eb6834", "C": "#1baf7a", "D": "#eda100"}
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def _fmt(v, nd=2):
    return "–" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.{nd}f}"


def _ci(t, nd=2, pct=False):
    v, lo, hi = t
    if math.isnan(v):
        return "–"
    k = 100.0 if pct else 1.0
    return f"{v * k:.{0 if pct else nd}f} [{lo * k:.{0 if pct else nd}f}, {hi * k:.{0 if pct else nd}f}]"


def group(rows, *keys):
    g = defaultdict(list)
    for r in rows:
        g[tuple(r[k] for k in keys)].append(r)
    return g


def col(rs, k):
    return np.array([r[k] for r in rs], float)


def summarize(rs):
    n = len(rs)
    s = int(sum(r["success"] for r in rs))
    return {
        "n": n,
        "success": wilson_ci(s, n),
        "collisions": bootstrap_ci(col(rs, "collisions")),
        "clearance": bootstrap_ci(col(rs, "min_clearance_people")),
        "intrusion": bootstrap_ci(col(rs, "personal_space_s")),
        "time": bootstrap_ci(col(rs, "time_to_goal")),
        "path": bootstrap_ci(np.where(col(rs, "success") > 0, col(rs, "path_length"), np.nan)),
        "ate": bootstrap_ci(col(rs, "ate_rmse"), stat=np.median),
        "recoveries": bootstrap_ci(col(rs, "recoveries")),
        "cpu": float(np.nanmean(col(rs, "nav_cpu_ms"))),
        "layer": float(np.nanmean(col(rs, "perception_layer_ms"))) if n else float("nan"),
        "reloc": bootstrap_ci(col(rs, "relocalization_time"), stat=np.median),
        "reloc_rate": wilson_ci(int(np.sum(~np.isnan(col(rs, "relocalization_time")))), n),
    }


def _style(ax):
    ax.set_facecolor(SURFACE)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def _grouped_bars(path, title, ylabel, worlds, configs, values, errs=None, ylim=None, pct=False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 4.2), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    _style(ax)
    width = 0.8 / len(configs)
    x = np.arange(len(worlds))
    for k, c in enumerate(configs):
        v = np.array([values[(w, c)] for w in worlds], float) * (100 if pct else 1)
        pos = x - 0.4 + width * (k + 0.5)
        ax.bar(pos, v, width - 0.02, color=COLORS[c], label=CONFIG_LABELS[c], edgecolor=SURFACE, linewidth=1)
        if errs is not None:
            lo = np.array([errs[(w, c)][0] for w in worlds], float) * (100 if pct else 1)
            hi = np.array([errs[(w, c)][1] for w in worlds], float) * (100 if pct else 1)
            ax.errorbar(pos, v, yerr=[np.maximum(v - lo, 0), np.maximum(hi - v, 0)], fmt="none",
                        ecolor=INK2, elinewidth=1, capsize=2)
    ax.set_xticks(x, [WORLD_LABELS.get(w, w) for w in worlds], fontsize=9, color=INK)
    ax.set_ylabel(ylabel, color=INK2, fontsize=9)
    ax.set_title(title, loc="left", color=INK, fontsize=11)
    if ylim:
        ax.set_ylim(*ylim)
    ax.legend(frameon=False, fontsize=8, ncol=4, loc="upper left", bbox_to_anchor=(0, -0.12))
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def _drift_figure(path, drift_rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    labels = {"wheel_only": "Wheel odometry only", "ekf_wheel_imu_tuned": "EKF wheel+IMU (tuned)",
              "ekf_wheel_imu_untuned": "EKF wheel+IMU (untuned cov.)"}
    colors = {"wheel_only": COLORS["A"], "ekf_wheel_imu_tuned": COLORS["B"], "ekf_wheel_imu_untuned": COLORS["C"]}
    fig, ax = plt.subplots(figsize=(8, 3.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    _style(ax)
    gyros = ["calibrated", "uncalibrated"]
    ests = list(labels)
    width = 0.8 / len(ests)
    for k, e in enumerate(ests):
        vals, los, his = [], [], []
        for g in gyros:
            x = [r["final_pos_error_m"] for r in drift_rows if r["estimator"] == e and r["gyro"] == g]
            v, lo, hi = bootstrap_ci(x)
            vals.append(v)
            los.append(v - lo)
            his.append(hi - v)
        pos = np.arange(len(gyros)) - 0.4 + width * (k + 0.5)
        ax.bar(pos, vals, width - 0.02, color=colors[e], label=labels[e], edgecolor=SURFACE)
        ax.errorbar(pos, vals, yerr=[los, his], fmt="none", ecolor=INK2, elinewidth=1, capsize=2)
    ax.set_xticks(range(len(gyros)), ["Gyro bias calibrated at start-up", "Gyro not calibrated"], color=INK, fontsize=9)
    ax.set_ylabel("Final position error after 20 m loop (m)", color=INK2, fontsize=9)
    ax.set_title("Odometry drift on a 20 m square loop (30 seeds, mean and 95% CI)", loc="left", color=INK, fontsize=11)
    ax.legend(frameon=False, fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def _pooled(rows, configs, worlds):
    out = {}
    for c in configs:
        rs = [r for r in rows if r["config"] == c and r["world"] in worlds]
        out[c] = summarize(rs)
    return out


def build_report(results_dir, out_md, title_note=None):
    results_dir, out_md = Path(results_dir), Path(out_md)
    fig_dir = results_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    rows = read_rows(results_dir / "ablation.csv")
    drift_path = results_dir / "drift.csv"
    drift = []
    if drift_path.exists():
        with open(drift_path) as f:
            for r in csv.DictReader(f):
                for k in ("final_pos_error_m", "max_pos_error_m", "final_yaw_error_deg", "drift_percent", "distance_m"):
                    r[k] = float(r[k])
                drift.append(r)

    controllers = sorted({r["controller"] for r in rows})
    worlds = [w for w in WORLD_LABELS if any(r["world"] == w for r in rows)]
    configs = [c for c in CONFIG_LABELS if any(r["config"] == c for r in rows)]
    nav_worlds = [w for w in worlds if w != "kidnapped"]
    L = []
    L.append("# Results\n")
    L.append(title_note or (
        "> **Where these numbers come from.** Every figure below was produced by the 2D surrogate "
        "simulator in `sim/` (`make results`), not by Gazebo. The surrogate runs the same C++ costmap "
        "core as the Nav2 plugin and models the specific failure modes under study (objects below the "
        "LiDAR plane, specular glass, pedestrians, wheel slip, gyro bias), but it is a simplified "
        "world: treat the numbers as evidence about the *mechanisms*, and re-run the identical "
        "protocol in Gazebo (`ros2 run cc_eval run_ablation`) before quoting them as "
        "robot results. The CSV schema and this report generator are shared, so the Gazebo run "
        "produces the same tables.\n"))
    n_seeds = len({r["seed"] for r in rows})
    L.append(f"Protocol: {len(rows)} episodes = {len(worlds)} worlds × {len(configs)} configs × "
             f"{len(controllers)} controllers × {n_seeds} fixed seeds. Seeds are shared across configs "
             "(paired design): seed *k* builds the same world, clutter and pedestrians for every config. "
             "Nav2-equivalent parameters are identical in every world and config. Intervals are 95% "
             "(Wilson for rates, percentile bootstrap for means/medians).\n")

    # ------------------------------------------------------------ headline
    main_ctl = "dwb" if "dwb" in controllers else controllers[0]
    main = [r for r in rows if r["controller"] == main_ctl]
    pooled = _pooled(main, configs, nav_worlds)
    L.append(f"## Headline (controller: {main_ctl.upper()}, worlds 1–4 pooled)\n")
    L.append("| Config | Goal success | Collisions / run | Min. clearance to people (m) | "
             "Time within 0.5 m of people (s) | Recoveries / run |")
    L.append("|---|---|---|---|---|---|")
    for c in configs:
        s = pooled[c]
        L.append(f"| {CONFIG_LABELS[c]} | {_ci(s['success'], pct=True)} % | {_ci(s['collisions'])} | "
                 f"{_ci(s['clearance'])} | {_ci(s['intrusion'])} | {_ci(s['recoveries'])} |")
    headline = {}
    if "A" in pooled and "D" in pooled:
        a, d = pooled["A"], pooled["D"]
        headline = {
            "collision_reduction_pct": relative_reduction(a["collisions"][0], d["collisions"][0]),
            "success_A": 100 * a["success"][0], "success_D": 100 * d["success"][0],
            "n_runs": a["n"] + d["n"],
        }
        L.append("")
        L.append(f"* Collisions per run fell from **{a['collisions'][0]:.2f} (A) to {d['collisions'][0]:.2f} (D)**, "
                 f"a **{headline['collision_reduction_pct']:.0f}%** reduction.")
        L.append(f"* Goal success rose from **{headline['success_A']:.0f}% to {headline['success_D']:.0f}%** "
                 f"({a['n']} paired runs per config).")
    if drift:
        w = np.mean([r["final_pos_error_m"] for r in drift if r["estimator"] == "wheel_only" and r["gyro"] == "calibrated"])
        e = np.mean([r["final_pos_error_m"] for r in drift if r["estimator"] == "ekf_wheel_imu_tuned" and r["gyro"] == "calibrated"])
        headline["drift_reduction_pct"] = relative_reduction(w, e)
        L.append(f"* The tuned EKF cut 20 m-loop odometry drift from **{w:.2f} m to {e:.2f} m "
                 f"({headline['drift_reduction_pct']:.0f}% less)**.")
    L.append("")

    # ------------------------------------------------------------ per world
    for ctl in controllers:
        sub = [r for r in rows if r["controller"] == ctl]
        g = group(sub, "world", "config")
        stats_ = {k: summarize(v) for k, v in g.items()}
        L.append(f"## Per-scenario results — {ctl.upper()} controller\n")
        L.append("| World | Config | Success % [CI] | Collisions/run [CI] | Min clearance people (m) | "
                 "Time to goal (s) | Path length (m) | ATE median (m) | Recoveries/run | Nav CPU ms/cycle |")
        L.append("|---|---|---|---|---|---|---|---|---|---|")
        for w in worlds:
            for c in configs:
                s = stats_.get((w, c))
                if not s:
                    continue
                L.append(f"| {WORLD_LABELS[w]} | {c} | {_ci(s['success'], pct=True)} | {_ci(s['collisions'])} | "
                         f"{_ci(s['clearance'])} | {_fmt(s['time'][0], 1)} | {_fmt(s['path'][0], 1)} | "
                         f"{_fmt(s['ate'][0], 3)} | {_fmt(s['recoveries'][0])} | {_fmt(s['cpu'], 1)} |")
        L.append("")
        # Paired significance tests vs A
        L.append(f"Paired tests against config A ({ctl.upper()}): exact McNemar on success, Wilcoxon signed-rank on collisions.\n")
        L.append("| World | Config | Δ success (pp) | p (success) | Δ collisions/run | p (collisions) |")
        L.append("|---|---|---|---|---|---|")
        for w in worlds:
            base = sorted(g.get((w, "A"), []), key=lambda r: r["seed"])
            for c in configs:
                if c == "A" or not base:
                    continue
                other = sorted(g.get((w, c), []), key=lambda r: r["seed"])
                if [r["seed"] for r in other] != [r["seed"] for r in base]:
                    continue
                ds = 100 * (np.mean(col(other, "success")) - np.mean(col(base, "success")))
                dc = np.mean(col(other, "collisions")) - np.mean(col(base, "collisions"))
                ps = mcnemar_exact(col(base, "success") > 0, col(other, "success") > 0)
                pc = wilcoxon_paired(col(base, "collisions"), col(other, "collisions"))
                L.append(f"| {WORLD_LABELS[w]} | {c} | {ds:+.0f} | {ps:.3g} | {dc:+.2f} | {pc:.3g} |")
        L.append("")
        succ = {k: v["success"][0] for k, v in stats_.items()}
        succ_err = {k: (v["success"][1], v["success"][2]) for k, v in stats_.items()}
        _grouped_bars(fig_dir / f"success_{ctl}.png", f"Goal success rate by scenario ({ctl.upper()}, 95% Wilson CI)",
                      "Success (%)", worlds, configs, succ, succ_err, ylim=(0, 105), pct=True)
        coll = {k: v["collisions"][0] for k, v in stats_.items()}
        coll_err = {k: (v["collisions"][1], v["collisions"][2]) for k, v in stats_.items()}
        _grouped_bars(fig_dir / f"collisions_{ctl}.png", f"Collisions per run ({ctl.upper()}, mean and 95% bootstrap CI)",
                      "Collisions / run", worlds, configs, coll, coll_err)
        L.append(f"![Success rate]({_rel(fig_dir / f'success_{ctl}.png', out_md)})\n")
        L.append(f"![Collisions]({_rel(fig_dir / f'collisions_{ctl}.png', out_md)})\n")

    # ------------------------------------------------------------ controllers
    if len(controllers) > 1:
        L.append("## DWB vs MPPI (same configs, same seeds, worlds 1–4 pooled)\n")
        L.append("| Config | Controller | Success % | Collisions/run | Min clearance people (m) | Time to goal (s) | Nav CPU ms/cycle |")
        L.append("|---|---|---|---|---|---|---|")
        for c in configs:
            for ctl in controllers:
                s = summarize([r for r in rows if r["controller"] == ctl and r["config"] == c and r["world"] in nav_worlds])
                L.append(f"| {c} | {ctl.upper()} | {_ci(s['success'], pct=True)} | {_ci(s['collisions'])} | "
                         f"{_ci(s['clearance'])} | {_ci(s['time'], 1)} | {_fmt(s['cpu'], 1)} |")
        L.append("")

    # ------------------------------------------------------------ people
    if "walking_actors" in worlds and "C" in configs and "D" in configs:
        L.append("## People-aware inflation (D vs C, walking-people world)\n")
        L.append("Paired by seed, Wilcoxon signed-rank. *Time within 0.5 m* is the total time the robot's surface "
                 "was within 0.5 m of a person's; it was added after the first full run because the minimum "
                 "clearance (a single worst moment per run) turned out to be too noisy to separate C from D.\n")
        L.append("| Controller | Metric | C | D | p |")
        L.append("|---|---|---|---|---|")
        for ctl in controllers:
            gc = sorted([r for r in rows if r["controller"] == ctl and r["world"] == "walking_actors" and r["config"] == "C"], key=lambda r: r["seed"])
            gd = sorted([r for r in rows if r["controller"] == ctl and r["world"] == "walking_actors" and r["config"] == "D"], key=lambda r: r["seed"])
            if [r["seed"] for r in gc] != [r["seed"] for r in gd]:
                continue
            for key, label in (("min_clearance_people", "Min. clearance (m), mean"),
                               ("personal_space_s", "Time within 0.5 m (s), mean"),
                               ("collisions_people", "Person contacts / run"),
                               ("time_to_goal", "Time to goal (s)")):
                a, b = col(gc, key), col(gd, key)
                L.append(f"| {ctl.upper()} | {label} | {np.nanmean(a):.2f} | {np.nanmean(b):.2f} | {wilcoxon_paired(a, b):.3g} |")
        L.append("")

    # ------------------------------------------------------------ localization
    L.append("## Localization (milestone 4) and kidnapped-robot recovery\n")
    L.append("ATE = RMSE between the AMCL pose (map frame) and ground truth over the whole episode, "
             "equivalent to `evo_ape` without alignment.\n")
    L.append("| World | Config | ATE median (m) [CI] | Re-localized after kidnap | Median time to re-localize (s) |")
    L.append("|---|---|---|---|---|")
    g = group(main, "world", "config")
    for w in worlds:
        for c in configs:
            rs = g.get((w, c))
            if not rs:
                continue
            s = summarize(rs)
            reloc = f"{_ci(s['reloc_rate'], pct=True)} %" if w == "kidnapped" else "–"
            rt = _fmt(s["reloc"][0], 1) if w == "kidnapped" else "–"
            L.append(f"| {WORLD_LABELS[w]} | {c} | {_ci(s['ate'], 3)} | {reloc} | {rt} |")
    L.append("")

    # ------------------------------------------------------------ drift
    if drift:
        L.append("## EKF drift on a 20 m loop (milestone 3)\n")
        L.append("| Gyro | Estimator | Final position error (m) [CI] | Max error (m) | Final yaw error (°) | Drift (% of distance) |")
        L.append("|---|---|---|---|---|---|")
        for gy in ("calibrated", "uncalibrated"):
            for e in ("wheel_only", "ekf_wheel_imu_tuned", "ekf_wheel_imu_untuned"):
                rs = [r for r in drift if r["gyro"] == gy and r["estimator"] == e]
                if not rs:
                    continue
                L.append(f"| {gy} | {e} | {_ci(bootstrap_ci([r['final_pos_error_m'] for r in rs]), 3)} | "
                         f"{np.mean([r['max_pos_error_m'] for r in rs]):.3f} | "
                         f"{np.mean([r['final_yaw_error_deg'] for r in rs]):.2f} | "
                         f"{np.mean([r['drift_percent'] for r in rs]):.2f} |")
        L.append("")
        _drift_figure(fig_dir / "drift.png", drift)
        L.append(f"![Drift]({_rel(fig_dir / 'drift.png', out_md)})\n")

    # ------------------------------------------------------------ compute
    L.append("## Compute\n")
    L.append("Mean wall-clock per 10 Hz control cycle of the whole navigation stack as implemented in the "
             "surrogate (Python; one core of the build machine), and of the C++ perception layer "
             "(association + class-aware stamping). These are *relative* costs; ONNX detector latency is "
             "benchmarked separately in `results/perception_latency.md` when available.\n")
    L.append("| Controller | Config | Nav stack ms/cycle | Perception layer ms/cycle |")
    L.append("|---|---|---|---|")
    for ctl in controllers:
        for c in configs:
            rs = [r for r in rows if r["controller"] == ctl and r["config"] == c]
            L.append(f"| {ctl.upper()} | {c} | {np.nanmean(col(rs, 'nav_cpu_ms')):.1f} | "
                     f"{_fmt(float(np.nanmean(col(rs, 'perception_layer_ms'))) if c in ('C', 'D') else float('nan'), 2)} |")
    L.append("")
    lat = results_dir / "perception_latency.md"
    if lat.exists():
        L.append(lat.read_text())

    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text("\n".join(L) + "\n")
    print(f"wrote {out_md}")
    return headline


def _rel(p, md):
    import os
    return os.path.relpath(p, md.parent)


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: report <results_dir> <out.md>")
    build_report(sys.argv[1], sys.argv[2])


if __name__ == "__main__":
    main()
