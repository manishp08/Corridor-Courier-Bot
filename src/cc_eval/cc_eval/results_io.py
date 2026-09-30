"""CSV schema shared by the Gazebo harness and the surrogate simulator."""
import csv
import math

FIELDS = [
    "world", "config", "controller", "seed", "success", "status", "time_to_goal", "path_length",
    "collisions", "collisions_static", "collisions_people", "min_clearance_people", "recoveries",
    "ate_rmse", "final_goal_error", "nav_cpu_ms", "perception_layer_ms", "global_relocalizations",
    "relocalization_time", "sim_time",
]
INT_FIELDS = {"seed", "success", "collisions", "collisions_static", "collisions_people", "recoveries",
              "global_relocalizations"}
STR_FIELDS = {"world", "config", "controller", "status"}


def write_rows(path, rows, mode="w"):
    with open(path, mode, newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        if mode == "w":
            w.writeheader()
        for r in rows:
            w.writerow(r)


def read_rows(path):
    out = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            row = {}
            for k, v in r.items():
                if k in STR_FIELDS:
                    row[k] = v
                elif k in INT_FIELDS:
                    row[k] = int(float(v))
                else:
                    row[k] = float(v) if v not in ("", "nan") else math.nan
            out.append(row)
    return out
