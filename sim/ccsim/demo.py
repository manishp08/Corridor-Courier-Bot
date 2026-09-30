"""Demo animations: the same seed run with config A and config D side by side."""
from pathlib import Path

import numpy as np

from .runner import run_episode
from .world import LIDAR_HEIGHT, ROBOT_RADIUS

SURFACE, INK, INK2 = "#fcfcfb", "#0b0b0b", "#52514e"
C_A, C_D = "#2a78d6", "#eda100"
GLASS = "#1baf7a"
PERSON = "#e34948"

DEMOS = [
    ("glass_wall", 1003, ["A", "D"]),
    ("cluttered_room", 1001, ["A", "D"]),
    ("walking_actors", 1002, ["B", "D"]),
]


def _draw_static(ax, world):
    import matplotlib.patches as mp
    for x1, y1, x2, y2 in world.walls:
        ax.plot([x1, x2], [y1, y2], color=INK, lw=2, solid_capstyle="round")
    for x1, y1, x2, y2 in world.glass:
        ax.plot([x1, x2], [y1, y2], color=GLASS, lw=3, alpha=0.8, solid_capstyle="round")
    for o in world.obstacles:
        invisible = o.height < LIDAR_HEIGHT or o.kind == "cart"
        r = mp.Rectangle((-o.sx / 2, -o.sy / 2), o.sx, o.sy, facecolor="#c3c2b7" if invisible else "#52514e",
                         edgecolor=INK2, hatch="///" if invisible else None, lw=0.8)
        import matplotlib.transforms as mt
        r.set_transform(mt.Affine2D().rotate(o.yaw).translate(o.x, o.y) + ax.transData)
        ax.add_patch(r)
    ax.plot(*world.goal, marker="*", color=INK, ms=12)
    x0, y0, x1, y1 = world.bounds
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)


def make_demo(out_dir, fps=10, stride=3):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for world_name, seed, configs in DEMOS:
        runs = [run_episode(world_name, c, seed, record=True) for c in configs]
        w = runs[0]["world_obj"]
        x0, y0, x1, y1 = w.bounds
        aspect = (y1 - y0) / (x1 - x0)
        fig, axes = plt.subplots(len(runs), 1, figsize=(8, max(2.0, 8 * aspect) * len(runs) + 0.6), dpi=80)
        fig.patch.set_facecolor(SURFACE)
        artists = []
        for ax, r, c, col in zip(axes, runs, configs, [C_A, C_D]):
            ax.set_facecolor(SURFACE)
            res, org = r["costmap_res"], r["costmap_origin"]
            cm0 = r["cost_trace"][0][1]
            im = ax.imshow(cm0, origin="lower", cmap="Greys", vmin=0, vmax=300, alpha=0.55,
                           extent=(org[0], org[0] + cm0.shape[1] * res, org[1], org[1] + cm0.shape[0] * res))
            _draw_static(ax, w)
            path_line, = ax.plot([], [], color=col, lw=1, ls="--")
            trail, = ax.plot([], [], color=col, lw=2)
            robot = plt.Circle((0, 0), ROBOT_RADIUS, color=col, zorder=5)
            ax.add_patch(robot)
            head, = ax.plot([], [], color=INK, lw=1.5, zorder=6)
            people = ax.scatter([], [], s=90, color=PERSON, zorder=4)
            txt = ax.text(0.01, 0.98, "", transform=ax.transAxes, va="top", fontsize=9, color=INK)
            status = "success" if r["success"] else r["status"]
            ax.set_title(f"Config {c}: {status}, {r['collisions']} collision(s), {r['recoveries']} recoveries",
                         loc="left", fontsize=10, color=INK)
            artists.append((r, im, path_line, trail, robot, head, people, txt))
        n = max(len(r["trace"]) for r in runs)
        frames = list(range(0, n, stride)) + [n - 1] * fps

        def update(k):
            out = []
            for r, im, path_line, trail, robot, head, people, txt in artists:
                tr = r["trace"]
                i = min(k, len(tr) - 1)
                t, x, y, th = tr[i, 0], tr[i, 1], tr[i, 2], tr[i, 3]
                trail.set_data(tr[:i + 1, 1], tr[:i + 1, 2])
                robot.center = (x, y)
                head.set_data([x, x + 0.4 * np.cos(th)], [y, y + 0.4 * np.sin(th)])
                snaps = [s for s in r["cost_trace"] if s[0] <= i]
                if snaps:
                    im.set_data(snaps[-1][1])
                    p = snaps[-1][2]
                    path_line.set_data(*(p.T if p is not None else ([], [])))
                a = r["actor_trace"][i]
                people.set_offsets(a if len(a) else np.zeros((0, 2)))
                txt.set_text(f"t = {t:5.1f} s")
                out += [im, path_line, trail, robot, head, people, txt]
            return out

        fig.tight_layout()
        anim = FuncAnimation(fig, update, frames=frames, blit=False)
        gif = out_dir / f"{world_name}_{'_vs_'.join(configs)}.gif"
        anim.save(gif, writer=PillowWriter(fps=fps))
        update(n - 1)
        fig.savefig(out_dir / f"{world_name}_{'_vs_'.join(configs)}_final.png", facecolor=SURFACE)
        plt.close(fig)
        print("wrote", gif)
