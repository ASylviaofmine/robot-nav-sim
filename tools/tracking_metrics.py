"""Quantify how well the controller follows the planned path.

Runs the closed loop (A* plans on a grid -> go-to-goal drives along the
waypoints) and reports four numbers per run:

    time to goal      - efficiency
    distance travelled- 16.00 m is the planned length; more means detours
    end-point error   - accuracy at the goal
    cross-track error - how far the robot strayed from the planned polyline

The last one is the interesting one, and it is easy to get wrong: measuring
"distance to the nearest waypoint" is NOT cross-track error.  Waypoints sit
1 m apart, so a robot perfectly on the path is still 0.5 m away from the
nearest waypoint half the time.  The correct measure is the distance to the
nearest *segment* of the polyline (see ``dist_point_to_segment``).

Then it sweeps three controller knobs one at a time (tol / speed / Kp) and
prints Markdown tables, so the sensitivity claims in the README can be
regenerated and checked.

Run:  python tools/tracking_metrics.py
"""

import numpy as np
import matplotlib.pyplot as plt
from heapq import heappush, heappop


class Grid:
    def __init__(self, size):
        self.size = size
        self.cells = np.zeros((size, size))

    def set_wall(self, row, col):
        self.cells[row, col] = 1

    def is_wall(self, row, col):
        if row < 0 or row >= self.size or col < 0 or col >= self.size:
            return True
        return self.cells[row, col] == 1


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal):
    open_list = []
    heappush(open_list, (0, start))
    came_from = {}
    g = {start: 0}
    visited = set()

    while open_list:
        f, cur = heappop(open_list)
        if cur == goal:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            return path[::-1]
        if cur in visited:
            continue
        visited.add(cur)

        r, c = cur
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nb = (r + dr, c + dc)
            if grid.is_wall(nb[0], nb[1]):
                continue
            ng = g[cur] + 1
            if nb not in g or ng < g[nb]:
                g[nb] = ng
                came_from[nb] = cur
                heappush(open_list, (ng + heuristic(nb, goal), nb))
    return None


def cell_to_world(cell):
    """Grid cell (row, col) -> world point (x, y) at the cell centre."""
    r, c = cell
    return (c + 0.5, r + 0.5)


# --------------------------------------------------------------------------
# error metrics
# --------------------------------------------------------------------------

def dist_point_to_segment(p, a, b):
    """Shortest distance from point p to segment ab.

    The foot of the perpendicular may fall outside the segment, in which
    case the closest point is one of the endpoints.  Clamping the projection
    ratio t to [0, 1] handles both cases in one expression.
    """
    ab = b - a
    L2 = ab @ ab
    if L2 < 1e-12:                       # degenerate: a == b
        return float(np.hypot(*(p - a)))
    t = np.clip((p - a) @ ab / L2, 0.0, 1.0)
    proj = a + t * ab
    return float(np.hypot(*(p - proj)))


def crosstrack_error(traj, waypoints):
    """Per-sample distance from the trajectory to the planned polyline."""
    W = np.array(waypoints)
    A, B = W[:-1], W[1:]
    return np.array([min(dist_point_to_segment(np.array(p), A[i], B[i])
                         for i in range(len(A)))
                     for p in traj])


# --------------------------------------------------------------------------
# closed loop: plan, then drive
# --------------------------------------------------------------------------

def follow(waypoints, tol=0.05, speed=0.5, Kp=5.0, dt=0.05, t_max=200.0):
    """Drive the robot along the waypoints with a go-to-goal controller.

    Heading error is normalised into (-pi, pi]; without that step the
    accumulated angle grows without bound and the robot orbits instead of
    converging (measured: still stuck at waypoint 4 after 200 s).
    """
    x, y, theta = waypoints[0][0], waypoints[0][1], 0.0
    idx, t = 1, 0.0
    pts = [(x, y)]
    while idx < len(waypoints) and t < t_max:
        tx, ty = waypoints[idx]
        theta_target = np.arctan2(ty - y, tx - x)
        e = theta_target - theta
        e = (e + np.pi) % (2 * np.pi) - np.pi
        theta = theta + Kp * e * dt
        x = x + speed * np.cos(theta) * dt
        y = y + speed * np.sin(theta) * dt
        t = t + dt
        pts.append((x, y))
        if np.hypot(tx - x, ty - y) < tol:
            idx = idx + 1
    return pts, t


def evaluate(waypoints, tol=0.05, speed=0.5, Kp=5.0):
    pts, t = follow(waypoints, tol=tol, speed=speed, Kp=Kp)
    pts = np.array(pts)
    dist = float(np.sum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))))
    end_err = float(np.hypot(pts[-1, 0] - waypoints[-1][0],
                             pts[-1, 1] - waypoints[-1][1]))
    ct = crosstrack_error(pts, waypoints)
    return {"time": t, "dist": dist, "end_err": end_err,
            "max_ct": ct.max(), "mean_ct": ct.mean()}


def make_map():
    g = Grid(10)
    for c in range(1, 9):
        g.set_wall(5, c)                 # horizontal wall, two openings
    path = astar(g, (1, 1), (8, 8))
    return g, path, [cell_to_world(cell) for cell in path]


# --------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------

BASE = dict(tol=0.05, speed=0.5, Kp=5.0)
SCANS = [
    ("tol",   [0.02, 0.05, 0.10, 0.15, 0.20, 0.30]),
    ("speed", [0.30, 0.50, 0.80, 1.00, 1.50]),
    ("Kp",    [2.0, 3.0, 5.0, 8.0, 12.0]),
]


def main():
    g, path, waypoints = make_map()
    planned_len = len(waypoints) - 1

    print("# Tracking quality of the closed plan-and-drive loop\n")
    print(f"Map: 10x10 grid, horizontal wall (only two openings). "
          f"A* plans {planned_len + 1} cells / {planned_len} steps.\n")

    m = evaluate(waypoints, **BASE)
    print("## Baseline (tol = 0.05, speed = 0.5, Kp = 5.0)\n")
    print(f"- time to goal: {m['time']:.2f} s")
    print(f"- distance travelled: {m['dist']:.2f} m "
          f"(planned {planned_len:.2f} m, overhead "
          f"{m['dist'] - planned_len:+.2f} m)")
    print(f"- end-point error: {m['end_err']:.3f} m")
    print(f"- cross-track error: max {m['max_ct']:.4f} m, "
          f"mean {m['mean_ct']:.4f} m")
    print()

    for param, values in SCANS:
        print(f"## Sweeping `{param}` (everything else fixed at baseline)\n")
        print(f"| {param} | Time (s) | Distance (m) | End-point error (m) "
              f"| Max cross-track (m) | Mean cross-track (m) |")
        print("|---|---|---|---|---|---|")
        for v in values:
            kw = dict(BASE)
            kw[param] = v
            mm = evaluate(waypoints, **kw)
            print(f"| {v} | {mm['time']:.2f} | {mm['dist']:.2f} "
                  f"| {mm['end_err']:.3f} | {mm['max_ct']:.4f} "
                  f"| {mm['mean_ct']:.4f} |")
        print()

    print("## Readings\n")
    print("- `tol` trades end-point accuracy for a little time and distance: "
          "tightening it from 0.30 to 0.02 cuts the end-point error by 19x "
          "(0.289 -> 0.015 m) for +2.45 s and +1.23 m of travel.")
    print("- `tol` below 0.15 does not change the max cross-track at all "
          "(0.0895 m); that number is set by the heading gain, not by the "
          "arrival tolerance.")
    print("- `speed` is the only knob that changes the time materially "
          "(53.0 s -> 11.2 s) and it costs path fidelity almost linearly "
          "(max cross-track 0.057 -> 0.283 m); the end-point error stays "
          "flat because the controller still converges on the last segment.")
    print("- `Kp` is the cheapest win for tracking accuracy: 12.0 reduces the "
          "max cross-track 10x (0.248 -> 0.023 m) with no time penalty and "
          "no instability observed in this range.")

    # figure
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for ax, (param, values) in zip(axes, SCANS):
        rows = []
        for v in values:
            kw = dict(BASE)
            kw[param] = v
            rows.append((v, evaluate(waypoints, **kw)))
        xs = [v for v, _ in rows]
        ax.plot(xs, [m["end_err"] for _, m in rows], "o-",
                color="#E24B4A", label="end-point error")
        ax.plot(xs, [m["max_ct"] for _, m in rows], "s-",
                color="#378ADD", label="max cross-track")
        ax2 = ax.twinx()
        ax2.plot(xs, [m["time"] for _, m in rows], "^--",
                 color="#888780", alpha=0.7, label="time to goal")
        ax2.set_ylabel("time to goal (s)", color="#888780")
        ax2.tick_params(axis="y", labelcolor="#888780")
        ax.set_xlabel(param)
        ax.set_ylabel("error (m)")
        ax.set_title(f"sensitivity to {param}")
        ax.grid(True, alpha=0.3)
        l1, lb1 = ax.get_legend_handles_labels()
        l2, lb2 = ax2.get_legend_handles_labels()
        ax.legend(l1 + l2, lb1 + lb2, fontsize=8, loc="best")
    plt.tight_layout()
    plt.suptitle("Tracking error vs controller parameters", y=1.02)
    plt.savefig("docs/tracking_sensitivity.png", dpi=120,
                bbox_inches="tight")
    print("\n(figure written to docs/tracking_sensitivity.png)")


if __name__ == "__main__":
    main()
