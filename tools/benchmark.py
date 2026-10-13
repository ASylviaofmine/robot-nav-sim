"""Benchmark harness for the A* grid planner.

This script mirrors the reference implementation in ``practice/day10_basics.py``
(same algorithm: 4-neighbour grid, Manhattan heuristic, binary-heap open list).
It is instrumented so we can report *nodes expanded*, and it adds a heuristic
weight ``w`` so the same code covers:

    w = 0  ->  Dijkstra (no heuristic, uniform-cost search)
    w = 1  ->  standard A* (optimal for a consistent heuristic)
    w > 1  ->  weighted A*, i.e. greedy-ish: faster but no optimality guarantee

Path lengths produced here are cross-checked against the reference
implementation so the numbers in the README stay honest.

Usage:
    python tools/benchmark.py
"""

from __future__ import annotations

import contextlib
import io
import os
import random
import statistics
import sys
import time
from heapq import heappop, heappush

import matplotlib

matplotlib.use("Agg")

# ---------------------------------------------------------------------------
# 1. Reference implementation (imported from the learning notes, output muted)
# ---------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "practice"))

import matplotlib.pyplot as plt  # noqa: E402

plt.show = lambda *args, **kwargs: None  # importing the notes triggers a demo

with contextlib.redirect_stdout(io.StringIO()):
    import day10_basics as ref  # noqa: E402

Grid = ref.Grid


# ---------------------------------------------------------------------------
# 2. Instrumented planner
# ---------------------------------------------------------------------------
def astar(grid, start, goal, w=1.0, mode="manhattan"):
    """A* on a 4-connected grid. Returns (path, stats).

    ``mode`` selects the heuristic: "manhattan" (default), "euclidean"
    (also admissible on a 4-connected grid, but weaker) or "none" (Dijkstra).

    Returned path includes start and goal; ``None`` means 'no path exists'.
    """
    t0 = time.perf_counter()
    math_hypot = __import__("math").hypot

    def h(node):
        dr, dc = abs(node[0] - goal[0]), abs(node[1] - goal[1])
        if mode == "manhattan":
            base = dr + dc
        elif mode == "euclidean":
            base = math_hypot(dr, dc)
        else:
            base = 0.0
        return w * base

    open_list = [(0.0, start)]
    came_from = {}
    g = {start: 0}
    expanded = 0

    while open_list:
        _, cur = heappop(open_list)
        if cur == goal:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            return path[::-1], {
                "expanded": expanded,
                "seconds": time.perf_counter() - t0,
            }

        expanded += 1
        r, c = cur
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nb = (r + dr, c + dc)
            if grid.is_wall(nb[0], nb[1]):
                continue
            ng = g[cur] + 1
            if nb not in g or ng < g[nb]:
                g[nb] = ng
                came_from[nb] = cur
                heappush(open_list, (ng + h(nb), (nb[0], nb[1])))

    return None, {"expanded": expanded, "seconds": time.perf_counter() - t0}


# ---------------------------------------------------------------------------
# 3. Test maps
# ---------------------------------------------------------------------------
def bfs_length(grid, start, goal):
    """Independent breadth-first search: ground truth for 'is there a shorter route?'."""
    from collections import deque

    if grid.is_wall(*start) or grid.is_wall(*goal):
        return None
    seen = {start: 0}
    q = deque([start])
    while q:
        cur = q.popleft()
        if cur == goal:
            return seen[cur]
        r, c = cur
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nb = (r + dr, c + dc)
            if nb in seen or grid.is_wall(nb[0], nb[1]):
                continue
            seen[nb] = seen[cur] + 1
            q.append(nb)
    return None


def build(size, walls, start, goal):
    g = Grid(size)
    for r, c in walls:
        g.set_wall(r, c)
    return g, start, goal


def map_open():
    return build(10, [], (1, 1), (8, 8))


def map_wall_horizontal():
    """Row 5 blocked except column 0 and column 9 -> one detour."""
    return build(10, [(5, c) for c in range(1, 9)], (1, 1), (8, 8))


def map_wall_vertical():
    """Column 5 blocked except row 0 and row 9 -> detour around the end."""
    return build(10, [(r, 5) for r in range(1, 9)], (1, 1), (8, 8))


def map_maze_serpentine():
    """Two half-walls forming an S-shaped corridor."""
    walls = [(r, 3) for r in range(1, 8)] + [(r, 6) for r in range(2, 9)]
    return build(10, walls, (1, 1), (8, 8))


def map_unreachable():
    """Goal is sealed off -> planner must report failure."""
    walls = [(7, 8), (9, 8), (8, 7), (8, 9)]
    return build(10, walls, (1, 1), (8, 8))


NAMED_MAPS = [
    ("open 10x10", map_open, 14),
    ("horizontal wall", map_wall_horizontal, 16),
    ("vertical wall", map_wall_vertical, 16),
    ("serpentine maze", map_maze_serpentine, 16),
    ("sealed goal (no path)", map_unreachable, None),
]

REPEATS = 200


def bench(fn, repeats=REPEATS):
    """Run a planner callable ``repeats`` times, return aggregated stats."""
    runs = [fn() for _ in range(repeats)]
    seconds = min(stats["seconds"] for _, stats in runs)   # least-contended sample
    return runs[0], seconds * 1000


# ---------------------------------------------------------------------------
# 4. Reports
# ---------------------------------------------------------------------------
def report_named_maps():
    print("### 1. Path length & runtime on hand-built maps (A*, w = 1, "
          f"best of {REPEATS} runs)\n")
    print("| Map | Grid | Solution | Path length | Steps | Nodes expanded | "
          "Runtime (ms) | BFS optimum |")
    print("|---|---|---|---|---|---|---|---|")
    for name, maker, _ in NAMED_MAPS:
        g, start, goal = maker()
        (path, stats), ms = bench(lambda g=g, s=start, t=goal: astar(g, s, t))
        opt = bfs_length(g, start, goal)
        opt_txt = "—" if opt is None else f"{opt} steps ✓" if path and len(path) - 1 == opt \
            else ("unreachable ✓" if path is None and opt is None else "MISMATCH")
        if path is None:
            print(f"| {name} | 10x10 | no | — | — | {stats['expanded']} | "
                  f"{ms:.3f} | {opt_txt} |")
        else:
            cells, steps = len(path), len(path) - 1
            print(f"| {name} | 10x10 | yes | {cells} | {steps} | "
                  f"{stats['expanded']} | {ms:.3f} | {opt_txt} |")


def path_is_valid(grid, path, start, goal):
    """Sanity checks: endpoints correct, contiguous moves, never inside a wall."""
    if path is None:
        return False
    if path[0] != start or path[-1] != goal:
        return False
    for (r, c) in path:
        if grid.is_wall(r, c):
            return False
    for a, b in zip(path, path[1:]):
        if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:   # 4-connected moves only
            return False
    return len(path) == len(set(path))                 # no revisits


def report_random_maps():
    print("\n### 2. Random obstacle fields (20x20 grid, 25 % walls, "
          "20 maps, seed 42)\n")
    rng = random.Random(42)
    solved, steps_list, expanded_list, ms_list = 0, [], [], []
    truly_blocked = 0
    optimal = 0
    valid = 0
    total = 20
    for _ in range(total):
        walls = []
        for r in range(20):
            for c in range(20):
                if rng.random() < 0.25:
                    walls.append((r, c))
        start, goal = (0, 0), (19, 19)
        walls = [w for w in walls if w not in (start, goal)]
        g, s, t = build(20, walls, start, goal)
        (path, stats), ms = bench(lambda g=g, s=s, t=t: astar(g, s, t), repeats=20)
        opt = bfs_length(g, s, t)
        if path is not None:
            solved += 1
            steps_list.append(len(path) - 1)
            if path_is_valid(g, path, s, t):
                valid += 1
            if opt is not None and len(path) - 1 == opt:
                optimal += 1
        elif opt is None:
            truly_blocked += 1
        expanded_list.append(stats["expanded"])
        ms_list.append(ms)
    print(f"- success rate: **{solved}/{total} = {100 * solved / total:.0f} %**; "
          f"the {total - solved} failures were confirmed blocked by an "
          f"independent BFS ({truly_blocked}/{total - solved})")
    print(f"- collision-free & contiguous paths (no wall hit, no diagonal jump): "
          f"**{valid}/{solved}**")
    print(f"- optimal step count, checked against BFS: **{optimal}/{solved}**")
    if steps_list:
        print(f"- path length: min {min(steps_list)}, median "
              f"{statistics.median(steps_list):.0f}, max {max(steps_list)} steps")
    print(f"- nodes expanded: median {statistics.median(expanded_list):.0f} "
          f"of 400 cells ({100 * statistics.median(expanded_list) / 400:.0f} %)")
    print(f"- runtime: median {statistics.median(ms_list):.3f} ms")


def report_heuristic_weight():
    print("\n### 3. Heuristic choice and weight (identical maps, identical code)\n")
    print("| Heuristic | w | Search | 10x10 wall | 20x20 random (25 %) |")
    print("|---|---|---|---|---|")
    small = map_wall_horizontal()
    g_s, s_s, t_s = small
    rng = random.Random(42)
    walls = [(r, c) for r in range(20) for c in range(20) if rng.random() < 0.25]
    g_b, s_b, t_b = build(20, walls, (0, 0), (19, 19))
    combos = [
        ("none", 0, "Dijkstra (no heuristic)"),
        ("euclidean", 1, "admissible, weaker"),
        ("manhattan", 1, "admissible, consistent"),
        ("manhattan", 2, "not admissible"),
        ("manhattan", 5, "near-greedy"),
    ]
    for mode, w, label in combos:
        (p1, st1), ms1 = bench(lambda: astar(g_s, s_s, t_s, w, mode), repeats=200)
        (p2, st2), ms2 = bench(lambda: astar(g_b, s_b, t_b, w, mode),
                               repeats=20)
        cell1 = (f"{len(p1) - 1} steps, {st1['expanded']} nodes, {ms1:.3f} ms"
                 if p1 else "no path")
        cell2 = (f"{len(p2) - 1} steps, {st2['expanded']} nodes, {ms2:.3f} ms"
                 if p2 else "no path")
        name = "none" if mode == "none" else f"{mode}"
        print(f"| {name} | {w} | {label} | {cell1} | {cell2} |")


def cross_check_reference():
    print("\n### 4. Cross-checks (reference implementation & BFS ground truth)\n")
    g, s, t = map_wall_horizontal()
    path = ref.astar(g, s, t)
    mine, _ = astar(g, s, t)
    print(f"- reference `practice/day10_basics.astar()` -> {len(path)} cells, "
          f"{len(path) - 1} steps")
    print(f"- benchmark `astar()`                        -> {len(mine)} cells, "
          f"{len(mine) - 1} steps")
    print(f"- identical path: **{path == mine}**")

    print("- A* vs BFS optimum on every hand-built map:")
    all_valid = 0
    solvable = 0
    for name, maker, _ in NAMED_MAPS:
        gx, sx, tx = maker()
        p, _stats = astar(gx, sx, tx)
        opt = bfs_length(gx, sx, tx)
        got = None if p is None else len(p) - 1
        if p is not None:
            solvable += 1
            if path_is_valid(gx, p, sx, tx):
                all_valid += 1
        print(f"    - {name}: A* = {got}, BFS = {opt} -> "
              f"**{'match' if got == opt else 'MISMATCH'}**"
              + ("" if p is None else f", path valid: {path_is_valid(gx, p, sx, tx)}"))
    print(f"- well-formed paths on hand-built maps: **{all_valid}/{solvable}**")

    print("- optimality is heuristic-independent (admissible heuristics only):")
    gh, sh, th = map_wall_horizontal()
    for mode in ("none", "euclidean", "manhattan"):
        p, _st = astar(gh, sh, th, 1.0, mode)
        print(f"    - {mode:>9}: {len(p) - 1} steps "
              f"(BFS optimum {bfs_length(gh, sh, th)})")
    p2, _ = astar(gh, sh, th, 5.0, "manhattan")
    print(f"    -  w=5 greedy: {len(p2) - 1} steps -> may lose optimality")

    g2, s2, t2 = map_unreachable()
    print(f"- sealed goal: reference returns `{ref.astar(g2, s2, t2)}` "
          "(caller must handle `None`)")


if __name__ == "__main__":
    report_named_maps()
    report_random_maps()
    report_heuristic_weight()
    cross_check_reference()
