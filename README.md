# robot-nav-sim

**A 2D mobile-robot autonomous navigation simulator — grid path planning and feedback control, implemented from scratch.**

Status: 🚧 in development — the planning and control core works and is measured (see [Measured results](#measured-results)); the real-time viewer and the end-to-end demo are still open.

---

## What this project is

`robot-nav-sim` is a small simulator for the classic navigation problem: *given a map with obstacles, get a differential-drive robot from a start pose to a goal pose — safely, accurately and observably.*

The robot is not teleoperated and nothing is scripted: the program computes a collision-free route over a grid, then a feedback controller drives the robot along it. Right now the pieces are implemented and measured separately; wiring them into one animated run is the next milestone.

```
        map + start + goal
                │
                ▼
      ┌───────────────────┐        planned path
      │  planner          │ ──────────────────────┐
      │  A* on a grid     │                       │
      │  (RRT: planned)   │                       ▼
      └───────────────────┘             ┌───────────────────┐
                                        │  controller       │
        robot pose (x, y, θ)            │  PID + go-to-goal │
                ▲                       └───────────────────┘
                │                                 │
                └──────── wheel commands ─────────┘
```

## Goals

- Implement the full navigation stack — planning, kinematics, control, simulation — **from scratch**, without a robotics framework, so every formula in the loop is code I can explain.
- Turn control-theory coursework (PID, damping, steady-state error, disturbances) into a **running, verifiable** system rather than transfer functions on paper.
- Keep the project honest: every claim in this README is backed by a script in `tools/` that anyone can re-run.

## Repository layout

| Path | Contents |
|---|---|
| `practice/` | Daily implementations (`dayNN_*.py`), one file per learning step. Kept on purpose: the Git history of this folder is the build-up order of the project. |
| `tools/benchmark.py` | Planner benchmark: path length, nodes expanded, runtime, success rate, heuristic comparison, plus a BFS ground-truth check. |
| `tools/pid_tuning.py` | Turns four PID parameter sets into measurable metrics (error, overshoot, rise/settling time). |
| `main.py` | Entry point — currently a placeholder while the pieces are still separate. |
| `requirements.txt`, `LICENSE`, `README.md` | Environment, licence, documentation. |

> The algorithms currently live in `practice/` (indexed by day). Consolidating them into a proper `src/` package with stable interfaces is on the roadmap; until then this README describes exactly what exists today.

## Algorithms

### 1. Grid path planning — A*

**How it works.** The world is a 4-connected grid where each cell is free or blocked. A* keeps a priority queue of candidate cells ordered by

```
f(n) = g(n) + h(n)
```

- `g(n)` — cost actually paid to reach `n` (1 per step, uniform);
- `h(n)` — Manhattan distance to the goal, an estimate of the cost still to go;
- the cheapest-`f` cell is expanded next; when the goal is popped, the route is recovered by walking the `came_from` dictionary backwards.

**Why A\* and not something else.**

| Alternative | Why not (for this project) |
|---|---|
| Dijkstra (or BFS) | Same result, but expands far more cells — it has no direction preference. See the measured 1.9× difference below. |
| Best-first / greedy search | Cheaper still, but no optimality guarantee; it can be lured into dead ends by obstacles. |
| RRT / RRT* | Sampling-based, built for high-dimensional or non-holonomic spaces; on a small 2D grid the solution is noisy and far from optimal for the same effort. Planned as a *comparison* baseline, not as the main planner. |

The choice of **Manhattan distance as heuristic** is deliberate: on a 4-connected grid where each step costs 1, it never overestimates the remaining cost (admissible) and satisfies the triangle inequality (consistent), which is exactly what makes A* return an optimal path — verified against an independent BFS in `tools/benchmark.py`.

**Complexity.** With `N` free cells and a binary heap:

- time: `O(E log V)`; on a 4-connected grid `E ≈ 4N`, so `O(N log N)` in the worst case (the planner may have to expand the whole reachable region);
- space: `O(N)` for the open list, the `g` table and `came_from`.

The worst case is rare in practice: the heuristic prunes the search, which is precisely what the measurements below show.

**Measured behaviour (A\*, `h` = Manhattan).** Best of 200 runs, 10×10 grids, Windows / Python 3.13.15 (node counts and path lengths are deterministic; runtimes move ±20 % between runs, so treat the last column as indicative):

| Map | Solution | Path length | Steps | Nodes expanded (of 100) | Runtime (ms) |
|---|---|---|---|---|---|
| open grid, S(1,1) → G(8,8) | yes | 15 cells | 14 | 63 | 0.43 |
| horizontal wall (only two openings) | yes | 17 cells | 16 | 78 | 0.49 |
| vertical wall (only two openings) | yes | 17 cells | 16 | 71 | 0.45 |
| serpentine maze (two half-walls) | yes | 17 cells | 16 | 65 | 0.40 |
| goal sealed off | **no path** | — | — | 94 | 0.55 |

Every returned path equals the BFS optimum, and the unreachable case is detected rather than looping forever (`astar()` returns `None`; the caller must handle it).

**Choosing the heuristic** — the same code with only the heuristic changed (`none` = Dijkstra, `w = 2` and `w = 5` are weighted A*: not admissible, so no optimality guarantee):

| Heuristic | w | Search type | 10×10 wall | 20×20 random (25 % walls) |
|---|---|---|---|---|
| none | 0 | Dijkstra | 16 steps, 88 nodes, 0.52 ms | 38 steps, 250 nodes, 1.55 ms |
| Euclidean | 1 | admissible, weaker | 16 steps, 79 nodes, 0.53 ms | 38 steps, 161 nodes, 1.04 ms |
| **Manhattan** | **1** | admissible, consistent | 16 steps, **78 nodes**, 0.44 ms | 38 steps, **130 nodes**, 0.76 ms |
| Manhattan | 2 | not admissible | 16 steps, 20 nodes, 0.14 ms | 38 steps, 46 nodes, 0.29 ms |
| Manhattan | 5 | near-greedy | 16 steps, 17 nodes, 0.13 ms | 40 steps, 45 nodes, 0.30 ms |

Three readings from this table:

- **The heuristic is what makes A\* fast.** Dropping it (Dijkstra) expands 250 nodes on the 20×20 map where A* needs 130 — the same answer for nearly twice the work, which is the concrete reason A* was chosen.
- **Manhattan beats Euclidean here.** On a 4-connected grid Euclidean distance underestimates the true remaining cost much more (161 vs 130 nodes); Manhattan exactly matches the cost model, so it prunes harder while staying admissible and consistent, and therefore still optimal.
- **Weighting is a real trade-off, not free speed.** `w = 2` cuts the search to 46 nodes with the same path on these maps; `w = 5` also looks fine on the wall map (16 steps) but already returns 40 steps instead of 38 on the random suite — suboptimal paths are the price.

### 2. Motion control — PID

A discrete PID controller drives the plant

```
force -> acceleration -> velocity -> position        (Euler integration, dt = 0.05 s)
u = Kp·e + Ki·∫e dt + Kd·de/dt,       e = target − x
```

The tuning experiment (four parameter sets, 20 s horizon) is where the classical story shows up as numbers:

| Controller | Kp / Ki / Kd | Final position (m) | Steady-state error (m) | Overshoot | Rise time, 90 % (s) | Settling time, ±2 % (s) |
|---|---|---|---|---|---|---|
| P only | 1 / 0 / 0 | 6.167 | +3.833 | 100 % | 1.40 | never (oscillates) |
| PD | 1 / 0 / 2 | 10.000 | +0.000 | 0.0 % | 3.85 | 5.90 |
| PD + friction | 1 / 0 / 2 | 9.500 | +0.500 | 0.0 % | 4.70 | never |
| PID + friction | 1 / 0.3 / 2 | 9.988 | +0.012 | 29.7 % | 2.50 | 16.05 |

Interpretation: on a double-integrator plant, P alone is an undamped oscillator (it never converges); the D term supplies damping; a constant disturbance (friction = 0.5 N) leaves a residual error that no amount of P or D can remove; the I term integrates that residual away, at the price of some overshoot.

### 3. Heading control — go-to-goal

For a differential-drive robot the interesting part is orientation, not just position. The controller commands an angular rate proportional to the heading error:

```
θ_target = atan2(goal_y − y, goal_x − x)
e        = θ_target − θ
θ += Kp · e · dt                       # turn towards the goal
x += speed · cos(θ) · dt                # move forwards
y += speed · sin(θ) · dt                #   in the direction faced
stop when hypot(goal − pose) < tolerance
```

Measured: from (0, 0) heading 0 rad towards (5, 5), the robot reaches (4.94, 4.94) — inside the 0.1 m tolerance — in 13.95 s of simulated time; a pure heading step to 45° settles at 45.0°.

### 4. RRT (planned, not implemented)

A sampling-based planner is planned as a comparison baseline on identical maps — same start/goal, same collision checker, different search paradigm (random sampling + rewiring vs. exhaustive best-first). It will be added to the same benchmark table so the comparison is measured, not asserted.

## Environment

| Item | Version used |
|---|---|
| Python | 3.13.15 |
| NumPy | 2.5.3 |
| Matplotlib | 3.11.2 |
| Pygame | not installed yet — required for the planned real-time viewer |
| Git | 2.55.0 |
| OS tested on | Windows (x64) |

`requirements.txt` lists the dependencies; `pygame` is included in advance for the viewer milestone.

## Usage

```bash
git clone git@github.com:ASylviaofmine/robot-nav-sim.git
cd robot-nav-sim

python -m venv .venv
.venv\Scripts\activate            # Windows PowerShell/CMD
# source .venv/bin/activate       # Linux / macOS

pip install -r requirements.txt
```

Planner demo — draws a 10×10 map with a wall and the path found by A\* (Matplotlib window):

```bash
python practice/day10_basics.py
# 路径长度 = 17 格，共走 16 步
```

Reproduce the measurements in this README:

```bash
python tools/benchmark.py     # planning: path length, nodes, runtime, success rate, BFS check
python tools/pid_tuning.py    # control: PID metrics table
```

Both scripts are deterministic (fixed seed 42 for the random maps) and print Markdown tables, so the numbers above can be regenerated and compared.

## Measured results

Summary of what is actually verified today:

- **Planner correctness:** on all five hand-built maps the A* path length equals an independent BFS optimum (4 solvable maps, 4/4 match); a blocked goal returns `None` instead of hanging.
- **Path well-formedness:** 4/4 hand-built paths and 17/17 random-map paths are collision-free, start at the start, end at the goal, never leave the grid, never revisit a cell, and use only 4-connected moves (no diagonal jumps).
- **Path quality:** the planner takes the optimal number of steps on every solvable case tested (14 steps on an empty 10×10 grid, 16 with a wall in the way — the Manhattan lower bound of 14 plus the detour around the wall); 17/17 random maps match the BFS optimum.
- **Scaling:** on 20×20 grids with 25 % random obstacles it expands a median of 186 / 400 cells (46 %), median runtime 1.3 ms.
- **Success rate:** 17 / 20 random 20×20 maps solved (85 %); all three failures were confirmed genuinely unreachable by BFS, i.e. zero failures caused by the planner.
- **Heuristic choice:** measured head-to-head on identical maps — Dijkstra 250 nodes vs A* 130 vs Euclidean-heuristic A* 161 (20×20 random), i.e. the Manhattan choice is justified by data, not by convention.
- **Control:** PID tuning metrics as tabulated above; the residual-error / overshoot story is reproduced numerically, not just qualitatively.

Not yet measured (and therefore not claimed): time-to-goal of the full plan-and-drive loop, path smoothness after the planned shortcutting step, and any real-time rendering performance.

## Roadmap

Done:

- [x] Project setup: Python + Git + GitHub, repo layout, licence, README (Day 1)
- [x] Data handling and map representation: lists/dicts, NumPy grids, Matplotlib heatmap of the map (Days 5–6)
- [x] Map as an object (`Grid`): free / wall cells, bounds checking (Day 7)
- [x] PID controller + DC-motor position simulation, with a tuning comparison (Day 8)
- [x] 2D heading control on a differential-drive model (go-to-goal) (Day 9)
- [x] A* grid planner with path reconstruction, plus a reproducible benchmark and BFS cross-check (Day 10)
- [x] Heuristic comparison (Manhattan vs Euclidean vs none) and heuristic-weight sweep, measured on identical maps
- [x] English README with problem statement, algorithm rationale, complexity, environment, usage and measured results

Open:

- [ ] Real-time visualization of map, start, goal and planned path (Pygame window; the per-part visualization today is static Matplotlib)
- [ ] Planner refactor into `src/` with a stable interface, so planning and control share one map model
- [ ] End-to-end demo: `python main.py` plans a route and drives the robot along it
- [ ] RRT / RRT* comparison on identical maps, same benchmark table
- [ ] Path smoothing / shortcutting for smoother tracking, with the effect measured
- [ ] Track a *differential-drive* robot along the path (today the planner is grid-discrete and the controller is straight-line go-to-goal)
- [ ] Automated tests instead of print-based checks
- [ ] Animated GIF / short video of a run, for the README
- [ ] A short technical report: assumptions, parameter choices, limitations

## Limitations

- **The pipeline is not yet closed.** Planning and control are implemented and measured separately; no single run yet plans a route and drives the robot along it. This is the biggest gap.
- **Grid-discrete planning.** A* on a uniform 4-connected grid yields paths with 90° corners and 1-cell clearance; it ignores robot footprint, so the trajectories are not yet physically smooth.
- **Straight-line convergence only.** The current controller assumes it can turn towards the goal and drive; there is no obstacle avoidance during tracking, so an unmodelled obstacle in the path is not handled.
- **State estimation not modelled.** Perfect pose is assumed (no noise, no drift, no sensor model).
- **Planner comparison is one-sided.** The heuristic comparison is measured; the RRT comparison is still a plan.
- **Scale.** Benchmarks are 10×10 and 20×20 grids with uniform step costs; weighted terrain, larger maps and diagonal moves are untested.
- **Reproducibility caveat.** Timings are from a single Windows machine; node counts and path lengths are deterministic, runtimes are not.

## Next steps

1. Consolidate the algorithms into a small `src/` package with tests.
2. Close the loop: feed the A* path to the go-to-goal controller, waypoint by waypoint, and record path length, tracking error, and time to goal.
3. Add the Pygame viewer for map, obstacles, path, robot pose and travelled trail; export an animated GIF.
4. Add RRT to the same benchmark and report the comparison honestly.
5. Simulate the robot as a real differential drive (wheel speeds, non-holonomic constraints) and re-tune the controller.

## License

MIT — see [LICENSE](LICENSE).
