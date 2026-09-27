# robot-nav-sim

**A 2D mobile-robot autonomous navigation simulator — PID control, A*/RRT path planning, and real-time visualization.**

Status: 🚧 **In development** (20-day build sprint, started 2026-09-27)

---

## What is this?

A lightweight 2D simulation in which a differential-drive mobile robot must travel from a start point to a goal point across a map filled with obstacles. Three subsystems cooperate:

1. **Path planning** — the robot computes a collision-free route using **A\*** on a grid map (with **RRT** as a sampling-based comparison), then smooths the path.
2. **Motion control** — a discrete **PID controller** regulates motor speed so the robot accurately tracks the planned path.
3. **Visualization** — the map, obstacles, planned path, robot pose, and traveled trail render in real time with **pygame**.

The end goal is a one-command demo: run `python main.py`, and watch the robot plan and drive.

## Why this project?

- To turn control-theory coursework (PID, step response, steady-state error) into **working code**, not just transfer functions on paper.
- To practice the core software stack of robotics — planning, kinematics, control, simulation — with every algorithm **implemented from scratch** (no robotics framework shortcuts), so each piece is understood end-to-end.
- To keep the whole system small and readable: a few hundred lines per module, clean interfaces between them.

## Modules

| Module | Status | Description |
|---|---|---|
| `src/pid.py` | ⬜ | Discrete PID controller + DC-motor step-response simulation; Kp/Ki/Kd tuning experiments |
| `src/planners/astar.py` | ⬜ | Grid-based A*; Manhattan vs. Euclidean heuristics comparison; path smoothing |
| `src/planners/rrt.py` | ⬜ | Sampling-based RRT planner, benchmarked against A* on identical maps |
| `src/robot.py` | ⬜ | Differential-drive kinematics + PID path tracking |
| `src/world.py` | ⬜ | Obstacle map generation and collision checking |
| `main.py` | ⬜ | End-to-end demo: plan → track → render |

## Getting started

```bash
git clone https://github.com/ASylviaofmine/robot-nav-sim.git
cd robot-nav-sim
python -m venv .venv
.venv\Scripts\activate        # Windows  (Linux/macOS: source .venv/bin/activate)
pip install -r requirements.txt
python main.py
```

Requires Python 3.10+. Dependencies: NumPy, Matplotlib, Pygame.

## Roadmap

- [x] Project init: environment, Git/GitHub, repo layout (Day 1)
- [ ] PID speed-control simulation with tuning experiments (Days 8–9)
- [ ] A* planner + visualization (Days 10–12)
- [ ] RRT comparison planner (Day 13, stretch goal)
- [ ] Full navigation demo + animated GIF (Day 14)
- [ ] Project report and polished documentation (Day 17)

## License

MIT — see [LICENSE](LICENSE).
