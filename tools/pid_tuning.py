"""PID tuning experiment for the 1D position controller.

Reuses ``simulate()`` from ``practice/day08_functions_practice.py`` (the daily
learning file) and turns four Kp/Ki/Kd choices into measurable metrics, so the
tuning claims in the README are backed by reproducible numbers instead of
by eyeballing the plot.

Plant model: force -> acceleration (u - friction) -> velocity -> position,
integrated with Euler steps of dt = 0.05 s over 20 s (400 steps).

Usage:
    python tools/pid_tuning.py
"""

from __future__ import annotations

import contextlib
import io
import os
import sys

import matplotlib

matplotlib.use("Agg")

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "practice"))

import matplotlib.pyplot as plt  # noqa: E402

plt.show = lambda *args, **kwargs: None

with contextlib.redirect_stdout(io.StringIO()):
    from day08_functions_practice import simulate  # noqa: E402

TARGET = 10.0
DT = 0.05

CASES = [
    ("P only", dict(Kp=1, Ki=0, Kd=0, friction=0.0)),
    ("PD", dict(Kp=1, Ki=0, Kd=2, friction=0.0)),
    ("PD + friction", dict(Kp=1, Ki=0, Kd=2, friction=0.5)),
    ("PID + friction", dict(Kp=1, Ki=0.3, Kd=2, friction=0.5)),
]


def metrics(xs, target=TARGET, dt=DT):
    final = xs[-1]
    overshoot = (max(xs) - target) / target * 100 if max(xs) > target else 0.0

    rise = next((i for i, x in enumerate(xs) if x >= 0.9 * target), None)

    band = 0.02 * target
    outside = [i for i, x in enumerate(xs) if abs(x - target) > band]
    settle = (outside[-1] + 1) * dt if outside else 0.0

    return {
        "final": final,
        "overshoot": overshoot,
        "rise": None if rise is None else rise * dt,
        "settle": settle,
        "error": target - final,
    }


def main():
    print(f"1D position control, target = {TARGET:g} m, dt = {DT:g} s, "
          "horizon = 20 s (400 steps)\n")
    print("| Controller | Kp / Ki / Kd | Final position (m) | Steady-state "
          "error (m) | Overshoot | Rise time, 90 % (s) | Settling time, ±2 % (s) |")
    print("|---|---|---|---|---|---|---|")
    for name, kw in CASES:
        xs = simulate(target=TARGET, T=20.0, **kw)
        m = metrics(xs)
        rise = "—" if m["rise"] is None else f"{m['rise']:.2f}"
        settle = ("never" if abs(m["settle"] - 20.0) < 1e-9
                  else f"{m['settle']:.2f}")
        print(f"| {name} | {kw['Kp']} / {kw['Ki']} / {kw['Kd']} | "
              f"{m['final']:.3f} | {m['error']:+.3f} | {m['overshoot']:.1f} % | "
              f"{rise} | {settle} |")
    print("\nNotes")
    print("- `P only` on this plant (force -> velocity -> position) is an "
          "undamped oscillator: it keeps swinging around the target instead of "
          "converging, so the reported 'final position' is just wherever the "
          "oscillation happened to be at t = 20 s.")
    print("- the D term supplies the damping, so the same Kp now converges "
          "(PD, 0 % overshoot).")
    print("- friction = 0.5 N is a constant disturbance: it leaves a 0.5 m "
          "steady-state error that P and D cannot remove.")
    print("- the I term integrates that residual error away (+0.5 m -> +0.012 m), "
          "at the cost of some overshoot (29.7 %).")


if __name__ == "__main__":
    main()
