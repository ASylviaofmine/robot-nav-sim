"""把地图、规划、控制串成"一次完整运行"。

调用方只需要记一件事：

    world = build_world()
    world["traj"]   # 小车实际走过的轨迹

返回的 world 是一个字典，viewer / 报告 / 基准测试都从它取数据 ——
这样各模块之间传的是"一次运行的全部结果"，而不是一堆散参数。
"""

from src.grid import Grid, cell_to_world
from src.planner import astar
from src.controller import follow

DEFAULT_SIZE = 10
DEFAULT_START = (1, 1)
DEFAULT_GOAL = (8, 8)


def make_map(size=DEFAULT_SIZE, wall_row=None):
    """建一张经典测试地图：中间一道横墙，两端各留一个口，逼着规划器绕路。

    10×10 时：第 5 行 1..8 列是墙，(5,0) 和 (5,9) 是仅有的两个通道口。
    """
    grid = Grid(size)
    row = size // 2 if wall_row is None else wall_row
    for c in range(1, size - 1):
        grid.set_wall(row, c)
    return grid


def build_world(size=DEFAULT_SIZE, start=DEFAULT_START, goal=DEFAULT_GOAL, **ctrl):
    """建图 → A* 规划 → 路点跟踪，返回这一次运行的全部数据。

    ctrl 里的关键字参数会原样传给 controller.follow()，
    所以 build_world(tol=0.2, Kp=8) 可以直接做参数实验。
    """
    grid = make_map(size)
    path = astar(grid, start, goal)
    if path is None:
        raise ValueError(f"goal {goal} is unreachable from {start} on this map")

    waypoints = [cell_to_world(cell) for cell in path]
    traj, thetas, t = follow(waypoints, **ctrl)

    return {
        "grid": grid,
        "start": start,
        "goal": goal,
        "path": path,
        "waypoints": waypoints,
        "traj": traj,
        "thetas": thetas,
        "time": t,
    }


def describe(world):
    """把一次运行压成一行摘要，方便在终端里核对。"""
    return (f"path {len(world['path'])} cells / {len(world['path']) - 1} steps, "
            f"trajectory {len(world['traj'])} samples, "
            f"time {world['time']:.2f} s")
