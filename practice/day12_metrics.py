"""D12：给昨天的路径跟踪量化误差 —— 一把尺子，三个旋钮。

昨天我们只是"看着图觉得还行"。今天要做的是：
    把"跟踪得好不好"变成一个能算出来的数，然后看每个参数把它推向哪边。

三个概念（今天的新东西）：
  1. 横向偏差 cross-track error：车偏离规划折线多少米（不是离最近路点多远！）
  2. 点到线段的距离：两路点之间的那段直线，才是车真正要贴着的"轨道"
  3. 参数敏感性扫描：固定其它旋钮，只拧一个，把结果列成表 / 画成曲线

目标（跑通后应该打印，数值我实测过）：
    tol=0.05 基线：用时 32.00 秒，终点误差 0.039，最大横向 0.0895

上面 Grid / astar / cell_to_world 沿用 day10、day11 的成果，不用重写。
今天你写的是：dist_to_segment / crosstrack_error / sweep 三处。
"""

import numpy as np
import matplotlib.pyplot as plt
from heapq import heappush, heappop


# ===== 以下沿用 day10_basics.py 与 day11_tracking.py，不用改 =====

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
    """栅格坐标 (行, 列) → 世界坐标 (x, y)，取格子正中心。"""
    r, c = cell
    return (c + 0.5, r + 0.5)


# ===== 今天要写的：误差度量 =====

def dist_point_to_segment(p, a, b):
    """点 p 到线段 ab 的最短距离。

    点 p、线段两端 a、b 都是 numpy 数组，形如 array([x, y])。

    想一想：如果 p 在 ab 的"正上方"（垂足落在线段中间），距离就是垂直距离；
    但如果 p 在线段端点外面，最近的点就是端点本身，不是垂足。
    所以要先算"垂足在线段上的比例 t"，把它夹在 [0, 1] 之间：
        t = clip( ((p - a) · (b - a)) / |b - a|², 0, 1 )
        proj = a + t * (b - a)          ← 线段上离 p 最近的那个点
        return |p - proj|

    提示：numpy 里 a @ b 就是点积；np.clip(x, 0.0, 1.0) 用于夹取。
    """
    ab = b - a
    L2 = ab @ ab                     # |b - a|²，线段的长度平方
    if L2 < 1e-12:                   # 退化成一个点
        return float(np.hypot(*(p - a)))
    t = np.clip( (p - a) @ ab / L2, 0.0 , 1.0 )        # 空 1：垂足在线段上的比例，记得夹进 [0, 1]
    proj = a + t * ab                # 线段上最近的点
    return float(np.hypot(*(p - proj)))


def crosstrack_error(traj, waypoints):
    """整条轨迹的横向偏差：每个轨迹点到【规划折线】的最短距离。

    traj：形如 [(x0,y0), (x1,y1), ...] 的轨迹点列表
    waypoints：路点列表，相邻两点连成的线段就是"规划折线"

    注意：这里量的不是"到最近路点的距离"——路点每 1 米才有一个，
    车走在两路点正中间时离最近路点有 0.5 米，那是个假误差。
    真正该量的是"到那条折线的距离"。
    """
    W = np.array(waypoints)
    A, B = W[:-1], W[1:]             # 把折线拆成一段段：A[i]→B[i]

    errs = []
    for p in traj:
        p = np.array(p)
        d = min(dist_point_to_segment(p, A[i], B[i]) for i in range(len(A)))        #空2
        #括号里填什么呢？
        # 想一想：A[i] 和 B[i] 是第 i 段的两端，p 是当前轨迹点。
        # 你要调用的那个函数就在上面 30 行处，名字是 dist_point_to_segment，它收三个参数——点在前面，线段两端在后面。
        #按这个顺序写出来，参数用 p, A[i], B[i]。
        errs.append(d)
    return np.array(errs)


# ===== 今天的第二个东西：仿真 + 参数扫描 =====

def follow(waypoints, tol=0.05, speed=0.5, Kp=5.0, dt=0.05, t_max=200.0):
    """昨天写过的跟踪循环，封装成函数，方便反复调用。"""
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
    """跑一次仿真，把它压成 4 个数字：用时 / 行驶距离 / 终点误差 / 横向偏差。"""
    pts, t = follow(waypoints, tol=tol, speed=speed, Kp=Kp)
    pts = np.array(pts)
    dist = float(np.sum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))))
    end_err = float(np.hypot(pts[-1, 0] - waypoints[-1][0],
                             pts[-1, 1] - waypoints[-1][1]))
    ct = crosstrack_error(pts, waypoints)
    return {"time": t, "dist": dist, "end_err": end_err,
            "max_ct": ct.max(), "mean_ct": ct.mean()}


def sweep(waypoints, param, values):
    """固定其它旋钮，只拧一个：param 取 'tol' / 'speed' / 'Kp'。

    返回一个列表，每项是 (value, 指标字典)。
    """
    results = []
    for v in values:
        m = evaluate(waypoints, **{param: v})       #空3：
        results.append((v, m))
    return results


# ===== 主程序 =====

g = Grid(10)
for c in range(1, 9):
    g.set_wall(5, c)

path = astar(g, (1, 1), (8, 8))
waypoints = [cell_to_world(cell) for cell in path]

print("=" * 72)
print("基线（tol=0.05, speed=0.5, Kp=5.0）")
print("=" * 72)
base = evaluate(waypoints)
print(f"用时 {base['time']:.2f} 秒，行驶距离 {base['dist']:.2f}，"
      f"终点误差 {base['end_err']:.3f}")
print(f"横向偏差：最大 {base['max_ct']:.4f} m，平均 {base['mean_ct']:.4f} m")
print()

# 三组扫描
SCANS = [
    ("tol",   [0.02, 0.05, 0.10, 0.15, 0.20, 0.30], "离路点多近算追上"),
    ("speed", [0.30, 0.50, 0.80, 1.00, 1.50],       "车速 m/s"),
    ("Kp",    [2.0, 3.0, 5.0, 8.0, 12.0],           "转向增益"),
]

for param, values, label in SCANS:
    print("=" * 72)
    print(f"扫描 {param}（{label}）")
    print("=" * 72)
    print(f"{param:>6} {'用时s':>8} {'距离m':>8} {'终点误差':>10} "
          f"{'最大横向':>10} {'平均横向':>10}")
    rows = sweep(waypoints, param, values)
    for v, m in rows:
        print(f"{v:6.2f} {m['time']:8.2f} {m['dist']:8.2f} "
              f"{m['end_err']:10.3f} {m['max_ct']:10.4f} {m['mean_ct']:10.4f}")
    print()

# 画一张权衡图：三个旋钮，每个一眼看出"拧紧它会牺牲什么"
fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
for ax, (param, values, label) in zip(axes, SCANS):
    rows = sweep(waypoints, param, values)
    xs = [v for v, _ in rows]
    end_errs = [m["end_err"] for _, m in rows]
    max_cts = [m["max_ct"] for _, m in rows]
    times = [m["time"] for _, m in rows]

    ax.plot(xs, end_errs, "o-", color="#E24B4A", label="end-point error")
    ax.plot(xs, max_cts, "s-", color="#378ADD", label="max cross-track")
    ax2 = ax.twinx()
    ax2.plot(xs, times, "^--", color="#888780", alpha=0.7, label="run time")
    ax2.set_ylabel("run time (s)", color="#888780")
    ax2.tick_params(axis="y", labelcolor="#888780")

    ax.set_xlabel(param)
    ax.set_ylabel("error (m)")
    ax.set_title(f"sensitivity to {param}")
    ax.grid(True, alpha=0.3)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc="best")

plt.tight_layout()
plt.suptitle("Tracking error vs controller parameters", y=1.02)
plt.show()
