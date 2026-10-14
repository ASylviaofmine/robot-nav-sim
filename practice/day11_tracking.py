"""D11：把 A* 规划出来的路径，喂给 go-to-goal 控制器，让小车真的沿路径开过去。

上面四个东西（Grid / heuristic / astar / plot_map）是从 practice/day10_basics.py
复制过来的 —— 它们昨天已经验收通过，今天不用重写。
下面 cell_to_world / main 里的跟踪循环，才是今天要自己写的新东西。

目标（跑通后应该打印）：
    用时 32.00 秒，到达 (8.50, 8.46)
    路点数 17，抵达 17
    行驶距离 16.00，路径最短 16.00
    终点误差 0.039
"""

import numpy as np
import matplotlib.pyplot as plt
from heapq import heappush, heappop


# ===== 以下 4 块直接复制 practice/day10_basics.py，不用改 =====

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


# ===== 今天是新的 =====

def cell_to_world(cell):
    """栅格坐标 (行, 列) → 世界坐标 (x, y)，取格子正中心。

    例：(1, 1) → (1.5, 1.5)      (5, 0) → (0.5, 5.5)      (8, 8) → (8.5, 8.5)
    提示：行 r 管 y，列 c 管 x，写出来的顺序和看上去的相反。
    """
    r, c = cell
    return (c+0.5, r+0.5)                     # 空 1


# ===== 主程序 =====

g = Grid(10)
for c in range(1, 9):
    g.set_wall(5, c)                            # 和 D10 同一张地图：一堵横墙

start = (1, 1)
goal = (8, 8)
path = astar(g, start, goal)
waypoints = [cell_to_world(cell) for cell in path]

print(f"路点数 {len(waypoints)}")
print(f"前 4 个路点 {waypoints[:4]}")

Kp = 5.0
speed = 0.5
dt = 0.05
tol = 0.05            # 离路点多近算"追上了"
t_max = 200.0         # 安全阀：最多模拟这么久，避免写错时死循环

x, y, theta = waypoints[0][0], waypoints[0][1], 0.0    # 从起点格子的中心出发
idx = 1               # 正在追第几个路点（第 0 个是出发点，跳过）
xs, ys = [x], [y]
t = 0.0

while idx < len(waypoints) and t < t_max:      # 两个刹车条件：追完所有路点 / 超时
    tx, ty = waypoints[idx]                            # 空 2：当前要追的路点坐标（x 和 y 两个数）

    theta_target = np.arctan2(ty - y, tx - x)  # 车头应该朝的方向
    e = theta_target - theta                   # 朝向误差（弧度）
    e =  (e + np.pi) % (2 * np.pi) - np.pi                                 # 空 3：把 e 收进 (-π, π]
    theta = theta + Kp * e * dt                # P 控制：先转车头

    x = x + speed * np.cos(theta) * dt         # 再沿车头方向前进
    y = y + speed * np.sin(theta) * dt
    t = t + dt

    xs.append(x)
    ys.append(y)

    if np.hypot(tx - x, ty - y) < tol:         # 追上当前路点了吗？
       idx = idx + 1                                 # 空 4：下一步该追哪个点？

# ===== 打印验收数字 =====

dist = 0.0
for i in range(1, len(xs)):
    dist = dist + np.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])

print(f"用时 {t:.2f} 秒，到达 ({x:.2f}, {y:.2f})")
print(f"路点数 {len(waypoints)}，抵达 {idx}")
print(f"行驶距离 {dist:.2f}，路径最短 {len(waypoints) - 1:.2f}")
print(f"终点误差 {np.hypot(x - waypoints[-1][0], y - waypoints[-1][1]):.3f}")

# ===== 出图：地图 + A* 路径 + 小车实际轨迹 =====

img = g.cells.copy()
for (r, c) in path:
    img[r, c] = 0.5                             # 规划路径：灰色格子
img[start[0], start[1]] = 2
img[goal[0], goal[1]] = 3

plt.imshow(img, cmap="gray_r", interpolation="nearest")
plt.plot(xs, ys, "r-", linewidth=2, label="robot trajectory")
plt.plot(xs[0], ys[0], "go", markersize=8, label="start")
plt.plot(waypoints[-1][0], waypoints[-1][1], "r*", markersize=15, label="goal")
plt.title("A* path vs robot trajectory")
plt.legend()
plt.grid(True, alpha=0.3)
plt.show()
