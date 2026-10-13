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
            return True                      # 走出地图外面 = 当成墙
        return self.cells[row, col] == 1


def heuristic(a, b):
    """估计 a 到 b 还要几步：横竖距离相加（假装路上没有墙）"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal):
    open_list = []
    heappush(open_list, (0, start))          # (f, 格子) 塞进待考察队列

    came_from = {}                           # 记录"我是从哪来的"
    g = {start: 0}                           # 每个格子"已经走了几步"
    visited = set()                          # 已经考察过的格子

    while open_list:
        f, cur = heappop(open_list)          # 取出 f 最小的那个格子

        if cur ==goal:               #line 64：goal = (2, 7)，goal = (8，8) # 空 1：什么时候算"找到路了"？
            path = [cur]
            while cur in came_from:          # 一路往回问"你从哪来"，问回起点
                cur = came_from[cur]
                path.append(cur)
            return path[::-1]                # 倒过来 = 从起点到终点的顺序

        if cur in visited:
            continue
        visited.add(cur)

        r, c = cur
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:   # 上下左右四个邻居
            nb = (r + dr, c + dc)
            if grid.is_wall(nb[0], nb[1]):
                continue                     # 是墙就跳过
            ng = g[cur] + 1             # 空 2：走一格，多走了几步？
            if nb not in g or ng < g[nb]:
                g[nb] = ng
                came_from[nb] = cur
                heappush(open_list, (ng + heuristic(nb, goal), nb))   # 空 3


g = Grid(10)
for c in range(1, 9):
    g.set_wall(5, c)                         # 一堵横墙，只留第 0 列和第 9 列的口

start = (1, 1)
goal = (8, 8) #goal = (2, 7)
path = astar(g, start, goal)
print(f"路径长度 = {len(path)} 格，共走 {len(path) - 1} 步")

img = g.cells.copy()
for (r, c) in path:
    img[r, c] = 0.5                          # 路径涂成灰色
img[start[0], start[1]] = 2
img[goal[0], goal[1]] = 3

plt.imshow(img, cmap="gray_r", interpolation="nearest")
plt.title("A* path")
plt.show()
