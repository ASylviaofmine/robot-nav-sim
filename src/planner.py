"""A* 网格路径规划（4 连通、每步代价 1、曼哈顿启发）。

f(n) = g(n) + h(n)
    g  从起点走到 n 已经花掉的真实步数
    h  从 n 到终点还剩多少步的"估计"（曼哈顿距离）
    只要 h 永远不高估，A* 找到的就是最短路径。

为什么曼哈顿而不是欧氏距离：机器人只能上下左右走，走一步就是 1，
走斜线不存在。曼哈顿距离正好等于"在不撞墙的最好情况下还要几步"，
是四连通网格上最紧的可采纳启发。
"""

from heapq import heappush, heappop


def heuristic(a, b):
    """曼哈顿距离 |Δr| + |Δc|。"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal):
    """在 grid 上找 start → goal 的最短格子路径。

    返回 [(r, c), ...]，包含起点和终点；不可达返回 None。
    调用方必须处理 None —— 封死的终点不会抛异常，也不会死循环。
    """
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
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nb = (r + dr, c + dc)
            if grid.is_wall(nb[0], nb[1]):
                continue
            ng = g[cur] + 1
            if nb not in g or ng < g[nb]:
                g[nb] = ng
                came_from[nb] = cur
                heappush(open_list, (ng + heuristic(nb, goal), nb))

    return None


def path_steps(path):
    """路径走多少步（格子数 = 步数 + 1）。"""
    return len(path) - 1
