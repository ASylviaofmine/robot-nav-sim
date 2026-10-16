"""D13：让小车在 pygame 窗口里真的动起来。

复用思路：上面 5 块（Grid / heuristic / astar / cell_to_world / follow）
全部从 practice/day11、day12 复制过来 —— 它们已验收通过，今天不碰。
今天的新东西只有一件事：**把这些算好的数据画到屏幕上，一帧一帧放**。

做法是"先算完，再回放"：
    1. 先用 follow() 把整条轨迹一次算完（641 个采样点）
    2. 再开一个窗口，第 k 帧只画轨迹的前 k 个点
       —— 轨迹越画越长，小车看起来就是在"开"。

验收标准（跑通后你应该看到）：
    - 弹出一个约 640x640 的窗口：深色横墙、蓝色 A* 路径、
      绿色小车沿路径开，身后拖出一条红色轨迹
    - 约 11 秒播完，底部进度条变满
    - 按键盘 R 重播；点窗口右上角 X 正常关闭（不卡死）

今天的 4 个空：搜「空 1」到「空 4」。
"""

import os

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"     # 关掉 pygame 的问候语

import numpy as np
import pygame
from heapq import heappush, heappop


# ===== 以下 5 块从 day11/day12 复制，已验收，不用改 =====

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
    r, c = cell
    return (c + 0.5, r + 0.5)


def follow(waypoints, tol=0.05, speed=0.5, Kp=5.0, dt=0.05, t_max=200.0):
    """昨天的跟踪循环，只加了一处：把每一步的车头角度 theta 也记下来，
    因为今天要在屏幕上把车头朝向画出来。"""
    x, y, theta = waypoints[0][0], waypoints[0][1], 0.0
    idx, t = 1, 0.0
    pts, thetas = [(x, y)], [theta]
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
        thetas.append(theta)
        if np.hypot(tx - x, ty - y) < tol:
            idx = idx + 1
    return pts, thetas, t


# ===== 今天的新东西：pygame =====

SIZE = 10           # 地图 10x10 格
CELL = 60           # 每个格子画成 60 像素见方
MARGIN = 26         # 窗口四周留白
WIDTH = MARGIN * 2 + SIZE * CELL
HEIGHT = MARGIN * 2 + SIZE * CELL
FPS = 60            # 每秒画 60 帧 —— 641 个点 / 60 帧 ≈ 11 秒播完

# 颜色用 (红, 绿, 蓝) 三元组，每个 0~255
BG = (248, 248, 245)          # 背景：近白
WALL = (48, 54, 66)           # 墙：深灰蓝
LINE = (208, 208, 202)        # 空地描边
PATH_C = (96, 140, 220)       # A* 规划路径：蓝
TRAIL = (226, 84, 72)         # 小车走过的轨迹：红
ROBOT = (58, 168, 96)         # 小车：绿
ROBOT_EDGE = (255, 255, 255)  # 小车描边 / 朝向线：白
START_C = (150, 150, 145)     # 起点：灰
GOAL_C = (240, 186, 60)       # 终点：金


def world_to_screen(x, y):
    """世界坐标 (x, y)，单位是"格" → 屏幕像素 (px, py)。

    这是今天第一个坑。两套坐标系的方向不一样：
        世界（数学）：x 向右，y 向上（y 越大越"北"）
        屏幕（像素）：px 向右，py 向下（py 越大越"靠下"）
    x 方向两者一致，直接乘 CELL 缩放；y 方向相反，必须翻过来。

    参照写法（px 那行已经给你写好了）：
        px = MARGIN + x * CELL
    py 应该是：先把 y 翻转（SIZE - y），再乘 CELL 加 MARGIN。
    """
    px = MARGIN + x * CELL
    py = MARGIN + (SIZE-y)*CELL               # 空 1：把 y 翻转后缩放
    return (int(round(px)), int(round(py)))


def build_world():
    """搭地图、跑 A*、算完整条轨迹 —— 全是前几天的东西。"""
    g = Grid(SIZE)
    for c in range(1, 9):
        g.set_wall(5, c)
    start, goal = (1, 1), (8, 8)
    path = astar(g, start, goal)
    waypoints = [cell_to_world(cell) for cell in path]
    traj, thetas, t = follow(waypoints)
    print(f"[world] 路径 {len(path)} 格 / {len(path) - 1} 步，"
          f"轨迹 {len(traj)} 个采样点，用时 {t:.2f} s")
    return g, start, goal, path, traj, thetas


def draw(screen, grid, path, start, goal, traj, thetas, k):
    """把第 k 帧的世界画到屏幕上。pygame 的画图函数都是"往 screen 上画"。"""
    screen.fill(BG)

    # ---- 1. 格子：墙涂实心，空地只描边 ----
    for r in range(grid.size):
        for c in range(grid.size):
            x0, y0 = world_to_screen(c, r + 1)          # 格子左上角
            rect = pygame.Rect(x0, y0, CELL, CELL)
            if grid.cells[r, c] == 1:
                pygame.draw.rect(screen, WALL, rect)    # 墙：实心
            else:
                pygame.draw.rect(screen, LINE, rect, 1) # 空地：只画 1 像素边框

    # ---- 2. A* 规划路径：整条画出来当参考线（蓝色细线）----
    pts = [world_to_screen(*cell_to_world(cell)) for cell in path]
    if len(pts) > 1:
        pygame.draw.lines(screen, PATH_C, False, pts, 2)

    # ---- 3. 小车走过的轨迹：只画前 k 个点，所以每帧都"长"一点 ----
    # 列表推导式 + 切片：把轨迹的前 k 个点挨个转成屏幕坐标
    if k > 1:
        walked = [world_to_screen(px, py) for px, py in traj[:k] ]   # 空 2：用切片取出轨迹的前 k 个点
        pygame.draw.lines(screen, TRAIL, False, walked, 3)

    # ---- 4. 起点 / 终点标记 ----
    pygame.draw.circle(screen, START_C, world_to_screen(*cell_to_world(start)), 7)
    pygame.draw.circle(screen, GOAL_C, world_to_screen(*cell_to_world(goal)), 11)
    pygame.draw.circle(screen, WALL, world_to_screen(*cell_to_world(goal)), 11, 2)

    # ---- 5. 小车：绿圆 + 一条白色短线表示车头朝向 ----
    x, y = traj[k - 1]
    theta = thetas[k - 1]
    cx, cy = world_to_screen(x, y)
    # 车头尖端：从车心沿朝向方向伸出 0.3 格
    tip = world_to_screen(x + 0.30 * np.cos(theta), y + 0.30 * np.sin(theta))
    pygame.draw.line(screen, ROBOT_EDGE, (cx, cy), tip, 3)
    pygame.draw.circle(screen, ROBOT, (cx, cy), 9)
    pygame.draw.circle(screen, ROBOT_EDGE, (cx, cy), 9, 2)

    # ---- 6. 底部进度条 ----
    n = len(traj)
    bar_w = WIDTH - 2 * MARGIN
    bar_y = HEIGHT - MARGIN // 2 - 3
    pygame.draw.rect(screen, LINE, (MARGIN, bar_y, bar_w, 6))
    pygame.draw.rect(screen, ROBOT, (MARGIN, bar_y, int(bar_w * k / n), 6))


def main():
    # pygame 三件套：初始化、开窗口、建时钟
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("D13 - A* path + go-to-goal tracking")
    clock = pygame.time.Clock()

    g, start, goal, path, traj, thetas = build_world()

    k = 1              # 动画进度：当前画到轨迹的第几个点
    running = True
    while running:
        # (a) 收事件：不写这段窗口会"未响应"，点 X 都关不掉
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                k = 1                          # 按 R 键重播

        # (b) 更新状态：让动画前进一个采样点
        if k < len(traj):
            k = k+1                                             # 空 3：每帧前进一步

        # (c) 重绘：把新的画面整个画一遍，再"翻"到屏幕上
        draw(screen, g, path, start, goal, traj, thetas, k)
        pygame.display.flip()

        # (d) 限速：不加这行，循环全速狂奔，动画一瞬间就播完了
   #     clock.tick(FPS)                                          # 空 4：让循环最多每秒跑 FPS 次

    pygame.quit()
    print("[anim] 窗口已关闭")


if __name__ == "__main__":
    main()
