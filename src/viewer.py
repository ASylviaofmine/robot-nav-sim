"""可视化：实时窗口播放 + 离屏录制 GIF。

两个公开函数：
    play(world)        开一个 pygame 窗口，一帧一帧播放（按 R 重播）
    record_gif(world)  不开窗口，把画面画到内存画布上，抽帧存 PNG，再合成 GIF

坐标换算在这里再做一次 y 翻转：世界的 y 朝上，屏幕的 py 朝下。
"""

import math
import os
import tempfile

import pygame

from src.grid import cell_to_world

CELL = 60           # 每格 60 像素
MARGIN = 26         # 四周留白
FPS = 60            # 窗口播放帧率

BG = (248, 248, 245)
WALL = (48, 54, 66)
LINE = (208, 208, 202)
PATH_C = (96, 140, 220)
TRAIL = (226, 84, 72)
ROBOT = (58, 168, 96)
ROBOT_EDGE = (255, 255, 255)
START_C = (150, 150, 145)
GOAL_C = (240, 186, 60)


def canvas_size(grid):
    side = MARGIN * 2 + grid.size * CELL
    return (side, side)


def world_to_screen(x, y, grid):
    """世界坐标（单位：格）→ 屏幕像素。y 要翻转：世界朝上，屏幕朝下。"""
    px = MARGIN + x * CELL
    py = MARGIN + (grid.size - y) * CELL
    return (int(round(px)), int(round(py)))


def draw(surface, world, k):
    """把"走到第 k 个采样点"的画面画到 surface 上。"""
    grid = world["grid"]
    surface.fill(BG)

    for r in range(grid.size):
        for c in range(grid.size):
            x0, y0 = world_to_screen(c, r + 1, grid)
            rect = pygame.Rect(x0, y0, CELL, CELL)
            if grid.is_wall(r, c):
                pygame.draw.rect(surface, WALL, rect)
            else:
                pygame.draw.rect(surface, LINE, rect, 1)

    pts = [world_to_screen(*cell_to_world(cell), grid) for cell in world["path"]]
    if len(pts) > 1:
        pygame.draw.lines(surface, PATH_C, False, pts, 2)

    if k > 1:
        walked = [world_to_screen(px, py, grid) for px, py in world["traj"][:k]]
        pygame.draw.lines(surface, TRAIL, False, walked, 3)

    pygame.draw.circle(surface, START_C,
                       world_to_screen(*cell_to_world(world["start"]), grid), 7)
    goal_px = world_to_screen(*cell_to_world(world["goal"]), grid)
    pygame.draw.circle(surface, GOAL_C, goal_px, 11)
    pygame.draw.circle(surface, WALL, goal_px, 11, 2)

    x, y = world["traj"][k - 1]
    theta = world["thetas"][k - 1]
    cx, cy = world_to_screen(x, y, grid)
    tip = world_to_screen(x + 0.30 * math.cos(theta),
                          y + 0.30 * math.sin(theta), grid)
    pygame.draw.line(surface, ROBOT_EDGE, (cx, cy), tip, 3)
    pygame.draw.circle(surface, ROBOT, (cx, cy), 9)
    pygame.draw.circle(surface, ROBOT_EDGE, (cx, cy), 9, 2)

    width, height = surface.get_size()
    n = len(world["traj"])
    bar_w = width - 2 * MARGIN
    bar_y = height - MARGIN // 2 - 3
    pygame.draw.rect(surface, LINE, (MARGIN, bar_y, bar_w, 6))
    pygame.draw.rect(surface, ROBOT, (MARGIN, bar_y, int(bar_w * k / n), 6))


def play(world, fps=FPS):
    """开窗口实时播放一次运行。按 R 重播，关窗口退出。"""
    pygame.init()
    screen = pygame.display.set_mode(canvas_size(world["grid"]))
    pygame.display.set_caption("robot-nav-sim - A* planning + go-to-goal tracking")
    clock = pygame.time.Clock()

    k, running = 1, True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                k = 1

        if k < len(world["traj"]):
            k = k + 1

        draw(screen, world, k)
        pygame.display.flip()
        clock.tick(fps)

    pygame.quit()


def record_gif(world, out_path, every=3, size=480, colors=128):
    """离屏录制：不弹窗，逐帧画到内存画布，每 every 帧存 1 张，再用 PIL 合成。

    返回 (帧数, 字节数)。
    """
    pygame.init()
    surface = pygame.Surface(canvas_size(world["grid"]))
    n_traj = len(world["traj"])
    frame_ms = int(round(1000 / FPS * every))

    with tempfile.TemporaryDirectory() as tmp:
        saved = 0
        for k in range(1, n_traj + 1):
            draw(surface, world, k)
            if (k - 1) % every == 0:
                pygame.image.save(surface, os.path.join(tmp, f"frame_{saved:04d}.png"))
                saved = saved + 1

        from PIL import Image
        frames = []
        for i in range(saved):
            img = Image.open(os.path.join(tmp, f"frame_{i:04d}.png")).convert("RGB")
            img = img.resize((size, size), Image.LANCZOS)
            frames.append(img.quantize(colors=colors))

        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        frames[0].save(out_path, save_all=True, append_images=frames[1:],
                       duration=frame_ms, loop=0, optimize=True)

    pygame.quit()
    return saved, os.path.getsize(out_path)
