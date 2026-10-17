"""D14：把 D13 的动画录成 GIF —— 项目的第一张"门面图"。

D13 的动画只在窗口里活着，关掉就没了。今天要把它变成文件：
    practice/day14_record.py  →  docs/demo.gif

做法分两步，今天就学这个"两步"：
    1. 录制：不开窗口，把每一帧画到内存里的一块画布上，每隔几帧存一张 PNG
    2. 合成：把所有 PNG 按顺序拼成一个 GIF

为什么录制时不开窗口？因为要存 641 帧，弹窗会闪 641 次；
而且窗口刷新受显示器刷新率限制，白白浪费时间。
所以用 SDL_VIDEODRIVER=dummy —— 一个"假装有显示器"的驱动，
画的东西全在内存里，谁也看不见，但能存成文件。

注意：今天不再复制 D13 的画图代码，直接 import 过来用
（day13_animation.py 的 main() 有 if __name__ == "__main__" 保护，
 导入它不会自动开窗口）。这就是"模块复用"。

验收标准：
    - 生成 docs/demo.gif，约 0.6 MB，能双击播放
    - 播放时小车沿蓝线走、身后拖红线，约 10.7 秒走完
    - 控制台打印：抽帧 214 张 / GIF 0.61 MB

今天的 3 个空：搜「空 1」到「空 3」。
"""

import os

# 这三行必须在 import pygame 之前：告诉 pygame 别开真窗口、别碰声卡
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pygame
from PIL import Image

import day13_animation as d13          # 复用昨天的地图、轨迹和画图函数


# ===== 录制参数 =====
# 641 个采样点全存的话文件大、也没必要；每 3 个取 1 个，动作依然连贯
EVERY = 3                 # 每 3 个采样点存 1 帧
GIF_SIZE = 480            # 合成时边长缩放到 480 像素
GIF_COLORS = 128          # GIF 最多 256 色，这里用 128 色再省一半体积
FRAME_MS = 50             # 每帧显示 50 毫秒 → 214 帧 × 50ms ≈ 10.7 秒

# 路径必须以"脚本位置"为准，不能依赖"你在哪个文件夹里运行"：
#   __file__        当前文件（practice/day14_record.py）
#   abspath(...)    转成绝对路径
#   dirname(...)    取所在目录 → practice/
#   再 dirname 一次  → 项目根目录
# 少了这一步，在 PyCharm 里跑（工作目录是 practice/）就会把图存到 practice/docs/ 里。
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

FRAME_DIR = os.path.join(BASE_DIR, "docs", "frames")   # 中间产物，存完就删，不进 Git
OUT_GIF = os.path.join(BASE_DIR, "docs", "demo.gif")   # 最终交付物


def record():
    """把动画逐帧渲染到内存画布上，每隔 EVERY 帧存一张 PNG。返回存了多少张。"""
    os.makedirs(FRAME_DIR, exist_ok=True)

    pygame.init()
    canvas = pygame.Surface((d13.WIDTH, d13.HEIGHT))   # 内存里的一块画布，不是窗口

    g, start, goal, path, traj, thetas = d13.build_world()
    print(f"轨迹共 {len(traj)} 个采样点")

    saved = 0
    for k in range(1, len(traj) + 1):
        d13.draw(canvas, g, path, start, goal, traj, thetas, k)

        if (k - 1) % EVERY == 0:                                    # 空 1：什么时候存这一帧？
            name = os.path.join(FRAME_DIR, f"frame_{saved:04d}.png")     # 空 2：这一帧的文件名
            pygame.image.save(canvas, name)
            saved = saved + 1

    print(f"抽帧完成：{saved} 张（每 {EVERY} 帧取 1 张）")
    return saved


def to_gif(n):
    """把 n 张 PNG 按顺序拼成一张 GIF。"""
    frames = []
    for i in range(n):
        img = Image.open(os.path.join(FRAME_DIR, f"frame_{i:04d}.png"))
        img = img.convert("RGB").resize((GIF_SIZE, GIF_SIZE), Image.LANCZOS)
        # GIF 是索引色格式，必须把真彩色图压成有限个颜色
        frames.append(img.quantize(colors=GIF_COLORS))

    # 第一帧当主图，其余全部追加进去；loop=0 表示无限循环播放
    frames[0].save(
        OUT_GIF,
        save_all=True,
        append_images=frames[1:],       # 空 3：除了第一帧，还要追加哪些？
        duration=FRAME_MS,
        loop=0,
        optimize=True,
    )

    mb = os.path.getsize(OUT_GIF) / 1024 / 1024
    print(f"GIF 已生成：{OUT_GIF}  {mb:.2f} MB  {n * FRAME_MS / 1000:.1f} 秒")
    return mb


def main():
    n = record()
    to_gif(n)


if __name__ == "__main__":
    main()
