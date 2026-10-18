"""robot-nav-sim 入口：一条命令跑完"规划 + 跟踪"。

用法：
    python main.py                    # 开窗口实时播放（按 R 重播，关窗口退出）
    python main.py --gif              # 不弹窗，录 docs/demo.gif
    python main.py --tol 0.2          # 改跟踪容差，做参数实验
    python main.py --gif --speed 1.0  # 录一段开得更快的版本

这个文件很短，因为它**不实现任何算法** —— 它只负责把模块接起来。
main.py 该干的事是"装配"，不是"干活"：算法在 src/ 里，各管一摊。

今天的 4 个空：搜「空 1」到「空 4」。
"""

import argparse
import os

from src.simulation import build_world, describe
from src.viewer import ______________            # 空 1：从 viewer 导入哪两个函数？

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def parse_args():
    p = argparse.ArgumentParser(
        description="robot-nav-sim: A* planning + go-to-goal tracking")
    p.add_argument("--gif", action="store_true",
                   help="record a GIF instead of opening a window")
    p.add_argument("--tol", type=float, default=0.05,
                   help="how close counts as 'reached' (metres)")
    p.add_argument("--speed", type=float, default=0.5, help="forward speed (m/s)")
    p.add_argument("--kp", type=float, default=5.0, help="heading gain")
    p.add_argument("--out", default=os.path.join(BASE_DIR, "docs", "demo.gif"))
    return p.parse_args()


def main():
    args = parse_args()

    # 1. 跑一次完整仿真：建图 → A* 规划 → 路点跟踪
    world = build_world(tol=args.tol, speed=args.speed, Kp=args.kp)
    print(describe(world))

    # 2. 决定这次怎么展示
    if args.gif:
        frames, nbytes = ______________          # 空 2：录 GIF，存到 args.out
        print(f"GIF: {args.out}  {frames} frames  {nbytes / 1024 / 1024:.2f} MB")
    else:
        ______________                           # 空 3：开窗口播放


if ____________:                                 # 空 4
    main()
