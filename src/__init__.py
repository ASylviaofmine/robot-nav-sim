"""robot-nav-sim：2D 移动机器人自主导航仿真。

模块划分（依赖方向单向，绝不循环）：

    grid        地图与坐标换算 —— 最底层，不依赖任何其它模块
    planner     A* 路径规划 —— 只依赖 grid
    controller  路点跟踪控制 —— 只处理坐标数组，连 grid 都不认识
    simulation  把上面三个串成"一次完整运行" —— 依赖 grid / planner / controller
    viewer      可视化与录制 —— 依赖 grid 与 simulation 产出的数据结构

这样分层的好处：想换规划算法（比如加 RRT），只动 planner.py，
controller 和 viewer 一行都不用改。
"""

__version__ = "0.15.0"
