"""格子地图与坐标换算。

地图是一个 size×size 的二维数组：0 = 空地，1 = 墙。
行号 r 向下增大（世界坐标里 y 向上），列号 c 向右增大（世界坐标里 x 向右）。
一格 = 1 米，格子 (r, c) 的世界坐标取格心 (c+0.5, r+0.5)。
"""

import numpy as np


class Grid:
    """方形栅格地图。越界一律当作墙，这样规划器不用额外的边界判断。"""

    FREE = 0
    WALL = 1

    def __init__(self, size):
        self.size = size
        self.cells = np.zeros((size, size), dtype=int)

    def set_wall(self, row, col):
        self.cells[row, col] = self.WALL

    def in_bounds(self, row, col):
        return 0 <= row < self.size and 0 <= col < self.size

    def is_wall(self, row, col):
        """越界 = 墙。这一条让规划器的邻居检查只需要一行。"""
        if not self.in_bounds(row, col):
            return True
        return self.cells[row, col] == self.WALL

    def wall_count(self):
        return int(self.cells.sum())

    def __repr__(self):
        return f"Grid(size={self.size}, walls={self.wall_count()})"


def cell_to_world(cell):
    """栅格坐标 (行, 列) → 世界坐标 (x, y)，取格子正中心。

    注意顺序是反的：列决定 x，行决定 y。
    (1, 1) → (1.5, 1.5)；  (5, 0) → (0.5, 5.5)
    """
    r, c = cell
    return (c + 0.5, r + 0.5)


def world_to_cell(x, y):
    """世界坐标 (x, y) → 所在格子的 (行, 列)。cell_to_world 的反函数。"""
    return (int(np.floor(y)), int(np.floor(x)))
