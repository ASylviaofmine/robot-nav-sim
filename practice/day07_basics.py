print("Mission 1:")
class Robot:
    pass                 # pass 的意思是"这里先空着，别报错"

r = Robot()              # 用模具压出一个零件
r.name = "小车轮"        # 给这个零件贴个标签
r.speed = 0.5
print(r.name, r.speed)   # 输出 小车轮 0.5
print()

print("Mission 2:")
class Robot:
    def __init__(self, name, speed):
        self.name = name
        self.speed = speed

r = Robot("小车轮", 0.5)     # 注意：这里要传两个参数，不是三个
print(r.name, r.speed)      # 输出 小车轮 0.5
print()

print("Mission 3:")
class Robot:
    def __init__(self, name):
        self.name = name
        self.battery = 100

    def drive(self, distance):
        self.battery = self.battery - distance
        print(f"{self.name} 跑了 {distance} 米，剩余电量 {self.battery}%")

r = Robot("小车轮")
r.drive(60)
r.drive(50)
print()

print("Mission 4:")
import numpy as np
class Grid:
    def __init__(self, size):
        self.size = size
        self.cells = np.zeros((size, size))

a = Grid(5)
b = Grid(5)
a.cells[0, 0] = 6        # 只在 a 身上划一刀
b.cells[0, 0] =6.5
print("a 的格子:", int(a.cells[0, 0]))   # 输出 1
print("b 的格子:", int(b.cells[0, 0]))   # 输出 0  ← b 毫发无伤
print()


print("Mission 5:")
import numpy as np
import matplotlib.pyplot as plt
class Grid:
    def __init__(self, size):
        self.size = size
        self.cells = np.zeros((size, size))      # 空 1、空 2：形状还是 (size, size) 吗？

    def set_wall(self, row, col):
        self.cells[row, col] = 1               # 空 3：墙是哪个数字？

    def set_start(self, row, col):
        self.cells[row, col] = 2

    def set_goal(self, row, col):
        self.cells[row, col] = 3                # 空 4：终点是哪个数字？

    def show(self):
        plt.imshow(self.cells, cmap="gray_r", interpolation="nearest")
        plt.title("Robot Grid Map")
        for i in range(self.size+1):                      # 空 5：格线要画几条？(提示：size+1)
            plt.axhline(i - 0.5, color="gray", linewidth=0.5)
            plt.axvline(i - 0.5, color="gray", linewidth=0.5)
        plt.text(1, 1, "S", color="red", ha="center", va="center", fontsize=14)
        plt.text(8, 8, "T", color="blue", ha="center", va="center", fontsize=14)
        plt.show()


# ===== 下面用这个类重建昨天那张地图（自己写）=====
g = Grid(10)
g.set_wall(0, 5)         # 只加一个障碍物就够了
g.set_start(1, 1)
g.set_goal(8, 8)
g.show()
