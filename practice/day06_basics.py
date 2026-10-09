print("mission 1:")
import numpy as np

a = np.array([1, 2, 3, 4])
print(a * 2)        # 输出 [2 4 6 8]  ← 整个数组一起乘 2
print()


print("mission 2:")
import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(0, 10, 100)     # 从 0 到 10 均匀取 100 个点
y = np.sin(x)
plt.plot(x, y)
plt.show()                       # 会弹出一个窗口显示正弦曲线


print("mission 3:")
import numpy as np
import matplotlib.pyplot as plt

grid = np.array([
    [1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1],
    [1, 0, 2, 0, 1],
    [1, 0, 0, 0, 1],
    [1, 1, 1, 3, 1],
])

plt.imshow(grid, cmap="viridis")   # imshow = 把每个数字翻译成一块颜色
plt.title("my grid")
plt.colorbar()                     # 右边那根颜色刻度条
plt.show()


print("mission 4:")
import numpy as np
import matplotlib.pyplot as plt

size = 10
grid = np.zeros((size, size))     # 10×10 全 0 矩阵（注意是双括号：形状要打包成一个整体）

grid[0, :]  = 1        # 第 0 行整行刷成墙
grid[-1, :] = 1        # 最后一行
grid[:, 0]  = 1   # 空 1：第 0 列整列（提示：逗号前面管"行"，冒号 = 全都要）
grid[:, -1] = 1        # 最后一列

grid[1, 1] = 2         # 起点
grid[8, 8] = 3         # 终点

plt.imshow(grid, cmap="viridis")
plt.title("Robot Grid Map")
plt.colorbar()
plt.show()

print("mission 5:")
grid[5, 2:8] = 1        # 第 5 行中间加一道横墙（切片赋值，一行刷一片）

plt.imshow(grid, cmap="gray_r", interpolation="nearest")  # gray_r: 墙黑、路白
plt.title("Robot Grid Map")
for i in range(size + 1):          # 画格线，让每格边界可见
    plt.axhline(i - 0.5, color="gray", linewidth=0.5)
    plt.axvline(i - 0.5, color="gray", linewidth=0.5)
plt.text(1, 1, "S", color="red", ha="center", va="center", fontsize=14)
plt.text(8, 8, "T", color="blue", ha="center", va="center", fontsize=14)
plt.show()
