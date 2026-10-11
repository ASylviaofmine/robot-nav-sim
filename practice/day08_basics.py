print("Mission 1:")
import numpy as np
import matplotlib.pyplot as plt

Kp = 1.0
Ki = 0.0
Kd = 0.0
friction = 0.0          # 地面摩擦力（扰动），先关掉
target = 10.0           # 目标位置

x = 0.0                 # 当前位置（米）
v = 0.0                 # 当前速度（米/秒）
integ = 0.0             # 误差累加（给 I 用）
prev_e = target - x     # 上一步的误差（给 D 用）
dt = 0.05               # 每步 0.05 秒
T = 20.0                # 模拟 20 秒

xs = []                 # 记录每一步的位置，用来画图

for i in range(int(T / dt)):
    e = target - x                  # ① 误差：还差多远
    integ = integ + e * dt          # ② 误差累加（I 用）
    d = (e - prev_e) / dt           # ③ 误差变化率（D 用）
    prev_e = e

    u = Kp * e + Ki * integ + Kd * d    # ④ PID 输出 = 推力

    v = v + (u - friction) * dt     # ⑤ 推力 → 速度
    x = x + v * dt                  # ⑥ 速度 → 位置

    xs.append(x)

print(f"20 秒后，位置 = {x:.2f} 米，最大到过 {max(xs):.2f} 米")
print()


print("Mission 2:")
import numpy as np
import matplotlib.pyplot as plt

Kp = 1.0
Ki = 0.0
Kd = 2.0
friction = 0.0          # 地面摩擦力（扰动），先关掉
target = 10.0           # 目标位置

x = 0.0                 # 当前位置（米）
v = 0.0                 # 当前速度（米/秒）
integ = 0.0             # 误差累加（给 I 用）
prev_e = target - x     # 上一步的误差（给 D 用）
dt = 0.05               # 每步 0.05 秒
T = 20.0                # 模拟 20 秒

xs = []                 # 记录每一步的位置，用来画图

for i in range(int(T / dt)):
    e = target - x                  # ① 误差：还差多远
    integ = integ + e * dt          # ② 误差累加（I 用）
    d = (e - prev_e) / dt           # ③ 误差变化率（D 用）
    prev_e = e

    u = Kp * e + Ki * integ + Kd * d    # ④ PID 输出 = 推力

    v = v + (u - friction) * dt     # ⑤ 推力 → 速度
    x = x + v * dt                  # ⑥ 速度 → 位置

    xs.append(x)

print(f"20 秒后，位置 = {x:.2f} 米，最大到过 {max(xs):.2f} 米")
print()

print("Mission 3:")
import numpy as np
import matplotlib.pyplot as plt

Kp = 1.0
Ki = 0.0
Kd = 2.0
friction = 0.5   # 地面摩擦力（扰动），先关掉
target = 10.0           # 目标位置

x = 0.0                 # 当前位置（米）
v = 0.0                 # 当前速度（米/秒）
integ = 0.0             # 误差累加（给 I 用）
prev_e = target - x     # 上一步的误差（给 D 用）
dt = 0.05               # 每步 0.05 秒
T = 20.0                # 模拟 20 秒

xs = []                 # 记录每一步的位置，用来画图

for i in range(int(T / dt)):
    e = target - x                  # ① 误差：还差多远
    integ = integ + e * dt          # ② 误差累加（I 用）
    d = (e - prev_e) / dt           # ③ 误差变化率（D 用）
    prev_e = e

    u = Kp * e + Ki * integ + Kd * d    # ④ PID 输出 = 推力

    v = v + (u - friction) * dt     # ⑤ 推力 → 速度
    x = x + v * dt                  # ⑥ 速度 → 位置

    xs.append(x)

print(f"20 秒后，位置 = {x:.2f} 米，最大到过 {max(xs):.2f} 米")
print()

print("Mission 4:")
import numpy as np
import matplotlib.pyplot as plt

Kp = 1.0
Ki =0.3
Kd = 2.0
friction = 0.5          # 地面摩擦力（扰动），先关掉
target = 10.0           # 目标位置


x = 0.0                 # 当前位置（米）
v = 0.0                 # 当前速度（米/秒）
integ = 0.0             # 误差累加（给 I 用）
prev_e = target - x     # 上一步的误差（给 D 用）
dt = 0.05               # 每步 0.05 秒
T = 20.0                # 模拟 20 秒

xs = []                 # 记录每一步的位置，用来画图

for i in range(int(T / dt)):
    e = target - x                  # ① 误差：还差多远
    integ = integ + e * dt          # ② 误差累加（I 用）
    d = (e - prev_e) / dt           # ③ 误差变化率（D 用）
    prev_e = e

    u = Kp * e + Ki * integ + Kd * d    # ④ PID 输出 = 推力

    v = v + (u - friction) * dt     # ⑤ 推力 → 速度
    x = x + v * dt                  # ⑥ 速度 → 位置

    xs.append(x)

print(f"20 秒后，位置 = {x:.2f} 米，最大到过 {max(xs):.2f} 米")

