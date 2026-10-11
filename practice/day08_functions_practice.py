import numpy as np
import matplotlib.pyplot as plt

def simulate(Kp, Ki, Kd, friction, target, T):
    x = 0.0  # 当前位置（米）
    v = 0.0  # 当前速度（米/秒）
    integ = 0.0  # 误差累加（给 I 用）
    prev_e = target - x  # 上一步的误差（给 D 用）
    dt = 0.05  # 每步 0.05 秒
    T = 20.0  # 模拟 20 秒

    xs = []  # 记录每一步的位置，用来画图

    for i in range(int(T / dt)):
        e = target - x  # ① 误差：还差多远
        integ = integ + e * dt  # ② 误差累加（I 用）
        d = (e - prev_e) / dt  # ③ 误差变化率（D 用）
        prev_e = e

        u = Kp * e + Ki * integ + Kd * d  # ④ PID 输出 = 推力

        v = v + (u - friction) * dt  # ⑤ 推力 → 速度
        x = x + v * dt  # ⑥ 速度 → 位置

        xs.append(x)
    return xs


xs = simulate(Kp=1, Ki=0, Kd=0, friction=0, target=10, T=20)
print(len(xs))       # 期望 400
print(xs[-1])        # 期望 6.17  ← 和 Mission 1 打印的那个数一样

plt.figure(figsize=(9, 5))
xs = simulate(Kp=1, Ki=0, Kd=0, friction=0, target=10, T=20)
t = np.arange(len(xs)) * 0.05        # 400 个点 × 每步 0.05 秒 = 时间轴
plt.plot(t, xs, label="P only")


xs = simulate(Kp=1, Ki=0, Kd=2, friction=0, target=10, T=20)
t = np.arange(len(xs)) * 0.05        # 400 个点 × 每步 0.05 秒 = 时间轴
plt.plot(t, xs, label="PD only")

xs = simulate(Kp=1, Ki=0, Kd=2, friction=0.5, target=10, T=20)
t = np.arange(len(xs)) * 0.05        # 400 个点 × 每步 0.05 秒 = 时间轴
plt.plot(t, xs, label="PD+friction only")


xs = simulate(Kp=1, Ki=0.3, Kd=2, friction=0.5, target=10, T=20)
t = np.arange(len(xs)) * 0.05        # 400 个点 × 每步 0.05 秒 = 时间轴
plt.plot(t, xs, label="PID+friction only")

plt.axhline(10, color="gray", linestyle="--", label="target")
plt.xlabel("time (s)")
plt.ylabel("position (m)")
plt.title("PID tuning comparison")
plt.legend()
plt.grid(True)
plt.show()




