print("Mission 1:")
import numpy as np
x,y=0.0,0.0
gx,gy=5.0,5.0
theta_target=np.arctan2(gy-y,gx-x)
print(f"目标方向角={theta_target:.4f}\n弧度={np.rad2deg(theta_target):.1f}度")
print()

print("Mission 2:")
import numpy as np
import matplotlib.pyplot as plt

x, y = 0.0, 0.0
theta = 0.0            # 当前朝向：0 = 朝右（东）
goal = (5.0, 5.0)
Kp = 2.0
dt = 0.05

thetas, errors = [], []

for i in range(int(20 / dt)):
    theta_target = np.arctan2(goal[1] - y, goal[0] - x)
    e = theta_target - theta             # 空 1：角度误差 = 谁减谁？（照上面那张图）
    omega = (Kp * e)                   # 空 2：P 控制，输出角速度
    theta = theta + omega * dt

    thetas.append(theta)
    errors.append(e)

print(f"20 秒后朝向 = {theta:.4f} 弧度 = {np.rad2deg(theta):.1f} 度")

t = np.arange(len(thetas)) * dt
plt.plot(t, np.rad2deg(thetas), label="heading")
plt.axhline(45, color="gray", linestyle="--", label="target 45 deg")
plt.xlabel("time (s)")
plt.ylabel("heading (deg)")
plt.legend()
plt.grid(True)
plt.show()
print()

print("Mission 3:")
import numpy as np
import matplotlib.pyplot as plt

x, y, theta = 0.0, 0.0, 0.0
goal = (5.0, 5.0)
Kp = 5.0
speed = 0.5
dt = 0.05
tol = 0.1              # 距目标小于这个就算到达

xs, ys = [x], [y]

for i in range(int(40 / dt)):
    theta_target = np.arctan2(goal[1] - y, goal[0] - x)
    e = theta_target - theta
    omega = Kp * e
    theta = theta + omega * dt

    x = x + speed * np.cos(theta) * dt     # 空 1：沿朝向走多远
    y = y + speed * np.sin(theta) * dt     # 空 2：照上一行，把 x 换成 y

    xs.append(x)
    ys.append(y)

    if np.hypot(goal[0] - x, goal[1] - y) < tol:   # hypot = 勾股定理算距离
        break                              # 空 3：到站了怎么办？

print(f"用时 {i * dt:.2f} 秒，到达 ({x:.2f}, {y:.2f})")

plt.plot(xs, ys, label="trajectory")
plt.plot(xs[0], ys[0], "go", label="start")
plt.plot(goal[0], goal[1], "r*", markersize=15, label="goal")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()
