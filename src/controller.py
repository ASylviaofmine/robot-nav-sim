"""路点跟踪控制器：go-to-goal 比例控制 + 角度归一化。

这个模块刻意不认识 Grid —— 它只接收一串 (x, y) 路点。
好处是同一个控制器既能跟栅格路径，也能跟手画的曲线。

每一步做三件事：
    1. 算出"目标在哪个方向"   theta_target = arctan2(dy, dx)
    2. 算出"车头偏了多少"     e = theta_target - theta，收进 (-π, π]
    3. 转车头 + 沿车头前进
"""

import numpy as np

# 默认参数：D9 调出来的
DEFAULT_TOL = 0.05        # 离路点多近算"追上了"（米）
DEFAULT_SPEED = 0.5       # 前进速度（米/秒）
DEFAULT_KP = 5.0          # 转向增益
DEFAULT_DT = 0.05         # 仿真步长（秒）


def wrap_angle(e):
    """把角度差折进 (-π, π]，让车头永远走"最近的那一侧"。

    不做这一步，theta 会一路累加到几十弧度，误差跟着爆炸，
    车就在原地打转、永远靠不上目标点（D11 实测过）。
    """
    return (e + np.pi) % (2 * np.pi) - np.pi


def follow(waypoints, tol=DEFAULT_TOL, speed=DEFAULT_SPEED,
           Kp=DEFAULT_KP, dt=DEFAULT_DT, t_max=200.0):
    """沿路点开过去，返回 (轨迹, 每步车头角, 总用时)。

    轨迹是 [(x, y), ...]，长度 = 仿真步数 + 1；
    thetas[i] 是走完第 i 步时的车头角，绘图时要靠它画朝向。

    t_max 是安全阀：路点写错时最多模拟这么久，避免死循环。
    """
    x, y, theta = waypoints[0][0], waypoints[0][1], 0.0
    idx, t = 1, 0.0
    traj, thetas = [(x, y)], [theta]

    while idx < len(waypoints) and t < t_max:
        tx, ty = waypoints[idx]

        theta_target = np.arctan2(ty - y, tx - x)
        e = wrap_angle(theta_target - theta)
        theta = theta + Kp * e * dt

        x = x + speed * np.cos(theta) * dt
        y = y + speed * np.sin(theta) * dt
        t = t + dt

        traj.append((x, y))
        thetas.append(theta)

        if np.hypot(tx - x, ty - y) < tol:
            idx = idx + 1

    return traj, thetas, t


def path_length(traj):
    """轨迹的实际行驶距离（各段直线距离之和）。"""
    return float(np.sum(np.hypot(np.diff([p[0] for p in traj]),
                                 np.diff([p[1] for p in traj]))))


def end_point_error(traj, goal_xy):
    """最终停的位置离目标点还有多远。"""
    return float(np.hypot(traj[-1][0] - goal_xy[0], traj[-1][1] - goal_xy[1]))
