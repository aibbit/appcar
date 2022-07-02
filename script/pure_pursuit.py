import numpy as np
import math
import matplotlib.pyplot as plt

import draw_car as draw

k = 0.1  # 前视距离系数
Lfc = 2.0  # 前视距离
Kp = 1.0  # 速度P控制器系数
dt = 0.2  # 时间间隔，单位：s
L = 2.9  # 车辆轴距，单位：m
PI = np.pi


class VehicleState:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, steer=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.steer = steer

    def update(state, a, delta):

        # if delta > PI/6:
        #     delta = PI/6
        # if delta < -PI/6:
        #     delta = -PI/6

        state.x = state.x + state.v * math.cos(state.yaw) * dt
        state.y = state.y + state.v * math.sin(state.yaw) * dt
        state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
        state.v = state.v + a * dt
        state.steer = delta
        return state


def PControl(target, current):
    a = Kp * (target - current)
    return a


def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    # if state.v < 0:  # back
    #     alpha = math.pi - alpha

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lfc, 1.0)

    return delta, ind


def calc_target_index(state, cx, cy):
    # 搜索最临近的路点
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx**2 + idy**2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L_steps = 0.0

    # 搜索到前视距离最近的路点
    while Lfc > L_steps and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L_steps += math.sqrt(dx**2 + dy**2)
        ind += 1

    return ind


def control(state, cx, cy, pind):

    ind = target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    tmp = (state.x - cx[0]) * (cy[-1] - cy[0]) - (state.y - cy[0]) * (cx[-1] -
                                                                      cx[0])

    # alpha = (math.sqrt( (tx - state.x)**2 + (ty - state.y)**2 ))
    # if tmp > 0:
    #     alpha =  -1 * alpha

    alpha = (point_to_line(state, cx[0], cy[0], cx[-1], cy[-1]))
    if tmp > 0:
        alpha = -1 * alpha

    return alpha * 0.0174533, ind


# 搜索最临近的路点
def target_index(state, cx, cy):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx**2 + idy**2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    return ind


# 计算点到直线之间的距离
def point_to_line(state, x1, y1, x2, y2):
    # 处理斜率不存在的情况
    if (x1 == x2):
        tmp = abs(state.x - x1)

    k = (y2 - y1) / (x2 - x1)
    b = y2 - x2 * k
    tmp = abs(k * state.x - state.y + b) / math.sqrt(k**2 + 1)

    return tmp


def main():
    #  设置目标路点
    cx = np.arange(0, 50, 1)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 1.8 / 3.6  # [m/s]

    T = 200.0  # 最大模拟时间

    # 设置车辆的初始状态
    state = VehicleState(x=5.0, y=0.0, yaw=PI / 2, v=0.0, steer=0.0)

    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)

    while T >= time and lastIndex > target_ind:
        ai = PControl(target_speed, state.v)
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        state = VehicleState.update(state, ai, di)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.plot(cx[target_ind], cy[target_ind], "go", label="target")

        draw.draw_car(state.x, state.y, PI + state.yaw, state.steer)

        plt.axis("equal")
        plt.grid(True)
        # plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4] + "  " + "angle:" + str(state.steer* 57.29578)[:4])
        plt.title("angle:" + str(state.steer * 57.29578)[:4] + "  " + "yaw:" +
                  str(state.yaw * 57.29578)[:4])
        plt.pause(0.01)
    plt.show()


if __name__ == '__main__':
    main()
