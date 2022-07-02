# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import pandas as pd
import draw_car as draw
import numpy as np

PI = np.pi


# 读取csv数据 转换为list
def read_csv(filename):
    readData = pd.read_csv(filename, header=None)
    # print(readDataRaw)

    # 获取readData中的第1列,并将此转换为list
    data_x = readData.iloc[:, 0].tolist()
    data_y = readData.iloc[:, 1].tolist()
    data_yaw = readData.iloc[:, 2].tolist()
    data_steer = readData.iloc[:, 3].tolist()
    data_v = readData.iloc[:, 4].tolist()
    timeData = list(range(1, len(data_x) + 1))  # 产生横轴坐标
    # print(data_x)
    return timeData, data_x, data_y, data_yaw, data_steer, data_v


class CarState:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, steer=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.steer = steer
        self.v = v

    def update(state, x, y, yaw, steer, v):
        state.x = x
        state.y = y
        state.yaw = state.yaw + yaw
        state.steer = steer
        state.v = v
        return state


def main():

    timeData, data_x, data_y, data_yaw, data_steer, data_v = read_csv(
        "CarState.csv")

    # 设置车辆的初始状态
    state = CarState(x=data_x[0],
                     y=data_y[0],
                     yaw=data_yaw[0],
                     steer=data_steer[0],
                     v=data_v[0])

    for index in range(len(timeData)):

        state = CarState.update(state, data_x[index], data_y[index],
                                data_yaw[index], data_steer[index],
                                data_v[index])
        cx = data_x[index]
        cy = data_y[index]

        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(data_x, data_y, "-b", label="trajectory")

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
