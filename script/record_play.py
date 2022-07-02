# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import pandas as pd
import draw_car as draw
import numpy as np
import math

dt = 0.2  # 时间间隔，单位：s
L = 2.9  # 车辆轴距，单位：m
PI = np.pi

#读取csv数据 转换为list
def read_csv(filename):
    readData = pd.read_csv(filename,header=None)
    #print(readDataRaw)

    #获取readData中的第1列,并将此转换为list
    data_x = readData.iloc[:,0].tolist()
    data_y = readData.iloc[:,1].tolist()
    data_yaw = readData.iloc[:,2].tolist()
    data_steer = readData.iloc[:,3].tolist()
    data_v = readData.iloc[:,4].tolist()
    timeData = list(range(1,len(data_x)+1))        #产生横轴坐标
    #print(data_x)
    return timeData,data_x,data_y,data_yaw,data_steer,data_v


class VehicleState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0,steer=0.0):
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

def main():

    T = 200.0  # 最大模拟时间



    read_csv()


if __name__ == '__main__':
    main()
