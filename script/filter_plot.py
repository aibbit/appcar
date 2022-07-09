# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import pandas as pd


# 卡尔曼滤波器
class KF:
    def __init__(self, Q=0.0, R=0.0):
        # KF参数
        self.Q = Q  # 预测(过程)噪声方差
        self.R = R  # 测量(观测)噪声方差
        self.Kg = 0
        self.lastP = 1  # lastP相当于上一次的值,初始值可以为1,不可以为0
        self.x_hat = 0
        self.nowP = 0

    # KF滤波
    def kalman(self, input):

        x_t = self.x_hat                            # 当前先验预测值 = 上一次最优值
        nowP = self.lastP + self.Q                  # 本次的协方差矩阵
        self.Kg = nowP / (nowP + self.R)            # 卡尔曼增益系数计算
        output = x_t + self.Kg * (input - x_t)      # 当前最优值
        self.x_hat = output                         # 更新最优值
        self.lastP = (1 - self.Kg) * nowP           # 更新协方差矩阵
        return output


# 一阶滞后滤波器
# A=0-1
def FirstOrderLagFilter(Data, A):
    ReturnData = [Data[0]]
    for Value in Data[1:]:
        ReturnValue = (1 - A) * Value + A * ReturnData[-1]
        ReturnData.append(ReturnValue)
    return ReturnData


# 限幅滤波器
# Amplitude 本次值与上次值之差
def LimitFilter(Data, Amplitude):
    ReturnData = [Data[0]]
    for Value in Data[1:]:
        # print(abs(Value - ReturnData[-1]))
        if abs(Value - ReturnData[-1]) < Amplitude:  # 限幅
            ReturnData.append(Value)
        else:
            ReturnData.append(ReturnData[-1])
    return ReturnData


# 读取csv数据 转换为list
def read_csv(filename):
    readData = pd.read_csv(filename, header=None)
    # print(readDataRaw)

    # 获取readData中的第1列,并将此转换为list
    data_x = readData.iloc[:, 0].tolist()
    data_y = readData.iloc[:, 1].tolist()
    timeData = list(range(1, len(data_x) + 1))  # 产生横轴坐标
    # print(data_x)
    return timeData, data_x, data_y


if __name__ == "__main__":

    timeData, data_x_raw, data_y_raw = read_csv("UwbDataRaw.csv")
    timeData2, data_x, data_y = read_csv("UwbData.csv")

    LimitFilterData = LimitFilter(data_x_raw, 10)
    # LagFilterData = FirstOrderLagFilter(LimitFilterData, 0.95)
    # print(LagFilterData)

    predData = []
    kf = KF(1e-6, 1e-4)   # 初始化KF
    for t in LimitFilterData:
        predVal = kf.kalman(t)
        predData.append(predVal)
    # print(predData)

    # 画图
    plt.rcParams['font.family'] = ['SimHei']    # 字体 支持中文
    plt.rcParams['axes.unicode_minus'] = False  # 正常显示负号

    # 数据 x
    plt.figure(figsize=(8, 5))                  # 这里定义了图像大小
    x1, = plt.plot(timeData, data_x_raw, 'r')   # 原始数据
    x3, = plt.plot(timeData, predData, 'g')     # 仿真滤波 参数相同 先验已知
    x2, = plt.plot(timeData2, data_x, 'b')      # 实际滤波结果

    plt.title("UwbData")  # 设置标题
    plt.xlabel("Times")   # 横轴名称
    plt.ylabel("Data_X")  # 纵轴名称
    plt.legend(handles=[x1, x2], labels=["滤波前", "滤波后"], loc="best")  # 图例

    # 数据 y
    plt.figure(figsize=(8, 5))
    plt.plot(timeData, data_y_raw, 'r')
    plt.plot(timeData2, data_y, 'b')

    plt.title("UwbData")
    plt.xlabel("Times")
    plt.ylabel("Data_Y")
    plt.show()
