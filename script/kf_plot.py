# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import pandas as pd

# KF参数
Q = 1e-6       #预测(过程)噪声方差
R = 0.002      #测量(观测)噪声方差 取正态分布的(3σ)^2作为r的初始化值
Kg = 0
lastP = 1      #lastP相当于上一次的值,初始值可以为1,不可以为0
x_hat = 0
nowP = 0

# KF滤波
def kalman(input):
    #python中如果若想在函数内部对函数外的变量进行操作,就需要在函数内部声明其为global.
    global Q
    global R
    global Kg
    global lastP
    global x_hat
    global nowP

    x_t = x_hat              #当前先验预测值 = 上一次最优值
    nowP = lastP + Q         #本次的协方差矩阵
    Kg = nowP/(nowP + R)     #卡尔曼增益系数计算
    output = x_t + Kg * (input - x_t)  #当前最优值
    x_hat = output                     #更新最优值
    lastP = (1 - Kg) * nowP            #更新协方差矩阵

    return output

#读取csv数据 转换为list
def read_csv(filename):
    readData = pd.read_csv(filename,header=None)
    #print(readDataRaw)

    #获取readData中的第1列,并将此转换为list
    data_x = readData.iloc[:,0].tolist()
    data_y = readData.iloc[:,1].tolist()
    timeData = list(range(1,len(data_x)+1))        #产生横轴坐标
    #print(data_x)
    return timeData,data_x,data_y

if __name__ == "__main__":

    timeData,data_x_raw,data_y_raw = read_csv("UwbDataRaw.csv")
    timeData2,data_x,data_y = read_csv("UwbData.csv")

    # 卡尔曼滤波
    predData = []
    for t in data_x_raw:
        predVal = kalman(t)
        predData.append(predVal)
    # print(predData)

    #画图
    plt.rcParams['font.family']=['SimHei']   #字体 支持中文
    plt.rcParams['axes.unicode_minus']=False #正常显示负号

    plt.figure(figsize=(8, 5))     # 这里定义了图像大小
    x1, = plt.plot(timeData, data_x_raw,'r')    #原始数据
    # x2, = plt.plot(timeData, predData,'g')    #仿真KF 参数相同
    x2, = plt.plot(timeData2,data_x,'b')        #KF结果

    plt.title("UwbData")               #设置标题
    plt.xlabel("Times")                #横轴名称
    plt.ylabel("Data_X")               #纵轴名称
    plt.legend(handles=[x1, x2], labels=["滤波前", "滤波后"], loc="best")#图例

    #画图 y轴
    plt.figure(figsize=(8, 5))     # 这里定义了图像大小
    plt.plot(timeData, data_y_raw,'r')  #原始数据
    plt.plot(timeData2,data_y,'b')      #KF结果

    plt.title("UwbData")               #设置标题
    plt.xlabel("Times")                #横轴名称
    plt.ylabel("Data_Y")               #纵轴名称
    plt.show()                         #画图
