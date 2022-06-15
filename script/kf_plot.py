#读取csv并作图
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math
import sys

Q = 1e-6       #预测(过程)噪声方差
R = 0.002   #测量(观测)噪声方差 取正态分布的(3σ)^2作为r的初始化值
Kg = 0
lastP = 1   #lastP相当于上一次的值,初始值可以为1,不可以为0
x_hat = 0
nowP = 0

# val: 本次测量值
def kalman(input):
    #python中如果若想在函数内部对函数外的变量进行操作，就需要在函数内部声明其为global。
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

if __name__ == "__main__":
    #读取csv数据

    if len(sys.argv) == 2 :
        filename_raw = str(sys.argv[0])
        filename = str(sys.argv[1])
    else :
        filename_raw = "UwbDataRaw.csv"
        filename = "UwbData.csv"

    readDataRaw = pd.read_csv(filename_raw,header=None)
    readData = pd.read_csv(filename,header=None)
    #print(readDataRaw)

    #获取readData中的第1列，并将此转换为list
    data_x_raw = readDataRaw.iloc[:,0].tolist()
    data_y_raw = readDataRaw.iloc[:,1].tolist()
    timeData = list(range(1,len(data_x_raw)+1))     #产生横轴坐标

    data_x = readData.iloc[:,0].tolist()
    data_y = readData.iloc[:,1].tolist()
    timeData2 = list(range(1,len(data_x)+1))        #产生横轴坐标
    #print(data_x)

    # 卡尔曼滤波
    predData = []
    for t in data_x_raw:
        predVal = kalman(t)
        predData.append(predVal)
    # print(predData)

    #画散点图
    plt.plot(timeData, data_x_raw,'r')  #原始数据
    #plt.plot(timeData, predData,'b')    #该程序KF 参数相同
    plt.plot(timeData2,data_x,'g')      #KF结果

    plt.title("UwbData")               #设置标题
    plt.xlabel("Times")                #横轴名称
    plt.ylabel("Data_X")                 #纵轴名称
    plt.show()                         #画图

    #画散点图 y轴
    plt.plot(timeData, data_y_raw,'r')  #原始数据
    plt.plot(timeData2,data_y,'g')      #KF结果

    plt.title("UwbData")               #设置标题
    plt.xlabel("Times")                #横轴名称
    plt.ylabel("Data_Y")                 #纵轴名称
    plt.show()                         #画图
