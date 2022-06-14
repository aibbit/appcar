#读取csv并作图
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math

lastTimePredVal = 5  # 上次估计值
lastTimePredCovVal = 0.02  # 上次估计协方差
lastTimeRealCovVal = 0.02  # 上次实际协方差
kg = 0.4 #卡尔曼增益

# val: 本次测量值
def kalman(val):
    #python中如果若想在函数内部对函数外的变量进行操作，就需要在函数内部声明其为global。
    global lastTimePredVal  # 上次估计值
    global lastTimePredCovVal  # 上次估计协方差
    global lastTimeRealCovVal  # 上次实际协方差
    global kg

    currRealVal = val  # 本次实际值
    currPredCovVal = lastTimePredCovVal  # 本次估计协方差值
    currRealCovVal = lastTimeRealCovVal  # 本次实际协方差值

    # 计算本次估计值，并更新保留上次预测值的变量
    currPredVal = lastTimePredVal + kg * (currRealVal - lastTimePredVal)
    lastTimePredVal = currPredVal

    #计算卡尔曼增益
    kg = math.sqrt(math.pow(lastTimePredCovVal, 2) / (math.pow(lastTimePredCovVal, 2) + math.pow(lastTimeRealCovVal, 2)+ 1e-10) )

    # 计算下次估计和实际协方差
    lastTimePredCovVal = math.sqrt(1.0 - kg) * currPredCovVal
    lastTimeRealCovVal = math.sqrt(1.0 - kg) * currRealCovVal

    # 返回本次的估计值,也就是滤波输出值
    return currPredVal

if __name__ == "__main__":
    #读取csv数据
    readData = pd.read_csv("UwbData.csv",header=None)
    readDataRaw = pd.read_csv("UwbDataRaw.csv",header=None)
    #print(readDataRaw)

    #获取readData中的第1列，并将此转换为list
    # data_x = readData.iloc[500:2500,0].tolist()
    # data_y = readData.iloc[:,1].tolist()
    # timeData = list(range(1,len(data_x)+1))       #产生横轴坐标
    #print(data_x)

    data_x_raw = readData.iloc[200:2600,0].tolist()
    data_y_raw = readData.iloc[:,1].tolist()
    timeData2 = list(range(1,len(data_x_raw)+1))       #产生横轴坐标

    # 卡尔曼滤波
    predData = []
    for t in data_x_raw:
        predVal = kalman(t)
        predData.append(predVal)
    # print(predData)

    #画散点图
    plt.plot(timeData2, data_x_raw,'r')
    plt.plot(timeData2, predData,'b')

    plt.title("UwbData")               #设置标题
    plt.xlabel("Times")                 #横轴名称
    plt.ylabel("Data")                    #纵轴名称
    plt.show()                               #画图
