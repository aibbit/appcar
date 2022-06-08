#读取csv并作图
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

readData = pd.read_csv("UwbData.csv",header=None)   #读取csv数据
readDataRaw = pd.read_csv("UwbDataRaw.csv",header=None)
#print(readDataRaw)

data_x = readData.iloc[:,0].tolist()       #获取readData中的第1列，并将此转换为list
data_y = readData.iloc[:,1].tolist()
xData = list(range(1,len(data_x)+1))       #产生横轴坐标
#print(data_x)
data_x_raw = readData.iloc[:,0].tolist()
data_y_raw = readData.iloc[:,1].tolist()

plt.plot(xData, data_x,'r')              #画散点图

plt.title("UwbData")               #设置标题
plt.xlabel("times")                 #横轴名称
plt.ylabel("x_data")                    #纵轴名称
plt.show()                               #画图
