# 读取csv并作图
import matplotlib.pyplot as plt
import pandas as pd
import sys

if __name__ == "__main__":

    filename = "UwbData.csv"
    if len(sys.argv) > 1:
        filename = str(sys.argv[1])

    # 读取csv数据
    readData = pd.read_csv(filename, header=None)
    # print(readData)

    # 获取readData中的第1列，并将此转换为list
    data_x = readData.iloc[:, 0].tolist()
    data_y = readData.iloc[:, 1].tolist()
    timeData = list(range(1, len(data_x) + 1))  # 产生横轴坐标
    # print(data_x)

    # 画散点图
    # plt.plot(data_x, data_y,'r')
    plt.plot(timeData, data_x, 'r')
    plt.plot(timeData, data_y, 'b')

    plt.title("UwbData")  # 设置标题
    plt.xlabel("Times")   # 横轴名称
    plt.ylabel("Data")    # 纵轴名称
    plt.show()            # 画图
