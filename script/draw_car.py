import matplotlib.pyplot as plt
import numpy as np
import math

PI = np.pi


class Arrow:

    def __init__(self, x, y, theta, L, c):
        angle = np.deg2rad(30)
        d = 0.3 * L
        w = 2

        x_start = x
        y_start = y
        x_end = x + L * np.cos(theta)
        y_end = y + L * np.sin(theta)

        theta_hat_L = theta + PI - angle
        theta_hat_R = theta + PI + angle

        x_hat_start = x_start
        x_hat_end_L = x_hat_start - d * np.cos(theta_hat_L)
        x_hat_end_R = x_hat_start - d * np.cos(theta_hat_R)

        y_hat_start = y_start
        y_hat_end_L = y_hat_start - d * np.sin(theta_hat_L)
        y_hat_end_R = y_hat_start - d * np.sin(theta_hat_R)

        plt.plot([x_start, x_end], [y_start, y_end], color=c, linewidth=w)
        plt.plot([x_hat_start, x_hat_end_L], [y_hat_start, y_hat_end_L],
                 color=c,
                 linewidth=w)
        plt.plot([x_hat_start, x_hat_end_R], [y_hat_start, y_hat_end_R],
                 color=c,
                 linewidth=w)


def draw_car(x, y, yaw, steer, color='black'):
    car = np.array([[-C.RB, -C.RB, C.RF, C.RF, -C.RB],
                    [C.W / 2, -C.W / 2, -C.W / 2, C.W / 2, C.W / 2]])

    wheel = np.array([[-C.TR, -C.TR, C.TR, C.TR, -C.TR],
                      [C.TW / 4, -C.TW / 4, -C.TW / 4, C.TW / 4, C.TW / 4]])

    rlWheel = wheel.copy()
    rrWheel = wheel.copy()
    frWheel = wheel.copy()

    yaw = yaw + PI

    # 车身旋转矩阵？
    Rot1 = np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])
    # 车轮旋转矩阵？
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    frWheel = np.dot(Rot2, frWheel)

    frWheel[0, :] += C.WB
    rrWheel[1, :] -= C.WD / 2
    rlWheel[1, :] += C.WD / 2

    frWheel = np.dot(Rot1, frWheel)
    rrWheel = np.dot(Rot1, rrWheel)
    rlWheel = np.dot(Rot1, rlWheel)
    car = np.dot(Rot1, car)

    frWheel += np.array([[x], [y]])
    rrWheel += np.array([[x], [y]])
    rlWheel += np.array([[x], [y]])
    car += np.array([[x], [y]])

    plt.plot(car[0, :], car[1, :], color)
    plt.plot(frWheel[0, :], frWheel[1, :], color)
    plt.plot(rrWheel[0, :], rrWheel[1, :], color)
    plt.plot(rlWheel[0, :], rlWheel[1, :], color)
    Arrow(x, y, yaw, C.WB * 0.6, color)


class C:
    # vehicle config
    # 基准点为两前轮(动力轮)中点
    # RF = 1          # [m] 基准点到车后身距离
    # RB = 0.2        # [m] 基准点到车前身距离
    # W = 0.6         # [m] 车宽
    # WD = 0.8 * W    # [m] 左右轮间距
    # WB = 1          # [m] 前后轮间距
    # TR = 0.15       # [m] 轮胎半径
    # TW = 0.2        # [m] 轮胎宽度

    RF = 3.3  # [m] distance from rear to vehicle front end of vehicle
    RB = 0.8  # [m] distance from rear to vehicle back end of vehicle
    W = 2.4  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 2.5  # [m] Wheel base
    TR = 0.44  # [m] Tyre radius
    TW = 0.7  # [m] Tyre width


if __name__ == "__main__":

    x = np.linspace(0, 5, 100)
    y = 3

    i = 0
    while i < 100:

        plt.cla()
        plt.plot(x[i], y, 'r', marker='.', linewidth=4)
        draw_car(x[i], y, 0, 0)
        # draw_car( 10, 0, PI/2, 0)
        plt.axis("equal")
        plt.pause(0.001)

        i += 1

    plt.show()  # 画图
