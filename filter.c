#include <stdlib.h>
#include <math.h>

#include "math_calc.h"
#include "filter.h"


// KF start
void Kalman_Init(Kalman_TypeDef *KF,double Q,double R) {
  KF->Q = Q;//预测(过程)噪声方差
  KF->R = R;//测量(观测)噪声方差 取正态分布的(3σ)^2作为r的初始化值
  KF->Kg = 0;
  KF->lastP = 1;// lastP相当于上一次的值,初始值可以为1,不可以为0
  KF->x_hat = 0;
}

double Kalman_Filter(Kalman_TypeDef *KF, double input) {
  double output = 0, x_t;        // output为卡尔曼滤波计算值
  x_t = KF->x_hat;              //当前先验预测值 = 上一次最优值
  KF->nowP = KF->lastP + KF->Q; //本次的协方差矩阵
  KF->Kg = KF->nowP / (KF->nowP + KF->R); //卡尔曼增益系数计算
  output = x_t + KF->Kg * (input - x_t);  //当前最优值
  KF->x_hat = output;                     //更新最优值
  KF->lastP = (1 - KF->Kg) * KF->nowP;    //更新协方差矩阵
  return output;
}
// KF end

//FIXME 只对单一变量滤波有效
// 一阶滞后滤波器
// a=0-1
double lag_filter(double input, double a) {
  static double last_value = 0.0;
  double tmp = 0.0;

  if (is_nearly_equal(last_value, 0.0)) {
    tmp = input;
  } else {
    tmp = a * input + (1 - a) * last_value;
  }
  last_value = tmp;
  return tmp;
}

//FIXME 只对单一变量滤波有效
// 限幅滤波器
// limit 本次值与上次值差的上限
double limit_filter(double input, double limit) {
  static double last_value = 0.0;
  double tmp = 0.0;

  if (is_nearly_equal(last_value, 0.0)) {
    tmp = input;
  } else {
    if (fabs(input - last_value) < limit)
      tmp = input;
    else
      tmp = last_value;
  }
    last_value = tmp;
    return tmp;
}
