#include <stdlib.h>
#include <math.h>

#include "math_calc.h"
#include "filter.h"


// KF start
void Kalman_Init(Kalman_TypeDef *KF, const double Q, const double R) {
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

void lag_filter_init(Lag_TypeDef *LAG, const double a) {
  LAG->last_value = 0;
  LAG->a = a;
}

/**
 * @brief 一阶滞后滤波器
 *
 * @param LAG 全局参数
 * @param input 输入值
 * @return double 输出值
**/
double lag_filter(Lag_TypeDef *LAG, double input) {
  double tmp = 0.0;

  if (is_nearly_equal(LAG->last_value, 0.0)) {
    tmp = input;
  } else {
    tmp = LAG->a * input + (1 - LAG->a) * LAG->last_value;
  }
  LAG->last_value = tmp;
  return tmp;
}

void limit_filter_init(Limit_TypeDef *LMF, const double limit) {
  LMF->last_value = 0;
  LMF->limit = limit;
}

/**
 * @brief 限幅滤波器
 *
 * @param LMF 全局参数
 * @param input 输入值
 * @return double 输出值
**/
double limit_filter(Limit_TypeDef *LMF, double input) {
  double tmp = 0.0;

  if (is_nearly_equal(LMF->last_value, 0.0)) {
    tmp = input;
  } else {
    if (fabs(input - LMF->last_value) < LMF->limit)
      tmp = input;
    else
      tmp = LMF->last_value;
  }
  LMF->last_value = tmp;
  return tmp;
}

// 一阶滞后滤波器
// a=0-1
// 只对单一变量滤波有效
double lag_filter_1(double input, double a) {
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

// 限幅滤波器
// limit 本次值与上次值差的上限
// 只对单一变量滤波有效
double limit_filter_1(double input, double limit) {
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
