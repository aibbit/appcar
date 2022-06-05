#include "filter.h"
#include "stdlib.h"

// KF start
void Kalman_Init_X(Kalman_TypeDef *KF) {
  KF->Q = 1e-6; //预测(过程)噪声方差
  KF->R = 0.002;  //测量(观测)噪声方差 取正态分布的(3σ)^2作为r的初始化值
  KF->Kg = 0;
  KF->lastP = 1; // lastP相当于上一次的值,初始值可以为1,不可以为0
  KF->x_hat = 0;
}

void Kalman_Init_Y(Kalman_TypeDef *KF) {
  KF->Q = 1e-6;
  KF->R = 0.002;
  KF->Kg = 0;
  KF->lastP = 1;
  KF->x_hat = 0;
}

float Kalman_Filter(Kalman_TypeDef *KF, float input) {
  float output = 0, x_t;        // output为卡尔曼滤波计算值
  x_t = KF->x_hat;              //当前先验预测值 = 上一次最优值
  KF->nowP = KF->lastP + KF->Q; //本次的协方差矩阵
  KF->Kg = KF->nowP / (KF->nowP + KF->R); //卡尔曼增益系数计算
  output = x_t + KF->Kg * (input - x_t);  //当前最优值
  KF->x_hat = output;                     //更新最优值
  KF->lastP = (1 - KF->Kg) * KF->nowP;    //更新协方差矩阵
  return output;
}
// KF end
