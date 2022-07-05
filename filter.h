#ifndef _FILTER_H_
#define _FILTER_H_

typedef struct {
    double lastP;    //上次的协方差
    double nowP;     //本次的协方差
    double x_hat;    //卡尔曼滤波的计算值，即为后验最优值
    double Kg;       //卡尔曼增益系数
    double Q;        //过程噪声(越小越相信预测，反之亦然)
    double R;        //测量噪声(越小越相信观测，反之亦然)
} Kalman_TypeDef;

typedef struct {
  double limit;       // limit 本次值与上次值差的上限
  double last_value;  // 上次的值
} Limit_TypeDef;

typedef struct {
  double a;           // a=0-1(越小越相信之前，越大越相信当前)
  double last_value;  // 上次的值
} Lag_TypeDef;

void Kalman_Init(Kalman_TypeDef *KF, const double Q, const double R);
double Kalman_Filter(Kalman_TypeDef *KF, double input);

void lag_filter_init(Lag_TypeDef *LAG, const double a);
double lag_filter(Lag_TypeDef *LAG, double input);

void limit_filter_init(Limit_TypeDef *LMF, const double limit);
double limit_filter(Limit_TypeDef *LMF, double input);

double lag_filter_1(double input, double a);
double limit_filter_1(double input, double limit);

#endif  //_FILTER_H_
