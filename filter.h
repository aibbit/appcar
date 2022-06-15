#ifndef _FILTER_H_
#define _FILTER_H_

typedef struct {
    double lastP;    //上次的协方差
    double nowP;     //本次的协方差
    double x_hat;    //卡尔曼滤波的计算值，即为后验最优值
    double Kg;       //卡尔曼增益系数
    double Q;        //过程噪声
    double R;        //测量噪声
} Kalman_TypeDef;

void Kalman_Init(Kalman_TypeDef *KF,double Q,double R);
double Kalman_Filter(Kalman_TypeDef *KF, double input);

#endif  //_FILTER_H_
