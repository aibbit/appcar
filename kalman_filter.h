#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

typedef struct {
    float lastP;    //上次的协方差
    float nowP;     //本次的协方差
    float x_hat;    //卡尔曼滤波的计算值，即为后验最优值
    float Kg;       //卡尔曼增益系数
    float Q;        //过程噪声
    float R;        //测量噪声
} Kalman1_TypeDef;


void Kalman_Init_X(Kalman1_TypeDef *KF);
void Kalman_Init_Y(Kalman1_TypeDef *KF);
float Kalman1_Filter(Kalman1_TypeDef *KF, float input);

#endif
