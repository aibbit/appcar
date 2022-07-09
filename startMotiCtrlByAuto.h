#ifndef __START_MOTI_CTRL_BY_AUTO_H__
#define __START_MOTI_CTRL_BY_AUTO_H__

typedef struct {
    float x;
    float y;
    int yaw;

    float angle;
    float speed;

    float target_x;
    float target_y;
} CarState;

void *startMotiCtrlByAuto(void *);


#endif // __START_MOTI_CTRL_BY_AUTO_H__
