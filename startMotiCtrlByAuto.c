#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

#include "startGamepadCapData.h"
#include "startLocalNetCapData.h"
#include "startUwbCapData.h"
#include "kalman_filter.h"
#include "log.h"
#include "math_calc.h"
#include "utils.h"

#define CMD_SIZE 6

extern _MySocketInfo g_gatewaySocket;
extern _GatewayInfo g_gateway_info; //网关
extern pthread_mutex_t g_gtwy_info_mutex;

extern _UwbData g_uwb_loc;//uwb数据

extern int g_a_suspend;//手柄开启自动模式flag

// static char turn_left(float angle) { return (ch); }
// static char turn_right(float angle) { return (ch); }
// static char back(float speed) { return (ch); }
// static char straight(float speed) { return (ch); }

//滤波器
_UwbData filter_debounce(_UwbData now_uwb) {
  _UwbData data;
  Kalman1_TypeDef KF_X; // KF滤波器
  Kalman1_TypeDef KF_Y; // KF滤波器
  Kalman_Init_X(&KF_X);
  Kalman_Init_Y(&KF_Y);

  data.x = Kalman1_Filter(&KF_X, now_uwb.x);
  data.y = Kalman1_Filter(&KF_Y, now_uwb.y);

  return data;
}

//根据路径点计算偏转角度
double path_track(Point2d now, Point2d start, Point2d end, double theta) {

  const int v = 10;

  // To check whether the point is left or right of the desired path
  double d = 0;
  double tmp = (now.x_ - start.x_) * (end.y_ - start.y_) -
               (now.y_ - start.y_) * (end.x_ - start.x_);

  if (tmp < 0)
    d = point_to_line(&now, &start, &end); // left
  else
    d = -point_to_line(&now, &start, &end); // right

  const int db = 4;
  double q1 = sqrt(fabs(db / ((db - d) + 1e-10)));

  double si_dot;

  if (tmp < 0)
    si_dot = -(q1 * d);
  else
    si_dot = q1 * d;

  if (fabs(si_dot) > 30)
    si_dot = 30;

  return si_dot;
}

void *startMotiCtrlByAuto(void *args) {

  char cmd[CMD_SIZE] = {0xCC, 0xCC, 0x01, 0x00, 0x00, 0x99};
  _GatewayInfo gwInfo;
  _UwbData uwb_now;

  Point2d start = {0, 3};
  Point2d end = {40, 3};
  Point2d now = start;

  float angle = 0;

  while (1) {
    pthread_mutex_lock(&(g_gtwy_info_mutex));
    gwInfo = g_gateway_info;
    pthread_mutex_unlock(&(g_gtwy_info_mutex));

    uwb_now = filter_debounce(g_uwb_loc);
    //Log(DEBUG, "before filter x=%.2f,y=%.2f", g_uwb_loc.x, g_uwb_loc.y);
    //Log(DEBUG, "after filter x=%.2f,y=%.2f", uwb_now.x, uwb_now.y);

    if (g_a_suspend) // Gpd开启自动模式
    {
      now.x_ = uwb_now.x;
      now.y_ = uwb_now.y;

      if (dist(&now, &end) > 1) {

        // path_plan();
        angle = path_track(now, start, end, 0);
        Log(DEBUG,"calc_angle=%.2f",angle);

        //发送控制命令
        cmd[2] = 1;
        cmd[3] = (char)-0x30; // move angle
        cmd[4] = (char)-0x30;  // move speed
        cmd[5] = cmdSum(cmd, (CMD_SIZE - 1));
        Log(DEBUG,"cmd=%x %x %x %x %x",cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5]);

        pthread_mutex_lock(&(g_gtwy_info_mutex));
        sendInfoToLocalNet(g_gatewaySocket, cmd, CMD_SIZE);
        pthread_mutex_unlock(&(g_gtwy_info_mutex));
      } else {
        //发送控制命令
        cmd[2] = 1;
        cmd[3] = 0; // move angle
        cmd[4] = 0; // move speed
        cmd[5] = cmdSum(cmd, (CMD_SIZE - 1));

        pthread_mutex_lock(&(g_gtwy_info_mutex));
        sendInfoToLocalNet(g_gatewaySocket, cmd, CMD_SIZE);
        pthread_mutex_unlock(&(g_gtwy_info_mutex));
      }
    }
    usleep(50000);
  }
  return NULL;
}
