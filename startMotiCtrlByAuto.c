#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "filter.h"
#include "log.h"
#include "math_calc.h"
#include "startGamepadCapData.h"
#include "startLocalNetCapData.h"
#include "startUwbCapData.h"
#include "utils.h"

#define CMD_SIZE 6

extern _MySocketInfo g_gatewaySocket;
extern _GatewayInfo g_gateway_info; //网关
extern pthread_mutex_t g_gtwy_info_mutex;

extern _UwbData g_uwb_loc; // uwb数据

extern int g_a_suspend; //手柄开启自动模式flag

//滤波器
_UwbData filter_debounce(_UwbData now_uwb) {
  _UwbData data;
  Kalman_TypeDef KF_X; // KF滤波器
  Kalman_TypeDef KF_Y; // KF滤波器
  Kalman_Init_X(&KF_X);
  Kalman_Init_Y(&KF_Y);

  data.x = Kalman_Filter(&KF_X, now_uwb.x);
  data.y = Kalman_Filter(&KF_Y, now_uwb.y);

  return data;
}

//保存uwb数据到文件 测试滤波器
void save_uwb_data(_UwbData data1,char *filename,int namesize) {
  FILE *fp = NULL;
  char file_name[128] = {0};
  char buf[1024] = {0};

  memcpy(file_name, filename,namesize);
  sprintf(buf,"%f,%f\n",data1.x,data1.y);

  if ((fp = fopen(file_name, "a+")) != NULL) {
    fprintf(fp, "%s", buf);
    fclose(fp);
  }
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

  if (fabs(si_dot) > 30)//过大限制
    si_dot = 30;

  if (fabs(si_dot) < 2)//过小限制
    si_dot = 0;


  return si_dot;
}

void *startMotiCtrlByAuto(void *args) {

  char cmd[CMD_SIZE] = {0xCC, 0xCC, 0x01, 0x00, 0x00, 0x99};
  _GatewayInfo gwInfo;
  _UwbData uwb_now;

  Point2d start = {2, 0};
  Point2d end = {2, 10};
  Point2d now = start;

  float angle = 0;
  float speed = 30;

  while (1) {
    pthread_mutex_lock(&(g_gtwy_info_mutex));
    gwInfo = g_gateway_info;
    pthread_mutex_unlock(&(g_gtwy_info_mutex));

    uwb_now = filter_debounce(g_uwb_loc);
    Log(DEBUG, "before filter x=%.2f,y=%.2f", g_uwb_loc.x, g_uwb_loc.y);
    Log(DEBUG, "after filter x=%.2f,y=%.2f", uwb_now.x, uwb_now.y);
    save_uwb_data(g_uwb_loc,"UwbDataRaw.csv",sizeof("UwbDataRaw.csv"));
    save_uwb_data(uwb_now,"UwbDataKF.csv",sizeof("UwbDataKF.csv"));



    if (g_a_suspend) // Gpd开启自动模式
    {
      now.x_ = uwb_now.x;
      now.y_ = uwb_now.y;

      if (dist(&now, &end) > 1) { //未到达终点

        // path_plan();
        angle = path_track(now, start, end, 0);
        Log(DEBUG, "calc_angle=%.2f", angle);

        send_control_cmd(angle, speed); //发送控制命令
      } else { //到达终点 停车
        send_control_cmd(0, 0); //发送控制命令
      }
    }
    usleep(50000);
  }
  return NULL;
}
