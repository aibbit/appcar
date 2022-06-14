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
extern _RV3399Info g_rv3399_info;

extern _UwbData g_uwb_loc; // uwb数据

extern int g_a_suspend; //手柄开启自动模式flag

// UWB滤波器使用
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

//保存uwb数据到文件
//测试滤波器
void save_uwb_data(_UwbData data1, char *filename, int namesize) {
  FILE *fp = NULL;
  char file_name[128] = {0};
  char buf[1024] = {0};

  memcpy(file_name, filename, namesize);
  sprintf(buf, "%.3f,%.3f\n", data1.x, data1.y);

  if ((fp = fopen(file_name, "a+")) != NULL) {
    fprintf(fp, "%s", buf);
    fclose(fp);
  }
}


/**
 * @brief 解析3399中陀螺仪数据
 *
 * @param g_rv3399_info
 * @return int -180到180的绝对角度
 **/
int parse_gyro(_RV3399Info g_rv3399_info) {

  if (g_rv3399_info.symbol == 0x00 || g_rv3399_info.symbol == 0x01) {
    //陀螺仪为正数
    return (g_rv3399_info.theta_gyro);
  } else if (g_rv3399_info.symbol == 0x02 || g_rv3399_info.symbol == 0x03) {
    //陀螺仪为负数
    return (-1 * g_rv3399_info.theta_gyro);
  } else {
    Log(WARN, "symbol error");
  }
   return 0;
}

/**
 * @brief 根据前后陀螺仪数值，计算差值，正值->顺时针转过theta
 *
 * @param first
 * @param now
 * @return float
 **/
float calc_gyro(float init, float now) {

  float theta = 0;
  theta = now - init;

  if (theta <= -180) {
    theta += 360;
  } else if (theta >= -180) {
    theta -= 360;
  } else {
    theta += 0;
  }

  return theta;
}

/**
 * @brief 根据路径点计算偏转角度
 *
 * @param now 当前点
 * @param start 起始点
 * @param end 终点
 * @param gyro 陀螺仪偏角
 * @return double 计算的转角
**/
double path_track(Point2d now, Point2d start, Point2d end,float gyro) {

  const int v = 10; //速度

  // To check whether the point is left or right of the desired path
  double d = 0;
  double tmp = (now.x_ - start.x_) * (end.y_ - start.y_) -
               (now.y_ - start.y_) * (end.x_ - start.x_);

  if (tmp < 0) //点now在start和end确立的直线的左边
    d = point_to_line(&now, &start, &end); // left
  else
    d = -point_to_line(&now, &start, &end); // right

  double si_p = atan2((end.y_ - start.y_), (end.x_ - start.x_)); // si desired

  const int db = 2;//限制偏移尺寸?
  const int q2 = 1;
  double q1 = sqrt(fabs(db / ((db - d) + 1e-10)));
  double vd = v * sin(gyro - si_p); // d_dot
  double si_dot = -(q1 * d + sqrt(2 * q1 + q2 * q2) * vd);

  si_dot *= sign(tmp);

  if (fabs(si_dot) > 30) //过大限制
    si_dot = 30;
  // if (fabs(si_dot) < 2)//过小限制
  //   si_dot = 0;

  return si_dot;
}

void *startMotiCtrlByAuto(void *args) {

  char cmd[CMD_SIZE] = {0xCC, 0xCC, 0x01, 0x00, 0x00, 0x99};
  _GatewayInfo gwInfo;
  _UwbData uwb_now;

  Point2d start = {4, 0};
  Point2d end = {4, 5};
  Point2d now = start;

  float angle = 0;
  float speed = 30;

  float gyro_init,gyro_now;

  while (1) {
    pthread_mutex_lock(&(g_gtwy_info_mutex));
    gwInfo = g_gateway_info;
    pthread_mutex_unlock(&(g_gtwy_info_mutex));

    uwb_now = filter_debounce(g_uwb_loc);
    gyro_init = parse_gyro(g_rv3399_info);
    // test
    // Log(DEBUG, "before filter x=%.2f,y=%.2f", g_uwb_loc.x, g_uwb_loc.y);
    // Log(DEBUG, "after filter x=%.2f,y=%.2f", uwb_now.x, uwb_now.y);
    // save_uwb_data(g_uwb_loc,"UwbDataRaw.csv",sizeof("UwbDataRaw.csv"));
    // save_uwb_data(uwb_now,"UwbData.csv",sizeof("UwbDataK.csv"));
    label_cam_send_start(5, 63);
    Log(DEBUG, "symbol=%d,theta_gyro=%d,theta_cam=%d", g_rv3399_info.symbol,g_rv3399_info.theta_gyro, g_rv3399_info.theta_cam);
    Log(DEBUG, "theta_gyro=%d", parse_gyro(g_rv3399_info));
    label_cam_send_end();

    // test end

    if (g_a_suspend) // Gpd开启自动模式
    {
      now.x_ = uwb_now.x;
      now.y_ = uwb_now.y;

      if (dist(&now, &end) > 1) { //未到达终点

        // path_plan();
        gyro_now = parse_gyro(g_rv3399_info);
        angle = path_track(now, start, end,calc_gyro(gyro_init,gyro_now));
        gyro_init = gyro_now;
        Log(DEBUG, "calc_angle=%.2f", angle);

        send_control_cmd(angle, speed); //发送控制命令
      } else {                          //到达终点 停车
        send_control_cmd(0, 0);         //发送控制命令
      }
    }
    usleep(100000); // 100ms
  }
  return NULL;
}
