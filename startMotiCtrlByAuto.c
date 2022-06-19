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

extern pthread_mutex_t g_gtwy_info_mutex;
extern _GatewayInfo g_gateway_info; // 网关

extern _RV3399Info g_rv3399_info; // 3399摄像头-陀螺仪

extern pthread_mutex_t g_uwb_mutex;
extern _UwbData g_uwb_loc; // uwb数据

extern int g_a_suspend; //手柄开启自动模式flag

//滤波器参数
Kalman_TypeDef KF_X; // UWB_X
Kalman_TypeDef KF_Y; // UWB_Y

// UWB滤波器使用
void kf_init(void){
  Kalman_Init(&KF_X,1e-6,0.002);
  Kalman_Init(&KF_Y,1e-6,0.002);
}

//对UWB的X Y坐标分别使用初始化的KF进行滤波
//除初始值0外 当x y同时为0时说明未找到uwb数据 维持上次的uwb数据
//当10次全为0后 输出0 并打印WARN日志
_UwbData filter_debounce(_UwbData now_uwb) {
  _UwbData data;

  static _UwbData last_data = {0};
  static int count = 0;//纪录数据为0次数

  if(now_uwb.x != 0 && now_uwb.y != 0){
    data.x = Kalman_Filter(&KF_X, now_uwb.x);
    data.y = Kalman_Filter(&KF_Y, now_uwb.y);
    count = 0;
  }else{
    data.x = last_data.x;
    data.y = last_data.y;
    count ++;
  }

  if(count > 10){
    data.x = 0;
    data.y = 0;
    Log(WARN,"%d times the uwb data is zero",count);
  }

  last_data = data;

  return data;
}

//保存uwb数据到文件
//测试滤波器
void save_uwb_data(_UwbData data, char *filename, int namesize) {
  FILE *fp = NULL;
  char file_name[128] = {0};
  char buf[64] = {0};

  memcpy(file_name, filename, namesize);
  sprintf(buf, "%.3f,%.3f\n", data.x, data.y);

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
 * @param init
 * @param now
 * @return int
 **/
int calc_gyro(int init, int now) {

  int theta = 0;
  theta = now - init;

  if (theta <= -180) {
    theta += 360;
  } else if (theta >= 180) {
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
// double path_track(Point2d now, Point2d start, Point2d end,float gyro) {

//   const int v = 10; //速度

//   // To check whether the point is left or right of the desired path
//   double d = 0;
//   double tmp = (now.x_ - start.x_) * (end.y_ - start.y_) -
//                (now.y_ - start.y_) * (end.x_ - start.x_);

//   if (tmp < 0) //点now在start和end确立的直线的左边
//     d = point_to_line(&now, &start, &end); // left
//   else
//     d = -point_to_line(&now, &start, &end); // right

//   double si_p = atan2((end.y_ - start.y_), (end.x_ - start.x_)); // si desired

//   const int db = 2;//限制偏移尺寸?
//   const int q2 = 1;
//   double q1 = sqrt(fabs(db / ((db - d) + 1e-10)));
//   double vd = v * sin(gyro - si_p); // d_dot
//   double si_dot = -(q1 * d + sqrt(2 * q1 + q2 * q2) * vd);

//   si_dot *= sign(tmp);

//   if (fabs(si_dot) > 30) //过大限制
//     si_dot = 30;
//   // if (fabs(si_dot) < 2)//过小限制
//   //   si_dot = 0;

//   return si_dot;
// }

double path_track(Point2d now, Point2d start, Point2d end) {

  const int v = 10; //速度

  // To check whether the point is left or right of the desired path
  double d = 0;
  double tmp = (now.x_ - start.x_) * (end.y_ - start.y_) -
               (now.y_ - start.y_) * (end.x_ - start.x_);

  if (tmp < 0) //点now在start和end确立的直线的左边
    d = point_to_line(&now, &start, &end); // left
  else
    d = -point_to_line(&now, &start, &end); // right


  //目标直线+-5cm内为死区，无偏移动作
  if(fabs(d) < 0.05)
    d = 0;

  //根据当前点到起点，终点确立的直线距离 确立后轮转向角
  //p控制系数20
  float si_dot = 20 * d;

  //过大限制
  if (si_dot > 20)
    si_dot = 20;
  if (si_dot < -20)
    si_dot = -20;

  return si_dot;
}

void *startMotiCtrlByAuto(void *args) {

  _GatewayInfo gwInfo;
  _UwbData uwb_now;

  Point2d start = {3, 0};
  Point2d end = {3, 20};
  Point2d now = start;

  float angle = 0;
  float speed = 30;

  float gyro_init,gyro_now;

  kf_init();//KF初始化

  while (1) {
    pthread_mutex_lock(&(g_gtwy_info_mutex));
    gwInfo = g_gateway_info;
    pthread_mutex_unlock(&(g_gtwy_info_mutex));

    pthread_mutex_lock(&(g_uwb_mutex));
    uwb_now = filter_debounce(g_uwb_loc);
    pthread_mutex_unlock(&(g_uwb_mutex));

    gyro_init = parse_gyro(g_rv3399_info);

    // test
    // Log(DEBUG, "before filter x=%.2f,y=%.2f", g_uwb_loc.x, g_uwb_loc.y);
    // Log(DEBUG, "after filter x=%.2f,y=%.2f", uwb_now.x, uwb_now.y);
    // save_uwb_data(g_uwb_loc,"/userdata/media/test/appcar/UwbDataRaw.csv",sizeof("/userdata/media/test/appcar/UwbDataRaw.csv"));
    // save_uwb_data(uwb_now,"/userdata/media/test/appcar/UwbData.csv",sizeof("/userdata/media/test/appcar/UwbDataK.csv"));
    // label_cam_send_start(5, 63);
    Log(DEBUG, "symbol=%d,theta_gyro=%d,theta_cam=%d", g_rv3399_info.symbol,g_rv3399_info.theta_gyro, g_rv3399_info.theta_cam);
    Log(DEBUG, "theta_gyro=%d", parse_gyro(g_rv3399_info));
    // label_cam_send_end();
    // test end

    if (g_a_suspend) // Gpd开启自动模式
    {
      now.x_ = uwb_now.x;
      now.y_ = uwb_now.y;

      if (dist(&now, &end) > 1) { //未到达终点

        // path_plan();
        gyro_now = parse_gyro(g_rv3399_info);
        angle = path_track(now, start, end);

        Log(DEBUG, "calc_angle=%.2f", angle);

        send_control_cmd(angle, speed); //发送控制命令
      } else {                          //到达终点 停车
        send_control_cmd(0, 0);         //发送控制命令
      }

        gyro_init = gyro_now;
    }
    usleep(50000); // 50ms
  }
  return NULL;
}
