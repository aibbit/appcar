#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "math_calc.h"
#include "filter.h"
#include "frenet.h"
#include "log.h"
#include "startGamepadCapData.h"
#include "startLocalNetCapData.h"
#include "startMotiCtrlByAuto.h"
#include "startUwbCapData.h"
#include "utils.h"

extern pthread_mutex_t g_gtwy_info_mutex;
extern _GatewayInfo g_gateway_info;  // 网关

extern _RV3399Info g_rv3399_info;  // 3399摄像头-陀螺仪

extern pthread_mutex_t g_uwb_mutex;
extern _UwbData g_uwb_loc;  // uwb数据

extern int g_a_suspend;  //手柄开启自动模式flag

extern _ComGpdKey g_gpd_key;
extern pthread_mutex_t g_gpd_mutex;

//滤波器参数
Kalman_TypeDef KF_X;  // UWB_X
Kalman_TypeDef KF_Y;  // UWB_Y

Limit_TypeDef LMF_X;  // UWB_X
Limit_TypeDef LMF_Y;  // UWB_Y

Lag_TypeDef LAG_X;    // UWB_X
Lag_TypeDef LAG_Y;    // UWB_Y

// 滤波器初始化
void filter_init(void) {
  Kalman_Init(&KF_X, 1e-6, 1e-4);
  Kalman_Init(&KF_Y, 1e-6, 1e-4);

  limit_filter_init(&LMF_X, 10);
  limit_filter_init(&LMF_Y, 10);

  lag_filter_init(&LAG_X, 0.95);
  lag_filter_init(&LAG_Y, 0.95);
}

//对UWB的X Y坐标分别使用滤波器进行滤波
//除初始值0外 当x y同时为0时说明未找到uwb数据 维持上次的uwb数据
_UwbData filter_debounce(_UwbData now_uwb) {
  _UwbData data = {0};

  static _UwbData last_data = {0};
  static int count = 0;  //纪录数据为0次数

  // 除0 -> 限幅 -> 一阶滞后 -> 卡尔曼
  // 数据不符合维持之前状态
  if (is_nearly_equal(now_uwb.x, 0) && is_nearly_equal(now_uwb.y, 0)) {
    data.x = last_data.x;
    data.y = last_data.y;
    count++;
    // printf("%d\t",count);
  } else {
    data.x = limit_filter(&LMF_X, now_uwb.x);
    data.y = limit_filter(&LMF_Y, now_uwb.y);
    // data.x = limit_filter_1(now_uwb.x,LMF_X.limit);

    // data.x = lag_filter(&LAG_X, data.x);
    // data.y = lag_filter(&LAG_Y, data.y);
    // data.x = lag_filter_1(data.x,0.95);

    data.x = Kalman_Filter(&KF_X, data.x);
    data.y = Kalman_Filter(&KF_Y, data.y);
    count = 0;
  }

  if (count > 100) {
    // 当100次全为0后 输出0 并打印ERROR日志
    data.x = 0;
    data.y = 0;
    Log(ERROR, "no uwb signal");
  } else if (count > 10) {
    // 当10次全为0后 输出0 并打印WARN日志
    data.x = 0;
    data.y = 0;
    Log(WARN, "%d times the uwb data is zero", count);
  }

  last_data = data;
  return data;
}

//保存gyro数据到文件
void save_gyro_data(int data, char *filename, int namesize) {
  FILE *fp = NULL;
  char file_name[128] = {0};
  char buf[64] = {0};

  memcpy(file_name, filename, namesize);
  sprintf(buf, "%d\n", data);

  if ((fp = fopen(file_name, "a+")) != NULL) {
    fprintf(fp, "%s", buf);
    fclose(fp);
  }
}

//保存行驶状态到文件
void save_state_data(CarState data, char *filename, int namesize) {
  FILE *fp = NULL;
  char file_name[128] = {0};
  char buf[128] = {0};

  memcpy(file_name, filename, namesize);
  sprintf(buf, "%.3f,%.3f,%d,%.3f,%.3f,%.3f,%.3f\n", data.x, data.y, data.yaw,
          data.angle, data.speed, data.target_x, data.target_y);

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
  return 0xFF;  // 返回 错误
}


// 根据前后陀螺仪数值,计算差值
int calc_gyro(int init, int now) {
  int theta = 0;
  theta = now - init;

  // FIXME 归一化
  if (theta <= -180) {
    theta += 360;
  } else if (theta >= 180) {
    theta -= 360;
  } else {
    theta += 0;
  }

  return theta;
}

// TODO 根据陀螺仪原地转相应角度 不能自动停车
void turn_angle(int angle) {
  int gyro_init = parse_gyro(g_rv3399_info);
  int gyro_now = gyro_init;
  int error = 0;

  while (1) {
    gyro_init = gyro_now;
    gyro_now = parse_gyro(g_rv3399_info);
    error += calc_gyro(gyro_init, gyro_now);

    send_control_cmd(90, 30);

    if (error >= abs(angle)) break;
  }

  send_control_cmd(0, 0);
}

// 在车头朝向东时初始化(x轴正方向)
// 东 0度 北 90度
int gyro_init = 0;   //陀螺仪清零值
volatile int gyro_now = 0;  //陀螺仪当前值
int get_gyro(void) { return calc_gyro(gyro_init, gyro_now); }

//根据起点和终点生成直线地图
//插值函数简单替代
//输入start,end,size
//输出map[]
void generate_path(const Point2d start, const Point2d end, const int size,
                   Point2d map[]) {
  double k = 0;
  double step = 0;
  double max_length = 0;
  if (start.y_ == end.y_) {
    max_length = end.x_ - start.x_;
    step = max_length / (size - 1);
    for (int m = 0; m < size; m++) {
      map[m].x_ = start.x_ + step * m;
      map[m].y_ = start.y_;
    }
  } else if (start.x_ == end.x_) {
    max_length = end.y_ - start.y_;
    step = max_length / (size - 1);
    for (int m = 0; m < size; m++) {
      map[m].x_ = start.x_;
      map[m].y_ = start.y_ + step * m;
    }
  } else {
    k = (end.y_ - start.y_) / (end.x_ - start.x_);
    // printf("k = %f\n, ", k);
    max_length = end.x_ - start.x_;
    // printf("max_length = %f\n, ", max_length);
    step = max_length / (size - 1);
    // printf("step = %f\n, ", step);

    for (int m = 0; m < size; m++) {
      map[m].x_ = start.x_ + step * m;
      map[m].y_ = k * step * m + start.y_;
    }
  }
}

void generate_map(const Point2d point[], const int point_size, Point2d map[],
                  const int map_size) {
  Point2d tmp[100] = {0};

  for (int i = 0; i < point_size - 1; i++) {
    generate_path(point[i], point[i + 1], 100, tmp);
    // 数组越界检查
    if (i * 100 > map_size) {
      Log(WARN, "array overflow");
      break;
    }
    memcpy(map + i * 100, tmp, 100 * sizeof(Point2d));
  }
  Log(INFO, "generate map suceessed");
}

// 引入横向偏差
// @deprecated 废弃
double calc_steering_angle_test(const double L, const double now_x, const double now_y,
                           const double now_yaw, const double target_x,
                           const double target_y) {
  double delta_x = target_x - now_x;
  double delta_y = target_y - now_y;
  // 旋转矩阵展开
  double x = delta_x * cos(now_yaw) - delta_y * sin(now_yaw);
  double y = delta_x * sin(now_yaw) + delta_y * cos(now_yaw);
  double alpha = atan2(y, x);
  double angle_error =
      atan2(2.0 * L * sin(alpha), sqrt(delta_x * delta_x + delta_y * delta_y));
  return angle_error;
}

// 根据横向误差计算角度误差项
// 输出角度误差(弧度)
double calc_steering_angle(const double now_x, const double now_y,
                           const double now_yaw, const double target_x,
                           const double target_y) {
  const float k = 2.0f; // 参数 与速度有关 自行调节
  double delta_x = target_x - now_x;
  double delta_y = target_y - now_y;

  // 根据百度Apollo 计算横向误差
  double lat_error = delta_y * cos(now_yaw) - delta_x * sin(now_yaw);
  double angle_error = atan2(k * lat_error, 1);

  // TODO 测试误差大小
  // test
  Log(DEBUG,"target_x=%.2f,target_y=%.2f,now_x=%.2f,now_y=%.2f",target_x,target_y,now_x,now_y);
  Log(DEBUG,"lat_error=%.3f",lat_error);
  Log(DEBUG,"angle_error=%.3f",angle_error * Rad2Degree);
  // test end

  return angle_error;
}

// 纯跟踪 路径跟踪算法
double pure_pursuit_control(Point2d now, const Point2d map[], const int size) {
  const float Kv = 0.01f;  // 前视距离系数
  const float Ld0 = 0.8f;  // 预瞄距离的下限值
  const float L = 1.0f;    // 车辆轴距

  // 搜索最临近的路点
  int ind = 0;
  ind = ClosestWayPoint(now, map, size);

  // 从该点开始向后搜索，找到与预瞄距离最相近的一个轨迹点
  float L_steps = 0.0f;  // 参考轨迹上几个点的临近距离


  float now_v = 30.0f;   // 约0.3 m/s
  float Lf = 0;
  Lf = Kv * now_v + Ld0;

  while (Lf > L_steps && (ind + 1) < size) {
    L_steps +=
        distance(map[ind].x_, map[ind].y_, map[ind + 1].x_, map[ind + 1].y_);
    ind += 1;
  }

  float tx = 0, ty = 0;  //待追踪目标点
  if (ind < size) {
    tx = map[ind].x_;
    ty = map[ind].y_;
  } else {
    tx = map[size - 1].x_;
    ty = map[size - 1].y_;
    ind = size - 1;
  }

  double alpha = 0, delta = 0;
  float now_yaw = 0;
  now_yaw = get_gyro() * Degree2Rad;  //航向角(绝对值)
  alpha = atan2(ty - now.y_, tx - now.x_) - now_yaw;
  delta = atan2(2.0 * L * sin(alpha) / Lf, 1.0);

  //引入横向偏差
  double angle_error = 0.0;
  angle_error = calc_steering_angle(now.x_,now.y_,now_yaw,tx,ty);
  delta += 0.3 * angle_error; //0.3为参数 自行调节

  // TODO 引入前馈控制

  delta *= Rad2Degree;  //转换为角度
  delta = -1 * delta;   //计算方向与命令方向相反
  // TODO 加入速度控制
  // now_v = 36 - 0.4 * fabs(delta);

  // 直接发送控制命令
  send_control_cmd(delta, now_v);

  // test
  CarState now_state = {0};
  now_state.x = now.x_;
  now_state.y = now.y_;
  now_state.yaw = get_gyro();
  now_state.angle = delta;
  now_state.speed = now_v;
  now_state.target_x = tx;
  now_state.target_y = ty;

  Log(DEBUG, "x=%.2f,y=%.2f,yaw=%d,angle=%.2f,speed=%.2f", now_state.x,
      now_state.y, now_state.yaw, now_state.angle, now_state.speed);
  Log(DEBUG, "target_x=%.2f,target_y=%.2f", tx, ty);
  save_state_data(now_state, "/userdata/media/test/appcar/CarState.csv",
                  sizeof("/userdata/media/test/appcar/CarState.csv"));
  // test end

  return delta;
}

// @deprecated 废弃
double path_track(Point2d now, Point2d start, Point2d end) {
  const int v = 10;  //速度

  // To check whether the point is left or right of the desired path
  double d = 0;
  double tmp = (now.x_ - start.x_) * (end.y_ - start.y_) -
               (now.y_ - start.y_) * (end.x_ - start.x_);

  if (tmp < 0)  //点now在start和end确立的直线的左边
    d = point_to_line(&now, &start, &end);  // left
  else
    d = -point_to_line(&now, &start, &end);  // right

  //目标直线+-5cm内为死区,无偏移动作
  // if (fabs(d) < 0.05)
  //   d = 0;

  //根据当前点到起点,终点确立的直线距离 确立后轮转向角
  // p控制系数20
  float si_dot = 20 * d;

  //过大限制
  if (si_dot > 20) si_dot = 20;
  if (si_dot < -20) si_dot = -20;

  return si_dot;
}

void *startMotiCtrlByAuto(void *args) {
  _GatewayInfo gwInfo;
  _UwbData uwb_now;
  _ComGpdKey gpd_info;

  Point2d start = {20, 75};
  Point2d end = {22, 25};
  Point2d now = start;

  float angle = 0;
  float speed = 35;

  filter_init();  // 滤波器参数初始化

  // TODO 生成地图
  const Point2d point[] = {{20, 75}, {20, 25}, {6, 25}, {6, 6},
                           {33, 6},  {33, 25}, {22, 25}};
  const int map_size = (sizeof(point) / sizeof(Point2d) - 1) * 100;
  Point2d map[map_size];
  // generate_path(start, end, 500, map);
  generate_map(point, sizeof(point) / sizeof(Point2d), map, map_size);

  // CarState now_state = {0};

  while (1) {
    pthread_mutex_lock(&(g_gtwy_info_mutex));
    gwInfo = g_gateway_info;
    pthread_mutex_unlock(&(g_gtwy_info_mutex));

    pthread_mutex_lock(&(g_uwb_mutex));
    uwb_now = filter_debounce(g_uwb_loc);
    pthread_mutex_unlock(&(g_uwb_mutex));

    pthread_mutex_lock(&(g_gpd_mutex));
    gpd_info = g_gpd_key;
    pthread_mutex_unlock(&(g_gpd_mutex));

    if (gpd_info.key == 0x0004) {  // R2按键 陀螺仪清零
      gyro_init = parse_gyro(g_rv3399_info);
    }
    gyro_now = parse_gyro(g_rv3399_info);

    // test
    Log(DEBUG, "before filter x=%.2f,y=%.2f", g_uwb_loc.x, g_uwb_loc.y);
    Log(DEBUG, "after filter x=%.2f,y=%.2f", uwb_now.x, uwb_now.y);
    save_uwb_data(g_uwb_loc, "/userdata/media/test/appcar/UwbDataRaw.csv",
                  sizeof("/userdata/media/test/appcar/UwbDataRaw.csv"));
    save_uwb_data(uwb_now, "/userdata/media/test/appcar/UwbData.csv",
                  sizeof("/userdata/media/test/appcar/UwbData.csv"));
    // label_cam_send_start(5, 63);
    Log(DEBUG, "gyro_init=%d", gyro_init);
    Log(DEBUG, "theta_gyro=%d", get_gyro());
    // Log(DEBUG,  "symbol=%x,theta_gyro=%x,theta_cam=%x",g_rv3399_info.symbol,g_rv3399_info.theta_gyro,g_rv3399_info.theta_cam);   //rv3399测试
    // Log(DEBUG, "1:%d,2:%d,3:%d,4:%d",gwInfo.ultra1,gwInfo.ultra2,gwInfo.ultra3,gwInfo.ultra4);   //超声波测试
    // Log(DEBUG, "5:%d,6:%d,7:%d,8:%d",gwInfo.ultra5,gwInfo.ultra6,gwInfo.ultra7,gwInfo.ultra8);   //超声波测试
    // save_gyro_data(get_gyro(), "/userdata/media/test/appcar/gyro.csv",
    //                sizeof("/userdata/media/test/appcar/gyro.csv"));

    // test end

    if (g_a_suspend)  // Gpd开启自动模式
    {
      now.x_ = uwb_now.x;
      now.y_ = uwb_now.y;
      // TODO 绕圈时判断是否到达终点有问题
      if (dist(&now, &end) > 1) {  //未到达终点
        pure_pursuit_control(now, map, map_size);
        // Log(DEBUG, "calc_angle=%.2f", angle);
        // send_control_cmd(angle, speed);  //发送控制命令
      } else {                           //到达终点 停车
        send_control_cmd(0, 0);          //发送控制命令
      }
    }
    usleep(50000);  // 50ms
  }                 // end while
  return NULL;
}
