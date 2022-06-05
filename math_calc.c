#include <math.h>
#include "math_calc.h"

#define SQ(x) ((x)*(x))
#define PI 3.14159265L
#define Degree2Rad	0.01745329251994327813L	//角度转弧度
#define Rad2Degree	57.29577951308237971L		//弧度转角度

//符号函数
int sign(double x) {
  if (x > 0)
    return 1;
  else if (x < 0)
    return -1;
  else
    return 0;
}

//求num平方根的倒数
//等价于 1.0/sqrt(num)
double Quick_rsqrt(double num)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5f;
    x2 = (float)num * 0.5f;
    y = (float)num;
    i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (threehalfs - (x2 * y * y));
    y = y * (threehalfs - (x2 * y * y));
    return y;
}

//计算两点间距离
double dist(Point2d *point1, Point2d *point2) {
  return sqrt(SQ(point2->x_ - point1->x_)  + SQ(point2->y_ - point1->y_) );
}

//计算两点角度
double angel(Point2d *point1, Point2d *point2) {
  return atan2(point2->y_ - point1->y_, point2->x_ - point1->x_) * Rad2Degree;
}

//计算点到直线之间的距离
double point_to_line(Point2d *point, Point2d *line_point_1, Point2d *line_point_2) {
  //处理斜率不存在的情况
  if (line_point_1->x_ == line_point_2->x_) {
    return fabs(point->x_ - line_point_1->x_);
  }
  double k, b; // y=kx+b
  k = (line_point_2->y_ - line_point_1->y_) /
      (line_point_2->x_ - line_point_1->x_);
  b = line_point_2->y_ - line_point_2->x_ * k;

  double res = fabs(k * point->x_ - point->y_ + b) / sqrt(k * k + 1);

  return res;
}
