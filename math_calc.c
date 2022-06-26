#include <math.h>
#include "math_calc.h"

//符号函数
int sign(double x) {
  if (x > 0)
    return 1;
  else if (x < 0)
    return -1;
  else
    return 0;
}

//浮点数相等
int nearly_equal(double x1, double x2)
{
    return (x1 == x2) || (fabs((x2-x1)/x2) < 0.00001);
}

//正数四舍五入
double round(double r)
{
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

//求num平方根的倒数
//等价于 1.0/sqrt(num)
double q_rsqrt(double num)
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

//计算距离
double distance ( double x1, double y1, double x2, double y2 ){
    return sqrt ( ( x2 - x1 ) * ( x2 - x1 ) + ( y2 - y1 ) * ( y2 - y1 ) );
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

  double res = fabs(k * point->x_ - point->y_ + b) * q_rsqrt( SQ(k) + 1);

  return res;
}
