#ifndef _MATH_CALC_H
#define _MATH_CALC_H

typedef struct {
  float x_;
  float y_;
} Point2d;

//符号函数
int sign(double x);

//计算两点间距离
double dist(Point2d *point1, Point2d *point2);

//计算两点角度
double angel(Point2d *point1, Point2d *point2);

//点到直线的距离
double point_to_line(Point2d *point, Point2d *line_point_1, Point2d *line_point_2);

#endif //_MATH_CALC_H
