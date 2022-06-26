#ifndef _MATH_CALC_H
#define _MATH_CALC_H

typedef struct {
  float x_;
  float y_;
} Point2d;

#define SQ(x) ((x)*(x))
#define PI 3.14159265L
#define Degree2Rad	0.01745329251994327813L	//角度转弧度
#define Rad2Degree	57.29577951308237971L		//弧度转角度

//符号函数
int sign(double x);

int nearly_equal(double x1, double x2);

double round(double r);

double distance ( double x1, double y1, double x2, double y2 );

//计算两点间距离
double dist(Point2d *point1, Point2d *point2);

//计算两点角度
double angel(Point2d *point1, Point2d *point2);

//点到直线的距离
double point_to_line(Point2d *point, Point2d *line_point_1, Point2d *line_point_2);

#endif //_MATH_CALC_H
