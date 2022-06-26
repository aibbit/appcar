#ifndef _FRENET_H_
#define _FRENET_H_

int ClosestWayPoint ( Point2d now, const Point2d vec[], const int size);
int NextWayPoint ( Point2d now, const Point2d vec[], const int size );
void Cartesian2Frenet( Point2d now, const Point2d vec[],const int size, Point2d *frenet_vec);


#endif //_FRENET_H_
