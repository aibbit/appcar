#include <math.h>
#include "math_calc.h"
#include "frenet.h"
/**
 * @brief 找到vec[]上距离x y最近的点
 * @warning 端点可能有bug
 * @return int vec位置i
**/
int ClosestWayPoint ( Point2d now, const Point2d vec[], const int size){

    double ClosestLen = 100000; //large number
    int closestwaypoint = 0;

    for ( int i = 0; i < size; i++ ) {
        double map_x = vec[i].x_;
        double map_y = vec[i].y_;

        double dist = distance( now.x_, now.y_, map_x, map_y );
        if ( dist < ClosestLen ) {
            ClosestLen = dist;
            closestwaypoint = i;
        }
    }

    return closestwaypoint;
}
/**
 * @brief 找到vec[]上距离x y次最近的点
 * @warning 端点可能有bug
**/
int NextWayPoint ( Point2d now, const Point2d vec[], const int size ){

    int closestwaypoint = ClosestWayPoint ( now, vec ,size);
    int nextwaypoint = 0;

    if(closestwaypoint == size){
        nextwaypoint = closestwaypoint - 1;
    }
    else if(closestwaypoint == 0){
        nextwaypoint = closestwaypoint + 1;
    }
    else{
        double dist1 = distance(now.x_, now.y_, vec[closestwaypoint + 1].x_, vec[closestwaypoint + 1].y_);
        double dist2 = distance(now.x_, now.y_,vec[closestwaypoint - 1].x_, vec[closestwaypoint - 1].y_);
        if(dist1 < dist2)
            nextwaypoint = closestwaypoint + 1;
        else
            nextwaypoint = closestwaypoint - 1;
    }
    return nextwaypoint;
}

/**
 * @brief 将某点笛卡尔坐标x y转化为Frenet坐标s d
 *
 * @param now 当前点x y
 * @param vec 参考路径
 * @param size vec大小
 * @param frenet_vec Frenet坐标s d
**/
void Cartesian2Frenet( Point2d now, const Point2d vec[],const int size, Point2d *frenet_vec){

    double frenet_s,frenet_d;

    int next_wp = NextWayPoint ( now, vec ,size);
    int prev_wp = ClosestWayPoint ( now, vec ,size);

    double n_x = vec[next_wp].x_ - vec[prev_wp].x_;
    double n_y = vec[next_wp].y_ - vec[prev_wp].y_;
    double x_x = now.x_ - vec[prev_wp].x_;
    double x_y = now.y_ - vec[prev_wp].y_;

    //Frenet下d分量
    // find the projection of x onto n
    double proj_norm = ( x_x * n_x + x_y * n_y ) / ( n_x * n_x + n_y * n_y );
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;
    frenet_d = distance ( x_x, x_y, proj_x, proj_y );

    //d分量正负判断

    //FIXME 正负号有时不正确 --修复
    // double center_x =  buffer[prev_wp].x;
    // double center_y =  buffer[prev_wp].y;
    // double centerToPos = distance ( center_x, center_y, x_x, x_y );
    // double centerToRef = distance ( center_x, center_y, proj_x, proj_y );

    // if ( centerToPos <= centerToRef ) {
    //     frenet.d *= -1;
    // }

    double fd = (now.x_ - vec[prev_wp].x_) * (vec[next_wp].y_ - vec[prev_wp].y_) - \
     (now.y_ - vec[prev_wp].y_) * (vec[next_wp].x_ - vec[prev_wp].x_);

    if ( sign(fd) > 0 ) {
        frenet_d *= -1;
    }

    //Frenet下s分量
    for ( int i = 0; i <= prev_wp; i++ ) {
        frenet_s += distance ( vec[i].x_, vec[i].y_, vec[i+1].x_, vec[i+1].y_ );
    }
    frenet_s += distance ( 0, 0, proj_x, proj_y );

    //return
    frenet_vec -> x_ = frenet_s;
    frenet_vec -> y_ = frenet_d;
}

/*
#include <stdio.h>
int main(void){
    //Frenet test
    // MAP_Cartesian buff1[10]={{0,0},{10,-4.0},{20.5,1},{30,6.5},{40.5,8},{50,10},{60,6},{85,8},{100,2},{80,8}};
    Point2d buff1[10]={{0,0},{1,1},{2,2},{3,3},{4,4},{6,6},{7,7},{8,8},{9,9},{10,10}};
    // double x_now = 28;
    // double y_now = 10;
    Point2d buff3 = {0};
    Point2d now = {5,5};

    Cartesian2Frenet(now,buff1,10,&buff3);
    printf("%f\t%f\n",buff3.x_,buff3.y_);

    return 0;
}
*/
