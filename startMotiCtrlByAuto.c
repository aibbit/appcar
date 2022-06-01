#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include "startLocalNetCapData.h"
#include "startUwbCapData.h"
#include "startGamepadCapData.h"
#include "utils.h"

#define pi 3.141592
#define INFO_SIZE 6

extern int g_a_suspend;

extern _MySocketInfo g_gatewaySocket;
extern _GatewayInfo g_gateway_info;
extern pthread_mutex_t g_gtwy_info_mutex;


static char turn_left(char ch){
	return (ch);
}

static char turn_right(char ch){
	return (ch);
}

static char back(char ch){
	return (ch);
}

//////////////////////////////////

//计算两点间距离
double Compute2PointSqDist(double x1, double y1, double x2, double y2)
{
	double dTemp = 0;
	dTemp = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	return dTemp;
}
//弧度转换为角度
double conver_radian(double radian)
{
	// double angle;
	double degree;
	degree = radian * 180 / pi;
	// degree = ()angle;
	return (degree);
}


void *startMotiCtrlByAuto(void *args)
{

	char cmd[INFO_SIZE] = {0xCC, 0xCC, 0x02, 0x00, 0x01, 0x9B}; // start and no move

	while (1)
	{
		if (g_a_suspend)
		{
			_GatewayInfo gwInfo;
			pthread_mutex_lock(&(g_gtwy_info_mutex));
			gwInfo = g_gateway_info;
			pthread_mutex_unlock(&(g_gtwy_info_mutex));

			printf("auto\n");

		}
		usleep(50000);
	}
	return NULL;
}
