#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <arpa/inet.h>
#include "startLocalNetCapData.h"
#include "startUwbCapData.h"
#include "startGamepadCapData.h"
#include "startMonitorPower.h"
#include "startCarByManual.h"
//#include "startCarByAuto.h"

int g_stop = 0;
int g_m_start = 1;////////1    20220307
int g_m_suspend = 0;
int g_m_stop = 0;
//int g_a_start = 0;
int g_a_start = 1;//cw
int g_a_suspend = 0;//////////0   20220307
int g_a_stop = 0;

int main(int argc, char *argv[])
{
	pthread_t car_pwr_monitor_tid, car_manual_tid, car_auto_tid;
	int err;

	startLocalNetInit();
	startComInitForUWB();
	startComInitForGamePad();

	err = pthread_create(&car_pwr_monitor_tid, NULL, startMonitorPower, NULL);
	if (0 != err)
	{
		printf("can't create startMonitorPower thread!\n");
		exit(1);
	}
	err = pthread_create(&car_manual_tid, NULL, startCarByManual, NULL);
	if (0 != err)
	{
		printf("can't create startCarByManual thread!\n");
		exit(1);
	}
	//err = pthread_create(&car_auto_tid, NULL, startCarByAuto, NULL);
	// if (0 != err)
	// {
	// 	printf("can't create startCarByAuto thread!\n");
	// 	exit(1);
	// }

	pthread_join(car_pwr_monitor_tid, NULL);
	pthread_join(car_manual_tid, NULL);
	//pthread_join(car_auto_tid, NULL);
	releaseLocalNet();

	return 0;
}
