#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>

#include "log.h"
#include "startLocalNetCapData.h"
#include "startUwbCapData.h"
#include "startGamepadCapData.h"
#include "startMonitorPower.h"
#include "startMotiCtrlByManual.h"
#include "startMotiCtrlByAuto.h"

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
		Log(ERROR,"can't create startMonitorPower thread!\n");
		exit(1);
	}
	err = pthread_create(&car_manual_tid, NULL, startMotiCtrlByManual, NULL);
	if (0 != err)
	{
		Log(ERROR,"can't create startMotiCtrlByManual thread!\n");
		exit(1);
	}
	err = pthread_create(&car_auto_tid, NULL, startMotiCtrlByAuto, NULL);
	if (0 != err)
	{
		Log(ERROR,"can't create startMotiCtrlByAuto thread!\n");
		exit(1);
	}

	pthread_join(car_pwr_monitor_tid, NULL);
	pthread_join(car_manual_tid, NULL);
	pthread_join(car_auto_tid, NULL);
	releaseLocalNet();

	return 0;
}
