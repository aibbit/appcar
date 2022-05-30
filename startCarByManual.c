#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include "startMotiCtrlByManual.h"
#include "appcar.h"

extern int g_stop;
extern int g_m_start;
extern int g_m_suspend;
extern int g_m_stop;

void *startCarByManual(void *args)
{
	pthread_t moti_ctrl_man_tid;
	int err;
	if (g_m_start)
	{
		err = pthread_create(&moti_ctrl_man_tid, NULL, startMotiCtrlByManual, NULL);
		if (0 != err)
		{
			printf("can't create startMotiCtrlByManual thread!\n");
			exit(1);
		}
	}
	else
	{
		printf("startCarByManual: no start, bye~!\n");
		return NULL;
	}
	while (1)
	{
		if (g_m_suspend)
		{
			// printf("startCarByManual: do nothing...\n");
			// system("echo `date`");
			// check car state, stop or not ?
			// if(stop) { check open container or not ? if(open) containerCtrl(whichone);}
			// else { do nothing and wait.}
			//
		}

		sleep(2);
		if (g_stop)
			break;
		if (g_m_stop)
			break;
	}
	pthread_join(moti_ctrl_man_tid, NULL);

	return NULL;
}
