#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>

#include "startMonitorPower.h"
#include "startLocalNetCapData.h"

// client data source
_GatewayInfo g_gateway_info;
pthread_mutex_t g_gtwy_info_mutex;

void *startMonitorPower(void *args)
{
    int nCurrPower = 0;
    while (1)
    {
        pthread_mutex_lock(&(g_gtwy_info_mutex));
        nCurrPower = (int)(g_gateway_info.illumi_pwr & 0x7F);
        pthread_mutex_unlock(&(g_gtwy_info_mutex));
        if ((15 < nCurrPower) && (nCurrPower <= 100))
        {
            printf("Normal: Power(%d) is ok!\n", nCurrPower);
        }
        else if ((5 < nCurrPower) && (nCurrPower <= 15))
        {
            printf("Warning: Power(%d) is low!\n", nCurrPower);
            // back to charge
        }
        else if ((0 <= nCurrPower) && (nCurrPower <= 5))
        {
            printf("Warning: Power(%d) is too low! Stopping for help!\n", nCurrPower);
            // stop and wait for help
        }
        sleep(1);
    }
    return NULL;
}
