#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "startGamepadCapData.h"
#include "startLocalNetCapData.h"
#include "startUwbCapData.h"
#include "utils.h"

//global data
extern _ComGpdKey g_gpd_key;
extern pthread_mutex_t g_gpd_mutex;

extern pthread_mutex_t g_uwb_mutex;
extern _UwbData g_uwb_loc;

extern _MySocketInfo g_gatewaySocket;
//global data end

//global flag
int g_mv_stop = 1;
int g_mv_spd_lock = 0;
int g_m_suspend = 0;
int g_a_suspend = 0;


#define INFO_SIZE 6
// start and no move
const char cmdStart[INFO_SIZE] = {0xCC, 0xCC, 0x02, 0x00, 0x01, 0x9B};
// stop and brake
const char cmdStop[INFO_SIZE] = {0xCC, 0xCC, 0x02, 0x00, 0x02, 0x9C};


static char turnKeyValue(uint8_t key) {
  if (key > 0x80)
    return ((char)key & 0x7F);
  else if (key < 0x7F)
    return (((char)abs((key - 0x7F))) | 0x80);
  else
    return 0;
}

void *startMotiCtrlByManual(void *args) {
  char speedLocked = 0;
  _ComGpdKey tmp;

  char cmd[INFO_SIZE] = {0xCC, 0xCC, 0x01,0x00, 0x00, 0x99}; // default: no move

  while (1) {

    pthread_mutex_lock(&(g_gpd_mutex));
    tmp = g_gpd_key;
    pthread_mutex_unlock(&(g_gpd_mutex));

    // transient key pressed
    if (tmp.key == 0x0200) {
      // g_m_suspend=0;//cw
      // g_a_suspend=0;//cw
      g_mv_stop = 1;
      // memcpy(cmd, cmdStop, sizeof(cmd));
      // sendInfoToLocalNet(g_gatewaySocket, cmd, INFO_SIZE);
      // usleep(50000);
      g_mv_spd_lock = 0;
      speedLocked = 0;
      //	continue;
    } else if (tmp.key == 0x0100) //启动手动控制 关闭自动控制cw
    {
      g_m_suspend = 1; // cw\]
      g_a_suspend = 0; // cw
      g_mv_stop = 0;   //解除急停 cw
      // memcpy(cmd, cmdStart, sizeof(cmd));
      // sendInfoToLocalNet(g_gatewaySocket, cmd, INFO_SIZE);
      // usleep(50000);
      g_mv_spd_lock = 0;
      speedLocked = 0;
      //	continue;
    } else if (tmp.key == 0x4000) //启动自动控制 关闭手动控制cw
    {
      g_m_suspend = 0; // cw
      g_a_suspend = 1; // cw
      g_mv_stop = 0;   //解除急停 cw
      // memcpy(cmd, cmdStart, sizeof(cmd));
      // sendInfoToLocalNet(g_gatewaySocket, cmd, INFO_SIZE);
      // printf("g_a_suspend = %d, ", g_a_suspend);
      // usleep(50000);
      // g_mv_spd_lock = 0;
      // speedLocked = 0;
      //	continue;
    }

    if (g_m_suspend) //手动控制才执行此段程序
    {
      if (g_mv_stop) {
        memcpy(cmd, cmdStop, sizeof(cmd));
        cmd[2] = 1;
        cmd[3] = 0;
        cmd[4] = 0;
        cmd[5] = cmdSum(cmd, (INFO_SIZE - 1));
        sendInfoToLocalNet(g_gatewaySocket, cmd, INFO_SIZE);
        printf("stop    Sucess ");
      } else if (!g_mv_spd_lock) {
        memcpy(cmd, cmdStart, sizeof(cmd));
        cmd[2] = 1;
        cmd[3] = turnKeyValue(tmp.lrk_h);
        cmd[4] = turnKeyValue(tmp.lrk_v);
        cmd[5] = cmdSum(cmd, (INFO_SIZE - 1));
        sendInfoToLocalNet(g_gatewaySocket, cmd, INFO_SIZE);
      } else if (g_mv_spd_lock) {
        memcpy(cmd, cmdStart, sizeof(cmd));
        cmd[2] = 1;
        cmd[3] = turnKeyValue(tmp.lrk_h);
        cmd[4] = speedLocked;
        cmd[5] = cmdSum(cmd, (INFO_SIZE - 1));
        sendInfoToLocalNet(g_gatewaySocket, cmd, INFO_SIZE);
      }

      //printf("g_gatewaySocket.socketCon  = %d, ", g_gatewaySocket.socketCon);
      //printf("Test cmd[]: %x, %x, %x, %x, %x, %x\n", cmd[0], cmd[1], cmd[2],cmd[3], cmd[4], cmd[5]);
    }

    usleep(50000); // 50 ms
    // sleep(1);
  }

  return NULL;
}
