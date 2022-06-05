#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "startGamepadCapData.h"
#include "startLocalNetCapData.h"
#include "startUwbCapData.h"
#include "utils.h"
#include "log.h"

// global data
extern _ComGpdKey g_gpd_key;
extern pthread_mutex_t g_gpd_mutex;
extern _UwbData g_uwb_loc;
extern pthread_mutex_t g_uwb_mutex;

extern _MySocketInfo g_gatewaySocket;
// global data end

// global flag
int g_mv_stop = 1;
int g_m_suspend = 0;
int g_a_suspend = 0;

#define CMD_SIZE 6

static char turnKeyValue(uint8_t key) {
  if (key > 0x80)
    return ((char)key & 0x7F);
  else if (key < 0x7F)
    return (((char)abs((key - 0x7F))) | 0x80);
  else
    return 0;
}

void *startMotiCtrlByManual(void *args) {

  _ComGpdKey tmp;

  // default: no move
  char cmd[CMD_SIZE] = {0xCC, 0xCC, 0x01, 0x00, 0x00, 0x99};

  while (1) {
    pthread_mutex_lock(&(g_gpd_mutex));
    tmp = g_gpd_key;
    pthread_mutex_unlock(&(g_gpd_mutex));

    // transient key pressed
    if (tmp.key == 0x0200) //停车
    {
      g_mv_stop   = 1;
      g_m_suspend = 1;
      g_a_suspend = 0;
    } else if (tmp.key == 0x0100) //启动手动控制 关闭自动控制
    {
      g_mv_stop   = 0;
      g_m_suspend = 1;
      g_a_suspend = 0;
    } else if (tmp.key == 0x4000) //启动自动控制 关闭手动控制
    {
      g_mv_stop   = 0;
      g_m_suspend = 0;
      g_a_suspend = 1;
    }

    if (g_m_suspend) //手动控制才执行此段程序
    {
      if (g_mv_stop) {//停车
        cmd[3] = 0;
        cmd[4] = 0;
        cmd[5] = cmdSum(cmd, (CMD_SIZE - 1));
        sendInfoToLocalNet(g_gatewaySocket, cmd, CMD_SIZE);
        //Log(DEBUG,"Stop Mode");
      } else {//手柄控制
        cmd[3] = turnKeyValue(tmp.lrk_h);
        cmd[4] = turnKeyValue(tmp.lrk_v);
        cmd[5] = cmdSum(cmd, (CMD_SIZE - 1));
        sendInfoToLocalNet(g_gatewaySocket, cmd, CMD_SIZE);
        //Log(DEBUG,"Manual Mode");
      }

      // printf("g_gatewaySocket.socketCon  = %d, ", g_gatewaySocket.socketCon);
      // printf("Test cmd[]: %x, %x, %x, %x, %x, %x\n", cmd[0], cmd[1],
      // cmd[2],cmd[3], cmd[4], cmd[5]);
    }

    usleep(50000); // 50 ms
    // sleep(1);
  }

  return NULL;
}
