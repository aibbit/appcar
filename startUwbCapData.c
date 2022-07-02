#include "startUwbCapData.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "log.h"
#include "rv1126com.h"
#include "utils.h"

#define MAX_BUFF 1024
#define UWB_DATA_SIZE 12

pthread_t com_rcv_uwb_tid;
pthread_mutex_t g_uwb_mutex;
_UwbData g_uwb_loc;

int fd_uwb = 0;

//数据帧头  999.999=   F0       FF       79        44
//                   data0     data1    data2    data3

// 4字节head 4字节x 4字节y
void ParseDataForUwb(char chr) {
  static uint8_t chrBuf[MAX_BUFF];
  static int chrCnt = 0;
  uint8_t x[4] = {0};
  uint8_t y[4] = {0};

  chrBuf[chrCnt++] = chr;
  if (chrCnt < UWB_DATA_SIZE) return;

  if ((chrBuf[0] != 0xF0) || (chrBuf[1] != 0xFF) || (chrBuf[2] != 0x79) ||
      (chrBuf[3] != 0x44)) {
    Log(WARN, "UWB check Error: %x, %x, %x, %x", chrBuf[0], chrBuf[1],
        chrBuf[2], chrBuf[3]);
    memcpy(&chrBuf[0], &chrBuf[1], UWB_DATA_SIZE - 1);  //向前移动1字节
    chrCnt--;
    return;
  }

  memcpy(x, chrBuf + 4, 4);
  memcpy(y, chrBuf + 8, 4);
  float x0 = byte2float(x);
  float y0 = byte2float(y);

  pthread_mutex_lock(&(g_uwb_mutex));
  g_uwb_loc.x = x0;
  g_uwb_loc.y = y0;
  pthread_mutex_unlock(&(g_uwb_mutex));
  // printf("UWB: ===>> (x,y) = (%f,%f)\n", g_uwb_loc.x, g_uwb_loc.y);
  // Log(DEBUG,"UWB: ===>> (x,y) = (%.2f,%.2f)", g_uwb_loc.x, g_uwb_loc.y);

  chrCnt = 0;
}

void *startComRcvUwbData(void *args) {
  char buffer[MAX_BUFF];    //接收串口缓冲数组
  bzero(buffer, MAX_BUFF);  //清除缓存

  while (1) {
    int sz = rv1126_com_receive(fd_uwb, buffer, 36);
    // printf(" ==================>>>UWB       size = %d\n", sz);
    if (sz == -1) {
      // fprintf(stderr, "uart read failed!\n");
      Log(ERROR, "UWB uart read failed!");
      sleep(1);
    } else {
      for (int i = 0; i < sz; i++) {
        ParseDataForUwb(buffer[i]);
      }
    }

    bzero(buffer, MAX_BUFF);
    usleep(200000);  // 200ms
    // sleep(1);
  }

  if (fd_uwb > -1) {
    rv1126_com_close(fd_uwb);
  }

  return NULL;
}

/**
 * @brief uwb串口初始化
 *
 **/
void startComInitForUWB() {
  int tryCounter = 0;

  while (1) {
    fd_uwb = rv1126_com_open(3, 115200);
    // printf("=======startUwbCapData:com3===>>> fd_uwb = %d\n", fd_uwb);
    Log(INFO, "COM3===>>>fd_uwb=%d", fd_uwb);
    if (fd_uwb <= -1) {
      // printf("Open COM3 fail, try another time ... \n");
      Log(WARN, "Open COM3 fail, try another time ... ");
      tryCounter++;
      if (tryCounter > 5) {
        // printf("Open COM3 fail, end.\n");
        Log(ERROR, "Open COM3 fail, end.");
        exit(EXIT_FAILURE);
      }
    } else {
      break;
    }
    sleep(1);
  }

  int err = pthread_create(&com_rcv_uwb_tid, NULL, startComRcvUwbData, NULL);
  if (0 != err) {
    // printf("can't create startComRcvUwbData thread!\n");
    Log(ERROR, "can't create startComRcvUwbData thread!");
    exit(1);
  }
  return;
}

/**
 * @brief 获取UWB数据接口
 *
 * @return _UwbData
 **/
_UwbData get_uwb_data(void) {
  _UwbData data = {0};

  pthread_mutex_lock(&(g_uwb_mutex));
  data = g_uwb_loc;
  pthread_mutex_unlock(&(g_uwb_mutex));

  return data;
}

/**
 * @brief 保存uwb数据到文件
 *
 * @param data
 * @param filename
 * @param namesize
**/
void save_uwb_data(_UwbData data, char *filename, int namesize) {
  FILE *fp = NULL;
  char file_name[128] = {0};
  char buf[64] = {0};

  memcpy(file_name, filename, namesize);
  sprintf(buf, "%.3f,%.3f\n", data.x, data.y);

  if ((fp = fopen(file_name, "a+")) != NULL) {
    fprintf(fp, "%s", buf);
    fclose(fp);
  }
}
