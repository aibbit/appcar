#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "rv1126com.h"
#include "startUwbCapData.h"

#define MAX_BUFF 1024

pthread_mutex_t g_uwb_mutex;
pthread_t com_rcv_uwb_tid;
int g_file_descriptor_uwb = 0;

struct{
	float x;
	float y;
 } g_uwb_loc;

//数据帧头  999.999=   F0       FF       79        44
//                   data0     data1    data2    data3

//字符转换成浮点数
static float exchange_data(char *data) {
  float float_data;
  float_data = *((float *)data);

  // printf("float_data = %f\n", float_data);

  return float_data;
}

void ParseDataForUwb(char chr, int count) {
  static char chrBuf[100];
  static int chrCnt = 0;
  char x[4] = {0};
  char y[4] = {0};
  //  if(count<1||count>sizeof(chrBuf))
  // count = sizeof(chrBuf);

  chrBuf[chrCnt++] = chr;
  if (chrCnt < 12)
    // if (chrCnt < count)
    return;

  if ((chrBuf[0] != 0xF0) || (chrBuf[1] != 0xFF) || (chrBuf[2] != 0x79) ||
      (chrBuf[3] != 0x44)) {
    // printf("UWB-Error: %x, %x, %x, %x\n", chrBuf[0], chrBuf[1], chrBuf[2],
    // chrBuf[3]);
    memcpy(&chrBuf[0], &chrBuf[1], 11);
    chrCnt--;
    return;
  }

  memcpy(&x[0], &chrBuf[4], 4);
  memcpy(&y[0], &chrBuf[8], 4);
  float x0 = exchange_data(x);
  float y0 = exchange_data(y);

  pthread_mutex_lock(&(g_uwb_mutex));
  g_uwb_loc.x = x0;
  g_uwb_loc.y = y0;
  pthread_mutex_unlock(&(g_uwb_mutex));
  printf("UWB: ===>> (x,y) = (%f,%f)\n", g_uwb_loc.x, g_uwb_loc.y);

  chrCnt = 0;
}

void *startComRcvUwbData(void *args) {

  char buffer[MAX_BUFF];   //接收串口缓冲数组
  bzero(buffer, MAX_BUFF); //清除缓存

  while (1) {
    int sz = rv1126_com_receive(g_file_descriptor_uwb, buffer, 36);
    printf(" ==================>>>UWB       size = %d\n", sz);
    if (sz == -1) {
      fprintf(stderr, "uart read failed!\n");
      sleep(1);
    } else {
      for (int i = 0; i < sz; i++) {
        printf("\n\n");
        ParseDataForUwb(buffer[i], sz);
        printf("\n\n");
      }
    }

    bzero(buffer, MAX_BUFF);
    usleep(200000);
    // sleep(1);
  }

  if (g_file_descriptor_uwb > -1) {
    rv1126_com_close(g_file_descriptor_uwb);
  }

  return NULL;
}

void startComInitForUWB() {
  int tryCounter = 0;

  while (1) {
    g_file_descriptor_uwb = rv1126_com_open(3, 115200);
    printf("=======startUwbCapData:com3===>>> g_file_descriptor_uwb = %d\n", g_file_descriptor_uwb);
    if (g_file_descriptor_uwb <= -1) {
      printf("Open COM3 fail, try another time ... \n");
      tryCounter++;
      if (tryCounter > 5) {
        printf("Open COM3 fail, end.\n");
        exit(EXIT_FAILURE);
      }
    } else {
      break;
    }
    sleep(1);
  }

  int err = pthread_create(&com_rcv_uwb_tid, NULL, startComRcvUwbData, NULL);
  if (0 != err) {
    printf("can't create startComRcvUwbData thread!\n");
    exit(1);
  }
  return;
}
