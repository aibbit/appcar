#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "appcar.h"
#include "rv1126com.h"
#include "startGamepadCapData.h"
#include "utils.h"

#define MAX_BUFF 1024

pthread_t com_rcv_gpd_tid;
pthread_mutex_t g_gpd_mutex;

_ComGpdKey g_gpd_key;
int g_file_descriptor_gpd;

void ParseDataForCtrlFdbk(char chr) {
  static char chrBuf[100];
  static uint8_t chrCnt = 0;
  uint8_t sData[10];
  _ComGpdKey tmpData;

  chrBuf[chrCnt++] = chr;
  if (chrCnt < 13)
    return;

  if ((chrBuf[0] != 0xAA) || (chrBuf[1] != 0xAA) || (chrBuf[2] != 0x01) ||
      (chrBuf[3] != 0x0D)) {
    printf("Error:%x, %x, %x, %x\n", chrBuf[0], chrBuf[1], chrBuf[2],
           chrBuf[3]);
    memcpy(&chrBuf[0], &chrBuf[1], 12);
    chrCnt--;
    return;
  }

  memcpy(&sData[0], &chrBuf[4], 9);
  uint16_t tmp1, tmp2, crc;
  tmp1 = (uint16_t)sData[7];
  tmp2 = (uint16_t)sData[8];
  crc = CRC_Table(chrBuf, 11);
  if (crc == ((tmp2 << 8) | (tmp1 & 0xFF))) {
    chrCnt = 0;
    // printf("Ok: checkSum = %x\n", sData[9]);
    tmpData.voltage = sData[0];
    tmpData.lrk_h = sData[1];
    tmpData.lrk_v = sData[2];
    tmpData.rrk_h = sData[3];
    tmpData.rrk_v = sData[4];
    tmp1 = (uint16_t)sData[5];
    tmp2 = (uint16_t)sData[6];
    tmpData.key = (tmp1 << 8) | (tmp2 & 0xFF);
    pthread_mutex_lock(&(g_gpd_mutex));
    g_gpd_key = tmpData;
    pthread_mutex_unlock(&(g_gpd_mutex));

    return;
  }

  chrCnt = 0;
}

void *startComRcvGamepadKey(void *args) {

  char buffer[MAX_BUFF];
  bzero(buffer, MAX_BUFF);

  printf("============startcomRcvGamepadKey....11111====\n");
  while (1) {
    int sz = rv1126_com_receive(g_file_descriptor_gpd, buffer, 52);
    if (sz == -1) {
      fprintf(stderr, "Com read failed!\n");
      usleep(500000);
      continue;
    }
    for (int i = 0; i < sz; i++) {
      ParseDataForCtrlFdbk(buffer[i]);
    }

    usleep(10000);
  }

  if (g_file_descriptor_gpd > -1) {
    rv1126_com_close(g_file_descriptor_gpd);
  }

  return NULL;
}

void startComInitForGamePad() {
  int tryCounter = 0;

  while (1) {
    g_file_descriptor_gpd = rv1126_com_open(1, 115200);
    printf(
        "=======startComInitForGamePad:com1===>>> g_file_descriptor_gpd = %d\n",
        g_file_descriptor_gpd);
    if (g_file_descriptor_gpd <= -1) {
      printf("Open COM1 fail, try another time ... \n");
      tryCounter++;
      if (tryCounter > 5) {
        printf("Open COM1 fail, end.\n");
        exit(EXIT_FAILURE);
      }
    } else {
      break;
    }
    sleep(1);
  }

  int err = pthread_create(&com_rcv_gpd_tid, NULL, startComRcvGamepadKey, NULL);
  if (0 != err) {
    printf("can't create startComRcvGamepadKey thread!\n");
    exit(1);
  }
  return;
}
