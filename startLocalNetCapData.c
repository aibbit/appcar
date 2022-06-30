#include <arpa/inet.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "log.h"
#include "startLocalNetCapData.h"
#include "utils.h"

//#define USE_TEST
//#define USE_FEEDBACK
//#define USE_TEST_CACHE 0

// for test
#if USE_TEST_CACHE
_TestGateWayCache g_gwy_tst_data = {0};
_TestIMGLOCCache g_image_tst_data = {0};
_TestBinCamCache g_BinCam_tst_data = {0};
#endif

// #define TEST_CACHE_MAX_LEN 1024 * 64  // 64k //(1024 * 1024 * 16) // 16 MB
// #define TEST_SAVE_MIN_LEN  1024 * 16  // 16k //(1024 * 1024 * 8)  //  8 MB
// #define TEST_OPEN_CLOSE_CONTAINER 0

// server
#define IP_ADDR "192.168.1.100"
#define SERVER_PORT 6000

// client
#define GATEWAY_IP_ADDR "192.168.1.30"
#define RV3399_IP_ADDR "192.168.1.20"
#define BINCAM_IP_ADDR "192.168.1.34"

void *thdAcceptHandler(void *socketListen);
void *thdRcvGatewayHandler(void *socketInfo);
void *thdRcvBinCamHandler(void *socketInfo);
void *thdRcvRV3399Handler(void *socketInfo);

int checkThrIsKill(pthread_t thd);

// 客户端数组
_MySocketInfo arrConSocket[10];
int conClientCount = 0;

// 接受客户端线程列表
pthread_t arrThrReceiveClient[10];
int thrReceiveClientCount = 0;

int g_socketListen = -1;
pthread_t g_thdAccept = 0;

// client socket
_MySocketInfo g_gatewaySocket;
_MySocketInfo g_bincamSocket;
_MySocketInfo g_rv3399Socket;

// client data source
_GatewayInfo g_gateway_info;
pthread_mutex_t g_gtwy_info_mutex;
_BinCamInfo g_bincam_info;
pthread_mutex_t g_bincam_info_mutex;

_RV3399Info g_rv3399_info = {0};
pthread_mutex_t g_rv3399_info_mutex;

// control cmd start
#define CONTROL_CMD_SIZE 6
// angle 大于0右转 小于0左转
//精度90/2^7-1度
static uint8_t angle_cmd(float angle) {
  uint8_t cmd = 0;
  const double epslon = 1e-6;

  if (angle > -1 * epslon && angle < epslon) // angle == 0
    cmd = 0x00;
  else if (angle > 0)
    cmd = ((uint8_t)(angle * 127 / 90) & 0x7F);
  else // angle < 0
    cmd = ((uint8_t)(-1 * angle * 127 / 90) | 0x80);

  return (cmd);
}

// speed 大于0前进 小于0后退
//精度Vmax/2^7-1
static uint8_t speed_cmd(float speed) {
  uint8_t cmd = 0;
  const double epslon = 1e-6;

  if (speed > -1 * epslon && speed < epslon) // speed == 0
    cmd = 0x00;
  else if (speed > 0)
    cmd = ((uint8_t)(speed * 127 / 90) & 0x7F);
  else // speed < 0
    cmd = ((uint8_t)(-1 * speed * 127 / 90) | 0x80);

  return (cmd);
}

//发送控制命令
// angle 大于0右转 小于0左转
// speed 大于0前进 小于0后退
void send_control_cmd(float angle, float speed) {

  uint8_t cmd[CONTROL_CMD_SIZE] = {0xCC, 0xCC, 0x01, 0x00, 0x00, 0x99};

  cmd[3] = angle_cmd(angle); // move angle
  cmd[4] = speed_cmd(speed); // move speed
  cmd[5] = cmdSum(cmd, (CONTROL_CMD_SIZE - 1));

  // Log(DEBUG, "cmd=%x %x %x %x %x", cmd[0], cmd[1], cmd[2], cmd[3],
  // cmd[4],cmd[5]);

  pthread_mutex_lock(&(g_gtwy_info_mutex));
  sendInfoToLocalNet(g_gatewaySocket, cmd, CONTROL_CMD_SIZE);
  pthread_mutex_unlock(&(g_gtwy_info_mutex));
}
// control cmd end

// label cam start
#define LABEL_CAM_BUFF_SIZE 6
int8_t label_cam_start_cmd[LABEL_CAM_BUFF_SIZE] = {0x9A, 0x9B, 0x01,
                                                   0x00, 0x00, 0x36};
const uint8_t label_cam_heart_cmd[LABEL_CAM_BUFF_SIZE] = {0x9A, 0x9B, 0x02,
                                                          0x00, 0x00, 0x37};
const uint8_t label_cam_end_cmd[LABEL_CAM_BUFF_SIZE] = {0x9A, 0x9B, 0x03,
                                                        0x00, 0x00, 0x38};

//向标识摄像头发送数据
int label_cam_send_data(_MySocketInfo skt, const uint8_t info[], int size) {
  if (skt.socketCon > 0) {
    int slen = write(skt.socketCon, (char *)info, size);
    if (slen > 0) {
      Log(DEBUG, "label_cam send oK!");
      return slen;
    } else {
      Log(WARN, "label_cam send fail!");
      return -1;
    }
  }
}

/**
 * @brief  发送开始读数命令
 * @param  None
 * @retval None
 */
void label_cam_send_start(int8_t x, int8_t y) {
  label_cam_start_cmd[3] = x;
  label_cam_start_cmd[4] = y;

  label_cam_start_cmd[5] = cmdSum(label_cam_start_cmd, LABEL_CAM_BUFF_SIZE - 1);

  label_cam_send_data(g_rv3399Socket, label_cam_start_cmd, LABEL_CAM_BUFF_SIZE);
}

/**
 * @brief  发送心跳
 * @param  None
 * @retval None
 */
void label_cam_send_heart(void) {
  label_cam_send_data(g_rv3399Socket, label_cam_heart_cmd, LABEL_CAM_BUFF_SIZE);
}

/**
 * @brief  发送结束读数命令
 * @param  None
 * @retval None
 */
void label_cam_send_end(void) {
  label_cam_send_data(g_rv3399Socket, label_cam_end_cmd, LABEL_CAM_BUFF_SIZE);
}
// label cam end

int releaseLocalNet() {
  Log(INFO, "waiting for exit!");
  char *message;
  pthread_join(g_thdAccept, (void *)&message);
  Log(INFO, "%s", message);
  close(g_socketListen);
  return 0;
}

int checkThrIsKill(pthread_t thd) {
  int res = 1;
  int res_kill = pthread_kill(thd, 0);
  if (res_kill == 0) {
    res = 0;
  }
  return res;
}

void sendInfoToLocalNet(_MySocketInfo skt, char info[], int size) {
  if (skt.socketCon > 0) {
    int slen = write(skt.socketCon, (char *)info, size);
    if (slen > 0) {
      // printf("\nGateway:%s:%d,send info oK!\n", skt.ipaddr, skt.port);
      // Log(DEBUG,"Gateway:%s:%d,send info oK!", skt.ipaddr, skt.port);
    } else {
      // printf("\nGateway: %s:%d, send info failed!\n", skt.ipaddr, skt.port);
      Log(WARN, "Gateway: %s:%d, send info failed!", skt.ipaddr, skt.port);
    }
  }
}

int startLocalNetInit() {
  memset(arrConSocket, 0, sizeof(_MySocketInfo) * 10);

  Log(INFO, "Socket start~");
  // create one socket for tcp
  g_socketListen = socket(AF_INET, SOCK_STREAM, 0);
  if (g_socketListen < 0) {
    Log(ERROR, "Create TCP socket fail!");
    exit(-1);
  } else {
    Log(INFO, "Create TCP socket ok!");
  }
  int opt = 1;
  setsockopt(g_socketListen, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt,
             sizeof(opt));
  // init ip addr and port
  struct sockaddr_in server_addr;
  bzero(&server_addr, sizeof(struct sockaddr_in));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = inet_addr(IP_ADDR);
  server_addr.sin_port = htons(SERVER_PORT);
  if (bind(g_socketListen, (struct sockaddr *)&server_addr,
           sizeof(struct sockaddr)) != 0) {
    // perror("Bind ip addr and port fail!");
    Log(ERROR, "Bind ip addr and port fail!");
    exit(-1);
  } else {
    Log(INFO, "Bind ip addr and port ok!");
  }
  // listening to the port
  if (listen(g_socketListen, 10) != 0) {
    Log(ERROR, "Keep listen fail!");
    exit(-1);
  } else {
    Log(INFO, "Keep listen ok!");
  }
  // start to accept
  pthread_create(&g_thdAccept, NULL, thdAcceptHandler, &g_socketListen);

  return 0;
}

void *thdAcceptHandler(void *g_socketListen) {
  while (1) {

    //判断线程存活多少
    if (thrReceiveClientCount > 0) {
      for (int i = 0; i < thrReceiveClientCount; i++) {
        if (checkThrIsKill(arrThrReceiveClient[i]) == 1) {
          Log(DEBUG, "A thread killed");
          thrReceiveClientCount--;
        }
      }
      //当前有接受数据线程多少个
      Log(INFO, "receiving data threads:%d", thrReceiveClientCount);
    }

    if (thrReceiveClientCount < 10) {
      int sockaddr_in_size = sizeof(struct sockaddr_in);
      struct sockaddr_in client_addr;
      int _g_socketListen = *((int *)g_socketListen);
      int socketCon = accept(_g_socketListen, (struct sockaddr *)(&client_addr),
                             (socklen_t *)(&sockaddr_in_size));
      if (socketCon < 0) {
        Log(ERROR, "Socket connect fail!");
      } else {
        Log(INFO, "Socket connect ok, ip: %s:%d",
            inet_ntoa(client_addr.sin_addr), client_addr.sin_port);
      }
      Log(INFO, "Connected socket:%d", socketCon);
      // create one thread to deal with one client
      _MySocketInfo socketInfo;
      socketInfo.socketCon = socketCon;
      socketInfo.ipaddr = inet_ntoa(client_addr.sin_addr);
      socketInfo.port = client_addr.sin_port;
      pthread_t thrReceive = 0;
      if (strcmp(socketInfo.ipaddr, GATEWAY_IP_ADDR) == 0) {
        g_gatewaySocket = socketInfo;
        pthread_create(&thrReceive, NULL, thdRcvGatewayHandler,
                       &g_gatewaySocket);
      } else if (strcmp(socketInfo.ipaddr, BINCAM_IP_ADDR) == 0) {
        g_bincamSocket = socketInfo;
        pthread_create(&thrReceive, NULL, thdRcvBinCamHandler, &g_bincamSocket);
      } else if (strcmp(socketInfo.ipaddr, RV3399_IP_ADDR) == 0) {
        g_rv3399Socket = socketInfo;
        pthread_create(&thrReceive, NULL, thdRcvRV3399Handler, &g_rv3399Socket);
      } else {
        Log(WARN, "%s is not in field.", socketInfo.ipaddr);
        close(socketCon);
        continue;
      }
      arrConSocket[conClientCount] = socketInfo;
      conClientCount++;
      Log(INFO, "Connected %d device.", conClientCount);

      arrThrReceiveClient[thrReceiveClientCount] = thrReceive;
      thrReceiveClientCount++;
    } else {
      Log(ERROR, "Received count(%d) is full.", thrReceiveClientCount);
    }
    sleep(1);
  }

  char *s = "Exit the accept function";
  pthread_exit(s);
}

void ParseDataForGateway(char chr) {
  static char chrBuf[100];
  static unsigned char chrCnt = 0;
  unsigned char sData[20];
  _GatewayInfo tmpData;

  chrBuf[chrCnt++] = chr;
  if (chrCnt < 22)
    return;

  if ((chrBuf[0] != 0x8A) || (chrBuf[1] != 0x8B)) {
    Log(ERROR, "Error:%x %x", chrBuf[0], chrBuf[1]);
    memcpy(&chrBuf[0], &chrBuf[1], 21);
    chrCnt--;
    return;
  }
  memcpy(&sData[0], &chrBuf[2], 20);

  if ((chrBuf[0] == 0x8A) && (chrBuf[1] == 0x8B)) {
    tmpData.dist1 = sData[0];
    tmpData.spd1 = sData[1];
    tmpData.dist2 = sData[2];
    tmpData.spd2 = sData[3];
    tmpData.ultra1 = sData[4]; // if (sData[x] * 5) = y cm
    tmpData.ultra2 = sData[5];
    tmpData.ultra3 = sData[6];
    tmpData.ultra4 = sData[7];
    tmpData.ultra5 = sData[8];
    tmpData.ultra6 = sData[9];
    tmpData.ultra7 = sData[10];
    tmpData.ultra8 = sData[11];
    tmpData.ultra_sts = sData[12];
    tmpData.illumi_pwr = sData[13];
    tmpData.box1_sts = sData[14];
    tmpData.box2_sts = sData[15];
    tmpData.box3_sts = sData[16];
    tmpData.yawH = sData[17];
    tmpData.yawL = sData[18];

    int sum = 0x8A + 0x8B;
    for (int i = 0; i < 19; i++) {
      sum += (int)sData[i];
    }
    if ((unsigned char)(sum & 0xFF) == sData[19]) {
      chrCnt = 0;
      // printf("\n----------check SUM ok!----------\n");
      //  give the data to global data
      pthread_mutex_lock(&(g_gtwy_info_mutex));
      g_gateway_info = tmpData;
      pthread_mutex_unlock(&(g_gtwy_info_mutex));
#if USE_TEST_CACHE
      pthread_mutex_lock(&(g_gwy_tst_data.mutex));
      uint8_t n = g_gwy_tst_data.used;
      pthread_mutex_unlock(&(g_gwy_tst_data.mutex));
      int len = g_gwy_tst_data.len[n];
      if ((g_gwy_tst_data.cache[n] != NULL) && (len < TEST_SAVE_MIN_LEN)) // 8MB
      {
        g_gwy_tst_data.cache[n][len] = tmpData;
        g_gwy_tst_data.len[n]++;
      }
#endif
      return;
    }
  }
  chrCnt = 0;
}

// pending
void ParseDataForBinCam(char chr) {
  static char BinCam_chrBuf[100];
  static unsigned char BinCam_chrCnt = 0;
  // unsigned char sData[100];
  _BinCamInfo tmpData;

  BinCam_chrBuf[BinCam_chrCnt++] = chr;
  if (BinCam_chrCnt < 3)
    return;

  if ((BinCam_chrBuf[0] != 0xFF) || (BinCam_chrBuf[1] != 0xEE)) {
    Log(ERROR, "Error:%x %x", BinCam_chrBuf[0], BinCam_chrBuf[1]);
    memcpy(&BinCam_chrBuf[0], &BinCam_chrBuf[1], 2);
    BinCam_chrCnt--;
    return;
  }
  if (BinCam_chrCnt < (BinCam_chrBuf[2] * 4 + 4))
    return;

  // memcpy(&sData[0], &BinCam_chrBuf[2], (BinCam_chrBuf[2]*4+2));
  // memcpy(&sData[0], &BinCam_chrBuf[0], (BinCam_chrBuf[2]*4+4));

  if ((BinCam_chrBuf[0] == 0xFF) && (BinCam_chrBuf[1] == 0xEE)) {

    int sum = 0;

    for (int i = 0; i < (BinCam_chrBuf[2] * 4 + 3); i++) {
      sum += (int)BinCam_chrBuf[i];
    }
    // if ((unsigned char)(sum & 0xFF) == sData[(BinCam_chrBuf[2]*4+1)])
    if ((unsigned char)(sum & 0xFF) ==
        BinCam_chrBuf[(BinCam_chrBuf[2] * 4 + 3)]) {
      BinCam_chrCnt = 0;
      // printf("\n----------check SUM ok!----------\n");
      //  give the data to global data

      for (int j = (BinCam_chrBuf[2] * 4 + 3); j < (12 * 4 + 3); j++) {
        BinCam_chrBuf[j] = 0;
      }
      tmpData.head[0] = BinCam_chrBuf[0];
      tmpData.head[1] = BinCam_chrBuf[1];
      tmpData.number = BinCam_chrBuf[2];

      tmpData.classific_1 = BinCam_chrBuf[3];
      tmpData.dist_1 = BinCam_chrBuf[4];
      tmpData.direction_1 = BinCam_chrBuf[5];
      tmpData.width_1 = BinCam_chrBuf[6];

      tmpData.classific_2 = BinCam_chrBuf[7];
      tmpData.dist_2 = BinCam_chrBuf[8];
      tmpData.direction_2 = BinCam_chrBuf[9];
      tmpData.width_2 = BinCam_chrBuf[10];

      tmpData.classific_3 = BinCam_chrBuf[11];
      tmpData.dist_3 = BinCam_chrBuf[12];
      tmpData.direction_3 = BinCam_chrBuf[13];
      tmpData.width_3 = BinCam_chrBuf[14];

      tmpData.classific_4 = BinCam_chrBuf[15];
      tmpData.dist_4 = BinCam_chrBuf[16];
      tmpData.direction_4 = BinCam_chrBuf[17];
      tmpData.width_4 = BinCam_chrBuf[18];

      tmpData.classific_5 = BinCam_chrBuf[19];
      tmpData.dist_5 = BinCam_chrBuf[20];
      tmpData.direction_5 = BinCam_chrBuf[21];
      tmpData.width_5 = BinCam_chrBuf[22];

      tmpData.classific_6 = BinCam_chrBuf[23];
      tmpData.dist_6 = BinCam_chrBuf[24];
      tmpData.direction_6 = BinCam_chrBuf[25];
      tmpData.width_6 = BinCam_chrBuf[26];

      tmpData.classific_7 = BinCam_chrBuf[27];
      tmpData.dist_7 = BinCam_chrBuf[28];
      tmpData.direction_7 = BinCam_chrBuf[29];
      tmpData.width_7 = BinCam_chrBuf[30];

      tmpData.classific_8 = BinCam_chrBuf[31];
      tmpData.dist_8 = BinCam_chrBuf[32];
      tmpData.direction_8 = BinCam_chrBuf[33];
      tmpData.width_8 = BinCam_chrBuf[34];

      tmpData.classific_9 = BinCam_chrBuf[35];
      tmpData.dist_9 = BinCam_chrBuf[36];
      tmpData.direction_9 = BinCam_chrBuf[37];
      tmpData.width_9 = BinCam_chrBuf[38];

      tmpData.classific_10 = BinCam_chrBuf[39];
      tmpData.dist_10 = BinCam_chrBuf[40];
      tmpData.direction_10 = BinCam_chrBuf[41];
      tmpData.width_10 = BinCam_chrBuf[42];

      tmpData.classific_11 = BinCam_chrBuf[43];
      tmpData.dist_11 = BinCam_chrBuf[44];
      tmpData.direction_11 = BinCam_chrBuf[45];
      tmpData.width_11 = BinCam_chrBuf[46];

      tmpData.classific_12 = BinCam_chrBuf[47];
      tmpData.dist_12 = BinCam_chrBuf[48];
      tmpData.direction_12 = BinCam_chrBuf[49];
      tmpData.width_12 = BinCam_chrBuf[50];

      pthread_mutex_lock(&(g_bincam_info_mutex));
      g_bincam_info = tmpData;
      pthread_mutex_unlock(&(g_bincam_info_mutex));

      return;
    }
  }
  BinCam_chrCnt = 0;
}

void *thdRcvGatewayHandler(void *socketInfo) {
  char buffer[1024];
  int buffer_length;
#if USE_TEST_CACHE
  for (int i = 0; i < 2; i++) {
    g_gwy_tst_data.cache[i] =
        (_GatewayInfo *)malloc(TEST_CACHE_MAX_LEN * sizeof(_GatewayInfo));
    memset(g_gwy_tst_data.cache[i], 0,
           TEST_CACHE_MAX_LEN * sizeof(_GatewayInfo));
  }
#endif
  _MySocketInfo _socketInfo = *((_MySocketInfo *)socketInfo);
  while (1) {
    bzero(&buffer, sizeof(buffer));

    buffer_length = read(_socketInfo.socketCon, buffer, 1024);
    if (buffer_length == 0) {
      printf("%s:%d client closed.\n", _socketInfo.ipaddr, _socketInfo.port);
      conClientCount--;
      break;
    } else if (buffer_length < 0) {
      printf("Receive data from client fail.\n");
      conClientCount--;
      break;
    }
    // printf("%s:%d:len=%d: \n", _socketInfo.ipaddr, _socketInfo.port,
    // buffer_length); Log(DEBUG,"%s:%d:len=%d", _socketInfo.ipaddr,
    // _socketInfo.port, buffer_length);
#ifdef USE_TEST
    for (int i = 0; i < buffer_length; i++) {
      // if (buffer[i] != 0)
      { printf("%d\t", buffer[i]); }
    }
#endif

    if (buffer_length < 22)
      continue;
    for (int i = 0; i < buffer_length; i++) {
      ParseDataForGateway(buffer[i]);
    }

#ifdef USE_FEEDBACK
    {
      char userStr[8] = {0x8A, 0x8B, 0x68, 0x65, 0x61, 0x72, 0x74, 0x8E};
      int sendMsg_len = write(_socketInfo.socketCon, userStr, 8);
      if (sendMsg_len > 0) {
        printf("\nTo %s:%d, send ok.\n", _socketInfo.ipaddr, _socketInfo.port);
      } else {
        printf("\nTo %s:%d, send fail.\n", _socketInfo.ipaddr,
               _socketInfo.port);
      }
    }
#endif
    // sleep(1);
    // usleep(50000); // 50ms
  }
  close(_socketInfo.socketCon);
  printf("%s, rcvGateway exit.\n", _socketInfo.ipaddr);
  memset(socketInfo, 0, sizeof(_MySocketInfo));

#if USE_TEST_CACHE
  for (int i = 0; i < 2; i++) {
    if (g_gwy_tst_data.cache[i] != NULL) {
      free(g_gwy_tst_data.cache[i]);
      g_gwy_tst_data.cache[i] = NULL;
    }
  }
#endif

  return NULL;
}

void *thdRcvBinCamHandler(void *socketInfo) {
  char buffer[1024];
  int buffer_length;

  _MySocketInfo _socketInfo = *((_MySocketInfo *)socketInfo);
  while (1) {
    bzero(&buffer, sizeof(buffer));

    buffer_length = read(_socketInfo.socketCon, buffer, 1024);
    if (buffer_length == 0) {
      printf("%s:%d client closed.\n", _socketInfo.ipaddr, _socketInfo.port);
      conClientCount--;
      break;
    } else if (buffer_length < 0) {
      printf("Receive data from client fail.\n");
      conClientCount--;
      break;
    }
    printf("%s:%d:len=%d: ", _socketInfo.ipaddr, _socketInfo.port,
           buffer_length);
    printf("\n");

    ////////////////////////////////////////////////////////////////////////////cw
    /// begin

    // if (buffer_length < 20)
    //     continue;

    for (int i = 0; i < buffer_length; i++) {
      ParseDataForBinCam(buffer[i]);
    }
    ///////////////////////////////////////////////////////////////////////////cw
    /// end
    sleep(0.2);
  }
  close(_socketInfo.socketCon);
  printf("%s, rcvBinCam exit.", _socketInfo.ipaddr);
  memset(socketInfo, 0, sizeof(_MySocketInfo));
  return NULL;
}

void ParseDataForRV3399(char chr) {
  static uint8_t chrBuf[100];
  static uint8_t chrCnt = 0;
  uint8_t sData[4];
  _RV3399Info tmpData;

  chrBuf[chrCnt++] = chr;
  if (chrCnt < 6)
    return;

  if ((chrBuf[0] != 0xC8) || (chrBuf[1] != 0xC9)) {
    Log(ERROR, "Error:%x %x", chrBuf[0], chrBuf[1]);
    memcpy(&chrBuf[0], &chrBuf[1], 5);
    chrCnt--;
    return;
  }
  memcpy(&sData[0], &chrBuf[2], 4);

  if ((chrBuf[0] == 0xC8) && (chrBuf[1] == 0xC9)) {
    tmpData.symbol = sData[0];
    tmpData.theta_gyro = sData[1];
    tmpData.theta_cam = sData[2];

    int sum = 0xC8 + 0xC9;

    for (int i = 0; i < 3; i++) {
      sum += (int)sData[i];
    }
    if ((sum & 0xFF) == sData[3]) //校验位
    {
      chrCnt = 0;
      pthread_mutex_lock(&(g_rv3399_info_mutex));
      g_rv3399_info = tmpData;
      pthread_mutex_unlock(&(g_rv3399_info_mutex));
      return;
    } else {
      Log(WARN, "check error");
    }
  } else {
    Log(ERROR, "Error:%x %x", chrBuf[0], chrBuf[1]);
  }
  chrCnt = 0;
}

void *thdRcvRV3399Handler(void *socketInfo) {
  char buffer[1024];
  int buffer_length;

  _MySocketInfo _socketInfo = *((_MySocketInfo *)socketInfo);
  while (1) {
    bzero(&buffer, sizeof(buffer));

    buffer_length = read(_socketInfo.socketCon, buffer, 1024);
    if (buffer_length == 0) {
      Log(WARN, "%s:%d client closed.", _socketInfo.ipaddr, _socketInfo.port);
      conClientCount--;
      break;
    } else if (buffer_length < 0) {
      Log(WARN, "Receive data from rv3399 fail.");
      conClientCount--;
      break;
    }
    Log(DEBUG, "%s:%d:len=%d", _socketInfo.ipaddr, _socketInfo.port,
        buffer_length);

    if (buffer_length < 6)
      continue;
    for (int i = 0; i < buffer_length; i++) {
      ParseDataForRV3399(buffer[i]);
    }

    sleep(0.2);
  }
  close(_socketInfo.socketCon);
  Log(ERROR, "%s, Receive data from rv3399 fail.", _socketInfo.ipaddr);
  memset(socketInfo, 0, sizeof(_MySocketInfo));
  return NULL;
}
