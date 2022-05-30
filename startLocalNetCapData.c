#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <arpa/inet.h>

#include "appcar.h"
#include "startLocalNetCapData.h"


#define USE_TEST
#define USE_FEEDBACK

// for test
#if USE_TEST_CACHE
_TestGateWayCache g_gwy_tst_data = {0};
_TestIMGLOCCache  g_image_tst_data ={0};
_TestBinCamCache  g_BinCam_tst_data ={0};

#endif

extern int g_stop;

#define LABEL_CAM_BUFF_SIZE 4
const uint8_t label_cam_start_cmd[LABEL_CAM_BUFF_SIZE] = {0x9A, 0x9B, 0x01, 0x36};
const uint8_t label_cam_end_cmd[LABEL_CAM_BUFF_SIZE] = {0x9A, 0x9B, 0x02, 0x37};



//server
#define IP_ADDR "192.168.1.100"

//#define IP_ADDR "192.168.3.49"

//client

//#define GATEWAY_IP_ADDR "192.168.3.30"
#define GATEWAY_IP_ADDR "192.168.1.30"


#define BINCAM_IP_ADDR "192.168.1.32"

//#define IMGLOC_IP_ADDR "192.168.1.33"
#define IMGLOC_IP_ADDR "192.168.1.31"    ////cw
#define SERVER_PORT 6000

void *thdRcvGatewayHandler(void *socketInfo);
void *thdRcvBinCamHandler(void *socketInfo);
void *thdRcvImgLocHandler(void *socketInfo);
void *thdAcceptHandler(void *socketListen);

int checkThrIsKill(pthread_t thd);

// clinet socket list
_MySocketInfo arrConSocket[10];
int conClientCount = 0;

// client thread list
pthread_t arrThrReceiveClient[10];
int thrReceiveClientCount = 0;

int g_socketListen = -1;
pthread_t g_thdAccept = 0;

// client socket
_MySocketInfo g_gatewaySocket;
_MySocketInfo g_bincamSocket;
_MySocketInfo g_imglocSocket;

// client data source
_GatewayInfo g_gateway_info;
pthread_mutex_t g_gtwy_info_mutex;

_IMGLOCInfo g_image_info;
pthread_mutex_t g_image_info_mutex;

_BinCamInfo g_bincam_info;
pthread_mutex_t g_bincam_info_mutex;


//向标识摄像头发送数据
 int label_cam_send_data(_MySocketInfo skt,const uint8_t info[], int size)
{
	if (skt.socketCon > 0)
	{
		int slen = write(skt.socketCon, (char *)info, size);
		if (slen > 0)
		{
            printf("label_cam send oK!\n");
            return slen;
		}
		else
		{
			printf("label_cam send fail!\n");
            return -1;
		}
	}
}


/**
  * @brief  发送开始读数命令
  * @param  None
  * @retval None
  */
void label_cam_send_start(void)
{
    label_cam_send_data(g_imglocSocket,label_cam_start_cmd,LABEL_CAM_BUFF_SIZE);
}

/**
  * @brief  发送结束读数命令
  * @param  None
  * @retval None
  */
void label_cam_send_end(void)
{
    label_cam_send_data(g_imglocSocket,label_cam_end_cmd,LABEL_CAM_BUFF_SIZE);
}


int releaseLocalNet()
{
    printf("waiting for exit!\n");
    char *message;
    pthread_join(g_thdAccept, (void *)&message);
    printf("%s\n", message);
    close(g_socketListen);
    return 0;
}

int checkThrIsKill(pthread_t thd)
{
    int res = 1;
    int res_kill = pthread_kill(thd, 0);
    if (res_kill == 0)
    {
        res = 0;
    }
    return res;
}

void sendInfoToLocalNet(_MySocketInfo skt, char info[], int size)
{
	if (skt.socketCon > 0)
	{
		int slen = write(skt.socketCon, (char *)info, size);
		if (slen > 0)
		{
			printf("\nGateway:%s:%d,send info oK!\n", skt.ipaddr, skt.port);
		}
		else
		{
			printf("\nGateway: %s:%d, send info failed!\n", skt.ipaddr, skt.port);
		}
	}
}

int startLocalNetInit()
{
    memset(arrConSocket,0,sizeof(_MySocketInfo)*10);

    printf("Socket start~\n");
    // create one socket for tcp
    g_socketListen = socket(AF_INET, SOCK_STREAM, 0);
    if (g_socketListen < 0)
    {
        printf("Create TCP socket fail!\n");
        exit(-1);
    }
    else
    {
        printf("Create TCP socket ok!\n");
    }
    int opt = 1;
    setsockopt(g_socketListen, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt));
    // init ip addr and port
    struct sockaddr_in server_addr;
    bzero(&server_addr, sizeof(struct sockaddr_in));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(IP_ADDR);
    server_addr.sin_port = htons(SERVER_PORT);
    if (bind(g_socketListen, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) != 0)
    {
        perror("Bind ip addr and port fail!\n");
        exit(-1);
    }
    else
    {
        printf("Bind ip addr and port ok!\n");
    }
    // listening to the port
    if (listen(g_socketListen, 10) != 0)
    {
        printf("Keep listen fail!\n");
        exit(-1);
    }
    else
    {
        printf("Keep listen ok!\n");
    }
    // start to accept
    pthread_create(&g_thdAccept, NULL, thdAcceptHandler, &g_socketListen);

    return 0;
}

void *thdAcceptHandler(void *g_socketListen)
{
    while (1)
    {
        if (thrReceiveClientCount < 4)
        {
            int sockaddr_in_size = sizeof(struct sockaddr_in);
            struct sockaddr_in client_addr;
            int _g_socketListen = *((int *)g_socketListen);
            int socketCon = accept(_g_socketListen, (struct sockaddr *)(&client_addr), (socklen_t *)(&sockaddr_in_size));
            if (socketCon < 0)
            {
                printf("Socket connect fail!\n");
            }
            else
            {
                printf("Socket connect ok, ip: %s:%d\r\n", inet_ntoa(client_addr.sin_addr), client_addr.sin_port);
            }
            printf("Connected socket:%d\n", socketCon);
            // create one thread to deal with one client
            _MySocketInfo socketInfo;
            socketInfo.socketCon = socketCon;
            socketInfo.ipaddr = inet_ntoa(client_addr.sin_addr);
            socketInfo.port = client_addr.sin_port;
            pthread_t thrReceive = 0;
            if (strcmp(socketInfo.ipaddr, GATEWAY_IP_ADDR) == 0)
            {
                g_gatewaySocket = socketInfo;
                pthread_create(&thrReceive, NULL, thdRcvGatewayHandler, &g_gatewaySocket);
            }
            else if (strcmp(socketInfo.ipaddr, BINCAM_IP_ADDR) == 0)
            {
                g_bincamSocket = socketInfo;
                pthread_create(&thrReceive, NULL, thdRcvBinCamHandler, &g_bincamSocket);
            }
            else if (strcmp(socketInfo.ipaddr, IMGLOC_IP_ADDR) == 0)
            {
                g_imglocSocket = socketInfo;
                pthread_create(&thrReceive, NULL, thdRcvBinCamHandler, &g_imglocSocket);
            }
            else
            {
                printf("Err: %s is not in field.\n", socketInfo.ipaddr);
                close(socketCon);
                continue;
            }
            arrConSocket[conClientCount] = socketInfo;
            conClientCount++;
            printf("Connected %d device.\n", conClientCount);

            arrThrReceiveClient[thrReceiveClientCount] = thrReceive;
            thrReceiveClientCount++;
        }
        else
        {
            printf("Received count(%d) is full.\n", thrReceiveClientCount);
        }

        sleep(1);
    }

    char *s = "Exit the accept function";
    pthread_exit(s);
}

void ParseDataForGateway(char chr)
{
    static char chrBuf[100];
    static unsigned char chrCnt = 0;
    unsigned char sData[20];
    _GatewayInfo tmpData;

    chrBuf[chrCnt++] = chr;
    if (chrCnt < 22)
        return;

    if ((chrBuf[0] != 0x8A) || (chrBuf[1] != 0x8B))
    {
        printf("Error:%x %x\r\n", chrBuf[0], chrBuf[1]);
        memcpy(&chrBuf[0], &chrBuf[1], 21);
        chrCnt--;
        return;
    }
    memcpy(&sData[0], &chrBuf[2], 20);

    if ((chrBuf[0] == 0x8A) && (chrBuf[1] == 0x8B))
    {
        tmpData.dist1 = sData[0];
        tmpData.spd1 = sData[1];
        tmpData.dist2 = sData[2];
        tmpData.spd2 = sData[3];
        tmpData.ultra1 = sData[4]; //if (sData[x] * 5) = y cm
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
        for (int i = 0; i < 19; i++)
        {
            sum += (int)sData[i];
        }
        if ((unsigned char)(sum & 0xFF) == sData[19])
        {
            chrCnt = 0;
            printf("\n----------check SUM ok!----------\n");
            // give the data to global data
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
void ParseDataForBinCam(char chr)
{
    static char BinCam_chrBuf[100];
    static unsigned char BinCam_chrCnt = 0;
   // unsigned char sData[100];
    _BinCamInfo tmpData;

    BinCam_chrBuf[BinCam_chrCnt++] = chr;
    if (BinCam_chrCnt < 3)
        return;

    if ((BinCam_chrBuf[0] != 0xFF) || (BinCam_chrBuf[1] != 0xEE))
    {
        printf("Error:%x %x\r\n", BinCam_chrBuf[0], BinCam_chrBuf[1]);
        memcpy(&BinCam_chrBuf[0], &BinCam_chrBuf[1], 2);
        BinCam_chrCnt--;
        return;
    }
    if(BinCam_chrCnt<(BinCam_chrBuf[2]*4+4))
     return;

   //memcpy(&sData[0], &BinCam_chrBuf[2], (BinCam_chrBuf[2]*4+2));
  // memcpy(&sData[0], &BinCam_chrBuf[0], (BinCam_chrBuf[2]*4+4));

    if ((BinCam_chrBuf[0] == 0xFF) && (BinCam_chrBuf[1] == 0xEE))
    {

        int sum = 0;

        for (int i = 0; i < (BinCam_chrBuf[2]*4+3); i++)
        {
            sum += (int)BinCam_chrBuf[i];
        }
       // if ((unsigned char)(sum & 0xFF) == sData[(BinCam_chrBuf[2]*4+1)])
        if ((unsigned char)(sum & 0xFF) == BinCam_chrBuf[(BinCam_chrBuf[2]*4+3)])
        {
            BinCam_chrCnt = 0;
            printf("\n----------check SUM ok!----------\n");
            // give the data to global data

        for (int j = (BinCam_chrBuf[2]*4+3); j < (12*4+3); j++)
        {
            BinCam_chrBuf[j]=0;
        }
            tmpData.head[0]     = BinCam_chrBuf[0];
            tmpData.head[1]     = BinCam_chrBuf[1];
            tmpData.number      = BinCam_chrBuf[2];

            tmpData.classific_1 = BinCam_chrBuf[3];
            tmpData.dist_1      = BinCam_chrBuf[4];
            tmpData.direction_1 = BinCam_chrBuf[5];
            tmpData.width_1     = BinCam_chrBuf[6];

            tmpData.classific_2  = BinCam_chrBuf[7];
            tmpData.dist_2       = BinCam_chrBuf[8];
            tmpData.direction_2  = BinCam_chrBuf[9];
            tmpData.width_2      = BinCam_chrBuf[10];

            tmpData.classific_3  = BinCam_chrBuf[11];
            tmpData.dist_3       = BinCam_chrBuf[12];
            tmpData.direction_3  = BinCam_chrBuf[13];
            tmpData.width_3      = BinCam_chrBuf[14];

            tmpData.classific_4  = BinCam_chrBuf[15];
            tmpData.dist_4       = BinCam_chrBuf[16];
            tmpData.direction_4  = BinCam_chrBuf[17];
            tmpData.width_4      = BinCam_chrBuf[18];

            tmpData.classific_5  = BinCam_chrBuf[19];
            tmpData.dist_5       = BinCam_chrBuf[20];
            tmpData.direction_5  = BinCam_chrBuf[21];
            tmpData.width_5      = BinCam_chrBuf[22];

            tmpData.classific_6  = BinCam_chrBuf[23];
            tmpData.dist_6       = BinCam_chrBuf[24];
            tmpData.direction_6  = BinCam_chrBuf[25];
            tmpData.width_6      = BinCam_chrBuf[26];

            tmpData.classific_7  = BinCam_chrBuf[27];
            tmpData.dist_7       = BinCam_chrBuf[28];
            tmpData.direction_7  = BinCam_chrBuf[29];
            tmpData.width_7      = BinCam_chrBuf[30];

            tmpData.classific_8  = BinCam_chrBuf[31];
            tmpData.dist_8       = BinCam_chrBuf[32];
            tmpData.direction_8  = BinCam_chrBuf[33];
            tmpData.width_8      = BinCam_chrBuf[34];

            tmpData.classific_9  = BinCam_chrBuf[35];
            tmpData.dist_9       = BinCam_chrBuf[36];
            tmpData.direction_9  = BinCam_chrBuf[37];
            tmpData.width_9      = BinCam_chrBuf[38];

            tmpData.classific_10 = BinCam_chrBuf[39];
            tmpData.dist_10      = BinCam_chrBuf[40];
            tmpData.direction_10 = BinCam_chrBuf[41];
            tmpData.width_10     = BinCam_chrBuf[42];

            tmpData.classific_11 = BinCam_chrBuf[43];
            tmpData.dist_11      = BinCam_chrBuf[44];
            tmpData.direction_11 = BinCam_chrBuf[45];
            tmpData.width_11     = BinCam_chrBuf[46];

            tmpData.classific_12 = BinCam_chrBuf[47];
            tmpData.dist_12      = BinCam_chrBuf[48];
            tmpData.direction_12 = BinCam_chrBuf[49];
            tmpData.width_12     = BinCam_chrBuf[50];

            pthread_mutex_lock(&(g_bincam_info_mutex));
            g_bincam_info = tmpData;
            pthread_mutex_unlock(&(g_bincam_info_mutex));
#if USE_TEST_CACHE
            pthread_mutex_lock(&(g_image_tst_data.mutex));
            uint8_t n = g_BinCam_tst_data.used;
            pthread_mutex_unlock(&(g_BinCam_tst_data.mutex));
            int len = g_BinCam_tst_data.len[n];
            if ((g_BinCam_tst_data.cache[n] != NULL) && (len < TEST_SAVE_MIN_LEN)) // 8MB
            {
                g_BinCam_tst_data.cache[n][len] = tmpData;
                g_BinCam_tst_data.len[n]++;
            }
#endif
            return;
        }
    }
    BinCam_chrCnt  = 0;

}

// pending
void ParseDataForImgLoc(char chr)
{
    static char IMG_chrBuf[100];
    static unsigned char IMG_chrCnt = 0;
    unsigned char sData[2];
    _IMGLOCInfo tmpData;

    IMG_chrBuf[IMG_chrCnt++] = chr;
    if (IMG_chrCnt < 4)
        return;

    if ((IMG_chrBuf[0] != 0x9A) || (IMG_chrBuf[1] != 0x9B))
    {
        printf("Error:%x %x\r\n", IMG_chrBuf[0], IMG_chrBuf[1]);
        memcpy(&IMG_chrBuf[0], &IMG_chrBuf[1], 3);
        IMG_chrCnt--;
        return;
    }
    memcpy(&sData[0], &IMG_chrBuf[2], 2);

    if ((IMG_chrBuf[0] == 0x9A) && (IMG_chrBuf[1] == 0x9B))
    {
        tmpData.theate = sData[0];

        int sum = 0x9A + 0x9B;

        for (int i = 0; i < 1; i++)
        {
            sum += (int)sData[i];
        }
        if ((unsigned char)(sum & 0xFF) == sData[1])
        {
            IMG_chrCnt = 0;
            printf("\n----------check SUM ok!----------\n");
            // give the data to global data
            pthread_mutex_lock(&(g_image_info_mutex));
            g_image_info = tmpData;
            pthread_mutex_unlock(&(g_image_info_mutex));
#if USE_TEST_CACHE
            pthread_mutex_lock(&(g_image_tst_data.mutex));
            uint8_t n = g_image_tst_data.used;
            pthread_mutex_unlock(&(g_image_tst_data.mutex));
            int len = g_image_tst_data.len[n];
            if ((g_image_tst_data.cache[n] != NULL) && (len < TEST_SAVE_MIN_LEN)) // 8MB
            {
                g_image_tst_data.cache[n][len] = tmpData;
                g_image_tst_data.len[n]++;
            }
#endif
            return;
        }
    }
    IMG_chrCnt  = 0;

}

void *thdRcvGatewayHandler(void *socketInfo)
{
    char buffer[1024];
    int buffer_length;
#if USE_TEST_CACHE
    for (int i = 0; i < 2; i++)
    {
        g_gwy_tst_data.cache[i] = (_GatewayInfo *)malloc(TEST_CACHE_MAX_LEN * sizeof(_GatewayInfo));
        memset(g_gwy_tst_data.cache[i], 0, TEST_CACHE_MAX_LEN * sizeof(_GatewayInfo));
    }
#endif
    _MySocketInfo _socketInfo = *((_MySocketInfo *)socketInfo);
    while (1)
    {
        bzero(&buffer, sizeof(buffer));

        buffer_length = read(_socketInfo.socketCon, buffer, 1024);
        if (buffer_length == 0)
        {
            printf("%s:%d client closed.\n", _socketInfo.ipaddr, _socketInfo.port);
            conClientCount--;
            break;
        }
        else if (buffer_length < 0)
        {
            printf("Receive data from client fail.\n");
            conClientCount--;
            break;
        }
        printf("%s:%d:len=%d: ", _socketInfo.ipaddr, _socketInfo.port, buffer_length);
#ifdef USE_TEST
        for (int i = 0; i < buffer_length; i++)
        {
            //if (buffer[i] != 0)
            {
                printf("%d\t", buffer[i]);
            }
        }
#endif
        printf("\n");

        if (buffer_length < 22)
            continue;
        for (int i = 0; i < buffer_length; i++)
        {
            ParseDataForGateway(buffer[i]);
        }

#ifdef USE_FEEDBACK
        {
            char userStr[8] = {0x8A, 0x8B, 0x68, 0x65, 0x61, 0x72, 0x74, 0x8E};
            int sendMsg_len = write(_socketInfo.socketCon, userStr, 8);
            if (sendMsg_len > 0)
            {
                printf("\nTo %s:%d, send ok.\n", _socketInfo.ipaddr, _socketInfo.port);
            }
            else
            {
                printf("\nTo %s:%d, send fail.\n", _socketInfo.ipaddr, _socketInfo.port);
            }
        }
#endif
        //sleep(1);
        //usleep(50000); // 50ms
        if (g_stop)
			break;
    }
    close(_socketInfo.socketCon);
    printf("%s, rcvGateway exit.\n", _socketInfo.ipaddr);
    memset(socketInfo, 0, sizeof(_MySocketInfo));

#if USE_TEST_CACHE
    for (int i = 0; i < 2; i++)
    {
        if (g_gwy_tst_data.cache[i] != NULL)
        {
            free(g_gwy_tst_data.cache[i]);
            g_gwy_tst_data.cache[i] = NULL;
        }
    }
#endif

    return NULL;
}

void *thdRcvBinCamHandler(void *socketInfo)
{
    char buffer[1024];
    int buffer_length;

#if USE_TEST_CACHE
    for (int i = 0; i < 2; i++)
    {
        g_BinCam_tst_data.cache[i] = (_BinCamInfo *)malloc(TEST_CACHE_MAX_LEN * sizeof(_BinCamInfo));
        memset(g_BinCam_tst_data.cache[i], 0, TEST_CACHE_MAX_LEN * sizeof(_BinCamInfo));
    }
#endif

    _MySocketInfo _socketInfo = *((_MySocketInfo *)socketInfo);
    while (1)
    {
        bzero(&buffer, sizeof(buffer));

        buffer_length = read(_socketInfo.socketCon, buffer, 1024);
        if (buffer_length == 0)
        {
            printf("%s:%d client closed.\n", _socketInfo.ipaddr, _socketInfo.port);
            conClientCount--;
            break;
        }
        else if (buffer_length < 0)
        {
            printf("Receive data from client fail.\n");
            conClientCount--;
            break;
        }
        printf("%s:%d:len=%d: ", _socketInfo.ipaddr, _socketInfo.port, buffer_length);
#ifdef USE_TEST
        for (int i = 0; i < buffer_length; i++)
        {
            //if (buffer[i] != 0)
            {
                printf("%d\t", buffer[i]);
            }
        }
#endif
        printf("\n");



////////////////////////////////////////////////////////////////////////////cw begin

        // if (buffer_length < 20)
        //     continue;

        for (int i = 0; i < buffer_length; i++)
        {
            ParseDataForBinCam(buffer[i]);
        }
///////////////////////////////////////////////////////////////////////////cw   end

#ifdef USE_FEEDBACK
        {
            char userStr[10] = {0x8A, 0x8B, 0x68, 0x65, 0x61, 0x72, 0x74, 0x8E};
            int sendMsg_len = write(_socketInfo.socketCon, userStr, 10);
            if (sendMsg_len > 0)
            {
                printf("\nTo %s:%d, send ok.\n", _socketInfo.ipaddr, _socketInfo.port);
            }
            else
            {
                printf("\nTo %s:%d, send fail.\n", _socketInfo.ipaddr, _socketInfo.port);
            }
        }
#endif
        sleep(0.2);
    }
    close(_socketInfo.socketCon);
    printf("%s, rcvBinCam exit.\n", _socketInfo.ipaddr);
    memset(socketInfo, 0, sizeof(_MySocketInfo));
    return NULL;
}

void *thdRcvImgLocHandler(void *socketInfo)
{
    char buffer[1024];
    int buffer_length;

#if USE_TEST_CACHE
    for (int i = 0; i < 2; i++)
    {
        g_image_tst_data.cache[i] = (_IMGLOCInfo *)malloc(TEST_CACHE_MAX_LEN * sizeof(_IMGLOCInfo));
        memset(g_image_tst_data.cache[i], 0, TEST_CACHE_MAX_LEN * sizeof(_IMGLOCInfo));
    }
#endif
    _MySocketInfo _socketInfo = *((_MySocketInfo *)socketInfo);
    while (1)
    {
        bzero(&buffer, sizeof(buffer));

        buffer_length = read(_socketInfo.socketCon, buffer, 1024);
        if (buffer_length == 0)
        {
            printf("%s:%d client closed.\n", _socketInfo.ipaddr, _socketInfo.port);
            conClientCount--;
            break;
        }
        else if (buffer_length < 0)
        {
            printf("Receive data from client fail.\n");
            conClientCount--;
            break;
        }
        printf("%s:%d:len=%d: ", _socketInfo.ipaddr, _socketInfo.port, buffer_length);
#ifdef USE_TEST
        for (int i = 0; i < buffer_length; i++)
        {
            //if (buffer[i] != 0)
            {
                printf("%d\t", buffer[i]);
            }
        }
#endif
        printf("\n");

        if (buffer_length < 4)   //////cw
            continue;
        for (int i = 0; i < buffer_length; i++)
        {
            ParseDataForImgLoc(buffer[i]);
        }

#ifdef USE_FEEDBACK
        {
            char userStr[10] = {0x8A, 0x8B, 0x68, 0x65, 0x61, 0x72, 0x74, 0x8E};
            int sendMsg_len = write(_socketInfo.socketCon, userStr, 10);
            if (sendMsg_len > 0)
            {
                printf("\nTo %s:%d, send ok.\n", _socketInfo.ipaddr, _socketInfo.port);
            }
            else
            {
                printf("\nTo %s:%d, send fail.\n", _socketInfo.ipaddr, _socketInfo.port);
            }
        }
#endif
        sleep(0.2);
    }
    close(_socketInfo.socketCon);
    printf("%s, rcvImgLoc fail.\n", _socketInfo.ipaddr);
    memset(socketInfo, 0, sizeof(_MySocketInfo));
    return NULL;
}
