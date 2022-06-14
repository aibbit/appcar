#ifndef __START_LOCAL_NET_CAP_DATA_H__
#define __START_LOCAL_NET_CAP_DATA_H__


typedef struct MySocketInfo
{
    int socketCon;
    char *ipaddr;
    uint16_t port;
} _MySocketInfo;

typedef struct GatewayInfo
{
    uint8_t head[2];
    uint8_t dist1;
    uint8_t spd1;
    uint8_t dist2;
    uint8_t spd2;
    uint8_t ultra1;
    uint8_t ultra2;
    uint8_t ultra3;
    uint8_t ultra4;
    uint8_t ultra5;
    uint8_t ultra6;
    uint8_t ultra7;
    uint8_t ultra8;
    uint8_t ultra_sts;
    uint8_t illumi_pwr;
    uint8_t box1_sts;
    uint8_t box2_sts;
    uint8_t box3_sts;
    uint8_t yawH;
    uint8_t yawL;
} _GatewayInfo;

typedef struct IMGLOCInfo
{
    uint8_t head[2];
    uint8_t theta;
} _IMGLOCInfo;

typedef struct BinCamInfo
{
    uint8_t head[2];
    uint8_t number;
    uint8_t classific_1;
    uint8_t dist_1;
    uint8_t direction_1;
    uint8_t width_1;
    uint8_t classific_2;
    uint8_t dist_2;
    uint8_t direction_2;
    uint8_t width_2;
    uint8_t classific_3;
    uint8_t dist_3;
    uint8_t direction_3;
    uint8_t width_3;
    uint8_t classific_4;
    uint8_t dist_4;
    uint8_t direction_4;
    uint8_t width_4;
    uint8_t classific_5;
    uint8_t dist_5;
    uint8_t direction_5;
    uint8_t width_5;
    uint8_t classific_6;
    uint8_t dist_6;
    uint8_t direction_6;
    uint8_t width_6;
    uint8_t classific_7;
    uint8_t dist_7;
    uint8_t direction_7;
    uint8_t width_7;
    uint8_t classific_8;
    uint8_t dist_8;
    uint8_t direction_8;
    uint8_t width_8;
    uint8_t classific_9;
    uint8_t dist_9;
    uint8_t direction_9;
    uint8_t width_9;
    uint8_t classific_10;
    uint8_t dist_10;
    uint8_t direction_10;
    uint8_t width_10;
    uint8_t classific_11;
    uint8_t dist_11;
    uint8_t direction_11;
    uint8_t width_11;
    uint8_t classific_12;
    uint8_t dist_12;
    uint8_t direction_12;
    uint8_t width_12;
} _BinCamInfo;

typedef struct RV3399Info
{
    uint8_t head[2];
    uint8_t symbol;
    uint8_t theta_gyro;
    uint8_t theta_cam;
} _RV3399Info;

#if USE_TEST_CACHE

typedef struct testBinCamCache
{
    int len[2];
    _BinCamInfo *cache[2];
    uint8_t used;
    pthread_mutex_t mutex;
} _TestBinCamCache;


typedef struct testIMGLOCCache
{
    int len[2];
    _IMGLOCInfo *cache[2];
    uint8_t used;
    pthread_mutex_t mutex;
} _TestIMGLOCCache;


typedef struct testGateWayCache
{
    int len[2];
    _GatewayInfo *cache[2];
    uint8_t used;
    pthread_mutex_t mutex;
} _TestGateWayCache;

#endif

int startLocalNetInit();
int releaseLocalNet();

void sendInfoToLocalNet(_MySocketInfo, char[], int size);


void label_cam_send_start(int8_t x,int8_t y);//向标识摄像头发送开始命令
void label_cam_send_heart(void);//向标识摄像头发送心跳
void label_cam_send_end(void);//向标识摄像头发送结束命令

void send_control_cmd(float angle,float speed);//向网关发送控制命令(角度,速度)

#endif // __START_LOCAL_NET_CAP_DATA_H__
