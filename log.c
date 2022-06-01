/**
日志打印示例.
使用:
Log(DEBUG, "This is debug info\n");
结果:
[2018-07-22 23:37:27:172] [DEBUG] [main.cpp:5] This is debug info
默认打印当前时间,文件名称,行号.
*/
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
//#include <sys/time.h>
#include <time.h>

#include "log.h"

#ifndef LOGLEVEL
#define LOGLEVEL DEBUG
#endif

#ifndef _USE_LOG_ECHO
#define _USE_LOG_ECHO
#endif

// #ifndef _USE_LOG_ECHO_COLOR
// #define _USE_LOG_ECHO_COLOR
// #endif

#ifndef _USE_LOG_FILE
#define _USE_LOG_FILE
#endif

// 使用了GNU C扩展语法,只在gcc(C语言)生效,
// g++的c++版本编译不通过
static const char *s_loginfo[] = {
    [ERROR] = "ERROR",
    [WARN] = "WARN",
    [INFO] = "INFO",
    [DEBUG] = "DEBUG",
};

static void get_timestamp(char *buffer) {
  time_t t;
  struct tm *p;

  int len;

  t = time(NULL);
  p = localtime(&t);

  // struct timeval tv;
  // int millsec;
  // gettimeofday(&tv, NULL);
  // millsec = (int)(tv.tv_usec / 1000);
  // /* 时间格式:[2011-11-15 12:47:34:888] */
  // len = snprintf(buffer, 32, "[%04d-%02d-%02d %02d:%02d:%02d:%03d] ",
  //                p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, p->tm_hour,
  //                p->tm_min, p->tm_sec, millsec);

  //精确到秒
  len = snprintf(buffer, 32, "[%04d/%02d/%02d %02d:%02d:%02d]",
                 p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, p->tm_hour,
                 p->tm_min, p->tm_sec);

  buffer[len] = '\0';
}

void save_log(char *buf) {
  FILE *fp = NULL;
  char file_name[256] = {0};
  char time[32] = {0};

  sprintf(file_name, "CarLog.log");
  get_timestamp(time);

  if ((fp = fopen(file_name, "a+")) != NULL) {
    fprintf(fp, "%s", buf);
    fclose(fp);
  }
}

void log_record(const char *filename, int line, ERROR_LEVEL level,
                const char *fmt, ...) {
  if (level > LOGLEVEL)
    return;

  va_list arg_list;
  char buf[1024];
  memset(buf, 0, 1024);

  va_start(arg_list, fmt);
  vsnprintf(buf, 1024, fmt, arg_list);
  char time[32] = {0};

  // 去掉*可能*存在的目录路径,只保留文件名
  const char *tmp = strrchr(filename, '/');
  if (!tmp)
    tmp = filename;
  else
    tmp++;

  get_timestamp(time);

  char string[1024];
  memset(string, 0, 1024);
  sprintf(string, "%s[%s][%s:%d]%s\n", time, s_loginfo[level], tmp, line, buf);

  va_end(arg_list);

#ifdef _USE_LOG_ECHO_COLOR
  switch (level) {
  case DEBUG:
    printf("\033[1;33m%s\n\033[0m", string); //黄色
    break;
  case INFO:
    printf("\033[1;32m%s\n\033[0m", string); //绿色
    break;
  case ERROR:
    printf("\033[1;31m%s\n\033[0m", string); //红色
    break;
  case WARN:
    printf("\033[1;34m%s\n\033[0m", string); //蓝色
    break;
  }
#endif //_USE_LOG_ECHO_COLOR

#ifdef _USE_LOG_ECHO
    printf("%s", string);
#endif //_USE_LOG_ECHO

#ifdef _USE_LOG_FILE
  save_log(string);
#endif //_USE_LOG_FILE
}
