#ifndef _LOG_H
#define _LOG_H

typedef enum {
    ERROR = 1,
    WARN  = 2,
    INFO  = 3,
    DEBUG = 4,
  } ERROR_LEVEL;

void log_record(const char* filename, int line, ERROR_LEVEL level, const char* fmt, ...) __attribute__((format(printf,4,5)));

#define Log(level, format, ...) log_record(__FILE__, __LINE__, level, format, ## __VA_ARGS__)

#endif //_LOG_H
