#ifndef _UTILS_H
#define _UTILS_H

uint16_t CRC_Table(uint8_t *ptr, uint16_t len);
uint8_t cmdSum(uint8_t *arr, int len);

void float2byte(float numeric,uint8_t* returnData);
float byte2float(uint8_t *str);

#endif //_UTILS_H
