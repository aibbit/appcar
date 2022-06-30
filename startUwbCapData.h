#ifndef __START_UWB_CAP_DATA_H__
#define __START_UWB_CAP_DATA_H__

typedef struct comUwbData {
    double x;
    double y;
} _UwbData;

void startComInitForUWB();

_UwbData get_uwb_data(void);
void save_uwb_data(_UwbData data, char *filename, int namesize);

#endif // __START_UWB_CAP_DATA_H__
