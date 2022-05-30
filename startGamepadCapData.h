#ifndef __START_GAMEPAD_CAP_DATA_H__
#define __START_GAMEPAD_CAP_DATA_H__

typedef struct comGampadkey {
  uint8_t voltage;
  uint8_t lrk_h;
  uint8_t lrk_v;
  uint8_t rrk_h;
  uint8_t rrk_v;
  uint16_t key;
} _ComGpdKey;

void startComInitForGamePad();

#endif // __START_GAMEPAD_CAP_DATA_H__
