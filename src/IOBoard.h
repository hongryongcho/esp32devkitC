#ifndef IO_BOARD_H
#define IO_BOARD_H

#include <Arduino.h>

class IOBoard {
public:
  int loop_cnt;
  int fet[16];              // i2c to gpio FET control
  int led[4];               // i2c to gpio LED control
  float inter_adc[6];       // internal ADC 6ch (Thermistor) voltage value
  float temperature[8];     // internal ADC 8ch temperature values
  float TC[4];
  int tcloopcnt;
  // Thermocouple 5개 버퍼와 평균값 처리
  float TC_buffer[4][5];    // 4채널 * 5개 버퍼
  int TC_buffer_index[4];   // 각 채널별 버퍼 인덱스
  bool TC_buffer_full[4];   // 각 채널별 버퍼가 가득 찬지 확인
  float TC_average[4];      // 각 채널별 평균값
  float dac[16];
  float freq[2];
  int gpi[2];
  int gpo[2];
  int pwm[2];               // GPIO PWM duty cycle values (0-255)
  float ads1015_adc[8];
  int ads1015_loop_cnt;
  int dac7678_target[8];
  int dac7678_current[8];
  int dac7678_loop_cnt;
  void init();
  void update();
};

extern IOBoard ioBoard;
extern IOBoard ioBoardBackup;

#endif
