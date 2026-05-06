#include "IOBoard.h"

IOBoard ioBoard;
IOBoard ioBoardBackup;

void IOBoard::init() {
  for (int i = 0; i < 16; i++) {
    fet[i] = 0;
    dac[i] = 0.0;
  }
  for (int i = 0; i < 8; i++) temperature[i] = 0.0;
  for (int i = 0; i < 4; i++) {
    TC[i] = 0.0;
    led[i] = 0;
    // Thermocouple 버퍼 초기화
    TC_buffer_index[i] = 0;
    TC_buffer_full[i] = false;
    TC_average[i] = 0.0;
    for (int j = 0; j < 5; j++) {
      TC_buffer[i][j] = 0.0;
    }
  }
  for (int i = 0; i < 2; i++) freq[i] = 0.0;
  for (int i = 0; i < 2; i++) {
    gpi[i] = 0;
    gpo[i] = 1;
  }
  for (int i = 0; i < 8; i++) {
    ads1015_adc[i] = 0.0;
    dac7678_target[i] = 0;
    dac7678_current[i] = 0;
  }
  for (int i = 0; i < 2; i++) pwm[i] = 0;  // PWM duty cycle 초기화 (0-255)
  tcloopcnt = 0;
  loop_cnt = 0;
  ads1015_loop_cnt = 0;
  dac7678_loop_cnt = 0;
}

void IOBoard::update() {
  // adc[0] += 0.01;
  // if (adc[0] > 10.0) adc[0] = 0.0;
}
