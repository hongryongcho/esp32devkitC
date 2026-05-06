#include "DAC7678Manager.h"
#include <Wire.h>
#include "IOBoard.h"
#include "main.h"

DAC7678Manager dac7678Manager;

void DAC7678Manager::init() {
    // Wire.begin(19, 18); // setup in main (SDA=19, SCL=18)
    lastUpdateTime = 0;
    
    // DAC7678 초기화 확인
    // Serial.println("DAC7678 초기화 시작...");
    
    // 모든 채널을 0으로 초기화
    for (uint8_t ch = 0; ch < 8; ch++) {
        writeDAC(ch, 0);
        delay(10);  // 채널 초기화 간 10ms 대기
    }
    
    // Serial.println("DAC7678 초기화 완료 - 모든 채널 0V로 설정");
}
void DAC7678Manager::updateOutputs() {
  static unsigned long lastLogTime = 0;
  static const unsigned long LOG_INTERVAL = 1000; // 1초마다 로그 출력
  
  unsigned long currentTime = millis();
  bool shouldLog = (currentTime - lastLogTime >= LOG_INTERVAL);
  
  if (shouldLog) {
    // Serial.print("DAC7678 Update: ");
    lastLogTime = currentTime;
  }
  
  bool anyChannelUpdated = false;
  
  for (int channel = 0; channel < 8; channel++) {
    int targetValue = ioBoard.dac7678_target[channel];
    int currentValue = ioBoard.dac7678_current[channel];
    
    if (targetValue != currentValue) {
      // 점진적 변화 (5% 스텝)
      int step = max(1, abs(targetValue - currentValue) / 20);
      if (targetValue > currentValue) {
        currentValue = min(targetValue, currentValue + step);
      } else {
        currentValue = max(targetValue, currentValue - step);
      }
      
      // DAC에 값 쓰기
      if (writeDAC(channel, currentValue)) {
        ioBoard.dac7678_current[channel] = currentValue;
        anyChannelUpdated = true;
        
        // 1초마다 로그 출력
        if (shouldLog) {
          // uint16_t dacValue = map(currentValue, 0, 500, 0, 4095);
          // Serial.printf("CH%d:%d->%d ", channel, 
          //               ioBoard.dac7678_target[channel], currentValue);
        }
      } else {
        // 에러 발생 시에만 즉시 출력 (재시도 없이 에러 코드 확인)
        Wire.beginTransmission(DAC7678_ADDRESS);
        uint8_t error = Wire.endTransmission();
        Serial.printf("DAC7678 I2C Error CH%d: %d\n", channel, error);
      }
    }
  }
  
  if (shouldLog && anyChannelUpdated) {
    // Serial.println();
  }
}
bool DAC7678Manager::writeDAC(uint8_t channel, int value) {
  if (channel >= 8) return false;
  
  // 값 범위 제한 (0 ~ 500, 즉 0.00V ~ 5.00V)
  if (value < 0) value = 0;
  if (value > 500) value = 500;
  
  // 0-500 범위를 0-4095 DAC 값으로 변환
  uint16_t dacValue = map(value, 0, 500, 0, 4095);

  // CH0~CH3 drive 특성: 5V=STOP, 0V=FULL 이므로 raw 12-bit 값을 반전한다.
  if (channel <= 3) {
    dacValue = (uint16_t)(4095 - dacValue);
  }
  
  // 최대 3번 재시도
  for (int retry = 0; retry < 3; retry++) {
    Wire.beginTransmission(DAC7678_ADDRESS);
    
    // 커맨드 바이트: 채널 선택 + 업데이트 모드
    // uint8_t command = (channel << 4) | 0x00;
    // Wire.write(command);
    uint8_t command = 0x30|(channel&0x0f);
    Wire.write(command);
    
    // 데이터 바이트 (12비트 값을 상위 바이트부터)
    // Wire.write((dacValue >> 8) & 0xFF);  // 상위 8비트
    // Wire.write(dacValue & 0xFF);         // 하위 8비트
    Wire.write((dacValue >> 4) & 0xFF);  // 상위 8비트
    Wire.write((dacValue<<4) & 0xF0);         // 하위 8비트
    
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      return true;  // 성공
    }
    
    // 재시도 전 지연
    delay(1);
  }
    // 3회 모두 실패하면 플래그 세팅
  g_i2cError = true;
  return false;  // 3번 시도 후 실패
}

int DAC7678Manager::calculateGradualChange(int current, int target) {
    if (current == target) return current;
    int diff = target - current;
    int step = (abs(diff) > 5) ? 5 : 1;
    if (diff > 0) return current + step;
    else return current - step;
}


 