#include "ADS1015Manager.h"
#include "IOBoard.h"
#include "main.h"

ADS1015Manager ads1015Manager;

void ADS1015Manager::init() {
    for (int i = 0; i < 2; i++) {
        if (!ads[i].begin(ads_addresses[i])) {
            Serial.print("Failed to initialize ADS1015 at address ");
            Serial.println(ads_addresses[i], HEX);
            // while (1);
            return;
        }
        
        // PGA 설정 (±6.144V 범위로 설정 - 0V~5V 측정용)
        ads[i].setGain(GAIN_TWOTHIRDS);  // ±6.144V range (1 bit = 3mV)
        
        Serial.print("ADS1015 initialized at address ");
        Serial.print(ads_addresses[i], HEX);
        Serial.println(" with ±6.144V range (0-5V measurement)");
    }
}

void ADS1015Manager::readAllChannels() {
    static unsigned long lastDebugTime = 0;
    bool shouldDebug = (millis() - lastDebugTime > 5000);  // 5초마다 디버그
    
    for (int chipIndex = 0; chipIndex < 2; chipIndex++) {
        for (int channel = 0; channel < 4; channel++) {
            // Raw ADC 값 읽기
            int16_t rawValue = ads[chipIndex].readADC_SingleEnded(channel);
            
            // ±6.144V 범위에서 전압 변환 (GAIN_TWOTHIRDS 설정)
            // ADS1015는 12비트이지만 상위 12비트만 사용 (16비트 값의 상위 12비트)
            // ±6.144V 범위에서 1 LSB = 3mV
            float voltage = rawValue * 3.0 / 1000.0;  // mV to V 변환
            
            // 음수 값 방지 (단방향 측정, 0V~5V)
            if (voltage < 0) voltage = 0;
            
            // 디버깅 정보 출력 (5초마다, 첫 번째 채널만)
            // if (shouldDebug && channel == 0) {
            //     Serial.printf("ADS1015[0x%02X][%d]: Raw=%d(0x%04X) Voltage=%.3fV\n", 
            //                 ads_addresses[chipIndex], channel, rawValue, rawValue, voltage);
            // }
            
            ioBoard.ads1015_adc[chipIndex * 4 + channel] = voltage;
        }
    }
    
    if (shouldDebug) {
        lastDebugTime = millis();
    }
}

void ADS1015Manager::updateData() {
    unsigned long currentTime = millis();
    if (currentTime - lastReadTime >= READ_INTERVAL) {
        readAllChannels();
        lastReadTime = currentTime;
    }
}

float ADS1015Manager::getVoltage(int chipIndex, int channel) {
    if (chipIndex < 0 || chipIndex >= 2 || channel < 0 || channel >= 4) {
        Serial.println("Invalid ADS1015 channel index");
        return -1.0;
    }
    return ioBoard.ads1015_adc[chipIndex * 4 + channel];
}
