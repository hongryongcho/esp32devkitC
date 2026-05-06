#ifndef ADS1015MANAGER_H
#define ADS1015MANAGER_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

class ADS1015Manager {
public:
    void init();
    void readAllChannels();
    void updateData();
    float getVoltage(int chipIndex, int channel);
private:
    Adafruit_ADS1015 ads[2];
    uint8_t ads_addresses[2] = {0x49, 0x48};  // ch0,1,2,3=0x49, ch4,5,6,7=0x48
    unsigned long lastReadTime = 0;
    const unsigned long READ_INTERVAL = 100;
    int currentChip = 0;
    int currentChannel = 0;
};

extern ADS1015Manager ads1015Manager;

#endif // ADS1015MANAGER_H
