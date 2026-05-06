#ifndef DAC7678MANAGER_H
#define DAC7678MANAGER_H

#include <Arduino.h>

class DAC7678Manager {
public:
    void init();
    void updateOutputs();
    bool writeDAC(uint8_t channel, int value);
    int calculateGradualChange(int current, int target);
private:
    static const uint8_t DAC7678_ADDRESS = 0x4C;
    unsigned long lastUpdateTime = 0;
    static const unsigned long UPDATE_INTERVAL = 100;
    static const uint8_t CMD_WRITE_UPDATE_ALL = 0x30;
    static const uint8_t CMD_WRITE_UPDATE_N = 0x10;
    static const uint16_t DAC_MAX_VALUE = 0xFFFF;
    static const int VOLTAGE_MAX_X100 = 500;
};

extern DAC7678Manager dac7678Manager;

#endif // DAC7678MANAGER_H
