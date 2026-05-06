#ifndef IOEXPANDER_H
#define IOEXPANDER_H

#include <Wire.h>
#include <TCA9534.h>

class IOExpander {
public:
    IOExpander(uint8_t address, TCA9534::Config config, TCA9534::Polarity polarity);
    void begin(TwoWire &wire = Wire);
    void write(uint8_t pin, bool level);
    void writePort(uint8_t value);
    uint8_t readAll();
    uint8_t getConfig();      // Config 레지스터 값 읽기
    uint8_t getPolarity();    // Polarity 레지스터 값 읽기
    uint8_t getOutputShadow() const;
private:
    uint8_t addr;
    TCA9534::Config cfg;
    TCA9534::Polarity pol;
    TCA9534 ioex;
    uint8_t outputShadow;
};

extern IOExpander ioEx_FET;
extern IOExpander ioEx_IO;
extern IOExpander ioEx_Relay;
void TCA9534_Setup(void);
void TCA9534_FET_TestLoop(void);
void Set_TCA9534_LED_ch_Status(uint8_t channel, uint8_t status);
void Set_TCA9534_FET_ch_Status(uint8_t channel, uint8_t status);
void Set_TCA9534_Relay_ch_Status(uint8_t channel, uint8_t status);
uint8_t Get_TCA9534_FET_Status(void);
uint8_t Get_TCA9534_Relay_Status(void);
uint8_t Set_TCA9534_Relay_Status(uint8_t status);
uint8_t Get_IO21_Raw_Port_Status(void);
uint8_t Get_IO21_Output_Port_Status(void);
uint8_t Get_IO21_Config_Status(void);
int Get_Board_ID_Number(void);  // 보드 ID 읽기 함수 추가

#endif // IOEXPANDER_H
