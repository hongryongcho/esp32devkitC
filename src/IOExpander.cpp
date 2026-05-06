#include "IOExpander.h"
#include "IOBoard.h"
#include "main.h"  // rSys 사용을 위한 main.h 추가
#include <Arduino.h>

// LED 출력 상태 추적을 위한 전역 변수
static uint8_t led_output_state = 0x00;  // 핀 4,5,6,7의 출력 상태 추적

IOExpander ioEx_FET(0x20, TCA9534::Config::OUT, TCA9534::Polarity::ORIGINAL);
// 0x21: P0-P3 input(Board ID), P4-P7 output => 0x0F
IOExpander ioEx_IO(0x21, static_cast<TCA9534::Config>(0x0F), TCA9534::Polarity::ORIGINAL);
// 새 Relay 디바이스 추가
IOExpander ioEx_Relay(0x22, TCA9534::Config::OUT, TCA9534::Polarity::ORIGINAL);

IOExpander::IOExpander(uint8_t address, TCA9534::Config config, TCA9534::Polarity polarity)
    : addr(address), cfg(config), pol(polarity), outputShadow(0x00) {}

void IOExpander::begin(TwoWire &wire) {
    ioex.attach(wire);
    ioex.setDeviceAddress(addr);
    ioex.config(cfg);
    ioex.polarity(pol);
    outputShadow = ioex.output();
    
    // 0x20 (FET) 의 경우 모든 출력을 0으로 초기화
    if (addr == 0x20) {
        Serial.println("Initializing 0x20 (FET) outputs to 0...");
        outputShadow = 0x00;
        ioex.output(outputShadow);  // 모든 핀을 LOW로 설정
        Serial.println("All FET outputs set to 0");
    }
    
    // 0x22 (Relay) 의 경우 모든 출력을 0으로 초기화 (FET와 동일)
    if (addr == 0x22) {
        Serial.println("Initializing 0x22 (Relay) outputs to 0...");
        outputShadow = 0x00;
        ioex.output(outputShadow);  // 모든 핀을 LOW로 설정
        Serial.println("All Relay outputs set to 0");
    }
    
    // 0x21: 운영 모드 설정 (P0-P3 INPUT, P4-P7 OUTPUT)
    if (addr == 0x21) {
        Serial.println("Forcing 0x21 config to 0x0F (P0-P3 input, P4-P7 output)...");
        Wire.beginTransmission(addr);
        Wire.write(0x03);  // Config register address
        Wire.write(0x0F);  // P0-P3 INPUT, P4-P7 OUTPUT
        uint8_t result = Wire.endTransmission();
        Serial.printf("Direct config write result: %d\n", result);
        delay(10);

        // P4-P7 출력 초기값 LOW 유지
        outputShadow &= 0x0F;
        ioex.output(outputShadow);
    }
    
    Serial.print("IOExpander (0x");
    Serial.print(addr, HEX);
    Serial.println(") initialized:");
    Serial.print("  config  : ");
    Serial.println(ioex.config(), HEX);
    Serial.print("  polarity: ");
    Serial.println(ioex.polarity(), HEX);
}

void IOExpander::write(uint8_t pin, bool level) {
    if (pin > 7) {
        return;
    }

    // shadow 값도 같이 유지
    if (level) {
        outputShadow |= (1 << pin);
    } else {
        outputShadow &= ~(1 << pin);
    }

    // 단일 비트 API의 라이브러리 동작 차이를 피하기 위해
    // shadow 기반으로 포트 전체를 다시 써서 다른 채널 상태를 보존한다.
    ioex.output(outputShadow);
}

void IOExpander::writePort(uint8_t value) {
    outputShadow = value;
    ioex.output(outputShadow);
}

uint8_t IOExpander::getOutputShadow() const {
    return outputShadow;
}
uint8_t IOExpander::readAll() {
    return ioex.input();
}

uint8_t IOExpander::getConfig() {
    return ioex.config();
}

uint8_t IOExpander::getPolarity() {
    return ioex.polarity();
}

void TCA9534_Setup(void)
{
    // Wire.begin(19, 18); // setup in main (SDA=19, SCL=18)
    ioEx_FET.begin(Wire);
    ioEx_IO.begin(Wire);
    ioEx_Relay.begin(Wire);
    ioBoard.loop_cnt=0;
    
    // TCA9534 설정 확인 및 부트 검증 루프
    delay(100); // 초기화 후 안정화 시간

    Serial.println("=====================================");
}

uint8_t Get_TCA9534_FET_Status(void)
{
    uint8_t status = ioEx_FET.readAll();
    Serial.print("IOExpander(0x20) FET status: ");
    Serial.println(status, BIN);
    return status;
}
uint8_t Get_TCA9534_Relay_Status(void)
{
    uint8_t status = ioEx_Relay.readAll();
    Serial.print("IOExpander(0x22) Relay status: ");
    Serial.println(status, BIN);
    return status;
}
uint8_t Get_TCA9534_INPUT_Status(void)
{
    uint8_t status = ioEx_IO.readAll();
    Serial.print("IOExpander(0x21) INPUT status: ");
    Serial.println(status, BIN);
    return status;
}


// 0x21 Input 레지스터 raw 8비트 읽기
uint8_t Get_IO21_Raw_Port_Status(void)
{
    uint8_t status = 0x00;

    while (Wire.available()) {
        Wire.read();
    }

    Wire.beginTransmission(0x21);
    Wire.write(0x00);  // Input register address
    Wire.endTransmission(false);  // repeated start
    delay(2);

    if (Wire.requestFrom(0x21, 1) > 0) {
        status = Wire.read();
    } else {
        Serial.println("[ERROR] Failed to read Input register from 0x21");
        status = 0xFF;
    }

    return status;
}

// 0x21 Output 레지스터 raw 8비트 읽기
uint8_t Get_IO21_Output_Port_Status(void)
{
    uint8_t status = 0x00;

    while (Wire.available()) {
        Wire.read();
    }

    Wire.beginTransmission(0x21);
    Wire.write(0x01);  // Output register address
    Wire.endTransmission(false);  // repeated start
    delay(2);

    if (Wire.requestFrom(0x21, 1) > 0) {
        status = Wire.read();
    } else {
        Serial.println("[ERROR] Failed to read Output register from 0x21");
        status = 0xFF;
    }

    return status;
}

// 0x21 Config 레지스터 raw 8비트 읽기
uint8_t Get_IO21_Config_Status(void)
{
    uint8_t status = 0x00;

    while (Wire.available()) {
        Wire.read();
    }

    Wire.beginTransmission(0x21);
    Wire.write(0x03);  // Config register address
    Wire.endTransmission(false);  // repeated start
    delay(2);

    if (Wire.requestFrom(0x21, 1) > 0) {
        status = Wire.read();
    } else {
        Serial.println("[ERROR] Failed to read Config register from 0x21");
        status = 0xFF;
    }

    return status;
}


// 보드 ID 읽기 함수 (핀 0,1,2,3에서 4비트 값)
int Get_Board_ID_Number(void)
{
    uint8_t status = Get_IO21_Raw_Port_Status();
    int board_id = status & 0x0F;  // 하위 4비트만 추출 (핀 0,1,2,3)
    return board_id;
}
void Set_TCA9534_FET_ch_Status(uint8_t channel, uint8_t status)
{
    if(channel > 7) {
        Serial.println("Invalid channel for Set_TCA9534_FET_ch_Status");
        return ; 
    }
    ioEx_FET.write(channel, status & 0x01);
    return;
}

void Set_TCA9534_Relay_ch_Status(uint8_t channel, uint8_t status)
{
    if(channel > 7) {
        Serial.println("Invalid channel for Set_TCA9534_Relay_ch_Status");
        return ; 
    }
    ioEx_Relay.write(channel, status & 0x01);
    return;
}

void Set_TCA9534_LED_ch_Status(uint8_t channel, uint8_t status)
{
    if(channel > 3) {
        Serial.println("Invalid channel for Set_TCA9534_LED_ch_Status");
        return;
    }
    
    // LED 상태 업데이트
    if (status & 0x01) {
        led_output_state |= (1 << channel);  // 해당 비트 설정
    } else {
        led_output_state &= ~(1 << channel); // 해당 비트 클리어
    }

    // LED0-3은 P4-P7에 1:1 매핑
    uint8_t output = ioEx_IO.getOutputShadow();
    uint8_t pin = channel + 4;
    if (status & 0x01) {
        output |= (1 << pin);
    } else {
        output &= ~(1 << pin);
    }
    ioEx_IO.writePort(output);

    return;
}

uint8_t Set_TCA9534_Relay_Status(uint8_t status)
{
    ioEx_Relay.writePort(status);
    return Get_TCA9534_Relay_Status();
}
uint8_t Set_TCA9534_OUTPUT_Status(uint8_t channel, uint8_t status)
{
    if(channel ==4){
        if(status & 0x01) {
            ioEx_IO.write(4, 1); // Set GPO4 high
        } else {
            ioEx_IO.write(4, 0); // Set GPO4 low
        }
    }
    else if(channel ==5){
        if(status & 0x01) {
            ioEx_IO.write(5, 1); // Set GPO5 high
        } else {
            ioEx_IO.write(5, 0); // Set GPO5 low
        }
    }
    else if(channel ==6){
        if(status & 0x01) {
            ioEx_IO.write(6, 1); // Set GPO6 high
        } else {
            ioEx_IO.write(6, 0); // Set GPO6 low
        }
    }
    else if(channel ==7){
        if(status & 0x01) {
            ioEx_IO.write(7, 1); // Set GPO7 high
        } else {
            ioEx_IO.write(7, 0); // Set GPO7 low
        }
    }
    else {
        Serial.println("Invalid channel for Set_TCA9534_OUTPUT_Status");
        return 0xFF; // Error code
    }
    return Get_TCA9534_INPUT_Status();
}

void TCA9534_FET_TestLoop(void)
{
    int dir;
    int channel;
    channel = ioBoard.loop_cnt%8;
    ioBoard.loop_cnt++;
    if(ioBoard.loop_cnt<8) dir =0;
    else if(ioBoard.loop_cnt<16) {
        dir=1;
    }
    else ioBoard.loop_cnt=0;
    ioEx_FET.write(channel, dir);
    uint8_t inputs = ioEx_IO.readAll();
    // Serial.print("IOExpander(0x21) inputs (0~3): ");
    // Serial.println(inputs & 0x0F, BIN);
}
