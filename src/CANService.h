#ifndef CANSERVICE_H
#define CANSERVICE_H

#include <Arduino.h>
#include <driver/twai.h> // ESP32 TWAI (CAN) 드라이버

class CANService {
public:
    bool init(long baudrate);
    void sendTestData();
    void loop();
    
    // CAN 메시지 송수신 함수들
    bool sendMessage(uint32_t id, uint8_t* data, uint8_t length);
    bool hasMessage();
    uint32_t getLastMessageId();
    uint8_t* getLastMessageData();
    uint8_t getLastMessageLength();
    
    // Interrupt 콜백 설정 함수
    void setRxInterruptCallback(void (*callback)());
    
private:
    unsigned long lastSendTime = 0;
    int sendCount = 0;
    // 전송 상태 관리용 멤버
    uint8_t txData[64] = {0};
    int txIndex = 0;
    bool sending = false;
    
    // 수신 메시지 저장용 멤버
    uint32_t lastRxId = 0;
    uint8_t lastRxData[8] = {0};
    uint8_t lastRxLength = 0;
    bool hasNewMessage = false;
    
    // Interrupt 콜백 함수 포인터
    void (*rxInterruptCallback)() = nullptr;
};

extern CANService canService;

#endif // CANSERVICE_H
