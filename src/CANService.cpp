#include "CANService.h"
#include "driver/twai.h"

CANService canService;


bool CANService::init(long baudrate) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_22, GPIO_NUM_21, TWAI_MODE_NORMAL);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_timing_config_t t_config;

    if (baudrate == 500000) {
        t_config.brp = 4;
        t_config.tseg_1 = 15;
        t_config.tseg_2 = 4;
        t_config.sjw = 3;
        t_config.triple_sampling = false;
    } else if (baudrate == 250000) {
        t_config.brp = 8;
        t_config.tseg_1 = 15;
        t_config.tseg_2 = 4;
        t_config.sjw = 3;
        t_config.triple_sampling = false;
    } else if (baudrate == 125000) {
        t_config.brp = 16;
        t_config.tseg_1 = 15;
        t_config.tseg_2 = 4;
        t_config.sjw = 3;
        t_config.triple_sampling = false;
    } else {
        Serial.println("Unsupported CAN baudrate");
        return false;
    }

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("CAN driver installed");
        if (twai_start() == ESP_OK) {
            Serial.println("CAN started");
            return true;
        }
    }

    Serial.println("CAN init failed");
    return false;
}

void CANService::sendTestData() {
    // 테스트 데이터 전송 비활성화 (실제 시스템에서는 불필요)
    // for (int i = 0; i < 64; ++i) {
    //     txData[i] = i + sendCount;
    // }
    // txIndex = 0;
    // sendCount++;
    // sending = true;
    Serial.println("CANService: Test data transmission disabled for production use");
}

void CANService::loop() {
    unsigned long now = millis();
    static unsigned long last1s = 0;
    static unsigned long lastSend10ms = 0;
    
    // 수신 메시지 확인
    twai_message_t rxMsg;
    if (twai_receive(&rxMsg, 0) == ESP_OK) {
        // 메시지 수신됨 표시
        // Serial.print("+");  // CANService에서 메시지 감지
        
        // 새 메시지 수신됨
        lastRxId = rxMsg.identifier;
        lastRxLength = rxMsg.data_length_code;
        for (int i = 0; i < lastRxLength && i < 8; i++) {
            lastRxData[i] = rxMsg.data[i];
        }
        hasNewMessage = true;
        // CAN 메시지 수신 로그 제거됨
        // Serial.printf("\nCANService: ID=0x%03X, Len=%d\n", lastRxId, lastRxLength);
        
        // Interrupt 콜백 호출 (설정된 경우)
        if (rxInterruptCallback != nullptr) {
            rxInterruptCallback();
        }
    } else {
        // 주기적으로 상태 표시 (1초마다)
        static unsigned long lastStatusTime = 0;
        unsigned long currentTime = millis();
        if (currentTime - lastStatusTime >= 1000) {
            // Serial.print("o");  // CAN 상태 정상 (메시지 없음)
            lastStatusTime = currentTime;
        }
    }
    
}

// CAN 메시지 전송 함수
bool CANService::sendMessage(uint32_t id, uint8_t* data, uint8_t length) {
    if (length > 8) length = 8; // CAN 메시지는 최대 8바이트
    
    twai_message_t msg = {};
    msg.identifier = id;
    msg.data_length_code = length;
    msg.flags = 0;
    
    for (int i = 0; i < length; i++) {
        msg.data[i] = data[i];
    }
    
    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        // Serial.printf("CAN TX: ID=0x%03X, Len=%d\n", id, length);
        return true;
    } else {
        Serial.printf("CAN TX failed: ID=0x%03X, Error=%d\n", id, err);
        return false;
    }
}

// 새 메시지가 있는지 확인
bool CANService::hasMessage() {
    return hasNewMessage;
}

// 마지막 수신 메시지 ID 반환
uint32_t CANService::getLastMessageId() {
    hasNewMessage = false; // 메시지 읽음 표시
    return lastRxId;
}

// 마지막 수신 메시지 데이터 반환
uint8_t* CANService::getLastMessageData() {
    return lastRxData;
}

// 마지막 수신 메시지 길이 반환
uint8_t CANService::getLastMessageLength() {
    return lastRxLength;
}

// Interrupt 콜백 설정 함수
void CANService::setRxInterruptCallback(void (*callback)()) {
    rxInterruptCallback = callback;
}
