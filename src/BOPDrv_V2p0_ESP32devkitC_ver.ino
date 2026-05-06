#include "main.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Adafruit_MAX31856.h>
#include <Preferences.h>
#include "batagota.h"

// 빌드 정보 매크로
#define COMPILE_DATE __DATE__
#define COMPILE_TIME __TIME__
#define BUILD_TIMESTAMP COMPILE_DATE " " COMPILE_TIME
#define FIRMWARE_VERSION "v2.0.1"
#define COMPILER_INFO "ESP32-Arduino"

// 빌드 타임스탬프 함수 선언 (정의는 구조체 이후에)
uint32_t getBuildTimestamp32();

// CAN 통신 설정
#define CAN_MESSAGE_COUNT 10  // CAN 메시지 개수 (빌드 타임스탬프 추가로 1개 증가)
#define CAN_MESSAGE_SIZE 8   // 각 CAN 메시지 크기 (바이트)
#define CAN_PACKET_SIZE (CAN_MESSAGE_COUNT * CAN_MESSAGE_SIZE)  // 총 패킷 크기: 80바이트

void setup();
void loop();
void setPWMValue(int channel, int value);  // PWM 값 설정 함수
void LED_ch2_Toggle(void);
void LED_ch1_Toggle(void);
void beepON(uint8_t count, uint8_t index);
void beepResetAndStart(uint8_t count, uint8_t index);
void UpdateBuzzerDriver100mSec(void);
void Buzzer_Test_Loop_100mSec(void);
void initInternalADC_AllChannels();  // SLAVE 모드용 8채널 ADC 초기화
// void handleCANSlaveCommunication();  // SLAVE 모드 CAN 통신 처리 (기존 polling 방식 - 주석 처리됨)
void sendCANSlaveResponse();         // SLAVE CAN 응답 전송
void handleCANMasterCommunication(); // MASTER 모드 CAN 통신 처리
void sendCANMasterData();            // MASTER CAN 데이터 전송
bool handleCANMasterReceive();       // MASTER CAN 응답 수신 (응답 여부 반환)
void processCANCommands();           // CAN 메시지 기반 명령 처리 서비스
bool loadCanCommEnableFlag();        // CAN 통신 enable 플래그 로드
void saveCanCommEnableFlag(bool enabled); // CAN 통신 enable 플래그 저장
void setCanCommEnable(bool enabled, const char* reason, bool persist); // CAN 통신 enable 상태 적용
bool loadMqttLogEnableFlag();        // MQTT 로그 enable 플래그 로드
void saveMqttLogEnableFlag(bool enabled); // MQTT 로그 enable 플래그 저장
void setMqttLogEnable(bool enabled, const char* reason, bool persist); // MQTT 로그 enable 상태 적용
// bool shouldStartTransmission(uint32_t lastReceivedId); // SLAVE 전송 타이밍 체크 (기존 polling 방식 - 주석 처리됨)

// 새로운 Interrupt 기반 CAN 함수들 (선언만)
void IRAM_ATTR canRxInterruptHandler();    // CAN RX interrupt 핸들러
// void processCAN_RX_Interrupt();            // CAN RX interrupt 처리 - canPacketManager 이후에 정의
// void handleCANSlaveInterruptBased();       // Interrupt 기반 CAN Slave 통신 - canPacketManager 이후에 정의
bool hasNewBoardData(uint8_t boardId);     // 보드 데이터 수신 확인
// static 함수는 선언 없이 정의에서 바로 사용 - readCANBoardData()
unsigned long getBoardLastRxTime(uint8_t boardId); // 보드 마지막 수신 시간
void markBoardDataProcessed(uint8_t boardId); // 보드 데이터 처리 완료 표시
void selectThermoChannel(uint8_t channel); // 써모커플 채널 선택
void verifyMAX31856Configuration(); // MAX31856 설정 확인
void applyOperationModeTemperatureLimits(); // mode별 temperature limit 적용
void printTemperatureLimitBootInfo(); // 부팅 시 mode/temperature limit 출력


// ESP32 내부 ADC 핀 정의 (8채널) - Wi-Fi 충돌 방지를 위해 ADC1만 사용
const int ADC_PINS[8] = {32, 33, 34, 35, 36, 39, 0, 0};  // ADC2 핀들 제거 (GPIO27, GPIO12는 Wi-Fi와 충돌)
const bool ADC_VALID[8] = {true, true, true, true, true, true, false, false};  // 유효한 ADC 채널 표시
const float ADC_REFERENCE_VOLTAGE = 3.3;  // ESP32 기준 전압
const int ADC_RESOLUTION = 4096;  // 12비트 ADC (2^12 = 4096)

// ============================================================================
// NTC Thermistor 설정 (10K NTC, 10K 풀업, 3.9K 병렬, 12V 전원, Diode 클램프)
// ============================================================================
// 회로 구성:
//   12V ─── [10K 풀업] ─┬─ ADC 입력 (Diode로 3.3V 클램프)
//                       │
//            [10K NTC] [3.9K 병렬]
//                 [||]
//                       │
//                      GND
const float THERMISTOR_SUPPLY_VOLTAGE = 12.0;      // 전원 전압 (V)
const float THERMISTOR_PULLUP_RESISTANCE = 10000.0; // 풀업 저항 (10K)
const float THERMISTOR_SERIES_RESISTANCE = 3900.0; // 병렬 저항 (3.9K)
const float THERMISTOR_NOMINAL_RESISTANCE = 10000.0; // 25°C에서 저항값 (10K)
const float THERMISTOR_NOMINAL_TEMPERATURE = 25.0 + 273.15;  // 기준 온도 (298.15K)
const float THERMISTOR_BETA_VALUE = 3950.0;     // Beta 값 (10K NTC 일반값)
const float ABSOLUTE_ZERO = 273.15;             // 절대영도 (K → °C 변환)
const float ADC_CLAMP_VOLTAGE = 3.3;            // Diode 클램프 전압 (3.3V)

// ESP32 내부 DAC 핀 정의 (2채널)
const int DAC_PINS[2] = {25, 26};  // DAC1=GPIO25, DAC2=GPIO26
const int DAC_RESOLUTION = 256;    // 8비트 DAC (2^8 = 256)
const float DAC_REFERENCE_VOLTAGE = 3.3;  // ESP32 기준 전압
const float DAC_CHANGE_THRESHOLD = 0.05;  // 5% 변화 임계값

// GPIO 입력 핀 정의 (4채널)
const int GPIO_OUT_PINS[2] = {16, 4};   // GPIO 출력 핀 (2채널)
const int GPIO_IN_PINS[2] = {23, 17};   // GPIO 입력 핀 (2채널)

// 주파수 측정용 변수들
volatile unsigned long pulseCount[2] = {0, 0};  // 각 핀의 펄스 카운트

// CAN 통신 타이밍 제어 변수들
unsigned long lastCANRequest = 0;      // 마지막 CAN 요청 시간
unsigned long canResponseDelay = 50;   // CAN 응답 지연 시간 (50ms)
bool pendingCANResponse = false;       // CAN 응답 대기 플래그
unsigned long lastFreqMeasure[2] = {0, 0};      // 마지막 측정 시간
unsigned long freqMeasureInterval = 1000;       // 1초마다 측정

// CAN MASTER 모드 전용 변수들
unsigned long lastCANMasterSend = 0;           // 마지막 MASTER 전송 시간
unsigned long canMasterInterval = 1000;        // 1초마다 전송
unsigned long lastCANReceived = 0;             // 마지막 CAN 수신 시간
unsigned long canMasterWaitTime = 100;         // 마지막 수신 후 100ms 대기
bool canMasterTransmitting = false;            // MASTER 전송 중 플래그
unsigned long masterTransmitStartTime = 0;     // MASTER 전송 시작 시간

// CAN MASTER 타임아웃 관리 변수들
int canMasterFailCount = 0;                    // 연속 실패 횟수
const int MAX_CAN_FAIL_COUNT = 10;             // 최대 실패 허용 횟수
bool canMasterSuspended = false;               // CAN 전송 중지 상태
unsigned long canMasterSuspendTime = 0;        // 전송 중지 시작 시간
const unsigned long CAN_RETRY_INTERVAL = 30000; // 30초 후 재시도
bool canSlaveResponseReceived[16] = {false};   // 각 Slave 보드별 응답 상태

// CAN 통신 Enable 제어 (기본값: 비활성)
bool g_canCommEnabled = false;
int canMasterTxFailCount = 0;                  // 실제 Tx 실패 누적 횟수
const int MAX_CAN_TX_FAIL_COUNT = 10;          // Tx 실패 임계값
Preferences g_canPreferences;
const char* CAN_PREF_NAMESPACE = "batagota";
const char* CAN_PREF_KEY_ENABLE = "can_en";

// MQTT 로그 Enable 제어
bool g_mqttLogEnabled = true;  // 기존 동작 호환을 위해 기본값 ENABLED
const char* MQTT_PREF_KEY_LOG_ENABLE = "mqtt_log_en";

// 패킷 완성도 추적용 변수들은 구조체 정의 이후에 선언됩니다
// uint8_t packetMessageCount[16] = {0};          // 각 SLAVE별 수신된 메시지 개수
// bool packetMessageReceived[16][CAN_MESSAGE_COUNT] = {false};   // 각 SLAVE별 메시지 수신 상태 (0~(CAN_MESSAGE_COUNT-1))
// unsigned long packetStartTime[16] = {0};       // 각 SLAVE별 패킷 시작 시간

// SLAVE 모드 전용 변수들  
unsigned long lastSlaveTransmit = 0;           // 마지막 SLAVE 전송 시간
unsigned long slaveResponseDelay = 50;         // SLAVE 응답 지연 시간 (50ms)
bool waitingToTransmit = false;                // SLAVE 전송 대기 플래그
unsigned long transmitWaitStartTime = 0;       // 전송 대기 시작 시간

// SLAVE 데이터 관련 변수들 (구조체 정의 이후로 이동됨)
// CANSensorPacket_t slaveData[16];               // 각 채널별 센서 데이터
bool slaveDataValid[16] = {false};             // 각 채널별 데이터 유효성
unsigned long slaveDataUpdateTime[16] = {0};   // 각 채널별 마지막 업데이트 시간

// 시스템 전역 변수
system_t rSys;  // 시스템 정보 (보드 ID 등)

// 전송 완료 플래그들
bool canTransmissionCompleted = false;         // 자신의 모든 메시지 전송 완료 플래그
unsigned long lastServiceCheckTime = 0;       // 마지막 서비스 체크 시간

bool g_i2cError = false;           // 전역 정의

// Relay 출력 상태(8ch) - TCA9534_Update()에서 변경분만 반영
int relayStatus[8] = {0};
int relayBackup[8] = {0};

// Batagota 제어 알고리즘 인스턴스
Batagota batagota;


// CAN 데이터 패킷 구조체 정의
#pragma pack(push, 1)  // 1바이트 정렬로 패킹

// CAN 전송용 센서 데이터 구조체 (총 76바이트 - 빌드 타임스탬프 4바이트 추가)
typedef struct {
    // === Header (4바이트) ===
    uint8_t board_id;           // Board ID (1바이트)
    uint8_t packet_type;        // 패킷 타입 (0=센서데이터, 1=제어명령)
    uint8_t sequence;           // 시퀀스 번호 (0-255)
    uint8_t data_length;        // 데이터 길이 (헤더 제외)
    
    // === FET 상태 (1바이트) ===
    uint8_t fet_status;         // FET 8채널 (각 비트가 1채널)
    
    // === ADC 데이터 (16바이트: 8ch × 2바이트) ===
    uint16_t internal_adc[8];   // 내부 ADC 8채널 (0-4095 → 12비트)
    
    // === External ADC 데이터 (16바이트: 8ch × 2바이트) ===
    uint16_t external_adc[8];   // ADS1015 외부 ADC 8채널
    
    // === DAC 데이터 (18바이트) ===
    uint16_t internal_dac[2];   // 내부 DAC 2채널 (0-4095)
    uint16_t external_dac[8];   // DAC7678 외부 DAC 8채널 (0-4095)
    
    // === GPIO 데이터 (2바이트) ===
    uint8_t gpio_input;         // GPIO 입력 2채널 (비트 0-1)
    uint8_t gpio_output;        // GPIO 출력 2채널 (비트 0-1)
    
    // === 주파수 데이터 (4바이트: 2ch × 2바이트) ===
    uint16_t input_freq[2];     // 입력 주파수 2채널 (Hz 단위)
    
    // === PWM 데이터 (2바이트) ===
    uint8_t output_pwm[2];      // 출력 PWM 2채널 (0-255)
    
    // === Thermocouple 데이터 (8바이트: 4ch × 2바이트) ===
    int16_t thermocouple[4];    // 온도 4채널 (0.1도 단위, -3276.8~+3276.7도, 실제 사용: -1200~+1200도)
    
    // === 빌드 정보 (4바이트) ===
    uint32_t build_timestamp;   // 빌드 타임스탬프 (압축된 32비트)
    
    // === 체크섬 (1바이트) ===
    uint8_t checksum;           // 데이터 무결성 검증용
    
} CANSensorPacket_t;

#pragma pack(pop)  // 패킹 해제

// CAN 순차 전송을 위한 구조체
struct CANTransmissionState {
    bool isTransmitting;              // 전송 중 플래그
    CANSensorPacket_t packetData;     // 전송할 패킷 데이터 (포인터 대신 실제 데이터)
    uint32_t baseCanId;               // 기본 CAN ID
    int currentMessageIndex;          // 현재 메시지 인덱스
    int totalMessages;                // 총 메시지 수
    unsigned long lastTransmitTime;   // 마지막 전송 시간
    unsigned long transmitInterval;   // 전송 간격 (10ms)
};

// 전역 변수 선언 (구조체 정의 이후)
CANSensorPacket_t masterData;
CANSensorPacket_t slaveData[16];               // 각 채널별 센서 데이터
uint8_t packetMessageCount[16] = {0}; // 각 슬레이브의 수신된 메시지 수 (0~15)
bool packetMessageReceived[16][CAN_MESSAGE_COUNT] = {false}; // 각 메시지 수신 여부
unsigned long packetStartTime[16] = {0}; // 패킷 시작 시간

// 전역 CAN 전송 상태
CANTransmissionState canTxState = {false, {}, 0, 0, 0, 0, 10};

// CAN Interrupt 기반 수신 시스템을 위한 구조체와 변수들
#define MAX_CAN_BOARDS 16  // 최대 보드 수 (Board ID 0~15)

// 각 보드별 수신 데이터 저장 구조체
struct CANRxBoardData {
    bool hasNewData;                  // 새 데이터 수신 플래그
    unsigned long lastRxTime;         // 마지막 수신 시간
    uint32_t lastRxId;               // 마지막 수신된 CAN ID
    CANSensorPacket_t sensorData;    // 수신된 센서 데이터
    uint8_t messageCount;            // 수신된 메시지 개수 (80byte = CAN_MESSAGE_COUNT개 메시지)
    uint8_t rxBuffer[72];            // 임시 수신 버퍼
};

// 빌드 타임스탬프를 32비트로 변환하는 함수 (컴파일 시간에 계산됨)
uint32_t getBuildTimestamp32() {
    // 간단한 해시 방식으로 빌드 타임스탬프를 32비트로 압축
    // 형식: YYYYMMDDHHMMSS를 32비트로 인코딩
    // 예: 2025년 7월 29일 14:30:15 → 20250729143015를 압축
    const char* date = __DATE__;  // "Jul 29 2025"
    const char* time = __TIME__;  // "14:30:15"
    
    // 월 이름을 숫자로 변환
    int month = 1;
    if (strncmp(date, "Jan", 3) == 0) month = 1;
    else if (strncmp(date, "Feb", 3) == 0) month = 2;
    else if (strncmp(date, "Mar", 3) == 0) month = 3;
    else if (strncmp(date, "Apr", 3) == 0) month = 4;
    else if (strncmp(date, "May", 3) == 0) month = 5;
    else if (strncmp(date, "Jun", 3) == 0) month = 6;
    else if (strncmp(date, "Jul", 3) == 0) month = 7;
    else if (strncmp(date, "Aug", 3) == 0) month = 8;
    else if (strncmp(date, "Sep", 3) == 0) month = 9;
    else if (strncmp(date, "Oct", 3) == 0) month = 10;
    else if (strncmp(date, "Nov", 3) == 0) month = 11;
    else if (strncmp(date, "Dec", 3) == 0) month = 12;
    
    int day = atoi(&date[4]);
    int year = atoi(&date[7]);
    int hour = atoi(&time[0]);
    int minute = atoi(&time[3]);
    int second = atoi(&time[6]);
    
    // 32비트로 압축: YYYYMMDDHHMMSS 형태를 비트로 압축
    // 년도(12비트) + 월(4비트) + 일(5비트) + 시(5비트) + 분(6비트)
    return ((year & 0xFFF) << 20) | ((month & 0xF) << 16) | ((day & 0x1F) << 11) | ((hour & 0x1F) << 6) | (minute & 0x3F);
}

// 빌드 타임스탬프 디코딩 함수
String decodeBuildTimestamp(uint32_t timestamp) {
    int year = ((timestamp >> 20) & 0xFFF);
    int month = ((timestamp >> 16) & 0xF);
    int day = ((timestamp >> 11) & 0x1F);
    int hour = ((timestamp >> 6) & 0x1F);
    int minute = (timestamp & 0x3F);
    
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d", 
             year, month, day, hour, minute);
    return String(buffer);
}

// CAN Interrupt TX 제어 구조체
struct CANTxControl {
    bool txFlag;                     // TX 시작 플래그
    bool waitingToTx;                // TX 대기 중 플래그
    unsigned long lastRxTime;        // 마지막 수신 시간 (50ms 타이밍용)
    uint8_t prevBoardId;            // 이전 보드 ID (나보다 1 작은 보드)
    unsigned long txDelayTime;       // TX 지연 시간 (50ms)
};

// 전역 변수들
volatile CANRxBoardData canRxData[MAX_CAN_BOARDS]; // 각 보드별 수신 데이터
volatile CANTxControl canTxControl = {false, false, 0, 0, 50}; // TX 제어
volatile bool canInterruptFlag = false;            // CAN interrupt 발생 플래그

bool loadCanCommEnableFlag() {
  if (!g_canPreferences.begin(CAN_PREF_NAMESPACE, false)) {
    Serial.println("[CAN CFG] NVS open failed. Default=DISABLED");
    return false;
  }

  // 기본값은 false(DISABLED)
  bool enabled = g_canPreferences.getBool(CAN_PREF_KEY_ENABLE, false);
  g_canPreferences.end();
  return enabled;
}

void saveCanCommEnableFlag(bool enabled) {
  if (!g_canPreferences.begin(CAN_PREF_NAMESPACE, false)) {
    Serial.println("[CAN CFG] NVS open failed. Save skipped");
    return;
  }

  g_canPreferences.putBool(CAN_PREF_KEY_ENABLE, enabled);
  g_canPreferences.end();
}

void setCanCommEnable(bool enabled, const char* reason, bool persist) {
  g_canCommEnabled = enabled;

  // 전송 상태 초기화/정지
  canMasterTxFailCount = 0;
  canMasterFailCount = 0;
  canMasterTransmitting = false;
  canTxState.isTransmitting = false;
  canTxState.currentMessageIndex = 0;
  memset(canSlaveResponseReceived, false, sizeof(canSlaveResponseReceived));

  if (enabled) {
    canMasterSuspended = false;
    lastCANMasterSend = millis();
  } else {
    canMasterSuspended = true;
  }

  if (persist) {
    saveCanCommEnableFlag(enabled);
  }

  Serial.printf("[CAN CFG] CAN communication %s (reason=%s, persist=%s)\n",
          enabled ? "ENABLED" : "DISABLED",
          reason,
          persist ? "YES" : "NO");
}

bool loadMqttLogEnableFlag() {
  if (!g_canPreferences.begin(CAN_PREF_NAMESPACE, false)) {
    Serial.println("[MQTT CFG] NVS open failed. Default=ENABLED");
    return true;
  }

  bool enabled = g_canPreferences.getBool(MQTT_PREF_KEY_LOG_ENABLE, true);
  g_canPreferences.end();
  return enabled;
}

void saveMqttLogEnableFlag(bool enabled) {
  if (!g_canPreferences.begin(CAN_PREF_NAMESPACE, false)) {
    Serial.println("[MQTT CFG] NVS open failed. Save skipped");
    return;
  }

  g_canPreferences.putBool(MQTT_PREF_KEY_LOG_ENABLE, enabled);
  g_canPreferences.end();
}

void setMqttLogEnable(bool enabled, const char* reason, bool persist) {
  g_mqttLogEnabled = enabled;

  if (persist) {
    saveMqttLogEnableFlag(enabled);
  }

  Serial.printf("[MQTT CFG] MQTT log %s (reason=%s, persist=%s)\n",
                enabled ? "ENABLED" : "DISABLED",
                reason,
                persist ? "YES" : "NO");
}

// CAN interrupt 처리 함수 (빠른 처리용)
void IRAM_ATTR canRxInterruptHandler() {
    canInterruptFlag = true;
    // ISR에서는 최소한의 작업만 수행
    // Serial.print("!");  // CAN interrupt 발생 표시
}

// CAN 수신 데이터 처리 함수 (canPacketManager 사용하지 않으므로 여기에 남겨둠)
void processCAN_RX_Interrupt() {
    if (!canService.hasMessage()) {
        // 메시지가 없는 경우에도 표시
        // Serial.print("x");  // 메시지 없음 표시
        return;
    }
    
    // 메시지 수신됨 표시
    // Serial.print(".");  // 메시지 수신 표시
    
    unsigned long currentTime = millis();
    uint32_t receivedId = canService.getLastMessageId();
    uint8_t* data = canService.getLastMessageData();
    uint8_t length = canService.getLastMessageLength();
    
    // 수신된 메시지 정보 출력
    // Serial.printf("\nCAN RX: ID=0x%03X, Len=%d, Data=[", receivedId, length);
    // for (int i = 0; i < length; i++) {
    //     Serial.printf("%02X", data[i]);
    //     if (i < length - 1) Serial.print(" ");
    // }
    // Serial.println("]");
    
    // Board ID 추출 수정
    uint8_t boardId = 0;
    uint8_t messageIndex = 0;
    bool validId = false;
    
    // MASTER 메시지 범위: 0x100~0x108
    if (receivedId >= 0x100 && receivedId <= 0x108) {
        boardId = 0;  // MASTER = Board 0
        messageIndex = receivedId - 0x100;  // 0~(CAN_MESSAGE_COUNT-1)
        validId = true;
        // Serial.printf("CAN RX: MASTER message - Board ID=%d, Message Index=%d\n", boardId, messageIndex);
    }
    // SLAVE 메시지 범위: 0x200~0x208 (Board 1), 0x300~0x308 (Board 2), etc.
    else if (receivedId >= 0x200 && receivedId <= 0x1F08) {  // SLAVE 16까지: 0x200 + 15*0x100 + 8 = 0x1F08
        boardId = (receivedId - 0x200) / 0x100 + 1;  // Board 1~16
        messageIndex = (receivedId - 0x200) % 0x100;  // 0~(CAN_MESSAGE_COUNT-1)
        
        // Message Index 범위 체크
        if (messageIndex <= (CAN_MESSAGE_COUNT - 1)) {
            validId = true;
            // Serial.printf("CAN RX: SLAVE message - Board ID=%d, Message Index=%d\n", boardId, messageIndex);
        } else {
            // Serial.printf("CAN RX: Invalid SLAVE message index %d (Board %d)\n", messageIndex, boardId);
        }
    }
    else {
        // Serial.printf("CAN RX: Unknown ID range 0x%03X (expected: 0x100-0x108 or 0x200-0x1F08)\n", receivedId);
        return;
    }
    
    if (!validId) {
        return;
    }
    
    // 자기 자신의 메시지는 무시
    if (boardId == rSys.hw_bd_id_num) {
        // Serial.printf("CAN RX: Ignoring own message from Board %d\n", boardId);
        return;
    }
    
    // Serial.printf("CAN RX: Extracted Board ID=%d from ID=0x%03X\n", boardId, receivedId);
    
    // 유효한 Board ID 범위 체크
    if (boardId >= MAX_CAN_BOARDS) {
        // Serial.printf("CAN RX: Invalid Board ID %d (max=%d)\n", boardId, MAX_CAN_BOARDS);
        return;
    }
    
    // Serial.printf("CAN RX: Message index=%d for Board %d\n", messageIndex, boardId);
    
    // 수신 데이터를 해당 보드 버퍼에 저장
    int bufferOffset = messageIndex * 8;
    int copyLength = (length > 8) ? 8 : length;
    
    memcpy((void*)&canRxData[boardId].rxBuffer[bufferOffset], data, copyLength);
    canRxData[boardId].lastRxTime = currentTime;
    canRxData[boardId].lastRxId = receivedId;
    canRxData[boardId].messageCount++;
    
    // Serial.printf("CAN RX: Stored %d bytes at offset %d for Board %d (msg count=%d)\n", 
    //               copyLength, bufferOffset, boardId, canRxData[boardId].messageCount);
    
    // 모든 메시지 수신 완료 시 센서 데이터 구조체로 복사
    if (canRxData[boardId].messageCount >= CAN_MESSAGE_COUNT) {
        memcpy((void*)&canRxData[boardId].sensorData, 
               (void*)canRxData[boardId].rxBuffer, 
               sizeof(CANSensorPacket_t));
        canRxData[boardId].hasNewData = true;
        canRxData[boardId].messageCount = 0; // 다음 수신을 위해 리셋
    }
    
    // TX 타이밍 체크 (나보다 1 작은 보드에서 전송 완료 시)
    uint8_t myBoardId = rSys.hw_bd_id_num;
    uint8_t prevBoardId = (myBoardId > 0) ? (myBoardId - 1) : 15; // Board 0의 이전은 Board 15
    
    if (boardId == prevBoardId && canRxData[boardId].messageCount == 0) {
        // 이전 보드의 전송이 완료됨 - TX 플래그 설정
        canTxControl.txFlag = true;
        canTxControl.waitingToTx = true;
        canTxControl.lastRxTime = currentTime;
        canTxControl.prevBoardId = prevBoardId;
    }
}

// 수신된 보드 데이터 접근 함수들
bool hasNewBoardData(uint8_t boardId) {
    if (boardId >= MAX_CAN_BOARDS) return false;
    return canRxData[boardId].hasNewData;
}

static CANSensorPacket_t* readCANBoardData(uint8_t boardId) {
    if (boardId >= MAX_CAN_BOARDS) return nullptr;
    canRxData[boardId].hasNewData = false; // 읽음 플래그 클리어
    return (CANSensorPacket_t*)&canRxData[boardId].sensorData;
}

unsigned long getBoardLastRxTime(uint8_t boardId) {
    if (boardId >= MAX_CAN_BOARDS) return 0;
    return canRxData[boardId].lastRxTime;
}

// 패킷 완성도 관리 함수들
void resetSlavePacket(uint8_t slaveIndex) {
    if (slaveIndex >= 16) return;
    
    packetMessageCount[slaveIndex] = 0;
    for (int i = 0; i < CAN_MESSAGE_COUNT; i++) {
        packetMessageReceived[slaveIndex][i] = false;
    }
    packetStartTime[slaveIndex] = 0;
    slaveDataValid[slaveIndex] = false;
    Serial.printf("SLAVE %d: Packet reset\n", slaveIndex + 1);
}

bool isPacketComplete(uint8_t slaveIndex) {
    if (slaveIndex >= 16) return false;
    
    // CAN_MESSAGE_COUNT개 메시지가 모두 수신되었는지 확인
    for (int i = 0; i < CAN_MESSAGE_COUNT; i++) {
        if (!packetMessageReceived[slaveIndex][i]) {
            return false;
        }
    }
    return true;
}

void checkPacketTimeouts() {
    unsigned long currentTime = millis();
    
    for (int i = 0; i < 16; i++) {
        // 패킷 시작 후 5초 타임아웃
        if (packetStartTime[i] > 0 && 
            (currentTime - packetStartTime[i]) > 5000) {
            Serial.printf("SLAVE %d: Packet timeout (%d/%d messages)\n", 
                         i + 1, packetMessageCount[i], CAN_MESSAGE_COUNT);
            resetSlavePacket(i);
        }
        
        // 완성된 패킷의 노후화 체크 (30초)
        if (slaveDataValid[i] && 
            (currentTime - slaveDataUpdateTime[i]) > 30000) {
            Serial.printf("SLAVE %d: Data expired - reset\n", i + 1);
            slaveDataValid[i] = false;
            canSlaveResponseReceived[i] = false;
        }
    }
}

// CAN 패킷 관리 클래스
class CANPacketManager {
private:
    uint8_t sequenceCounter;
    
    // 체크섬 계산 함수
    uint8_t calculateChecksum(const CANSensorPacket_t* packet) {
        uint8_t checksum = 0;
        uint8_t* data = (uint8_t*)packet;
        int length = sizeof(CANSensorPacket_t) - 1; // checksum 필드 제외
        
        for (int i = 0; i < length; i++) {
            checksum ^= data[i];
        }
        return checksum;
    }
    
public:
    CANPacketManager() : sequenceCounter(0) {}
    
    // 센서 데이터를 CAN 패킷으로 변환
    void createSensorPacket(CANSensorPacket_t* packet);
    
    // CAN 패킷에서 센서 데이터 추출
    bool parseSensorPacket(const CANSensorPacket_t* packet);
    
    // 순차 CAN 메시지 전송 시작 (10ms 간격)
    bool startSequentialTransmission(uint32_t canId, const CANSensorPacket_t* packet);
    
    // 순차 전송 처리 (loop에서 호출)
    void handleSequentialTransmission();
    
    // 전송 상태 확인
    bool isTransmissionActive() { return canTxState.isTransmitting; }
    
    // 전송 완료 확인
    bool isTransmissionComplete() { 
        return !canTxState.isTransmitting && canTxState.currentMessageIndex > 0; 
    }
    
    // 전송 중단
    void stopTransmission() { canTxState.isTransmitting = false; }
    
    // 다중 CAN 메시지로 분할 전송 (64바이트 초과 시)
    void sendPacketAsMultipleCANMessages(uint32_t canId, const CANSensorPacket_t* packet);
};

// 전역 CAN 패킷 매니저 인스턴스
CANPacketManager canPacketManager;

// canPacketManager를 사용하는 Interrupt 기반 CAN Slave 통신 처리 함수
void handleCANSlaveInterruptBased() {
    unsigned long currentTime = millis();
    
    // CAN interrupt 플래그 체크 및 처리
    if (canInterruptFlag) {
        canInterruptFlag = false;
        processCAN_RX_Interrupt();
    } else {
        // interrupt가 없는 경우 주기적으로 표시
        static unsigned long lastNoIntTime = 0;
        if (currentTime - lastNoIntTime >= 3000) {  // 3초마다
            Serial.print("_");  // interrupt 없음 표시
            lastNoIntTime = currentTime;
        }
    }
    
    // canService.loop()도 호출하여 메시지 처리
    canService.loop();
    
    // TX 대기 중이고 50ms가 지났다면 전송 시작
    if (canTxControl.waitingToTx && canTxControl.txFlag) {
        if ((currentTime - canTxControl.lastRxTime) >= canTxControl.txDelayTime) {
            // 센서 데이터 생성 및 전송
            CANSensorPacket_t sensorPacket;
            canPacketManager.createSensorPacket(&sensorPacket);
            
            // SLAVE ID 계산 수정: Board 1 → 0x200, Board 2 → 0x300, etc.
            uint32_t myCanId;
            if (rSys.hw_bd_id_num == 0) {
                myCanId = 0x100;  // MASTER: 0x100~0x108
            } else {
                myCanId = 0x200 + ((rSys.hw_bd_id_num - 1) * 0x100);  // SLAVE: 0x200, 0x300, etc.
            }
            
            // Serial.printf("\nCAN SLAVE %d: Starting TX after Board %d (50ms delay)\n", 
            //              rSys.hw_bd_id_num, canTxControl.prevBoardId);
            
            if (canPacketManager.startSequentialTransmission(myCanId, &sensorPacket)) {
                canTxControl.txFlag = false;
                canTxControl.waitingToTx = false;
                // 전송 시작됨
            } else {
                // 전송 시작 실패
            }
        }
    }
}

// 인터럽트 핸들러 함수들
void IRAM_ATTR pulseCounterISR0() {
  pulseCount[0]++;
}

void IRAM_ATTR pulseCounterISR1() {
  pulseCount[1]++;
}

// ADC 초기화 함수
void initInternalADC() {
  // ADC 핀들을 입력으로 설정 (유효한 채널만)
  for (int i = 0; i < 8; i++) {
    if (ADC_VALID[i]) {
      pinMode(ADC_PINS[i], INPUT);
    }
  }
  // ADC 해상도 설정 (12비트)
  analogReadResolution(12);
  // ADC 감쇠 설정 (0-3.3V 범위)
  analogSetAttenuation(ADC_11db);
  Serial.println("Internal ADC initialized with 6 valid channels (ADC1 only)");
  Serial.print("Valid ADC pins: ");
  for (int i = 0; i < 8; i++) {
    if (ADC_VALID[i]) {
      Serial.print(ADC_PINS[i]);
      Serial.print(" ");
    }
  }
  Serial.println();
  Serial.println("Note: GPIO27 and GPIO12 skipped to avoid Wi-Fi conflict");
}

// DAC 초기화 함수
void initInternalDAC() {
  // DAC 핀들을 출력으로 설정
  for (int i = 0; i < 2; i++) {
    pinMode(DAC_PINS[i], OUTPUT);
  }
  // DAC 해상도 설정 (8비트)
  // analogWriteResolution(8);  // ESP32에서는 자동으로 8비트
  Serial.println("Internal DAC initialized with 2 channels");
  Serial.print("DAC pins: ");
  for (int i = 0; i < 2; i++) {
    Serial.print(DAC_PINS[i]);
    Serial.print(",");
  }
  Serial.println();
}

// SLAVE 모드용 8채널 ADC 초기화 함수 (WiFi 충돌 무시)
void initInternalADC_AllChannels() {
  // 8채널 모두 활성화 (WiFi 없는 SLAVE 모드에서는 충돌 없음)
  const int ADC_PINS_ALL[8] = {32, 33, 34, 35, 36, 39, 27, 12};  // 모든 ADC 핀 사용
  
  // ADC 핀들을 입력으로 설정 (8채널 모두)
  for (int i = 0; i < 8; i++) {
    pinMode(ADC_PINS_ALL[i], INPUT);
  }
  // ADC 해상도 설정 (12비트)
  analogReadResolution(12);
  // ADC 감쇠 설정 (0-3.3V 범위)
  analogSetAttenuation(ADC_11db);
  Serial.println("Internal ADC initialized with 8 channels (SLAVE mode - WiFi disabled)");
  Serial.print("ADC pins: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(ADC_PINS_ALL[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("Note: All ADC channels active (including GPIO27, GPIO12)");
}

// GPIO 입출력 초기화 함수
void initGPIO() {
  WiFiConfig& cfg = configManager.getConfig();
  
  Serial.println("GPIO initialization with modes:");
  
  // GPIO 출력 핀 초기화 (모드별)
  for (int i = 0; i < 2; i++) {
    if (cfg.gpo_mode[i] == GPO_PWM) { // PWM 모드
      // PWM 채널 설정 (8비트, 1kHz)
      if (!ledcAttachChannel(GPIO_OUT_PINS[i], 1000, 8, i)) {
        Serial.printf("  GPIO%d (OUT%d): PWM attach failed\n", GPIO_OUT_PINS[i], i);
      }
      ledcWriteChannel(i, 0); // 초기값 0% duty
      Serial.printf("  GPIO%d (OUT%d): PWM mode (1kHz, 8bit)\n", GPIO_OUT_PINS[i], i);
    } else { // GPO 모드 (기본)
      pinMode(GPIO_OUT_PINS[i], OUTPUT);
      digitalWrite(GPIO_OUT_PINS[i], HIGH); // 초기값 HIGH
      Serial.printf("  GPIO%d (OUT%d): GPO mode\n", GPIO_OUT_PINS[i], i);
    }
  }
  
  // GPIO 입력 핀 초기화 (모드별)
  for (int i = 0; i < 2; i++) {
    if (cfg.gpi_mode[i] == GPI_FREQ) { // FREQ 모드
      pinMode(GPIO_IN_PINS[i], INPUT);  // 풀업 없이 설정 (주파수 측정용)
      // 인터럽트 설정 (상승 에지에서 카운트)
      if (i == 0) {
        attachInterrupt(digitalPinToInterrupt(GPIO_IN_PINS[i]), pulseCounterISR0, RISING);
      } else {
        attachInterrupt(digitalPinToInterrupt(GPIO_IN_PINS[i]), pulseCounterISR1, RISING);
      }
      Serial.printf("  GPIO%d (IN%d): FREQ mode with interrupt\n", GPIO_IN_PINS[i], i);
    } else { // GPI 모드 (기본)
      pinMode(GPIO_IN_PINS[i], INPUT_PULLUP);
      Serial.printf("  GPIO%d (IN%d): GPI mode (pullup)\n", GPIO_IN_PINS[i], i);
    }
  }
  
  Serial.println("GPIO initialization complete");
}

// GPIO 입력 읽기 함수
void readGPIOInputs() {
  WiFiConfig& cfg = configManager.getConfig();
  unsigned long currentTime = millis();
  
  for (int i = 0; i < 2; i++) {
    if (cfg.gpi_mode[i] == GPI_FREQ) { // FREQ 모드
      // 1초마다 주파수 계산
      if (currentTime - lastFreqMeasure[i] >= freqMeasureInterval) {
        // 주파수 = 펄스 수 / 측정 시간(초)
        float frequency = (float)pulseCount[i] * 1000.0 / freqMeasureInterval;
        ioBoard.freq[i] = frequency;
        
        // 카운터 리셋
        pulseCount[i] = 0;
        lastFreqMeasure[i] = currentTime;
      }
      // GPI 값은 0으로 설정 (FREQ 모드에서는 사용 안함)
      ioBoard.gpi[i] = 0;
    } else { // GPI 모드 (기본)
      int inputValue = digitalRead(GPIO_IN_PINS[i]);
      // 풀업 저항 사용으로 버튼 눌렸을 때 LOW가 됨 (반전)
      ioBoard.gpi[i] = (inputValue == LOW) ? 1 : 0;
      // FREQ 값은 0으로 설정 (GPI 모드에서는 사용 안함)
      ioBoard.freq[i] = 0.0;
    }
  }
}

// GPIO 출력 업데이트 함수 (백업 값과 비교하여 변경 시에만 처리)
void updateGPIOOutputs() {
  WiFiConfig& cfg = configManager.getConfig();
  static unsigned long lastLogTime = 0;
  static const unsigned long LOG_INTERVAL = 1000; // 1초마다 로그 출력
  
  unsigned long currentTime = millis();
  bool shouldLog = (currentTime - lastLogTime >= LOG_INTERVAL);
  bool hasChanged = false;
  
  for (int i = 0; i < 2; i++) {
    int targetValue = ioBoard.gpo[i];
    int currentValue = ioBoardBackup.gpo[i];
    
    // 값이 변경되었을 때만 출력 업데이트
    if (targetValue != currentValue) {
      if (cfg.gpo_mode[i] == GPO_PWM) { // PWM 모드
        // gpo 값을 PWM duty로 사용 (0-255 범위)
        int pwmValue = constrain(targetValue, 0, 255);
        ledcWriteChannel(i, pwmValue);
        ioBoard.pwm[i] = pwmValue;  // PWM 값을 ioBoard에 기록
        if (shouldLog) {
          Serial.printf("GPIO OUT%d (GPIO%d) PWM: %d->%d (%.1f%%) ", 
                        i, GPIO_OUT_PINS[i], currentValue, targetValue, 
                        (targetValue * 100.0 / 255.0));
        }
      } else { // GPO 모드 (기본)
        digitalWrite(GPIO_OUT_PINS[i], targetValue ? HIGH : LOW);
        ioBoard.pwm[i] = 0;  // GPO 모드에서는 PWM 값을 0으로 설정
        if (shouldLog) {
          Serial.printf("GPIO OUT%d (GPIO%d) GPO: %d->%d ", 
                        i, GPIO_OUT_PINS[i], currentValue, targetValue);
        }
      }
      
      ioBoardBackup.gpo[i] = targetValue;
      hasChanged = true;
    }
  }
  
  if (shouldLog && hasChanged) {
    lastLogTime = currentTime;
    Serial.println();
  }
}

// GPIO 상태 출력 함수 (1초마다 호출)
void printGPIOValues() {
  WiFiConfig& cfg = configManager.getConfig();
  
  Serial.println("=== GPIO Status ===");
  
  // 출력 핀 상태
  Serial.print("OUT: ");
  for (int i = 0; i < 2; i++) {
    if (cfg.gpo_mode[i] == GPO_PWM) { // PWM 모드
      Serial.printf("GPIO%d(PWM)=%d(%.1f%%) ", 
                    GPIO_OUT_PINS[i], ioBoard.gpo[i], 
                    (ioBoard.gpo[i] * 100.0 / 255.0));
    } else { // GPO 모드
      Serial.printf("GPIO%d(GPO)=%d ", GPIO_OUT_PINS[i], ioBoard.gpo[i]);
    }
  }
  Serial.println();
  
  // 입력 핀 상태
  Serial.print("IN:  ");
  for (int i = 0; i < 2; i++) {
    if (cfg.gpi_mode[i] == GPI_FREQ) { // FREQ 모드
      Serial.printf("GPIO%d(FREQ)=%.2fHz ", GPIO_IN_PINS[i], ioBoard.freq[i]);
    } else { // GPI 모드
      Serial.printf("GPIO%d(GPI)=%d ", GPIO_IN_PINS[i], ioBoard.gpi[i]);
    }
  }
  Serial.println("==================");
}

// 내부 ADC 읽기 함수
// ============================================================================
// NTC Thermistor 온도 계산 함수 (Beta 공식)
// ============================================================================
// 회로: 12V ─ [10K 풀업] ─ ADC (3.3V Diode 클램프)
//                        └[10K NTC || 3.9K] ─ GND
// 
// 분압 회로 분석:
// 1. R_parallel = (V_adc × R_pullup) / (V_supply - V_adc)
// 2. R_th = (R_parallel × R_series) / (R_series - R_parallel)
// 3. Beta 공식으로 온도 계산
// ============================================================================
float calculateThermistorTemperature(float thermistorVoltage) {
    // 극한값 체크 (Diode 클램프로 0~3.3V)
    if (thermistorVoltage < 0.001 || thermistorVoltage > ADC_CLAMP_VOLTAGE) {
        return -999.0;  // 에러값
    }
    
    // ========== Step 1: 분압 회로에서 병렬 저항값 역산 ==========
    // V_adc = V_supply × R_parallel / (R_pullup + R_parallel)
    // 역산:
    // R_parallel = (V_adc × R_pullup) / (V_supply - V_adc)
    
    float denominator = THERMISTOR_SUPPLY_VOLTAGE - thermistorVoltage;
    if (denominator <= 0.0) {
        return -999.0;  // 에러: 전압이 전원 전압 이상
    }
    
    float rParallel = (thermistorVoltage * THERMISTOR_PULLUP_RESISTANCE) / denominator;
    
    if (rParallel <= 0.0 || rParallel >= THERMISTOR_SERIES_RESISTANCE) {
        return -999.0;  // 에러: 병렬 저항값이 범위 외
    }
    
    // ========== Step 2: Thermistor 저항값 역산 ==========
    // R_parallel = (R_th × R_series) / (R_th + R_series)
    // 역산:
    // R_th = (R_parallel × R_series) / (R_series - R_parallel)
    
    float seriesDenominator = THERMISTOR_SERIES_RESISTANCE - rParallel;
    if (seriesDenominator <= 0.0) {
        return -999.0;  // 에러
    }
    
    float resistance = (rParallel * THERMISTOR_SERIES_RESISTANCE) / seriesDenominator;
    
    if (resistance <= 0.0) {
        return -999.0;  // 에러값
    }
    
    // ========== Step 3: Beta 공식으로 온도 계산 ==========
    // 1/T = 1/T0 + (1/B) × ln(R/R0)
    // T = 절대온도(K), T0 = 기준온도(K), B = Beta값, R = 저항값, R0 = 기준저항값
    
    float logValue = logf(resistance / THERMISTOR_NOMINAL_RESISTANCE);
    float invT = (1.0 / THERMISTOR_NOMINAL_TEMPERATURE) + (logValue / THERMISTOR_BETA_VALUE);
    
    if (invT <= 0.0) {
        return -999.0;  // 에러값
    }
    
    float temperatureK = 1.0 / invT;  // 절대온도 (K)
    float temperatureC = temperatureK - ABSOLUTE_ZERO;  // 섭씨온도 (°C)
    
    // 온도 범위 체크 (-50°C ~ +100°C 정상범위)
    // 범위 외에서도 계산값을 반환하지만, 값이 이상할 수 있음
    
    return temperatureC;
}

// ============================================================================
// 내부 ADC 읽기 함수 (NTC Thermistor 온도값 포함)
// ============================================================================
void readInternalADC() {
  // 내부 ADC는 최대 6채널까지만 사용 (inter_adc 크기: 6ch)
  for (int i = 0; i < 6; i++) {
    if (ADC_VALID[i]) {
      int rawValue = analogRead(ADC_PINS[i]);
      
      // ADC 값을 3.3V 기준 전압으로 변환
      float voltage = (rawValue * ADC_REFERENCE_VOLTAGE) / ADC_RESOLUTION;
      
      // 내부 ADC 전압값 저장
      ioBoard.inter_adc[i] = voltage;
      
      // Thermistor 온도 계산 및 저장
      float temperature = calculateThermistorTemperature(voltage);
      ioBoard.temperature[i] = temperature;
      
      // 런타임 UART 출력은 JSON 로그만 유지 (추가 ADC 디버그 출력 비활성화)
    } else {
      ioBoard.inter_adc[i] = 0.0;           // 유효하지 않은 채널은 0으로 설정
      ioBoard.temperature[i] = -999.0;       // 온도값도 에러값 설정
    }
  }
  
  // 나머지 2개 채널의 temperature 값 초기화 (사용 안함)
  for (int i = 6; i < 8; i++) {
    ioBoard.temperature[i] = -999.0;
  }
}

// 내부 DAC 쓰기 함수 (10ms마다 호출)
void updateInternalDAC() {
  static unsigned long lastUpdateTime = 0;
  static unsigned long lastLogTime = 0;
  static const unsigned long UPDATE_INTERVAL = 10; // 10ms 간격
  static const unsigned long LOG_INTERVAL = 1000; // 1초마다 로그 출력
  
  unsigned long currentTime = millis();
  bool shouldLog = (currentTime - lastLogTime >= LOG_INTERVAL);
  
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    for (int i = 0; i < 2; i++) {
      float targetValue = ioBoard.dac[i];
      float currentValue = ioBoardBackup.dac[i];
      float maxValue = 4095.0;  // 12비트 스케일
      
      // 1% 변위폭 계산
      float stepSize = maxValue * 0.01;  // 1% 변위폭
      
      // 점진적 변화 계산
      float nextValue = currentValue;
      if (abs(targetValue - currentValue) > stepSize) {
        // 목표값과 현재값의 차이가 1%보다 클 때만 점진적 변화
        if (targetValue > currentValue) {
          nextValue = currentValue + stepSize;
        } else {
          nextValue = currentValue - stepSize;
        }
      } else {
        // 1% 이내로 수렴하면 목표값으로 설정
        nextValue = targetValue;
      }
      
      // 값이 변경되었을 때만 DAC 출력
      if (abs(nextValue - currentValue) > 0.1) {
        // 0-4095 범위를 0-255 DAC 범위로 변환
        int dacValue = (int)((nextValue / maxValue) * (DAC_RESOLUTION - 1));
        if (dacValue < 0) dacValue = 0;
        if (dacValue > 255) dacValue = 255;
        
        // DAC 출력
        dacWrite(DAC_PINS[i], dacValue);
        
        // 백업 값 업데이트
        ioBoardBackup.dac[i] = nextValue;
        
        // 1초마다 로그 출력
        // if (shouldLog) {
        //   float voltage = (dacValue * DAC_REFERENCE_VOLTAGE) / DAC_RESOLUTION;
        //   Serial.printf("Internal DAC CH%d: %.0f->%.0f(%d,%.3fV) ", 
        //                 i, targetValue, nextValue, dacValue, voltage);
        // }
      }
    }
    
    if (shouldLog) {
      lastLogTime = currentTime;
    }
    
    lastUpdateTime = currentTime;
  }
}

// PWM 값 설정 함수
void setPWMValue(int channel, int value) {
  if (channel < 0 || channel >= 2) {
    Serial.printf("[PWM] Invalid channel: %d (0-1 allowed)\n", channel);
    return;
  }
  
  WiFiConfig& cfg = configManager.getConfig();
  
  // PWM 모드가 아닌 경우 경고
  if (cfg.gpo_mode[channel] != GPO_PWM) {
    Serial.printf("[PWM] Warning: GPIO%d is not in PWM mode\n", channel);
    return;
  }
  
  // 값 범위 제한 (0-255)
  int pwmValue = constrain(value, 0, 255);
  
  // PWM 출력 및 ioBoard 값 업데이트
  ledcWriteChannel(channel, pwmValue);
  ioBoard.pwm[channel] = pwmValue;
  ioBoard.gpo[channel] = pwmValue;  // gpo 값도 동기화
  
  Serial.printf("[PWM] CH%d set to %d (%.1f%%)\n", 
                channel, pwmValue, (pwmValue * 100.0 / 255.0));
}


// LED ch1 toggle function (1 second period - P0, P2)
void LED_ch1_Toggle(void)
{
  ioBoard.led[0] = !ioBoard.led[0];  // P0 토글
  ioBoard.led[2] = !ioBoard.led[2];  // P2 토글
  // ioBoard.led[3] = !ioBoard.led[3];  // 정확한 토글 로직  // buzzer signal
}
// LED ch2 toggle function (100ms period - P1)
void LED_ch2_Toggle(void)
{
  ioBoard.led[1] = !ioBoard.led[1];  // P1 토글
}

// ============================================================================
// Buzzer Driver (ioBoard.led[3])
// - 100ms tick 기반
// - 총 4초(4 슬롯), 슬롯당 1초
// - 단음: 300ms ON + 700ms OFF (3 tick ON)
// - 장음: 700ms ON + 300ms OFF (7 tick ON)
// - pattern index(0~15): 4비트 패턴
//   * 0  = S,S,S,S
//   * 15 = L,L,L,L
//   * 5  = S,L,S,L (MSB -> LSB 순서)
// ============================================================================
namespace {
  constexpr uint8_t BUZZER_LED_CHANNEL = 3;
  constexpr uint8_t BUZZER_PATTERN_STEPS = 4;
  constexpr uint16_t BUZZER_SLOT_TICKS = 5;       // 1.0s / 100ms
  constexpr uint16_t BUZZER_SHORT_ON_TICKS = 1;    // 0.3s
  constexpr uint16_t BUZZER_LONG_ON_TICKS = 3;     // 0.7s
  constexpr uint16_t BUZZER_TEST_INTERVAL_TICKS = 100; // 10s / 100ms

  struct BuzzerDriverState {
    bool active = false;
    uint8_t patternIndex = 0;
    uint8_t step = 0;
    uint16_t stepTick = 0;
    uint8_t repeatRemaining = 0;
    uint16_t intervalTick = 0;
  } g_buzzer;

  bool isLongToneAtStep(uint8_t patternIndex, uint8_t step) {
    // step 0이 MSB(bit3), step 3이 LSB(bit0)
    uint8_t shift = (BUZZER_PATTERN_STEPS - 1) - step;
    return ((patternIndex >> shift) & 0x01) != 0;
  }
}

static void StartSingleBuzzerPattern(void)
{
  g_buzzer.step = 0;
  g_buzzer.stepTick = 0;
  g_buzzer.active = true;
  if (g_buzzer.repeatRemaining > 0) {
    g_buzzer.repeatRemaining--;
  }
}

void beepON(uint8_t count, uint8_t index)
{
  // 재생 중(또는 반복 대기 중)에는 새 요청을 무시
  if (g_buzzer.active || g_buzzer.repeatRemaining > 0 || g_buzzer.intervalTick > 0) {
    return;
  }

  if (count == 0) {
    return;
  }

  g_buzzer.patternIndex = index & 0x0F;
  g_buzzer.repeatRemaining = count;
  g_buzzer.intervalTick = 0;
  StartSingleBuzzerPattern();
}

void beepResetAndStart(uint8_t count, uint8_t index)
{
  // 상태 전이 시 기존 반복/재생을 즉시 중지하고 새 요청으로 덮어쓴다.
  g_buzzer.active = false;
  g_buzzer.patternIndex = 0;
  g_buzzer.step = 0;
  g_buzzer.stepTick = 0;
  g_buzzer.repeatRemaining = 0;
  g_buzzer.intervalTick = 0;
  ioBoard.led[BUZZER_LED_CHANNEL] = 0;

  if (count == 0) {
    return;
  }

  g_buzzer.patternIndex = index & 0x0F;
  g_buzzer.repeatRemaining = count;
  StartSingleBuzzerPattern();
}

void UpdateBuzzerDriver100mSec(void)
{
  if (!g_buzzer.active) {
    // 반복 대기 타이머 처리: interval 경과 후 다음 패턴 1회 시작
    if (g_buzzer.repeatRemaining > 0) {
      if (g_buzzer.intervalTick < BUZZER_TEST_INTERVAL_TICKS) {
        g_buzzer.intervalTick++;
      }

      if (g_buzzer.intervalTick >= BUZZER_TEST_INTERVAL_TICKS) {
        g_buzzer.intervalTick = 0;
        StartSingleBuzzerPattern();
      }
    }
    return;
  }

  bool isLong = isLongToneAtStep(g_buzzer.patternIndex, g_buzzer.step);
  uint16_t onTicks = isLong ? BUZZER_LONG_ON_TICKS : BUZZER_SHORT_ON_TICKS;
  ioBoard.led[BUZZER_LED_CHANNEL] = (g_buzzer.stepTick < onTicks) ? 1 : 0;

  g_buzzer.stepTick++;
  if (g_buzzer.stepTick >= BUZZER_SLOT_TICKS) {
    g_buzzer.stepTick = 0;
    g_buzzer.step++;

    if (g_buzzer.step >= BUZZER_PATTERN_STEPS) {
      g_buzzer.active = false;
      ioBoard.led[BUZZER_LED_CHANNEL] = 0;

      // 다음 반복이 남아있으면 interval 카운트 시작
      if (g_buzzer.repeatRemaining > 0) {
        g_buzzer.intervalTick = 0;
      }
    }
  }
}

void Buzzer_Test_Loop_100mSec(void)
{
  static uint16_t testTick = 0;
  static uint8_t testIndex = 0;
  constexpr uint8_t testCount = 1;

  if (testTick == 0 && !g_buzzer.active && g_buzzer.repeatRemaining == 0) {
    beepON(testCount, testIndex);
    Serial.printf("[BUZZER TEST] start count=%u index=%u\n", testCount, testIndex);
    testIndex = (testIndex + 1) & 0x0F;
  }

  UpdateBuzzerDriver100mSec();

  testTick++;
  if (testTick >= BUZZER_TEST_INTERVAL_TICKS) {
    testTick = 0;
  }
}


// 내부 DAC 상태 출력 함수 (1초마다 호출)
void printInternalDACValues() {
  Serial.print("Internal DAC Values: ");
  for (int i = 0; i < 2; i++) {
    float targetValue = ioBoard.dac[i];
    float currentValue = ioBoardBackup.dac[i];
    float maxValue = 4095.0;
    
    // 현재 DAC 출력값 계산
    int dacValue = (int)((currentValue / maxValue) * (DAC_RESOLUTION - 1));
    float voltage = (dacValue * DAC_REFERENCE_VOLTAGE) / DAC_RESOLUTION;
    
    // Serial.printf("CH%d: T=%.0f->C=%.0f(%d,%.3fV) ", i, targetValue, currentValue, dacValue, voltage);
  }
  // Serial.println();
}

// DAC7678 상태 출력 함수 (1초마다 호출)
void printDAC7678Values() {
  Serial.print("DAC7678 Values: ");
  for (int i = 0; i < 8; i++) {
    float targetVoltage = ioBoard.dac7678_target[i] / 100.0;  // 0~500 -> 0~5V
    float currentVoltage = ioBoard.dac7678_current[i] / 100.0;  // 0~500 -> 0~5V
    Serial.printf("CH%d: T=%.2fV C=%.2fV ", i, targetVoltage, currentVoltage);
  }
  Serial.println();
}


class UARTService {
public:
  void init();
};
UARTService uartService;
void UARTService::init() {
  // Serial1.begin(9600, SERIAL_8N1, 14, 15);     // example pins (TX: GPIO2, RX: GPIO14), (TX: GPIO15, RX: GPIO14)
  Serial.begin(115200, SERIAL_8N1, 3, 1);  // RX=16, TX=17 (UART2)
  Serial.println("UART Service initialized.");
}
void printConfig() {
  WiFiConfig& cfg = configManager.getConfig();
  Serial.println("==== Current Config ====");
  Serial.printf("SSID: %s\n", cfg.ssid);
  Serial.printf("MQTT Server: %s:%d\n", cfg.mqttServer, cfg.mqttPort);
  Serial.printf("MQTT ID (Username): %s\n", cfg.mqttId);
  Serial.printf("MQTT user passwd: %s\n", cfg.mqttPass); 
  Serial.printf("MQTT Client ID: %s\n", cfg.mqttClientId);
  Serial.printf("Log Subscribe Topic: %s\n", cfg.mqttSubTopic);
  Serial.printf("Cmd Subscribe Topic: %s\n", cfg.mqttCMDTopic);
  Serial.printf("Status Publish Topic: %s\n", cfg.mqttPubTopic);
  Serial.printf("MQTT Mode: %c\n", cfg.mqttlogMode);
  Serial.printf("MQTT Log Enabled: %s\n", g_mqttLogEnabled ? "YES" : "NO");
  Serial.printf("Device No: %d\n", cfg.mqttlogNumber);
  Serial.printf("Boot Count: %lu\n", cfg.bootCount);
  Serial.printf("Version: %s\n", cfg.version);
  // GPIO 모드 설정 출력
  Serial.println("--- GPIO Modes ---");
  for (int i = 0; i < 2; i++) {
    Serial.printf("GPI%d (GPIO%d): %s\n", i, GPIO_IN_PINS[i], 
                  cfg.gpi_mode[i] == GPI_FREQ ? "FREQ" : "GPI");
  }
  for (int i = 0; i < 2; i++) {
    Serial.printf("GPO%d (GPIO%d): %s\n", i, GPIO_OUT_PINS[i], 
                  cfg.gpo_mode[i] == GPO_PWM ? "PWM" : "GPO");
  }
  Serial.printf("Operation Mode: %u\n", configManager.getOperationMode());
  Serial.printf("Pellet Feed Duration: %u sec\n", configManager.getPelletFeedDurationSec());
  for (uint8_t mode = 0; mode < configManager.getRecipeCount(); mode++) {
    Serial.printf("TEMP LIMIT MODE %d: low=%.2f high=%.2f\n",
                  mode,
                  configManager.getTempLimitLow(mode),
                  configManager.getTempLimitHigh(mode));
  }
  Serial.println("========================");
}

void applyOperationModeTemperatureLimits() {
  uint8_t mode = configManager.getOperationMode();
  float low = configManager.getTempLimitLow(mode);
  float high = configManager.getTempLimitHigh(mode);

  for (int ch = 0; ch < 6; ch++) {
    batagota.setTemperatureLimits((Batagota::InternalADCChannel)ch, low, high);
  }

  Serial.printf("[TEMP LIMIT] Applied mode=%u low=%.2f high=%.2f\n", mode, low, high);
}

void printTemperatureLimitBootInfo() {
  uint8_t mode = configManager.getOperationMode();
  const RecipeStageProfile& recipe = configManager.getActiveRecipeProfile();
  Serial.println("[BOOT] Temperature limit configuration from Flash");
  Serial.printf("[BOOT] Current OP_MODE: %u\n", mode);
  Serial.printf("[BOOT] Recipe Count: %u\n", configManager.getRecipeCount());
  Serial.printf("[BOOT] Active Recipe FuelType: %u\n", recipe.fuel_type);
  Serial.printf("[BOOT] Ignition: smoke=%u t1=%u t2=%u reignite_t1=%u reignite_t2=%u pump_on=%u pump_off=%u\n",
                recipe.ignition.smoke_enable,
                recipe.ignition.ignite_t1,
                recipe.ignition.ignite_t2,
                recipe.ignition.reignite_t1,
                recipe.ignition.reignite_t2,
                recipe.ignition.pump_on_sec,
                recipe.ignition.pump_off_sec);
  Serial.printf("[BOOT] Pellet Feed Duration: %u sec\n", configManager.getPelletFeedDurationSec());
  for (uint8_t m = 0; m < configManager.getRecipeCount(); m++) {
    Serial.printf("[BOOT] MODE %d LIMIT: low=%.2f high=%.2f\n",
                  m,
                  configManager.getTempLimitLow(m),
                  configManager.getTempLimitHigh(m));
  }
}
// sample uart message
// EXioBd_:status:"[5D6B] [4.94 3.40 1.86 2.94 3.24 4.94 1.30 4.55 2.92 1.63 4.95 4.53 2.76 2.71 1.80 3.24] [28.29 28.87 23.70 24.04 28.20 25.53 25.53 27.90 29.52 24.13 20.73 28.66 20.20 22.72 23.57 20.58 24.59 25.01 26.07 21.70] [3.49 2.49 0.82 3.84 1.21 1.56 2.09 3.29 4.08 4.68 2.38 2.85 0.83 1.30 3.91 3.93] [58.03 53.47] [00] [13]"
int splitString(String input, char delimiter, String *resultArray, int maxParts) {
  int count = 0;
  int startIndex = 0;
  int endIndex = input.indexOf(delimiter);
  while (endIndex != -1 && count < maxParts) {
    resultArray[count++] = input.substring(startIndex, endIndex);
    startIndex = endIndex + 1;
    endIndex = input.indexOf(delimiter, startIndex);
  }
  // 마지막 조각 추가
  if (startIndex < input.length() && count < maxParts) {
    resultArray[count++] = input.substring(startIndex);
  }
  return count;  // 분리된 항목 수 반환
}

void parseIOBoardStatus(const String& msg) {
  int start = msg.indexOf('[');
  int end = msg.lastIndexOf(']');
  if (start == -1 || end == -1 || end <= start) {
    Serial.println("Invalid message format");
    return;
  }

  String content = msg.substring(start + 1, end);
  String sections[7];
  int sectionCount = splitString(content, ',', sections, 7);

  if (sectionCount != 7) {
    Serial.print("Section count mismatch: ");
    Serial.println(sectionCount);
    return;
  }

  // FET
  {
    unsigned int fetBits = (unsigned int)strtoul(sections[0].c_str(), nullptr, 10);  // decimal임
    for (int i = 0; i < 16; i++) {
      ioBoardBackup.fet[i] = (fetBits >> i) & 0x01;
    }
  }

  // ADC - ads1015_adc는 8채널만 지원하므로 처음 8개만 저장
  {
    String tokens[16];
    int count = splitString(sections[1], ' ', tokens, 16);
    if (count != 16) { Serial.println("ADC parse error"); return; }
    for (int i = 0; i < 8 && i < count; i++) {
      ioBoardBackup.ads1015_adc[i] = tokens[i].toFloat();
    }
  }

  // TEMPERATURE - temperature는 8채널만 지원하므로 처음 8개만 저장
  {
    String tokens[20];
    int count = splitString(sections[2], ' ', tokens, 20);
    if (count != 20) { Serial.println("Temperature parse error"); return; }
    for (int i = 0; i < 8 && i < count; i++) {
      ioBoardBackup.temperature[i] = tokens[i].toFloat();
    }
  }

  // DAC
  {
    String tokens[16];
    int count = splitString(sections[3], ' ', tokens, 16);
    if (count != 16) { Serial.println("DAC parse error"); return; }
    for (int i = 0; i < 16; i++) {
      ioBoardBackup.dac[i] = tokens[i].toFloat();
    }
  }

  // FREQ
  {
    String tokens[2];
    int count = splitString(sections[4], ' ', tokens, 2);
    if (count != 2) { Serial.println("Freq parse error"); return; }
    for (int i = 0; i < 2; i++) {
      ioBoardBackup.freq[i] = tokens[i].toFloat();
    }
  }

  // GPI
  {
    unsigned int gpiBits = (unsigned int)strtoul(sections[5].c_str(), nullptr, 10);
    for (int i = 0; i < 2; i++) {
      ioBoardBackup.gpi[i] = (gpiBits >> i) & 0x01;
    }
  }

  // GPO
  {
    unsigned int gpoBits = (unsigned int)strtoul(sections[6].c_str(), nullptr, 10);
    for (int i = 0; i < 2; i++) {
      ioBoardBackup.gpo[i] = (gpoBits >> i) & 0x01;
    }
  }

  Serial.println("IOBoard status parsed successfully");
}



/////////////////////////////MAX31856  Start /////////////////////////////////
#define PIN_MUX_SEL0    0  // DG409 A0
#define PIN_MUX_SEL1    2   // DG409 A1
#define PIN_SPI_CS_TC   5   // MAX31856 CS
// MAX31856 객체 생성
Adafruit_MAX31856 max31856(PIN_SPI_CS_TC);

// Thermocouple 5개 버퍼에 값 추가 및 평균 계산
void addThermocoupleToBuffer(int channel, float temperature) {
  if (channel < 0 || channel >= 4) return;
  
  // 버퍼에 새 값 추가
  ioBoard.TC_buffer[channel][ioBoard.TC_buffer_index[channel]] = temperature;
  
  // 인덱스 업데이트
  ioBoard.TC_buffer_index[channel]++;
  if (ioBoard.TC_buffer_index[channel] >= 5) {
    ioBoard.TC_buffer_index[channel] = 0;
    ioBoard.TC_buffer_full[channel] = true;  // 버퍼가 한바퀴 돌았음
  }
  
  // 평균값 계산
  float sum = 0.0;
  int count = ioBoard.TC_buffer_full[channel] ? 5 : ioBoard.TC_buffer_index[channel];
  
  for (int i = 0; i < count; i++) {
    sum += ioBoard.TC_buffer[channel][i];
  }
  
  ioBoard.TC_average[channel] = sum / count;
  
  // 원본 TC 값도 업데이트 (호환성 유지)
  ioBoard.TC[channel] = ioBoard.TC_average[channel];
}

// Thermocouple 평균값 반환
float getThermocoupleAverage(int channel) {
  if (channel < 0 || channel >= 4) return 0.0;
  return ioBoard.TC_average[channel];
}

// Thermocouple 버퍼 상태 출력 (디버깅용)
void printThermocoupleBuffer(int channel) {
  if (channel < 0 || channel >= 4) return;
  
  Serial.printf("TC CH%d Buffer: [", channel);
  int count = ioBoard.TC_buffer_full[channel] ? 5 : ioBoard.TC_buffer_index[channel];
  for (int i = 0; i < count; i++) {
    Serial.printf("%.2f", ioBoard.TC_buffer[channel][i]);
    if (i < count - 1) Serial.print(", ");
  }
  Serial.printf("] Avg: %.2f°C\n", ioBoard.TC_average[channel]);
}

// MUX 핀 상태 확인 함수
void testMuxPins(void)
{
  Serial.println("=== MUX Pin Test ===");
  for (int ch = 0; ch < 4; ch++) {
    selectThermoChannel(ch);
    
    // 실제 핀 상태 읽기
    int sel0_state = digitalRead(PIN_MUX_SEL0);
    int sel1_state = digitalRead(PIN_MUX_SEL1);
    
    Serial.printf("CH%d: Expected(SEL0=%d,SEL1=%d) Actual(SEL0=%d,SEL1=%d)\n", 
                  ch, 
                  ch & 0x01, 
                  (ch >> 1) & 0x01,
                  sel0_state,
                  sel1_state);
    
    delay(500); // 0.5초 지연으로 변화 확인
  }
  Serial.println("==================");
}
void initThermoCoupleSystem() {
  // 사용자 정의 SPI 핀 설정
  // SPI.begin(SCK, MISO, MOSI, SS)
  SPI.begin(14, 13, 15, 5); // SCK=14, MISO=13, MOSI=15, CS=5
  Serial.println("SPI initialized with custom pins: SCK=14, MISO=13, MOSI=15, CS=5");
  
  // SPI 속도 설정 (1MHz로 안정적 통신)
  SPI.setClockDivider(SPI_CLOCK_DIV16); // 16MHz / 16 = 1MHz
  
  // MAX31856 초기화 전 지연
  delay(100);

  // MAX31856 초기화
  if (!max31856.begin()) {
    Serial.println("Could not initialize MAX31856. Check wiring!");
    Serial.println("SPI Connections:");
    Serial.println("- SCK: GPIO14");
    Serial.println("- MISO: GPIO13");
    Serial.println("- MOSI: GPIO15");
    Serial.println("- CS: GPIO5");
    Serial.println("- VIN: 3.3V");
    Serial.println("- GND: GND");
    while (1) delay(10);
  }
  
  Serial.println("MAX31856 initialized successfully!");
  
  // MAX31856 설정 (Type K Thermocouple)
  max31856.setThermocoupleType(MAX31856_TCTYPE_K);
  max31856.setConversionMode(MAX31856_CONTINUOUS);
  
  // 설정 안정화 시간
  delay(250);
  
  // MAX31856 내부 레지스터 상태 확인
  verifyMAX31856Configuration();
  
  // DG409 MUX GPIO 초기화
  pinMode(PIN_MUX_SEL0, OUTPUT);
  pinMode(PIN_MUX_SEL1, OUTPUT);
  
  // 초기 MUX 채널 선택 (0)
  selectThermoChannel(0);
  
  Serial.println("Thermocouple system initialization complete!");
}

// MAX31856 레지스터 상태 확인 함수
void verifyMAX31856Configuration() {
  Serial.println("=== MAX31856 Register Verification ===");
  
  // Fault Status 확인 (public 함수 사용)
  uint8_t fault = max31856.readFault();
  Serial.printf("Fault Status: 0x%02X\n", fault);
  if (fault != 0) {
    Serial.println("  Fault detected:");
    if (fault & MAX31856_FAULT_CJHIGH) Serial.println("    - Cold Junction High Fault");
    if (fault & MAX31856_FAULT_CJLOW) Serial.println("    - Cold Junction Low Fault");
    if (fault & MAX31856_FAULT_TCHIGH) Serial.println("    - Thermocouple High Fault");
    if (fault & MAX31856_FAULT_TCLOW) Serial.println("    - Thermocouple Low Fault");
    if (fault & MAX31856_FAULT_OVUV) Serial.println("    - Overvoltage/Undervoltage Fault");
    if (fault & MAX31856_FAULT_OPEN) Serial.println("    - Thermocouple Open Circuit Fault");
  } else {
    Serial.println("  No faults detected");
  }
  
  // Cold Junction Temperature 읽기
  float coldJunctionTemp = max31856.readCJTemperature();
  Serial.printf("Cold Junction Temperature: %.2f°C\n", coldJunctionTemp);
  
  // 첫 번째 온도 읽기 테스트
  Serial.println("Testing initial temperature reading...");
  for (int i = 0; i < 3; i++) {
    float temp = max31856.readThermocoupleTemperature();
    Serial.printf("  Test read %d: %.2f°C\n", i+1, temp);
    delay(100);
  }
  
  Serial.println("=====================================");
}

// DG409 채널 선택 함수
void selectThermoChannel(uint8_t channel) {
  // channel: 0~3
  digitalWrite(PIN_MUX_SEL0, channel & 0x01);
  digitalWrite(PIN_MUX_SEL1, (channel >> 1) & 0x01);
  
  // MUX switching 디버그 출력
  // Serial.printf("MUX Switch: CH%d (SEL0=%d, SEL1=%d)\n", 
  //               channel, 
  //               channel & 0x01, 
  //               (channel >> 1) & 0x01);
  
  // MUX 안정화를 위한 지연 (CAN 전송에 영향을 주지 않도록 최소화)
  delayMicroseconds(500);  // 50ms에서 0.5ms로 단축
}

// MAX31856 수동 테스트 함수
void testMAX31856Manual() {
  Serial.println("=== Manual MAX31856 Test ===");
  
  for (int ch = 0; ch < 4; ch++) {
    Serial.printf("Testing Channel %d:\n", ch);
    selectThermoChannel(ch);
    
    // 채널 변경 후 더 긴 안정화 시간
    delay(200);
    
    // 여러 번 읽기 테스트
    for (int i = 0; i < 3; i++) {
      float temp = max31856.readThermocoupleTemperature();
      float coldJunc = max31856.readCJTemperature();
      uint8_t fault = max31856.readFault();
      
      Serial.printf("  Read %d: TC=%.2f°C, CJ=%.2f°C, Fault=0x%02X\n", 
                    i+1, temp, coldJunc, fault);
      delay(100);
    }
    Serial.println();
  }
  Serial.println("============================");
}

// MAX31856 Value Save
void Get_MAX31856_Data(void)  // 100mSec loop scan
{
  // 현재 채널의 온도를 읽기 (MUX가 이미 해당 채널로 설정된 상태)
  float temperature = max31856.readThermocoupleTemperature();
  
  // 에러 체크 (public 함수 사용)
  uint8_t fault = max31856.readFault();
  
  if (fault != 0) {
    // fault 메시지를 30초마다 한 번씩만 출력
    static unsigned long lastFaultPrintTime[4] = {0, 0, 0, 0}; // 각 채널별 마지막 출력 시간
    unsigned long currentTime = millis();
    
    if (!batagota.isDebugJsonPrintEnabled() &&
        currentTime - lastFaultPrintTime[ioBoard.tcloopcnt] >= 30000) { // 30초 = 30000ms
      lastFaultPrintTime[ioBoard.tcloopcnt] = currentTime;
      
      Serial.printf("TC CH%d: FAULT (0x%02X) - ", ioBoard.tcloopcnt, fault);
      if (fault & MAX31856_FAULT_OPEN) Serial.print("OPEN_CIRCUIT ");
      if (fault & MAX31856_FAULT_OVUV) Serial.print("OVERVOLTAGE ");
      if (fault & MAX31856_FAULT_TCLOW) Serial.print("TC_LOW ");
      if (fault & MAX31856_FAULT_TCHIGH) Serial.print("TC_HIGH ");
      if (fault & MAX31856_FAULT_CJLOW) Serial.print("CJ_LOW ");
      if (fault & MAX31856_FAULT_CJHIGH) Serial.print("CJ_HIGH ");
      Serial.println();
    }
    
    // 에러가 있어도 온도값은 버퍼에 저장 (디버깅 목적)
    addThermocoupleToBuffer(ioBoard.tcloopcnt, temperature);
  } else {
    // 정상적인 온도값을 버퍼에 저장하고 평균 계산
    addThermocoupleToBuffer(ioBoard.tcloopcnt, temperature);
    // Serial.printf("TC CH%d: %.2f°C (OK)\n", ioBoard.tcloopcnt, temperature);
  }

  // 다음 채널로 이동
  ioBoard.tcloopcnt++;
  if(ioBoard.tcloopcnt >= 4) ioBoard.tcloopcnt = 0;
  
  // 다음 채널로 MUX 설정 (다음 100ms에 읽을 준비)
  selectThermoChannel(ioBoard.tcloopcnt);
}
/////////////////////////////MAX31856  End /////////////////////////////////
// FET & LED val update start
void Drive_FET_Ch_Status(int ch, bool status)
{
  // Set FET status on TCA9534 IO Expander
  if (ch < 0 || ch >= 8) return; // 범위 체크
  ioBoard.fet[ch] = status;
}


void FET_Test_Loop(void)
{
  // FET 채널 0~7번 순서대로 1(on) shift 테스트 (1초마다)
  static int current_channel = 0;
  
  // 모든 FET 채널을 먼저 OFF로 설정
  for (int i = 0; i < 8; i++) {
    ioBoard.fet[i] = 0;
  }
  
  // 현재 채널만 ON
  ioBoard.fet[current_channel] = 1;
  
  // Serial.printf("FET Test Loop - Channel %d ON: ", current_channel);
  // for (int i = 0; i < 8; i++) {
  //   Serial.printf("FET%d=%d ", i, ioBoard.fet[i]);
  // }
  // Serial.println();
  
  // 다음 채널로 이동 (0~7 순환)
  current_channel++;
  if (current_channel >= 8) {
    current_channel = 0;
  }
}

void RELAY_Test_Loop(void)
{
  // Relay 채널 0~7번 순서대로 1(on) shift 테스트 (1초마다)
  static int current_channel = 0;

  // 모든 Relay 채널을 먼저 OFF로 설정
  for (int i = 0; i < 8; i++) {
    relayStatus[i] = 0;
  }

  // 현재 채널만 ON
  relayStatus[current_channel] = 1;

  // 다음 채널로 이동 (0~7 순환)
  current_channel++;
  if (current_channel >= 8) {
    current_channel = 0;
  }
}



void DAC7678_Test_Loop(void)
{
  // 8채널 DAC 테스트 - 1초마다 0.1V씩 증가 (0V~5V 순환)
  static float dac_test_voltage[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static int test_cycle = 0;
  
  // 모든 채널에 대해 0.1V씩 증가
  for (int ch = 0; ch < 8; ch++) {
    dac_test_voltage[ch] += 0.1;
    
    // 5V 초과시 0V로 리셋
    if (dac_test_voltage[ch] > 5.0) {
      dac_test_voltage[ch] = 0.0;
    }
    
    // 전압을 DAC7678 target 값으로 변환 (0~500 스케일)
    ioBoard.dac7678_target[ch] = (int)(dac_test_voltage[ch] * 100);
    
    // 범위 체크
    if (ioBoard.dac7678_target[ch] < 0) ioBoard.dac7678_target[ch] = 0;
    if (ioBoard.dac7678_target[ch] > 500) ioBoard.dac7678_target[ch] = 500;
  }
  test_cycle++;
}

void Internal_DAC_Test_Loop(void)
{
  // 내부 DAC 2채널 테스트 - 1초마다 0.1V씩 증가 (0V~3.3V 순환)
  static float internal_dac_voltage[2] = {0.0, 0.0};
  static int test_cycle = 0;
  
  // 2채널에 대해 0.1V씩 증가
  for (int ch = 0; ch < 2; ch++) {
    internal_dac_voltage[ch] += 0.1;
    
    // 3.3V 도달 또는 초과시 0V로 리셋
    if (internal_dac_voltage[ch] >= 3.3) {
      internal_dac_voltage[ch] = 0.0;
    }
    
    // 전압을 내부 DAC 값으로 변환 (0~4095 스케일)
    ioBoard.dac[ch] = (internal_dac_voltage[ch] / 3.3) * 4095.0;
    
    // 범위 체크
    if (ioBoard.dac[ch] < 0) ioBoard.dac[ch] = 0;
    if (ioBoard.dac[ch] > 4095) ioBoard.dac[ch] = 4095;
  }
  
  // 현재 상태 출력
  // Serial.printf("Internal DAC Test Loop (Cycle %d): ", test_cycle);
  // for (int ch = 0; ch < 2; ch++) {
    // Serial.printf("CH%d=%.1fV(RAW:%.0f) ", ch, internal_dac_voltage[ch], ioBoard.dac[ch]);
  // }
  // Serial.println();
  
  test_cycle++;
}

void TCA9534_Update(void)
{
  // Update FET values to TCA9534 IO Expander
  for (int i = 0; i < 8; i++) {
    if (ioBoard.fet[i] != ioBoardBackup.fet[i]) {
      ioBoardBackup.fet[i] = ioBoard.fet[i]; // backup copy from io board uart status message
      Set_TCA9534_FET_ch_Status(i, ioBoard.fet[i]); // Set FET status on TCA9534
    }
  }
  
  // LED 값은 더 이상 TCA9534 hw sel id 핀과 연계하지 않는다.

  // Update Relay values to TCA9534 IO Expander
  for (int i = 0; i < 8; i++) {
    if (relayStatus[i] != relayBackup[i]) {
      relayBackup[i] = relayStatus[i];
      Set_TCA9534_Relay_ch_Status(i, relayStatus[i]);
    }
  }

}
// FET & LED val update end

void print_current_time() {
  char timeStr[20];
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Serial.printf("time: %s --", timeStr);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////Setup() & Loop()/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variables for loop timing
unsigned long lastMillis = 0;
unsigned long Millis100mSec =0;
unsigned long Millis200mSec = 0;  // 200ms 타이밍을 위한 변수 추가
const unsigned long interval = 1000; // 1 second
const unsigned long interval100mSec = 100; // .1 second
const unsigned long interval200mSec = 250; // .25 second

#define UART_BUFFER_SIZE 1024
#define UART_TIMEOUT_MS  50   // 타임아웃 기준 (ms)

String uartBuffer = "";
unsigned long lastUartTime = 0;

// UART Command List (ba+ header only)
// - Header policy:
//   1) Input must start with "ba+"
//   2) Non-matching header line is discarded immediately
//
// - Common format:
//   ba+<command>=arg1,arg2,arg3
//
// - Supported commands:
//   1) ba+set=arg1,arg2,arg3
//      - arg1: 1(DEBUG_JSON), 5(MQTT), 6(CAN), 7(RELAY), 8(GPO), 9(FET), 10(INT_DAC), 11(EXT_DAC), 15(STATE_UART_LOG), 16(HW_SEL_IO_OUT),
//              20(ACTIVE_RECIPE), 21~52(RECIPE_FIELD)
//      - Channel index policy: 0-based index (not 1-based)
//      - 1(DEBUG_JSON): arg2=enable(0|1), arg3=reserved
//      - 5(MQTT):     arg2=enable(0|1),      arg3=persist(0|1)
//      - 6(CAN):      arg2=enable(0|1),      arg3=persist(0|1)
//      - 7(RELAY):    arg2=channel(0~7),     arg3=value(0|1)
//      - 8(GPO):      arg2=channel(0~1),     arg3=value(0|1)
//      - 9(FET):      arg2=channel(0~7),     arg3=value(0|1)
//      - 10(INT_DAC): arg2=channel(0~1),     arg3=level(0~4095, 12-bit)
//      - 11(EXT_DAC): arg2=channel(0~7),     arg3=target(0~500, 0.01V step)
//      - 15(STATE_UART_LOG): arg2=enable(0|1), arg3=reserved
//      - 16(HW_SEL_IO_OUT):
//          * pin set:  arg2=pin(0~3), arg3=value(0|1)
//          * nibble set: arg2=15,     arg3=value(0~15, P0~P3 bitfield)
//      - 20(ACTIVE_RECIPE): arg2=recipe_index(0~3), arg3=reserved
//      - 21~52(RECIPE_FIELD): arg2=recipe_index(0~3), arg3=value
//
//   1-1) ba+recipe={json}
//      - JSON must include: recipe_no, stage_no
//      - stage_no: 0(ignition), 1(cooking), 2(drying), 3(holding)
//      - stage field payload can be sent as root keys or data object keys
//      - Example:
//        ba+recipe={"recipe_no":1,"stage_no":0,"data":{"smoke_enable":1,"ignite_t1":120,"ignite_t2":140}}
//
//   2) ba+get=arg1,arg2,arg3
//      - arg1: 0(all), 1(DEBUG_JSON), 2(HW_BD_ID), 5(MQTT), 6(CAN), 7(RELAY), 8(GPO&GPI), 9(FET), 10(INT_DAC&EXT_DAC), 12(ADC 8ch), 13(TEMP 6ch), 14(TC), 15(STATE_UART_LOG), 16(HW_SEL_IO_OUT)
//      - arg2,arg3: reserved (currently ignored)
//
//   3) ba+mqtt=arg1,arg2,arg3
//      - arg1: 1(enable), 0(disable)
//      - arg2: persist flag (0=no save, 1=save)
//      - arg3: reserved (currently ignored)
//
//   4) ba+can=arg1,arg2,arg3
//      - arg1: 1(enable), 0(disable)
//      - arg2: persist flag (0=no save, 1=save)
//      - arg3: reserved (currently ignored)
//
// - Other ba+ commands:
//   delegated to batagota.handleUartCommandLine()

static bool setRecipeFieldIfPresent(uint8_t recipeNo, JsonObject obj, const char* key, uint8_t fieldId) {
  if (!obj.containsKey(key)) {
    return true;
  }

  int32_t value = obj[key] | INT32_MIN;
  if (value == INT32_MIN) {
    return false;
  }
  return configManager.setRecipeField(recipeNo, fieldId, value);
}

static bool applyRecipeJsonByStage(uint8_t recipeNo, uint8_t stageNo, JsonObject obj) {
  bool ok = true;

  switch (stageNo) {
    case 0: {
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "fuel_type", RECIPE_FIELD_FUEL_TYPE);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "smoke_enable", RECIPE_FIELD_SMOKE_ENABLE);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "ignite_t1", RECIPE_FIELD_IGNITE_T1);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "ignite_t2", RECIPE_FIELD_IGNITE_T2);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "pump_condition", RECIPE_FIELD_PUMP_CONDITION);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "pump_power", RECIPE_FIELD_PUMP_POWER);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "reignite_t1", RECIPE_FIELD_REIGNITE_T1);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "reignite_t2", RECIPE_FIELD_REIGNITE_T2);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "pump_on_sec", RECIPE_FIELD_PUMP_ON_SEC);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "pump_off_sec", RECIPE_FIELD_PUMP_OFF_SEC);
      break;
    }
    case 1: {
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "cook_minutes", RECIPE_FIELD_COOK_MINUTES);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "oven_min", RECIPE_FIELD_COOK_OVEN_MIN);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "oven_max", RECIPE_FIELD_COOK_OVEN_MAX);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "oven_error_pct", RECIPE_FIELD_COOK_OVEN_ERROR_PCT);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "heater_on_sec", RECIPE_FIELD_COOK_HEATER_ON_SEC);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "heater_off_sec", RECIPE_FIELD_COOK_HEATER_OFF_SEC);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "fan_on_sec", RECIPE_FIELD_COOK_FAN_ON_SEC);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "fan_off_sec", RECIPE_FIELD_COOK_FAN_OFF_SEC);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "spray_time_sec", RECIPE_FIELD_COOK_SPRAY_TIME_SEC);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "spray_power", RECIPE_FIELD_COOK_SPRAY_POWER);
      break;
    }
    case 2: {
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "dry_minutes", RECIPE_FIELD_DRY_MINUTES);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "oven_min", RECIPE_FIELD_DRY_OVEN_MIN);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "oven_max", RECIPE_FIELD_DRY_OVEN_MAX);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "oven_error_pct", RECIPE_FIELD_DRY_OVEN_ERROR_PCT);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "heater_on_sec", RECIPE_FIELD_DRY_HEATER_ON_SEC);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "heater_off_sec", RECIPE_FIELD_DRY_HEATER_OFF_SEC);
      break;
    }
    case 3: {
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "hold_minutes", RECIPE_FIELD_HOLD_MINUTES);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "oven_min", RECIPE_FIELD_HOLD_OVEN_MIN);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "oven_max", RECIPE_FIELD_HOLD_OVEN_MAX);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "oven_error_pct", RECIPE_FIELD_HOLD_OVEN_ERROR_PCT);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "heater_on_sec", RECIPE_FIELD_HOLD_HEATER_ON_SEC);
      ok = ok && setRecipeFieldIfPresent(recipeNo, obj, "heater_off_sec", RECIPE_FIELD_HOLD_HEATER_OFF_SEC);
      break;
    }
    default:
      return false;
  }

  return ok;
}

void handleUartReceive() {
  while (Serial.available()) {
    char c = Serial.read();
    uartBuffer += c;
    lastUartTime = millis();  // 수신이 일어나면 시간 갱신
    // Accept both LF and CR as command terminator.
    if (c == '\n' || c == '\r') {
      uartBuffer.trim(); // 개행 제거
      // CRLF 조합에서 두 번째 종단문자로 인해 빈 라인이 들어올 수 있으므로 무시
      if (uartBuffer.length() == 0) {
        uartBuffer = "";
        continue;
      }
      String lowerCmd = uartBuffer;
      lowerCmd.toLowerCase();

      // UART 헤더는 ba+만 허용
      if (!lowerCmd.startsWith("ba+")) {
        uartBuffer = "";
        return;
      }

      // ba+<command>=... 형식 파싱
      // 예) ba+set=5,1,1 / ba+get=6,0,0 / ba+mqtt=1,1,0 / ba+recipe={...}
      String commandBodyRaw = uartBuffer.substring(3); // "ba+" 제거 (원본 유지)
      int eqIndex = commandBodyRaw.indexOf('=');
      String commandType = (eqIndex >= 0) ? commandBodyRaw.substring(0, eqIndex) : commandBodyRaw;
      String argPart = (eqIndex >= 0) ? commandBodyRaw.substring(eqIndex + 1) : "";
      commandType.trim();
      argPart.trim();
      commandType.toLowerCase();

      int arg1 = -1;
      int arg2 = -1;
      int arg3 = -1;
      int parsed = 0;
      if (argPart.length() > 0) {
        parsed = sscanf(argPart.c_str(), "%d,%d,%d", &arg1, &arg2, &arg3);
      }

      if (commandType == "recipe") {
        if (argPart.length() == 0) {
          uartBuffer = "";
          continue;
        }

        StaticJsonDocument<1024> doc;
        DeserializationError err = deserializeJson(doc, argPart);
        if (err) {
          uartBuffer = "";
          continue;
        }

        int recipeNo = doc["recipe_no"] | -1;
        int stageNo = doc["stage_no"] | -1;
        if (recipeNo < 0 || recipeNo >= configManager.getRecipeCount()) {
          uartBuffer = "";
          continue;
        }
        if (stageNo < 0 || stageNo > 3) {
          uartBuffer = "";
          continue;
        }

        JsonObject payloadObj;
        if (doc.containsKey("data")) {
          payloadObj = doc["data"].as<JsonObject>();
        } else {
          payloadObj = doc.as<JsonObject>();
        }

        if (payloadObj.isNull()) {
          uartBuffer = "";
          continue;
        }

        if (!applyRecipeJsonByStage((uint8_t)recipeNo, (uint8_t)stageNo, payloadObj)) {
          uartBuffer = "";
          continue;
        }

        if ((uint8_t)recipeNo == configManager.getOperationMode()) {
          applyOperationModeTemperatureLimits();
        }
      } else if (commandType == "set") {
        // ba+set=arg1,arg2,arg3
        // arg1: 1(DEBUG_JSON), 5(MQTT), 6(CAN), 7(RELAY), 8(GPO), 9(FET), 10(INT_DAC), 11(EXT_DAC), 15(STATE_UART_LOG), 16(HW_SEL_IO_OUT), 20(ACTIVE_RECIPE), 21~52(RECIPE_FIELD)
        if (parsed == 3) {
          switch (arg1) {
            case 1: {
              // ba+set=1,<enable 0|1>,<reserved>
              if (arg2 != 0 && arg2 != 1) {
                break;
              }
              batagota.setDebugJsonPrintEnabled(arg2 == 1);
              break;
            }
            case 5: {
              if (arg2 != 0 && arg2 != 1) {
                break;
              }
              bool enable = (arg2 == 1);
              bool persist = (arg3 != 0);
              setMqttLogEnable(enable, "UART_BA_SET", persist);
              break;
            }
            case 6: {
              if (arg2 != 0 && arg2 != 1) {
                break;
              }
              bool enable = (arg2 == 1);
              bool persist = (arg3 != 0);
              setCanCommEnable(enable, "UART_BA_SET", persist);
              break;
            }
            case 7: {
              // ba+set=7,<relay_ch>,<0|1>
              if (arg2 < 0 || arg2 > 7) {
                break;
              }
              if (arg3 != 0 && arg3 != 1) {
                break;
              }
              relayStatus[arg2] = arg3;
              break;
            }
            case 8: {
              // ba+set=8,<gpo_ch>,<0|1>
              if (arg2 < 0 || arg2 > 1) {
                break;
              }
              if (arg3 != 0 && arg3 != 1) {
                break;
              }
              ioBoard.gpo[arg2] = arg3;
              break;
            }
            case 9: {
              // ba+set=9,<fet_ch>,<0|1>
              if (arg2 < 0 || arg2 > 7) {
                break;
              }
              if (arg3 != 0 && arg3 != 1) {
                break;
              }
              ioBoard.fet[arg2] = arg3;
              Set_TCA9534_FET_ch_Status((uint8_t)arg2, (uint8_t)arg3);
              ioBoardBackup.fet[arg2] = arg3;
              break;
            }
            case 10: {
              // ba+set=10,<int_dac_ch>,<level 0-4095>
              if (arg2 < 0 || arg2 > 1) {
                break;
              }
              if (arg3 < 0 || arg3 > 4095) {
                break;
              }
              ioBoard.dac[arg2] = (float)arg3;
              break;
            }
            case 11: {
              // ba+set=11,<ext_dac_ch>,<target 0-500 (0.01V step)>
              if (arg2 < 0 || arg2 > 7) {
                break;
              }
              // 입력값을 target 스케일(0~500 = 0.00V~5.00V, 0.01V step)로 직접 사용
              // 범위를 벗어나면 안전하게 clamp
              int extDacTarget = arg3;
              if (extDacTarget < 0) {
                extDacTarget = 0;
              } else if (extDacTarget > 500) {
                extDacTarget = 500;
              }
              ioBoard.dac7678_target[arg2] = extDacTarget;
              break;
            }
            case 15: {
              // ba+set=15,<enable 0|1>,<reserved>
              if (arg2 != 0 && arg2 != 1) {
                break;
              }
              batagota.setStateUartLogEnabled(arg2 == 1);
              break;
            }
            case 16: {
              // ba+set=16,<pin 0~3>,<0|1>
              // ba+set=16,15,<0~15>  (P0~P3 bitfield)
              uint8_t cfg = Get_IO21_Config_Status();
              uint8_t out = Get_IO21_Output_Port_Status();

              if (arg2 >= 0 && arg2 <= 3) {
                if (arg3 != 0 && arg3 != 1) {
                  break;
                }

                if ((cfg >> arg2) & 0x01) {
                  Serial.printf("[HW SEL IO SET] WARN: P%d is INPUT (cfg bit=1)\n", arg2);
                }

                if (arg3 == 1) {
                  out |= (1 << arg2);
                } else {
                  out &= ~(1 << arg2);
                }
              } else if (arg2 == 15) {
                if (arg3 < 0 || arg3 > 15) {
                  break;
                }

                if ((cfg & 0x0F) != 0x00) {
                  Serial.printf("[HW SEL IO SET] WARN: P0~P3 not all OUTPUT (cfg=0x%02X)\n", cfg);
                }

                out = (out & 0xF0) | (arg3 & 0x0F);
              } else {
                break;
              }

              ioEx_IO.writePort(out);
              Serial.printf("[HW SEL IO SET] cfg=0x%02X out=0x%02X\n", cfg, out);
              break;
            }
            case 20: {
              // ba+set=20,<recipe_index 0~3>,<reserved>
              if (arg2 < 0 || arg2 >= configManager.getRecipeCount()) {
                break;
              }
              configManager.setOperationMode((uint8_t)arg2);
              applyOperationModeTemperatureLimits();
              break;
            }
            case RECIPE_FIELD_FUEL_TYPE:
            case RECIPE_FIELD_SMOKE_ENABLE:
            case RECIPE_FIELD_IGNITE_T1:
            case RECIPE_FIELD_IGNITE_T2:
            case RECIPE_FIELD_PUMP_CONDITION:
            case RECIPE_FIELD_PUMP_POWER:
            case RECIPE_FIELD_REIGNITE_T1:
            case RECIPE_FIELD_REIGNITE_T2:
            case RECIPE_FIELD_PUMP_ON_SEC:
            case RECIPE_FIELD_PUMP_OFF_SEC:
            case RECIPE_FIELD_COOK_MINUTES:
            case RECIPE_FIELD_COOK_OVEN_MIN:
            case RECIPE_FIELD_COOK_OVEN_MAX:
            case RECIPE_FIELD_COOK_OVEN_ERROR_PCT:
            case RECIPE_FIELD_COOK_HEATER_ON_SEC:
            case RECIPE_FIELD_COOK_HEATER_OFF_SEC:
            case RECIPE_FIELD_COOK_FAN_ON_SEC:
            case RECIPE_FIELD_COOK_FAN_OFF_SEC:
            case RECIPE_FIELD_COOK_SPRAY_TIME_SEC:
            case RECIPE_FIELD_COOK_SPRAY_POWER:
            case RECIPE_FIELD_DRY_MINUTES:
            case RECIPE_FIELD_DRY_OVEN_MIN:
            case RECIPE_FIELD_DRY_OVEN_MAX:
            case RECIPE_FIELD_DRY_OVEN_ERROR_PCT:
            case RECIPE_FIELD_DRY_HEATER_ON_SEC:
            case RECIPE_FIELD_DRY_HEATER_OFF_SEC:
            case RECIPE_FIELD_HOLD_MINUTES:
            case RECIPE_FIELD_HOLD_OVEN_MIN:
            case RECIPE_FIELD_HOLD_OVEN_MAX:
            case RECIPE_FIELD_HOLD_OVEN_ERROR_PCT:
            case RECIPE_FIELD_HOLD_HEATER_ON_SEC:
            case RECIPE_FIELD_HOLD_HEATER_OFF_SEC: {
              if (arg2 < 0 || arg2 >= configManager.getRecipeCount()) {
                break;
              }
              if (!configManager.setRecipeField((uint8_t)arg2, (uint8_t)arg1, (int32_t)arg3)) {
                break;
              }
              if ((uint8_t)arg2 == configManager.getOperationMode()) {
                applyOperationModeTemperatureLimits();
              }
              break;
            }
            default:
              break;
          }
        } else {
        }
      } else if (commandType == "get") {
        // ba+get=arg1,arg2,arg3
        // arg1: 0(all), 1(DEBUG_JSON), 2(HW_BD_ID), 5(MQTT), 6(CAN), 7(RELAY), 8(GPO&GPI), 9(FET), 10(INT_DAC&EXT_DAC), 12(ADC 8ch), 13(TEMP 6ch), 14(TC), 15(STATE_UART_LOG), 16(HW_SEL_IO_OUT), 20(RECIPE)
        if (parsed >= 1) {
          switch (arg1) {
            case 0:
              Serial.printf("[DEBUG JSON] status=%s\n", batagota.isDebugJsonPrintEnabled() ? "ENABLED" : "DISABLED");
              Serial.printf("[STATE UART LOG] status=%s\n", batagota.isStateUartLogEnabled() ? "ENABLED" : "DISABLED");
              Serial.printf("[MQTT CFG] status=%s\n", g_mqttLogEnabled ? "ENABLED" : "DISABLED");
              Serial.printf("[CAN CFG] status=%s\n", g_canCommEnabled ? "ENABLED" : "DISABLED");
              break;
            case 1:
              Serial.printf("[DEBUG JSON] status=%s\n", batagota.isDebugJsonPrintEnabled() ? "ENABLED" : "DISABLED");
              break;
            case 2: {
              uint8_t rawPort = Get_IO21_Raw_Port_Status();
              int liveBoardId = rawPort & 0x0F;
              Serial.printf("[HW BD ID] raw=0x%02X live=%d cached=%d\n", rawPort, liveBoardId, rSys.hw_bd_id_num);
              break;
            }
            case 16: {
              uint8_t cfg = Get_IO21_Config_Status();
              uint8_t out = Get_IO21_Output_Port_Status();
              uint8_t in = Get_IO21_Raw_Port_Status();
              Serial.printf("[HW SEL IO] cfg=0x%02X out=0x%02X in=0x%02X\n", cfg, out, in);
              Serial.printf("[HW SEL IO P0~P3] cfg=%d%d%d%d out=%d%d%d%d in=%d%d%d%d\n",
                            (cfg >> 3) & 0x01, (cfg >> 2) & 0x01, (cfg >> 1) & 0x01, cfg & 0x01,
                            (out >> 3) & 0x01, (out >> 2) & 0x01, (out >> 1) & 0x01, out & 0x01,
                            (in >> 3) & 0x01, (in >> 2) & 0x01, (in >> 1) & 0x01, in & 0x01);
              break;
            }
            case 5:
              Serial.printf("[MQTT CFG] status=%s\n", g_mqttLogEnabled ? "ENABLED" : "DISABLED");
              break;
            case 6:
              Serial.printf("[CAN CFG] status=%s\n", g_canCommEnabled ? "ENABLED" : "DISABLED");
              break;
            case 7: {
              Serial.print("[RELAY] ");
              for (int i = 0; i < 8; i++) {
                Serial.printf("CH%d=%d ", i, relayStatus[i]);
              }
              Serial.println();
              break;
            }
            case 8:
              Serial.print("[GPO] ");
              for (int i = 0; i < 2; i++) {
                Serial.printf("CH%d=%d ", i, ioBoard.gpo[i]);
              }
              Serial.println();
              Serial.print("[GPI] ");
              for (int i = 0; i < 2; i++) {
                Serial.printf("CH%d=%d ", i, ioBoard.gpi[i]);
              }
              Serial.println();
              break;
            case 9:
              Serial.print("[FET] ");
              for (int i = 0; i < 8; i++) {
                Serial.printf("CH%d=%d ", i, ioBoard.fet[i]);
              }
              Serial.println();
              break;
            case 10:
              Serial.print("[INT_DAC] ");
              for (int i = 0; i < 2; i++) {
                Serial.printf("CH%d=%d ", i, (int)ioBoard.dac[i]);
              }
              Serial.println();
              Serial.print("[EXT_DAC] ");
              for (int i = 0; i < 8; i++) {
                int raw12 = map(ioBoard.dac7678_target[i], 0, 500, 0, 4095);
                Serial.printf("CH%d=%d ", i, raw12);
              }
              Serial.println();
              break;
            case 12:
              Serial.print("[ADC 8CH] ");
              for (int i = 0; i < 8; i++) {
                Serial.printf("CH%d=%.3fV ", i, ioBoard.ads1015_adc[i]);
              }
              Serial.println();
              break;
            case 13:
              Serial.print("[TEMP 6CH] ");
              for (int i = 0; i < 6; i++) {
                Serial.printf("CH%d=%.2fC ", i, ioBoard.temperature[i]);
              }
              Serial.println();
              break;
            case 14:
              Serial.print("[TC] ");
              for (int i = 0; i < 4; i++) {
                Serial.printf("CH%d=%.2fC ", i, ioBoard.TC_average[i]);
              }
              Serial.println();
              break;
            case 15:
              Serial.printf("[STATE UART LOG] status=%s\n", batagota.isStateUartLogEnabled() ? "ENABLED" : "DISABLED");
              break;
            case 20: {
              uint8_t recipeIndex = configManager.getOperationMode();
              if (arg2 >= 0 && arg2 < configManager.getRecipeCount()) {
                recipeIndex = (uint8_t)arg2;
              }
              const RecipeStageProfile& recipe = configManager.getRecipeProfile(recipeIndex);
              Serial.printf("[RECIPE] index=%u fuel_type=%u\n", recipeIndex, recipe.fuel_type);
              Serial.printf("[RECIPE][IGN] smoke=%u ignite_t1=%u ignite_t2=%u pump_cond=%u pump_power=%u reignite_t1=%u reignite_t2=%u pump_on=%u pump_off=%u\n",
                            recipe.ignition.smoke_enable,
                            recipe.ignition.ignite_t1,
                            recipe.ignition.ignite_t2,
                            recipe.ignition.pump_condition,
                            recipe.ignition.pump_power,
                            recipe.ignition.reignite_t1,
                            recipe.ignition.reignite_t2,
                            recipe.ignition.pump_on_sec,
                            recipe.ignition.pump_off_sec);
              Serial.printf("[RECIPE][COOK] min=%u max=%u err_pct=%u cook_min=%u heater_on=%u heater_off=%u fan_on=%u fan_off=%u spray_t=%u spray_p=%u\n",
                            recipe.cooking.oven_min,
                            recipe.cooking.oven_max,
                            recipe.cooking.oven_error_pct,
                            recipe.cooking.cook_minutes,
                            recipe.cooking.heater_on_sec,
                            recipe.cooking.heater_off_sec,
                            recipe.cooking.fan_on_sec,
                            recipe.cooking.fan_off_sec,
                            recipe.cooking.spray_time_sec,
                            recipe.cooking.spray_power);
              Serial.printf("[RECIPE][DRY] min=%u max=%u err_pct=%u dry_min=%u heater_on=%u heater_off=%u\n",
                            recipe.drying.oven_min,
                            recipe.drying.oven_max,
                            recipe.drying.oven_error_pct,
                            recipe.drying.dry_minutes,
                            recipe.drying.heater_on_sec,
                            recipe.drying.heater_off_sec);
              Serial.printf("[RECIPE][HOLD] min=%u max=%u err_pct=%u hold_min=%u heater_on=%u heater_off=%u\n",
                            recipe.holding.oven_min,
                            recipe.holding.oven_max,
                            recipe.holding.oven_error_pct,
                            recipe.holding.hold_minutes,
                            recipe.holding.heater_on_sec,
                            recipe.holding.heater_off_sec);
              break;
            }
            default:
              break;
          }
        } else {
        }
      } else if (commandType == "mqtt") {
        // ba+mqtt=arg1,arg2,arg3
        // arg1: 1(enable), 0(disable)
        // arg2: 1(persist), 0(no persist)
        if (parsed >= 2) {
          bool persist = (arg2 != 0);

          switch (arg1) {
            case 0:
              setMqttLogEnable(false, "UART_BA_MQTT", persist);
              break;
            case 1:
              setMqttLogEnable(true, "UART_BA_MQTT", persist);
              break;
            default:
              break;
          }
        } else {
        }
      } else if (commandType == "can") {
        // ba+can=arg1,arg2,arg3
        // arg1: 1(enable), 0(disable)
        // arg2: 1(persist), 0(no persist)
        if (parsed >= 2) {
          bool persist = (arg2 != 0);

          switch (arg1) {
            case 0:
              setCanCommEnable(false, "UART_BA_CAN", persist);
              break;
            case 1:
              setCanCommEnable(true, "UART_BA_CAN", persist);
              break;
            default:
              break;
          }
        } else {
        }
      } else {
        // 그 외 ba+ 명령은 기존 batagota 명령 파서로 전달
        if (!batagota.handleUartCommandLine(uartBuffer.c_str())) {
        }
      }
      uartBuffer = "";  // 버퍼 초기화
    }
    // 버퍼 오버플로 방지
    if (uartBuffer.length() > UART_BUFFER_SIZE) {
      uartBuffer = "";
    }
  }
  // 타임아웃으로 인한 버퍼 정리
  if (uartBuffer.length() > 0 && (millis() - lastUartTime > UART_TIMEOUT_MS)) {
    uartBuffer = "";
  }
}
void check_device_info(void)
{
  uint64_t chipid = ESP.getEfuseMac(); // Get the chip ID
    Serial.printf("Chip ID: %04X\n", (uint32_t)(chipid >> 32));
  Serial.printf("MAC Address: %04X%08X\n", (uint32_t)(chipid >> 32), (uint32_t)chipid);

  // Flash Chip Size
  Serial.printf("Flash Chip Size: %u MB\n", ESP.getFlashChipSize() / (1024 * 1024));

  // CPU Frequency
  Serial.printf("CPU Frequency: %u MHz\n", ESP.getCpuFreqMHz());

  // SDK Version
  Serial.printf("SDK Version: %s\n", ESP.getSdkVersion());

  // Chip Revision
  Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());

  // Free Heap Size
  Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
}

static bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t &value) {
  while (Wire.available()) {
    Wire.read();
  }

  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  delay(1);
  if (Wire.requestFrom((int)addr, 1) != 1) {
    return false;
  }

  value = Wire.read();
  return true;
}

static void runI2CAddressScanAndDump(void) {
  Serial.println("=== I2C Address Scan Start ===");

  uint8_t found[112];
  uint8_t foundCount = 0;

  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      found[foundCount++] = addr;
      Serial.printf("[I2C SCAN] found addr=0x%02X\n", addr);
    }
  }

  if (foundCount == 0) {
    Serial.println("[I2C SCAN] no device found");
    Serial.println("=== I2C Address Scan End ===");
    return;
  }

  Serial.println("=== I2C Register Dump (0x00~0x03) ===");
  for (uint8_t i = 0; i < foundCount; i++) {
    uint8_t addr = found[i];
    Serial.printf("[I2C DUMP] addr=0x%02X ", addr);

    for (uint8_t reg = 0x00; reg <= 0x03; reg++) {
      uint8_t v = 0xFF;
      bool ok = i2cReadReg(addr, reg, v);
      if (ok) {
        Serial.printf("r%02X=0x%02X ", reg, v);
      } else {
        Serial.printf("r%02X=ERR ", reg);
      }
    }
    Serial.println();
  }

  Serial.println("=== I2C Address Scan End ===");
}

#define PIN_BOOT 0  // Boot mode pin, set to GPIO0 (D0) for ESP32
void setup() {
  
  delay(100);                  // 핀 상태 안정화

  Serial.begin(115200);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  Serial.println("PIN_BOOT configured.");

  delay(1000); // Reset 후 1초 대기

  
  // I2C 진단 추가
  Wire.begin(19, 18); // SDA=19, SCL=18
  Wire.setClock(400000); // 전체 구간 400kHz 고정
  Serial.println("I2C initialized at 400kHz");

  // Boot 직후 DAC를 안전값(0 target)으로 즉시 기록해 모터 drive 시간을 최소화한다.
  dac7678Manager.init();
  Serial.println("DAC7678 early-safe init done.");
  
  // I2C 주소 스캔 및 레지스터 덤프
  runI2CAddressScanAndDump();
  
  // Serial.println("=== I2C Device Scan ===");
  // bool i2c_found = false;
  // for (byte addr = 8; addr < 120; addr++) {
  //   Wire.beginTransmission(addr);
  //   byte error = Wire.endTransmission();
  //   if (error == 0) {
  //     Serial.printf("I2C device found at 0x%02X\n", addr);
  //     i2c_found = true;
  //   }
  // }
  // if (!i2c_found) {
  //   Serial.println("No I2C devices found! Check wiring:");
  //   Serial.println("- SDA (GPIO19) connection");
  //   Serial.println("- SCL (GPIO18) connection");
  //   Serial.println("- 4.7kΩ pull-up resistors");
  //   Serial.println("- 3.3V power supply");
  // }
  // Serial.println("=====================");

  configManager.load();
  printTemperatureLimitBootInfo();
  check_device_info();

  // I2C 초기화 완료 후 보드 ID 읽기 (WiFi 설정 전에 확인)
  TCA9534_Setup();      // IO Expander 초기화 (Board ID 읽기를 위해 먼저 실행)
  rSys.hw_bd_id_num = Get_Board_ID_Number();

  // CAN 모드 결정
  bool isMasterMode = (rSys.hw_bd_id_num == 0);
  g_canCommEnabled = loadCanCommEnableFlag();
  g_mqttLogEnabled = loadMqttLogEnableFlag();

  Serial.printf("Board ID Number: %d -> %s MODE\n", 
                rSys.hw_bd_id_num, 
                isMasterMode ? "CAN MASTER" : "CAN SLAVE");
  Serial.printf("[CAN CFG] Boot flag: %s\n", g_canCommEnabled ? "ENABLED" : "DISABLED");
  Serial.printf("[MQTT CFG] Boot flag: %s\n", g_mqttLogEnabled ? "ENABLED" : "DISABLED");

  Serial.println("ConfigManager loaded.");
  bool wifiConnected = false;
  
  // WiFi 및 MQTT는 MASTER 모드에서만 활성화
  if (isMasterMode) {
    if (digitalRead(PIN_BOOT) == LOW || !configManager.hasValidWiFiConfig()) {
      Serial.println("Entering AP mode (PIN_BOOT LOW or no valid WiFi config).");
      wifiManager.startAPMode(); // This function blocks until reboot
    } else {
      Serial.println("Connecting to WiFi with stored credentials.");
      wifiConnected = wifiManager.connectToWiFi();
    }
    // Initialize NTP only when WiFi is connected.
    if (wifiConnected) {
      timeService.init(); 
      Serial.println("NTP Time Service initialized.");
    } else {
      Serial.println("[NET-FAIL] Skip NTP init (WiFi not connected).");
    }
  } else {
    Serial.println("SLAVE MODE: WiFi and MQTT disabled, using CAN communication only.");
    
    // SLAVE 모드: 8채널 ADC 활성화 (WiFi 충돌 없음)
    initInternalADC_AllChannels();
  }
  
  uartService.init();
  
  // CAN 초기화 (500kbps) - MASTER/SLAVE 모드 설정
  if (g_canCommEnabled) {
    if (isMasterMode) {
      canService.init(500000); // MASTER: 0x100 번지부터 데이터 송신
      
      // MASTER 모드에서도 CAN RX interrupt 콜백 설정 (SLAVE 응답 수신용)
      canService.setRxInterruptCallback(canRxInterruptHandler);
      
      // MASTER 모드 CAN RX 데이터 초기화
      for (int i = 0; i < MAX_CAN_BOARDS; i++) {
          canRxData[i].hasNewData = false;
          canRxData[i].lastRxTime = 0;
          canRxData[i].lastRxId = 0;
          canRxData[i].messageCount = 0;
          memset((void*)canRxData[i].rxBuffer, 0, 72);
          memset((void*)&canRxData[i].sensorData, 0, sizeof(CANSensorPacket_t));
      }
    } else {
      canService.init(500000); // SLAVE: 0x100 + ID*0x100 번지로 응답
      
      // SLAVE 모드에서 CAN RX interrupt 콜백 설정
      canService.setRxInterruptCallback(canRxInterruptHandler);
      
      // CAN RX 데이터 초기화
      for (int i = 0; i < MAX_CAN_BOARDS; i++) {
          canRxData[i].hasNewData = false;
          canRxData[i].lastRxTime = 0;
          canRxData[i].lastRxId = 0;
          canRxData[i].messageCount = 0;
          memset((void*)canRxData[i].rxBuffer, 0, 72);
          memset((void*)&canRxData[i].sensorData, 0, sizeof(CANSensorPacket_t));
      }
      
      // TX 제어 초기화
      canTxControl.txFlag = false;
      canTxControl.waitingToTx = false;
      canTxControl.lastRxTime = 0;
      canTxControl.prevBoardId = (rSys.hw_bd_id_num > 0) ? (rSys.hw_bd_id_num - 1) : 15;
      canTxControl.txDelayTime = 50; // 50ms
      
      uint32_t slaveAddress = 0x100 + (rSys.hw_bd_id_num * 0x100);
      Serial.printf("CAN SLAVE initialized: Interrupt-based RX, Will transmit on 0x%03X\n", slaveAddress);
      Serial.printf("CAN SLAVE: Waiting for data from Board %d\n", canTxControl.prevBoardId);
    }
  } else {
    Serial.println("[CAN CFG] CAN communication is DISABLED. CAN init skipped.");
  }
  
  // MQTT 연결은 MASTER 모드에서만
  if (isMasterMode) {
    if (!wifiConnected) {
      Serial.println("[NET-FAIL] Skip MQTT init (WiFi not connected).");
    } else if (g_mqttLogEnabled) {
      // MQTT log number는 Flash 저장값 사용 (HW SEL ID 자동 덮어쓰기 제거)
      Serial.printf("[MQTT] mqttlogNumber from Flash: %d\n", configManager.getConfig().mqttlogNumber);

      if (!mqttService.begin(configManager.getConfig().mqttServer,
                             configManager.getConfig().mqttPort,
                             configManager.getConfig().mqttClientId,
                             configManager.getConfig().mqttId,
                             configManager.getConfig().mqttPass)) {
        Serial.println("[MQTT] Initialization failed. Continuing without MQTT.");
      }
    } else {
      Serial.println("[MQTT CFG] MQTT log disabled. MQTT connection skipped at boot.");
    }
  } else {
    Serial.println("SLAVE MODE: MQTT service disabled.");
  }
  
  ioBoard.init();       // board resource value init
  // TCA9534_Setup();      // IO Expander 초기화 (이미 위에서 실행됨)
  
  Serial.println("IOBoard Service initialized.");
  
  ads1015Manager.init();  // ADS1015 ADC 초기화
  Serial.println("ADS1015 ADC Manager initialized.");
  
  Serial.println("DAC7678 I2C Manager initialized (already early-safe initialized).");
  
  // ESP32 내부 ADC 초기화 - SLAVE 모드에서는 8채널 모두 활성화
  if (isMasterMode) {
    initInternalADC();  // 기존 6채널 초기화
    Serial.println("ESP32 Internal ADC initialized (6 channels for MASTER).");
  } else {
    initInternalADC_AllChannels();  // 8채널 모두 활성화
    Serial.println("ESP32 Internal ADC initialized (8 channels for SLAVE).");
  }  // ESP32 내부 DAC 초기화
  initInternalDAC();
  Serial.println("ESP32 Internal DAC initialized.");
  
  // GPIO 입출력 초기화
  initGPIO();
  Serial.println("GPIO Input/Output initialized.");
  
  // MAX31856 Thermocouple 초기화
  initThermoCoupleSystem();
  Serial.println("MAX31856 Thermocouple System initialized.");
  
  timeService.start1msTimer();
  Serial.println("1ms Timer started.");
  printConfig();
  
  // Batagota 제어 알고리즘 초기화
  // All channels are now statically defined via enums, no dynamic registration needed
  batagota.init();
  batagota.clearErrorHistory();  // Boot 시 Error history 초기화
  applyOperationModeTemperatureLimits();
  batagota.stateInit();  // State Machine 초기화
  Serial.println("Batagota Control Algorithm initialized.");
  Serial.println("  - ADC Channels: NTC_Thermistor_0, NTC_Thermistor_1");
  Serial.println("  - Temperature Channels: BBQ1, BBQ2, BBQ3, BBQ4, SMOKE_TANK, OUTSIDE");
  Serial.println("  - All channels auto-initialized with enum-based definitions");
  Serial.println("  - State Machine: START_INIT");
}

void mqtt_cmd_check(void) {   // uart command send to io board by Seril1 uart channel
  // check and apply subscribed changes only if different
  for (int i = 0; i < 8; i++) {
    if (ioBoard.fet[i] != ioBoardBackup.fet[i]) {
      // apply updated FET value here if needed (e.g., digitalWrite)
      // ioBoardBackup.fet[i] = ioBoard.fet[i];          // backup copy from io board uart status message
      // String msg = "FET[" + String(i) + "]=" + String(ioBoard.fet[i]);
      // Serial.println(msg);
    }
  }
  for (int i = 0; i < 2; i++) {
    if (abs(ioBoard.dac[i] - ioBoardBackup.dac[i]) > 0.01) {
      // apply updated DAC value here if needed
      // ioBoardBackup.dac[i] = ioBoard.dac[i];          // backup copy from io board uart status message
      // String msg = "DAC[" + String(i) + "]=" + String(ioBoard.dac[i], 2);
      // Serial.println(msg);
    }
    if (ioBoard.gpo[i] != ioBoardBackup.gpo[i]) {
      // apply updated GPO value here if needed
      // ioBoardBackup.gpo[i] = ioBoard.gpo[i];          // backup copy from io board uart status message
      // String msg = "GPO[" + String(i) + "]=" + String(ioBoard.gpo[i]);
      // Serial.println(msg);
    }
  }
}
void loop() {
  bool mcu_only_flag =false;
  static unsigned long max_loop_time_ms=0;
  unsigned long loop_start_ms = millis();
  
  // Batagota 제어 알고리즘 타이밍 제어
  static unsigned long lastCall10mSec = 0;
  static unsigned long lastCall100mSec = 0;
  static unsigned long lastCall1Sec = 0;
  
  unsigned long currentTime = millis();
  
  // 10ms 호출
  if (currentTime - lastCall10mSec >= 10) {
    lastCall10mSec = currentTime;
    batagota.loop10mSec();
  }
  
  // 100ms 호출
  if (currentTime - lastCall100mSec >= 100) {
    lastCall100mSec = currentTime;
    batagota.loop100mSec();
    LED_ch2_Toggle();
  }
  
  // 1초 호출
  if (currentTime - lastCall1Sec >= 1000) {
    lastCall1Sec = currentTime;
    batagota.loop1Sec();
  LED_ch1_Toggle();

    // DAC test loops run before stateUpdate so JSON reflects current-cycle values.
    // DAC7678_Test_Loop();
    // Internal_DAC_Test_Loop();

      // Thermocouple 데이터를 int16_t 배열로 변환 (0.1도 단위)
      int16_t tc_data[4];
      for (int i = 0; i < 4; i++) {
          tc_data[i] = (int16_t)(ioBoard.TC[i] * 10);
      }
      
       batagota.stateUpdate(ioBoard.fet, relayStatus, ioBoard.led, ioBoard.gpo, ioBoard.gpi, tc_data, ioBoard.dac, ioBoard.dac7678_target);
    if (canTransmissionCompleted) {
      canTransmissionCompleted = false;  // 플래그 리셋
      processCANCommands();              // 수신된 CAN 메시지 기반 명령 처리
    }
  }
  
  // 패킷 타임아웃 체크 (10초마다)
  static unsigned long lastTimeoutCheck = 0;
  if (currentTime - lastTimeoutCheck >= 10000) {
    lastTimeoutCheck = currentTime;
    checkPacketTimeouts();
  }
  
  // 순차 CAN 전송 처리 (우선순위 높음)
  if (g_canCommEnabled) {
    canPacketManager.handleSequentialTransmission();
  }
  
  // Board ID에 따른 통신 처리
  if (rSys.hw_bd_id_num == 0) {
    // MASTER 모드: MQTT + CAN 통신 (interrupt 기반 RX 추가)

    if (g_mqttLogEnabled && WiFi.status() == WL_CONNECTED) {
      mqttService.loop();
    }

    if (g_canCommEnabled) {
      // CAN interrupt 플래그 체크 및 처리 (MASTER도 SLAVE 응답을 interrupt로 수신)
      if (canInterruptFlag) {
          canInterruptFlag = false;
          processCAN_RX_Interrupt();
          // Serial.print("M"); // MASTER에서 interrupt 수신 표시
      }

      // CANService loop (하드웨어 레벨 메시지 감지)
      canService.loop();
      handleCANMasterCommunication();
    }
  } else {
    // SLAVE 모드: Interrupt 기반 CAN 통신
    if (g_canCommEnabled) {
      handleCANSlaveInterruptBased();
    }
  }
  
  // canService.loop();
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= interval) {                 // 1Sec loop
    lastMillis = currentMillis;
    mqtt_cmd_check();       // Check and send commands to IO Board via UART
    
    // MQTT는 MASTER 모드에서만 처리
    if (rSys.hw_bd_id_num == 0 && g_mqttLogEnabled && WiFi.status() == WL_CONNECTED) {
      // MQTT publish 전에 연결 상태 확인
      mqttService.loop(); // MQTT loop to maintain connection and process messages
      if (client.connected()) {
        mqttService.publish();
      } else {
        // Serial.println("[MAIN] MQTT not connected, skipping publish");
      }
    }
    
    timeService.update();
    
    // print_current_time();  // 현재 시간 출력
    
    // FET 순차 테스트 (채널 0~7번 순서대로 1초마다 shift)
    // FET_Test_Loop();

    // Relay 순차 테스트 (채널 0~7번 순서대로 1초마다 shift)
    // RELAY_Test_Loop();
    
    // LED test toggle 비활성화 (hw sel id 핀과 LED 연계 제거)
  }

  else if(currentMillis - Millis100mSec >= interval100mSec){      // 100mSec loop
    Millis100mSec = currentMillis;
    if(mcu_only_flag != true){
      // ADS1015 ADC 데이터 읽기 (100ms마다)
      ads1015Manager.updateData();

      // Debug JSON의 adc 필드와 ba+get ADC 값을 동일 소스로 맞춘다.
      for (int i = 0; i < 8; i++) {
        batagota.setExternalADCValue((Batagota::ExternalADCInputChannel)i, ioBoard.ads1015_adc[i]);
      }
      
      // ESP32 내부 ADC 읽기 (100ms마다)
      readInternalADC();
      
      // Batagota에 온도값 전달 (ADC에서 계산된 온도)
      // NOTE: InternalADCChannel로 정의된 8개 채널 중 6개 사용 (BBQ1-4, SMOKE_TANK, OUTSIDE)
      for (int i = 0; i < 6; i++) {  // 6개의 활성 온도 제어 채널
        if (ioBoard.temperature[i] > -999.0) {  // 유효한 온도값만
          batagota.setInternalTemperatureValue((Batagota::InternalADCChannel)i, ioBoard.temperature[i]);
        }
      }
      // 나머지 2개 채널(INT_TEMP_6, INT_TEMP_7)은 예약됨 (향후 확장 가능)
      
      // GPIO 입력 읽기 (100ms마다)
      readGPIOInputs();
      
      // GPIO 출력 업데이트 (100ms마다)
      updateGPIOOutputs();
      
      // LED test toggle 비활성화 (hw sel id 핀과 LED 연계 제거)

      // State 변화 기반 buzzer 패턴 실행
      uint8_t buzzerCount = 0;
      uint8_t buzzerId = 0;
      if (batagota.consumePendingBuzzerCommand(buzzerCount, buzzerId)) {
        beepResetAndStart(buzzerCount, buzzerId);
      }
      UpdateBuzzerDriver100mSec();
      
      // DAC7678 업데이트 (100ms마다)
      dac7678Manager.updateOutputs();

      // TCA9534 업데이트 (100ms마다)
      TCA9534_Update();  // FET & LED status update
    }
  }

  else if(currentMillis - Millis200mSec >= interval200mSec){      // 250mSec loop
    Millis200mSec = currentMillis;
    if(mcu_only_flag != true){
      // MAX31856 TC 데이터 읽기 (250ms마다)
      Get_MAX31856_Data();
    }
  }
  
  // ESP32 내부 DAC 업데이트 (10ms 간격으로 자체 제어)
  updateInternalDAC();
  
  handleUartReceive();  // UART 메시지 수신 체크
  if (flag_1s) {
    flag_1s = false;
  }
  if(timeService.check1msFlag()){
    timeService.clear1msFlag();
    // 1ms 타이머 이벤트 처리
    // 예: ADC 샘플링, 센서 데이터 업데이트 등
    // ioBoard.update();  // ADC 업데이트 (필요시)
  }

  unsigned long loop_elapsed_ms = millis() - loop_start_ms;
  if (loop_elapsed_ms > max_loop_time_ms) {
    max_loop_time_ms = loop_elapsed_ms;
    Serial.printf("Max loop time: %lu ms\n", max_loop_time_ms);
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// MQTT 로그 토픽 처리 함수
void handleLogTopic(const String& message) {
  Serial.println("[MQTT Log] " + message);
  // 로그 메시지 처리 로직을 여기에 추가할 수 있습니다.
  // 예: SD 카드에 저장, 특정 로그 레벨 필터링 등
}

// 명령 전송 함수
void handleCmdTopic(const String& message);

void sendIOBoardCommand(const String& type, int channel, int value) {
  String cmd = "EXCTL_:set:";
  cmd += type;
  cmd += "=" + String(channel) + ":" + String(value) + ":0\r\n";
  Serial.print(cmd);
}
// MQTT 콜백 및 명령 처리 함수들
// void handleCmdTopic(String payload) {
void handleCmdTopic(const String& payload){
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.println("JSON parse error");
    return;
  }

  // HMI UART <-> MQTT command protocol bridge
  // Priority handling for:
  //  - {"set":[cmd,ch,val]}
  //  - {"run":[cmd,arg1,arg2]}
  //  - {"recipe":{...}}
  // Unknown/invalid protocol payload falls through to legacy key handlers below.
  if (doc.containsKey("set")) {
    JsonArray setArr = doc["set"].as<JsonArray>();
    if (setArr.isNull() || setArr.size() != 3) {
      Serial.println("[MQTT CMD] Invalid set payload. Use {\"set\":[cmd,ch,val]}");
      return;
    }

    int cmd = setArr[0] | INT32_MIN;
    int ch = setArr[1] | INT32_MIN;
    int val = setArr[2] | INT32_MIN;
    if (cmd == INT32_MIN || ch == INT32_MIN || val == INT32_MIN) {
      Serial.println("[MQTT CMD] Invalid set payload value type");
      return;
    }

    switch (cmd) {
      case 7: {
        if (ch < 0 || ch > 7 || (val != 0 && val != 1)) {
          Serial.println("[MQTT CMD] set=7 invalid range (ch:0~7, val:0|1)");
          return;
        }
        relayStatus[ch] = val;
        Serial.printf("[MQTT CMD] RELAY CH%d <- %d\n", ch, val);
        return;
      }
      case 8: {
        if (ch < 0 || ch > 1 || (val != 0 && val != 1)) {
          Serial.println("[MQTT CMD] set=8 invalid range (ch:0~1, val:0|1)");
          return;
        }
        ioBoard.gpo[ch] = val;
        Serial.printf("[MQTT CMD] GPO CH%d <- %d\n", ch, val);
        return;
      }
      case 9: {
        if (ch < 0 || ch > 7 || (val != 0 && val != 1)) {
          Serial.println("[MQTT CMD] set=9 invalid range (ch:0~7, val:0|1)");
          return;
        }
        ioBoard.fet[ch] = val;
        Set_TCA9534_FET_ch_Status((uint8_t)ch, (uint8_t)val);
        ioBoardBackup.fet[ch] = val;
        Serial.printf("[MQTT CMD] FET CH%d <- %d\n", ch, val);
        return;
      }
      case 10: {
        if (ch < 0 || ch > 1 || val < 0 || val > 4095) {
          Serial.println("[MQTT CMD] set=10 invalid range (ch:0~1, val:0~4095)");
          return;
        }
        ioBoard.dac[ch] = (float)val;
        Serial.printf("[MQTT CMD] INT_DAC CH%d <- %d\n", ch, val);
        return;
      }
      case 11: {
        if (ch < 0 || ch > 7) {
          Serial.println("[MQTT CMD] set=11 invalid range (ch:0~7)");
          return;
        }
        int extDacTarget = val;
        if (extDacTarget < 0) {
          extDacTarget = 0;
        } else if (extDacTarget > 500) {
          extDacTarget = 500;
        }
        ioBoard.dac7678_target[ch] = extDacTarget;
        Serial.printf("[MQTT CMD] EXT_DAC CH%d <- %d (%.2fV)\n", ch, extDacTarget, extDacTarget / 100.0f);
        return;
      }
      case 20: {
        if (ch < 0 || ch >= configManager.getRecipeCount()) {
          Serial.printf("[MQTT CMD] set=20 invalid recipe index. Use 0-%u\n", (unsigned)(configManager.getRecipeCount() - 1));
          return;
        }
        configManager.setOperationMode((uint8_t)ch);
        applyOperationModeTemperatureLimits();
        Serial.printf("[MQTT CMD] ACTIVE_RECIPE <- %d\n", ch);
        return;
      }
      default:
        Serial.printf("[MQTT CMD] Unsupported set cmd=%d. Fallback to legacy handler\n", cmd);
        break;
    }
  }

  if (doc.containsKey("run")) {
    JsonArray runArr = doc["run"].as<JsonArray>();
    if (runArr.isNull() || runArr.size() != 3) {
      Serial.println("[MQTT CMD] Invalid run payload. Use {\"run\":[cmd,arg1,arg2]}");
      return;
    }

    int cmd = runArr[0] | INT32_MIN;
    int arg1 = runArr[1] | INT32_MIN;
    int arg2 = runArr[2] | INT32_MIN;
    if (cmd == INT32_MIN || arg1 == INT32_MIN || arg2 == INT32_MIN) {
      Serial.println("[MQTT CMD] Invalid run payload value type");
      return;
    }

    String uartLine = "ba+run=" + String(cmd) + "," + String(arg1) + "," + String(arg2);
    if (!batagota.handleUartCommandLine(uartLine.c_str())) {
      Serial.println("[MQTT CMD] run command rejected by Batagota parser");
      return;
    }
    Serial.printf("[MQTT CMD] %s\n", uartLine.c_str());
    return;
  }

  if (doc.containsKey("recipe")) {
    JsonObject recipeObj = doc["recipe"].as<JsonObject>();
    if (recipeObj.isNull()) {
      Serial.println("[MQTT CMD] Invalid recipe payload. Use {\"recipe\":{...}}");
      return;
    }

    int recipeNo = recipeObj["recipe_no"] | -1;
    int stageNo = recipeObj["stage_no"] | -1;
    if (recipeNo < 0 || recipeNo >= configManager.getRecipeCount()) {
      Serial.printf("[MQTT CMD] Invalid recipe_no. Use 0-%u\n", (unsigned)(configManager.getRecipeCount() - 1));
      return;
    }
    if (stageNo < 0 || stageNo > 3) {
      Serial.println("[MQTT CMD] Invalid stage_no. Use 0~3");
      return;
    }

    JsonObject dataObj;
    if (recipeObj.containsKey("data")) {
      dataObj = recipeObj["data"].as<JsonObject>();
    } else {
      dataObj = recipeObj;
    }

    if (dataObj.isNull()) {
      Serial.println("[MQTT CMD] Invalid recipe.data object");
      return;
    }

    if (!applyRecipeJsonByStage((uint8_t)recipeNo, (uint8_t)stageNo, dataObj)) {
      Serial.println("[MQTT CMD] Failed to apply recipe JSON (validation failed)");
      return;
    }

    if ((uint8_t)recipeNo == configManager.getOperationMode()) {
      applyOperationModeTemperatureLimits();
    }

    Serial.printf("[MQTT CMD] RECIPE_JSON applied: recipe=%d stage=%d\n", recipeNo, stageNo);
    return;
  }

  // CAN 통신 enable/disable 설정 (MQTT)
  // Example: {"CAN_ENABLE":1} or {"CAN_ENABLE":true}
  if (doc.containsKey("CAN_ENABLE")) {
    bool enableCmd = false;
    if (doc["CAN_ENABLE"].is<bool>()) {
      enableCmd = doc["CAN_ENABLE"].as<bool>();
    } else {
      enableCmd = ((doc["CAN_ENABLE"] | 0) == 1);
    }
    setCanCommEnable(enableCmd, "MQTT", true);
  }

  // MQTT 로그 enable/disable 설정
  // Example: {"MQTT_LOG_ENABLE":1} or {"MQTT_LOG_ENABLE":true}
  if (doc.containsKey("MQTT_LOG_ENABLE")) {
    bool enableCmd = false;
    if (doc["MQTT_LOG_ENABLE"].is<bool>()) {
      enableCmd = doc["MQTT_LOG_ENABLE"].as<bool>();
    } else {
      enableCmd = ((doc["MQTT_LOG_ENABLE"] | 0) == 1);
    }
    setMqttLogEnable(enableCmd, "MQTT", true);
  }
    // FET
  if (doc.containsKey("FET")) {
    JsonArray arr = doc["FET"];
    for (int i = 0; i < 8 && i < arr.size(); i++) {
      int val = arr[i];
      if (val != ioBoard.fet[i]) {
        sendIOBoardCommand("FET", i, val);    //         // cmd send from  main 1sec loop
        ioBoard.fet[i] = val;
      }
    }
  }

  // DAC7678 Target 값 설정 (0-500, 0.00V-5.00V * 100)
  if (doc.containsKey("DAC7678")) {
    JsonArray arr = doc["DAC7678"];
    for (int i = 0; i < 8 && i < arr.size(); i++) {
      int val = arr[i];
      // 값 범위 제한 (0 ~ 500, 즉 0.00V ~ 5.00V)
      if (val < 0) val = 0;
      if (val > 500) val = 500;
      
      if (val != ioBoard.dac7678_target[i]) {
        ioBoard.dac7678_target[i] = val;
        Serial.print("DAC7678 Target CH");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(val / 100.0, 2);
        Serial.println("V");
      }
    }
  }
  // GPO
  if (doc.containsKey("GPO")) {
    JsonArray arr = doc["GPO"];
    for (int i = 0; i < 2 && i < arr.size(); i++) {
      int val = arr[i];
      if (val != ioBoard.gpo[i]) {
        sendIOBoardCommand("GPO", i, val);         // cmd send from  main 1sec loop
        ioBoard.gpo[i] = val;
      }
    }
  }
  
  // PWM (0-255 범위)
  if (doc.containsKey("PWM")) {
    JsonArray arr = doc["PWM"];
    for (int i = 0; i < 2 && i < arr.size(); i++) {
      int val = arr[i];
      // 값 범위 제한 (0-255)
      if (val < 0) val = 0;
      if (val > 255) val = 255;
      
      if (val != ioBoard.pwm[i]) {
        setPWMValue(i, val);  // PWM 설정 함수 호출
        Serial.printf("PWM CH%d set to %d via MQTT\n", i, val);
      }
    }
  }
  
  // LED (0-1 범위)
  // if (doc.containsKey("LED")) {
  //   JsonArray arr = doc["LED"];
  //   for (int i = 0; i < 4 && i < arr.size(); i++) {
  //     int val = arr[i];
  //     // 값 범위 제한 (0-1)
  //     if (val < 0) val = 0;
  //     if (val > 1) val = 1;
      
  //     if (val != ioBoard.led[i]) {
        
  //       ioBoard.led[i] = val;
  //       // Serial.printf("LED CH%d set to %d via MQTT\n", i+1, val);  // CH1,2,3,4로 표시
  //     }
  //   }
  // }

  // Operation Mode 변경 (0~3)
  if (doc.containsKey("OP_MODE")) {
    int mode = doc["OP_MODE"];
    if (mode >= 0 && mode < 4) {
      configManager.setOperationMode((uint8_t)mode);
      applyOperationModeTemperatureLimits();
      Serial.printf("[CFG] OP_MODE updated: %d\n", mode);
    } else {
      Serial.printf("[CFG] Invalid OP_MODE: %d (valid: 0~3)\n", mode);
    }
  }

  // Mode별 Temperature Limit 설정
  // Example: {"TEMP_LIMIT_MODE":{"mode":1,"low":55.0,"high":120.0}}
  if (doc.containsKey("TEMP_LIMIT_MODE")) {
    JsonObject limitObj = doc["TEMP_LIMIT_MODE"].as<JsonObject>();
    if (!limitObj.isNull()) {
      int mode = limitObj["mode"] | -1;
      float low = limitObj["low"] | NAN;
      float high = limitObj["high"] | NAN;

      if (mode >= 0 && mode < 4 && !isnan(low) && !isnan(high) && low < high) {
        configManager.setTempLimits((uint8_t)mode, low, high);
        if ((uint8_t)mode == configManager.getOperationMode()) {
          applyOperationModeTemperatureLimits();
        }
        Serial.printf("[CFG] TEMP_LIMIT_MODE updated: mode=%d low=%.2f high=%.2f\n", mode, low, high);
      } else {
        Serial.println("[CFG] Invalid TEMP_LIMIT_MODE payload");
      }
    }
  }

  // Pellet feed duration 설정(초)
  // Example: {"PELLET_FEED_SEC":7}
  if (doc.containsKey("PELLET_FEED_SEC")) {
    int sec = doc["PELLET_FEED_SEC"] | -1;
    if (sec > 0 && sec <= 600) {
      configManager.setPelletFeedDurationSec((uint16_t)sec);
      Serial.printf("[CFG] PELLET_FEED_SEC updated: %d\n", sec);
    } else {
      Serial.println("[CFG] Invalid PELLET_FEED_SEC (valid: 1~600)");
    }
  }
}




// EXioBd_:status:"[0,0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0,28.29 28.87 23.70 24.04 28.20 25.53 25.53 27.90 29.52 24.13 20.73 28.66 20.20 22.72 23.57 20.58 24.59 25.01 26.07 21.70,0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0,0 0,00,0]

// 센서 데이터를 CAN 패킷으로 변환
void CANPacketManager::createSensorPacket(CANSensorPacket_t* packet) {
    // 헤더 설정
    packet->board_id = rSys.hw_bd_id_num;
    packet->packet_type = 0;  // 센서 데이터
    packet->sequence = sequenceCounter++;
    packet->data_length = sizeof(CANSensorPacket_t) - 4; // 헤더 제외
    
    // FET 상태 (8비트로 압축)
    packet->fet_status = 0;
    for (int i = 0; i < 8; i++) {
        if (ioBoard.fet[i]) {
            packet->fet_status |= (1 << i);
        }
    }
    
    // 내부 ADC (0-3.3V → 0-4095)
    for (int i = 0; i < 8; i++) {
        if (i < 6) {  // 유효한 ADC 채널은 6개
            packet->internal_adc[i] = (uint16_t)(ioBoard.inter_adc[i] * 4095.0 / 3.3);
        } else {
            packet->internal_adc[i] = 0;  // 무효한 채널은 0
        }
    }
    
    // 외부 ADC (0-5.0V → 0-4095)
    for (int i = 0; i < 8; i++) {
        packet->external_adc[i] = (uint16_t)(ioBoard.ads1015_adc[i] * 4095.0 / 5.0);
    }
    
    // 내부 DAC
    for (int i = 0; i < 2; i++) {
        packet->internal_dac[i] = (uint16_t)ioBoard.dac[i];
    }
    
    // 외부 DAC
    for (int i = 0; i < 8; i++) {
        packet->external_dac[i] = (uint16_t)ioBoard.dac7678_target[i];
    }
    
    // GPIO 입력 (2비트로 압축)
    packet->gpio_input = 0;
    for (int i = 0; i < 2; i++) {
        if (ioBoard.gpi[i]) {
            packet->gpio_input |= (1 << i);
        }
    }
    
    // GPIO 출력 (2비트로 압축)
    packet->gpio_output = 0;
    for (int i = 0; i < 2; i++) {
        if (ioBoard.gpo[i]) {
            packet->gpio_output |= (1 << i);
        }
    }
    
    // 주파수 (Hz 단위, 최대 65535Hz)
    for (int i = 0; i < 2; i++) {
        packet->input_freq[i] = (uint16_t)constrain(ioBoard.freq[i], 0, 65535);
    }
    
    // PWM (0-255)
    for (int i = 0; i < 2; i++) {
        packet->output_pwm[i] = (uint8_t)ioBoard.pwm[i];
    }
    
    // 온도 (0.1도 단위, -1200~+1200도 범위)
    for (int i = 0; i < Batagota::TC_CHANNEL_COUNT; i++) {
        float tempValue = ioBoard.TC_average[i] * 10.0;  // 0.1도 단위로 변환
        // 범위 제한: -1200°C ~ +1200°C (-12000 ~ +12000)
        tempValue = constrain(tempValue, -12000.0, 12000.0);
        packet->thermocouple[i] = (int16_t)tempValue;
    }
    
    // 빌드 타임스탬프 추가
    packet->build_timestamp = getBuildTimestamp32();
    
    // 디버깅: 빌드 타임스탬프 확인
    // Serial.printf("[DEBUG] Build timestamp: 0x%08X -> %s\n", 
    //               packet->build_timestamp, 
    //               decodeBuildTimestamp(packet->build_timestamp).c_str());
    
    // 체크섬 계산
    packet->checksum = calculateChecksum(packet);
}

// CAN 패킷에서 센서 데이터 추출
bool CANPacketManager::parseSensorPacket(const CANSensorPacket_t* packet) {
    // 체크섬 검증
    uint8_t calculatedChecksum = calculateChecksum(packet);
    if (calculatedChecksum != packet->checksum) {
        Serial.printf("CAN: Checksum error (calc=0x%02X, recv=0x%02X)\n", 
                     calculatedChecksum, packet->checksum);
        return false;
    }
    
    // 패킷 타입 확인
    if (packet->packet_type != 0) {
        Serial.printf("CAN: Invalid packet type: %d\n", packet->packet_type);
        return false;
    }
    
    Serial.printf("CAN: Received sensor data from Board ID %d (seq=%d)\n", 
                 packet->board_id, packet->sequence);
    
    // 데이터 출력 (선택적)
    Serial.printf("  FET: 0x%02X, GPIO_IN: 0x%02X, GPIO_OUT: 0x%02X\n",
                 packet->fet_status, packet->gpio_input, packet->gpio_output);
    
    Serial.printf("  TC: [%.1f, %.1f, %.1f, %.1f]°C\n",
                 packet->thermocouple[Batagota::TC_NEAR_IGNITOR]/10.0, packet->thermocouple[Batagota::TC_FAR_IGNITOR]/10.0,
                 packet->thermocouple[Batagota::TC_FRONT_OVEN]/10.0, packet->thermocouple[Batagota::TC_BACK_OVEN]/10.0);
    
    return true;
}

// 순차 CAN 메시지 전송 시작
bool CANPacketManager::startSequentialTransmission(uint32_t canId, const CANSensorPacket_t* packet) {
  if (!g_canCommEnabled) {
    return false;
  }

    if (canTxState.isTransmitting) {
        Serial.println("CAN: Already transmitting, cannot start new transmission");
        return false;
    }
    
    // **CAN ID 범위 검증: 올바른 범위인지 확인**
    bool validIdRange = false;
    uint32_t maxAllowedId = 0;
    
    if (canId == 0x100) {
        // MASTER: 0x100~0x108
        maxAllowedId = 0x108;
        validIdRange = true;
    } else if (canId >= 0x200 && canId <= 0x1F00 && (canId % 0x100) == 0) {
        // SLAVE: 0x200~0x208, 0x300~0x308, etc.
        maxAllowedId = canId + 8;
        validIdRange = true;
    }
    
    if (!validIdRange) {
        Serial.printf("CAN: Invalid base ID 0x%03X (expected: 0x100, 0x200, 0x300, etc.)\n", canId);
        return false;
    }
    
    // 전송 상태 초기화
    canTxState.isTransmitting = true;
    canTxState.baseCanId = canId;
    canTxState.currentMessageIndex = 0;
    canTxState.lastTransmitTime = millis();
    
    // 패킷 데이터 복사
    memcpy(&canTxState.packetData, packet, sizeof(CANSensorPacket_t));
    
    return true;
}

// 순차 전송 처리 (10ms 간격)
void CANPacketManager::handleSequentialTransmission() {
  if (!g_canCommEnabled) {
    return;
  }

    if (!canTxState.isTransmitting) {
        return;
    }
    
    // 10ms 간격 체크
    uint32_t currentTime = millis();
    if (currentTime - canTxState.lastTransmitTime < 10) {
        return;
    }
    
    // 전송할 데이터 준비
    uint8_t* packetData = (uint8_t*)&canTxState.packetData;
    
    // 전송 완료 체크
    if (canTxState.currentMessageIndex >= CAN_MESSAGE_COUNT) {
        // 전송 완료
        canTxState.isTransmitting = false;
        canTransmissionCompleted = true;  // 전송 완료 플래그 설정
        return;
    }
    
    // 현재 메시지 데이터 추출
    int dataOffset = canTxState.currentMessageIndex * CAN_MESSAGE_SIZE;
    int remainingBytes = CAN_PACKET_SIZE - dataOffset;
    int messageLength = (remainingBytes >= CAN_MESSAGE_SIZE) ? CAN_MESSAGE_SIZE : remainingBytes;
    
    // 마지막 메시지 길이 검증
    if (canTxState.currentMessageIndex == (CAN_MESSAGE_COUNT - 1)) { // 마지막 메시지
        messageLength = CAN_PACKET_SIZE - (dataOffset);
        if (messageLength <= 0 || messageLength > CAN_MESSAGE_SIZE) {
            messageLength = CAN_MESSAGE_SIZE;
        }
    }
    
    // CAN 메시지 ID 계산 (Base ID + Message Index)
    uint32_t messageId = canTxState.baseCanId + canTxState.currentMessageIndex;
    
    // ID 범위 검증
    uint32_t expectedMinId = canTxState.baseCanId;
    uint32_t expectedMaxId = canTxState.baseCanId + (CAN_MESSAGE_COUNT - 1);
    
    if (messageId < expectedMinId || messageId > expectedMaxId) {
        canTxState.isTransmitting = false;
        return;
    }
    
    // CAN 메시지 전송
    uint8_t messageData[CAN_MESSAGE_SIZE] = {0};
    memcpy(messageData, &packetData[dataOffset], messageLength);
    
    if (canService.sendMessage(messageId, messageData, messageLength)) {
        canTxState.currentMessageIndex++;
        canTxState.lastTransmitTime = currentTime;
      canMasterTxFailCount = 0;
    } else {
      canMasterTxFailCount++;
      Serial.printf("CAN TX fail: %d/%d (ID=0x%03X)\n",
              canMasterTxFailCount,
              MAX_CAN_TX_FAIL_COUNT,
              messageId);

      if (canMasterTxFailCount >= MAX_CAN_TX_FAIL_COUNT) {
        setCanCommEnable(false, "TX_FAIL_10", true);
        Serial.println("[CAN CFG] Auto-disabled due to 10 consecutive TX failures");
      }
    }
}

// 다중 CAN 메시지로 분할 전송 (64바이트 초과 시)
void CANPacketManager::sendPacketAsMultipleCANMessages(uint32_t canId, const CANSensorPacket_t* packet) {
    // 새로운 순차 전송 방식 사용 (10ms 간격)
    if (!startSequentialTransmission(canId, packet)) {
        Serial.println("CAN: Failed to start sequential transmission");
    }
}

// CAN MASTER 모드 통신 처리 함수 (새로운 시퀀스)
void handleCANMasterCommunication() {
  if (!g_canCommEnabled) {
    return;
  }

    unsigned long currentTime = millis();
    
    // CAN 전송이 중지된 상태인지 확인
    if (canMasterSuspended) {
        // CAN 수신 처리 (중지 중에도 수신은 확인)
        bool receivedAnyResponse = handleCANMasterReceive();
        
        // SLAVE 응답이 감지되면 즉시 중지 해제
        bool hasSlaveResponse = false;
        for (int i = 0; i < 16; i++) {
            if (canSlaveResponseReceived[i]) {
                hasSlaveResponse = true;
                break;
            }
        }
        
        if (hasSlaveResponse) {
            // Serial.println("CAN MASTER: SLAVE response detected - resuming immediately");
            canMasterSuspended = false;
            canMasterFailCount = 0;
        } else if (currentTime - canMasterSuspendTime >= CAN_RETRY_INTERVAL) {
            // 30초 후 재시도 (응답이 없는 경우)
            // Serial.println("CAN MASTER: Retrying after 30s suspension...");
            canMasterSuspended = false;
            canMasterFailCount = 0;
            // Slave 응답 상태 초기화
            memset(canSlaveResponseReceived, false, sizeof(canSlaveResponseReceived));
        } else {
            return; // 아직 재시도 시간이 안됨
        }
    }
    
    // CAN 수신 처리 (항상 실행)
    bool receivedAnyResponse = handleCANMasterReceive();
    
    // 순차 전송 중인지 확인
    if (canPacketManager.isTransmissionActive()) {
        canMasterTransmitting = true;
        return; // 순차 전송 중에는 새로운 전송 시작하지 않음
    }
    
    // 순차 전송이 완료된 경우 상태 업데이트
    if (canMasterTransmitting && !canPacketManager.isTransmissionActive()) {
        canMasterTransmitting = false;
        // Serial.printf("CAN MASTER: Transmission completed at %lu ms\n", currentTime);
        
        // 응답 체크 - 실제 SLAVE 응답이 있었는지 확인
        bool hasSlaveResponse = false;
        int responseCount = 0;
        for (int i = 0; i < 16; i++) {
            if (canSlaveResponseReceived[i]) {
                hasSlaveResponse = true;
                responseCount++;
                // Serial.printf("CAN MASTER: Response from SLAVE %d detected\n", i + 1);
            }
        }
        
        // Serial.printf("CAN MASTER: Total responses received: %d/16 slaves\n", responseCount);
        
        if (hasSlaveResponse) {
            canMasterFailCount = 0; // 응답이 있으면 실패 카운터 리셋
            // SLAVE 응답이 하나라도 있으면 전송 중지 해제
            if (canMasterSuspended) {
                canMasterSuspended = false;
                // Serial.println("CAN MASTER: Resuming transmission - SLAVE response detected");
            }
            // Serial.println("CAN MASTER: Cycle completed with SLAVE responses");
        } else {
            canMasterFailCount++;
            // Serial.printf("CAN MASTER: No response received (fail count: %d/%d)\n", 
            //              canMasterFailCount, MAX_CAN_FAIL_COUNT);
            
            // 최대 실패 횟수 도달 시 전송 중지 (모든 SLAVE 응답이 없을 때만)
            if (canMasterFailCount >= MAX_CAN_FAIL_COUNT) {
                canMasterSuspended = true;
                canMasterSuspendTime = currentTime;
                // Serial.println("CAN MASTER: Suspended transmission - no SLAVE responses");
                return;
            }
        }
    }
    
    // **단순화된 전송 조건: 1초마다 전송**
    if (!canMasterTransmitting && 
        (currentTime - lastCANMasterSend >= canMasterInterval)) { // 1초마다
        
        // 새로운 전송 사이클 시작
        memset(canSlaveResponseReceived, false, sizeof(canSlaveResponseReceived));
        
        // Serial.printf("\n=== CAN MASTER: Starting cycle (interval: %lums) ===\n", 
        //              currentTime - lastCANMasterSend);
        sendCANMasterData();
        canMasterTransmitting = true;
        lastCANMasterSend = currentTime;
    }
    
    // 디버그: 다음 전송까지 남은 시간 출력 (5초마다)
    static unsigned long lastDebugTime = 0;
    if (currentTime - lastDebugTime >= 5000) {
        if (!canMasterTransmitting) {
            unsigned long timeToNext = canMasterInterval - (currentTime - lastCANMasterSend);
            // Serial.printf("CAN MASTER: Next TX in %lu ms (suspended: %s)\n", 
            //              timeToNext, canMasterSuspended ? "YES" : "NO");
        }
        lastDebugTime = currentTime;
    }
}

// MASTER CAN 데이터 전송 함수  
void sendCANMasterData() {
    // MASTER ID: 0x100 번지 대역으로 전송
    uint32_t masterId = 0x100;
    
    // 센서 데이터 패킷 생성
    CANSensorPacket_t sensorPacket;
    canPacketManager.createSensorPacket(&sensorPacket);
    
    // 다중 CAN 메시지로 전송
    canPacketManager.sendPacketAsMultipleCANMessages(masterId, &sensorPacket);
    
    // Serial.printf("CAN MASTER: Data transmitted (ID: 0x%03X)\n", masterId);
}

// CAN MASTER 수신 함수 (SLAVE 데이터 수신)
bool handleCANMasterReceive() {
    bool receivedAnyResponse = false;
    
    if (canService.hasMessage()) {
        // Serial.print("R");  // MASTER에서 수신 표시
        
        uint32_t receivedId = canService.getLastMessageId();
        uint8_t* data = canService.getLastMessageData();
        int dataLength = canService.getLastMessageLength();
        
        // Serial.printf("\nCAN MASTER RX: ID=0x%03X, Len=%d\n", receivedId, dataLength);
        
        // SLAVE 데이터 ID 범위: 각 SLAVE는 9개 메시지만 전송 (0x200~0x208, 0x300~0x308, ...)
        if (receivedId >= 0x200 && receivedId <= 0x1F08) {  // SLAVE 16까지: 0x200 + 15*0x100 + 8 = 0x1F08
            // Board ID 계산: (ID - 0x200) / 0x100 + 1
            int boardId = (receivedId - 0x200) / 0x100 + 1;  // Board 1~16
            int messageIndex = (receivedId - 0x200) % 0x100;  // 0~(CAN_MESSAGE_COUNT-1)
            
            // 각 SLAVE는 0~(CAN_MESSAGE_COUNT-1) 범위의 메시지만 유효
            if (messageIndex <= (CAN_MESSAGE_COUNT - 1)) {
                if (boardId >= 1 && boardId <= 16) {
                    int slaveIndex = boardId - 1; // 0-15 범위로 변환
                    
                    lastCANReceived = millis(); // 수신 시간 업데이트
                    receivedAnyResponse = true;
                    
                    // 해당 Slave 보드의 응답 상태 업데이트
                    canSlaveResponseReceived[slaveIndex] = true;
                    
                    // Serial.printf("CAN MASTER: Valid SLAVE %d message (ID: 0x%03X, msg: %d)\n", 
                    //              boardId, receivedId, messageIndex);
                    
                    // 첫 번째 메시지인 경우 새로운 패킷 시작
                    if (messageIndex == 0) {
                        if (dataLength >= 4) {
                            // 완성된 패킷은 보존하고, 미완성 패킷만 리셋
                            if (!slaveDataValid[slaveIndex]) {
                                resetSlavePacket(slaveIndex);
                            } else {
                                // 완성된 패킷 위에 새 데이터 덮어쓰기 준비
                                packetMessageCount[slaveIndex] = 0;
                                for (int i = 0; i < CAN_MESSAGE_COUNT; i++) {
                                    packetMessageReceived[slaveIndex][i] = false;
                                }
                            }
                            
                            // 새 패킷 시작
                            packetStartTime[slaveIndex] = millis();
                            packetMessageReceived[slaveIndex][0] = true;
                            packetMessageCount[slaveIndex] = 1;
                            
                            // 헤더 정보 저장
                            memcpy(&slaveData[slaveIndex], data, min(dataLength, (int)sizeof(CANSensorPacket_t)));
                            
                            // Serial.printf("  SLAVE %d: Starting new packet (msg 1/%d) - Previous valid data preserved\n", boardId, CAN_MESSAGE_COUNT);
                        }
                    } else {
                        // 추가 메시지들을 패킷에 재조립
                        if (packetStartTime[slaveIndex] > 0 && !packetMessageReceived[slaveIndex][messageIndex]) {
                            packetMessageReceived[slaveIndex][messageIndex] = true;
                            packetMessageCount[slaveIndex]++;
                            
                            int dataOffset = messageIndex * 8;
                            uint8_t* packetPtr = (uint8_t*)&slaveData[slaveIndex];
                            
                            // 패킷 크기 범위 내에서만 복사
                            if (dataOffset < sizeof(CANSensorPacket_t)) {
                                int copyLength = min(dataLength, (int)(sizeof(CANSensorPacket_t) - dataOffset));
                                memcpy(&packetPtr[dataOffset], data, copyLength);
                                
                                // Serial.printf("  SLAVE %d: Message %d/%d received\n", boardId, packetMessageCount[slaveIndex], CAN_MESSAGE_COUNT);
                                
                                // 마지막 메시지인 경우 완성도 확인
                                if (messageIndex == (CAN_MESSAGE_COUNT - 1)) {
                                    if (isPacketComplete(slaveIndex)) {
                                        slaveDataValid[slaveIndex] = true;
                                        slaveDataUpdateTime[slaveIndex] = millis();
                                        // Serial.printf("  SLAVE %d: Packet COMPLETE (%d/%d messages) - MQTT ready\n", boardId, CAN_MESSAGE_COUNT, CAN_MESSAGE_COUNT);
                                    } else {
                                        // Serial.printf("  SLAVE %d: Packet INCOMPLETE (%d/%d messages) - resetting\n", 
                                        //              boardId, packetMessageCount[slaveIndex], CAN_MESSAGE_COUNT);
                                        resetSlavePacket(slaveIndex);
                                    }
                                }
                            }
                        } else if (packetMessageReceived[slaveIndex][messageIndex]) {
                            // 중복 메시지 수신
                            Serial.printf("  SLAVE %d: Duplicate message %d ignored\n", boardId, messageIndex);
                        }
                    }
                }
            } else {
                // Serial.printf("CAN MASTER: Invalid SLAVE message index (Board=%d, msg=%d > 8)\n", boardId, messageIndex);
            }
        } else {
            // Serial.printf("CAN MASTER: Non-SLAVE ID=0x%03X (expected: 0x200-0x1F08)\n", receivedId);
        }
    } else {
        // 주기적으로 수신 대기 상태 표시
        static unsigned long lastNoMsgTime = 0;
        unsigned long currentTime = millis();
        if (currentTime - lastNoMsgTime >= 2000) {  // 2초마다
            // Serial.print("~");  // MASTER 수신 대기 중
            lastNoMsgTime = currentTime;
        }
    }
    
    return receivedAnyResponse;
}

// CAN 메시지 기반 명령 처리 서비스 루틴
void processCANCommands() {
    if (rSys.hw_bd_id_num == 0) {
        // MASTER 모드: 수신된 SLAVE 데이터를 분석하여 명령 처리
        for (int i = 1; i <= 16; i++) {  // SLAVE 1~16 체크
            if (hasNewBoardData(i)) {
                CANSensorPacket_t* slaveData = readCANBoardData(i);
                if (slaveData != nullptr) {
                    // SLAVE 데이터 분석 및 명령 처리
                    // 예: 온도 임계값 체크, 알람 처리, DAC 출력 조정 등
                    
                    // MQTT로 데이터 전송 (기존 로직 활용)
                    // publishSensorDataToMQTT(i, slaveData);
                    
                    // 데이터 처리 완료 표시
                    markBoardDataProcessed(i);
                }
            }
        }
    } else {
        // SLAVE 모드: 수신된 MASTER 명령을 분석하여 실행
        // SLAVE 모드에서는 MASTER로부터 받은 명령을 처리
        // 예: GPIO 제어, DAC 설정, PWM 조정 등
    }
}

// 보드 데이터 처리 완료 표시 함수
void markBoardDataProcessed(uint8_t boardId) {
    if (boardId < MAX_CAN_BOARDS) {
        canRxData[boardId].hasNewData = false;
    }
}
