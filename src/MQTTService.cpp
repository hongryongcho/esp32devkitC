#include "IOBoard.h"
#include "MQTTService.h"
#include "ConfigManager.h"
#include "main.h"
#include "batagota.h"
#include <WiFi.h>
#include <PubSubClient.h>

// CAN 센서 패킷 구조체 정의 (main.h와 동일하게)
#pragma pack(push, 1)
typedef struct {
    uint8_t board_id;
    uint8_t packet_type;
    uint8_t sequence;
    uint8_t data_length;
    uint8_t fet_status;
    uint16_t internal_adc[8];
    uint16_t external_adc[8];
    uint16_t internal_dac[2];
    uint16_t external_dac[8];
    uint8_t gpio_input;
    uint8_t gpio_output;
    uint16_t input_freq[2];
    uint8_t output_pwm[2];
    int16_t thermocouple[4];
    uint32_t build_timestamp;
    uint8_t checksum;
} CANSensorPacket_t;
#pragma pack(pop)

WiFiClient espClient;
PubSubClient client(espClient);
MQTTService mqttService;

extern void handleLogTopic(const String& message);
extern void handleCmdTopic(const String& message);
extern IOBoard ioBoard;
extern IOBoard ioBoardBackup;
extern CANSensorPacket_t slaveData[16];
extern bool slaveDataValid[16];
extern unsigned long slaveDataUpdateTime[16];
extern bool canSlaveResponseReceived[16];  // CAN Slave 응답 상태
extern ConfigManager configManager;
extern String decodeBuildTimestamp(uint32_t timestamp);  // 빌드 타임스탬프 디코딩 함수
extern Batagota batagota;
extern int relayStatus[8];

namespace {
const uint8_t MQ2_ADC_CHANNEL = 1;
const uint8_t MQ7_ADC_CHANNEL = 2;
const uint8_t MQ135_ADC_CHANNEL = 3;

const float ADC_VOLTAGE_MIN = 0.0f;
const float ADC_VOLTAGE_MAX = 5.0f;

// Alarm-only rough estimate ranges (not calibrated ppm)
const float MQ2_PPM_EST_MIN = 300.0f;
const float MQ2_PPM_EST_MAX = 10000.0f;
const float MQ7_PPM_EST_MIN = 20.0f;
const float MQ7_PPM_EST_MAX = 2000.0f;
// MQ135 is a multi-gas air-quality sensor; use representative air-pollution proxy range.
const float MQ135_PPM_EST_MIN = 10.0f;
const float MQ135_PPM_EST_MAX = 300.0f;

float mapVoltageToPpmEstimate(float voltage, float ppmMin, float ppmMax) {
  float clamped = constrain(voltage, ADC_VOLTAGE_MIN, ADC_VOLTAGE_MAX);
  return ppmMin + ((clamped - ADC_VOLTAGE_MIN) * (ppmMax - ppmMin) / (ADC_VOLTAGE_MAX - ADC_VOLTAGE_MIN));
}
}

// MQTT 상태 코드 해석 함수
const char* getMQTTStateDescription(int state) {
  switch (state) {
    case -4: return "MQTT_CONNECTION_TIMEOUT";
    case -3: return "MQTT_CONNECTION_LOST";
    case -2: return "MQTT_CONNECT_FAILED";
    case -1: return "MQTT_DISCONNECTED";
    case 0: return "MQTT_CONNECTED";
    case 1: return "MQTT_CONNECT_BAD_PROTOCOL";
    case 2: return "MQTT_CONNECT_BAD_CLIENT_ID";
    case 3: return "MQTT_CONNECT_UNAVAILABLE";
    case 4: return "MQTT_CONNECT_BAD_CREDENTIALS";
    case 5: return "MQTT_CONNECT_UNAUTHORIZED";
    default: return "UNKNOWN_STATE";
  }
}

// 토픽 이름 검증 함수
bool isValidMQTTTopic(const char* topic) {
  if (topic == nullptr || strlen(topic) == 0) return false;
  if (strlen(topic) > 65535) return false; // MQTT spec limit
  
  // 금지된 문자 검사
  for (int i = 0; i < strlen(topic); i++) {
    char c = topic[i];
    if (c == '+' || c == '#') {
      // + 와 # 는 subscribe에서만 사용 가능, publish에서는 불가
      return false;
    }
    if (c < 32 || c > 126) {
      // 제어 문자나 확장 ASCII 문자 금지
      return false;
    }
  }
  return true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  if (topicStr == configManager.getConfig().mqttSubTopic) {
    // Log 토픽 메시지 처리
    handleLogTopic(message);
  } else if (topicStr == configManager.getConfig().mqttCMDTopic) {
    // Cmd 토픽 메시지 처리
    handleCmdTopic(message);
  } else {
    Serial.println("[MQTT] Unknown topic: " + topicStr);
  }

}



bool MQTTService::begin(const char* broker, int port, const char* clientId, const char* username, const char* password) {
  Serial.printf("[MQTT] Initializing connection to %s:%d\n", broker, port);
  Serial.printf("[MQTT] Client ID: %s, Username: %s\n", clientId, username);
  
  // PubSubClient 버퍼 크기 설정 (요청사항: 4KB)
  client.setBufferSize(4096);
  client.setServer(broker, port);
  client.setCallback(callback);
  reconnect(clientId, username, password);
  return connected;
}

bool MQTTService::connectAndSubscribe(const char* clientId, const char* username, const char* password) {
  Serial.print("Attempting MQTT connection...");

  if (!client.connect(clientId, username, password)) {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.printf(" (%s)\n", getMQTTStateDescription(client.state()));
    connected = false;
    return false;
  }

  Serial.println("connected");

  // ConfigManager에서 설정된 Subscribe 토픽들을 등록
  if (strlen(configManager.getConfig().mqttSubTopic) > 0) {
    client.subscribe(configManager.getConfig().mqttSubTopic);
    Serial.printf("[MQTT] Subscribed to Log topic: %s\n", configManager.getConfig().mqttSubTopic);
  }
  if (strlen(configManager.getConfig().mqttCMDTopic) > 0) {
    client.subscribe(configManager.getConfig().mqttCMDTopic);
    Serial.printf("[MQTT] Subscribed to Cmd topic: %s\n", configManager.getConfig().mqttCMDTopic);
  }

  connected = true;
  reconnectCooldownMode = false;
  nextReconnectAttemptMs = 0;
  return true;
}

void MQTTService::reconnect(const char* clientId, const char* username, const char* password) {
  const unsigned long now = millis();

  // Cooldown mode: allow only one reconnect attempt every 60 seconds.
  if (reconnectCooldownMode) {
    if ((long)(now - nextReconnectAttemptMs) < 0) {
      return;
    }

    Serial.println("[MQTT] Cooldown reconnect attempt (1/min)");
    if (!connectAndSubscribe(clientId, username, password)) {
      nextReconnectAttemptMs = now + RECONNECT_COOLDOWN_MS;
      Serial.println("[MQTT] Cooldown reconnect failed. Next attempt in 60 seconds.");
    }
    return;
  }

  // Normal mode: try up to MAX_RECONNECT_ATTEMPTS times.
  for (uint8_t attempt = 0; attempt < MAX_RECONNECT_ATTEMPTS; attempt++) {
    if (connectAndSubscribe(clientId, username, password)) {
      return;
    }

    if (attempt < (MAX_RECONNECT_ATTEMPTS - 1)) {
      Serial.println("[MQTT] Retry in 5 seconds...");
      delay(5000);
    }
  }

  reconnectCooldownMode = true;
  nextReconnectAttemptMs = now + RECONNECT_COOLDOWN_MS;
  Serial.println("[MQTT] Reconnect failed 5 times. Entering cooldown mode (1 attempt/min).");
}

void MQTTService::publish() {
  struct tm timeinfo;
  char timeStr[20];
  if (getLocalTime(&timeinfo)) {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  } else {
    strcpy(timeStr, "Time_Not_Synced");
  }
  String jsonBase = "{";
  jsonBase += "\"rtc_time\": \"" + String(timeStr) + "\", ";
  jsonBase += "\"BD_ID\": " + String(rSys.hw_bd_id_num) + ", ";  // Board ID 추가
  
  // MASTER 빌드 정보 추가
  jsonBase += "\"BUILD_INFO\": {";
  jsonBase += "\"version\": \"v2.0.1\", ";
  jsonBase += "\"build_timestamp\": \"" __DATE__ " " __TIME__ "\", ";
  jsonBase += "\"compiler\": \"ESP32-Arduino\"";
  jsonBase += "}, ";
  
  jsonBase += "\"FET\": [";
  for (int i = 0; i < 8; i++) {
    jsonBase += String(ioBoard.fet[i]);
    if (i < 7) jsonBase += ",";
  }
  jsonBase += "], ";

  // UART debug JSON align: IADC 대신 temp[6]를 사용한다.
  jsonBase += "\"temp\": [";
  for (int i = 0; i < 6; i++) {
    jsonBase += String(batagota.getTemperatureByIndex((uint8_t)i), 2);
    if (i < 5) jsonBase += ",";
  }
  jsonBase += "], ";
  jsonBase += "\"ExADC\": [";
  for (int i = 0; i < 8; i++) {
    jsonBase += String(ioBoard.ads1015_adc[i], 2);
    if (i < 7) jsonBase += ",";
  }
  jsonBase += "], ";

  // Alarm-only rough ppm estimate based on 0~5V scaling.
  // NOTE: This is NOT a calibrated concentration value.
  // Accurate ppm requires Rs/R0 calibration, gas-specific curve fitting, and environmental compensation.
  jsonBase += "\"AIR_CONDITION\": {";
  jsonBase += "\"UNIT\": \"ppm_est\", ";
  jsonBase += "\"MQ2\": " + String(mapVoltageToPpmEstimate(ioBoard.ads1015_adc[MQ2_ADC_CHANNEL], MQ2_PPM_EST_MIN, MQ2_PPM_EST_MAX), 1) + ", ";
  jsonBase += "\"MQ7\": " + String(mapVoltageToPpmEstimate(ioBoard.ads1015_adc[MQ7_ADC_CHANNEL], MQ7_PPM_EST_MIN, MQ7_PPM_EST_MAX), 1) + ", ";
  jsonBase += "\"MQ135\": " + String(mapVoltageToPpmEstimate(ioBoard.ads1015_adc[MQ135_ADC_CHANNEL], MQ135_PPM_EST_MIN, MQ135_PPM_EST_MAX), 1) + ", ";
  jsonBase += "\"RAW_UNIT\": \"V\", ";
  jsonBase += "\"MQ2_V\": " + String(ioBoard.ads1015_adc[MQ2_ADC_CHANNEL], 3) + ", ";
  jsonBase += "\"MQ7_V\": " + String(ioBoard.ads1015_adc[MQ7_ADC_CHANNEL], 3) + ", ";
  jsonBase += "\"MQ135_V\": " + String(ioBoard.ads1015_adc[MQ135_ADC_CHANNEL], 3);
  jsonBase += "}, ";

  jsonBase += "\"TC\": [";
  for (int i = 0; i < 4; i++) {
    jsonBase += String(ioBoard.TC_average[i], 2);
    if (i < 3) jsonBase += ",";
  }
  jsonBase += "], ";
  jsonBase += "\"DAC\": [";
  for (int i = 0; i < 8; i++) {
    jsonBase += String(ioBoard.dac7678_target[i]);
    if (i < 7) jsonBase += ",";
  }
  jsonBase += "], ";
  jsonBase += "\"IDAC\": [";
  for (int i = 0; i < 2; i++) {
    jsonBase += String(ioBoard.dac[i], 2);
    if (i < 1) jsonBase += ",";
  }
  jsonBase += "], ";
  jsonBase += "\"GPI\": [";
  for (int i = 0; i < 2; i++) {
    jsonBase += String(ioBoard.gpi[i]);
    if (i < 1) jsonBase += ",";
  }
  jsonBase += "], ";
  jsonBase += "\"GPO\": [";
  for (int i = 0; i < 2; i++) {
    jsonBase += String(ioBoard.gpo[i]);
    if (i < 1) jsonBase += ",";
  }
  jsonBase += "], ";
  jsonBase += "\"FREQ\": [";
  for (int i = 0; i < 2; i++) {
    jsonBase += String(ioBoard.freq[i], 2);
    if (i < 1) jsonBase += ",";
  }
  jsonBase += "], ";
  jsonBase += "\"PWM\": [";
  for (int i = 0; i < 2; i++) {
    jsonBase += String(ioBoard.pwm[i]);
    if (i < 1) jsonBase += ",";
  }
  jsonBase += "], ";
  jsonBase += "\"LED\": [";
  for (int i = 0; i < 4; i++) {
    jsonBase += String(ioBoard.led[i]);
    if (i < 3) jsonBase += ",";
  }
  jsonBase += "], ";

  // Batagota current state information
  jsonBase += "\"currentState\": " + String(batagota.getCurrentState()) + ", ";
  jsonBase += "\"stateTimeInSeconds\": " + String(batagota.getStateTimeInSeconds()) + ", ";
  jsonBase += "\"name\": \"" + String(batagota.getStateNameStr()) + "\", ";
  jsonBase += "\"ig_st\": " + String(batagota.getIgnitorStateCode()) + ", ";
  jsonBase += "\"ig_nm\": \"" + String(batagota.getIgnitorStateName()) + "\", ";
  jsonBase += "\"ig_on\": " + String(batagota.getIgnitorTurnOnCommand() ? 1 : 0) + ", ";
  jsonBase += "\"ig_off\": " + String(batagota.getIgnitorTurnOffCommand() ? 1 : 0) + ", ";
  jsonBase += "\"ig_t1\": " + String(configManager.getActiveRecipeProfile().ignition.ignite_t1) + ", ";
  jsonBase += "\"ig_t2\": " + String(configManager.getActiveRecipeProfile().ignition.ignite_t2) + ", ";

  const uint32_t remainSec = batagota.getRemainingSecondsForDisplay();
  if (batagota.shouldReportRemainingTime()) {
    time_t nowSec = time(NULL);
    time_t endSec = nowSec + (time_t)remainSec;
    struct tm* endTm = localtime(&endSec);
    char endTimeStr[24] = "";
    if (endTm != NULL) {
      snprintf(endTimeStr, sizeof(endTimeStr), "%02d-%02d %02d:%02d:%02d",
               endTm->tm_mon + 1, endTm->tm_mday,
               endTm->tm_hour, endTm->tm_min, endTm->tm_sec);
    } else {
      strcpy(endTimeStr, "Time_Not_Synced");
    }

    // UART debug JSON policy: remain is minutes.
    jsonBase += "\"remain\": " + String(remainSec / 60U) + ", ";
    jsonBase += "\"end\": \"" + String(endTimeStr) + "\", ";
  } else {
    jsonBase += "\"remain\": 0, ";
    jsonBase += "\"end\": \"\", ";
  }

  jsonBase += "\"relay\": [";
  for (int i = 0; i < 8; i++) {
    jsonBase += String(relayStatus[i] != 0 ? 1 : 0);
    if (i < 7) jsonBase += ",";
  }
  jsonBase += "], ";

  jsonBase += "\"ERR_HIST_STATE\": [";
  for (int i = 0; i < 4; i++) {
    uint16_t stateCode = 0;
    if (i < batagota.getErrorHistoryCount()) {
      stateCode = (uint16_t)(batagota.getErrorHistoryCode(i) & 0xFFFF);
    }
    jsonBase += String(stateCode);
    if (i < 3) jsonBase += ",";
  }
  jsonBase += "], ";

  jsonBase += "\"ERR_HIST_CHANNEL\": [";
  for (int i = 0; i < 4; i++) {
    uint16_t channelCode = 0xFFFF;
    if (i < batagota.getErrorHistoryCount()) {
      channelCode = (uint16_t)(batagota.getErrorHistoryCode(i) >> 16);
    }
    jsonBase += String(channelCode);
    if (i < 3) jsonBase += ",";
  }
  jsonBase += "]";

  // MASTER 모드일 때 사용할 SLAVE JSON 오브젝트 목록을 미리 구성한다.
  String slaveObjects[16];
  int slaveCount = 0;
  if (rSys.hw_bd_id_num == 0) {
    for (int i = 0; i < 16; i++) {
      if (!slaveDataValid[i]) {
        continue;
      }

      String slaveJson = "{";
      slaveJson += "\"CH\": " + String(i + 1) + ", ";
      slaveJson += "\"BD_ID\": " + String(slaveData[i].board_id) + ", ";

      slaveJson += "\"IADC\": [";
      for (int j = 0; j < 8; j++) {
        slaveJson += String(slaveData[i].internal_adc[j]);
        if (j < 7) slaveJson += ",";
      }
      slaveJson += "], ";

      slaveJson += "\"ExADC\": [";
      for (int j = 0; j < 8; j++) {
        slaveJson += String(slaveData[i].external_adc[j]);
        if (j < 7) slaveJson += ",";
      }
      slaveJson += "], ";

      slaveJson += "\"IDAC\": [";
      for (int j = 0; j < 2; j++) {
        slaveJson += String(slaveData[i].internal_dac[j]);
        if (j < 1) slaveJson += ",";
      }
      slaveJson += "], ";

      slaveJson += "\"ExDAC\": [";
      for (int j = 0; j < 8; j++) {
        slaveJson += String(slaveData[i].external_dac[j]);
        if (j < 7) slaveJson += ",";
      }
      slaveJson += "], ";

      slaveJson += "\"GPI\": " + String(slaveData[i].gpio_input) + ", ";
      slaveJson += "\"GPO\": " + String(slaveData[i].gpio_output) + ", ";

      slaveJson += "\"FREQ\": [";
      for (int j = 0; j < 2; j++) {
        slaveJson += String(slaveData[i].input_freq[j]);
        if (j < 1) slaveJson += ",";
      }
      slaveJson += "], ";

      slaveJson += "\"PWM\": [";
      for (int j = 0; j < 2; j++) {
        slaveJson += String(slaveData[i].output_pwm[j]);
        if (j < 1) slaveJson += ",";
      }
      slaveJson += "], ";

      slaveJson += "\"TC\": [";
      for (int j = 0; j < 4; j++) {
        slaveJson += String(slaveData[i].thermocouple[j] / 10.0, 1);
        if (j < 3) slaveJson += ",";
      }
      slaveJson += "], ";

      slaveJson += "\"FET\": [";
      for (int j = 0; j < 8; j++) {
        int fetBit = (slaveData[i].fet_status >> j) & 0x01;
        slaveJson += String(fetBit);
        if (j < 7) slaveJson += ",";
      }
      slaveJson += "], ";

      String decodedTimestamp = decodeBuildTimestamp(slaveData[i].build_timestamp);
      slaveJson += "\"BUILD_TIMESTAMP\": \"" + decodedTimestamp + "\", ";
      slaveJson += "\"UPDATE\": " + String(slaveDataUpdateTime[i]) + ", ";
      slaveJson += "\"RESPONSE\": true";
      slaveJson += "}";

      slaveObjects[slaveCount] = slaveJson;
      slaveCount++;
    }

    if (slaveCount > 0) {
      Serial.printf("[MQTT] Total valid SLAVES: %d\n", slaveCount);
    }
  }
  // LED[0]: CH1 (1초 자동 토글)
  // LED[1]: CH2 (100ms 자동 토글) 
  // LED[2]: CH3 (MQTT 제어)
  // LED[3]: CH4 (MQTT 제어)
  char publishTopic[128];
  // 항상 BAGO 형태의 Topic 사용
  snprintf(publishTopic, sizeof(publishTopic), "BAGO/%c%d/Status", 
           configManager.getConfig().mqttlogMode, 
           configManager.getConfig().mqttlogNumber);
  
  // 토픽 검증
  if (!isValidMQTTTopic(publishTopic)) {
    Serial.printf("[MQTT] Invalid topic format: %s\n", publishTopic);
    return;
  }
  
  // Serial.printf("[MQTT] Publishing to: %s\n", publishTopic);
  // Serial.printf("[MQTT] JSON: %s\n", json.c_str());
  
  // publish 시도 전에 연결 상태 재확인
  if (!client.connected()) {
    Serial.println("[MQTT] Client disconnected before publish!");
    return;
  }

  const int mqttBufferSize = (int)client.getBufferSize();
  const int topicLen = (int)strlen(publishTopic);
  const int mqttProtocolOverhead = topicLen + 16;
  int safePayloadLimit = mqttBufferSize - mqttProtocolOverhead;
  if (safePayloadLimit < 256) {
    safePayloadLimit = mqttBufferSize - 16;
  }
  if (safePayloadLimit < 128) {
    safePayloadLimit = mqttBufferSize;
  }

  auto publishPayload = [&](const String& payload, const char* tag) -> bool {
    bool ok = client.publish(publishTopic, payload.c_str());
    if (!ok) {
      Serial.print("Failed to publish MQTT message. State: ");
      Serial.print(client.state());
      Serial.printf(" (%s)\n", getMQTTStateDescription(client.state()));
      Serial.printf("Topic: %s\n", publishTopic);
      Serial.printf("Connected: %s\n", client.connected() ? "YES" : "NO");
      Serial.printf("Payload tag: %s\n", tag);
      Serial.printf("JSON length: %d bytes\n", payload.length());
      Serial.printf("Safe payload limit: %d bytes\n", safePayloadLimit);
      Serial.printf("Buffer size: %d bytes\n", mqttBufferSize);
      if (payload.length() > safePayloadLimit) {
        Serial.println("ERROR: MQTT payload is larger than safe limit!");
      }
    }
    return ok;
  };

  // MASTER가 아니거나 유효한 SLAVE가 없으면 단일 패킷으로 전송.
  if (rSys.hw_bd_id_num != 0 || slaveCount == 0) {
    String singleJson = jsonBase + "}";
    if (singleJson.length() > safePayloadLimit) {
      Serial.printf("[MQTT] WARN payload=%d exceeds safe limit=%d (no SLAVES)\n",
                    singleJson.length(), safePayloadLimit);
    }
    publishPayload(singleJson, "single-no-slaves");
    return;
  }

  // 먼저 기존과 동일하게 SLAVES를 모두 포함한 단일 패킷 가능 여부를 점검한다.
  String allSlavesJson;
  for (int i = 0; i < slaveCount; i++) {
    if (i > 0) allSlavesJson += ",";
    allSlavesJson += slaveObjects[i];
  }
  String fullJson = jsonBase + ", \"SLAVES\": [" + allSlavesJson + "]}";

  if (fullJson.length() <= safePayloadLimit) {
    publishPayload(fullJson, "single-all-slaves");
    return;
  }

  Serial.printf("[MQTT] WARN payload=%d exceeds safe limit=%d, splitting SLAVES\n",
                fullJson.length(), safePayloadLimit);

  String slaveBatches[16];
  int batchCount = 0;
  String currentBatch;

  for (int i = 0; i < slaveCount; i++) {
    String candidateBatch = currentBatch;
    if (candidateBatch.length() > 0) {
      candidateBatch += ",";
    }
    candidateBatch += slaveObjects[i];

    String probePayload = jsonBase +
                          ", \"SLAVES_CHUNK\": 1, \"SLAVES_CHUNK_TOTAL\": 1, \"SLAVES\": [" +
                          candidateBatch + "]}";

    if (probePayload.length() <= safePayloadLimit) {
      currentBatch = candidateBatch;
      continue;
    }

    if (currentBatch.length() > 0 && batchCount < 16) {
      slaveBatches[batchCount] = currentBatch;
      batchCount++;
      currentBatch = slaveObjects[i];

      String singleProbe = jsonBase +
                           ", \"SLAVES_CHUNK\": 1, \"SLAVES_CHUNK_TOTAL\": 1, \"SLAVES\": [" +
                           currentBatch + "]}";
      if (singleProbe.length() > safePayloadLimit) {
        Serial.printf("[MQTT] WARN single SLAVE payload too large. CH=%d dropped (len=%d, limit=%d)\n",
                      i + 1, singleProbe.length(), safePayloadLimit);
        currentBatch = "";
      }
    } else {
      String singleProbe = jsonBase +
                           ", \"SLAVES_CHUNK\": 1, \"SLAVES_CHUNK_TOTAL\": 1, \"SLAVES\": [" +
                           slaveObjects[i] + "]}";
      Serial.printf("[MQTT] WARN single SLAVE payload too large. CH=%d dropped (len=%d, limit=%d)\n",
                    i + 1, singleProbe.length(), safePayloadLimit);
    }
  }

  if (currentBatch.length() > 0 && batchCount < 16) {
    slaveBatches[batchCount] = currentBatch;
    batchCount++;
  }

  if (batchCount == 0) {
    Serial.println("[MQTT] WARN all SLAVES dropped during split, sending master-only payload");
    String fallbackJson = jsonBase + "}";
    publishPayload(fallbackJson, "split-fallback-no-slaves");
    return;
  }

  for (int chunkIdx = 0; chunkIdx < batchCount; chunkIdx++) {
    String chunkJson = jsonBase +
                       ", \"SLAVES_CHUNK\": " + String(chunkIdx + 1) +
                       ", \"SLAVES_CHUNK_TOTAL\": " + String(batchCount) +
                       ", \"SLAVES\": [" + slaveBatches[chunkIdx] + "]}";

    publishPayload(chunkJson, "split-chunk");
  }
}

void MQTTService::loop() {
  if (!client.connected()) {
    connected = false;
    reconnect(configManager.getConfig().mqttClientId, 
              configManager.getConfig().mqttId, 
              configManager.getConfig().mqttPass);
  }
  client.loop();
}

void MQTTService::checkSubscribeCommands() {
  // This is handled by client.loop() and the callback function.
}
