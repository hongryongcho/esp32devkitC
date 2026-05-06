#ifndef MQTT_SERVICE_H
#define MQTT_SERVICE_H

#include <Arduino.h>
#include <PubSubClient.h>

class MQTTService {
public:
  bool begin(const char* broker, int port, const char* clientId, const char* username, const char* password);
  void publish();
  void checkSubscribeCommands();
  void loop();
private:
  bool connected = false;
  bool reconnectCooldownMode = false;
  unsigned long nextReconnectAttemptMs = 0;
  static const uint8_t MAX_RECONNECT_ATTEMPTS = 5;
  static const unsigned long RECONNECT_COOLDOWN_MS = 60000UL;

  bool connectAndSubscribe(const char* clientId, const char* username, const char* password);
  void reconnect(const char* clientId, const char* username, const char* password);
};

extern MQTTService mqttService;
extern PubSubClient client;

#endif
