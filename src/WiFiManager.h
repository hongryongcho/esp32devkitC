#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WebServer.h>

class WiFiManager {
public:
  bool connectToWiFi();
  void startAPMode();
};

extern WiFiManager wifiManager;
extern WebServer server;

#endif
