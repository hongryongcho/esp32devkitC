#include "WiFiManager.h"
#include "ConfigManager.h"
#include <WiFi.h>
#include <WebServer.h>

WiFiManager wifiManager;
WebServer server(80);

static const int WIFI_CONNECT_RETRY_INTERVAL_MS = 500;
static const int WIFI_CONNECT_MAX_ATTEMPTS = 40;  // 40 * 500ms = 20s

String getSignalStrength(int rssi) {
  if (rssi >= -50) return "Excellent";
  if (rssi >= -60) return "Good";
  if (rssi >= -70) return "Normal";
  return "Poor";
}

void handleRoot() {
  String html = "<html><head><meta charset='UTF-8'>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; background: #f9f9f9; color: #333; padding: 20px; }";
  html += "h1 { font-size: 28px; margin-bottom: 20px; }";
  html += "form { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); max-width: 500px; }";
  html += "label { display: block; font-weight: bold; margin-top: 15px; margin-bottom: 5px; font-size: 18px; }";
  html += "select, input[type='text'], input[type='password'], input[type='number'] { width: 100%; padding: 10px; font-size: 16px; border: 1px solid #ccc; border-radius: 4px; }";
  html += "select { cursor: pointer; }";
  html += "input[type='submit'] { margin-top: 25px; padding: 12px 20px; font-size: 18px; background-color: #007BFF; color: white; border: none; border-radius: 5px; cursor: pointer; }";
  html += "input[type='submit']:hover { background-color: #0056b3; }";
  html += ".section { border-bottom: 1px solid #ddd; padding-bottom: 15px; margin-bottom: 15px; }";
  html += "</style></head><body>";
  html += "<h1>WiFi & MQTT Setup</h1>";
  html += "<form method='POST' action='/save'>";
  html += "<div class='section'>";
  html += "<label for='ssidSelect'>Available SSIDs</label>";
  html += "<select id='ssidSelect' onchange='setSSID()'>";
  html += "<option value=''>-- Select SSID --</option>";
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; ++i) {
    String ssid = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);
    html += "<option value='" + ssid + "'>" + ssid + " (" + getSignalStrength(rssi) + ")</option>";
  }
  html += "</select>";
  html += "<input id='ssid' name='ssid' type='text' value='' hidden>";
  html += "<p>Selected SSID: <span id='selectedSSID'>None</span></p>";
  html += "<label for='port'>MQTT Port</label>";
  html += "<input id='port' name='port' type='number' value='" + String(configManager.getConfig().mqttPort) + "'>";
  html += "<label for='wpass'>WiFi Password</label>";
  html += "<input id='wpass' name='wpass' type='password' autocomplete='off' placeholder='Leave blank to keep current'>";
  html += "</div>";
  html += "<div class='section'>";
  html += "<label for='mqtt'>MQTT Server</label>";
  html += "<input id='mqtt' name='mqtt' type='text' value='" + String(configManager.getConfig().mqttServer) + "'>";
  html += "<label for='id'>MQTT ID</label>";
  html += "<input id='id' name='id' type='text' value='" + String(configManager.getConfig().mqttId) + "'>";
  html += "<label for='sub'>MQTT Subscribe Topic</label>";
  html += "<input id='sub' name='sub' type='text' value='" + String(configManager.getConfig().mqttSubTopic) + "' readonly>"; // Readonly 추가
  html += "<label for='mpass'>MQTT Password</label>";
  html += "<input id='mpass' name='mpass' type='password' autocomplete='off' placeholder='Leave blank to keep current'>";
  html += "</div>";
  html += "<div class='section'>";
  html += "<label for='mode'>MQTT Mode</label>";
  html += "<select id='mode' name='mode'>";
  html += "<option value='M'" + String(configManager.getConfig().mqttlogMode == 'M' ? " selected" : "") + ">M</option>";
  html += "<option value='P'" + String(configManager.getConfig().mqttlogMode == 'P' ? " selected" : "") + ">P</option>";
  html += "<option value='E'" + String(configManager.getConfig().mqttlogMode == 'E' ? " selected" : "") + ">E</option>";
  html += "</select>";
  html += "<label for='devno'>Device No (0~999)</label>";
  html += "<input id='devno' name='devno' type='number' min='0' max='999' value='" + String(configManager.getConfig().mqttlogNumber) + "'>";
  html += "</div>";
  html += "<label for='generatedTopic'>Generated Subscribe Topic</label>";
  html += "<input id='generatedTopic' type='text' readonly style='background:#eee;font-weight:bold;' value=''>";
  html += "<input type='submit' value='Save'>";
  html += "</form>";
  html += R"rawliteral(
    <script>
    function setSSID() {
      var sel = document.getElementById('ssidSelect');
      var input = document.getElementById('ssid');
      var display = document.getElementById('selectedSSID');
      input.value = sel.value;
      display.textContent = sel.value || 'None';
    }
    function updateGeneratedTopic() {
      var mode = document.getElementById('mode').value;
      var devno = document.getElementById('devno').value.padStart(3, '0');
      var topic = "BAGO_" + mode + devno + "/Log";
      document.getElementById('generatedTopic').value = topic;
      document.getElementById('sub').value = topic; // Update the actual MQTT subscribe topic field
    }
    document.addEventListener('DOMContentLoaded', function() {
      updateGeneratedTopic(); // 페이지 로딩 시 초기값 설정
      document.getElementById('mode').addEventListener('change', updateGeneratedTopic);
      document.getElementById('devno').addEventListener('input', updateGeneratedTopic);
      setSSID(); // 페이지 로딩 시 선택된 SSID 초기화
    });
    </script>
  )rawliteral";
  html += "</body></html>";
  server.send(200, "text/html", html);
}
void handleRescan() {
  WiFi.scanDelete(); // 이전 스캔 결과 삭제
  WiFi.scanNetworks(true); // 비동기 스캔 시작 (완료까지 시간이 걸림)
  delay(2000);  // 2초 대기 (스캔 완료 예상 시간)
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}


void handleSave() {
  WiFiConfig& cfg = configManager.getConfig();
  // SSID
  if (server.hasArg("ssid") && server.arg("ssid").length() > 0) {
    strncpy(cfg.ssid, server.arg("ssid").c_str(), sizeof(cfg.ssid));
  }
  // WiFi Password
  if (server.hasArg("wpass")) {
    String newPass = server.arg("wpass");
    if (newPass.length() > 0) {
      strncpy(cfg.password, newPass.c_str(), sizeof(cfg.password));
    }
  }
  // MQTT Server
  if (server.hasArg("mqtt") && server.arg("mqtt").length() > 0) {
    strncpy(cfg.mqttServer, server.arg("mqtt").c_str(), sizeof(cfg.mqttServer));
  }
  // MQTT ID (Username)
  if (server.hasArg("id") && server.arg("id").length() > 0) {
    strncpy(cfg.mqttId, server.arg("id").c_str(), sizeof(cfg.mqttId));
  }
  // MQTT Password
  if (server.hasArg("mpass")) {
    String newMqttPass = server.arg("mpass");
    if (newMqttPass.length() > 0) {
      strncpy(cfg.mqttPass, newMqttPass.c_str(), sizeof(cfg.mqttPass));
    }
  }
  // MQTT Mode
  if (server.hasArg("mode") && server.arg("mode").length() == 1) {
    cfg.mqttlogMode = server.arg("mode").charAt(0);
  }
  // MQTT Port
  if (server.hasArg("port") && server.arg("port").length() > 0) {
    cfg.mqttPort = server.arg("port").toInt();
  }
  // Device No
  if (server.hasArg("devno") && server.arg("devno").length() > 0) {
    int num = server.arg("devno").toInt();
    if (num >= 0 && num <= 999) {
      cfg.mqttlogNumber = num;
    }
  }
  // 자동 Topic 구성 (예: BAGO_M123/Log)
  char topicBuf[64];
  snprintf(topicBuf, sizeof(topicBuf), "BAGO/%c%d/Log", cfg.mqttlogMode, cfg.mqttlogNumber);
  strncpy(cfg.mqttSubTopic, topicBuf, sizeof(cfg.mqttSubTopic));
  snprintf(topicBuf, sizeof(topicBuf), "BAGO/%c%d/Cmd", cfg.mqttlogMode, cfg.mqttlogNumber);
  strncpy(cfg.mqttCMDTopic, topicBuf, sizeof(cfg.mqttCMDTopic));
  // MQTT Publish Topic을 BAGO_ 형태로 구성
  snprintf(topicBuf, sizeof(topicBuf), "BAGO/%c%d/Status", cfg.mqttlogMode, cfg.mqttlogNumber);
  strncpy(cfg.mqttPubTopic, topicBuf, sizeof(cfg.mqttPubTopic));
  configManager.save();
  server.send(200, "text/html", "<h1>Saved. Rebooting...</h1>");
  delay(1000);
  ESP.restart();
}

// ... WiFiManager 함수 구현은 기존 코드에서 복사하여 붙여넣으세요 ...

#define AP_SSID "ESP32-SETUP"
#define AP_PASS "12345678"

void WiFiManager::startAPMode() {
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.on("/", handleRoot);
  server.on("/save", handleSave);
  server.on("/rescan", handleRescan);
  server.begin();
  while (true) {
    server.handleClient();
    delay(10);
  }
}
bool WiFiManager::connectToWiFi() {
  String ssid_Msg = configManager.getConfig().ssid;
  String pass_Msg = configManager.getConfig().password;
  Serial.println("Attempting WiFi connection...");
  Serial.println("SSID :" + ssid_Msg);
  // Serial.println("PASSWD :" + pass_Msg); // 보안상 비밀번호는 출력하지 않는 것이 좋습니다.
  WiFi.begin(configManager.getConfig().ssid, configManager.getConfig().password);
  int connectAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && connectAttempts < WIFI_CONNECT_MAX_ATTEMPTS) {
    delay(WIFI_CONNECT_RETRY_INTERVAL_MS);
    Serial.print(".");
    connectAttempts++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("\n[NET-FAIL] WiFi connect timeout after %d ms\n",
                  WIFI_CONNECT_MAX_ATTEMPTS * WIFI_CONNECT_RETRY_INTERVAL_MS);
    Serial.println("\n[NET-FAIL] WiFi connection failed. Continue boot without reboot.");
    return false;
  }
  Serial.println("\nWiFi connected successfully.");
  Serial.print("Device IP Address: ");
  Serial.println(WiFi.localIP());
  return true;
}
