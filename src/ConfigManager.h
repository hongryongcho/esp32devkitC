#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <WiFi.h>

// GPIO 모드 정의
#define GPI_IO    0    // GPI 입력 모드 (디지털 입력)
#define GPI_FREQ  1    // 주파수 측정 모드

#define GPO_IO    0    // GPO 출력 모드 (디지털 출력)
#define GPO_PWM   1    // PWM 출력 모드

struct WiFiConfig {
  char ssid[32];
  char password[32];
  char mqttServer[64];
  char mqttId[32];
  char mqttPass[32];
  char mqttClientId[32];
  char mqttSubTopic[64];
  char mqttCMDTopic[64];
  char mqttPubTopic[64];  // MQTT Publish Topic 추가
  char mqttlogMode;
  int mqttlogNumber;
  int mqttPort;
  uint32_t bootCount;
  char version[8];
  // GPIO 모드 설정 추가
  uint8_t gpi_mode[2];    // 0=GPI, 1=FREQ (2개 입력 핀)
  uint8_t gpo_mode[2];    // 0=GPO, 1=PWM (2개 출력 핀)
  uint8_t operationMode;  // 0~3
  float tempLimitLow[4];  // mode별 low limit
  float tempLimitHigh[4]; // mode별 high limit
  uint16_t pelletFeedDurationSec; // 펠릿 투입 시간(초)
};

static const uint8_t RECIPE_PROFILE_VERSION = 1;
static const uint8_t MAX_RECIPE_PROFILES = 8;
static const uint8_t DEFAULT_RECIPE_PROFILE_COUNT = 4;

struct SmokeIgnitionConfig {
  uint8_t smoke_enable;
  uint16_t ignite_t1;
  uint16_t ignite_t2;
  uint8_t pump_condition;
  uint8_t pump_power;
  uint16_t reignite_t1;
  uint16_t reignite_t2;
  uint16_t pump_on_sec;
  uint16_t pump_off_sec;
};

struct CookingConfig {
  uint16_t cook_minutes;
  uint16_t oven_min;
  uint16_t oven_max;
  uint8_t oven_error_pct;
  uint16_t heater_on_sec;
  uint16_t heater_off_sec;
  uint16_t fan_on_sec;
  uint16_t fan_off_sec;
  uint16_t spray_time_sec;
  uint8_t spray_power;
};

struct DryingConfig {
  uint16_t dry_minutes;
  uint16_t oven_min;
  uint16_t oven_max;
  uint8_t oven_error_pct;
  uint16_t heater_on_sec;
  uint16_t heater_off_sec;
};

struct HoldingConfig {
  uint16_t hold_minutes;
  uint16_t oven_min;
  uint16_t oven_max;
  uint8_t oven_error_pct;
  uint16_t heater_on_sec;
  uint16_t heater_off_sec;
};

struct RecipeStageProfile {
  uint8_t fuel_type;
  SmokeIgnitionConfig ignition;
  CookingConfig cooking;
  DryingConfig drying;
  HoldingConfig holding;
};

struct RecipeProfileStorage {
  uint8_t version;
  uint8_t recipeCount;
  RecipeStageProfile recipes[MAX_RECIPE_PROFILES];
};

enum RecipeFieldId {
  RECIPE_FIELD_FUEL_TYPE = 21,
  RECIPE_FIELD_SMOKE_ENABLE = 22,
  RECIPE_FIELD_IGNITE_T1 = 23,
  RECIPE_FIELD_IGNITE_T2 = 24,
  RECIPE_FIELD_PUMP_CONDITION = 25,
  RECIPE_FIELD_PUMP_POWER = 26,
  RECIPE_FIELD_REIGNITE_T1 = 27,
  RECIPE_FIELD_REIGNITE_T2 = 28,
  RECIPE_FIELD_PUMP_ON_SEC = 29,
  RECIPE_FIELD_PUMP_OFF_SEC = 30,
  RECIPE_FIELD_COOK_MINUTES = 31,
  RECIPE_FIELD_COOK_OVEN_MIN = 32,
  RECIPE_FIELD_COOK_OVEN_MAX = 33,
  RECIPE_FIELD_COOK_OVEN_ERROR_PCT = 34,
  RECIPE_FIELD_COOK_HEATER_ON_SEC = 35,
  RECIPE_FIELD_COOK_HEATER_OFF_SEC = 36,
  RECIPE_FIELD_COOK_FAN_ON_SEC = 37,
  RECIPE_FIELD_COOK_FAN_OFF_SEC = 38,
  RECIPE_FIELD_COOK_SPRAY_TIME_SEC = 39,
  RECIPE_FIELD_COOK_SPRAY_POWER = 40,
  RECIPE_FIELD_DRY_MINUTES = 41,
  RECIPE_FIELD_DRY_OVEN_MIN = 42,
  RECIPE_FIELD_DRY_OVEN_MAX = 43,
  RECIPE_FIELD_DRY_OVEN_ERROR_PCT = 44,
  RECIPE_FIELD_DRY_HEATER_ON_SEC = 45,
  RECIPE_FIELD_DRY_HEATER_OFF_SEC = 46,
  RECIPE_FIELD_HOLD_MINUTES = 47,
  RECIPE_FIELD_HOLD_OVEN_MIN = 48,
  RECIPE_FIELD_HOLD_OVEN_MAX = 49,
  RECIPE_FIELD_HOLD_OVEN_ERROR_PCT = 50,
  RECIPE_FIELD_HOLD_HEATER_ON_SEC = 51,
  RECIPE_FIELD_HOLD_HEATER_OFF_SEC = 52
};

class ConfigManager {
public:
  void load();
  void save();
  bool hasValidWiFiConfig();
  WiFiConfig& getConfig();
  uint8_t getOperationMode() const;
  void setOperationMode(uint8_t mode);
  float getTempLimitLow(uint8_t mode) const;
  float getTempLimitHigh(uint8_t mode) const;
  void setTempLimits(uint8_t mode, float low, float high);
  uint16_t getPelletFeedDurationSec() const;
  void setPelletFeedDurationSec(uint16_t sec);
  uint8_t getRecipeCount() const;
  const RecipeStageProfile& getRecipeProfile(uint8_t recipeIndex) const;
  const RecipeStageProfile& getActiveRecipeProfile() const;
  bool setRecipeField(uint8_t recipeIndex, uint8_t fieldId, int32_t value);
  void saveRecipeProfiles();

private:
  bool loadRecipeProfiles();
  bool validateRecipeProfile(const RecipeStageProfile& profile) const;
  void setDefaultRecipeProfile(uint8_t recipeIndex, RecipeStageProfile& profile) const;

  WiFiConfig config;
  RecipeProfileStorage recipeStorage;
  Preferences recipePrefs;
};

extern ConfigManager configManager;

#endif
