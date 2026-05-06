#include "ConfigManager.h"
#include <EEPROM.h>
#include <math.h>
#include <string.h>

#define EEPROM_START 0
#define OP_MODE_COUNT 4
#define DEFAULT_TEMP_LIMIT_LOW 50.0f
#define DEFAULT_TEMP_LIMIT_HIGH 100.0f
#define DEFAULT_PELLET_FEED_DURATION_SEC 5
#define MAX_PELLET_FEED_DURATION_SEC 600
#define RECIPE_NAMESPACE "recipe_cfg"
#define RECIPE_KEY "profiles"

ConfigManager configManager;

void ConfigManager::load() {
  EEPROM.begin(sizeof(WiFiConfig));
  EEPROM.get(EEPROM_START, config);
  config.bootCount++;
  if (config.mqttPort <= 0 || config.mqttPort > 65535) {
    config.mqttPort = 1883;
  }
  if (config.version[0] == '\0' || !isprint(config.version[0])) {
    strcpy(config.version, "v1.0.0");
  }
  if (config.mqttClientId[0] == '\0' || !isprint(config.mqttClientId[0])) {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(config.mqttClientId, sizeof(config.mqttClientId), "ESP32-%02X%02X%02X", mac[3], mac[4], mac[5]);
  }
  if (config.mqttPubTopic[0] == '\0' || !isprint(config.mqttPubTopic[0]) || 
      strstr(config.mqttPubTopic, "BAGO") != NULL) {
    snprintf(config.mqttPubTopic, sizeof(config.mqttPubTopic), 
             "BAGO/%c%d/Status", config.mqttlogMode, config.mqttlogNumber);
  }
  // GPIO 모드 기본값 설정 (모든 핀을 기본 모드로)
  for (int i = 0; i < 2; i++) {
    if (config.gpi_mode[i] > GPI_FREQ) config.gpi_mode[i] = GPI_IO;  // 기본값: GPI IO 모드
    if (config.gpo_mode[i] > GPO_PWM) config.gpo_mode[i] = GPO_IO;  // 기본값: GPO IO 모드
  }

  // mode별 temperature limit 기본값 보정
  for (int mode = 0; mode < OP_MODE_COUNT; mode++) {
    bool lowInvalid = isnan(config.tempLimitLow[mode]) || !isfinite(config.tempLimitLow[mode]);
    bool highInvalid = isnan(config.tempLimitHigh[mode]) || !isfinite(config.tempLimitHigh[mode]);

    if (lowInvalid) {
      config.tempLimitLow[mode] = DEFAULT_TEMP_LIMIT_LOW;
    }
    if (highInvalid) {
      config.tempLimitHigh[mode] = DEFAULT_TEMP_LIMIT_HIGH;
    }

    // low >= high 또는 비정상 범위도 기본값으로 복구
    if (config.tempLimitLow[mode] >= config.tempLimitHigh[mode]) {
      config.tempLimitLow[mode] = DEFAULT_TEMP_LIMIT_LOW;
      config.tempLimitHigh[mode] = DEFAULT_TEMP_LIMIT_HIGH;
    }
  }

  // pellet feed duration 기본값 보정 (초)
  if (config.pelletFeedDurationSec == 0 ||
      config.pelletFeedDurationSec > MAX_PELLET_FEED_DURATION_SEC) {
    config.pelletFeedDurationSec = DEFAULT_PELLET_FEED_DURATION_SEC;
  }

  if (!loadRecipeProfiles()) {
    memset(&recipeStorage, 0, sizeof(recipeStorage));
    recipeStorage.version = RECIPE_PROFILE_VERSION;
    recipeStorage.recipeCount = DEFAULT_RECIPE_PROFILE_COUNT;
    for (uint8_t i = 0; i < recipeStorage.recipeCount; i++) {
      setDefaultRecipeProfile(i, recipeStorage.recipes[i]);
    }
    saveRecipeProfiles();
  }

  // Operation mode 기본값 보정 (recipe profile 범위 기준)
  if (config.operationMode >= getRecipeCount()) {
    config.operationMode = 0;
  }

  EEPROM.put(EEPROM_START, config);
  EEPROM.commit();
}

void ConfigManager::save() {
  EEPROM.put(EEPROM_START, config);
  EEPROM.commit();
}

bool ConfigManager::hasValidWiFiConfig() {
  return strlen(config.ssid) > 0;
}

WiFiConfig& ConfigManager::getConfig() {
  return config;
}

uint8_t ConfigManager::getOperationMode() const {
  if (config.operationMode >= getRecipeCount()) {
    return 0;
  }
  return config.operationMode;
}

void ConfigManager::setOperationMode(uint8_t mode) {
  if (mode >= getRecipeCount()) {
    return;
  }
  config.operationMode = mode;
  save();
}

float ConfigManager::getTempLimitLow(uint8_t mode) const {
  if (mode < recipeStorage.recipeCount) {
    return (float)recipeStorage.recipes[mode].cooking.oven_min;
  }
  if (mode >= OP_MODE_COUNT) {
    return DEFAULT_TEMP_LIMIT_LOW;
  }
  return config.tempLimitLow[mode];
}

float ConfigManager::getTempLimitHigh(uint8_t mode) const {
  if (mode < recipeStorage.recipeCount) {
    return (float)recipeStorage.recipes[mode].cooking.oven_max;
  }
  if (mode >= OP_MODE_COUNT) {
    return DEFAULT_TEMP_LIMIT_HIGH;
  }
  return config.tempLimitHigh[mode];
}

void ConfigManager::setTempLimits(uint8_t mode, float low, float high) {
  if (mode >= OP_MODE_COUNT) {
    return;
  }
  if (low >= high) {
    return;
  }
  config.tempLimitLow[mode] = low;
  config.tempLimitHigh[mode] = high;
  save();
}

uint16_t ConfigManager::getPelletFeedDurationSec() const {
  const uint8_t mode = getOperationMode();
  if (mode < recipeStorage.recipeCount) {
    const uint16_t sec = recipeStorage.recipes[mode].ignition.pump_on_sec;
    if (sec > 0 && sec <= MAX_PELLET_FEED_DURATION_SEC) {
      return sec;
    }
  }

  if (config.pelletFeedDurationSec == 0 || config.pelletFeedDurationSec > MAX_PELLET_FEED_DURATION_SEC) {
    return DEFAULT_PELLET_FEED_DURATION_SEC;
  }
  return config.pelletFeedDurationSec;
}

void ConfigManager::setPelletFeedDurationSec(uint16_t sec) {
  if (sec == 0) {
    sec = DEFAULT_PELLET_FEED_DURATION_SEC;
  }
  if (sec > MAX_PELLET_FEED_DURATION_SEC) {
    sec = MAX_PELLET_FEED_DURATION_SEC;
  }
  config.pelletFeedDurationSec = sec;
  save();
}

uint8_t ConfigManager::getRecipeCount() const {
  if (recipeStorage.recipeCount == 0 || recipeStorage.recipeCount > MAX_RECIPE_PROFILES) {
    return DEFAULT_RECIPE_PROFILE_COUNT;
  }
  return recipeStorage.recipeCount;
}

const RecipeStageProfile& ConfigManager::getRecipeProfile(uint8_t recipeIndex) const {
  static RecipeStageProfile fallback;
  static bool fallbackInited = false;

  if (!fallbackInited) {
    setDefaultRecipeProfile(0, fallback);
    fallbackInited = true;
  }

  if (recipeIndex >= getRecipeCount()) {
    return fallback;
  }
  return recipeStorage.recipes[recipeIndex];
}

const RecipeStageProfile& ConfigManager::getActiveRecipeProfile() const {
  const uint8_t mode = getOperationMode();
  if (mode < getRecipeCount()) {
    return recipeStorage.recipes[mode];
  }
  return getRecipeProfile(0);
}

void ConfigManager::setDefaultRecipeProfile(uint8_t recipeIndex, RecipeStageProfile& profile) const {
  (void)recipeIndex;
  memset(&profile, 0, sizeof(profile));

  profile.fuel_type = 0;

  profile.ignition.smoke_enable = 1;
  profile.ignition.ignite_t1 = 120;
  profile.ignition.ignite_t2 = 140;
  profile.ignition.pump_condition = 1;
  profile.ignition.pump_power = 60;
  profile.ignition.reignite_t1 = 90;
  profile.ignition.reignite_t2 = 110;
  profile.ignition.pump_on_sec = 6;
  profile.ignition.pump_off_sec = 12;

  profile.cooking.cook_minutes = 480;
  profile.cooking.oven_min = 105;
  profile.cooking.oven_max = 125;
  profile.cooking.oven_error_pct = 5;
  profile.cooking.heater_on_sec = 8;
  profile.cooking.heater_off_sec = 18;
  profile.cooking.fan_on_sec = 20;
  profile.cooking.fan_off_sec = 15;
  profile.cooking.spray_time_sec = 5;
  profile.cooking.spray_power = 40;

  profile.drying.dry_minutes = 120;
  profile.drying.oven_min = 65;
  profile.drying.oven_max = 80;
  profile.drying.oven_error_pct = 5;
  profile.drying.heater_on_sec = 6;
  profile.drying.heater_off_sec = 20;

  profile.holding.hold_minutes = 180;
  profile.holding.oven_min = 60;
  profile.holding.oven_max = 70;
  profile.holding.oven_error_pct = 4;
  profile.holding.heater_on_sec = 5;
  profile.holding.heater_off_sec = 24;
}

bool ConfigManager::validateRecipeProfile(const RecipeStageProfile& p) const {
  if (p.fuel_type > 3) return false;
  if (p.ignition.smoke_enable > 1) return false;
  if (p.ignition.pump_condition > 1) return false;
  if (p.ignition.pump_power > 100) return false;
  // if (p.ignition.ignite_t1 >= p.ignition.ignite_t2) return false;
  if (p.ignition.reignite_t1 >= p.ignition.reignite_t2) return false;
  if (p.cooking.oven_min >= p.cooking.oven_max) return false;
  if (p.drying.oven_min >= p.drying.oven_max) return false;
  if (p.holding.oven_min >= p.holding.oven_max) return false;
  if (p.cooking.oven_error_pct > 100 || p.drying.oven_error_pct > 100 || p.holding.oven_error_pct > 100) return false;
  if (p.cooking.spray_power > 100) return false;
  return true;
}

bool ConfigManager::loadRecipeProfiles() {
  if (!recipePrefs.begin(RECIPE_NAMESPACE, true)) {
    return false;
  }

  RecipeProfileStorage temp;
  const size_t got = recipePrefs.getBytes(RECIPE_KEY, &temp, sizeof(temp));
  recipePrefs.end();

  if (got != sizeof(temp)) {
    return false;
  }
  if (temp.version != RECIPE_PROFILE_VERSION) {
    return false;
  }
  if (temp.recipeCount == 0 || temp.recipeCount > MAX_RECIPE_PROFILES) {
    return false;
  }
  for (uint8_t i = 0; i < temp.recipeCount; i++) {
    if (!validateRecipeProfile(temp.recipes[i])) {
      return false;
    }
  }

  recipeStorage = temp;
  return true;
}

void ConfigManager::saveRecipeProfiles() {
  if (!recipePrefs.begin(RECIPE_NAMESPACE, false)) {
    return;
  }
  recipePrefs.putBytes(RECIPE_KEY, &recipeStorage, sizeof(recipeStorage));
  recipePrefs.end();
}

bool ConfigManager::setRecipeField(uint8_t recipeIndex, uint8_t fieldId, int32_t value) {
  if (recipeIndex >= getRecipeCount()) {
    return false;
  }

  RecipeStageProfile temp = recipeStorage.recipes[recipeIndex];

  switch (fieldId) {
    case RECIPE_FIELD_FUEL_TYPE: temp.fuel_type = (uint8_t)value; break;
    case RECIPE_FIELD_SMOKE_ENABLE: temp.ignition.smoke_enable = (uint8_t)value; break;
    case RECIPE_FIELD_IGNITE_T1: temp.ignition.ignite_t1 = (uint16_t)value; break;
    case RECIPE_FIELD_IGNITE_T2: temp.ignition.ignite_t2 = (uint16_t)value; break;
    case RECIPE_FIELD_PUMP_CONDITION: temp.ignition.pump_condition = (uint8_t)value; break;
    case RECIPE_FIELD_PUMP_POWER: temp.ignition.pump_power = (uint8_t)value; break;
    case RECIPE_FIELD_REIGNITE_T1: temp.ignition.reignite_t1 = (uint16_t)value; break;
    case RECIPE_FIELD_REIGNITE_T2: temp.ignition.reignite_t2 = (uint16_t)value; break;
    case RECIPE_FIELD_PUMP_ON_SEC: temp.ignition.pump_on_sec = (uint16_t)value; break;
    case RECIPE_FIELD_PUMP_OFF_SEC: temp.ignition.pump_off_sec = (uint16_t)value; break;
    case RECIPE_FIELD_COOK_MINUTES: temp.cooking.cook_minutes = (uint16_t)value; break;
    case RECIPE_FIELD_COOK_OVEN_MIN: temp.cooking.oven_min = (uint16_t)value; break;
    case RECIPE_FIELD_COOK_OVEN_MAX: temp.cooking.oven_max = (uint16_t)value; break;
    case RECIPE_FIELD_COOK_OVEN_ERROR_PCT: temp.cooking.oven_error_pct = (uint8_t)value; break;
    case RECIPE_FIELD_COOK_HEATER_ON_SEC: temp.cooking.heater_on_sec = (uint16_t)value; break;
    case RECIPE_FIELD_COOK_HEATER_OFF_SEC: temp.cooking.heater_off_sec = (uint16_t)value; break;
    case RECIPE_FIELD_COOK_FAN_ON_SEC: temp.cooking.fan_on_sec = (uint16_t)value; break;
    case RECIPE_FIELD_COOK_FAN_OFF_SEC: temp.cooking.fan_off_sec = (uint16_t)value; break;
    case RECIPE_FIELD_COOK_SPRAY_TIME_SEC: temp.cooking.spray_time_sec = (uint16_t)value; break;
    case RECIPE_FIELD_COOK_SPRAY_POWER: temp.cooking.spray_power = (uint8_t)value; break;
    case RECIPE_FIELD_DRY_MINUTES: temp.drying.dry_minutes = (uint16_t)value; break;
    case RECIPE_FIELD_DRY_OVEN_MIN: temp.drying.oven_min = (uint16_t)value; break;
    case RECIPE_FIELD_DRY_OVEN_MAX: temp.drying.oven_max = (uint16_t)value; break;
    case RECIPE_FIELD_DRY_OVEN_ERROR_PCT: temp.drying.oven_error_pct = (uint8_t)value; break;
    case RECIPE_FIELD_DRY_HEATER_ON_SEC: temp.drying.heater_on_sec = (uint16_t)value; break;
    case RECIPE_FIELD_DRY_HEATER_OFF_SEC: temp.drying.heater_off_sec = (uint16_t)value; break;
    case RECIPE_FIELD_HOLD_MINUTES: temp.holding.hold_minutes = (uint16_t)value; break;
    case RECIPE_FIELD_HOLD_OVEN_MIN: temp.holding.oven_min = (uint16_t)value; break;
    case RECIPE_FIELD_HOLD_OVEN_MAX: temp.holding.oven_max = (uint16_t)value; break;
    case RECIPE_FIELD_HOLD_OVEN_ERROR_PCT: temp.holding.oven_error_pct = (uint8_t)value; break;
    case RECIPE_FIELD_HOLD_HEATER_ON_SEC: temp.holding.heater_on_sec = (uint16_t)value; break;
    case RECIPE_FIELD_HOLD_HEATER_OFF_SEC: temp.holding.heater_off_sec = (uint16_t)value; break;
    default:
      return false;
  }

  if (!validateRecipeProfile(temp)) {
    return false;
  }

  recipeStorage.recipes[recipeIndex] = temp;
  saveRecipeProfiles();
  return true;
}
