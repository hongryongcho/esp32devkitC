#ifndef MODE_PROFILE_H
#define MODE_PROFILE_H

#include <stdint.h>
#include <stdbool.h>
#include <Preferences.h>

#include "ErrorCode.h"

static const uint8_t MODE_PROFILE_VERSION = 1;
static const uint8_t MAX_MODE_PROFILES = 8;
static const uint8_t MAX_MODE_STAGES = 6;

// One profile can express multi-step cook logic.
struct ModeStageConfig {
    float targetTempC;
    uint32_t durationSec;
    uint8_t fanPercent;
    uint8_t smokePercent;
};

struct EmergencyThresholds {
    // Thermocouple max in deci-Celsius (e.g. 4500 = 450.0C)
    int16_t thermocoupleMaxDeciC;

    // Internal temperature range checks
    float intTempMinC;
    float intTempMaxC;

    // ADC range checks
    float extAdcMinV;
    float extAdcMaxV;

    // Smoke density upper bound on EXT_ADC_0
    float smokeDensityMaxV;

    // Ignitor max on time
    uint32_t ignitorMaxOnMs;

    // Sensor stale timeout
    uint32_t sensorStaleTimeoutMs;
};

struct ModeProfileConfig {
    uint8_t version;
    uint8_t modeId;
    bool enabled;

    char name[24];

    uint8_t stageCount;
    ModeStageConfig stages[MAX_MODE_STAGES];

    float keepWarmTempC;
    uint16_t pelletFeedDurationSec;

    EmergencyThresholds emergency;
};

class ModeProfileStore {
public:
    // Returns 0 on success, otherwise ErrorCode with profile/storage error number.
    uint32_t begin(const char* nameSpace = "mode_prof");
    void end();

    uint32_t load(uint8_t modeId, ModeProfileConfig& outProfile);
    uint32_t save(const ModeProfileConfig& profile);
    uint32_t erase(uint8_t modeId);

    bool exists(uint8_t modeId);
    static void setDefaults(uint8_t modeId, ModeProfileConfig& profile);
    static bool validate(const ModeProfileConfig& profile);

private:
    Preferences prefs;
    bool opened = false;

    bool makeKey(uint8_t modeId, char* outKey, uint8_t outSize) const;
};

#endif
