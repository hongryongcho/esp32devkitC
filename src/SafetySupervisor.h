#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

#include <stdint.h>

#include "ModeProfile.h"
#include "ErrorCode.h"

static const uint8_t SAFETY_INT_TEMP_CH_COUNT = 6;
static const uint8_t SAFETY_EXT_ADC_CH_COUNT = 8;
static const uint8_t SAFETY_TC_CH_COUNT = 4;

struct SafetyInputs {
    float intTempC[SAFETY_INT_TEMP_CH_COUNT];
    float extAdcV[SAFETY_EXT_ADC_CH_COUNT];
    int16_t tcDeciC[SAFETY_TC_CH_COUNT];

    bool tcDataValid;
    bool ignitorOn;

    // Runtime values for timeout checks.
    uint32_t nowMs;
    uint32_t ignitorOnStartMs;
    uint32_t lastSensorUpdateMs;
};

struct SafetyResult {
    uint32_t errorCode;
    uint16_t errorNumber;
    bool emergencyStop;
};

class SafetySupervisor {
public:
    SafetyResult checkCommonEmergency(const SafetyInputs& in,
                                      const ModeProfileConfig& profile) const;

private:
    SafetyResult makeResult(uint8_t source, uint16_t errorNumber, bool emergencyStop) const;
};

#endif
