#include "SafetySupervisor.h"

SafetyResult SafetySupervisor::makeResult(uint8_t source, uint16_t errorNumber, bool emergencyStop) const
{
    SafetyResult out;
    out.errorCode = ErrorCode::make(ErrorCode::MODULE_SAFETY, source, errorNumber);
    out.errorNumber = errorNumber;
    out.emergencyStop = emergencyStop;
    return out;
}

SafetyResult SafetySupervisor::checkCommonEmergency(const SafetyInputs& in,
                                                    const ModeProfileConfig& profile) const
{
    if (!ModeProfileStore::validate(profile)) {
        return makeResult(ErrorCode::SRC_PROFILE_STORE,
                          ErrorCode::SAFETY_PROFILE_INVALID,
                          true);
    }

    if (!in.tcDataValid) {
        return makeResult(ErrorCode::SRC_THERMOCOUPLE,
                          ErrorCode::SAFETY_TC_DATA_MISSING,
                          true);
    }

    // Thermocouple max check
    for (uint8_t i = 0; i < SAFETY_TC_CH_COUNT; i++) {
        if (in.tcDeciC[i] > profile.emergency.thermocoupleMaxDeciC) {
            return makeResult(ErrorCode::SRC_THERMOCOUPLE,
                              ErrorCode::SAFETY_TC_OVER_MAX,
                              true);
        }
    }

    // Internal temperature range check
    for (uint8_t i = 0; i < SAFETY_INT_TEMP_CH_COUNT; i++) {
        if (in.intTempC[i] < profile.emergency.intTempMinC ||
            in.intTempC[i] > profile.emergency.intTempMaxC) {
            return makeResult(ErrorCode::SRC_INT_TEMP,
                              ErrorCode::SAFETY_INT_TEMP_OUT_OF_RANGE,
                              true);
        }
    }

    // External ADC range check
    for (uint8_t i = 0; i < SAFETY_EXT_ADC_CH_COUNT; i++) {
        if (in.extAdcV[i] < profile.emergency.extAdcMinV ||
            in.extAdcV[i] > profile.emergency.extAdcMaxV) {
            return makeResult(ErrorCode::SRC_EXT_ADC,
                              ErrorCode::SAFETY_EXT_ADC_OUT_OF_RANGE,
                              true);
        }
    }

    // Smoke density high check on EXT_ADC_0
    if (in.extAdcV[0] > profile.emergency.smokeDensityMaxV) {
        return makeResult(ErrorCode::SRC_EXT_ADC,
                          ErrorCode::SAFETY_SMOKE_DENSITY_HIGH,
                          true);
    }

    // Ignitor timeout check
    if (in.ignitorOn) {
        const uint32_t ignitorElapsed = in.nowMs - in.ignitorOnStartMs;
        if (ignitorElapsed > profile.emergency.ignitorMaxOnMs) {
            return makeResult(ErrorCode::SRC_IGNITOR,
                              ErrorCode::SAFETY_IGNITOR_TIMEOUT,
                              true);
        }
    }

    // Sensor stale watchdog check
    const uint32_t sensorStaleMs = in.nowMs - in.lastSensorUpdateMs;
    if (sensorStaleMs > profile.emergency.sensorStaleTimeoutMs) {
        return makeResult(ErrorCode::SRC_RUNTIME,
                          ErrorCode::SAFETY_SENSOR_STALE,
                          true);
    }

    SafetyResult ok;
    ok.errorCode = 0;
    ok.errorNumber = ErrorCode::SAFETY_OK;
    ok.emergencyStop = false;
    return ok;
}
