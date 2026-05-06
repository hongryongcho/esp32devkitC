#ifndef ERROR_CODE_H
#define ERROR_CODE_H

#include <stdint.h>

namespace ErrorCode {

// 32-bit error code layout
// [31:24] module, [23:16] source, [15:0] error number
enum Module : uint8_t {
    MODULE_NONE = 0,
    MODULE_SAFETY = 1,
    MODULE_MODE_PROFILE = 2
};

enum Source : uint8_t {
    SRC_NONE = 0,
    SRC_INT_TEMP = 1,
    SRC_EXT_ADC = 2,
    SRC_THERMOCOUPLE = 3,
    SRC_IGNITOR = 4,
    SRC_PROFILE_STORE = 5,
    SRC_RUNTIME = 6
};

inline uint32_t make(uint8_t module, uint8_t source, uint16_t errorNumber)
{
    return ((uint32_t)module << 24) | ((uint32_t)source << 16) | (uint32_t)errorNumber;
}

inline uint8_t module(uint32_t code)
{
    return (uint8_t)((code >> 24) & 0xFFU);
}

inline uint8_t source(uint32_t code)
{
    return (uint8_t)((code >> 16) & 0xFFU);
}

inline uint16_t number(uint32_t code)
{
    return (uint16_t)(code & 0xFFFFU);
}

inline bool isError(uint32_t code)
{
    return code != 0U;
}

// Safety error numbers (1000 range)
enum SafetyErrorNumber : uint16_t {
    SAFETY_OK = 0,
    SAFETY_TC_DATA_MISSING = 1001,
    SAFETY_TC_OVER_MAX = 1002,
    SAFETY_INT_TEMP_OUT_OF_RANGE = 1003,
    SAFETY_EXT_ADC_OUT_OF_RANGE = 1004,
    SAFETY_SMOKE_DENSITY_HIGH = 1005,
    SAFETY_IGNITOR_TIMEOUT = 1006,
    SAFETY_SENSOR_STALE = 1007,
    SAFETY_PROFILE_INVALID = 1008
};

// Mode profile/storage error numbers (2000 range)
enum ModeProfileErrorNumber : uint16_t {
    PROFILE_OK = 0,
    PROFILE_NOT_FOUND = 2001,
    PROFILE_INVALID = 2002,
    PROFILE_VERSION_MISMATCH = 2003,
    PROFILE_STORAGE_OPEN_FAILED = 2004,
    PROFILE_STORAGE_READ_FAILED = 2005,
    PROFILE_STORAGE_WRITE_FAILED = 2006
};

inline const char* toString(uint16_t errorNumber)
{
    switch (errorNumber) {
        case SAFETY_OK: return "OK";
        case SAFETY_TC_DATA_MISSING: return "SAFETY_TC_DATA_MISSING";
        case SAFETY_TC_OVER_MAX: return "SAFETY_TC_OVER_MAX";
        case SAFETY_INT_TEMP_OUT_OF_RANGE: return "SAFETY_INT_TEMP_OUT_OF_RANGE";
        case SAFETY_EXT_ADC_OUT_OF_RANGE: return "SAFETY_EXT_ADC_OUT_OF_RANGE";
        case SAFETY_SMOKE_DENSITY_HIGH: return "SAFETY_SMOKE_DENSITY_HIGH";
        case SAFETY_IGNITOR_TIMEOUT: return "SAFETY_IGNITOR_TIMEOUT";
        case SAFETY_SENSOR_STALE: return "SAFETY_SENSOR_STALE";
        case SAFETY_PROFILE_INVALID: return "SAFETY_PROFILE_INVALID";
        case PROFILE_NOT_FOUND: return "PROFILE_NOT_FOUND";
        case PROFILE_INVALID: return "PROFILE_INVALID";
        case PROFILE_VERSION_MISMATCH: return "PROFILE_VERSION_MISMATCH";
        case PROFILE_STORAGE_OPEN_FAILED: return "PROFILE_STORAGE_OPEN_FAILED";
        case PROFILE_STORAGE_READ_FAILED: return "PROFILE_STORAGE_READ_FAILED";
        case PROFILE_STORAGE_WRITE_FAILED: return "PROFILE_STORAGE_WRITE_FAILED";
        default: return "UNKNOWN_ERROR_NUMBER";
    }
}

} // namespace ErrorCode

#endif
