#include "ModeProfile.h"

#include <string.h>
#include <stdio.h>

uint32_t ModeProfileStore::begin(const char* nameSpace)
{
    if (opened) {
        return 0;
    }
    if (!prefs.begin(nameSpace, false)) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_STORAGE_OPEN_FAILED);
    }
    opened = true;
    return 0;
}

void ModeProfileStore::end()
{
    if (!opened) {
        return;
    }
    prefs.end();
    opened = false;
}

bool ModeProfileStore::makeKey(uint8_t modeId, char* outKey, uint8_t outSize) const
{
    if (outKey == NULL || outSize < 4U) {
        return false;
    }
    if (modeId >= MAX_MODE_PROFILES) {
        return false;
    }
    // p00 ~ p07
    snprintf(outKey, outSize, "p%02u", (unsigned)modeId);
    return true;
}

bool ModeProfileStore::validate(const ModeProfileConfig& profile)
{
    if (profile.version != MODE_PROFILE_VERSION) {
        return false;
    }
    if (profile.modeId >= MAX_MODE_PROFILES) {
        return false;
    }
    if (profile.stageCount == 0 || profile.stageCount > MAX_MODE_STAGES) {
        return false;
    }
    if (profile.pelletFeedDurationSec == 0 || profile.pelletFeedDurationSec > 600U) {
        return false;
    }
    if (profile.emergency.intTempMinC >= profile.emergency.intTempMaxC) {
        return false;
    }
    if (profile.emergency.extAdcMinV >= profile.emergency.extAdcMaxV) {
        return false;
    }
    if (profile.emergency.ignitorMaxOnMs == 0U) {
        return false;
    }
    if (profile.emergency.sensorStaleTimeoutMs == 0U) {
        return false;
    }
    return true;
}

void ModeProfileStore::setDefaults(uint8_t modeId, ModeProfileConfig& profile)
{
    memset(&profile, 0, sizeof(profile));

    profile.version = MODE_PROFILE_VERSION;
    profile.modeId = (modeId < MAX_MODE_PROFILES) ? modeId : 0;
    profile.enabled = true;

    snprintf(profile.name, sizeof(profile.name), "MODE_%u", (unsigned)profile.modeId);

    profile.stageCount = 1;
    profile.stages[0].targetTempC = 120.0f;
    profile.stages[0].durationSec = 3600U;
    profile.stages[0].fanPercent = 40U;
    profile.stages[0].smokePercent = 40U;

    profile.keepWarmTempC = 70.0f;
    profile.pelletFeedDurationSec = 5U;

    profile.emergency.thermocoupleMaxDeciC = 4500;
    profile.emergency.intTempMinC = -40.0f;
    profile.emergency.intTempMaxC = 350.0f;
    profile.emergency.extAdcMinV = 0.0f;
    profile.emergency.extAdcMaxV = 5.0f;
    profile.emergency.smokeDensityMaxV = 4.5f;
    profile.emergency.ignitorMaxOnMs = 120000U;
    profile.emergency.sensorStaleTimeoutMs = 3000U;
}

uint32_t ModeProfileStore::load(uint8_t modeId, ModeProfileConfig& outProfile)
{
    if (!opened) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_STORAGE_OPEN_FAILED);
    }

    char key[8];
    if (!makeKey(modeId, key, sizeof(key))) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_NOT_FOUND);
    }

    const size_t expected = sizeof(ModeProfileConfig);
    const size_t got = prefs.getBytes(key, &outProfile, expected);

    if (got == 0U) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_NOT_FOUND);
    }
    if (got != expected) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_STORAGE_READ_FAILED);
    }
    if (outProfile.version != MODE_PROFILE_VERSION) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_VERSION_MISMATCH);
    }
    if (!validate(outProfile)) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_INVALID);
    }

    return 0;
}

uint32_t ModeProfileStore::save(const ModeProfileConfig& profile)
{
    if (!opened) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_STORAGE_OPEN_FAILED);
    }

    if (!validate(profile)) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_INVALID);
    }

    char key[8];
    if (!makeKey(profile.modeId, key, sizeof(key))) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_INVALID);
    }

    const size_t wrote = prefs.putBytes(key, &profile, sizeof(profile));
    if (wrote != sizeof(profile)) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_STORAGE_WRITE_FAILED);
    }

    return 0;
}

uint32_t ModeProfileStore::erase(uint8_t modeId)
{
    if (!opened) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_STORAGE_OPEN_FAILED);
    }

    char key[8];
    if (!makeKey(modeId, key, sizeof(key))) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_NOT_FOUND);
    }

    const bool ok = prefs.remove(key);
    if (!ok) {
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_NOT_FOUND);
    }

    return 0;
}

bool ModeProfileStore::exists(uint8_t modeId)
{
    if (!opened) {
        return false;
    }

    char key[8];
    if (!makeKey(modeId, key, sizeof(key))) {
        return false;
    }

    return prefs.isKey(key);
}
