#include "batagota.h"
#include <cstdio>
#include <algorithm>
#include <cstdlib>
#include <climits>
#include <time.h>
#include <Arduino.h>
#include <WiFi.h>
#include "ConfigManager.h"

namespace {
const float IGNITOR_SMOKE_ADC0_READY_V = 0.30f;
const unsigned long IGNITOR_MAX_ON_MS = 120000UL;
const unsigned long FIRE_IGNITION_TIMEOUT_MS = 60000UL;
const float SMOKE_DENSITY_ON_THRESHOLD_V = 0.35f;
const float SMOKE_DENSITY_OFF_THRESHOLD_V = 0.25f;

int clampPercentToExtDacTarget(int percent)
{
    if (percent < 0) {
        percent = 0;
    } else if (percent > 100) {
        percent = 100;
    }
    return percent * 5;
}
}

// ============================================================================
// Constructor
// ============================================================================
Batagota::Batagota()
    : currentState((uint16_t)StartSubState::INIT),
      tempCount(0),
      initialized(false),
        loopCounter10mSec(0), loopCounter100mSec(0), loopCounter1Sec(0)
{
}

// ============================================================================
// Destructor
// ============================================================================
Batagota::~Batagota()
{
}

// ============================================================================
// Initialization
// ============================================================================
void Batagota::init()
{
    uint32_t profileErr = modeProfileStore.begin();
    modeProfileStoreReady = (profileErr == 0U);
    if (!modeProfileStoreReady) {
        Serial.printf("[MODE-PROFILE] begin failed, code=0x%08lX\n", (unsigned long)profileErr);
    }

    // Initialize External ADC channels (8 channels)
    for (uint8_t i = 0; i < MAX_EXT_ADC_CHANNELS; i++) {
        snprintf(extADCChannels[i].name, sizeof(extADCChannels[i].name), "EXT_ADC_%d", i);
        extADCChannels[i].value = 0.0f;
        extADCChannels[i].limitLow = 0.0f;
        extADCChannels[i].limitHigh = 5.0f;  // ADS1015 manager stores channel values as voltage (0~5V)
    }

    // Air condition sensors on External ADC channels (voltage input: 0~5V)
    // Channel 1: MQ2 (100~20000 ppm), Channel 2: MQ7 (10~1000 ppm), Channel 3: MQ135 (10~300 ppm)
    strncpy(extADCChannels[1].name, "MQ2", sizeof(extADCChannels[1].name) - 1);
    strncpy(extADCChannels[2].name, "MQ7", sizeof(extADCChannels[2].name) - 1);
    strncpy(extADCChannels[3].name, "MQ135", sizeof(extADCChannels[3].name) - 1);
    extADCChannels[1].limitLow = 0.0f;
    extADCChannels[1].limitHigh = 5.0f;
    extADCChannels[2].limitLow = 0.0f;
    extADCChannels[2].limitHigh = 5.0f;
    extADCChannels[3].limitLow = 0.0f;
    extADCChannels[3].limitHigh = 5.0f;
    
    // Initialize Temperature channels (6 BBQ channels from 8 InternalADC channels)
    tempCount = 6;
    strncpy(tempChannels[0].name, "BBQ1", sizeof(tempChannels[0].name) - 1);
    strncpy(tempChannels[1].name, "BBQ2", sizeof(tempChannels[1].name) - 1);
    strncpy(tempChannels[2].name, "BBQ3", sizeof(tempChannels[2].name) - 1);
    strncpy(tempChannels[3].name, "BBQ4", sizeof(tempChannels[3].name) - 1);
    strncpy(tempChannels[4].name, "SMOKE_TANK", sizeof(tempChannels[4].name) - 1);
    strncpy(tempChannels[5].name, "OUTSIDE", sizeof(tempChannels[5].name) - 1);
    
    // Reserved channels (INT_TEMP_6, INT_TEMP_7) for future use
    for (uint8_t i = 6; i < MAX_INT_ADC_CHANNELS; i++) {
        tempChannels[i].name[0] = '\0';
    }
    
    initialized = true;
    loopCounter10mSec = 0;
    loopCounter100mSec = 0;
    loopCounter1Sec = 0;
    lastSensorUpdateMs = millis();

    if (ensureActiveModeProfileLoaded() != 0U) {
        Serial.println("[MODE-PROFILE] fallback to default profile");
    }
}

// ============================================================================
// Main Loop Functions
// ============================================================================

void Batagota::loop10mSec()
{
    if (!initialized) return;
    
    loopCounter10mSec++;
    
    // Call control algorithm for 10ms loop
    controlAlgorithm10mSec();
}

void Batagota::loop100mSec()
{
    if (!initialized) return;
    
    loopCounter100mSec++;
    
    // Call control algorithm for 100ms loop
    controlAlgorithm100mSec();
}

void Batagota::loop1Sec()
{
    if (!initialized) return;
    
    loopCounter1Sec++;
    
    // Call control algorithm for 1 second loop
    controlAlgorithm1Sec();
}

// ============================================================================
// Data Update Methods (using enum-based channel access)
// ============================================================================

void Batagota::setExternalADCValue(ExternalADCInputChannel channel, float value)
{
    // External ADC values can be stored if needed
    if (channel < MAX_EXT_ADC_CHANNELS) {
        extADCChannels[(uint8_t)channel].value = value;
    }
}

void Batagota::setADCLimits(ExternalADCInputChannel channel, float limitLow, float limitHigh)
{
    // Set ADC value range limits for error checking
    if (channel < MAX_EXT_ADC_CHANNELS) {
        extADCChannels[(uint8_t)channel].limitLow = limitLow;
        extADCChannels[(uint8_t)channel].limitHigh = limitHigh;
    }
}

void Batagota::setInternalTemperatureValue(InternalADCChannel channel, float value)
{
    // Set temperature value from ADC (not voltage)
    if (channel < MAX_INT_ADC_CHANNELS) {
        tempChannels[(uint8_t)channel].currentValue = value;
    }
}

void Batagota::setInternalTemperatureSetpoint(InternalADCChannel channel, float setpoint)
{
    // Set temperature setpoint
    if (channel < MAX_INT_ADC_CHANNELS) {
        tempChannels[(uint8_t)channel].setpoint = setpoint;
    }
}

void Batagota::setTemperatureLimits(InternalADCChannel channel, float limitLow, float limitHigh)
{
    // Set temperature range limits for error checking
    if (channel < MAX_INT_ADC_CHANNELS) {
        tempChannels[(uint8_t)channel].limitLow = limitLow;
        tempChannels[(uint8_t)channel].limitHigh = limitHigh;
    }
}

void Batagota::setInternalDACValue(InternalDACOutputChannel channel, float value)
{
    // Internal DAC values can be stored if needed
    // Currently just a placeholder for future use
}

void Batagota::setExternalDACValue(ExternalDACOutputChannel channel, float value)
{
    // External DAC values can be stored if needed
    // Currently just a placeholder for future use
}

// ============================================================================
// Data Read Methods (using enum-based channel access)
// ============================================================================

float Batagota::getExternalADCValue(ExternalADCInputChannel channel) const
{
    // Get external ADC value
    if (channel < MAX_EXT_ADC_CHANNELS) {
        return extADCChannels[(uint8_t)channel].value;
    }
    return 0.0f;
}

float Batagota::getInternalTemperatureValue(InternalADCChannel channel) const
{
    // Get temperature value (not voltage)
    if (channel < MAX_INT_ADC_CHANNELS) {
        return tempChannels[(uint8_t)channel].currentValue;
    }
    return 0.0f;
}

float Batagota::getInternalTemperatureSetpoint(InternalADCChannel channel) const
{
    // Get temperature setpoint
    if (channel < MAX_INT_ADC_CHANNELS) {
        return tempChannels[(uint8_t)channel].setpoint;
    }
    return 0.0f;
}

float Batagota::getInternalControlOutput(InternalADCChannel channel) const
{
    // Get control output for temperature channel
    if (channel < MAX_INT_ADC_CHANNELS) {
        return tempChannels[(uint8_t)channel].output;
    }
    return 0.0f;
}

float Batagota::getInternalDACValue(InternalDACOutputChannel channel) const
{
    // Get internal DAC value
    // Placeholder - can be implemented if needed
    return 0.0f;
}

float Batagota::getExternalDACValue(ExternalDACOutputChannel channel) const
{
    // Get external DAC value
    // Placeholder - can be implemented if needed
    return 0.0f;
}

// ============================================================================
// ADC Range Error Check Methods
// ============================================================================

bool Batagota::checkADCError(ExternalADCInputChannel channel) const
{
    // Check if ADC value is within acceptable range (limitLow to limitHigh)
    // Returns: true if WITHIN limits, false if OUT OF range
    if (channel < MAX_EXT_ADC_CHANNELS) {
        float adc = extADCChannels[(uint8_t)channel].value;
        float low = extADCChannels[(uint8_t)channel].limitLow;
        float high = extADCChannels[(uint8_t)channel].limitHigh;
        
        // Check if ADC value is within the valid range
        return (adc >= low && adc <= high);
    }
    return false;  // Invalid channel
}

float Batagota::getADCLimitLow(ExternalADCInputChannel channel) const
{
    // Get lower limit for ADC channel
    if (channel < MAX_EXT_ADC_CHANNELS) {
        return extADCChannels[(uint8_t)channel].limitLow;
    }
    return 0.0f;
}

float Batagota::getADCLimitHigh(ExternalADCInputChannel channel) const
{
    // Get upper limit for ADC channel
    if (channel < MAX_EXT_ADC_CHANNELS) {
        return extADCChannels[(uint8_t)channel].limitHigh;
    }
    return 0.0f;
}

bool Batagota::checkADCErrorByIndex(uint8_t index) const
{
    // Check if ADC value is within limits by numeric index
    // Returns: true if WITHIN limits, false if OUT OF range
    if (index < MAX_EXT_ADC_CHANNELS) {
        return checkADCError((ExternalADCInputChannel)index);
    }
    return false;  // Invalid index
}

// ============================================================================
// Temperature Range Error Check Methods
// ============================================================================

bool Batagota::checkTemperatureError(InternalADCChannel channel) const
{
    // Check if temperature is within acceptable range (limitLow to limitHigh)
    // Returns: true if WITHIN limits, false if OUT OF range
    if (channel < MAX_INT_ADC_CHANNELS) {
        float temp = tempChannels[(uint8_t)channel].currentValue;
        float low = tempChannels[(uint8_t)channel].limitLow;
        float high = tempChannels[(uint8_t)channel].limitHigh;
        
        // Check if temperature is within the valid range
        return (temp >= low && temp <= high);
    }
    return false;  // Invalid channel
}

bool Batagota::checkBBQTemperatureError(BBQTemperatureChannel channel) const
{
    // Check if BBQ temperature is within acceptable range
    // Returns: true if WITHIN limits, false if OUT OF range
    if (channel >= TR_BBQ1 && channel <= TR_OUTSIDE) {
        return checkTemperatureError((InternalADCChannel)channel);
    }
    return false;  // Invalid channel
}

float Batagota::getTemperatureLimitLow(InternalADCChannel channel) const
{
    // Get lower limit for temperature channel
    if (channel < MAX_INT_ADC_CHANNELS) {
        return tempChannels[(uint8_t)channel].limitLow;
    }
    return 0.0f;
}

float Batagota::getTemperatureLimitHigh(InternalADCChannel channel) const
{
    // Get upper limit for temperature channel
    if (channel < MAX_INT_ADC_CHANNELS) {
        return tempChannels[(uint8_t)channel].limitHigh;
    }
    return 0.0f;
}

// ============================================================================
// Channel Query Methods
// ============================================================================

const char* Batagota::getTemperatureChannelName(uint8_t index) const
{
    if (index < tempCount) {
        return tempChannels[index].name;
    }
    return "";
}

// ============================================================================
// Control Algorithm Methods
// ============================================================================

void Batagota::controlAlgorithm10mSec()
{
    // ========== 10ms Loop ==========
    // Fast control loop - use for:
    // - Fast sensor read and validation
    // - Quick feedback control
    // - Real-time interrupt-driven tasks
    
    // Example placeholder:
    // - Read ADC channels
    // - Early warning checks
    // - Fast PID calculations if needed
}

void Batagota::controlAlgorithm100mSec()
{
    // ========== 100ms Loop ==========
    // Medium speed control - use for:
    // - DAC output updates
    // - Standard control calculations
    // - Temperature control commands
    
    // Compute control outputs for all temperature channels
    computeControlOutputs();
}

void Batagota::controlAlgorithm1Sec()
{
    // ========== 1 Second Loop ==========
    // Slow control loop - use for:
    // - Logging and diagnostics
    // - Slow parameter tuning
    // - Status updates
    // - Communication with external systems
    
    // Example placeholder:
    // - Log temperature and output values
    // - Print diagnostics to debug console
}

void Batagota::computeControlOutputs()
{
    // Simple proportional controller for temperature regulation
    // Can be extended to implement PID, fuzzy logic, etc.
    
    for (uint8_t i = 0; i < tempCount; i++) {
        // Calculate error (setpoint - current value)
        float error = tempChannels[i].setpoint - tempChannels[i].currentValue;
        
        // Simple proportional gain (adjust KP for your application)
        const float KP = 0.1f;  
        tempChannels[i].output = error * KP;
        
        // Clamp output to valid range [0.0, 1.0]
        if (tempChannels[i].output < 0.0f) {
            tempChannels[i].output = 0.0f;
        }
        if (tempChannels[i].output > 1.0f) {
            tempChannels[i].output = 1.0f;
        }
    }
}

// ============================================================================
// Helper Methods for Temperature Index Access
// ============================================================================

float Batagota::getTemperatureByIndex(uint8_t index) const
{
    // Returns temperature value by numeric index
    if (index < MAX_INT_ADC_CHANNELS) {
        return getInternalTemperatureValue((InternalADCChannel)index);
    }
    return -999.0f;  // Error value
}

float Batagota::getSetpointByIndex(uint8_t index) const
{
    // Returns temperature setpoint by numeric index
    if (index < MAX_INT_ADC_CHANNELS) {
        return getInternalTemperatureSetpoint((InternalADCChannel)index);
    }
    return 0.0f;  // Error value
}

float Batagota::getControlOutputByIndex(uint8_t index) const
{
    // Returns control output by numeric index
    if (index < MAX_INT_ADC_CHANNELS) {
        return getInternalControlOutput((InternalADCChannel)index);
    }
    return 0.0f;  // Error value
}

bool Batagota::checkTemperatureErrorByIndex(uint8_t index) const
{
    // Check if temperature is within limits by numeric index
    // Returns: true if WITHIN limits, false if OUT OF range
    if (index < MAX_INT_ADC_CHANNELS) {
        return checkTemperatureError((InternalADCChannel)index);
    }
    return false;  // Invalid index
}

// ============================================================================
// BBQ Temperature Sensor Access Methods (by Channel Name)
// ============================================================================

float Batagota::getBBQTemperature(BBQTemperatureChannel channel) const
{
    // Get current temperature by BBQ channel
    if (channel >= TR_BBQ1 && channel <= TR_OUTSIDE) {
        return getInternalTemperatureValue((InternalADCChannel)channel);
    }
    return -999.0f;  // Error value
}

float Batagota::getBBQSetpoint(BBQTemperatureChannel channel) const
{
    // Get target setpoint by BBQ channel
    if (channel >= TR_BBQ1 && channel <= TR_OUTSIDE) {
        return getInternalTemperatureSetpoint((InternalADCChannel)channel);
    }
    return 0.0f;  // Error value
}

float Batagota::getBBQControlOutput(BBQTemperatureChannel channel) const
{
    // Get control output by BBQ channel
    if (channel >= TR_BBQ1 && channel <= TR_OUTSIDE) {
        return getInternalControlOutput((InternalADCChannel)channel);
    }
    return 0.0f;  // Error value
}

void Batagota::setBBQSetpoint(BBQTemperatureChannel channel, float setpoint)
{
    // Set target setpoint by BBQ channel
    if (channel >= TR_BBQ1 && channel <= TR_OUTSIDE) {
        setInternalTemperatureSetpoint((InternalADCChannel)channel, setpoint);
    }
}

void Batagota::setBBQTemperatureLimits(BBQTemperatureChannel channel, float limitLow, float limitHigh)
{
    // Set temperature range limits for BBQ channel
    if (channel >= TR_BBQ1 && channel <= TR_OUTSIDE) {
        setTemperatureLimits((InternalADCChannel)channel, limitLow, limitHigh);
    }
}

bool Batagota::getBBQTemperatureLimitStatus(BBQTemperatureChannel channel) const
{
    // Check if BBQ temperature is within acceptable range
    // Returns: true if WITHIN limits, false if OUT OF range
    if (channel >= TR_BBQ1 && channel <= TR_OUTSIDE) {
        return checkTemperatureError((InternalADCChannel)channel);
    }
    return false;  // Invalid channel
}

const char* Batagota::getBBQChannelName(BBQTemperatureChannel channel) const
{
    // Return human-readable name for BBQ channel
    switch (channel) {
        case TR_BBQ1:      return "TR_BBQ1";
        case TR_BBQ2:      return "TR_BBQ2";
        case TR_BBQ3:      return "TR_BBQ3";
        case TR_BBQ4:      return "TR_BBQ4";
        case TR_SMOKE_TANK: return "TR_SMOKE_TANK";
        case TR_OUTSIDE:   return "TR_OUTSIDE";
        default:        return "UNKNOWN";
    }
}

const char* Batagota::getInternalTemperatureChannelName(InternalADCChannel channel) const
{
    // Return human-readable name for internal temperature channel
    if (channel < tempCount) {
        return tempChannels[(uint8_t)channel].name;
    }
    return "UNKNOWN";
}

const char* Batagota::getExternalADCChannelName(ExternalADCInputChannel channel) const
{
    switch (channel) {
        case EXT_ADC_0: return "EXT_ADC_0";
        case EXT_ADC_1: return "MQ2";
        case EXT_ADC_2: return "MQ7";
        case EXT_ADC_3: return "MQ135";
        case EXT_ADC_4: return "EXT_ADC_4";
        case EXT_ADC_5: return "EXT_ADC_5";
        case EXT_ADC_6: return "EXT_ADC_6";
        case EXT_ADC_7: return "EXT_ADC_7";
        default: return "UNKNOWN";
    }
}

const char* Batagota::getThermocoupleChannelName(ThermocoupleChannel channel) const
{
    switch (channel) {
        case TC_NEAR_IGNITOR: return "TC_NEAR_IGNITOR";
        case TC_FAR_IGNITOR: return "TC_FAR_IGNITOR";
        case TC_FRONT_OVEN: return "TC_FRONT_OVEN";
        case TC_BACK_OVEN: return "TC_BACK_OVEN";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// State Machine Implementation
// ============================================================================

void Batagota::stateInit()
{
    // State Machine 초기화
    currentState = (uint16_t)StartSubState::INIT;
    stateEntryTime = millis();
    algoRuntime.stateTimeElapsed = 0;
    algoRuntime.stateTimeInSeconds = 0;
    algoRuntime.lastErrorCode = 0;
    algoRuntime.pelletFeedStartMs = 0;
    algoRuntime.ignitorOnStartMs = 0;
    algoRuntime.debugJsonPrintEnabled = true;   // HMI JSON stream ON by default at boot
    algoRuntime.pelletFeedFlag = false;
    algoRuntime.ignitorState = IgnitorState::OFF;
    algoRuntime.ignitorTurnOnCommand = false;
    algoRuntime.ignitorTurnOffCommand = false;
    algoRuntime.fireIgnitionSequenceStarted = false;
    algoRuntime.stateUartLogEnabled = false;
    algoRuntime.runCommandReservedArg1 = 0;
    algoRuntime.runCommandReservedArg2 = 0;

    pendingBuzzerCommand = false;
    pendingBuzzerCount = 0;
    pendingBuzzerId = 0;
    lastAnnouncedMainState = (uint16_t)MainState::START;
    cookMainEntryMs = millis();
    lastCookAnnouncedHour = 0;
    airFanCycleStartMs = 0;
    airFanCycleMainState = 0;
    cookSprayCycleStartMs = 0;
    cookSprayOnStartMs = 0;
    cookSprayActive = false;
    queueBuzzerCommand(1, 15);  // START 진입 사운드

    lastSensorUpdateMs = millis();
    if (ensureActiveModeProfileLoaded() != 0U) {
        Serial.println("[MODE-PROFILE] stateInit load failed, using defaults");
    }
}

uint32_t Batagota::ensureActiveModeProfileLoaded()
{
    activeModeId = configManager.getOperationMode();

    // When profile store is unavailable, force deterministic defaults.
    if (!modeProfileStoreReady) {
        ModeProfileStore::setDefaults(activeModeId, activeModeProfile);
        activeModeProfileLoaded = true;
        return ErrorCode::make(ErrorCode::MODULE_MODE_PROFILE,
                               ErrorCode::SRC_PROFILE_STORE,
                               ErrorCode::PROFILE_STORAGE_OPEN_FAILED);
    }

    uint32_t err = modeProfileStore.load(activeModeId, activeModeProfile);
    if (err == 0U) {
        activeModeProfileLoaded = true;
        return 0U;
    }

    // Auto-heal missing/invalid profile by writing defaults.
    ModeProfileStore::setDefaults(activeModeId, activeModeProfile);
    uint32_t saveErr = modeProfileStore.save(activeModeProfile);
    activeModeProfileLoaded = true;
    if (saveErr != 0U) {
        return saveErr;
    }
    return err;
}

SafetyResult Batagota::runCommonSafetyCheck(unsigned long nowMs) const
{
    SafetyInputs in = {};

    for (uint8_t i = 0; i < SAFETY_INT_TEMP_CH_COUNT; i++) {
        in.intTempC[i] = tempChannels[i].currentValue;
    }

    for (uint8_t i = 0; i < SAFETY_EXT_ADC_CH_COUNT; i++) {
        in.extAdcV[i] = extADCChannels[i].value;
    }

    in.tcDataValid = (driveThermocoupleData != NULL);
    if (in.tcDataValid) {
        for (uint8_t i = 0; i < SAFETY_TC_CH_COUNT; i++) {
            in.tcDeciC[i] = driveThermocoupleData[i];
        }
    }

    in.ignitorOn = (algoRuntime.ignitorState == IgnitorState::OnGoing || algoRuntime.ignitorState == IgnitorState::ON);
    in.nowMs = (uint32_t)nowMs;
    in.ignitorOnStartMs = (uint32_t)algoRuntime.ignitorOnStartMs;
    in.lastSensorUpdateMs = (uint32_t)lastSensorUpdateMs;

    return safetySupervisor.checkCommonEmergency(in, activeModeProfile);
}

void Batagota::setStateErrorCodeDirect(uint32_t errorCode)
{
    algoRuntime.lastErrorCode = errorCode;

    if (errorHistoryCount < MAX_ERROR_HISTORY) {
        errorHistory[errorHistoryCount] = errorCode;
        errorHistoryCount++;
    }
}

void Batagota::queueBuzzerCommand(uint8_t count, uint8_t id)
{
    pendingBuzzerCount = count;
    pendingBuzzerId = (uint8_t)(id & 0x0F);
    pendingBuzzerCommand = true;
}

void Batagota::handleMainStateBuzzerOnTransition(uint16_t prevMainState, uint16_t nextMainState, unsigned long nowMs)
{
    (void)prevMainState;

    lastAnnouncedMainState = nextMainState;

    if (nextMainState == (uint16_t)MainState::COOK ||
        nextMainState == (uint16_t)MainState::RAP ||
        nextMainState == (uint16_t)MainState::DRY) {
        airFanCycleStartMs = nowMs;
        airFanCycleMainState = nextMainState;
    } else {
        airFanCycleStartMs = 0;
        airFanCycleMainState = 0;
    }

    if (nextMainState == (uint16_t)MainState::COOK) {
        cookSprayCycleStartMs = nowMs;
    } else {
        cookSprayCycleStartMs = 0;
        cookSprayActive = false;
        cookSprayOnStartMs = 0;
    }

    switch (nextMainState) {
        case (uint16_t)MainState::START:
            queueBuzzerCommand(1, 15);
            break;
        case (uint16_t)MainState::IDLE:
            queueBuzzerCommand(1, 14);
            break;
        case (uint16_t)MainState::FIRE:
            queueBuzzerCommand(100, 0);
            break;
        case (uint16_t)MainState::HEAT:
            queueBuzzerCommand(100, 14);
            break;
        case (uint16_t)MainState::COOK:
            cookMainEntryMs = nowMs;
            lastCookAnnouncedHour = 0;
            queueBuzzerCommand(10, 1);
            break;
        case (uint16_t)MainState::RAP:
            rapMainEntryMs = nowMs;
            queueBuzzerCommand(10, 11);
            break;
        case (uint16_t)MainState::DRY:
            dryMainEntryMs = nowMs;
            queueBuzzerCommand(10, 12);
            break;
        case (uint16_t)MainState::RESET:
            queueBuzzerCommand(10, 13);
            break;
        default:
            // 매핑되지 않은 메인 상태(OIL 등)는 사운드를 중지하도록 count=0 전달
            queueBuzzerCommand(0, 0);
            break;
    }
}

void Batagota::handleCookHourBuzzer(uint16_t currentMainState, unsigned long nowMs)
{
    if (currentMainState != (uint16_t)MainState::COOK) {
        return;
    }

    if (nowMs < cookMainEntryMs) {
        cookMainEntryMs = nowMs;
        return;
    }

    const uint32_t elapsedHours = (uint32_t)((nowMs - cookMainEntryMs) / 3600000UL);
    if (elapsedHours <= lastCookAnnouncedHour) {
        return;
    }

    lastCookAnnouncedHour = elapsedHours;
    uint8_t cookSoundId = 10;
    if (elapsedHours < 10U) {
        cookSoundId = (uint8_t)(elapsedHours + 1U);
    }

    queueBuzzerCommand(10, cookSoundId);
}

void Batagota::applyRunCommandAlgoTime(int32_t hourArg, int32_t minuteArg, uint16_t targetMainState, unsigned long nowMs)
{
    int32_t safeHour = hourArg;
    int32_t safeMinute = minuteArg;

    if (safeHour < 0) {
        safeHour = 0;
    }
    if (safeMinute < 0) {
        safeMinute = 0;
    }

    // Normalize minute overflow into hour.
    safeHour += (safeMinute / 60);
    safeMinute = (safeMinute % 60);

    const uint32_t totalMinutes = ((uint32_t)safeHour * 60U) + (uint32_t)safeMinute;
    const uint32_t totalSeconds = totalMinutes * 60U;
    const unsigned long elapsedMs = (unsigned long)totalMinutes * 60000UL;

    algoRuntime.stateTimeInSeconds = totalSeconds;
    algoRuntime.stateTimeElapsed = elapsedMs;

    if (nowMs >= elapsedMs) {
        stateEntryTime = nowMs - elapsedMs;
    } else {
        stateEntryTime = 0;
    }

    if (targetMainState == (uint16_t)MainState::COOK) {
        if (nowMs >= elapsedMs) {
            cookMainEntryMs = nowMs - elapsedMs;
        } else {
            cookMainEntryMs = 0;
        }

        const uint32_t elapsedHours = (uint32_t)(totalMinutes / 60U);
        if (elapsedHours == 0U) {
            lastCookAnnouncedHour = 0;
        } else if (elapsedHours >= 9U) {
            // Ensure next update emits id=10 once for >=9h injection.
            lastCookAnnouncedHour = 8;
        } else {
            // Ensure next update emits the matching hour sound once.
            lastCookAnnouncedHour = elapsedHours - 1U;
        }
    } else if (targetMainState == (uint16_t)MainState::RAP) {
        if (nowMs >= elapsedMs) {
            rapMainEntryMs = nowMs - elapsedMs;
        } else {
            rapMainEntryMs = 0;
        }
    } else if (targetMainState == (uint16_t)MainState::DRY) {
        if (nowMs >= elapsedMs) {
            dryMainEntryMs = nowMs - elapsedMs;
        } else {
            dryMainEntryMs = 0;
        }
    }
}

uint32_t Batagota::buildStateErrorCode(uint16_t channel) const
{
    return ((uint32_t)channel << 16) | (uint32_t)currentState;
}

void Batagota::setStateError(uint16_t channel)
{
    algoRuntime.lastErrorCode = buildStateErrorCode(channel);

    // Keep only the first 4 detected errors until clearErrorHistory() is called.
    if (errorHistoryCount < MAX_ERROR_HISTORY) {
        errorHistory[errorHistoryCount] = algoRuntime.lastErrorCode;
        errorHistoryCount++;
    }
}

bool Batagota::parseUartCommandLine(const char* line, UartCommand& cmd) const
{
    if (line == NULL) {
        return false;
    }

    auto toLowerAscii = [](char ch) -> char {
        if (ch >= 'A' && ch <= 'Z') {
            return (char)(ch - 'A' + 'a');
        }
        return ch;
    };

    // Expected format: ba+set=1,1,0 or ba+run=1,0,0
    if (toLowerAscii(line[0]) != 'b' || toLowerAscii(line[1]) != 'a' || line[2] != '+') {
        return false;
    }

    const char* typeStart = line + 3;
    const char* equalPos = strchr(typeStart, '=');
    if (equalPos == NULL) {
        return false;
    }

    const size_t typeLen = (size_t)(equalPos - typeStart);
    if (typeLen == 3 &&
        toLowerAscii(typeStart[0]) == 's' &&
        toLowerAscii(typeStart[1]) == 'e' &&
        toLowerAscii(typeStart[2]) == 't') {
        cmd.type = UartCommandType::SET;
    } else if (typeLen == 3 &&
               toLowerAscii(typeStart[0]) == 'r' &&
               toLowerAscii(typeStart[1]) == 'u' &&
               toLowerAscii(typeStart[2]) == 'n') {
        cmd.type = UartCommandType::RUN;
    } else {
        cmd.type = UartCommandType::UNKNOWN;
        return false;
    }

    const char* argPtr = equalPos + 1;
    cmd.argCount = 0;
    const char* lastEndPtr = argPtr;

    while (*argPtr != '\0' && cmd.argCount < 3) {
        char* endPtr = NULL;
        long parsed = strtol(argPtr, &endPtr, 10);
        if (endPtr == argPtr) {
            return false;
        }
        if (parsed < INT32_MIN || parsed > INT32_MAX) {
            return false;
        }

        cmd.args[cmd.argCount] = (int32_t)parsed;
        cmd.argCount++;
        lastEndPtr = endPtr;

        if (*endPtr == ',') {
            argPtr = endPtr + 1;
            continue;
        }
        if (*endPtr == '\0') {
            break;
        }
        return false;
    }

    return (cmd.argCount == 3 && *lastEndPtr == '\0');
}

bool Batagota::executeSetCommand(const UartCommand& cmd)
{
    if (cmd.argCount < 3) {
        return false;
    }

    // set target 2: IDLE_WAIT or IDLE_MONITOR -> IDLE_SHUTDOWN
    if (cmd.args[0] == 2) {
        if (currentState == (uint16_t)IdleSubState::WAIT ||
            currentState == (uint16_t)IdleSubState::MONITOR) {
            setState((uint16_t)IdleSubState::SHUTDOWN);
            Serial.println("[UART-CMD] set state -> IDLE_SHUTDOWN");
            return true;
        }
        Serial.printf("[UART-CMD] set target 2 ignored at state=%u\n", currentState);
        return false;
    }

    // set target 3: IDLE_SHUTDOWN -> IDLE_MONITOR
    if (cmd.args[0] == 3) {
        if (currentState == (uint16_t)IdleSubState::SHUTDOWN) {
            setState((uint16_t)IdleSubState::MONITOR);
            Serial.println("[UART-CMD] set state -> IDLE_MONITOR");
            return true;
        }
        Serial.printf("[UART-CMD] set target 3 ignored at state=%u\n", currentState);
        return false;
    }

    // set target 4: IDLE_SHUTDOWN -> IDLE_WAIT
    if (cmd.args[0] == 4) {
        if (currentState == (uint16_t)IdleSubState::SHUTDOWN) {
            setState((uint16_t)IdleSubState::WAIT);
            Serial.println("[UART-CMD] set state -> IDLE_WAIT");
            return true;
        }
        Serial.printf("[UART-CMD] set target 4 ignored at state=%u\n", currentState);
        return false;
    }

    // set target 15: state UART log print flag control
    if (cmd.args[0] == 15) {
        setStateUartLogEnabled(cmd.args[1] != 0);
        Serial.printf("[UART-CMD] set flag(state_uart_log)=%d, rsv=%ld\n",
                      algoRuntime.stateUartLogEnabled ? 1 : 0,
                      (long)cmd.args[2]);
        return true;
    }

    // set target 16: HMI JSON stream ON/OFF  (ba+set=16,1,0 = ON / ba+set=16,0,0 = OFF)
    if (cmd.args[0] == 16) {
        setDebugJsonPrintEnabled(cmd.args[1] != 0);
        Serial.printf("[UART-CMD] set flag(hmi_json)=%d\n",
                      algoRuntime.debugJsonPrintEnabled ? 1 : 0);
        return true;
    }

    Serial.printf("[UART-CMD] Unknown set target: %ld\n", (long)cmd.args[0]);
    return false;
}

bool Batagota::executeRunCommand(const UartCommand& cmd)
{
    if (cmd.argCount < 3) {
        return false;
    }

    // Save arg1/arg2 for later diagnostics and future command extension.
    algoRuntime.runCommandReservedArg1 = cmd.args[1];
    algoRuntime.runCommandReservedArg2 = cmd.args[2];

    const unsigned long nowMs = millis();

    // Recipe select + return packet mode:
    // ba+run=1,<recipe_no>,0
    // - recipe_no를 active recipe로 설정
    // - HMI/MQTT 공용 JSON 패킷 1회 출력
    if (cmd.args[0] == 1 && cmd.args[2] == 0) {
        const int32_t recipeArg = cmd.args[1];
        const uint8_t recipeCount = configManager.getRecipeCount();
        if (recipeArg < 0 || recipeArg >= (int32_t)recipeCount) {
            Serial.printf("[UART-CMD] Invalid recipe index: %ld (valid: 0~%u)\n",
                          (long)recipeArg,
                          (unsigned)(recipeCount == 0 ? 0 : (recipeCount - 1)));
            return false;
        }

        const uint8_t recipeNo = (uint8_t)recipeArg;
        configManager.setOperationMode(recipeNo);
        const RecipeStageProfile& recipe = configManager.getRecipeProfile(recipeNo);

        Serial.print("{\"recipe_no\":");
        Serial.print((unsigned)recipeNo);
        Serial.print(",\"stages\":[");

        Serial.print("{\"stage_no\":0");
        Serial.print(",\"fuel_type\":");
        Serial.print(recipe.fuel_type);
        Serial.print(",\"smoke_enable\":");
        Serial.print(recipe.ignition.smoke_enable);
        Serial.print(",\"ignite_t1\":");
        Serial.print(recipe.ignition.ignite_t1);
        Serial.print(",\"ignite_t2\":");
        Serial.print(recipe.ignition.ignite_t2);
        Serial.print(",\"pump_condition\":");
        Serial.print(recipe.ignition.pump_condition);
        Serial.print(",\"pump_power\":");
        Serial.print(recipe.ignition.pump_power);
        Serial.print(",\"reignite_t1\":");
        Serial.print(recipe.ignition.reignite_t1);
        Serial.print(",\"reignite_t2\":");
        Serial.print(recipe.ignition.reignite_t2);
        Serial.print(",\"pump_on_sec\":");
        Serial.print(recipe.ignition.pump_on_sec);
        Serial.print(",\"pump_off_sec\":");
        Serial.print(recipe.ignition.pump_off_sec);
        Serial.print("},");

        Serial.print("{\"stage_no\":1");
        Serial.print(",\"cook_minutes\":");
        Serial.print(recipe.cooking.cook_minutes);
        Serial.print(",\"oven_min\":");
        Serial.print(recipe.cooking.oven_min);
        Serial.print(",\"oven_max\":");
        Serial.print(recipe.cooking.oven_max);
        Serial.print(",\"oven_error_pct\":");
        Serial.print(recipe.cooking.oven_error_pct);
        Serial.print(",\"heater_on_sec\":");
        Serial.print(recipe.cooking.heater_on_sec);
        Serial.print(",\"heater_off_sec\":");
        Serial.print(recipe.cooking.heater_off_sec);
        Serial.print(",\"fan_on_sec\":");
        Serial.print(recipe.cooking.fan_on_sec);
        Serial.print(",\"fan_off_sec\":");
        Serial.print(recipe.cooking.fan_off_sec);
        Serial.print(",\"spray_time_sec\":");
        Serial.print(recipe.cooking.spray_time_sec);
        Serial.print(",\"spray_power\":");
        Serial.print(recipe.cooking.spray_power);
        Serial.print("},");

        Serial.print("{\"stage_no\":2");
        Serial.print(",\"dry_minutes\":");
        Serial.print(recipe.drying.dry_minutes);
        Serial.print(",\"oven_min\":");
        Serial.print(recipe.drying.oven_min);
        Serial.print(",\"oven_max\":");
        Serial.print(recipe.drying.oven_max);
        Serial.print(",\"oven_error_pct\":");
        Serial.print(recipe.drying.oven_error_pct);
        Serial.print(",\"heater_on_sec\":");
        Serial.print(recipe.drying.heater_on_sec);
        Serial.print(",\"heater_off_sec\":");
        Serial.print(recipe.drying.heater_off_sec);
        Serial.print("},");

        Serial.print("{\"stage_no\":3");
        Serial.print(",\"hold_minutes\":");
        Serial.print(recipe.holding.hold_minutes);
        Serial.print(",\"oven_min\":");
        Serial.print(recipe.holding.oven_min);
        Serial.print(",\"oven_max\":");
        Serial.print(recipe.holding.oven_max);
        Serial.print(",\"oven_error_pct\":");
        Serial.print(recipe.holding.oven_error_pct);
        Serial.print(",\"heater_on_sec\":");
        Serial.print(recipe.holding.heater_on_sec);
        Serial.print(",\"heater_off_sec\":");
        Serial.print(recipe.holding.heater_off_sec);
        Serial.println("}]}");

        Serial.printf("[UART-CMD] run(recipe-select) -> recipe=%u\n", (unsigned)recipeNo);

        // smoke_enable=1이면 FIRE 점화 시퀀스로, 0이면 COOK으로 바로 진입
        if (recipe.ignition.smoke_enable != 0) {
            setState((uint16_t)FireSubState::IGNITION);
            Serial.println("[UART-CMD] Auto-transition to FIRE_IGNITION (smoke_enable=1)");
        } else {
            setState((uint16_t)HeatSubState::RAMP_UP);
            Serial.println("[UART-CMD] Auto-transition to HEAT_RAMP_UP (smoke_enable=0)");
        }
        algoRuntime.bypassTcOverMaxSafetyOnce = true;
        return true;
    }

    uint16_t targetState = 0;
    const char* targetName = "";

    switch (cmd.args[0]) {
        case 0: // START
            targetState = (uint16_t)StartSubState::INIT;
            targetName = "START_INIT";
            break;
        case 1: // FIRE (legacy compatible)
            targetState = (uint16_t)FireSubState::IGNITION;
            targetName = "FIRE_IGNITION";
            break;
        case 2: // IDLE
            targetState = (uint16_t)IdleSubState::WAIT;
            targetName = "IDLE_WAIT";
            break;
        case 3: // HEAT
            targetState = (uint16_t)HeatSubState::RAMP_UP;
            targetName = "HEAT_RAMP_UP";
            break;
        case 4: // COOK
            targetState = (uint16_t)CookSubState::START;
            targetName = "COOK_START";
            break;
        case 5: // RAP
            targetState = (uint16_t)RapSubState::START;
            targetName = "RAP_START";
            break;
        case 6: // DRY
            targetState = (uint16_t)DrySubState::START;
            targetName = "DRY_START";
            break;
        case 7: // RESET
            targetState = (uint16_t)ResetSubState::START;
            targetName = "RESET_START";
            break;
        case 8: // OIL
            targetState = (uint16_t)OilSubState::PREHEAT;
            targetName = "OIL_PREHEAT";
            break;
        default:
            Serial.printf("[UART-CMD] Unknown run target: %ld, hour=%ld, minute=%ld\n",
                          (long)cmd.args[0],
                          (long)algoRuntime.runCommandReservedArg1,
                          (long)algoRuntime.runCommandReservedArg2);
            return false;
    }

    const uint16_t targetMainState = (targetState / 1000U) * 1000U;
    const uint16_t currentMainState = (currentState / 1000U) * 1000U;
    const bool supportsInPlaceTimeAdjust =
        targetMainState == (uint16_t)MainState::HEAT ||
        targetMainState == (uint16_t)MainState::COOK ||
        targetMainState == (uint16_t)MainState::RAP ||
        targetMainState == (uint16_t)MainState::DRY ||
        targetMainState == (uint16_t)MainState::OIL;

    if (!(supportsInPlaceTimeAdjust && currentMainState == targetMainState)) {
        setState(targetState);
    }

    // Manual run command can clear one-cycle TC over-max lock so requested state can be entered.
    algoRuntime.bypassTcOverMaxSafetyOnce = true;

    // run arg1/arg2 are interpreted as hour/minute for algorithm-time injection.
    applyRunCommandAlgoTime(algoRuntime.runCommandReservedArg1,
                            algoRuntime.runCommandReservedArg2,
                            targetMainState,
                            nowMs);

    if (supportsInPlaceTimeAdjust && currentMainState == targetMainState) {
        Serial.printf("[UART-CMD] run(time-adjust) -> %s, hour=%ld, minute=%ld\n",
                      targetName,
                      (long)algoRuntime.runCommandReservedArg1,
                      (long)algoRuntime.runCommandReservedArg2);
    } else {
        Serial.printf("[UART-CMD] run -> %s, hour=%ld, minute=%ld\n",
                      targetName,
                      (long)algoRuntime.runCommandReservedArg1,
                      (long)algoRuntime.runCommandReservedArg2);
    }
    return true;
}

bool Batagota::executeUartCommand(const UartCommand& cmd)
{
    switch (cmd.type) {
        case UartCommandType::SET:
            return executeSetCommand(cmd);
        case UartCommandType::RUN:
            return executeRunCommand(cmd);
        default:
            break;
    }
    return false;
}

bool Batagota::handleUartCommandLine(const char* line)
{
    UartCommand cmd;
    if (!parseUartCommandLine(line, cmd)) {
        return false;
    }
    return executeUartCommand(cmd);
}

void Batagota::clearFETAll()
{
    for (uint8_t i = 0; i < 8; i++) {
        setFETChannel((FETOutputChannel)i, 0);
    }
}

void Batagota::clearRelayAll()
{
    for (uint8_t i = 0; i < 8; i++) {
        setRelayChannel((RelayOutputChannel)i, 0);
    }
}

void Batagota::clearLEDAll()
{
    if (driveLedStatus != NULL) {
        for (uint8_t i = 0; i < 4; i++) {
            driveLedStatus[i] = 0;
        }
    }
}

void Batagota::clearGPOAll()
{
    for (uint8_t i = 0; i < 2; i++) {
        setGPOChannel((GPOOutputChannel)i, 0);
    }
}

void Batagota::clearInternalDACAll()
{
    for (uint8_t i = 0; i < MAX_INT_DAC_CHANNELS; i++) {
        setInternalDACChannel((InternalDACOutputChannel)i, 0.0f);
    }
}

void Batagota::clearExternalDACAll()
{
    for (uint8_t i = 0; i < MAX_EXT_DAC_CHANNELS; i++) {
        setExternalDACChannel((ExternalDACOutputChannel)i, 0);
    }
}

void Batagota::clearInternalDACChannel(InternalDACOutputChannel channel)
{
    setInternalDACChannel(channel, 0.0f);
}

void Batagota::clearExternalDACChannel(ExternalDACOutputChannel channel)
{
    setExternalDACChannel(channel, 0);
}

void Batagota::setFETChannel(FETOutputChannel channel, int value)
{
    uint8_t index = (uint8_t)channel;
    if (driveFetStatus != NULL && index < 8) {
        driveFetStatus[index] = (value != 0) ? 1 : 0;
    }
}

void Batagota::setRelayChannel(RelayOutputChannel channel, int value)
{
    uint8_t index = (uint8_t)channel;
    if (driveRelayStatus != NULL && index < 8) {
        driveRelayStatus[index] = (value != 0) ? 1 : 0;
    }
}

void Batagota::setGPOChannel(GPOOutputChannel channel, int value)
{
    uint8_t index = (uint8_t)channel;
    if (driveGpoStatus != NULL && index < 2) {
        driveGpoStatus[index] = (value != 0) ? 1 : 0;
    }
}

void Batagota::setInternalDACChannel(InternalDACOutputChannel channel, float value)
{
    uint8_t index = (uint8_t)channel;
    if (driveIntDacData != NULL && index < MAX_INT_DAC_CHANNELS) {
        driveIntDacData[index] = value;
    }
}

void Batagota::setExternalDACChannel(ExternalDACOutputChannel channel, int value)
{
    uint8_t index = (uint8_t)channel;
    if (driveExtDacData != NULL && index < MAX_EXT_DAC_CHANNELS) {
        driveExtDacData[index] = value;
    }
}

bool Batagota::areAllFETLow() const
{
    if (driveFetStatus == NULL) {
        return false;
    }

    for (uint8_t i = 0; i < 8; i++) {
        if (driveFetStatus[i] != 0) {
            return false;
        }
    }
    return true;
}

void Batagota::clearAllOutputsToLow()
{
    clearFETAll();
    clearRelayAll();
    clearLEDAll();
    clearGPOAll();
    // DAC targets are NOT cleared here: sensor-fed targets and test-loop setpoints
    // must not be corrupted by safety/reset logic.
}

void Batagota::autoControl()
{
    const unsigned long nowMs = millis();
    autoControlPelletFeed(nowMs);
    autoControlIgnitor(nowMs);
    autoControlSmokeDensity(nowMs);
}

void Batagota::autoControlPelletFeed(unsigned long nowMs)
{
    if (!algoRuntime.pelletFeedFlag) {
        return;
    }

    const unsigned long durationMs =
        (unsigned long)configManager.getPelletFeedDurationSec() * 1000UL;
    const unsigned long elapsedMs = nowMs - algoRuntime.pelletFeedStartMs;

    if (elapsedMs < durationMs) {
        setFETChannel(FETOutputChannel::FET_FAN_AIR, 1);
    } else {
        setFETChannel(FETOutputChannel::FET_FAN_AIR, 0);
        algoRuntime.pelletFeedFlag = false;
    }
}

void Batagota::autoControlIgnitor(unsigned long nowMs)
{
    const bool forceIgnitionRelayOn =
        (currentState == (uint16_t)FireSubState::IGNITION ||
         currentState == (uint16_t)FireSubState::FIRING);
    const bool smokeEnabled = (configManager.getActiveRecipeProfile().ignition.smoke_enable != 0);

    // smoke_enable==0(disable)이면 Ignitor 완전 차단
    const RecipeStageProfile& recipe = configManager.getActiveRecipeProfile();
    if (recipe.ignition.smoke_enable == 0 && !forceIgnitionRelayOn) {
        // 모든 상태에서 Ignitor OFF, 상태머신도 OFF로 강제
        algoRuntime.ignitorState = IgnitorState::OFF;
        algoRuntime.ignitorTurnOnCommand = false;
        algoRuntime.ignitorTurnOffCommand = false;
        setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 0);
        return;
    }

    // During FIRE_IGNITION/FIRE_FIRING, keep ignitor relay energized while
    // ignition is expected to run.
    if (forceIgnitionRelayOn && smokeEnabled &&
        (algoRuntime.ignitorState == IgnitorState::OFF ||
         algoRuntime.ignitorState == IgnitorState::OnGoing)) {
        setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 1);
        if (algoRuntime.ignitorState == IgnitorState::OFF) {
            algoRuntime.ignitorState = IgnitorState::OnGoing;
        }
    }

    // Early return if state is OFF and no commands pending
    if (algoRuntime.ignitorState == IgnitorState::OFF && 
        !algoRuntime.ignitorTurnOnCommand && 
        !algoRuntime.ignitorTurnOffCommand) {
        return;
    }

    // Fail-safe: without thermocouple feedback, never keep ignitor energized.
    if (driveThermocoupleData == NULL) {
        algoRuntime.ignitorState = IgnitorState::OFF;
        algoRuntime.ignitorTurnOnCommand = false;
        algoRuntime.ignitorTurnOffCommand = false;
        setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 0);
        return;
    }

    const int16_t nearIgnitorTc = driveThermocoupleData[(uint8_t)ThermocoupleChannel::TC_NEAR_IGNITOR];
    const int16_t farIgnitorTc = driveThermocoupleData[(uint8_t)ThermocoupleChannel::TC_FAR_IGNITOR];
    const float smokeDensityAdc0 = getExternalADCValue(ExternalADCInputChannel::EXT_ADC_0);

    const float igniteNearThresholdDeciC = (float)recipe.ignition.ignite_t1 * 10.0f;
    const float igniteFarThresholdDeciC = (float)recipe.ignition.ignite_t2 * 10.0f;
    const float reigniteNearThresholdDeciC = (float)recipe.ignition.reignite_t1 * 10.0f;
    const float reigniteFarThresholdDeciC = (float)recipe.ignition.reignite_t2 * 10.0f;

    // Temperature thresholds with 5% hysteresis (임계오차값)
    const float NEAR_THRESHOLD = igniteNearThresholdDeciC;
    const float FAR_THRESHOLD = igniteFarThresholdDeciC;
    const float HYSTERESIS_RATIO = 0.95f;  // 5% 임계오차 허용
    const float NEAR_THRESHOLD_WITH_HYSTERESIS = NEAR_THRESHOLD * HYSTERESIS_RATIO;
    const float FAR_THRESHOLD_WITH_HYSTERESIS = FAR_THRESHOLD * HYSTERESIS_RATIO;
    const float REIGNITE_NEAR_WITH_HYSTERESIS = reigniteNearThresholdDeciC * HYSTERESIS_RATIO;
    const float REIGNITE_FAR_WITH_HYSTERESIS = reigniteFarThresholdDeciC * HYSTERESIS_RATIO;
    
    const bool nearReady = nearIgnitorTc >= NEAR_THRESHOLD;
    const bool farReady = farIgnitorTc >= FAR_THRESHOLD;
    const bool smokeReady = (smokeDensityAdc0 >= IGNITOR_SMOKE_ADC0_READY_V);
    const bool maxOnTimedOut = (nowMs - algoRuntime.ignitorOnStartMs) >= IGNITOR_MAX_ON_MS;
    
    const bool nearHysteresisOk = nearIgnitorTc >= NEAR_THRESHOLD_WITH_HYSTERESIS;
    const bool farHysteresisOk = farIgnitorTc >= FAR_THRESHOLD_WITH_HYSTERESIS;
    const bool reigniteNearHysteresisOk = nearIgnitorTc >= REIGNITE_NEAR_WITH_HYSTERESIS;
    const bool reigniteFarHysteresisOk = farIgnitorTc >= REIGNITE_FAR_WITH_HYSTERESIS;

    // State machine based on commands and temperature conditions
    switch (algoRuntime.ignitorState) {
        case IgnitorState::OFF:
            // OFF → 점화 명령 대기
            if (algoRuntime.ignitorTurnOnCommand) {
                algoRuntime.ignitorTurnOnCommand = false;
                algoRuntime.ignitorOnStartMs = nowMs;

                if (forceIgnitionRelayOn) {
                    // IGNITION/FIRING 단계에서는 recipe smoke_enable과 무관하게 점화 릴레이를 우선 ON
                    algoRuntime.ignitorState = IgnitorState::OnGoing;
                    setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 1);
                    break;
                }
                
                // Check temperature conditions to transition to next state
                if (nearReady && !farReady) {
                    // Near ready, Far not ready → OnGoing
                    algoRuntime.ignitorState = IgnitorState::OnGoing;
                    setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 1);
                } else if (nearReady && farReady && smokeReady) {
                    // Both ready → ON (발화)
                    algoRuntime.ignitorState = IgnitorState::ON;
                    setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 1);
                } else {
                    // Temperature not ready, stay OFF
                    setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 0);
                }
            }
            break;

        case IgnitorState::OnGoing:
            // 점화 중: Near OK, Far Not Ready
            setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 1);
            
            if (algoRuntime.ignitorTurnOffCommand) {
                // 소화 명령 수신 시 OFF로 전환
                algoRuntime.ignitorTurnOffCommand = false;
                algoRuntime.ignitorState = IgnitorState::OffGoing;
            } else if (farReady && smokeReady) {
                // Far 온도 상승 → 발화 완료
                algoRuntime.ignitorState = IgnitorState::ON;
            } else if (maxOnTimedOut) {
                // 점화 최대 시간 초과 → 안전하게 OFF
                algoRuntime.ignitorState = IgnitorState::OFF;
                setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 0);
            }
            break;

        case IgnitorState::ON:
            // 발화 상태: 온도 유지
            setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 1);
            
            if (algoRuntime.ignitorTurnOffCommand) {
                // 소화 명령 수신 시 OffGoing으로 전환
                algoRuntime.ignitorTurnOffCommand = false;
                algoRuntime.ignitorState = IgnitorState::OffGoing;
            } else if (!reigniteFarHysteresisOk || !reigniteNearHysteresisOk) {
                // FAR 온도 하강 → 소화 중으로 전환
                algoRuntime.ignitorState = IgnitorState::OffGoing;
            }
            break;

        case IgnitorState::OffGoing:
            // 소화 중: FAR 온도 하강 대기
            setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 0);
            
            if (!reigniteFarHysteresisOk) {
                // FAR 온도가 충분히 내려감 → OFF 상태로 완료
                algoRuntime.ignitorState = IgnitorState::OFF;
            }
            // 다시 ON 명령이 들어와도 현재는 OFF로 완전히 전환될 때까지 대기
            break;

        default:
            // Unknown state → Safe OFF
            algoRuntime.ignitorState = IgnitorState::OFF;
            setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 0);
            break;
    }
}

void Batagota::autoControlSmokeDensity(unsigned long nowMs)
{
    (void)nowMs;

    const float smokeDensityAdc0 = getExternalADCValue(ExternalADCInputChannel::EXT_ADC_0);

    // Hysteresis avoids rapid ON/OFF chattering near threshold.
    if (smokeDensityAdc0 >= SMOKE_DENSITY_ON_THRESHOLD_V) {
        setFETChannel(FETOutputChannel::FET_FAN_SMOGE, 1);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 1);
    } else if (smokeDensityAdc0 <= SMOKE_DENSITY_OFF_THRESHOLD_V) {
        setFETChannel(FETOutputChannel::FET_FAN_SMOGE, 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 0);
    }
}

void Batagota::stopCookSprayOutput()
{
    setRelayChannel(RelayOutputChannel::DC_RLY_LIQ_PUMP, 0);
    setExternalDACChannel(ExternalDACOutputChannel::EXT_DAC_1, 0);
    cookSprayActive = false;
    cookSprayOnStartMs = 0;
}

void Batagota::applyStateDrivenExternalDacOutputs(uint16_t mainState, unsigned long nowMs)
{
    const RecipeStageProfile& recipe = configManager.getActiveRecipeProfile();

    if (mainState == (uint16_t)MainState::FIRE || mainState == (uint16_t)MainState::HEAT) {
        airFanCycleStartMs = 0;
        airFanCycleMainState = mainState;
        setExternalDACChannel(ExternalDACOutputChannel::EXT_DAC_FAN_DAC, 500);
    } else if (mainState == (uint16_t)MainState::COOK ||
               mainState == (uint16_t)MainState::RAP ||
               mainState == (uint16_t)MainState::DRY) {
        const unsigned long onMs = (unsigned long)recipe.ignition.pump_on_sec * 1000UL;
        const unsigned long offMs = (unsigned long)recipe.ignition.pump_off_sec * 1000UL;
        const unsigned long cycleMs = onMs + offMs;
        const int fanTarget = clampPercentToExtDacTarget((int)recipe.ignition.pump_power);

        if (airFanCycleStartMs == 0 || airFanCycleMainState != mainState) {
            airFanCycleStartMs = nowMs;
            airFanCycleMainState = mainState;
        }

        if (fanTarget <= 0 || onMs == 0) {
            setExternalDACChannel(ExternalDACOutputChannel::EXT_DAC_FAN_DAC, 0);
        } else if (cycleMs == 0 || offMs == 0) {
            setExternalDACChannel(ExternalDACOutputChannel::EXT_DAC_FAN_DAC, fanTarget);
        } else {
            const unsigned long elapsedInCycle = (nowMs - airFanCycleStartMs) % cycleMs;
            setExternalDACChannel(ExternalDACOutputChannel::EXT_DAC_FAN_DAC,
                                  (elapsedInCycle < onMs) ? fanTarget : 0);
        }
    } else {
        airFanCycleStartMs = 0;
        airFanCycleMainState = 0;
        setExternalDACChannel(ExternalDACOutputChannel::EXT_DAC_FAN_DAC, 0);
    }

    if (currentState != (uint16_t)CookSubState::IN_PROGRESS) {
        if (mainState != (uint16_t)MainState::COOK) {
            cookSprayCycleStartMs = 0;
        }
        stopCookSprayOutput();
        return;
    }

    const unsigned long sprayPeriodMs = 20UL * 60UL * 1000UL;
    const unsigned long sprayOnMs = (unsigned long)recipe.cooking.spray_time_sec * 1000UL;
    const int sprayTarget = clampPercentToExtDacTarget((int)recipe.cooking.spray_power);

    if (cookSprayCycleStartMs == 0) {
        cookSprayCycleStartMs = cookMainEntryMs > 0 ? cookMainEntryMs : nowMs;
    }

    if (cookSprayActive) {
        if (sprayOnMs == 0 || (nowMs - cookSprayOnStartMs) >= sprayOnMs) {
            stopCookSprayOutput();
            cookSprayCycleStartMs = nowMs;
        } else {
            setRelayChannel(RelayOutputChannel::DC_RLY_LIQ_PUMP, 1);
            setExternalDACChannel(ExternalDACOutputChannel::EXT_DAC_1, sprayTarget);
        }
        return;
    }

    if (sprayOnMs == 0 || sprayTarget <= 0) {
        stopCookSprayOutput();
        return;
    }

    if ((nowMs - cookSprayCycleStartMs) >= sprayPeriodMs) {
        cookSprayActive = true;
        cookSprayOnStartMs = nowMs;
        setRelayChannel(RelayOutputChannel::DC_RLY_LIQ_PUMP, 1);
        setExternalDACChannel(ExternalDACOutputChannel::EXT_DAC_1, sprayTarget);
    } else {
        stopCookSprayOutput();
    }
}

void Batagota::setIgnitorCommand(bool turnOn)
{
    // 점화/소화 명령 설정
    // turnOn = true: 점화 명령
    // turnOn = false: 소화 명령
    if (turnOn) {
        algoRuntime.ignitorTurnOnCommand = true;
        algoRuntime.ignitorTurnOffCommand = false;
    } else {
        algoRuntime.ignitorTurnOnCommand = false;
        algoRuntime.ignitorTurnOffCommand = true;
    }
}

uint32_t Batagota::stateUpdate(int* fetStatus, int* relayStatus, int* ledStatus, int* gpoStatus, int* gpiStatus, int16_t* thermocoupleData, float* intDacData, int* extDacData)
{
    if (!initialized) return 0;

    algoRuntime.lastErrorCode = 0;

    // Cache output buffers so every resource drive is executed only in state handlers.
    driveFetStatus = fetStatus;
    driveRelayStatus = relayStatus;
    driveLedStatus = ledStatus;
    driveGpoStatus = gpoStatus;
    driveThermocoupleData = thermocoupleData;
    driveIntDacData = intDacData;
    // Snapshot int_dac values before any state processing may zero them via clearAllOutputsToLow().
    for (uint8_t _i = 0; _i < MAX_INT_DAC_CHANNELS; _i++) {
        driveIntDacSnapshot[_i] = (intDacData != NULL) ? intDacData[_i] : 0.0f;
    }
    driveExtDacData = extDacData;
    // Snapshot ext_dac values for the same reason.
    for (uint8_t _i = 0; _i < MAX_EXT_DAC_CHANNELS; _i++) {
        driveExtDacSnapshot[_i] = (extDacData != NULL) ? extDacData[_i] : 0;
    }
    // Snapshot FET / relay / LED before any clearAllOutputsToLow() call.
    for (uint8_t _i = 0; _i < 8; _i++) {
        driveFetSnapshot[_i]   = (fetStatus   != NULL) ? fetStatus[_i]   : 0;
        driveRelaySnapshot[_i] = (relayStatus != NULL) ? relayStatus[_i] : 0;
    }
    for (uint8_t _i = 0; _i < 4; _i++) {
        driveLedSnapshot[_i] = (ledStatus != NULL) ? ledStatus[_i] : 0;
    }

    unsigned long now = millis();
    lastSensorUpdateMs = now;

    uint32_t profileErr = ensureActiveModeProfileLoaded();
    if (profileErr != 0U) {
        // Keep running on defaults, but preserve profile storage error for diagnostics.
        setStateErrorCodeDirect(profileErr);
    }

    SafetyResult safety;
    safety.errorCode = 0;
    safety.errorNumber = ErrorCode::SAFETY_OK;
    safety.emergencyStop = false;

    // In IDLE_WAIT, skip fail checks so standby state is not forced into RESET by safety errors.
    if (currentState != (uint16_t)IdleSubState::WAIT) {
        safety = runCommonSafetyCheck(now);
    }

    // Consume one-shot TC over-max bypass requested by UART run command.
    if (algoRuntime.bypassTcOverMaxSafetyOnce &&
        safety.emergencyStop &&
        safety.errorNumber == ErrorCode::SAFETY_TC_OVER_MAX) {
        algoRuntime.bypassTcOverMaxSafetyOnce = false;
        safety.emergencyStop = false;
        safety.errorCode = 0;
        safety.errorNumber = ErrorCode::SAFETY_OK;
    }

    if (safety.emergencyStop) {
        // Emergency path: force safe outputs and transition to RESET.
        clearAllOutputsToLow();
        algoRuntime.ignitorState = IgnitorState::OFF;
        algoRuntime.pelletFeedFlag = false;

        setStateErrorCodeDirect(safety.errorCode);
        setState((uint16_t)ResetSubState::START);

        // Always print JSON even on emergency so HMI stream is not interrupted.
        // err_code in the output will identify what triggered the emergency.
        if (algoRuntime.debugJsonPrintEnabled) {
            printDebugJSON(fetStatus, relayStatus, ledStatus, gpoStatus, gpiStatus, thermocoupleData, intDacData, extDacData);
        }
        return algoRuntime.lastErrorCode;
    }

    const uint16_t prevMainState = (currentState / 1000) * 1000;
    algoRuntime.stateTimeElapsed = now - stateEntryTime;
    
    // State 머무른 시간 증가 (1초마다 호출되므로 +1)
    algoRuntime.stateTimeInSeconds++;

    // 주어진 currentState에 맞는 main state 처리 함수 호출
    uint16_t mainState = (currentState / 1000) * 1000;

    switch (mainState) {
        case (uint16_t)MainState::START:
            processStartState();
            break;
        case (uint16_t)MainState::IDLE:
            processIdleState();
            break;
        case (uint16_t)MainState::FIRE:
            processFireState();
            break;
        case (uint16_t)MainState::HEAT:
            processHeatState();
            break;
        case (uint16_t)MainState::COOK:
            processCookState();
            break;
        case (uint16_t)MainState::OIL:
            processOilState();
            break;
        case (uint16_t)MainState::RAP:
            processRapState();
            break;
        case (uint16_t)MainState::DRY:
            processDryState();
            break;
        case (uint16_t)MainState::RESET:
            processResetState();
            break;
        default:
            setStateError(ERROR_NO_CHANNEL);
            setState((uint16_t)StartSubState::INIT);
            break;
    }

    const uint16_t controlledMainState = (currentState / 1000) * 1000;
    applyStateDrivenExternalDacOutputs(controlledMainState, now);

    // Apply time-based auto control before returning outputs.
    autoControl();

    const uint16_t nextMainState = (currentState / 1000) * 1000;
    if (nextMainState != lastAnnouncedMainState) {
        handleMainStateBuzzerOnTransition(prevMainState, nextMainState, now);
    }
    handleCookHourBuzzer(nextMainState, now);
    
    // Debug JSON 출력 (flag enabled only)
    if (algoRuntime.debugJsonPrintEnabled) {
        printDebugJSON(fetStatus, relayStatus, ledStatus, gpoStatus, gpiStatus, thermocoupleData, intDacData, extDacData);
    }

    return algoRuntime.lastErrorCode;
}

bool Batagota::consumePendingBuzzerCommand(uint8_t& count, uint8_t& id)
{
    if (!pendingBuzzerCommand) {
        return false;
    }

    count = pendingBuzzerCount;
    id = pendingBuzzerId;
    pendingBuzzerCommand = false;
    return true;
}

void Batagota::setState(uint16_t newState)
{
    if (currentState != newState) {
        // Debug JSON flag transition policy:
        // JSON mode is set at boot and controlled only via ba+set=16 command.
        if (newState == (uint16_t)FireSubState::IGNITION) {
            algoRuntime.fireIgnitionSequenceStarted = false;
        }

        currentState = newState;
        stateEntryTime = millis();
        algoRuntime.stateTimeElapsed = 0;
        algoRuntime.stateTimeInSeconds = 1;  // 새 state 진입 시 1초로 시작
    }
}

uint16_t Batagota::getCurrentState() const
{
    return currentState;
}

void Batagota::setStateUartLogEnabled(bool enabled)
{
    algoRuntime.stateUartLogEnabled = enabled;
}

bool Batagota::isStateUartLogEnabled() const
{
    return algoRuntime.stateUartLogEnabled;
}

void Batagota::setDebugJsonPrintEnabled(bool enabled)
{
    algoRuntime.debugJsonPrintEnabled = enabled;
}

bool Batagota::isDebugJsonPrintEnabled() const
{
    return algoRuntime.debugJsonPrintEnabled;
}

uint8_t Batagota::getIgnitorStateCode() const
{
    return (uint8_t)algoRuntime.ignitorState;
}

const char* Batagota::getIgnitorStateName() const
{
    switch (algoRuntime.ignitorState) {
        case IgnitorState::OFF:      return "OFF";
        case IgnitorState::OnGoing:  return "ONG";
        case IgnitorState::ON:       return "ON";
        case IgnitorState::OffGoing: return "OFG";
        default:                     return "UNK";
    }
}

bool Batagota::getIgnitorTurnOnCommand() const
{
    return algoRuntime.ignitorTurnOnCommand;
}

bool Batagota::getIgnitorTurnOffCommand() const
{
    return algoRuntime.ignitorTurnOffCommand;
}

void Batagota::printStateLogIfEnabled(const char* stateText) const
{
    // JSON debug log enabled 상태에서는 state text 로그를 억제한다.
    if (algoRuntime.stateUartLogEnabled && !algoRuntime.debugJsonPrintEnabled) {
        if (ErrorCode::isError(algoRuntime.lastErrorCode)) {
            Serial.printf("%s err=0x%08lX module=%u source=%u number=%u(%s)\n",
                          stateText,
                          (unsigned long)algoRuntime.lastErrorCode,
                          (unsigned)ErrorCode::module(algoRuntime.lastErrorCode),
                          (unsigned)ErrorCode::source(algoRuntime.lastErrorCode),
                          (unsigned)ErrorCode::number(algoRuntime.lastErrorCode),
                          ErrorCode::toString(ErrorCode::number(algoRuntime.lastErrorCode)));
            return;
        }
        Serial.println(stateText);
    }
}

uint32_t Batagota::getStateTimeInSeconds() const
{
    return algoRuntime.stateTimeInSeconds;
}

bool Batagota::shouldReportRemainingTime() const
{
    const uint16_t mainState = (currentState / 1000U) * 1000U;
    return (mainState == (uint16_t)MainState::COOK ||
            mainState == (uint16_t)MainState::RAP ||
            mainState == (uint16_t)MainState::DRY);
}

uint32_t Batagota::getRemainingSecondsForDisplay() const
{
    if (!shouldReportRemainingTime()) {
        return 0U;
    }

    const RecipeStageProfile& recipe = configManager.getActiveRecipeProfile();
    const uint32_t cookSec = (uint32_t)recipe.cooking.cook_minutes * 60U;
    const uint32_t drySec = (uint32_t)recipe.drying.dry_minutes * 60U;
    const uint32_t holdSec = (uint32_t)recipe.holding.hold_minutes * 60U;
    const uint16_t mainState = (currentState / 1000U) * 1000U;

    uint32_t remainSec = 0U;
    if (mainState == (uint16_t)MainState::COOK) {
        if (currentState == (uint16_t)CookSubState::HOLD) {
            const uint32_t holdElapsedSec = (uint32_t)(algoRuntime.stateTimeElapsed / 1000U);
            remainSec = (holdSec > holdElapsedSec) ? (holdSec - holdElapsedSec) : 0U;
            remainSec += drySec;
        } else {
            const unsigned long nowMs = millis();
            const uint32_t elapsedSec = (cookMainEntryMs > 0 && nowMs >= cookMainEntryMs)
                ? (uint32_t)((nowMs - cookMainEntryMs) / 1000UL)
                : 0U;
            remainSec = (cookSec > elapsedSec) ? (cookSec - elapsedSec) : 0U;
            remainSec += drySec + holdSec;
        }
    } else if (mainState == (uint16_t)MainState::RAP) {
        const unsigned long nowMs = millis();
        const uint32_t elapsedSec = (rapMainEntryMs > 0 && nowMs >= rapMainEntryMs)
            ? (uint32_t)((nowMs - rapMainEntryMs) / 1000UL)
            : 0U;
        remainSec = (holdSec > elapsedSec) ? (holdSec - elapsedSec) : 0U;
        remainSec += drySec;
    } else if (mainState == (uint16_t)MainState::DRY) {
        const unsigned long nowMs = millis();
        const uint32_t elapsedSec = (dryMainEntryMs > 0 && nowMs >= dryMainEntryMs)
            ? (uint32_t)((nowMs - dryMainEntryMs) / 1000UL)
            : 0U;
        remainSec = (drySec > elapsedSec) ? (drySec - elapsedSec) : 0U;
    }

    return remainSec;
}

const char* Batagota::getStateNameStr() const
{
    switch (currentState) {
        // START States (0-3)
        case (uint16_t)StartSubState::INIT:         return "START_INIT";
        case (uint16_t)StartSubState::CHECK_SENSOR: return "START_CHECK_SENSOR";
        case (uint16_t)StartSubState::READY:        return "START_READY";
        case (uint16_t)StartSubState::COMPLETE:     return "START_COMPLETE";

        // IDLE States (1000-1002)
        case (uint16_t)IdleSubState::WAIT:          return "IDLE_WAIT";
        case (uint16_t)IdleSubState::MONITOR:       return "IDLE_MONITOR";
        case (uint16_t)IdleSubState::SHUTDOWN:      return "IDLE_SHUTDOWN";

        // FIRE States (2000-2004)
        case (uint16_t)FireSubState::IGNITION:      return "FIRE_IGNITION";
        case (uint16_t)FireSubState::FIRING:        return "FIRE_FIRING";
        case (uint16_t)FireSubState::STABILIZE:     return "FIRE_STABILIZE";
        case (uint16_t)FireSubState::RUNNING:       return "FIRE_RUNNING";
        case (uint16_t)FireSubState::ALARM:         return "FIRE_ALARM";

        // HEAT States (3000-3003)
        case (uint16_t)HeatSubState::RAMP_UP:       return "HEAT_RAMP_UP";
        case (uint16_t)HeatSubState::STABILIZE:     return "HEAT_STABILIZE";
        case (uint16_t)HeatSubState::MAINTAIN:      return "HEAT_MAINTAIN";
        case (uint16_t)HeatSubState::ALARM:         return "HEAT_ALARM";

        // COOK States (4000-4003)
        case (uint16_t)CookSubState::START:         return "COOK_START";
        case (uint16_t)CookSubState::IN_PROGRESS:   return "COOK_IN_PROGRESS";
        case (uint16_t)CookSubState::COMPLETE:      return "COOK_COMPLETE";
        case (uint16_t)CookSubState::HOLD:          return "COOK_HOLD";

        // OIL States (5000-5003)
        case (uint16_t)OilSubState::PREHEAT:        return "OIL_PREHEAT";
        case (uint16_t)OilSubState::READY:          return "OIL_READY";
        case (uint16_t)OilSubState::COOKING:        return "OIL_COOKING";
        case (uint16_t)OilSubState::COOLING:        return "OIL_COOLING";

        // RAP States (6000-6002)
        case (uint16_t)RapSubState::START:          return "RAP_START";
        case (uint16_t)RapSubState::RUNNING:        return "RAP_RUNNING";
        case (uint16_t)RapSubState::COMPLETE:       return "RAP_COMPLETE";

        // DRY States (7000-7002)
        case (uint16_t)DrySubState::START:          return "DRY_START";
        case (uint16_t)DrySubState::RUNNING:        return "DRY_RUNNING";
        case (uint16_t)DrySubState::COMPLETE:       return "DRY_COMPLETE";

        // RESET States (8000-8002)
        case (uint16_t)ResetSubState::START:        return "RESET_START";
        case (uint16_t)ResetSubState::CLEANUP:      return "RESET_CLEANUP";
        case (uint16_t)ResetSubState::COMPLETE:     return "RESET_COMPLETE";

        default:                                      return "UNKNOWN";
    }
}

void Batagota::clearErrorHistory()
{
    for (uint8_t i = 0; i < MAX_ERROR_HISTORY; i++) {
        errorHistory[i] = 0;
    }
    errorHistoryCount = 0;
    algoRuntime.lastErrorCode = 0;
}

uint8_t Batagota::getErrorHistoryCount() const
{
    return errorHistoryCount;
}

uint32_t Batagota::getErrorHistoryCode(uint8_t index) const
{
    if (index < errorHistoryCount) {
        return errorHistory[index];
    }
    return 0;
}

// ============================================================================
// State Process Functions - Dispatchers
// ============================================================================

void Batagota::processStartState()
{
    switch (currentState) {
        case (uint16_t)StartSubState::INIT:
            processStartInit();
            currentState++;
            printStateLogIfEnabled("[START INIT]");
            break;
        case (uint16_t)StartSubState::CHECK_SENSOR:
            processStartCheckSensor();
            if (currentState == (uint16_t)StartSubState::CHECK_SENSOR && algoRuntime.stateTimeElapsed > 5000) {
                currentState++;
                stateEntryTime = millis();
                algoRuntime.stateTimeElapsed = 0;
                algoRuntime.stateTimeInSeconds = 1;
            }
            printStateLogIfEnabled("[START CHECK_SENSOR]");
            break;
        case (uint16_t)StartSubState::READY:
            processStartReady();
            printStateLogIfEnabled("[START READY]");
            break;
        case (uint16_t)StartSubState::COMPLETE:
            processStartComplete();
            printStateLogIfEnabled("[START COMPLETE]");
            break;
    }
}

void Batagota::processIdleState()
{
    switch (currentState) {
        case (uint16_t)IdleSubState::WAIT:
            processIdleWait();
            printStateLogIfEnabled("[IDLE WAIT]");
            break;
        case (uint16_t)IdleSubState::MONITOR:
            processIdleMonitor();
            printStateLogIfEnabled("[IDLE MONITOR]");
            break;
        case (uint16_t)IdleSubState::SHUTDOWN:
            processIdleShutdown();
            printStateLogIfEnabled("[IDLE SHUTDOWN]");
            break;
    }
}

void Batagota::processFireState()
{
    switch (currentState) {
        case (uint16_t)FireSubState::IGNITION:
            processFireIgnition();
            printStateLogIfEnabled("[FIRE IGNITION]");
            break;
        case (uint16_t)FireSubState::FIRING:
            processFireFiring();
            printStateLogIfEnabled("[FIRE FIRING]");
            break;
        case (uint16_t)FireSubState::STABILIZE:
            processFireStabilize();
            printStateLogIfEnabled("[FIRE STABILIZE]");
            break;
        case (uint16_t)FireSubState::RUNNING:
            processFireRunning();
            printStateLogIfEnabled("[FIRE RUNNING]");
            break;
        case (uint16_t)FireSubState::ALARM:
            processFireAlarm();
            printStateLogIfEnabled("[FIRE ALARM]");
            break;
    }
}

void Batagota::processHeatState()
{
    switch (currentState) {
        case (uint16_t)HeatSubState::RAMP_UP:
            processHeatRampUp();
            printStateLogIfEnabled("[HEAT RAMP_UP]");
            break;
        case (uint16_t)HeatSubState::STABILIZE:
            processHeatStabilize();
            printStateLogIfEnabled("[HEAT STABILIZE]");
            break;
        case (uint16_t)HeatSubState::MAINTAIN:
            processHeatMaintain();
            printStateLogIfEnabled("[HEAT MAINTAIN]");
            break;
        case (uint16_t)HeatSubState::ALARM:
            processHeatAlarm();
            printStateLogIfEnabled("[HEAT ALARM]");
            break;
    }
}

void Batagota::processCookState()
{
    switch (currentState) {
        case (uint16_t)CookSubState::START:
            processCookStart();
            break;
        case (uint16_t)CookSubState::IN_PROGRESS:
            processCookInProgress();
            break;
        case (uint16_t)CookSubState::COMPLETE:
            processCookComplete();
            printStateLogIfEnabled("[COOK COMPLETE]");
            break;
        case (uint16_t)CookSubState::HOLD:
            processCookHold();
            printStateLogIfEnabled("[COOK HOLD]");
            break;
    }
}

void Batagota::processOilState()
{
    switch (currentState) {
        case (uint16_t)OilSubState::PREHEAT:
            processOilPreheat();
            printStateLogIfEnabled("[OIL PREHEAT]");
            break;
        case (uint16_t)OilSubState::READY:
            processOilReady();
            printStateLogIfEnabled("[OIL READY]");
            break;
        case (uint16_t)OilSubState::COOKING:
            processOilCooking();
            printStateLogIfEnabled("[OIL COOKING]");
            break;
        case (uint16_t)OilSubState::COOLING:
            processOilCooling();
            printStateLogIfEnabled("[OIL COOLING]");
            break;
    }
}

void Batagota::processRapState()
{
    switch (currentState) {
        case (uint16_t)RapSubState::START:
            processRapStart();  
            printStateLogIfEnabled("[RAP START]");
            break;
        case (uint16_t)RapSubState::RUNNING:
            processRapRunning();
            printStateLogIfEnabled("[RAP RUNNING]");
            break;
        case (uint16_t)RapSubState::COMPLETE:
            processRapComplete();
            printStateLogIfEnabled("[RAP COMPLETE]");
            break;
    }
}

void Batagota::processDryState()
{
    switch (currentState) {
        case (uint16_t)DrySubState::START:
            processDryStart();  
            printStateLogIfEnabled("[DRY START]");
            break;
        case (uint16_t)DrySubState::RUNNING:
            processDryRunning();
            printStateLogIfEnabled("[DRY RUNNING]");
            break;
        case (uint16_t)DrySubState::COMPLETE:
            processDryComplete();
            printStateLogIfEnabled("[DRY COMPLETE]");
            break;
    }
}

void Batagota::processResetState()
{
    switch (currentState) {
        case (uint16_t)ResetSubState::START:
            processResetStart();
            printStateLogIfEnabled("[RESET START]");
            break;
        case (uint16_t)ResetSubState::CLEANUP:
            processResetCleanup();
            printStateLogIfEnabled("[RESET CLEANUP]");
            break;
        case (uint16_t)ResetSubState::COMPLETE:
            processResetComplete();
            printStateLogIfEnabled("[RESET COMPLETE]");
            break;
    }
}

// ============================================================================
// START State Sub-process Functions (0-3)
// ============================================================================

void Batagota::processStartInit()
{
    // START_INIT safety: force all outputs to LOW/0.
    clearAllOutputsToLow();

    if (algoRuntime.stateTimeElapsed > 1000) {
        setState((uint16_t)StartSubState::CHECK_SENSOR);
    }
}

void Batagota::processStartCheckSensor()
{
    // START_CHECK_SENSOR에서는 thermistor 범위 검사를 수행하지 않는다.
    // 상태 전이는 processStartState()의 elapsed time 조건(5초)으로 처리된다.
}

void Batagota::processStartReady()
{
    if (algoRuntime.stateTimeElapsed > 2000) {
        setState((uint16_t)StartSubState::COMPLETE);
    }
}

void Batagota::processStartComplete()
{
    setState((uint16_t)IdleSubState::WAIT);
}

// ============================================================================
// IDLE State Sub-process Functions (1000-1002)
// ============================================================================

void Batagota::processIdleWait()
{
    // Serial.println("[IDLE_WAIT] Waiting for command");
    // Ready for command - no output
}

void Batagota::processIdleMonitor()
{
    // Serial.println("[IDLE_MONITOR] Monitoring sensors");
    computeControlOutputs();
}

void Batagota::processIdleShutdown()
{
    if (algoRuntime.stateTimeElapsed > 2000) {
        setState((uint16_t)IdleSubState::WAIT);
    }
}

// ============================================================================
// FIRE State Sub-process Functions (2000-2003)
// ============================================================================

void Batagota::processFireIgnition()
{
    // FIRE_IGNITION entry policy (one-shot on state entry):
    // - Trigger pellet feed timed auto control
    // - Trigger ignitor auto control
    if (!algoRuntime.fireIgnitionSequenceStarted) {
        algoRuntime.fireIgnitionSequenceStarted = true;

        algoRuntime.pelletFeedFlag = true;
        algoRuntime.pelletFeedStartMs = millis();

        setIgnitorCommand(true);  // 점화 명령 전송
        algoRuntime.ignitorOnStartMs = millis();
    }

    // IGNITION 단계에서는 점화 릴레이/에어펌프 릴레이를 동기 ON으로 유지
    setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 1);
    setRelayChannel(RelayOutputChannel::AC_RLY_AIRPUMP, 1);

    // ignitor state가 OnGoing이 되면 FIRE_FIRING으로 전환
    if (algoRuntime.ignitorState == IgnitorState::OnGoing) {
        setState((uint16_t)FireSubState::FIRING);
        return;
    }

    // Timeout protection: if ignitor did not progress within 60s, raise error.
    if ((millis() - algoRuntime.ignitorOnStartMs) >= FIRE_IGNITION_TIMEOUT_MS) {
        algoRuntime.ignitorState = IgnitorState::OFF;
        setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 0);
        setStateError((uint16_t)RelayOutputChannel::AC_RLY_IGNITOR);
        setState((uint16_t)FireSubState::ALARM);
        return;
    }

    setExternalDACChannel(ExternalDACOutputChannel::EXT_DAC_FAN_DAC, 500);
}

void Batagota::processFireFiring()
{
        // FIRING 단계에서도 에어펌프는 계속 ON 유지
        setRelayChannel(RelayOutputChannel::AC_RLY_AIRPUMP, 1);

    // FIRE_FIRING: 실제 점화 중 상태 (OnGoing)
    // 온도가 충분히 올라가서 ON 상태(발화)가 되면 STABILIZE로 전환
    if (algoRuntime.ignitorState == IgnitorState::ON) {
        setState((uint16_t)FireSubState::STABILIZE);
        return;
    }

    // Timeout protection: if ignitor did not reach ON state within 60s, raise error.
    if ((millis() - algoRuntime.ignitorOnStartMs) >= FIRE_IGNITION_TIMEOUT_MS) {
        algoRuntime.ignitorState = IgnitorState::OFF;
        setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 0);
        setStateError((uint16_t)RelayOutputChannel::AC_RLY_IGNITOR);
        setState((uint16_t)FireSubState::ALARM);
        return;
    }

    setExternalDACChannel(ExternalDACOutputChannel::EXT_DAC_FAN_DAC, 500);
}

void Batagota::processFireStabilize()
{
    if (algoRuntime.stateTimeElapsed > 5000) {
        setState((uint16_t)FireSubState::RUNNING);
    }
}

void Batagota::processFireRunning()
{
    computeControlOutputs();
}

void Batagota::processFireAlarm()
{
}

// ============================================================================
// HEAT State Sub-process Functions (3000-3003)
// ============================================================================

void Batagota::processHeatRampUp()
{
    // 승온 목표: T3, T4가 oven_min(레시피) 이상 도달 시 HeatState 종료, CookState로 전이
    // T3: TC_FRONT_OVEN, T4: TC_BACK_OVEN (ThermocoupleChannel)
    // Heater: DC_RLY_VAL_AIR(5), DC_RLY_VAL_SMOG(6)
    const int16_t t3 = driveThermocoupleData[(uint8_t)ThermocoupleChannel::TC_FRONT_OVEN];
    const int16_t t4 = driveThermocoupleData[(uint8_t)ThermocoupleChannel::TC_BACK_OVEN];
    const float oven_min = activeModeProfile.stages[0].targetTempC; // 예시: 첫 스테이지의 목표 온도 사용

    // RAMP에서는 점화 릴레이는 OFF, 에어펌프는 ON 유지
    algoRuntime.ignitorState = IgnitorState::OFF;
    algoRuntime.ignitorTurnOnCommand = false;
    algoRuntime.ignitorTurnOffCommand = false;
    setRelayChannel(RelayOutputChannel::AC_RLY_IGNITOR, 0);
    setRelayChannel(RelayOutputChannel::AC_RLY_AIRPUMP, 1);

    setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 1);
    setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 1);

    // 두 온도 모두 oven_min 이상 도달 시 Heater OFF 및 CookState로 전이
    if (t3 >= oven_min && t4 >= oven_min) {
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 0);
        setState((uint16_t)CookSubState::START); // CookState로 전이
    }
}

void Batagota::processHeatStabilize()
{
    computeControlOutputs();
    if (algoRuntime.stateTimeElapsed > 15000) {
        setState((uint16_t)HeatSubState::MAINTAIN);
    }
}

void Batagota::processHeatMaintain()
{
    computeControlOutputs();
}

void Batagota::processHeatAlarm()
{
}

// ============================================================================
// COOK State Sub-process Functions (4000-4003)
// ============================================================================

void Batagota::processCookStart()
{
    const RecipeStageProfile& recipe = configManager.getActiveRecipeProfile();
    setRelayChannel(RelayOutputChannel::AC_RLY_AIRPUMP, recipe.ignition.smoke_enable != 0 ? 1 : 0);

    computeControlOutputs();
    if (algoRuntime.stateTimeElapsed > 2000) {
        setState((uint16_t)CookSubState::IN_PROGRESS);
    }
}

void Batagota::processCookInProgress()
{
    // --- 쿠킹 시간/온도 제어 ---
    const RecipeStageProfile& recipe = configManager.getActiveRecipeProfile();
    setRelayChannel(RelayOutputChannel::AC_RLY_AIRPUMP, recipe.ignition.smoke_enable != 0 ? 1 : 0);

    unsigned long now = millis();
    unsigned long elapsedMs = (cookMainEntryMs > 0 && now >= cookMainEntryMs) ?
        (now - cookMainEntryMs) : 0;

    const float oven_min = (float)recipe.cooking.oven_min;
    const float oven_max = (float)recipe.cooking.oven_max;
    const float oven_center = (oven_min + oven_max) / 2.0f;
    const uint32_t cook_ms = (uint32_t)recipe.cooking.cook_minutes * 60UL * 1000UL;

    // T3(Front Oven) 온도
    const int16_t t3 = driveThermocoupleData[(uint8_t)ThermocoupleChannel::TC_FRONT_OVEN];

    // 히터 제어: oven_min 미만이면 ON, oven_max 초과면 OFF, 사이면 중심값에 맞춰 ON/OFF
    if (t3 < oven_min) {
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 1);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 1);
    } else if (t3 > oven_max) {
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 0);
    } else {
        // 중심값에 맞춰 ON/OFF (간단히 deadband 제어)
        static bool heaterOn = false;
        const float deadband = 1.0f; // 1도 deadband
        if (t3 < oven_center - deadband) {
            heaterOn = true;
        } else if (t3 > oven_center + deadband) {
            heaterOn = false;
        }
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, heaterOn ? 1 : 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, heaterOn ? 1 : 0);
    }

    // 쿠킹 시간 종료 시 COMPLETE로 전이
    if (elapsedMs >= cook_ms) {
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 0);
        stopCookSprayOutput();
        setState((uint16_t)CookSubState::COMPLETE);
    }
}

void Batagota::processCookComplete()
{
    const RecipeStageProfile& recipe = configManager.getActiveRecipeProfile();
    setRelayChannel(RelayOutputChannel::AC_RLY_AIRPUMP, recipe.ignition.smoke_enable != 0 ? 1 : 0);

    if (algoRuntime.stateTimeElapsed > 3000) {
        setState((uint16_t)RapSubState::START);
    }
}

void Batagota::processCookHold()
{
    const RecipeStageProfile& recipe = configManager.getActiveRecipeProfile();
    setRelayChannel(RelayOutputChannel::AC_RLY_AIRPUMP, recipe.ignition.smoke_enable != 0 ? 1 : 0);

    const float oven_min = (float)recipe.holding.oven_min;
    const float oven_max = (float)recipe.holding.oven_max;
    const float oven_center = (oven_min + oven_max) / 2.0f;
    const uint32_t holdMs = (uint32_t)recipe.holding.hold_minutes * 60UL * 1000UL;

    // T3(Front Oven) 온도
    const int16_t t3 = driveThermocoupleData[(uint8_t)ThermocoupleChannel::TC_FRONT_OVEN];

    // HOLD 단계는 holding 온도 범위를 유지한다.
    if (t3 < oven_min) {
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 1);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 1);
    } else if (t3 > oven_max) {
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 0);
    } else {
        static bool holdHeaterOn = false;
        const float deadband = 1.0f;
        if (t3 < oven_center - deadband) {
            holdHeaterOn = true;
        } else if (t3 > oven_center + deadband) {
            holdHeaterOn = false;
        }
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, holdHeaterOn ? 1 : 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, holdHeaterOn ? 1 : 0);
    }

    // HOLD 시간 만료 시 자동 OFF 후 IDLE_WAIT로 전환
    if (holdMs == 0U || algoRuntime.stateTimeElapsed >= holdMs) {
        clearAllOutputsToLow();
        stopCookSprayOutput();
        setState((uint16_t)IdleSubState::WAIT);
    }
}

// ============================================================================
// OIL State Sub-process Functions (5000-5003)
// ============================================================================

void Batagota::processOilPreheat()
{
    computeControlOutputs();
    if (algoRuntime.stateTimeElapsed > 8000) {
        setState((uint16_t)OilSubState::READY);
    }
}

void Batagota::processOilReady()
{
}

void Batagota::processOilCooking()
{
    computeControlOutputs();
}

void Batagota::processOilCooling()
{
    computeControlOutputs();
}

// ============================================================================
// RAP State Sub-process Functions (6000-6002)
// ============================================================================

void Batagota::processRapStart()
{
    computeControlOutputs();
    if (algoRuntime.stateTimeElapsed > 2000) {
        setState((uint16_t)RapSubState::RUNNING);
    }
}

void Batagota::processRapRunning()
{
    const RecipeStageProfile& recipe = configManager.getActiveRecipeProfile();
    unsigned long now = millis();
    unsigned long elapsedMs = (rapMainEntryMs > 0 && now >= rapMainEntryMs) ?
        (now - rapMainEntryMs) : 0;

    const float oven_min = (float)recipe.holding.oven_min;
    const float oven_max = (float)recipe.holding.oven_max;
    const float oven_center = (oven_min + oven_max) / 2.0f;
    const uint32_t rapMs = (uint32_t)recipe.holding.hold_minutes * 60UL * 1000UL;

    const int16_t t3 = driveThermocoupleData[(uint8_t)ThermocoupleChannel::TC_FRONT_OVEN];

    if (t3 < oven_min) {
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 1);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 1);
    } else if (t3 > oven_max) {
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 0);
    } else {
        static bool rapHeaterOn = false;
        const float deadband = 1.0f;
        if (t3 < oven_center - deadband) {
            rapHeaterOn = true;
        } else if (t3 > oven_center + deadband) {
            rapHeaterOn = false;
        }
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, rapHeaterOn ? 1 : 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, rapHeaterOn ? 1 : 0);
    }

    if (rapMs == 0U || elapsedMs >= rapMs) {
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 0);
        setState((uint16_t)RapSubState::COMPLETE);
    }
}

void Batagota::processRapComplete()
{
    if (algoRuntime.stateTimeElapsed > 1000) {
        setState((uint16_t)DrySubState::START);
    }
}

// ============================================================================
// DRY State Sub-process Functions (7000-7002)
// ============================================================================

void Batagota::processDryStart()
{
    computeControlOutputs();
    if (algoRuntime.stateTimeElapsed > 2000) {
        setState((uint16_t)DrySubState::RUNNING);
    }
}

void Batagota::processDryRunning()
{
    const RecipeStageProfile& recipe = configManager.getActiveRecipeProfile();
    unsigned long now = millis();
    unsigned long elapsedMs = (dryMainEntryMs > 0 && now >= dryMainEntryMs) ?
        (now - dryMainEntryMs) : 0;

    const float oven_min = (float)recipe.drying.oven_min;
    const float oven_max = (float)recipe.drying.oven_max;
    const float oven_center = (oven_min + oven_max) / 2.0f;
    const uint32_t dryMs = (uint32_t)recipe.drying.dry_minutes * 60UL * 1000UL;

    const int16_t t3 = driveThermocoupleData[(uint8_t)ThermocoupleChannel::TC_FRONT_OVEN];

    if (t3 < oven_min) {
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 1);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 1);
    } else if (t3 > oven_max) {
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 0);
    } else {
        static bool dryHeaterOn = false;
        const float deadband = 1.0f;
        if (t3 < oven_center - deadband) {
            dryHeaterOn = true;
        } else if (t3 > oven_center + deadband) {
            dryHeaterOn = false;
        }
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, dryHeaterOn ? 1 : 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, dryHeaterOn ? 1 : 0);
    }

    if (dryMs == 0U || elapsedMs >= dryMs) {
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_1, 0);
        setRelayChannel(RelayOutputChannel::AC_RLY_HEATER_2, 0);
        setState((uint16_t)DrySubState::COMPLETE);
    }
}

void Batagota::processDryComplete()
{
    if (algoRuntime.stateTimeElapsed > 1000) {
        clearAllOutputsToLow();
        stopCookSprayOutput();
        setState((uint16_t)IdleSubState::WAIT);
    }
}

// ============================================================================
// RESET State Sub-process Functions (8000-8002)
// ============================================================================

void Batagota::processResetStart()
{
    // RESET_START safety: force all outputs to LOW/0.
    clearAllOutputsToLow();
    algoRuntime.ignitorState = IgnitorState::OFF;  // 리셋 시 점화 상태 OFF

    // Reset low-hold timers when entering RESET flow.
    algoRuntime.resetCleanupFet1LowStart = 0;
    algoRuntime.resetCleanupFet2LowStart = 0;

    if (algoRuntime.stateTimeElapsed > 0) {
        setState((uint16_t)ResetSubState::CLEANUP);
    }
}

void Batagota::processResetCleanup()
{
    const float MQ2_MQ7_ON_THRESHOLD = 0.3f;
    const float MQ135_ON_THRESHOLD = 0.5f;
    const unsigned long FET_OFF_HOLD_MS = 5000;

    const float mq2 = getExternalADCValue(ExternalADCInputChannel::EXT_ADC_1);
    const float mq7 = getExternalADCValue(ExternalADCInputChannel::EXT_ADC_2);
    const float mq135 = getExternalADCValue(ExternalADCInputChannel::EXT_ADC_3);
    const unsigned long now = millis();

    const bool mq2OrMq7High = (mq2 > MQ2_MQ7_ON_THRESHOLD) || (mq7 > MQ2_MQ7_ON_THRESHOLD);
    const bool mq135High = (mq135 > MQ135_ON_THRESHOLD);

    bool fet1LowStable = false;
    bool fet2LowStable = false;

    if (mq2OrMq7High) {
        setFETChannel(FETOutputChannel::FET_FAN_AIR, 1);
        algoRuntime.resetCleanupFet1LowStart = 0;
    } else {
        if (algoRuntime.resetCleanupFet1LowStart == 0) {
            algoRuntime.resetCleanupFet1LowStart = now;
        }

        if ((now - algoRuntime.resetCleanupFet1LowStart) >= FET_OFF_HOLD_MS) {
            setFETChannel(FETOutputChannel::FET_FAN_AIR, 0);
            fet1LowStable = true;
        }
    }

    if (mq135High) {
        setFETChannel(FETOutputChannel::FET_FAN_SMOGE, 1);
        algoRuntime.resetCleanupFet2LowStart = 0;
    } else {
        if (algoRuntime.resetCleanupFet2LowStart == 0) {
            algoRuntime.resetCleanupFet2LowStart = now;
        }

        if ((now - algoRuntime.resetCleanupFet2LowStart) >= FET_OFF_HOLD_MS) {
            setFETChannel(FETOutputChannel::FET_FAN_SMOGE, 0);
            fet2LowStable = true;
        }
    }

    if (fet1LowStable && fet2LowStable && areAllFETLow()) {
        setState((uint16_t)ResetSubState::COMPLETE);
    }
}

void Batagota::processResetComplete()
{
    setState((uint16_t)IdleSubState::WAIT);
}

void Batagota::printDebugJSON(int* fetStatus, int* relayStatus, int* ledStatus, int* gpoStatus, int* gpiStatus, int16_t* thermocoupleData, float* intDacData, int* extDacData)
{
    const uint8_t errModule = ErrorCode::module(algoRuntime.lastErrorCode);
    const uint8_t errSource = ErrorCode::source(algoRuntime.lastErrorCode);
    const uint16_t errNumber = ErrorCode::number(algoRuntime.lastErrorCode);
    const RecipeStageProfile& recipe = configManager.getActiveRecipeProfile();
    const char* ignitorStateName = "UNK";
    switch (algoRuntime.ignitorState) {
        case IgnitorState::OFF:      ignitorStateName = "OFF"; break;
        case IgnitorState::OnGoing:  ignitorStateName = "ONG"; break;
        case IgnitorState::ON:       ignitorStateName = "ON";  break;
        case IgnitorState::OffGoing: ignitorStateName = "OFG"; break;
        default:                     ignitorStateName = "UNK"; break;
    }

    struct tm timeinfo;
    char rtcTime[20];

    // Avoid 5s getLocalTime blocking when WiFi is down or SNTP is not synced.
    time_t unixNow = time(nullptr);
    if (WiFi.status() == WL_CONNECTED && unixNow > 100000 && localtime_r(&unixNow, &timeinfo) != nullptr) {
        strftime(rtcTime, sizeof(rtcTime), "%Y-%m-%d %H:%M:%S", &timeinfo);
    } else {
        strcpy(rtcTime, "Time_Not_Synced");
    }

    Serial.print("{\"state\":");
    Serial.print(currentState);
    Serial.print(",\"name\":\"");
    Serial.print(getStateNameStr());
    Serial.print("\",\"time\":");
    Serial.print(algoRuntime.stateTimeInSeconds);
    Serial.print(",\"rtc_time\":\"");
    Serial.print(rtcTime);
    Serial.print("\"");

    Serial.print(",\"ig_st\":");
    Serial.print((uint8_t)algoRuntime.ignitorState);
    Serial.print(",\"ig_nm\":\"");
    Serial.print(ignitorStateName);
    Serial.print("\"");
    Serial.print(",\"ig_on\":");
    Serial.print(algoRuntime.ignitorTurnOnCommand ? 1 : 0);
    Serial.print(",\"ig_off\":");
    Serial.print(algoRuntime.ignitorTurnOffCommand ? 1 : 0);
    Serial.print(",\"ig_t1\":");
    Serial.print(recipe.ignition.ignite_t1);
    Serial.print(",\"ig_t2\":");
    Serial.print(recipe.ignition.ignite_t2);

    Serial.print(",\"adc\":[");
    for (int i = 0; i < 8; i++) {
        Serial.print(extADCChannels[i].value, 2);
        if (i < 7) Serial.print(",");
    }
    Serial.print("]");

    Serial.print(",\"temp\":[");
    for (int i = 0; i < tempCount; i++) {
        Serial.print(tempChannels[i].currentValue, 2);
        if (i < tempCount - 1) Serial.print(",");
    }
    Serial.print("]");

    Serial.print(",\"int_dac\":[");
    for (int i = 0; i < MAX_INT_DAC_CHANNELS; i++) {
        Serial.print(driveIntDacSnapshot[i], 2);
        if (i < MAX_INT_DAC_CHANNELS - 1) Serial.print(",");
    }
    Serial.print("]");

    Serial.print(",\"ext_dac\":[");
    for (int i = 0; i < MAX_EXT_DAC_CHANNELS; i++) {
        Serial.print(driveExtDacSnapshot[i]);
        if (i < MAX_EXT_DAC_CHANNELS - 1) Serial.print(",");
    }
    Serial.print("]");

    Serial.print(",\"fet\":[");
    for (int i = 0; i < 8; i++) {
        Serial.print(driveFetSnapshot[i] ? 1 : 0);
        if (i < 7) Serial.print(",");
    }
    Serial.print("]");

    Serial.print(",\"relay\":[");
    for (int i = 0; i < 8; i++) {
        Serial.print(driveRelaySnapshot[i] ? 1 : 0);
        if (i < 7) Serial.print(",");
    }
    Serial.print("]");

    Serial.print(",\"led\":[");
    for (int i = 0; i < 4; i++) {
        Serial.print(driveLedSnapshot[i] ? 1 : 0);
        if (i < 3) Serial.print(",");
    }
    Serial.print("]");

    Serial.print(",\"gpo\":[");
    for (int i = 0; i < 2; i++) {
        Serial.print((gpoStatus != NULL && gpoStatus[i]) ? 1 : 0);
        if (i < 1) Serial.print(",");
    }
    Serial.print("]");

    Serial.print(",\"gpi\":[");
    for (int i = 0; i < 2; i++) {
        Serial.print((gpiStatus != NULL && gpiStatus[i]) ? 1 : 0);
        if (i < 1) Serial.print(",");
    }
    Serial.print("]");

    // Error history (first 4 events only, persisted until clear command)
    Serial.print(",\"err_hist_count\":");
    Serial.print(errorHistoryCount);

    Serial.print(",\"err_hist_state\":[");
    for (uint8_t i = 0; i < MAX_ERROR_HISTORY; i++) {
        uint16_t stateCode = 0;
        if (i < errorHistoryCount) {
            stateCode = (uint16_t)(errorHistory[i] & 0xFFFF);
        }
        Serial.print(stateCode);
        if (i < MAX_ERROR_HISTORY - 1) Serial.print(",");
    }
    Serial.print("]");

    Serial.print(",\"err_hist_channel\":[");
    for (uint8_t i = 0; i < MAX_ERROR_HISTORY; i++) {
        uint16_t channelCode = ERROR_NO_CHANNEL;
        if (i < errorHistoryCount) {
            channelCode = (uint16_t)(errorHistory[i] >> 16);
        }
        Serial.print(channelCode);
        if (i < MAX_ERROR_HISTORY - 1) Serial.print(",");
    }
    Serial.print("]");

    // Parsed error history for unified ErrorCode format.
    Serial.print(",\"err_hist_module\":[");
    for (uint8_t i = 0; i < MAX_ERROR_HISTORY; i++) {
        uint8_t moduleCode = 0;
        if (i < errorHistoryCount) {
            moduleCode = ErrorCode::module(errorHistory[i]);
        }
        Serial.print(moduleCode);
        if (i < MAX_ERROR_HISTORY - 1) Serial.print(",");
    }
    Serial.print("]");

    Serial.print(",\"err_hist_source\":[");
    for (uint8_t i = 0; i < MAX_ERROR_HISTORY; i++) {
        uint8_t sourceCode = 0;
        if (i < errorHistoryCount) {
            sourceCode = ErrorCode::source(errorHistory[i]);
        }
        Serial.print(sourceCode);
        if (i < MAX_ERROR_HISTORY - 1) Serial.print(",");
    }
    Serial.print("]");

    Serial.print(",\"err_hist_number\":[");
    for (uint8_t i = 0; i < MAX_ERROR_HISTORY; i++) {
        uint16_t numberCode = 0;
        if (i < errorHistoryCount) {
            numberCode = ErrorCode::number(errorHistory[i]);
        }
        Serial.print(numberCode);
        if (i < MAX_ERROR_HISTORY - 1) Serial.print(",");
    }
    Serial.print("]");

    // Serial.print(",\"run_arg1\":");
    // Serial.print(algoRuntime.runCommandReservedArg1);
    // Serial.print(",\"run_arg2\":");
    // Serial.print(algoRuntime.runCommandReservedArg2);

    Serial.print(",\"err_code\":");
    Serial.print((unsigned long)algoRuntime.lastErrorCode);
    Serial.print(",\"err_module\":");
    Serial.print(errModule);
    Serial.print(",\"err_source\":");
    Serial.print(errSource);
    Serial.print(",\"err_number\":");
    Serial.print(errNumber);
    Serial.print(",\"err_name\":\"");
    Serial.print(ErrorCode::toString(errNumber));
    Serial.print("\"");

    Serial.print(",\"tc\":[");
    for (int i = 0; i < TC_CHANNEL_COUNT; i++) {
        if (thermocoupleData != NULL) {
            Serial.print(thermocoupleData[i] / 10.0, 1);
        } else {
            Serial.print(0.0, 1);
        }
        if (i < (TC_CHANNEL_COUNT - 1)) Serial.print(",");
    }
    Serial.print("]");

    // --- Cook/Rap/Dry 상태에서 remain, end 필드 추가 ---
    uint16_t mainState = (currentState / 1000) * 1000;
    if (mainState == (uint16_t)MainState::COOK || mainState == (uint16_t)MainState::RAP || mainState == (uint16_t)MainState::DRY) {
        uint32_t remainSec = 0;
        uint32_t cookSec = (uint32_t)recipe.cooking.cook_minutes * 60U;
        uint32_t drySec = (uint32_t)recipe.drying.dry_minutes * 60U;
        uint32_t holdSec = (uint32_t)recipe.holding.hold_minutes * 60U;

        if (mainState == (uint16_t)MainState::COOK) {
            if (currentState == (uint16_t)CookSubState::HOLD) {
                uint32_t holdElapsedSec = algoRuntime.stateTimeElapsed / 1000U;
                remainSec = (holdSec > holdElapsedSec) ? (holdSec - holdElapsedSec) : 0;
                remainSec += drySec;
            } else {
                unsigned long nowMs = millis();
                uint32_t elapsed = (cookMainEntryMs > 0 && nowMs >= cookMainEntryMs)
                    ? (uint32_t)((nowMs - cookMainEntryMs) / 1000UL) : 0;
                remainSec = (cookSec > elapsed) ? (cookSec - elapsed) : 0;
                remainSec += drySec + holdSec;
            }
        } else if (mainState == (uint16_t)MainState::RAP) {
            unsigned long nowMs = millis();
            uint32_t elapsed = (rapMainEntryMs > 0 && nowMs >= rapMainEntryMs)
                ? (uint32_t)((nowMs - rapMainEntryMs) / 1000UL) : 0;
            remainSec = (holdSec > elapsed) ? (holdSec - elapsed) : 0;
            remainSec += drySec;
        } else if (mainState == (uint16_t)MainState::DRY) {
            unsigned long nowMs = millis();
            uint32_t elapsed = (dryMainEntryMs > 0 && nowMs >= dryMainEntryMs)
                ? (uint32_t)((nowMs - dryMainEntryMs) / 1000UL) : 0;
            remainSec = (drySec > elapsed) ? (drySec - elapsed) : 0;
        }

        // 종료 시각 계산 (RTC 기준)
        time_t nowSec = time(NULL);
        time_t endSec = nowSec + remainSec;
        struct tm* endTm = localtime(&endSec);
        char endTimeStr[24] = "";
        if (endTm) {
            snprintf(endTimeStr, sizeof(endTimeStr), "%02d-%02d %02d:%02d:%02d",
                endTm->tm_mon + 1, endTm->tm_mday,
                endTm->tm_hour, endTm->tm_min, endTm->tm_sec);
        } else {
            strcpy(endTimeStr, "Time_Not_Synced");
        }

        // 남은 시간 (분)
        Serial.print(",\"remain\":");
        Serial.print(remainSec / 60);
        Serial.print(",\"end\":\"");
        Serial.print(endTimeStr);
        Serial.print("\"");
    } else {
        // COOK/RAP/DRY 외 상태: 0 표시
        Serial.print(",\"remain\":0");
        Serial.print(",\"end\":\"\"");
    }
        Serial.println("}");
}
