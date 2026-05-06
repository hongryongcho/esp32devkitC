#ifndef BATAGOTA_H
#define BATAGOTA_H

#include <stdint.h>
#include <string.h>

#include "ModeProfile.h"
#include "SafetySupervisor.h"

// ============================================================================
// Temperature Controller - Batagota
// 
// Simple temperature control system with ADC/DAC/Temperature channel management
// Control loop runs at 10ms, 100ms, and 1 second intervals
// ============================================================================

class Batagota {
public:
    // ==================== Channel Enumerations ====================
    
    // External ADC Input Channel (ADS1015 - 8 channels)
    enum ExternalADCInputChannel {
        EXT_ADC_0 = 0,      // External ADC Channel 0
        EXT_ADC_1 = 1,      // External ADC Channel 1
        EXT_ADC_2 = 2,      // External ADC Channel 2
        EXT_ADC_3 = 3,      // External ADC Channel 3
        EXT_ADC_4 = 4,      // External ADC Channel 4
        EXT_ADC_5 = 5,      // External ADC Channel 5
        EXT_ADC_6 = 6,      // External ADC Channel 6
        EXT_ADC_7 = 7       // External ADC Channel 7
    };
    
    // Internal Temperature Channel (receives temperature values from ADC - 8 channels)
    // Batagota uses only 6 channels (0-5), but enum defined for all 8
    enum InternalADCChannel {
        INT_TEMP_0 = 0,     // Internal Temperature Channel 0 (BBQ1)
        INT_TEMP_1 = 1,     // Internal Temperature Channel 1 (BBQ2)
        INT_TEMP_2 = 2,     // Internal Temperature Channel 2 (BBQ3)
        INT_TEMP_3 = 3,     // Internal Temperature Channel 3 (BBQ4)
        INT_TEMP_4 = 4,     // Internal Temperature Channel 4 (SMOKE_TANK)
        INT_TEMP_5 = 5,     // Internal Temperature Channel 5 (OUTSIDE)
        INT_TEMP_6 = 6,     // Internal Temperature Channel 6 (reserved)
        INT_TEMP_7 = 7      // Internal Temperature Channel 7 (reserved)
    };
    
    // Internal DAC Output Channel (GPIO25, GPIO26 - 2 channels)
    enum InternalDACOutputChannel {
        INT_DAC_GPIO25 = 0,  // Internal DAC Channel 0 (GPIO25)
        INT_DAC_GPIO26 = 1   // Internal DAC Channel 1 (GPIO26)
    };
    
    // External DAC Output Channel (DAC7678 - 8 channels)
    enum ExternalDACOutputChannel {
        EXT_DAC_FAN_DAC = 1,      // External DAC Channel 1 (algorithm fan output)
        EXT_DAC_1 = 2,            // External DAC Channel 2 (algorithm spray output)
        EXT_DAC_2 = 3,            // External DAC Channel 3
        EXT_DAC_3 = 4,            // External DAC Channel 4
        EXT_DAC_4 = 5,            // External DAC Channel 5
        EXT_DAC_5 = 6,            // External DAC Channel 6
        EXT_DAC_6 = 7,            // External DAC Channel 7
        EXT_DAC_7 = 0             // External DAC Channel 0 (manual/external use)
    };

    // FET Output Channel (TCA9534 FET outputs, hardware index mapping)
    enum FETOutputChannel {
        FET_FAN_AIR = 0,        // Hardware FET channel 1 -> index 0
        FET_FAN_SMOGE = 1,      // Hardware FET channel 2 -> index 1
        FET_CH3 = 2,
        FET_CH4 = 3,
        FET_CH5 = 4,
        FET_CH6 = 5,
        FET_CH7 = 6,
        FET_CH8 = 7
    };

    // Relay/SSR Output Channel (8 channels, 사용자 정의 순서)
    enum RelayOutputChannel {
        AC_RLY_HEATER_1 = 0,    // ch1: CookState Heater 1
        AC_RLY_HEATER_2 = 1,    // ch2: CookState Heater 2
        AC_RLY_IGNITOR = 2,     // ch3: 점화용 AC
        AC_RLY_AIRPUMP = 3,     // ch4: 점화용 Air Pump
        DC_RLY_AIR = 4,         // ch5: Oven Smoge 배출 Air Pump
        DC_RLY_LIQ_PUMP = 5,    // ch6: CookState 스프레이용
        DC_RLY_FAN = 6,         // ch7: 외부 공기순환 Fan
        DC_RLY_TEST = 7         // ch8: 테스트용
    };

    // GPO Output Channel (2 channels)
    enum GPOOutputChannel {
        GPO_CH1 = 0,
        GPO_CH2 = 1
    };

    // Thermocouple Channel (4 channels)
    enum ThermocoupleChannel {
        TC_NEAR_IGNITOR = 0,
        TC_FAR_IGNITOR = 1,
        TC_FRONT_OVEN = 2,
        TC_BACK_OVEN = 3,
        TC_CHANNEL_COUNT = 4
    };
    
    // BBQ Temperature Channel (maps to InternalADCChannel - 6 channels used)
    enum BBQTemperatureChannel {
        TR_BBQ1 = 0,           // BBQ 1번 (INT_TEMP_0)
        TR_BBQ2 = 1,           // BBQ 2번 (INT_TEMP_1)
        TR_BBQ3 = 2,           // BBQ 3번 (INT_TEMP_2)
        TR_BBQ4 = 3,           // BBQ 4번 (INT_TEMP_3)
        TR_SMOKE_TANK = 4,     // 스모크 탱크 (INT_TEMP_4)
        TR_OUTSIDE = 5         // 외부 온도 (INT_TEMP_5)
    };
    
    // Ignitor Control State (for autoControlIgnitor state machine)
    enum class IgnitorState : uint8_t {
        OFF = 0,        // 소화 상태
        OnGoing = 1,    // 점화 중 (Near OK, Far Not Ready)
        ON = 2,         // 발화 상태 (Both Near & Far Ready)
        OffGoing = 3    // 소화 중 (FAR 온도 하강)
    };
    
    // ==================== Channel Structures ====================
    
    // ADC Input Channel (with limit checking)
    struct ADCChannel {
        char name[32];          // Channel name/identifier
        float value;            // Current measurement
        float limitLow;         // Minimum acceptable ADC value
        float limitHigh;        // Maximum acceptable ADC value 
        
        ADCChannel() : value(0.0f), limitLow(0.0f), limitHigh(4095.0f) 
        { memset(name, 0, sizeof(name)); }
    };

    // DAC Output Channel
    struct DACChannel {
        char name[32];          // Channel name/identifier
        float value;            // Current output value
        
        DACChannel() : value(0.0f) { memset(name, 0, sizeof(name)); }
    };

    // Temperature Channel with Setpoint and Limits
    struct TemperatureChannel {
        char name[32];          // Channel name/identifier
        float currentValue;     // Current temperature reading
        float setpoint;         // Target temperature
        float output;           // Control output signal
        float limitLow;         // Minimum acceptable temperature
        float limitHigh;        // Maximum acceptable temperature
        
        TemperatureChannel() : currentValue(0.0f), setpoint(0.0f), output(0.0f),
                              limitLow(0.0f), limitHigh(100.0f) 
        { memset(name, 0, sizeof(name)); }
    };

    // ==================== Constructor / Destructor ====================
    
    Batagota();
    ~Batagota();

    // ==================== Initialization ====================
    
    void init();

    // ==================== Main Loop Functions ====================
    // Call these from main() at the specified intervals:
    // - timer interrupt, or
    // - check millis()/micros() and call when interval elapses
    
    void loop10mSec();        // Fast control loop (10 milliseconds)
    void loop100mSec();       // Medium loop (100 milliseconds)
    void loop1Sec();          // Slow loop (1 second)

    // ==================== Data Update Methods ====================
    // (called from main when new sensor readings are available)
    
    // External ADC Update (ADS1015 - 8 channels)
    void setExternalADCValue(ExternalADCInputChannel channel, float value);
    void setADCLimits(ExternalADCInputChannel channel, float limitLow, float limitHigh);
    
    // Internal Temperature Update (receives temperature values from ADC - 8 channels)
    void setInternalTemperatureValue(InternalADCChannel channel, float value);
    void setInternalTemperatureSetpoint(InternalADCChannel channel, float setpoint);
    void setTemperatureLimits(InternalADCChannel channel, float limitLow, float limitHigh);
    
    // Internal DAC Update (GPIO25, GPIO26 - 2 channels)
    void setInternalDACValue(InternalDACOutputChannel channel, float value);
    
    // External DAC Update (DAC7678 - 8 channels)
    void setExternalDACValue(ExternalDACOutputChannel channel, float value);

    // ==================== Data Read Methods ====================
    
    // External ADC Read (ADS1015 - 8 channels)
    float getExternalADCValue(ExternalADCInputChannel channel) const;
    
    // ADC Range Error Check
    bool checkADCError(ExternalADCInputChannel channel) const;  // true if within limits, false if out of range
    float getADCLimitLow(ExternalADCInputChannel channel) const;
    float getADCLimitHigh(ExternalADCInputChannel channel) const;
    
    // Internal Temperature Read (temperature values - 8 channels, 6 used)
    float getInternalTemperatureValue(InternalADCChannel channel) const;
    float getInternalTemperatureSetpoint(InternalADCChannel channel) const;
    float getInternalControlOutput(InternalADCChannel channel) const;
    
    // Temperature Range Error Check
    bool checkTemperatureError(InternalADCChannel channel) const;  // true if within limits, false if out of range
    bool checkBBQTemperatureError(BBQTemperatureChannel channel) const;  // true if within limits, false if out of range
    
    // Internal DAC Read (GPIO25, GPIO26 - 2 channels)
    float getInternalDACValue(InternalDACOutputChannel channel) const;
    
    // External DAC Read (DAC7678 - 8 channels)
    float getExternalDACValue(ExternalDACOutputChannel channel) const;
    
    // ==================== Helper Methods for Temperature Index Access ====================
    // Convert numeric index to InternalADCChannel access
    float getTemperatureByIndex(uint8_t index) const;
    float getSetpointByIndex(uint8_t index) const;
    float getControlOutputByIndex(uint8_t index) const;
    bool checkTemperatureErrorByIndex(uint8_t index) const;  // true if within limits, false if out of range
    
    // Get temperature limits
    float getTemperatureLimitLow(InternalADCChannel channel) const;
    float getTemperatureLimitHigh(InternalADCChannel channel) const;
    
    // ADC error check by index
    bool checkADCErrorByIndex(uint8_t index) const;  // true if within limits, false if out of range
    
    // ==================== BBQ Temperature Sensor Access ====================
    // Get temperature by BBQ channel name
    float getBBQTemperature(BBQTemperatureChannel channel) const;
    float getBBQSetpoint(BBQTemperatureChannel channel) const;
    float getBBQControlOutput(BBQTemperatureChannel channel) const;
    void setBBQSetpoint(BBQTemperatureChannel channel, float setpoint);
    void setBBQTemperatureLimits(BBQTemperatureChannel channel, float limitLow, float limitHigh);
    bool getBBQTemperatureLimitStatus(BBQTemperatureChannel channel) const;  // true if within limits
    
    // Helper function to get channel name by enum
    const char* getBBQChannelName(BBQTemperatureChannel channel) const;

    // ==================== Channel Query ====================
    
    uint8_t getExternalADCChannelCount() const { return MAX_EXT_ADC_CHANNELS; }
    uint8_t getInternalADCChannelCount() const { return MAX_INT_ADC_CHANNELS; }
    uint8_t getInternalDACChannelCount() const { return MAX_INT_DAC_CHANNELS; }
    uint8_t getExternalDACChannelCount() const { return MAX_EXT_DAC_CHANNELS; }
    uint8_t getTemperatureChannelCount() const { return MAX_TEMPERATURE_CHANNELS; }
    
    const char* getInternalTemperatureChannelName(InternalADCChannel channel) const;
    const char* getTemperatureChannelName(uint8_t index) const;
    const char* getExternalADCChannelName(ExternalADCInputChannel channel) const;
    const char* getThermocoupleChannelName(ThermocoupleChannel channel) const;

    // ==================== Status ====================
    
    bool isInitialized() const { return initialized; }

    // External ADC array for storage
    ADCChannel extADCChannels[8];  // External ADC channels with limit checking

private:
    // ==================== Channel Configuration ====================
    
    // ADC Channel Counts
    static const uint8_t MAX_EXT_ADC_CHANNELS = 8;   // External ADS1015 ADC channels
    static const uint8_t MAX_INT_ADC_CHANNELS = 8;   // Internal Temperature channels (6 used, 8 defined)
    
    // DAC Channel Counts
    static const uint8_t MAX_INT_DAC_CHANNELS = 2;   // Internal DAC (GPIO25, GPIO26)
    static const uint8_t MAX_EXT_DAC_CHANNELS = 8;   // External DAC7678 channels
    
    // Temperature Channel Counts
    static const uint8_t MAX_TEMPERATURE_CHANNELS = 6;  // BBQ1-4, SMOKE_TANK, OUTSIDE (using 6 of 8 INT_ADC)

    // Internal Temperature Channels (receives temperature values from ADC)
    TemperatureChannel tempChannels[MAX_INT_ADC_CHANNELS];  // 8 channels defined, 6 used

    uint8_t tempCount;

    // ==================== Status ====================
    
    bool initialized;
    uint32_t loopCounter10mSec;
    uint32_t loopCounter100mSec;
    uint32_t loopCounter1Sec;

    // ==================== Control Algorithm Methods ====================
    
    void controlAlgorithm10mSec();
    void controlAlgorithm100mSec();
    void controlAlgorithm1Sec();
    
    // Helper: Compute control outputs for all temperature channels
    void computeControlOutputs();
    
    // Debug Output Method
    void printDebugJSON(int* fetStatus, int* relayStatus, int* ledStatus, int* gpoStatus, int* gpiStatus, int16_t* thermocoupleData, float* intDacData, int* extDacData);    // JSON 형식으로 모든 채널 상태 및 현재 state 출력

    // ==================== State Machine ====================
    // Main States (using 1000-unit intervals: 0, 1000, 2000, ...)
    enum class MainState : uint16_t {
        START = 0,      // 0-999
        IDLE = 1000,    // 1000-1999
        FIRE = 2000,    // 2000-2999
        HEAT = 3000,    // 3000-3999
        COOK = 4000,    // 4000-4999
        OIL = 5000,     // 5000-5999
        RAP = 6000,     // 6000-6999
        DRY = 7000,     // 7000-7999
        RESET = 8000    // 8000-8999
    };

    // START State Sub-states (0-999 range)
    enum class StartSubState : uint16_t {
        INIT = 0,
        CHECK_SENSOR = 1,
        READY = 2,
        COMPLETE = 3
    };

    // IDLE State Sub-states (1000-1999 range)
    enum class IdleSubState : uint16_t {
        WAIT = 1000,
        MONITOR = 1001,
        SHUTDOWN = 1002
    };

    // FIRE State Sub-states (2000-2999 range)
    enum class FireSubState : uint16_t {
        IGNITION = 2000,     // 점화 명령 처리
        FIRING = 2001,       // 실제 점화 중 (OnGoing 단계)
        STABILIZE = 2002,    // 온도 회복 대기
        RUNNING = 2003,      // 정상 작동
        ALARM = 2004         // 경보
    };

    // HEAT State Sub-states (3000-3999 range)
    enum class HeatSubState : uint16_t {
        RAMP_UP = 3000,
        STABILIZE = 3001,
        MAINTAIN = 3002,
        ALARM = 3003
    };

    // COOK State Sub-states (4000-4999 range)
    enum class CookSubState : uint16_t {
        START = 4000,
        IN_PROGRESS = 4001,
        COMPLETE = 4002,
        HOLD = 4003
    };

    // OIL State Sub-states (5000-5999 range)
    enum class OilSubState : uint16_t {
        PREHEAT = 5000,
        READY = 5001,
        COOKING = 5002,
        COOLING = 5003
    };

    // RAP State Sub-states (6000-6999 range)
    enum class RapSubState : uint16_t {
        START = 6000,
        RUNNING = 6001,
        COMPLETE = 6002
    };

    // DRY State Sub-states (7000-7999 range)
    enum class DrySubState : uint16_t {
        START = 7000,
        RUNNING = 7001,
        COMPLETE = 7002
    };

    // RESET State Sub-states (8000-8999 range)
    enum class ResetSubState : uint16_t {
        START = 8000,
        CLEANUP = 8001,
        COMPLETE = 8002
    };

public:
    // State Machine Methods
    void stateInit();                          // State machine 초기화
    uint32_t stateUpdate(int* fetStatus, int* relayStatus, int* ledStatus, int* gpoStatus, int* gpiStatus, int16_t* thermocoupleData, float* intDacData, int* extDacData);  // State machine 1초마다 호출, error code 반환
    void setState(uint16_t newState);          // State 변경
    uint16_t getCurrentState() const;          // 현재 State 조회
    uint32_t getStateTimeInSeconds() const;    // 현재 State 체류 시간(초)
    const char* getStateNameStr() const;       // State 이름 조회
    bool shouldReportRemainingTime() const;    // COOK/RAP/DRY 메인 상태 여부
    uint32_t getRemainingSecondsForDisplay() const; // UART remain과 동일 계산(초)
    uint8_t getIgnitorStateCode() const;       // Ignitor 상태 코드 조회
    const char* getIgnitorStateName() const;   // Ignitor 상태 이름 조회
    bool getIgnitorTurnOnCommand() const;      // Ignitor ON 명령 플래그 조회
    bool getIgnitorTurnOffCommand() const;     // Ignitor OFF 명령 플래그 조회
    void setStateUartLogEnabled(bool enabled); // 1초 state UART 로그 출력 on/off
    bool isStateUartLogEnabled() const;        // 1초 state UART 로그 출력 상태 조회
    void setDebugJsonPrintEnabled(bool enabled); // debug JSON UART 로그 출력 on/off
    bool isDebugJsonPrintEnabled() const;        // debug JSON UART 로그 출력 상태 조회
    bool consumePendingBuzzerCommand(uint8_t& count, uint8_t& id); // 상태 변화 기반 buzzer 명령 소비
    void clearErrorHistory();                   // Error history clear
    uint8_t getErrorHistoryCount() const;       // 저장된 Error 개수
    uint32_t getErrorHistoryCode(uint8_t index) const; // index 기반 Error code 조회

    // UART command parser/dispatcher
    enum class UartCommandType : uint8_t {
        SET = 0,
        RUN = 1,
        UNKNOWN = 255
    };

    struct UartCommand {
        UartCommandType type = UartCommandType::UNKNOWN;
        int32_t args[3] = {0, 0, 0};
        uint8_t argCount = 0;
    };

    bool handleUartCommandLine(const char* line);

private:
    struct AlgorithmRuntime {
        unsigned long stateTimeElapsed = 0;         // Current state elapsed time (ms)
        uint32_t stateTimeInSeconds = 0;            // Current state residence time (sec)
        uint32_t lastErrorCode = 0;                 // [31:16]=channel, [15:0]=state
        unsigned long resetCleanupFet1LowStart = 0;
        unsigned long resetCleanupFet2LowStart = 0;
        unsigned long pelletFeedStartMs = 0;
        unsigned long ignitorOnStartMs = 0;
        bool debugJsonPrintEnabled = false;
        bool pelletFeedFlag = false;
        IgnitorState ignitorState = IgnitorState::OFF;  // Ignitor state machine
        bool ignitorTurnOnCommand = false;               // 점화 명령 플래그
        bool ignitorTurnOffCommand = false;              // 소화 명령 플래그
        bool fireIgnitionSequenceStarted = false;
        bool stateUartLogEnabled = false;
        bool bypassTcOverMaxSafetyOnce = false;     // One-shot bypass consumed by next stateUpdate safety check
        // Last received run command parameters for traceability.
        // run arg1=hour, arg2=minute
        int32_t runCommandReservedArg1 = 0;
        int32_t runCommandReservedArg2 = 0;
    };

    static const uint16_t ERROR_NO_CHANNEL = 0xFFFF;
    static const uint8_t MAX_ERROR_HISTORY = 4;

    uint16_t currentState = (uint16_t)StartSubState::INIT;  // 현재 state
    unsigned long stateEntryTime = 0;          // State 진입 시간
    AlgorithmRuntime algoRuntime;
    uint32_t errorHistory[MAX_ERROR_HISTORY] = {0, 0, 0, 0};
    uint8_t errorHistoryCount = 0;
    bool pendingBuzzerCommand = false;
    uint8_t pendingBuzzerCount = 0;
    uint8_t pendingBuzzerId = 0;
    uint16_t lastAnnouncedMainState = (uint16_t)MainState::START;
    unsigned long cookMainEntryMs = 0;
    unsigned long rapMainEntryMs = 0;
    unsigned long dryMainEntryMs = 0;
    uint32_t lastCookAnnouncedHour = 0;
    unsigned long airFanCycleStartMs = 0;
    uint16_t airFanCycleMainState = 0;
    unsigned long cookSprayCycleStartMs = 0;
    unsigned long cookSprayOnStartMs = 0;
    bool cookSprayActive = false;

    uint32_t buildStateErrorCode(uint16_t channel) const;
    void setStateError(uint16_t channel);
    bool parseUartCommandLine(const char* line, UartCommand& cmd) const;
    bool executeUartCommand(const UartCommand& cmd);
    bool executeSetCommand(const UartCommand& cmd);
    bool executeRunCommand(const UartCommand& cmd);
    void applyRunCommandAlgoTime(int32_t hourArg, int32_t minuteArg, uint16_t targetMainState, unsigned long nowMs);
    void queueBuzzerCommand(uint8_t count, uint8_t id);
    void handleMainStateBuzzerOnTransition(uint16_t prevMainState, uint16_t nextMainState, unsigned long nowMs);
    void handleCookHourBuzzer(uint16_t currentMainState, unsigned long nowMs);
    void printStateLogIfEnabled(const char* stateText) const;

    // Cached drive buffers from stateUpdate() for state-specific driving.
    int* driveFetStatus = NULL;
    int* driveRelayStatus = NULL;
    int* driveLedStatus = NULL;
    int* driveGpoStatus = NULL;
    int16_t* driveThermocoupleData = NULL;
    float* driveIntDacData = NULL;
    float  driveIntDacSnapshot[MAX_INT_DAC_CHANNELS] = {};
    int*   driveExtDacData = NULL;
    int    driveExtDacSnapshot[MAX_EXT_DAC_CHANNELS] = {};
    int    driveFetSnapshot[8]   = {};
    int    driveRelaySnapshot[8] = {};
    int    driveLedSnapshot[4]   = {};

    // Resource driving helpers (called only from state handlers).
    void clearFETAll();
    void clearRelayAll();
    void clearLEDAll();
    void clearGPOAll();
    void clearInternalDACAll();
    void clearExternalDACAll();
    void clearInternalDACChannel(InternalDACOutputChannel channel);
    void clearExternalDACChannel(ExternalDACOutputChannel channel);
    void setFETChannel(FETOutputChannel channel, int value);
    void setRelayChannel(RelayOutputChannel channel, int value);
    void setGPOChannel(GPOOutputChannel channel, int value);
    void setInternalDACChannel(InternalDACOutputChannel channel, float value);
    void setExternalDACChannel(ExternalDACOutputChannel channel, int value);
    bool areAllFETLow() const;
    void clearAllOutputsToLow();
    void autoControl();
    void autoControlPelletFeed(unsigned long nowMs);
    void autoControlIgnitor(unsigned long nowMs);
    void autoControlSmokeDensity(unsigned long nowMs);
    void applyStateDrivenExternalDacOutputs(uint16_t mainState, unsigned long nowMs);
    void stopCookSprayOutput();
    void setIgnitorCommand(bool turnOn);  // 점화/소화 명령 설정 (true=점화명령, false=소화명령)
    uint32_t ensureActiveModeProfileLoaded();
    SafetyResult runCommonSafetyCheck(unsigned long nowMs) const;
    void setStateErrorCodeDirect(uint32_t errorCode);

    // State Process Functions (각 State 처리)
    void processStartState();
    void processIdleState();
    void processFireState();
    void processHeatState();
    void processCookState();
    void processOilState();
    void processRapState();
    void processDryState();
    void processResetState();

    // Sub-state Process Functions - START (0-3)
    void processStartInit();
    void processStartCheckSensor();
    void processStartReady();
    void processStartComplete();

    // Sub-state Process Functions - IDLE (1000-1002)
    void processIdleWait();
    void processIdleMonitor();
    void processIdleShutdown();

    // Sub-state Process Functions - FIRE (2000-2004)
    void processFireIgnition();
    void processFireFiring();
    void processFireStabilize();
    void processFireRunning();
    void processFireAlarm();

    // Sub-state Process Functions - HEAT (3000-3003)
    void processHeatRampUp();
    void processHeatStabilize();
    void processHeatMaintain();
    void processHeatAlarm();

    // Sub-state Process Functions - COOK (4000-4003)
    void processCookStart();
    void processCookInProgress();
    void processCookComplete();
    void processCookHold();

    // Sub-state Process Functions - OIL (5000-5003)
    void processOilPreheat();
    void processOilReady();
    void processOilCooking();
    void processOilCooling();

    // Sub-state Process Functions - RAP (6000-6002)
    void processRapStart();
    void processRapRunning();
    void processRapComplete();

    // Sub-state Process Functions - DRY (7000-7002)
    void processDryStart();
    void processDryRunning();
    void processDryComplete();

    // Sub-state Process Functions - RESET (8000-8002)
    void processResetStart();
    void processResetCleanup();
    void processResetComplete();

    // Mode and safety supervisors
    ModeProfileStore modeProfileStore;
    ModeProfileConfig activeModeProfile;
    SafetySupervisor safetySupervisor;
    bool modeProfileStoreReady = false;
    bool activeModeProfileLoaded = false;
    uint8_t activeModeId = 0;
    unsigned long lastSensorUpdateMs = 0;
};

#endif // BATAGOTA_H
