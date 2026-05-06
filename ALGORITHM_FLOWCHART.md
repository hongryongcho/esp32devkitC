# Batagota Algorithm Flowchart

## 시스템 개요

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Batagota Control System                             │
│                                                                             │
│  INPUT SENSORS                    STATE MACHINE            OUTPUT DRIVERS   │
│  ─────────────────                ─────────────            ──────────────── │
│  [INT ADC] NTC Thermistor ──────► │ stateUpdate() │──────► FET (8ch)       │
│   - TR_BBQ1~4 (ch0~3)            │  (1sec loop)  │        - FET_FAN_AIR   │
│   - TR_SMOKE_TANK (ch4)           │               │        - FET_FAN_SMOGE │
│   - TR_OUTSIDE (ch5)              └───────────────┘       Relay/SSR (8ch)  │
│                                                            - AC_RLY_HEATER_1│
│  [EXT ADC] ADS1015 (8ch)  ─────► autoControl()           - AC_RLY_HEATER_2│
│   - EXT_ADC_0: Smoke density       (매 stateUpdate        - AC_RLY_IGNITOR │
│   - EXT_ADC_1: MQ2 (연기)          마다 호출)             Internal DAC (2ch)│
│   - EXT_ADC_2: MQ7 (CO)                                   - INT_DAC_GPIO25  │
│   - EXT_ADC_3: MQ135 (공기질)     UART Commands           - INT_DAC_GPIO26  │
│                                    ba+run=N,H,M            External DAC(8ch)│
│  [Thermocouple] MAX31856 (4ch)    ba+set=N,V,0            - EXT_DAC_FAN_DAC │
│   - TC_NEAR_IGNITOR (ch0)                                  GPO (2ch)         │
│   - TC_FAR_IGNITOR (ch1)                                   LED (4ch)         │
│   - TC_FRONT_OVEN (ch2)                                    Buzzer            │
│   - TC_BACK_OVEN (ch3)                                                       │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 메인 상태 전이 다이어그램

```
BOOT
  ↓
START(0~3) → IDLE_WAIT(1000)
  │
  ├─ ba+run=3,H,M → HEAT_RAMP_UP(3000)
  ├─ ba+run=4,H,M → COOK_START(4000)
  ├─ ba+run=5,H,M → RAP_START(6000)
  ├─ ba+run=6,H,M → DRY_START(7000)
  ├─ ba+run=7,0,0 → RESET_START(8000)
  ├─ ba+run=8,H,M → OIL_PREHEAT(5000)
  ├─ ba+run=1,recipe,0 → 레시피 선택 + COOK_START(4000)
  └─ ba+run=1,H,M(legacy) → FIRE_IGNITION(2000)

FIRE: 2000(IGNITION) → 2001(FIRING) → 2002(STABILIZE) → 2003(RUNNING)
  └─ timeout/error → 2004(ALARM)

HEAT(3000)는 TC_FRONT/TC_BACK가 목표 온도 도달 시 COOK_START(4000)로 전이
COOK(4000/4001) 완료 시 COOK_COMPLETE(4002) 3초 후 RAP_START(6000)
RAP_COMPLETE(6002) 1초 후 DRY_START(7000)
DRY_COMPLETE(7002) 1초 후 IDLE_WAIT(1000)

어느 상태에서든 safety emergency 발생 시 RESET_START(8000)로 강제 전이
```

---

## 상태별 상세 Flowchart

---

### ■ START States (0~3)

```
┌─────────────────────────────────────────────────────────────────┐
│ STATE: START_INIT [0]                                           │
│                                                                 │
│ ENTRY:                                                          │
│  ● Buzzer: beep(1, 15) — 부팅 사운드                           │
│  ● clearAllOutputsToLow() — 모든 출력 강제 OFF                 │
│                                                                 │
│ CONDITION:                                                      │
│  stateTimeElapsed > 1000ms ─► 자동 진행                        │
│                                                                 │
│ TRANSITION:                                                     │
│  ──► START_CHECK_SENSOR [1]                                     │
└─────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│ STATE: START_CHECK_SENSOR [1]                                   │
│                                                                 │
│ LOOP (매 호출마다):                                              │
│  ● tempChannels[0~5] 온도 범위 검사                             │
│    ┌─ FOR i = 0 to tempCount-1                                  │
│    │   checkTemperatureErrorByIndex(i)                          │
│    │   ├─ TRUE  (정상범위): 계속                                │
│    │   └─ FALSE (범위 초과): ERROR!                             │
│    │       setStateError(channel=i)                             │
│    │       ──► RESET_START [8000]                               │
│    └────────────────────────────                                │
│                                                                 │
│ CONDITION (정상 시):                                             │
│  stateTimeElapsed > 5000ms ─► 다음 상태로                      │
│                                                                 │
│ 검사 대상 데이터:                                                │
│  INT ADC: TR_BBQ1~4, TR_SMOKE_TANK, TR_OUTSIDE                 │
│  limitLow ~ limitHigh 범위 내 여부 확인                         │
│                                                                 │
│ TRANSITION:                                                     │
│  정상 ──► START_READY [2]                                       │
│  에러 ──► RESET_START [8000]                                    │
└─────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│ STATE: START_READY [2]                                          │
│                                                                 │
│ CONDITION:                                                      │
│  stateTimeElapsed > 2000ms                                      │
│                                                                 │
│ TRANSITION:                                                     │
│  ──► START_COMPLETE [3]                                         │
└─────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│ STATE: START_COMPLETE [3]                                       │
│                                                                 │
│ TRANSITION: (즉시)                                              │
│  ──► IDLE_WAIT [1000]                                           │
└─────────────────────────────────────────────────────────────────┘
```

---

### ■ IDLE States (1000~1002)

```
┌─────────────────────────────────────────────────────────────────┐
│ STATE: IDLE_WAIT [1000]                                         │
│                                                                 │
│ ENTRY:                                                          │
│  ● Buzzer: beep(1, 14)                                         │
│  ● debugJsonPrintEnabled = FALSE                                │
│                                                                 │
│ LOOP:                                                           │
│  ● 출력 없음 — 명령 대기                                        │
│                                                                 │
│ TRANSITION (UART 명령만):                                        │
│  ba+set=3,0,0 ──► IDLE_MONITOR [1001]                          │
│  ba+run=1,recipe,0 ──► 레시피 선택 + COOK_START [4000]         │
│  ba+run=1,H,M (legacy) ──► FIRE_IGNITION [2000]               │
│  ba+run=2,0,0 ──► IDLE_WAIT [1000] (재진입)                    │
│  ba+run=3,H,M ──► HEAT_RAMP_UP [3000]                          │
│  ba+run=4,H,M ──► COOK_START [4000]                            │
│  ba+run=5,H,M ──► RAP_START [6000]                             │
│  ba+run=6,H,M ──► DRY_START [7000]                             │
│  ba+run=7,0,0 ──► RESET_START [8000]                           │
│  ba+run=8,0,0 ──► OIL_PREHEAT [5000]                           │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ STATE: IDLE_MONITOR [1001]                                      │
│                                                                 │
│ ENTRY:                                                          │
│  ● debugJsonPrintEnabled = TRUE (JSON 로그 시작)                │
│                                                                 │
│ LOOP:                                                           │
│  ● computeControlOutputs() — 온도 제어 출력 계산                │
│    ├─ error = setpoint - currentValue                           │
│    ├─ output = error × KP (KP=0.1)                             │
│    └─ output clamp: [0.0, 1.0]                                 │
│  ● 매 stateUpdate: JSON 전체 상태 출력 (UART)                  │
│    {"state":N, "name":"...", "adc":[...], "temp":[...], ...}   │
│                                                                 │
│ 모니터링 데이터:                                                 │
│  INT ADC: TR_BBQ1~4, TR_SMOKE_TANK, TR_OUTSIDE                 │
│  EXT ADC: EXT_ADC_0~7 (MQ2, MQ7, MQ135 포함)                  │
│  TC: TC_NEAR_IGNITOR, TC_FAR_IGNITOR, TC_FRONT_OVEN, TC_BACK_OVEN│
│                                                                 │
│ TRANSITION (UART 명령):                                          │
│  ba+set=2,0,0 ──► IDLE_SHUTDOWN [1002]                         │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ STATE: IDLE_SHUTDOWN [1002]                                     │
│                                                                 │
│ CONDITION:                                                      │
│  stateTimeElapsed > 2000ms                                      │
│                                                                 │
│ TRANSITION:                                                     │
│  ──► IDLE_WAIT [1000]                                           │
│                                                                 │
│ UART 복구 명령:                                                  │
│  ba+set=3,0,0 ──► IDLE_MONITOR [1001]                          │
│  ba+set=4,0,0 ──► IDLE_WAIT [1000]                             │
└─────────────────────────────────────────────────────────────────┘
```

---

### ■ FIRE States (2000~2004)

```
┌─────────────────────────────────────────────────────────────────────────┐
│ STATE: FIRE_IGNITION [2000]                                             │
│                                                                         │
│ ENTRY (일회성, fireIgnitionSequenceStarted=false 시):                   │
│  ● fireIgnitionSequenceStarted = TRUE                                   │
│  ● pelletFeedFlag = TRUE, pelletFeedStartMs = now                       │
│    → autoControl: FET_FAN_AIR = 1 (ConfigManager.pelletFeedDurationSec초)│
│  ● setIgnitorCommand(true), ignitorOnStartMs = now                      │
│    → autoControlIgnitor() state machine 시작                            │
│  ● Buzzer: beep(100, 0)                                                 │
│                                                                         │
│ autoControlIgnitor() 핵심 조건 (매 stateUpdate):                         │
│  감시 데이터:                                                            │
│  ├─ TC_NEAR_IGNITOR, TC_FAR_IGNITOR (레시피 임계값 기반)               │
│  └─ EXT_ADC_0 (연기 밀도 V): getExternalADCValue(EXT_ADC_0) ≥ 0.30V   │
│                                                                         │
│  점화 상태 머신: OFF → OnGoing → ON → OffGoing                          │
│  (재점화 판단 시 5% hysteresis, max on 120초)                           │
│                                                                         │
│ FIRE_IGNITION 내부 타임아웃:                                             │
│  nowMs - ignitorOnStartMs ≥ 60,000ms (1분)                             │
│  → ignitorState = OFF                                                   │
│  → AC_RLY_IGNITOR = 0                                                   │
│  → setStateError(channel=AC_RLY_IGNITOR)                                │
│  ──► FIRE_ALARM [2004]                                                  │
│                                                                         │
│ 정상 동작 중:                                                            │
│  ● EXT_DAC_FAN_DAC = 500 (팬 DAC 출력)                                 │
│                                                                         │
│ TRANSITION:                                                             │
│  ignitorState == OnGoing ──► FIRE_FIRING [2001]                        │
│  타임아웃(60s)          ──► FIRE_ALARM [2004]                           │
└─────────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│ STATE: FIRE_FIRING [2001]                                       │
│                                                                 │
│ CONDITION:                                                      │
│  ignitorState == ON                                             │
│                                                                 │
│ TRANSITION:                                                     │
│  ──► FIRE_STABILIZE [2002]                                      │
│  타임아웃(60s) ──► FIRE_ALARM [2004]                            │
└─────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│ STATE: FIRE_STABILIZE [2002]                                    │
│                                                                 │
│ CONDITION:                                                      │
│  stateTimeElapsed > 5000ms                                      │
│                                                                 │
│ TRANSITION:                                                     │
│  ──► FIRE_RUNNING [2003]                                        │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ STATE: FIRE_RUNNING [2003]                                      │
│                                                                 │
│ LOOP:                                                           │
│  ● computeControlOutputs()                                     │
│    TR_BBQ1~4, TR_SMOKE_TANK: error × KP → output[0~1]         │
│                                                                 │
│ TRANSITION:                                                     │
│  UART 명령으로만 외부 상태로 전이                                │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ STATE: FIRE_ALARM [2004]                                        │
│                                                                 │
│ ● 대기 상태 (별도 처리 없음)                                     │
│ ● 에러코드 errorHistory에 기록됨                                │
│                                                                 │
│ TRANSITION:                                                     │
│  UART 명령으로만 탈출 가능                                       │
└─────────────────────────────────────────────────────────────────┘
```

---

### ■ HEAT States (3000~3003)

```
┌─────────────────────────────────────────────────────────────────┐
│ STATE: HEAT_RAMP_UP [3000]                                      │
│                                                                 │
│ ENTRY:                                                          │
│  ● Buzzer: beep(100, 14)                                        │
│                                                                 │
│ LOOP:                                                           │
│  ● TC_FRONT_OVEN(T3), TC_BACK_OVEN(T4) 모니터링                │
│  ● Relay CH0/CH1(HEATER_1/2) 강제 ON                            │
│                                                                 │
│ CONDITION:                                                      │
│  T3 >= activeModeProfile.stages[0].targetTempC                 │
│  AND T4 >= activeModeProfile.stages[0].targetTempC             │
│                                                                 │
│ TRANSITION:                                                     │
│  히터 OFF 후 ──► COOK_START [4000]                              │
└─────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│ STATE: HEAT_STABILIZE [3001]                                    │
│                                                                 │
│ LOOP: computeControlOutputs() (수동 진입 시 동작)               │
│                                                                 │
│ CONDITION:                                                      │
│  stateTimeElapsed > 15,000ms                                    │
│                                                                 │
│ TRANSITION:                                                     │
│  ──► HEAT_MAINTAIN [3002]                                       │
└─────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│ STATE: HEAT_MAINTAIN [3002]                                     │
│                                                                 │
│ LOOP: computeControlOutputs()                                   │
│                                                                 │
│ TRANSITION:                                                     │
│  UART 명령으로만 전이                                            │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ STATE: HEAT_ALARM [3003]                                        │
│ ● 대기 상태                                                      │
└─────────────────────────────────────────────────────────────────┘
```

---

### ■ COOK States (4000~4003)

```
┌─────────────────────────────────────────────────────────────────┐
│ STATE: COOK_START [4000]                                        │
│                                                                 │
│ ENTRY:                                                          │
│  ● Buzzer: beep(10, 1)                                          │
│  ● cookMainEntryMs = now                                        │
│  ● lastCookAnnouncedHour = 0                                    │
│  ● ba+run=4,H,M 시 H시간,M분 오프셋 적용                       │
│    → stateEntryTime = now - elapsedMs                           │
│    → cookMainEntryMs = now - elapsedMs                          │
│                                                                 │
│ LOOP: computeControlOutputs()                                   │
│                                                                 │
│ CONDITION:                                                      │
│  stateTimeElapsed > 2000ms                                      │
│                                                                 │
│ TRANSITION:                                                     │
│  ──► COOK_IN_PROGRESS [4001]                                    │
└─────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│ STATE: COOK_IN_PROGRESS [4001]                                  │
│                                                                 │
│ LOOP:                                                           │
│  ● 조리 시간/온도 기반 제어 (레시피 cooking.* 사용)             │
│    - t3 < oven_min: Heater1/2 ON                               │
│    - t3 > oven_max: Heater1/2 OFF                              │
│    - 구간 내: oven_center ±1.0 deadband 제어                    │
│  ● 20분 주기 분사 제어: DC_RLY_LIQ_PUMP + EXT_DAC_1            │
│                                                                 │
│ 시간 경과 Buzzer (handleCookHourBuzzer):                         │
│  ├─ 1시간 경과 → beep(10, 2)  [cookSoundId = hour+1]           │
│  ├─ 2시간 경과 → beep(10, 3)                                    │
│  ├─ ...                                                         │
│  ├─ 9시간 이상 → beep(10, 10)                                   │
│  └─ 매 정시마다 1회씩만 발생                                     │
│                                                                 │
│ TRANSITION:                                                     │
│  elapsed >= cooking.cook_minutes ──► COOK_COMPLETE [4002]       │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ STATE: COOK_COMPLETE [4002]                                     │
│                                                                 │
│ CONDITION:                                                      │
│  stateTimeElapsed > 3000ms                                      │
│                                                                 │
│ TRANSITION:                                                     │
│  ──► RAP_START [6000]                                           │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ STATE: COOK_HOLD [4003]                                         │
│                                                                 │
│ LOOP:                                                           │
│  ● holding.* 파라미터로 Heater1/2 제어                          │
│  ● hold_minutes 만료(또는 0) 시 clearAllOutputsToLow()          │
│                                                                 │
│ TRANSITION:                                                     │
│  hold 만료 시 ──► IDLE_WAIT [1000]                              │
└─────────────────────────────────────────────────────────────────┘
```

---

### ■ OIL States (5000~5003)

```
┌─────────────────────────────────────────────────────────────────┐
│ STATE: OIL_PREHEAT [5000]                                       │
│                                                                 │
│ LOOP: computeControlOutputs()                                   │
│                                                                 │
│ CONDITION:                                                      │
│  stateTimeElapsed > 8000ms                                      │
│                                                                 │
│ TRANSITION:                                                     │
│  ──► OIL_READY [5001]                                           │
└─────────────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────────────┐
│ STATE: OIL_READY   [5001] │ 대기                                │
│ STATE: OIL_COOKING [5002] │ computeControlOutputs()             │
│ STATE: OIL_COOLING [5003] │ computeControlOutputs()             │
└─────────────────────────────────────────────────────────────────┘
```

---

### ■ RAP States (6000~6002)

```
┌─────────────────────────────────────────────────────────────────┐
│ STATE: RAP_START [6000]                                         │
│                                                                 │
│ ENTRY: Buzzer: beep(10, 11)                                     │
│ LOOP: computeControlOutputs()                                   │
│                                                                 │
│ CONDITION: stateTimeElapsed > 2000ms                            │
│ TRANSITION: ──► RAP_RUNNING [6001]                              │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ STATE: RAP_RUNNING [6001]                                       │
│ LOOP:                                                           │
│  ● holding.* 파라미터로 Heater1/2 제어                          │
│  ● elapsed >= holding.hold_minutes 시 COMPLETE                  │
│ TRANSITION:                                                      │
│  elapsed 종료 ──► RAP_COMPLETE [6002]                            │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ STATE: RAP_COMPLETE [6002]                                     │
│ CONDITION: stateTimeElapsed > 1000ms                           │
│ TRANSITION: ──► DRY_START [7000]                               │
└─────────────────────────────────────────────────────────────────┘
```

---

### ■ DRY States (7000~7002)

```
┌─────────────────────────────────────────────────────────────────┐
│ STATE: DRY_START [7000]                                         │
│                                                                 │
│ ENTRY: Buzzer: beep(10, 12)                                     │
│ LOOP: computeControlOutputs()                                   │
│                                                                 │
│ CONDITION: stateTimeElapsed > 2000ms                            │
│ TRANSITION: ──► DRY_RUNNING [7001]                              │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ STATE: DRY_RUNNING [7001]                                       │
│ LOOP:                                                           │
│  ● drying.* 파라미터로 Heater1/2 제어                           │
│  ● elapsed >= drying.dry_minutes 시 COMPLETE                    │
│ TRANSITION:                                                      │
│  elapsed 종료 ──► DRY_COMPLETE [7002]                            │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ STATE: DRY_COMPLETE [7002]                                     │
│ CONDITION: stateTimeElapsed > 1000ms                           │
│ TRANSITION: clearAllOutputsToLow() 후 ──► IDLE_WAIT [1000]     │
└─────────────────────────────────────────────────────────────────┘
```

---

### ■ RESET States (8000~8002)

```
┌─────────────────────────────────────────────────────────────────┐
│ STATE: RESET_START [8000]                                       │
│                                                                 │
│ ENTRY:                                                          │
│  ● Buzzer: beep(10, 13)                                         │
│  ● clearAllOutputsToLow() — 모든 출력 강제 OFF                 │
│  ● resetCleanupFet1LowStart = 0                                 │
│  ● resetCleanupFet2LowStart = 0                                 │
│                                                                 │
│ CONDITION: stateTimeElapsed > 0 (즉시)                          │
│                                                                 │
│ TRANSITION:                                                     │
│  ──► RESET_CLEANUP [8001]                                       │
└─────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ STATE: RESET_CLEANUP [8001]                                             │
│                                                                         │
│ LOOP (매 stateUpdate):                                                  │
│                                                                         │
│ 감시 데이터:                                                             │
│  ├─ EXT_ADC_1 (MQ2):   mq2   = getExternalADCValue(EXT_ADC_1)         │
│  ├─ EXT_ADC_2 (MQ7):   mq7   = getExternalADCValue(EXT_ADC_2)         │
│  └─ EXT_ADC_3 (MQ135): mq135 = getExternalADCValue(EXT_ADC_3)         │
│                                                                         │
│ FET_CH1 (FET_FAN_AIR) 제어: ── MQ2 or MQ7 기준                        │
│  ┌─ mq2 > 0.3V OR mq7 > 0.3V                                           │
│  │   → FET_FAN_AIR = 1 (환기팬 ON)                                     │
│  │   → resetCleanupFet1LowStart 리셋                                    │
│  └─ mq2 ≤ 0.3V AND mq7 ≤ 0.3V                                         │
│      → resetCleanupFet1LowStart 시작                                    │
│      → 경과 ≥ 5,000ms → FET_FAN_AIR = 0, fet1LowStable = TRUE         │
│                                                                         │
│ FET_CH2 (FET_FAN_SMOGE) 제어: ── MQ135 기준                            │
│  ┌─ mq135 > 0.5V                                                        │
│  │   → FET_FAN_SMOGE = 1 (연기팬 ON)                                   │
│  │   → resetCleanupFet2LowStart 리셋                                    │
│  └─ mq135 ≤ 0.5V                                                        │
│      → resetCleanupFet2LowStart 시작                                    │
│      → 경과 ≥ 5,000ms → FET_FAN_SMOGE = 0, fet2LowStable = TRUE       │
│                                                                         │
│ COMPLETE 조건:                                                           │
│  fet1LowStable AND fet2LowStable AND areAllFETLow()                    │
│                                                                         │
│ TRANSITION:                                                             │
│  조건 충족 ──► RESET_COMPLETE [8002]                                    │
└─────────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│ STATE: RESET_COMPLETE [8002]                                    │
│                                                                 │
│ TRANSITION: (즉시)                                              │
│  ──► IDLE_WAIT [1000]                                           │
└─────────────────────────────────────────────────────────────────┘
```

---

## AutoControl 상시 동작 (stateUpdate 매 호출마다)

```
┌──────────────────────────────────────────────────────────────────────────┐
│ autoControl() — 상태와 무관하게 매 stateUpdate()마다 실행               │
│                                                                          │
│ ① autoControlPelletFeed()                                               │
│    pelletFeedFlag == TRUE 시:                                            │
│    ├─ 경과시간 < pelletFeedDurationSec × 1000ms                         │
│    │   → FET_FAN_AIR = 1                                                │
│    └─ 경과시간 ≥ pelletFeedDurationSec × 1000ms                         │
│        → FET_FAN_AIR = 0, pelletFeedFlag = FALSE                        │
│                                                                          │
│ ② autoControlIgnitor()                                                  │
│    smoke_enable==0 이면 즉시 OFF 강제                                    │
│    점화 상태머신(OFF/OnGoing/ON/OffGoing) 기반 릴레이 제어              │
│    임계값: ignition/reignite 레시피 값 + 5% hysteresis                  │
│    보호: thermocouple 데이터 없음 또는 max-on(120초) 초과 시 OFF         │
│                                                                          │
│ ③ autoControlSmokeDensity()                                             │
│    감시: EXT_ADC_0 (연기 밀도)                                           │
│    ┌─ ≥ 0.35V → FET_FAN_SMOGE = 1, AC_RLY_HEATER_2 = 1                 │
│    └─ ≤ 0.25V → FET_FAN_SMOGE = 0, AC_RLY_HEATER_2 = 0                 │
│    (히스테리시스: ON=0.35V, OFF=0.25V)                                   │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## UART 명령 프로토콜

```
포맷: ba+<type>=<arg0>,<arg1>,<arg2>

■ ba+run=<target>,<hour>,<minute>
  ┌─────────────────────────────────────────────────────────────────
  │ target │ 진입 상태            │ hour/minute 효과
  ├────────┼──────────────────────┼──────────────────────────────
  │   0    │ START_INIT  [0]      │ 무관
  │   1    │ 특수: recipe 선택     │ minute=0이면 recipe 선택+COOK_START
  │        │ 일반: FIRE_IGNITION   │ minute!=0이면 legacy FIRE 진입
  │   2    │ IDLE_WAIT [1000]     │ 무관
  │   3    │ HEAT_RAMP_UP [3000]  │ 알고리즘 경과시간 주입
  │   4    │ COOK_START [4000]    │ 알고리즘 경과시간 주입
  │   5    │ RAP_START [6000]     │ 알고리즘 경과시간 주입
  │   6    │ DRY_START [7000]     │ 알고리즘 경과시간 주입
  │   7    │ RESET_START [8000]   │ 무관
  │   8    │ OIL_PREHEAT [5000]   │ 알고리즘 경과시간 주입
  └─────────────────────────────────────────────────────────────────

■ ba+set=<target>,<value>,<rsv>
  ┌─────────────────────────────────────────────────────────────────
  │ target │ 조건                      │ 동작
  ├────────┼───────────────────────────┼────────────────────────
  │   2    │ IDLE_WAIT or IDLE_MONITOR │ → IDLE_SHUTDOWN [1002]
  │   3    │ IDLE_SHUTDOWN             │ → IDLE_MONITOR [1001]
  │   4    │ IDLE_SHUTDOWN             │ → IDLE_WAIT [1000]
  │  15    │ 항상                      │ stateUartLog on/off (value=1: on)
  │  16    │ 항상                      │ hmi_json on/off (value=1: on)
  └─────────────────────────────────────────────────────────────────

※ `ba+set=1/5/6/7/8/9/10/11/20/21~52`, `ba+get`, `ba+recipe` 확장 명령은
  [UART_COMMAND_PROTOCOL.md](UART_COMMAND_PROTOCOL.md), [UART_RECIPE_JSON_PROTOCOL.md](UART_RECIPE_JSON_PROTOCOL.md) 참고
```

---

## 데이터 입출력 채널 요약

```
┌───────────────────────────────────────────────────────────────────────┐
│ INPUT                                                                 │
├────────────────────────┬──────────────────────────────────────────────┤
│ INT ADC (NTC Thermistor)│ ch0: TR_BBQ1    ch1: TR_BBQ2               │
│  12V / 10K PU / 3.9K 병렬│ ch2: TR_BBQ3    ch3: TR_BBQ4              │
│  ADC1 GPIO32~36,39      │ ch4: TR_SMOKE_TANK  ch5: TR_OUTSIDE        │
├────────────────────────┬──────────────────────────────────────────────┤
│ EXT ADC (ADS1015 I2C)  │ ch0: 연기밀도(V)    ch1: MQ2(V)            │
│  0~5V 입력              │ ch2: MQ7(V)        ch3: MQ135(V)           │
│                         │ ch4~7: 예약                                 │
├────────────────────────┬──────────────────────────────────────────────┤
│ Thermocouple (MAX31856) │ ch0: TC_NEAR_IGNITOR  ch1: TC_FAR_IGNITOR  │
│  단위: 0.1°C (deciC)   │ ch2: TC_FRONT_OVEN    ch3: TC_BACK_OVEN    │
├────────────────────────┴──────────────────────────────────────────────┤
│ OUTPUT                                                                │
├────────────────────────┬──────────────────────────────────────────────┤
│ FET (TCA9534)          │ ch0: FET_FAN_AIR      ch1: FET_FAN_SMOGE   │
│  8ch, 0/1              │ ch2~7: 예약                                  │
├────────────────────────┬──────────────────────────────────────────────┤
│ Relay/SSR              │ ch0: AC_RLY_HEATER_1  ch1: AC_RLY_HEATER_2 │
│  8ch, 0/1              │ ch2: AC_RLY_IGNITOR   ch3: AC_RLY_AIRPUMP  │
│                         │ ch4: DC_RLY_AIR       ch5: DC_RLY_LIQ_PUMP │
│                         │ ch6: DC_RLY_FAN       ch7: DC_RLY_TEST     │
├────────────────────────┬──────────────────────────────────────────────┤
│ INT DAC (GPIO25/26)    │ ch0: INT_DAC_GPIO25   ch1: INT_DAC_GPIO26  │
│  UART 입력 0~4095       │                                             │
├────────────────────────┬──────────────────────────────────────────────┤
│ EXT DAC (DAC7678 I2C)  │ ch1: EXT_DAC_FAN_DAC  ch2: EXT_DAC_1(분사) │
│  UART target 0~500      │ ch0/ch3~7: 외부설정 전용                    │
└────────────────────────┴──────────────────────────────────────────────┘
```

---

## 에러 코드 구조

```
이 펌웨어는 2가지 에러코드 포맷을 사용함.

1) State-local 에러 (setStateError)
   uint32_t code = [31:16] channel | [15:0] currentState

2) Safety/Profile 에러 (ErrorCode::make)
   uint32_t code = [31:24] module | [23:16] source | [15:0] error_number

예시:
  state-local: channel=2, state=2000  -> 0x00022000
  safety: module=1, source=3, number=1002 -> 0x010303EA

최대 4개까지 errorHistory[]에 누적 보관
clearErrorHistory() 명령으로 초기화
JSON 출력: "err_hist_count", "err_hist_state[]", "err_hist_channel[]"
```

---

## 상태 번호 빠른 참조표

| State 이름         | State 번호 | 진입 조건                        | 자동 전이 조건             |
|--------------------|-----------|----------------------------------|----------------------------|
| START_INIT         | 0         | 시스템 부팅                      | elapsed > 1000ms           |
| START_CHECK_SENSOR | 1         | 자동                             | elapsed > 5000ms (정상)    |
| START_READY        | 2         | 자동                             | elapsed > 2000ms           |
| START_COMPLETE     | 3         | 자동                             | 즉시                       |
| IDLE_WAIT          | 1000      | 자동/UART                        | UART 명령                  |
| IDLE_MONITOR       | 1001      | ba+set=3                         | ba+set=2                   |
| IDLE_SHUTDOWN      | 1002      | ba+set=2                         | elapsed > 2000ms           |
| FIRE_IGNITION      | 2000      | ba+run=1(legacy)                 | ignitorState=OnGoing / 60s T.O. |
| FIRE_FIRING        | 2001      | FIRE_IGNITION                    | ignitorState=ON / 60s T.O. |
| FIRE_STABILIZE     | 2002      | FIRE_FIRING                      | elapsed > 5000ms           |
| FIRE_RUNNING       | 2003      | 자동                             | UART 명령                  |
| FIRE_ALARM         | 2004      | ignition 타임아웃                | UART 명령                  |
| HEAT_RAMP_UP       | 3000      | ba+run=3                         | T3,T4 >= targetTempC       |
| HEAT_STABILIZE     | 3001      | 수동(UART)                       | elapsed > 15000ms          |
| HEAT_MAINTAIN      | 3002      | 자동                             | UART 명령                  |
| HEAT_ALARM         | 3003      | (미구현)                         | UART 명령                  |
| COOK_START         | 4000      | ba+run=4 / ba+run=1,recipe,0     | elapsed > 2000ms           |
| COOK_IN_PROGRESS   | 4001      | 자동                             | cook_minutes 만료          |
| COOK_COMPLETE      | 4002      | COOK_IN_PROGRESS                 | elapsed > 3000ms 후 RAP    |
| COOK_HOLD          | 4003      | UART(수동)                       | hold_minutes 만료 시 IDLE  |
| OIL_PREHEAT        | 5000      | ba+run=8                         | elapsed > 8000ms           |
| OIL_READY          | 5001      | 자동                             | UART 명령                  |
| OIL_COOKING        | 5002      | UART                             | UART 명령                  |
| OIL_COOLING        | 5003      | UART                             | UART 명령                  |
| RAP_START          | 6000      | ba+run=5                         | elapsed > 2000ms           |
| RAP_RUNNING        | 6001      | 자동                             | hold_minutes 만료          |
| RAP_COMPLETE       | 6002      | RAP_RUNNING                      | elapsed > 1000ms 후 DRY    |
| DRY_START          | 7000      | ba+run=6                         | elapsed > 2000ms           |
| DRY_RUNNING        | 7001      | 자동                             | dry_minutes 만료           |
| DRY_COMPLETE       | 7002      | DRY_RUNNING                      | elapsed > 1000ms 후 IDLE   |
| RESET_START        | 8000      | ba+run=7 / 센서 에러             | elapsed > 0ms (즉시)       |
| RESET_CLEANUP      | 8001      | 자동                             | 가스센서 안정 + AllFETLow  |
| RESET_COMPLETE     | 8002      | 자동                             | 즉시                       |
