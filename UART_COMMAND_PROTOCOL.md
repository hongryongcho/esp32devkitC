# Controller-HMI Interface Protocol (UART + MQTT)

## 1. 목적과 범위
이 문서는 외부 HMI가 Controller(ESP32 firmware)와 일치된 방식으로 연동되도록,
현재 프로그램 구현 기준의 UART/MQTT 명령 프로토콜을 통합 정리한 문서입니다.

기준 코드:
- src/BOPDrv_V2p0_ESP32devkitC_ver.ino
  - handleUartReceive
  - handleCmdTopic
  - applyRecipeJsonByStage
- src/batagota.cpp
  - handleUartCommandLine
  - executeRunCommand
- src/MQTTService.cpp
  - MQTT subscribe/publish 동작

---

## 2. 핵심 정책 요약
- UART 입력은 반드시 ba+ 헤더로 시작해야 함
- UART 명령 구분자: CR 또는 LF 둘 다 허용
- UART에서 commandType(set/get/mqtt/can/recipe/기타)은 소문자 기준 처리
- MQTT 명령 payload는 JSON만 허용
- 레시피 데이터 갱신은 UART(ba+recipe)와 MQTT({"recipe":...}) 모두 동일한 필드 규칙 사용
- 레시피 조회 요청(run=1,recipe_no,0)은 현재 구현상 "응답 JSON 1회 출력"이 UART Serial로 발생함

---

## 3. UART 프로토콜

### 3.1 공통 프레임
기본 형식:
ba+<command>=<arg1>,<arg2>,<arg3>

예:
- ba+set=5,1,1
- ba+get=0,0,0
- ba+mqtt=1,1,0
- ba+can=0,1,0
- ba+run=4,2,30
- ba+recipe={...json...}

처리 정책:
1. 입력 문자열 trim
2. 소문자 기준으로 ba+ 헤더 검사
3. 헤더 불일치면 즉시 폐기
4. commandType, argPart 분리
5. 숫자형은 sscanf("%d,%d,%d") 파싱

주의:
- set는 parsed==3일 때만 동작
- get/mqtt/can은 최소 인자 개수만 충족하면 동작
- recipe는 JSON 파싱 실패/검증 실패 시 조용히 무시(별도 에러 문자열 없음)

### 3.2 ba+set=arg1,arg2,arg3
지원 arg1 목록:
- 1: DEBUG_JSON on/off
- 5: MQTT 로그 enable/disable + persist
- 6: CAN 통신 enable/disable + persist
- 7: RELAY 출력 제어
- 8: GPO 출력 제어
- 9: FET 출력 제어
- 10: 내부 DAC 제어
- 11: 외부 DAC target 제어
- 15: STATE_UART_LOG on/off
- 16: HW_SEL_IO_OUT 제어
- 20: ACTIVE_RECIPE 선택
- 21~52: RECIPE_FIELD 직접 설정

세부 규칙:
- arg1=1: ba+set=1,<0|1>,<reserved>
- arg1=5: ba+set=5,<0|1>,<persist>
- arg1=6: ba+set=6,<0|1>,<persist>
- arg1=7: ba+set=7,<ch 0~7>,<0|1>
- arg1=8: ba+set=8,<ch 0~1>,<0|1>
- arg1=9: ba+set=9,<ch 0~7>,<0|1>
- arg1=10: ba+set=10,<ch 0~1>,<level 0~4095>
- arg1=11: ba+set=11,<ch 0~7>,<target>
  - target는 내부적으로 0~500으로 clamp (0.00V~5.00V, 0.01V step)
- arg1=15: ba+set=15,<0|1>,<reserved>
- arg1=16:
  - 핀 제어: ba+set=16,<pin 0~3>,<0|1>
  - 비트필드 제어: ba+set=16,15,<0~15>
- arg1=20: ba+set=20,<recipe_index>,<reserved>
- arg1=21~52: ba+set=<field_id>,<recipe_index>,<value>

### 3.3 ba+get=arg1,arg2,arg3
지원 arg1 목록:
- 0: DEBUG_JSON/STATE_UART_LOG/MQTT/CAN 상태 일괄 출력
- 1: DEBUG_JSON 상태
- 2: HW Board ID
- 5: MQTT 상태
- 6: CAN 상태
- 7: RELAY 상태
- 8: GPO/GPI 상태
- 9: FET 상태
- 10: INT_DAC/EXT_DAC 상태
- 12: ADC 8ch
- 13: TEMP 6ch
- 14: TC 4ch
- 15: STATE_UART_LOG 상태
- 16: HW_SEL_IO 상태
- 20: RECIPE 조회
  - arg2가 유효 recipe index면 해당 레시피 조회
  - 아니면 active recipe 조회

### 3.4 ba+mqtt=arg1,arg2,arg3
- arg1: 1(enable), 0(disable)
- arg2: persist(0/1)
- arg3: reserved

예:
- ba+mqtt=1,1,0
- ba+mqtt=0,1,0

### 3.5 ba+can=arg1,arg2,arg3
- arg1: 1(enable), 0(disable)
- arg2: persist(0/1)
- arg3: reserved

예:
- ba+can=1,1,0
- ba+can=0,1,0

### 3.6 ba+recipe={json}
형식:
ba+recipe={"recipe_no":<n>,"stage_no":<s>, ...}

필수 키:
- recipe_no: 0 <= recipe_no < getRecipeCount()
- stage_no: 0~3

payload 스타일:
1. flat 스타일
- stage 필드를 루트에 직접 기입
2. data 스타일
- stage 필드를 data 객체에 기입

예:
ba+recipe={"recipe_no":1,"stage_no":0,"data":{"smoke_enable":1,"ignite_t1":120,"ignite_t2":140}}

stage_no별 필드:
- stage 0 (ignition)
  - fuel_type
  - smoke_enable
  - ignite_t1
  - ignite_t2
  - pump_condition
  - pump_power
  - reignite_t1
  - reignite_t2
  - pump_on_sec
  - pump_off_sec
- stage 1 (cooking)
  - cook_minutes
  - oven_min
  - oven_max
  - oven_error_pct
  - heater_on_sec
  - heater_off_sec
  - fan_on_sec
  - fan_off_sec
  - spray_time_sec
  - spray_power
- stage 2 (drying)
  - dry_minutes
  - oven_min
  - oven_max
  - oven_error_pct
  - heater_on_sec
  - heater_off_sec
- stage 3 (holding)
  - hold_minutes
  - oven_min
  - oven_max
  - oven_error_pct
  - heater_on_sec
  - heater_off_sec

검증:
- ConfigManager::setRecipeField + validateRecipeProfile 기준
- 예: oven_min < oven_max, spray_power <= 100, fuel_type <= 3 등

### 3.7 ba+run=arg1,arg2,arg3 (batagota 위임)
set/get/mqtt/can/recipe 외 ba+ 명령은 batagota.handleUartCommandLine로 전달됩니다.
대표적으로 run 명령이 여기에 해당합니다.

run target:
- 0: START_INIT
- 1: FIRE_IGNITION (legacy)
- 2: IDLE_WAIT
- 3: HEAT_RAMP_UP
- 4: COOK_START
- 5: RAP_START
- 6: DRY_START
- 7: RESET_START
- 8: OIL_PREHEAT

특수 동작(레시피 전송 요청):
- ba+run=1,<recipe_no>,0
- 동작 순서:
  1. active recipe를 recipe_no로 설정
  2. 레시피 전체 JSON 1회 Serial 출력
  3. smoke_enable 값에 따라 자동 상태 전이
     - smoke_enable=1: FIRE_IGNITION
     - smoke_enable=0: HEAT_RAMP_UP

주의:
- 과거 문서와 달리, 현재 구현은 recipe select 후 상태 전이가 발생함

레시피 JSON 응답 예시:
{"recipe_no":2,"stages":[{"stage_no":0,"fuel_type":0,"smoke_enable":1,"ignite_t1":120,"ignite_t2":140,"pump_condition":1,"pump_power":60,"reignite_t1":90,"reignite_t2":110,"pump_on_sec":6,"pump_off_sec":12},{"stage_no":1,"cook_minutes":480,"oven_min":105,"oven_max":125,"oven_error_pct":5,"heater_on_sec":8,"heater_off_sec":18,"fan_on_sec":20,"fan_off_sec":15,"spray_time_sec":5,"spray_power":40},{"stage_no":2,"dry_minutes":120,"oven_min":65,"oven_max":80,"oven_error_pct":5,"heater_on_sec":6,"heater_off_sec":20},{"stage_no":3,"hold_minutes":180,"oven_min":60,"oven_max":70,"oven_error_pct":4,"heater_on_sec":5,"heater_off_sec":24}]}

---

## 4. MQTT 프로토콜

### 4.1 토픽 구조
Subscribe:
- Log topic: config.mqttSubTopic
- Cmd topic: config.mqttCMDTopic

Publish:
- Status topic은 실제 publish 시 다음 포맷으로 강제 생성됨
  - BAGO/<mqttlogMode><mqttlogNumber>/Status
- 즉, publish는 config.mqttPubTopic 문자열을 그대로 사용하지 않음

### 4.2 Cmd topic JSON 명령 우선순위
handleCmdTopic에서 우선 처리되는 JSON 키:
1. set
2. run
3. recipe
4. 그 외 legacy 키(CAN_ENABLE, MQTT_LOG_ENABLE, FET, DAC7678, GPO, PWM, OP_MODE, TEMP_LIMIT_MODE, PELLET_FEED_SEC ...)

### 4.3 MQTT set 명령
형식:
{"set":[cmd,ch,val]}

현재 우선처리 지원 cmd:
- 7: RELAY ch(0~7), val(0|1)
- 8: GPO ch(0~1), val(0|1)
- 9: FET ch(0~7), val(0|1)
- 10: INT_DAC ch(0~1), val(0~4095)
- 11: EXT_DAC ch(0~7), val clamp 0~500
- 20: ACTIVE_RECIPE ch=recipe index

예:
- {"set":[7,3,1]}
- {"set":[11,2,375]}
- {"set":[20,1,0]}

### 4.4 MQTT run 명령
형식:
{"run":[cmd,arg1,arg2]}

동작:
- 내부에서 UART 라인 ba+run=cmd,arg1,arg2 를 구성해 batagota 파서로 전달

예:
- {"run":[4,2,30]}
- {"run":[1,2,0]}

### 4.5 MQTT recipe 명령
형식:
{"recipe":{"recipe_no":n,"stage_no":s,...}}
또는
{"recipe":{"recipe_no":n,"stage_no":s,"data":{...}}}

규칙/검증/stage 필드는 UART ba+recipe와 동일

성공 로그:
- [MQTT CMD] RECIPE_JSON applied: recipe=<n> stage=<s>

실패 로그 예:
- Invalid recipe_no
- Invalid stage_no
- Failed to apply recipe JSON (validation failed)

### 4.6 MQTT에서 레시피 자료 전송 요청 프로토콜
요청:
{"run":[1,<recipe_no>,0]}

현재 구현 응답 채널:
- 레시피 JSON 본문은 UART Serial로 1회 출력됨
- MQTT Cmd topic으로는 별도 ACK JSON을 publish하지 않음

따라서 MQTT-only HMI가 "레시피 전체 JSON 응답"을 반드시 MQTT로 받아야 하는 경우,
현행 firmware에는 전용 MQTT response 프로토콜이 구현되어 있지 않습니다.

---

## 5. UART/MQTT 공통 레시피 필드 ID 맵
(ba+set의 arg1=21~52와 1:1 대응)

- 21 fuel_type
- 22 smoke_enable
- 23 ignite_t1
- 24 ignite_t2
- 25 pump_condition
- 26 pump_power
- 27 reignite_t1
- 28 reignite_t2
- 29 pump_on_sec
- 30 pump_off_sec
- 31 cook_minutes
- 32 cook oven_min
- 33 cook oven_max
- 34 cook oven_error_pct
- 35 cook heater_on_sec
- 36 cook heater_off_sec
- 37 cook fan_on_sec
- 38 cook fan_off_sec
- 39 cook spray_time_sec
- 40 cook spray_power
- 41 dry_minutes
- 42 dry oven_min
- 43 dry oven_max
- 44 dry oven_error_pct
- 45 dry heater_on_sec
- 46 dry heater_off_sec
- 47 hold_minutes
- 48 hold oven_min
- 49 hold oven_max
- 50 hold oven_error_pct
- 51 hold heater_on_sec
- 52 hold heater_off_sec

---

## 6. HMI 연동 권장 시나리오
- UART 기반 HMI
  - 제어: ba+set / ba+run / ba+recipe
  - 조회: ba+get
  - 레시피 요청: ba+run=1,recipe_no,0
- MQTT 기반 HMI
  - 제어: {"set":[...]} / {"run":[...]} / {"recipe":{...}}
  - 상태 수신: BAGO/<mode><number>/Status
  - 레시피 요청: {"run":[1,recipe_no,0]} 가능
    - 단, 현재 레시피 전체 응답 JSON은 UART Serial 출력 경로

---

## 7. 변경 이력
- 2026-04-22
  - UART 프로토콜 문서를 실제 코드 구현(handleUartReceive) 기준으로 전면 정정
  - set/get 지원 항목을 1,5,6,7,8,9,10,11,15,16,20,21~52 기준으로 정리
  - recipe JSON(stage 0~3) 규칙을 실제 applyRecipeJsonByStage와 일치화
  - run=1,recipe_no,0의 실제 동작(레시피 JSON 출력 + 자동 상태 전이) 반영
  - MQTT Cmd protocol(set/run/recipe + legacy keys) 통합 추가
  - 레시피 자료 전송 요청 프로토콜(UART/MQTT) 포함
