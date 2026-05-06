# NTC Thermistor 온도 센서 통합 가이드 (수정판)

## 📌 회로 구성 (최종 반영)

```
        12V
         │
        [10K 풀업 저항]
         │
         ├─────────────── ADC 입력 (Diode로 3.3V 클램프)
         │
      [10K NTC] [3.9K 병렬]
           [||]
         │
        GND
```

### 회로 설명
- **전원**: 12V DC
- **풀업 저항**: 10K Ω (12V 쪽에 연결)
- **병렬 연결**: 
  - 10K NTC Thermistor
  - 3.9K Ω 저항 (Thermistor와 병렬)
- **ADC 보호**: Diode로 3.3V 클램프
- **ADC 핀**: GPIO32 ~ GPIO39 (6개 채널 유효)

---

## 🔧 온도 계산 원리 (수정됨)

### Step 1: ADC 전압값 읽기
```
V_adc = ADC 읽은 값  (0 ~ 3.3V, Diode 클램프)
```

### Step 2: 병렬 저항값 역산
```
분압 공식:
V_adc = V_supply × R_parallel / (R_pullup + R_parallel)

역산:
R_parallel = (V_adc × R_pullup) / (V_supply - V_adc)
R_parallel = (V_adc × 10000) / (12 - V_adc)  Ω

유효 범위: 0 < R_parallel < 3900Ω (Diode 클램프로 인해 자동)
```

### Step 3: Thermistor 저항값 역산
```
병렬 저항 공식:
R_parallel = (R_th × R_series) / (R_th + R_series)

역산:
R_th = (R_parallel × R_series) / (R_series - R_parallel)
R_th = (R_parallel × 3900) / (3900 - R_parallel)  Ω

주의: R_parallel < 3900Ω이어야 양수
```

### Step 4: Beta 공식으로 온도 계산
```
Steinhart-Hart의 Beta 형식:
1/T = 1/T0 + (1/B) × ln(R/R0)

계산:
ln_ratio = ln(R_th / R0)  = ln(R_th / 10000)
invT = (1/298.15) + (ln_ratio / 3950)
T_kelvin = 1 / invT
T_celsius = T_kelvin - 273.15

여기서:
- T = 절대온도 (K)
- T0 = 기준온도 (298.15K = 25°C)
- B = Beta값 (3950K)
- R = Thermistor 저항값 (Ω)
- R0 = 기준저항값 (10000Ω)
```

---

## 📊 코드 구현

### 1. ADC 설정 상수

**파일: BOPDrv_V2p0_ESP32devkitC_ver.ino (행 661-673)**

```cpp
// NTC Thermistor 설정 (10K NTC, 10K 풀업, 3.9K 병렬, 12V 전원, Diode 클램프)
const float THERMISTOR_SUPPLY_VOLTAGE = 12.0;      // 12V 전원
const float THERMISTOR_PULLUP_RESISTANCE = 10000.0; // 10K 풀업
const float THERMISTOR_SERIES_RESISTANCE = 3900.0; // 3.9K 병렬
const float THERMISTOR_NOMINAL_RESISTANCE = 10000.0; // 25°C 기준값
const float THERMISTOR_NOMINAL_TEMPERATURE = 298.15;  // 25°C = 298.15K
const float THERMISTOR_BETA_VALUE = 3950.0;       // Beta값
const float ABSOLUTE_ZERO = 273.15;               // K → °C 변환
const float ADC_CLAMP_VOLTAGE = 3.3;              // Diode 클램프 3.3V
```

### 2. 온도 계산 함수

**파일: BOPDrv_V2p0_ESP32devkitC_ver.ino (행 677-744)**

```cpp
float calculateThermistorTemperature(float thermistorVoltage)
{
    // Step 1: 극한값 체크
    if (thermistorVoltage < 0.001 || thermistorVoltage > ADC_CLAMP_VOLTAGE) {
        return -999.0;  // 에러값
    }
    
    // Step 2: 병렬 저항값 역산
    // R_parallel = (V_adc × R_pullup) / (V_supply - V_adc)
    float denominator = THERMISTOR_SUPPLY_VOLTAGE - thermistorVoltage;
    if (denominator <= 0.0) {
        return -999.0;
    }
    float rParallel = (thermistorVoltage * THERMISTOR_PULLUP_RESISTANCE) / denominator;
    
    // Step 3: Thermistor 저항값 역산
    // R_th = (R_parallel × R_series) / (R_series - R_parallel)
    float seriesDenominator = THERMISTOR_SERIES_RESISTANCE - rParallel;
    if (seriesDenominator <= 0.0) {
        return -999.0;
    }
    float resistance = (rParallel * THERMISTOR_SERIES_RESISTANCE) / seriesDenominator;
    
    // Step 4: Beta 공식으로 온도 계산
    // 1/T = 1/T0 + (1/B) × ln(R/R0)
    float logValue = logf(resistance / THERMISTOR_NOMINAL_RESISTANCE);
    float invT = (1.0 / THERMISTOR_NOMINAL_TEMPERATURE) + (logValue / THERMISTOR_BETA_VALUE);
    
    if (invT <= 0.0) {
        return -999.0;
    }
    
    float temperatureK = 1.0 / invT;
    float temperatureC = temperatureK - ABSOLUTE_ZERO;
    
    return temperatureC;
}
```

### 3. ADC 읽기 및 온도 계산

**파일: BOPDrv_V2p0_ESP32devkitC_ver.ino (행 746-789)**

```cpp
void readInternalADC() {
  for (int i = 0; i < 8; i++) {
    if (ADC_VALID[i]) {
      int rawValue = analogRead(ADC_PINS[i]);
      float voltage = (rawValue * ADC_REFERENCE_VOLTAGE) / ADC_RESOLUTION;
      
      // ADC 전압값 저장
      ioBoard.adc[i] = voltage;
      
      // Thermistor 온도 계산 및 저장
      float temperature = calculateThermistorTemperature(voltage);
      ioBoard.temperature[i] = temperature;
      
      // 1초마다 로그 출력
      static unsigned long lastLogTime = 0;
      static unsigned long LOG_INTERVAL = 1000;
      if (millis() - lastLogTime >= LOG_INTERVAL) {
        if (i == 0) Serial.printf("[ADC Thermistor] ");
        Serial.printf("Ch%d: V=%.3fV, T=%.2f°C  ", i, voltage, temperature);
        if (i == 5) {
            Serial.println();
            lastLogTime = millis();
        }
      }
    } else {
      ioBoard.adc[i] = 0.0;
      ioBoard.temperature[i] = -999.0;
    }
  }
}
```

### 4. Batagota 초기화 (setup)

**파일: BOPDrv_V2p0_ESP32devkitC_ver.ino (setup 함수 내)**

```cpp
// Batagota에 NTC Thermistor 채널 추가
batagota.addADCChannel("NTC_Thermistor_0");
batagota.addADCChannel("NTC_Thermistor_1");
batagota.addADCChannel("NTC_Thermistor_2");
batagota.addADCChannel("NTC_Thermistor_3");
batagota.addADCChannel("NTC_Thermistor_4");
batagota.addADCChannel("NTC_Thermistor_5");

batagota.addTemperatureChannel("Temperature_Control_0");
batagota.addTemperatureChannel("Temperature_Control_1");
```

### 5. Batagota에 온도값 전달 (loop)

**파일: BOPDrv_V2p0_ESP32devkitC_ver.ino (loop 함수, 100ms 구간)**

```cpp
// ESP32 내부 ADC 읽기
readInternalADC();

// Batagota에 온도값 전달
for (int i = 0; i < 6; i++) {
    if (ioBoard.temperature[i] > -999.0) {
        batagota.setADCValue(i, ioBoard.adc[i]);
        batagota.setTemperatureValue(i, ioBoard.temperature[i]);
    }
}
```

---

## 📈 사용 예시

### 온도값 읽기
```cpp
// Batagota에서 온도값 읽기
float currentTemp = batagota.getThermistorTemperature(0);  // °C
float voltage = batagota.getThermistorVoltage(0);          // V
```

### 온도 제어
```cpp
// 목표 온도 설정 (50°C)
batagota.setTemperatureSetpoint(0, 50.0);

// 100ms마다 자동으로 제어 출력 계산됨
float output = batagota.getThermistorControlOutput(0);  // 0.0 ~ 1.0

// DAC에 연결
int dacValue = (int)(output * 255);
analogWrite(DAC_PIN, dacValue);
```

---

## 🐛 디버깅

### 직렬 출력 예시
```
[ADC Thermistor] Ch0: V=1.500V, T=25.43°C  Ch1: V=0.987V, T=35.67°C  Ch2: V=2.145V, T=15.28°C  Ch3: V=0.567V, T=50.12°C  Ch4: V=2.876V, T=8.45°C  Ch5: V=1.234V, T=32.78°C
```

### 온도값 검증

| 전압 | 저항 | 온도 | 설명 |
|------|------|------|------|
| 0.1V | ~45Ω | ~85°C | 매우 높은 온도 |
| 0.5V | ~231Ω | ~65°C | 높은 온도 |
| 1.5V | ~1429Ω | ~25°C | 상온 |
| 2.5V | ~6250Ω | -5°C | 낮은 온도 |
| 3.3V | ~16000Ω | -20°C | 매우 낮은 온도 |
| -999.0 | - | - | 에러 |

### 에러 진단

**온도 = -999.0인 경우:**
- 회로 단절 확인
- ADC 핀 확인 (GPIO32-39)
- Diode가 제대로 클램프되는지 확인
- 12V 전원 확인

**온도 범위가 이상한 경우:**
- 저항값 확인 (10K 풀업, 3.9K 병렬)
- Beta값 확인 (NTC 제조사 스펙시트)
- Diode 레벨 확인 (정확히 3.3V)

---

## ✅ 검증 체크리스트

- [ ] 12V 전원이 안정적인가?
- [ ] 10K 풀업 저항이 12V에 연결되어 있는가?
- [ ] 10K NTC와 3.9K 저항이 병렬로 연결되어 있는가?
- [ ] Diode가 3.3V 레벨에서 정확히 클램프하는가?
- [ ] ADC 핀이 GPIO 32-39 범위인가?
- [ ] ADC 값이 0 ~ 3.3V 범위인가?
- [ ] 온도값이 -50°C ~ 100°C 범위인가?
- [ ] Batagota에서 온도값을 읽을 수 있는가?
- [ ] 제어 출력이 0.0 ~ 1.0 범위인가?

---

## 📁 수정된 파일 목록

1. **BOPDrv_V2p0_ESP32devkitC_ver.ino**
   - ADC 설정 상수 (행 661-673)
   - 온도 계산 함수 (행 677-744)
   - readInternalADC() 함수 (행 746-789)
   - setup() 함수 - Batagota 초기화
   - loop() 함수 - 온도값 전달

2. **batagota.h**
   - Thermistor 접근 함수 추가

3. **batagota.cpp**
   - Thermistor 접근 함수 구현

---

## 🔗 참고 자료

- **NTC Thermistor 공식**: [Steinhart-Hart 방정식](https://en.wikipedia.org/wiki/Thermistor)
- **Beta 공식**: B = (T1 × T2) / (T2 - T1) × ln(R1/R2)
- **10K NTC 일반 Beta값**: 3900 ~ 4000K

