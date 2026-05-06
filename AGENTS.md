# AGENTS.md — Workspace Copilot Instructions

## 1. 빌드/업로드/테스트
- **PlatformIO** 기반 프로젝트입니다.
- 빌드: `platformio run`
- 업로드: `platformio run --target upload`
- 시리얼 모니터: `platformio device monitor`
- 테스트 폴더: `test/` (테스트 자동화는 별도 구현 필요)

## 2. 주요 설계/구조
- **src/**: 모든 펌웨어 소스(C++), 주요 클래스: `Batagota`, `ConfigManager` 등
- **include/**: 헤더 파일
- **lib/**: 외부/공용 라이브러리
- **platformio.ini**: 빌드 환경 설정
- **문서**: 주요 기능별 md 파일 존재 (아래 링크 참고)

## 3. 프로젝트 관례 및 주의사항
- **채널/센서/출력 등은 enum 기반으로 관리** (동적 등록 X, 정적 enum 사용)
- **상태머신 기반 제어**: `Batagota` 클래스의 state machine 구조 참고
- **UART 명령/레시피/NTC 등은 별도 문서 참고**
- **WiFi/MQTT/EEPROM/Preferences 등 ESP32 표준 라이브러리 사용**
- **코드/문서 분리 원칙**: 문서화된 내용은 중복 구현하지 말고 링크로 안내

## 4. 주요 문서 링크
- [ALGORITHM_FLOWCHART.md](ALGORITHM_FLOWCHART.md): 전체 시스템/상태 다이어그램
- [ENUM_BASED_REFACTORING.md](ENUM_BASED_REFACTORING.md): enum 기반 채널 관리
- [UART_COMMAND_PROTOCOL.md](UART_COMMAND_PROTOCOL.md): UART 명령 프로토콜
- [UART_RECIPE_JSON_PROTOCOL.md](UART_RECIPE_JSON_PROTOCOL.md): 레시피 JSON 프로토콜
- [NTC_THERMISTOR_INTEGRATION.md](NTC_THERMISTOR_INTEGRATION.md): NTC 회로/계산법

## 5. Agent 원칙
- **Link, don't embed**: 문서화된 내용은 링크로 안내, 중복 설명 금지
- **관례/패턴 우선**: enum/static 구조, 상태머신, 문서화된 정책 우선 적용
- **PlatformIO 명령 자동화**: 빌드/업로드/모니터 명령 자동 실행
- **코드/문서 분리**: 주석/문서/코드 중복 금지

## 6. 예시 프롬프트
- "NTC 온도센서 회로와 계산법 요약해줘"
- "UART 명령 프로토콜에 맞는 파싱 함수 예시"
- "enum 기반 채널 관리 방식 설명해줘"
- "platformio로 빌드/업로드 자동화 task 만들어줘"

## 7. 추가 agent-customization 제안
- **/create-instruction platformio-tasks**: PlatformIO 빌드/업로드/모니터 task 자동화
- **/create-skill uart-recipe**: UART 레시피 JSON 파싱/적용 예시 및 테스트 자동화
- **/create-skill ntc-thermistor**: NTC 회로/계산/ADC 변환 공식 자동화

---
> 이 파일은 workspace 내 Copilot/agent의 기본 행동 원칙과 관례를 정의합니다. 문서화된 정책/구조/패턴을 우선 적용하고, 중복 설명은 피하세요.
