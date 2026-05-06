#include "TimeService.h"
#include <time.h>

TimeService timeService;
hw_timer_t * timer = NULL;
volatile bool flag_1s = false;
volatile bool flag_1ms = false;

void onTimer() {
    static int counter = 0;
    if (++counter >= 1000) {
        counter = 0;
        flag_1s = true;
    }
    flag_1ms = true;
}

void TimeService::init() {
    configTime(9 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    Serial.println("NTP time configured.");
    struct tm timeinfo;
    int retry_count = 0;
    const int max_retries = 10;
    Serial.print("Waiting for NTP time synchronization");
    while (!getLocalTime(&timeinfo) && retry_count < max_retries) {
        delay(500);
        Serial.print(".");
        retry_count++;
    }
    Serial.println();
    if (retry_count == max_retries) {
        Serial.println("Failed to synchronize time with NTP server after multiple retries.");
    } else {
        Serial.println("Time synchronized successfully with NTP server.");
        char timeStr[64];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
        Serial.printf("Current time: %s\n", timeStr);
    }
}

void TimeService::start1msTimer() {
    // Arduino-ESP32 new timer API: set timer frequency in Hz.
    // 1 MHz gives 1 tick = 1 us, so alarm 1000 -> 1 ms period.
    timer = timerBegin(1000000);

    if (timer == NULL) {
        Serial.println("Failed to start timer");
        return;
    }

    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, 1000, true, 0);
    Serial.println("1ms Timer started with GPTimer API.");
}

bool TimeService::check1msFlag() {
    return flag_1ms;
}

bool TimeService::check1sFlag() {
    return flag_1s;
}

void TimeService::clear1msFlag() {
    flag_1ms = false;
}

void TimeService::clear1sFlag() {
    flag_1s = false;
}

void TimeService::update() {
    // 필요한 경우, 업타임 업데이트나 다른 시간 관련 로직을 여기에 추가할 수 있습니다.
}
