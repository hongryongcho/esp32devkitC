#ifndef TIMESERVICE_H
#define TIMESERVICE_H

#include <Arduino.h>

class TimeService {
public:
    void init();
    void start1msTimer();
    void update();
        // 플래그 확인 메서드 추가
    bool check1msFlag();
    bool check1sFlag();
    void clear1msFlag();
    void clear1sFlag();
};

extern TimeService timeService;
extern hw_timer_t * timer;
extern volatile bool flag_1s;
extern volatile bool flag_1ms;

void onTimer();

#endif // TIMESERVICE_H
