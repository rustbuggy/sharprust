#ifndef TIMER_H_
#define TIMER_H_

#define NON_ACTIVE 0xFFFFFFFF
#define ONE_SHOT 0xFFFFFFFF

class timer {
private:
  uint32_t timeout;
  uint32_t trig_time;

public:
  timer() :
      timeout(ONE_SHOT), trig_time(NON_ACTIVE) {
  }

  inline bool running() {
    return trig_time != NON_ACTIVE;
  }

  bool start(uint32_t time, uint32_t timeout, bool one_shot) {
    if (!running()) {
      if (one_shot) {
        this->timeout = ONE_SHOT;
      }
      else {
        this->timeout = timeout;
      }
      trig_time = time + timeout;
      return true;
    }
    else {
      return false;
    }
  }

  bool triggered(uint32_t time) {
    if (time > trig_time) {
      if (timeout == ONE_SHOT) {
        trig_time = NON_ACTIVE;
      }
      else {
        trig_time = time + timeout;
      }
      return true;
    }
    else {
      return false;
    }
  }

  bool start_or_triggered(uint32_t time, uint32_t timeout, bool one_shot, bool start_triggered) {
    if (start(time, timeout, one_shot)) {
      return start_triggered;
    }
    else if (triggered(time)) {
      return true;
    }
    else {
      return false;
    }
  }

  void stop() {
    timeout = ONE_SHOT;
    trig_time = NON_ACTIVE;
  }

};

#endif // TIMER_H_
