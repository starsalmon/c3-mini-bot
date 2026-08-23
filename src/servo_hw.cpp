#include "servo_hw.h"

#include <ESP32PWM.h>

void servo_hw_init() {
  static bool done = false;
  if (done) {
    return;
  }
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  done = true;
}
