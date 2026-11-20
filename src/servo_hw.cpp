#include "servo_hw.h"

#include <ESP32PWM.h>

#include "bot_pins.h"

void servo_hw_init() {
  static bool done = false;
  if (done) {
    return;
  }
  // Dedicated ESP32Servo channels: 0=L, 1=R, 2=pan (do not share timer with speaker/IR).
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  done = true;
}

void servo_hw_prepare(Servo& servo) {
  servo.setPeriodHertz(SERVO_PWM_HZ);
  servo.setTimerWidth(SERVO_PWM_RESOLUTION_BITS);
}
