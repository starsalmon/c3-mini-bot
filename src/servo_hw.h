#pragma once

#include <ESP32Servo.h>

// Call once before any ESP32Servo attach (3 timers for 2× wheel + pan).
void servo_hw_init();

// 50 Hz + 16-bit LEDC before attach — keeps µs mapping stable on Arduino 3.x.
void servo_hw_prepare(Servo& servo);
