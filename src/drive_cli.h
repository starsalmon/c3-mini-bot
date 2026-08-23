#pragma once

#include <Arduino.h>

class DriveServos;

// Interactive wheel test over USB serial (rover-style). Press Enter after each line.
//   L 15   left wheel 15%   R 20   right 20%   B 25   both 25%
//   S      stop             ?      help
// Percent is -100..100 (negative = reverse).
bool drive_cli_poll(DriveServos& drive, Stream& io = Serial);
bool drive_cli_handle_line(DriveServos& drive, Stream& io, const char* line);
bool drive_cli_active();
