#include <ctype.h>
#include <stdlib.h>

#include "drive_cli.h"

#include "drive_servos.h"

namespace {

bool s_active = false;

float pct_to_speed(int pct) {
  if (pct > 100) pct = 100;
  if (pct < -100) pct = -100;
  return static_cast<float>(pct) / 100.0f;
}

void print_help(Stream& io) {
  io.println();
  io.println("Drive test (percent = -100..100):");
  io.println("  L <pct>   left wheel only");
  io.println("  R <pct>   right wheel only");
  io.println("  B <pct>   both wheels (same speed)");
  io.println("  S         stop");
  io.println("  ?         this help");
  io.println("Examples:  L 5   R 15   B 30   B -10");
  io.println();
}

}  // namespace

bool drive_cli_active() { return s_active; }

bool drive_cli_handle_line(DriveServos& drive, Stream& io, const char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (*line == '\0') return false;

  const char cmd = static_cast<char>(toupper(static_cast<unsigned char>(*line)));
  if (cmd != 'L' && cmd != 'R' && cmd != 'B' && cmd != 'S' && cmd != '?') {
    return false;
  }

  const char* rest = line + 1;
  while (*rest == ' ' || *rest == '\t') ++rest;

  if (cmd == '?' || (cmd == 'H' && *rest == '\0')) {
    print_help(io);
    return true;
  }

  if (cmd == 'S' && *rest == '\0') {
    drive.stop();
    s_active = false;
    io.println("stop");
    return true;
  }

  if (*rest == '\0') {
    io.println("need percent, e.g.  L 15");
    return true;
  }

  char* end = nullptr;
  const long pct = strtol(rest, &end, 10);
  if (end == rest) {
    io.println("bad number");
    return true;
  }

  const float spd = pct_to_speed(static_cast<int>(pct));
  switch (cmd) {
    case 'L':
      drive.set_wheel_speeds(spd, 0.0f);
      s_active = (fabsf(spd) > 0.001f);
      io.printf("L %ld%%\n", pct);
      break;
    case 'R':
      drive.set_wheel_speeds(0.0f, spd);
      s_active = (fabsf(spd) > 0.001f);
      io.printf("R %ld%%\n", pct);
      break;
    case 'B':
      drive.set_wheel_speeds(spd, spd);
      s_active = (fabsf(spd) > 0.001f);
      io.printf("B %ld%%\n", pct);
      break;
    default:
      return false;
  }
  return true;
}

bool drive_cli_poll(DriveServos& drive, Stream& io) {
  static char line[32];
  static uint8_t len = 0;

  while (io.available()) {
    const char c = static_cast<char>(io.read());
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = '\0';
      drive_cli_handle_line(drive, io, line);
      len = 0;
      continue;
    }
    if (len + 1 < sizeof(line)) {
      line[len++] = c;
    }
  }
  return s_active;
}

bool drive_cli_read_line(DriveServos& drive, Stream& io, const char* line) {
  return drive_cli_handle_line(drive, io, line);
}
