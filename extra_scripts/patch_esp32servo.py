"""Patch ESP32Servo double-attach on Arduino-ESP32 3.x (clean LEDC boot)."""
Import("env")

import glob
import os
import re

_MARKER = "c3-mini-bot: skip redundant attachPin on Arduino 3.x"
_PATTERN = re.compile(
    r'(\t\tESP_LOGI\(TAG, "Pin Setup %d with code %d",pin,ret\);\n)\s+attachPin\(pin\);',
    re.MULTILINE,
)
_REPLACEMENT = (
    r'\1#if defined(ESP_ARDUINO_VERSION_MAJOR) && '
    r'ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)\n'
    f"\t\t// {_MARKER}\n"
    r"\t\tattach(pin);\n"
    r"#else\n"
    r"\t\tattachPin(pin);\n"
    r"#endif"
)


def _patch(path):
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    if _MARKER in text:
        return False
    new_text, count = _PATTERN.subn(_REPLACEMENT, text, count=1)
    if count == 0:
        print(f"WARN patch_esp32servo: pattern missing in {path}")
        return False
    with open(path, "w", encoding="utf-8") as f:
        f.write(new_text)
    print(f"patch_esp32servo: fixed {path}")
    return True


def _apply(source, target, env):
    root = env.subst("$PROJECT_LIBDEPS_DIR")
    if not os.path.isdir(root):
        return
    for path in glob.glob(os.path.join(root, "*", "ESP32Servo", "src", "ESP32PWM.cpp")):
        _patch(path)


env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", _apply)
_apply(None, None, env)
