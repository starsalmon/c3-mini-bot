# Roby — future plans

## Roby

- [ ] Work out what happened to the RGB LED strip. Roby has one fitted and it used to work. **Investigation (docs only, no code change):** Front bar support has been in the tree since initial commit `321b322` (`bot_front_bar.cpp`, `FRONT_BAR_PIN` **GPIO19**, `FRONT_BAR_COUNT=8`). The pin was **not** reassigned or removed in the fleet-audit PR (`e556667` / prior `0b30b69`). Firmware still calls `front_bar.begin(FRONT_BAR_PIN, …)` and `front_bar.tick()` in `bot_main.cpp` and `autonomous_main.cpp` for all normal envs. Likely regressions to check on the bench: (1) commit `0b30b69` changed `FRONT_BAR_PIXEL_ORDER` from **`NEO_RGB` → `NEO_GRB`** — wrong order often looks like a dead or nonsense strip; (2) on the Waveshare C3 Zero, **GPIO19 is USB D+** and **GPIO18 is USB D−** while `ARDUINO_USB_CDC_ON_BOOT=1` — confirm the strip DIN is actually on **GPIO19** and not another wire; (3) brownout if bar + servos share a weak 5 V rail (`FRONT_BAR_BRIGHTNESS` default 28). Serial should still show front-bar init if `begin()` succeeds.

## Fleet-wide (same section in every bot's TODO)

- [ ] Infrared ID detection of other robots: when two bots meet, play a special tune and behaviour specific to the bot met.
- [ ] A single shared charging dock that every robot can find and self-attach to.
- [ ] Use Rover's house map for doing things.

It's not the destination, it's the journey.