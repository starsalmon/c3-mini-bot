#pragma once

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

class OLEDSSD1306
{
public:
    OLEDSSD1306();

    bool begin();

    void clear();
    void display();

    void setTextSize(uint8_t size);
    void setCursor(int16_t x, int16_t y);
    void print(const char *text);
    void println(const char *text);

    void drawBitmap(
        int16_t x,
        int16_t y,
        const uint8_t *bitmap,
        int16_t w,
        int16_t h,
        uint16_t color = SSD1306_WHITE
    );

    void drawBitmapVertical(
      int16_t x,
      int16_t y,
      const uint8_t *bitmap,
      int16_t w,
      int16_t h
    );    

    void showStartup();

    bool isOK() const;

private:
    Adafruit_SSD1306 _display;
    bool _ok;
};