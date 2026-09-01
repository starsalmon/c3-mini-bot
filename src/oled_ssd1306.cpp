#include "oled_ssd1306.h"

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C

#ifndef OLED_ENABLED
#define OLED_ENABLED 0
#endif


// -------------------------------------------------------------------------
// Cat bitmap - 32 x 32 pixels
// -------------------------------------------------------------------------

const int cat_with_wry_smile_1f63c_32_width  = 32;
const int cat_with_wry_smile_1f63c_32_height = 32;

const unsigned char cat_with_wry_smile_1f63c_32[] PROGMEM =
{
    0x00, 0x00, 0x00, 0x80, 0xf0, 0x18, 0x88, 0x90,
    0x30, 0x60, 0xc0, 0x80, 0x00, 0x80, 0x80, 0x80,
    0x80, 0x80, 0x80, 0x00, 0x80, 0xc0, 0x60, 0x30,
    0x90, 0x88, 0x18, 0xf0, 0x80, 0x00, 0x00, 0x00,

    0x00, 0x00, 0x00, 0x03, 0xfe, 0x40, 0x07, 0x0f,
    0x07, 0x02, 0x80, 0x81, 0x03, 0x0f, 0x0f, 0x00,
    0x00, 0x0f, 0x0f, 0x03, 0x81, 0x80, 0x02, 0x07,
    0x0f, 0x07, 0x40, 0xfe, 0x03, 0x00, 0x00, 0x00,

    0x00, 0x01, 0x19, 0x8a, 0x7f, 0xea, 0xca, 0x20,
    0x00, 0x00, 0x03, 0x03, 0x00, 0x40, 0x40, 0x4c,
    0x3c, 0x40, 0x40, 0x40, 0x33, 0x03, 0x00, 0x00,
    0x20, 0xca, 0xea, 0x7f, 0x8a, 0x19, 0x01, 0x00,

    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02,
    0x06, 0x04, 0x08, 0x08, 0x08, 0x10, 0x10, 0x11,
    0x11, 0x11, 0x10, 0x08, 0x08, 0x08, 0x04, 0x06,
    0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


OLEDSSD1306::OLEDSSD1306()
    : _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET),
      _ok(false)
{
}


bool OLEDSSD1306::begin()
{
#if OLED_ENABLED

    if (!_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        _ok = false;
        return false;
    }

    _ok = true;

    showStartup();

    return true;

#else

    _ok = false;
    return false;

#endif
}


void OLEDSSD1306::showStartup()
{
#if OLED_ENABLED

    _display.clearDisplay();

    // Cat
    drawBitmapVertical(
      0,
      16,
      cat_with_wry_smile_1f63c_32,
      cat_with_wry_smile_1f63c_32_width,
      cat_with_wry_smile_1f63c_32_height
    );

    // Bot name
    _display.setTextColor(SSD1306_WHITE);
    _display.setTextSize(2);
    _display.setCursor(40, 8);
    _display.println(F("Roby"));

    // Status
    _display.setTextSize(1);
    _display.setCursor(40, 32);
    _display.println(F("All systems OK"));

    _display.setTextSize(1);
    _display.setCursor(40, 48);
    _display.println(F("Seeking to"));

    _display.setTextSize(1);
    _display.setCursor(40, 56);
    _display.println(F("KILL ALL HUMANS!"));

    _display.display();

#endif
}


void OLEDSSD1306::clear()
{
#if OLED_ENABLED
    _display.clearDisplay();
#endif
}


void OLEDSSD1306::display()
{
#if OLED_ENABLED
    _display.display();
#endif
}


void OLEDSSD1306::setTextSize(uint8_t size)
{
#if OLED_ENABLED
    _display.setTextSize(size);
#endif
}


void OLEDSSD1306::setCursor(int16_t x, int16_t y)
{
#if OLED_ENABLED
    _display.setCursor(x, y);
#endif
}


void OLEDSSD1306::print(const char *text)
{
#if OLED_ENABLED
    _display.print(text);
#endif
}


void OLEDSSD1306::println(const char *text)
{
#if OLED_ENABLED
    _display.println(text);
#endif
}


void OLEDSSD1306::drawBitmap(
    int16_t x,
    int16_t y,
    const uint8_t *bitmap,
    int16_t w,
    int16_t h,
    uint16_t color)
{
#if OLED_ENABLED
    _display.drawBitmap(x, y, bitmap, w, h, color);
#endif
}


void OLEDSSD1306::drawBitmapVertical(
  int16_t x,
  int16_t y,
  const uint8_t *bitmap,
  int16_t w,
  int16_t h)
{
#if OLED_ENABLED

  for (int16_t py = 0; py < h; py++)
  {
      for (int16_t px = 0; px < w; px++)
      {
          int16_t byteIndex = (py / 8) * w + px;
          uint8_t bit = 1 << (py % 8);

          if (pgm_read_byte(&bitmap[byteIndex]) & bit)
          {
              _display.drawPixel(
                  x + px,
                  y + py,
                  SSD1306_WHITE
              );
          }
      }
  }

#endif
}

bool OLEDSSD1306::isOK() const
{
    return _ok;
}