#include "screen.h"

Screen::Screen(uint8_t w, uint8_t h, TwoWire *twi,
               int8_t rst_pin, uint32_t clkDuring,
               uint32_t clkAfter) {
    display = Adafruit_SSD1306(w, h, twi, rst_pin, clkDuring, clkAfter);
}

void Screen::setMode(Screen::Mode mode) {
    _currentMode = mode;
}