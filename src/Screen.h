#ifndef SCREEN__H_
#define SCREEN__H_
#include <Adafruit_SSD1306.h>
#include <Wire.h>

class Screen {
   public:
    Screen(uint8_t w, uint8_t h, TwoWire *twi = &Wire,
           int8_t rst_pin = -1, uint32_t clkDuring = 400000UL,
           uint32_t clkAfter = 100000UL);
    Adafruit_SSD1306 display;
    enum Mode {
        POST_MOTORS,
        POST_SERIAL,
        POST_TOF,
        POST_ADC,
        POST_IMU,
        POST_IMU_CALIBRATION_STATUS,
        POST_IMU_CALIBRATE,
        POST_IMU_CALIBRATION_COMPLETE,
        START_UP,
        IMU,
        SENSOR_DATA,
        TIMING,
    };
    void setMode(Screen::Mode mode);

   private:
    Screen::Mode _currentMode = Screen::Mode::IMU;
};

#endif  //  SCREEN__H_