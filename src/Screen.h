#ifndef SCREEN__H_
#define SCREEN__H_
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

#include "status.h"

#define LOGO_HEIGHT 32
#define LOGO_WIDTH 128
static const unsigned char PROGMEM logo_bmp[] =
    {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x3f, 0xff, 0x80, 0x03, 0x87, 0x07, 0xe0, 0x01, 0xc7, 0x07, 0x00, 0x71, 0xc3, 0x8e, 0x01, 0xc0,
        0x3f, 0xff, 0x80, 0x03, 0xbf, 0x3f, 0xf0, 0x07, 0xf7, 0x07, 0x01, 0xfd, 0xc3, 0xbf, 0x8f, 0xe0,
        0x3f, 0xff, 0x80, 0x03, 0xff, 0x38, 0x78, 0x0f, 0x3f, 0x07, 0x07, 0xcf, 0xc3, 0xf9, 0xde, 0xf0,
        0x00, 0xe0, 0x00, 0x03, 0xe0, 0x00, 0x1c, 0x1c, 0x0f, 0x07, 0x07, 0x07, 0xc3, 0xe0, 0xf0, 0x70,
        0x00, 0xe0, 0x00, 0x03, 0xc0, 0x00, 0x1c, 0x1c, 0x0f, 0x07, 0x0e, 0x03, 0xc3, 0xc0, 0xf0, 0x38,
        0x00, 0xe0, 0x00, 0x03, 0x80, 0x00, 0x1c, 0x38, 0x07, 0x07, 0x0e, 0x03, 0xc3, 0x80, 0xe0, 0x38,
        0x00, 0xe0, 0x00, 0x03, 0x80, 0x07, 0xfc, 0x38, 0x07, 0x07, 0x0c, 0x01, 0xc3, 0x80, 0xe0, 0x38,
        0x00, 0xe0, 0x1f, 0xe3, 0x80, 0x1f, 0xfc, 0x38, 0x07, 0x07, 0x0c, 0x01, 0xc3, 0x80, 0xe0, 0x38,
        0x00, 0xe0, 0x1f, 0xe3, 0x80, 0x3c, 0x1c, 0x38, 0x07, 0x07, 0x0c, 0x01, 0xc3, 0x80, 0xe0, 0x38,
        0x00, 0xe0, 0x00, 0x03, 0x80, 0x70, 0x1c, 0x38, 0x07, 0x07, 0x0c, 0x01, 0xc3, 0x80, 0xe0, 0x38,
        0x00, 0xe0, 0x00, 0x03, 0x80, 0x70, 0x1c, 0x38, 0x07, 0x07, 0x0e, 0x03, 0xc3, 0x80, 0xe0, 0x38,
        0x00, 0xe0, 0x00, 0x03, 0x80, 0x70, 0x1c, 0x38, 0x07, 0x07, 0x0e, 0x03, 0xc3, 0x80, 0xe0, 0x38,
        0x00, 0xe0, 0x00, 0x03, 0x80, 0x70, 0x3c, 0x1c, 0x0f, 0x07, 0x07, 0x07, 0xc3, 0x80, 0xe0, 0x38,
        0x00, 0x70, 0x00, 0x03, 0x80, 0x38, 0x7c, 0x0e, 0x1f, 0x07, 0x07, 0xcf, 0xc3, 0x80, 0xe0, 0x38,
        0x00, 0x7c, 0x00, 0x03, 0x80, 0x3f, 0xfc, 0x07, 0xf7, 0x07, 0x01, 0xfd, 0x83, 0x80, 0xe0, 0x38,
        0x00, 0x1c, 0x00, 0x03, 0x80, 0x0f, 0xdc, 0x03, 0xe7, 0x07, 0x00, 0x71, 0x83, 0x80, 0xe0, 0x38,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x07, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xfe, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

class Screen {
   public:
    Screen(Status &status, uint8_t w, uint8_t h, TwoWire *twi = &Wire,
           int8_t rst_pin = -1, uint32_t clkDuring = 400000UL,
           uint32_t clkAfter = 100000UL);
    Adafruit_SSD1306 display;
    enum Mode {
        PRE_POST,
        POST_MOTORS,
        POST_SERIAL,
        POST_TOF,
        POST_ADC,
        POST_IMU,
        POST_IMU_CALIBRATION_STATUS,
        POST_IMU_CALIBRATE,
        POST_IMU_CALIBRATION_COMPLETE,
        START_UP,
        GIT_STATUS,
        IMU,
        SENSOR_DATA,
        TIMING,
        ERROR,
    };
    bool initDisplay();
    void setMode(Screen::Mode mode);
    void showScreen();

   private:
    Screen::Mode _currentMode = Screen::Mode::START_UP;
    Status &_status;
    void showPrePost();
    void showPostMotors();
    void showPostSerial();
    void showPostTOF();
    void showPostADC();
    void showPostIMU();
    void showPostIMUCalibrationStatus();
    void showPostIMUCalibrate();
    void showPostIMUCalibrationComplete();
    void showStartup();
    void showGitStatus();
    void showIMU();
    void showSensorData();
    void showTiming();
    void showError();
};

#endif  //  SCREEN__H_