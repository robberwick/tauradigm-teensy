#include "screen.h"

Screen::Screen(Status &status, uint8_t w, uint8_t h, TwoWire *twi,
               int8_t rst_pin, uint32_t clkDuring,
               uint32_t clkAfter) : _status(status) {
    display = Adafruit_SSD1306(w, h, twi, rst_pin, clkDuring, clkAfter);
}

void Screen::setMode(Screen::Mode mode) {
    _currentMode = mode;
}

void Screen::showScreen() {
    switch (_currentMode) {
        case Mode::POST_MOTORS:
            break;

        case Mode::POST_SERIAL:
            break;

        case Mode::POST_TOF:
            break;

        case Mode::POST_ADC:
            break;

        case Mode::POST_IMU:
            break;

        case Mode::POST_IMU_CALIBRATION_STATUS:
            break;

        case Mode::POST_IMU_CALIBRATE:
            break;

        case Mode::POST_IMU_CALIBRATION_COMPLETE:
            break;

        case Mode::START_UP:
            break;

        case Mode::IMU:
            break;

        case Mode::SENSOR_DATA:
            break;

        case Mode::TIMING:
            break;

        case Mode::ERROR:
            showError();

            break;
    }
}

void Screen::showError() {
    bool shouldInvertDisplay = (_status.motorMessageCommsDown() || _status.batteryIsLow());
    display.clearDisplay();
    display.setCursor(0, 0);
    if (_status.motorMessageCommsDown()) {
        display.printf("missed message %d", _status.missedMotorMessageCount);
        display.println();
    }
    // is battery going flat?
    if (_status.batteryIsLow()) {
        display.printf("low battery");
        display.println();
    }
    display.display();

    display.invertDisplay(shouldInvertDisplay);
}