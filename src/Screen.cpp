#include "screen.h"

#include <Arduino.h>

#include "config.h"

Screen::Screen(Status &status, uint8_t w, uint8_t h, TwoWire *twi,
               int8_t rst_pin, uint32_t clkDuring,
               uint32_t clkAfter) : _status(status) {
    display = Adafruit_SSD1306(w, h, twi, rst_pin, clkDuring, clkAfter);
}

bool Screen::initDisplay() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDR)) {
        return false;
    }
    display.setTextSize(1);                              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);  // Draw white text
    display.setCursor(0, 0);                             // Start at top-left corner
    display.cp437(true);                                 // Use full 256 char 'Code Page 437' font

    return true;
}

void Screen::setMode(Screen::Mode mode) {
    _currentMode = mode;
}

void Screen::showScreen() {
    // clear the display
    display.clearDisplay();
    // invert the display if we're displaying the error screen
    display.invertDisplay(_currentMode == Mode::ERROR);
    switch (_currentMode) {
        case Mode::POST_MOTORS:
            showPostMotors();
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
            showStartup();
            break;

        case Mode::GIT_STATUS:
            showGitStatus();
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
            // flush changes to the display
    }
    display.display();
}

void Screen::showError() {
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
}

void Screen::showStartup() {
    display.drawBitmap(
        (display.width() - LOGO_WIDTH) / 2,
        (display.height() - LOGO_HEIGHT) / 2,
        logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
}

void Screen::showGitStatus() {
    display.setCursor(0, 10);
    display.println("Git Branch:");
    display.println(_status.git.branch);
    display.println("Git commit hash:");
    display.println(_status.git.commit);
}

void Screen::showPostMotors() {
    display.setCursor(0, 0);
    display.println("Motors");
    display.setCursor(0, 10);
    display.print("OK");
}