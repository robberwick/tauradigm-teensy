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

    display.setCursor(0, 0);

    switch (_currentMode) {
        case Mode::START_UP:
            showStartup();
            break;

        case Mode::DEBUG_WARNING:
            showDebugWarning();
            break;

        case Mode::GIT_STATUS:
            showGitStatus();
            break;

        case Mode::PRE_POST:
            showPrePost();
            break;

        case Mode::POST_START:
            showPostStart();
            break;

        case Mode::POST_I2C:
            showPostI2C();
            break;

        case Mode::POST_MOTORS:
            showPostMotors();
            break;

        case Mode::POST_SERIAL:
            showPostSerial();
            break;

        case Mode::POST_TOF:
            showPostTOF();
            break;

        case Mode::POST_IMU_INIT_STATUS:
            showPostIMUInitStatus();
            break;

        case Mode::POST_IMU_CALIBRATION_RESTORE_OK:
            showPostIMUCalibrationRestoreOK();
            break;

        case Mode::POST_IMU_CALIBRATION_RESTORE_FAIL:
            showPostIMUCalibrationRestoreFail();
            break;

        case Mode::POST_IMU_CALIBRATE:
            showPostIMUCalibrateSystem();
            break;

        case Mode::POST_IMU_CALIBRATE_WITH_EVENT:
            showPostIMUCalibrateSystem();
            showPostIMUCalibrateEvent();
            break;

        case Mode::POST_IMU_CALIBRATION_COMPLETE:
            showPostIMUCalibrationComplete();
            break;

        case Mode::POST_IMU_CALIBRATION_SAVED:
            showPostIMUCalibrationSaved();
            break;

        case Mode::RUNNING:
            showRunning();
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
            
        case Mode::WAYPOINTS:
            showWaypoints();
            break;
            // flush changes to the display
    }
    display.display();
}

void Screen::showError() {
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
    display.println("Git Branch:");
    display.println(_status.git.branch);
    display.println("Git commit hash:");
    display.println(_status.git.commit);
}

void Screen::showPostMotors() {
    display.println("Motors");
    display.print("OK");
}

void Screen::showPrePost() {
    display.println("Press button now");
    display.println("to enter POST");
    display.println();
    display.println("Battery Voltage:");
    display.printf("%2.2f V", _status.getBatteryVoltage());
}

void Screen::showPostSerial() {
    display.println("Serial transfer");
    display.print("OK");
}

void Screen::showPostTOF() {
    display.println("ToF sensors");
    for (uint8_t t = 0; t < 8; t++) {
        if (_status.activation.tofSensors[t]) {
            display.printf("%d: OK", t);
        } else {
            display.printf("%d: FAIL", t);
        }
        if (t % 2 == 1) {
            display.println("");
        } else {
            display.setCursor(64, display.getCursorY());
        }
    }
}

void Screen::showPostIMUInitStatus() {
    if (!_status.activation.imu) {
        display.println("IMU FAIL");
    } else {
        display.println("IMU INIT OK:");
    }
}

void Screen::showPostIMUCalibrationRestoreFail() {
    display.println("No IMU offset Data");
    display.println("found in EEPROM");
}

void Screen::showPostIMUCalibrationRestoreOK() {
    display.println("IMU offset Data");
    display.println("restored successfully");
}

void Screen::showPostIMUCalibrateSystem() {
    display.println("Move robot to");
    display.println("calibrate IMU");
    display.println("");
    display.print("Sys:");
    display.print(_status.sensors.imu.calibration.system, DEC);
    display.print(" G:");
    display.print(_status.sensors.imu.calibration.gyro, DEC);
    display.print(" A:");
    display.print(_status.sensors.imu.calibration.accel, DEC);
    display.print(" M:");
    display.println(_status.sensors.imu.calibration.mag, DEC);
}

void Screen::showPostIMUCalibrateEvent() {
    display.print("X:");
    display.println(_status.sensors.imu.imuEvent.orientation.x, 4);
    display.print("Y:");
    display.println(_status.sensors.imu.imuEvent.orientation.y, 4);
    display.print("Z:");
    display.println(_status.sensors.imu.imuEvent.orientation.z, 4);
}

void Screen::showPostIMUCalibrationSaved() {
    display.println("IMU offset data");
    display.println("stored to EEPROM.");
}

void Screen::showPostIMUCalibrationComplete() {
    display.println("Calibrated OK");
}

void Screen::showRunning() {
    display.println("Running");
    display.println("");
    display.println("Battery Voltage:");
    display.printf("%2.2f V", _status.getBatteryVoltage());
}

void Screen::showPostI2C() {
    display.println("I2c Devices");

    for (uint8_t addr = 1; addr <= 127; addr++) {
        if (_status.activation.i2c[addr]) {
            //C++20 has a contains() method for unordered_map
            // but find() is only one available to us?
            if (I2C_ADDRESS_NAMES.find(addr) != I2C_ADDRESS_NAMES.end()) {
                display.println(I2C_ADDRESS_NAMES.at(addr));
            } else {
                display.print("0x");
                display.println(addr, HEX);
            }
        }
    }
}

void Screen::showPostStart() {
    display.println("Entering POST...");
}

void Screen::showDebugWarning() {
    display.println("*** WARNING ***");
    display.println("");
    display.println("Debug flag is set");
    display.println("Waiting for\nUSB serial");
}

void Screen::showWaypoints() {
    display.println("Navigating Waypoints");
    display.printf("W Pose:");
    display.printf("%1.1f,%3.0f,%3.0f", _status.waypointPose.heading, _status.waypointPose.x, _status.waypointPose.y);
    display.println(" ");
    display.printf("A Pose:");
    display.printf("%1.1f,%3.0f,%3.0f", _status.pose.heading, _status.pose.x, _status.pose.y);
    display.println(" ");
    display.printf("Waypoint #%d / %d", _status.waypointNumber,_status.numWaypoints);
    display.println(" ");
    display.printf("dist:%3.0f, head:%1.1f", _status.wayDist, _status.wayHead);
    display.printf("waypoints: %1.1f", _status.numWaypoints);
}