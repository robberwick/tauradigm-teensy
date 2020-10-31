#ifndef _CONFIG__H_
#define _CONFIG__H_

#include <unordered_map>
using namespace std;

// Pins
#ifdef ARDUINO_TEENSY31
#include "config/teensy.h"
#endif

#ifdef ARDUINO_BLACKPILL_F411CE
#include "config/blackpill.h"
#endif

// i2c Addresses
#define TCA_ADDR 0x70
#define DISPLAY_ADDR 0x3c
#define IMU_ADDR 0x28
#define TOF_ADDR 0x29
#define ADC_ADDR 0x48
typedef const std::unordered_map<uint8_t, String> HexIntMap;
HexIntMap I2C_ADDRESS_NAMES = {
    {TCA_ADDR, "Multiplexer"},
    {IMU_ADDR, "IMU"},
    {TOF_ADDR, "Distance sensor"},
    {DISPLAY_ADDR, "OLED display"},
    {ADC_ADDR, "Minesweeper ADC"}};
// hardware counts
#define NUM_ENCODERS 6

// timing constants
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define MOTOR_MESSAGE_TIMEOUT_MS 20
#define SEND_SENSOR_DATA_TIMEOUT_MS 100
#define UPDATE_IMU_TIMEOUT_MS 20
#define UPDATE_TOF_TIMEOUT_MS 100
#define UPDATE_ENCODER_TIMEOUT_MS 20
#define UPDATE_BATT_TIMEOUT_MS 1000

// Comms constants
#define MAX_MISSED_MOTOR_MESSAGES 10

#endif  //_CONFIG__H_