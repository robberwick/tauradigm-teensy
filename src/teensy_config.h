#include <unordered_map>
using namespace std; 

// Pins
#define TEENSY_PIN_ENC1A (0)
#define TEENSY_PIN_ENC1B (1)
#define TEENSY_PIN_ENC2A (2)
#define TEENSY_PIN_ENC2B (3)
#define TEENSY_PIN_ENC3A (4)
#define TEENSY_PIN_ENC3B (5)
#define TEENSY_PIN_ENC4A (6)
#define TEENSY_PIN_ENC4B (7)
#define TEENSY_PIN_ENC5A (8)
#define TEENSY_PIN_ENC5B (11)
#define TEENSY_PIN_ENC6A (12)
#define TEENSY_PIN_ENC6B (13)
#define TEENSY_PIN_SERIAL_RX (9)
#define TEENSY_PIN_SERIAL_TX (10)
#define TEENSY_PIN_BATT_SENSE (14)
#define TEENSY_PIN_EXT_PWR (15)
#define TEENSY_PIN_SERVO (16)
#define TEENSY_PIN_LH_BALL_ESC (17)
#define TEENSY_PIN_I2C_SDA (18)
#define TEENSY_PIN_I2C_SCL (19)
#define TEENSY_PIN_RH_BALL_ESC (20)
#define TEENSY_PIN_HEADLIGHT_PWM (21)
#define TEENSY_PIN_DRIVE_LEFT (22)
#define TEENSY_PIN_DRIVE_RIGHT (23)

// i2c Addresses
#define TCA_ADDR 0x70
#define DISPLAY_ADDR 0x3c
#define IMU_ADDR 0x28
#define TOF_ADDR 0x29
typedef const std::unordered_map<uint8_t, String> HexIntMap;
HexIntMap I2C_ADDRESS_NAMES = {
   {TCA_ADDR, "Multiplexer"},
   {IMU_ADDR, "IMU"},
   {TOF_ADDR, "Distance sensor"},
   {DISPLAY_ADDR, "OLED display"}
};
// hardware counts
#define NUM_ENCODERS 6