#include <Arduino.h>
#include <SerialTransfer.h>
#include <Servo.h>

uint8_t motorPinLeft = 22;
uint8_t motorPinRight = 23;

Servo motorLeft;
Servo motorRight;

struct MotorSpeeds
{
  float left;
  float right;
} motorSpeeds;

SerialTransfer myTransfer;

int8_t step = 1;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(1152000);
  while (!Serial)
  {
  };

  while (!Serial2)
  {
  };

  motorLeft.attach(motorPinLeft);
  motorRight.attach(motorPinRight);

  myTransfer.begin(Serial2);
  motorSpeeds.left = 0;
  motorSpeeds.right = 0;
}

void loop()
{
  // Serial.println("waiting for data");
  if (myTransfer.available())
  {
    uint8_t recSize = 0;
    myTransfer.rxObj(motorSpeeds, sizeof(motorSpeeds), recSize);
    Serial.print(motorSpeeds.left);
    Serial.print(' ');
    Serial.print(motorSpeeds.right);
    Serial.println();

    motorLeft.writeMicroseconds(map(motorSpeeds.left, -100, 100, 1000, 2000));
    motorRight.writeMicroseconds(map(motorSpeeds.right, -100, 100, 1000, 2000));
  }
  // else if (myTransfer.status < 0)
  // {
  //   Serial.print("ERROR: ");
  //   Serial.println(myTransfer.status);
  // }
  // else
  // {
  //   Serial.print("waiting:");
  //   Serial.println(myTransfer.status);
  // }
  delay(20);
}