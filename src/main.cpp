#include <Arduino.h>
#include <SerialTransfer.h>

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
  myTransfer.begin(Serial2);
  motorSpeeds.left = 0;
  motorSpeeds.right = 0;
  while (!Serial)
  {
  };

  while (!Serial2)
  {
  };
}

void loop()
{
  Serial.println("waiting for data");
  if (myTransfer.available())
  {
    uint8_t recSize = 0;
    myTransfer.rxObj(motorSpeeds, sizeof(motorSpeeds), recSize);
    Serial.print(motorSpeeds.left);
    Serial.print(' ');
    Serial.print(motorSpeeds.right);
    Serial.println();
  }
  else if (myTransfer.status < 0)
  {
    Serial.print("ERROR: ");
    Serial.println(myTransfer.status);
  }
  else
  {
    Serial.print("waiting:");
    Serial.println(myTransfer.status);
  }
  delay(20);
}