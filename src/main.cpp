#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <RH_ASK.h>
#include <Servo.h>

int rfPin = 28;
int servoPin = 27;

RH_ASK driver(2000, rfPin, 29);
Servo myservo;

void setup()
{
  Serial.begin(9600); // Debugging only
  if (!driver.init())
    Serial.println("init failed");

  myservo.attach(servoPin);
  myservo.write(90);
  delay(15);

  pinMode(PICO_DEFAULT_LED_PIN, OUTPUT);
  digitalWrite(PICO_DEFAULT_LED_PIN, LOW);
}

void loop()
{
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);

  if (driver.recv(buf, &buflen)) // Non-blocking
  {
    char command = ((char *)buf)[0];
    // Message with a good checksum received, dump it.'

    Serial.print("Received(");
    Serial.print(buflen);
    Serial.print(",");
    Serial.print(command);
    Serial.println(")");
    // driver.printBuffer(buf, buf, buflen);

    if (command == 'L')
    {
      myservo.write(40);
      delay(15);
    }
        if (command == 'R')
    {
      myservo.write(140);
      delay(15);
    }
  }
}