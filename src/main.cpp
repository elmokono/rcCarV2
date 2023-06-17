// #include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <RH_ASK.h>
#include <Servo.h>

// raspberry pi pico
#define GAS_PIN 26
#define SERVO_PIN 27
#define RX_PIN 28

#define SERVO_DELAY_MS 15

RH_ASK driver(2000, RX_PIN, 20);
Servo servo;

void setup()
{
  Serial.begin(9600); // Debugging only

  if (!driver.init())
  {
    pinMode(PICO_DEFAULT_LED_PIN, OUTPUT);
    while (true)
    {
      digitalWrite(PICO_DEFAULT_LED_PIN, HIGH);
      delay(500);
      digitalWrite(PICO_DEFAULT_LED_PIN, LOW);
      delay(500);
    }
  }

  servo.attach(SERVO_PIN);
  servo.write(90);
  delay(50);

  pinMode(GAS_PIN, OUTPUT);

  digitalWrite(PICO_DEFAULT_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(PICO_DEFAULT_LED_PIN, LOW);

  Serial.println("RC Car V2 ready");
}

void checkGas(char command)
{
  if (command == 'F')
  {
    analogWrite(GAS_PIN, 255);
  }
  if (command == 'S')
  {
    analogWrite(GAS_PIN, 0);
  }
}

void checkServo(char command)
{
  if (command == 'L')
  {
    servo.write(40);
    delay(SERVO_DELAY_MS);
  }
  if (command == 'R')
  {
    servo.write(140);
    delay(SERVO_DELAY_MS);
  }
}

void loop()
{
  uint8_t buf[1];
  uint8_t buflen = sizeof(buf);

  if (driver.recv(buf, &buflen)) // Non-blocking
  {
    digitalWrite(PICO_DEFAULT_LED_PIN, HIGH);

    char command = ((char *)buf)[0];
    Serial.println(command);

    checkGas(command);

    checkServo(command);

    digitalWrite(PICO_DEFAULT_LED_PIN, LOW);
  }
}