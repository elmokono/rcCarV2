// #include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <RH_ASK.h>
#include <Servo.h>

// raspberry pi pico
#define GAS_PIN 26
#define SERVO_PIN 27
#define RX_PIN 22 // 28

#define SERVO_DELAY_MS 15

RH_ASK driver(2000, RX_PIN, 20);
Servo servo;
unsigned long lastGasCheck = 0;
uint8_t lastGas = 0;
uint8_t lastSteer = 0;

void setup()
{
  Serial.begin(9600); // Debugging only

  if (!driver.init())
  {
    pinMode(PICO_DEFAULT_LED_PIN, OUTPUT);
    while (true)
    {
      digitalWrite(PICO_DEFAULT_LED_PIN, HIGH);
      delay(250);
      digitalWrite(PICO_DEFAULT_LED_PIN, LOW);
      delay(250);
    }
  }

  servo.attach(SERVO_PIN);
  servo.write(90);
  delay(SERVO_DELAY_MS);

  pinMode(GAS_PIN, OUTPUT);
  lastGasCheck = millis();

  digitalWrite(PICO_DEFAULT_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(PICO_DEFAULT_LED_PIN, LOW);

  Serial.println("RC Car V2 ready");
}

void checkGas(uint8_t gasLevel)
{
  if (gasLevel == lastGas)
  {
    return;
  }

  // Serial.print("',gas:");
  // Serial.print(gasLevel);
  analogWrite(GAS_PIN, gasLevel);
  lastGas = gasLevel;
}

void checkServo(uint8_t steerLevel)
{
  if (steerLevel == lastSteer)
  {
    return;
  }
  // Serial.print("',steer:");
  // Serial.print(steerLevel);

  // 0 >= steerLevel <= 255
  // angle between 40 and 140
  int value = (steerLevel * 100.0 / 255.0);
  servo.write(value + 40);
  lastSteer = steerLevel;
}

void loop()
{
  uint8_t buf[4];
  uint8_t buflen = sizeof(buf);

  if (driver.recv(buf, &buflen))
  {
    driver.printBuffer("Got:", buf, buflen);

    digitalWrite(PICO_DEFAULT_LED_PIN, HIGH);

    // buff[0]=0x0
    // buff[1]=0-255
    // buff[2]=0-255
    // buff[3]=0-3
    // char command = ((char *)buf)[0];
    // Serial.println(command);

    checkGas(buf[1]);
    checkServo(buf[2]);

    digitalWrite(PICO_DEFAULT_LED_PIN, LOW);
    lastGasCheck = millis();
  }
  else
  {
    // no signal, slow down to zero
    if (millis() - lastGasCheck > 250)
    {
      checkGas(0);
    }
  }

  delay(SERVO_DELAY_MS);
}