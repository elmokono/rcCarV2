// #include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <RH_ASK.h>
#include <Servo.h>
#include <RH_NRF24.h>
#include <RHSoftwareSPI.h>

// raspberry pi pico
#define GAS_PIN 26
#define SERVO_PIN 27

#define CE_PIN 18
#define CS_PIN 21
#define SCK_PIN 19
#define MISO_PIN 20
#define MOSI_PIN 22

#define SERVO_DELAY_MS 15

RHSoftwareSPI spi;
RH_NRF24 nrf24(CE_PIN, CS_PIN, spi);
Servo servo;
unsigned long lastGasCheck = 0;
uint8_t lastGas = 0;
uint8_t lastSteer = 0;

void setup()
{
  Serial.begin(9600); // Debugging only

  spi.setPins(MISO_PIN, MOSI_PIN, SCK_PIN);  
  if (!nrf24.init() || !nrf24.setChannel(1) || !nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
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
  if (nrf24.available())
  {
    uint8_t buf[4];
    uint8_t len = sizeof(buf);
    if (nrf24.recv(buf, &len))
    {
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