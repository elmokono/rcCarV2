#include <Arduino.h>
#include <RF24.h>
#include "RP2040_ISR_Servo.h"
#include  <Adafruit_NeoPixel.h>

#define LED_PIN 16 //rp2040 zero
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);

/*
This is for CAR
*/

/*
---------------------------
| MISO | SCK  | CE  | GND |
| #    | MOSI | CNS | VCC |
|                         |
|        (ANTENNA)        |
---------------------------
//rp2040 zero spi0 pins
//---------------------------
MISO = SPI RX - receiver
MOSI = SPI TX - transmitter (not used)
SCK = SPI Clock
CE = Custom PIN on RP2040
CNS = Custom PIN on RP2040
//---------------------------
//#define PIN_SPI_MISO  (0u)
//#define PIN_SPI_SS    (1u)
//#define PIN_SPI_SCK   (2u)
//#define PIN_SPI_MOSI  (3u)
*/
const byte address[6] = "00010";
#define CE_PIN 8
#define CS_PIN 7

#define SERVO_PIN 0
#define SERVO_DELAY_MS 50
#define SERVO_MIN_MICROS 800
#define SERVO_MAX_MICROS 2450
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 90
//#define SERVO_REF_ANGLE 45

#define GAS_PIN 1
#define GAS_ZERO_MICROS 0
#define GAS_MIN_MICROS 1500
#define GAS_MAX_MICROS 2000
#define GAS_LIMIT 1900

RF24 radio(CE_PIN, CS_PIN);
unsigned long lastGasCheck = 0;
uint8_t lastGas = 0;
uint8_t lastSteer = 0;
unsigned long lastServo = 0;
unsigned long lastAlive = 0;
int aliveLedState = LOW;


void doLed(int r, int g, int b)
{
  led.setPixelColor(0, led.Color(r,g,b,255)); 
  led.show();
}

void initRadio()
{
  if (!radio.begin())
  {
    Serial.println("Failed to init radio");
    doLed(255,0,0);
    while(true){};
  }

  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(124);
  radio.openReadingPipe(0, address);
  radio.setAutoAck(true);
  radio.startListening();
}

void initSteering()
{
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  RP2040_ISR_Servos.setupServo(SERVO_PIN, SERVO_MIN_MICROS, SERVO_MAX_MICROS);
  RP2040_ISR_Servos.setPosition(SERVO_PIN, SERVO_MIN_ANGLE);
  delay(500);
  RP2040_ISR_Servos.setPosition(SERVO_PIN, SERVO_MAX_ANGLE);
  delay(500);
  RP2040_ISR_Servos.setPosition(SERVO_PIN, (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)/2);
  lastServo = millis();
}

void initThrottle()
{
  pinMode(GAS_PIN, OUTPUT);
  digitalWrite(GAS_PIN, LOW);
  RP2040_ISR_Servos.setupServo(GAS_PIN, GAS_MIN_MICROS, GAS_MAX_MICROS);
  //arm sequence
  RP2040_ISR_Servos.setPosition(GAS_PIN, GAS_MIN_MICROS);
  delay(2000);
  RP2040_ISR_Servos.setPosition(GAS_PIN, GAS_MAX_MICROS);
  delay(1000);
  RP2040_ISR_Servos.setPosition(GAS_PIN, GAS_MIN_MICROS);

  lastGasCheck = lastAlive = millis();
}

void setup()
{
  led.begin(); 
  doLed(0,0,255);

  delay(2000);
  Serial.begin(9600); // Debugging only
  Serial.println("init..");
  doLed(255,255,0);

  initRadio();
  Serial.println("radio ok");
  
  initSteering();
  Serial.println("steering ok");

  initThrottle();
  Serial.println("throttle ok");

  doLed(0,255,0);
  Serial.println("RC Car V4 ready");
}

void checkGas(uint8_t gasLevel)
{
  if (gasLevel == lastGas)
  {
    return;
  }

#ifdef GAS_LIMIT
  int escGas = map(gasLevel, 0, 255, GAS_MIN_MICROS, GAS_LIMIT);
  RP2040_ISR_Servos.setPosition(GAS_PIN, escGas);
#else
  int escGas = map(gasLevel, 0, 255, GAS_MIN_MICROS, GAS_MAX_MICROS);
  RP2040_ISR_Servos.setPosition(GAS_PIN, escGas);
#endif
  lastGas = gasLevel;
  // Serial.println(gasLevel);
}

void checkServo(uint8_t steerLevel)
{
  //no changes in steer or minimun time for servo has not been reached
  if (steerLevel == lastSteer || millis() - lastServo < SERVO_DELAY_MS)
  {
    return;
  }

  // Serial.print("',steer:");
  // Serial.print(steerLevel);

  // 0 >= steerLevel <= 255
  // SERVO_MAX_ANGLE >= steerLevel <= 100+SERVO_MAX_ANGLE
  // angle between 40 and 140

//50 - 90 - 130

  //int value = (steerLevel * SERVO_MAX_ANGLE * 2 / 255.0);
  //RP2040_ISR_Servos.setPosition(SERVO_PIN, value + SERVO_REF_ANGLE - SERVO_MAX_ANGLE);
  int value = map(steerLevel, 0, 255, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  RP2040_ISR_Servos.setPosition(SERVO_PIN, value);
  lastSteer = steerLevel;
  lastServo = millis();
}

void loop()
{
  if (radio.available())
  {
    uint8_t buf[32];
    radio.read(&buf, sizeof(buf));
    digitalWrite(LED_PIN, HIGH);
    /*Serial.print("command received ");
    Serial.print(buf[0]);Serial.print(" ");
    Serial.print(buf[1]);Serial.print(" ");
    Serial.print(buf[2]);Serial.print(" ");
    Serial.print(buf[3]);Serial.println();*/

    // buff[0]=0x0
    // buff[1]=0-255
    // buff[2]=0-255
    // buff[3]=0-3

    checkGas(buf[1]);
    checkServo(buf[2]);

    digitalWrite(LED_PIN, LOW);
    lastGasCheck = millis();
  }
  else
  {
    // signal lost, slow down to zero
    if (millis() - lastGasCheck > 250)
    {
      checkGas(0);
    }
  }

  if (millis() - lastAlive > 2000)
  {
    aliveLedState = aliveLedState == LOW ? HIGH : LOW;
    digitalWrite(LED_PIN, aliveLedState);
    lastAlive = millis();
  }

  delay(10);
}