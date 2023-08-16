/*
This is for CAR

---------------------------
| MISO | SCK  | CE  | GND |
| #    | MOSI | CNS | VCC |
|                         |
|        (ANTENNA)        |
---------------------------
*/
/*
pico spi0 pins
---------------------------
#define PIN_SPI_MISO  (16u)
#define PIN_SPI_MOSI  (19u)
#define PIN_SPI_SCK   (18u)
#define PIN_SPI_SS    (17u)
*/

#include <Arduino.h>
#include <Servo.h>
#include <RF24.h>

const byte address[6] = "00010";
#define CE_PIN 22
#define CS_PIN 21

#define SERVO_PIN 27
#define SERVO_DELAY_MS 50

#define GAS_PIN 26
// #define GAS_LIMIT 128

#define SERVO_MAX_ANGLE 35

RF24 radio(CE_PIN, CS_PIN);
Servo servo;
unsigned long lastGasCheck = 0;
uint8_t lastGas = 0;
uint8_t lastSteer = 0;
unsigned long lastServo = 0;
unsigned long lastAlive = 0;
int aliveLedState = LOW;

void initRadio()
{
  if (!radio.begin())
  {
    Serial.println("Failed to init radio");
    pinMode(PICO_DEFAULT_LED_PIN, OUTPUT);
    while (true)
    {
      digitalWrite(PICO_DEFAULT_LED_PIN, HIGH);
      delay(50);
      digitalWrite(PICO_DEFAULT_LED_PIN, LOW);
      delay(50);
    }
  }

  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(124);
  radio.openReadingPipe(0, address);
  radio.setAutoAck(true);
  radio.startListening();
}

void setup()
{
  Serial.begin(9600); // Debugging only
  initRadio();

  servo.attach(SERVO_PIN);
  servo.write(90+SERVO_MAX_ANGLE);
  delay(500);
  servo.write(90-SERVO_MAX_ANGLE);
  delay(500);
  servo.write(90);
  lastServo = millis();

  pinMode(GAS_PIN, OUTPUT);
  lastGasCheck = lastAlive = millis();

  for (int i = 0; i < 5; i++)
  {
    delay(100);
    digitalWrite(PICO_DEFAULT_LED_PIN, HIGH);
    delay(1000);
    digitalWrite(PICO_DEFAULT_LED_PIN, LOW);
  }

  Serial.println("RC Car V3 ready");
}

void checkGas(uint8_t gasLevel)
{
  if (gasLevel == lastGas)
  {
    return;
  }

#ifdef GAS_LIMIT
  analogWrite(GAS_PIN, gasLevel > GAS_LIMIT ? GAS_LIMIT : gasLevel);
#else
  analogWrite(GAS_PIN, gasLevel);
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

  int value = (steerLevel * SERVO_MAX_ANGLE * 2 / 255.0);
  servo.write(value + 90 - SERVO_MAX_ANGLE);
  lastSteer = steerLevel;
  lastServo = millis();
}

void loop()
{
  if (radio.available())
  {
    uint8_t buf[32];
    radio.read(&buf, sizeof(buf));
    digitalWrite(PICO_DEFAULT_LED_PIN, HIGH);
    // Serial.println("command received");

    // buff[0]=0x0
    // buff[1]=0-255
    // buff[2]=0-255
    // buff[3]=0-3

    checkGas(buf[1]);
    checkServo(buf[2]);

    digitalWrite(PICO_DEFAULT_LED_PIN, LOW);
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
    digitalWrite(PICO_DEFAULT_LED_PIN, aliveLedState);
    lastAlive = millis();
  }

  delay(10);
}