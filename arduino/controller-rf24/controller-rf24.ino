#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define GAS_PIN 3
#define STEER_PIN 2
#define LEDS_PIN 1
#define CE_PIN 8
#define CS_PIN 9

RF24 radio(CE_PIN, CS_PIN); // CE, CSN
const byte address[6] = "00010";

int minGas = 1023;
int maxGas = 0;
int minSteer = 1023;
int maxSteer = 0;

void setup()
{
  Serial.begin(9600);
  if (!radio.begin())
  {
    Serial.println("failed to initialize radio");
    while (true)
    {
      // error!
    }
  }

  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(124);
  radio.openWritingPipe(address);
  radio.setAutoAck(true);
  radio.stopListening();

  pinMode(STEER_PIN, INPUT_PULLUP);
  pinMode(GAS_PIN, INPUT_PULLUP);
  pinMode(LEDS_PIN, INPUT_PULLUP);
}

void loop()
{
  int gas = analogRead(GAS_PIN); // 0-1023
  if (gas < minGas)
    minGas = gas;
  if (gas > maxGas)
    maxGas = gas;

  int steer = analogRead(STEER_PIN); // 0-1023
  if (steer < minSteer)
    minSteer = steer;
  if (steer > maxSteer)
    maxSteer = steer;

  int leds = digitalRead(LEDS_PIN);

  uint8_t buffer[4];
  buffer[0] = 0x0;
  buffer[1] = map(gas, minGas, maxGas, 0, 255);
  buffer[2] = map(steer, minSteer, maxSteer, 0, 255);
  buffer[3] = leds == LOW ? 255 : 0;

  radio.write(&buffer, sizeof(buffer));
  delay(1000);
}