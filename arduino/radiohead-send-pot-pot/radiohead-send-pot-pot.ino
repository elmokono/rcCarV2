#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

/*
  attiny85
  ---------
  5   - VCC
  3   - 2
  4   - 1
  GND - 0
  ---------
*/

//attiny85
#define TX 0
#define gasAxis 3
#define steerAxis 2
#define ledsPin 1

RH_ASK driver(2000, -1, TX, -1, false, 1);

uint8_t buffer[4];
int  minGas = 1023;
int  maxGas = 0;
int  minSteer = 1023;
int  maxSteer = 0;

void setup()
{
  driver.init();

  pinMode(steerAxis, INPUT_PULLUP);
  pinMode(gasAxis, INPUT_PULLUP);
  pinMode(ledsPin, INPUT_PULLUP);
}

void loop()
{
  int gas = analogRead(gasAxis);//0-1023
  if (gas < minGas) minGas = gas;
  if (gas > maxGas) maxGas = gas;
  //uint8_t gasByte = (gas - minGas) * 255.0 / (maxGas - minGas);

  int steer = analogRead(steerAxis);//0-1023
  if (steer < minSteer) minSteer = steer;
  if (steer > maxSteer) maxSteer = steer;
  //uint8_t steerByte = (steer - minSteer) * 255.0 / (maxSteer - minSteer);

  int leds = digitalRead(ledsPin);

  buffer[0] = 0x0;
  buffer[1] = map(gas, minGas, maxGas, 0, 255);
  buffer[2] = map(steer, minSteer, maxSteer, 0, 255);
  buffer[3] = leds == LOW ? 255 : 0;

  driver.send((uint8_t *)buffer, sizeof(buffer));
  //driver.waitPacketSent();

  delay(10);
}
