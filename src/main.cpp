#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>

uint lastMillis;

void setup()
{
  while (!Serial) { delay(10); }
  Serial.begin(115200);
  
  lastMillis = millis();

  Serial.println("hi!");
  
}

void loop(void)
{
  
  
}
