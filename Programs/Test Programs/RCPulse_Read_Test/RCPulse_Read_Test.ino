#define programname "RCPulse_Read_Test"
#define programversion "V1.0"
#define dateproduced "15/11/2017"
#define aurthorname "Stuart Robinson"

/*
**************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 15/11/2017

http://www.LoRaTracker.uk

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
free from errors.

**************************************************************************************************
*/

/*
********************************************************************************************************************************
The test program has been written to check out that the hardware and connectors for the tracker boards have been assembled correctly
such that there are funtional, with no open or short circuits. 
********************************************************************************************************************************
*/

#include <Arduino.h>
#include "Locator2_Board_Definitions.h"          //specify PCB type board definition file


unsigned int RCPulseLen;


void loop()
{
  Serial.println();
  Serial.println(F("LED Flash"));
  led_Flash(5,100);
  
  Serial.println(F("Checking RC Pulse"));
  checkRCpulse();

  delay(1500);
}


void led_Flash(unsigned int flashes, unsigned int delaymS)
{
  //flash LED to show tracker is alive
  unsigned int index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    digitalWrite(13, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    digitalWrite(13, LOW);
    delay(delaymS);
  }
}


void checkRCpulse()
{
  //reads RC pulse 
  pinMode(RCPulse, INPUT); 			        
  RCPulseLen = pulseIn(RCPulse, HIGH);
  Serial.print(F("Servo Pulse "));
  Serial.print(RCPulseLen);
  Serial.println(F("uS"));
}


void setup()
{
  pinMode(LED1, OUTPUT); 			               //for PCB LED
  pinMode(13, OUTPUT);                       //for Pro Mini LED, if fitted
  Serial.begin(38400);                       //setup Serial console ouput
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  Serial.println();
 
}
