#define programname "Battery_Voltage_Read_Test"
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
The test program has been written to check out that the hardware for reading the battery voltage has been assembled correctly
such that it is funtional. 
********************************************************************************************************************************
*/

#include <Arduino.h>
#include "Locator2_Board_Definitions.h"                     //specify PCB type


void loop()
{
  Serial.println(F("LED Flash"));
  led_Flash(5, 100);
  print_SupplyVoltage();
  Serial.println(); 
  delay(1500);
}


void led_Flash(unsigned int flashes, unsigned int delaymS)
{
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


unsigned int read_SupplyVoltage()
{
  //relies on 1V1 internal reference and 91K & 11K resistor divider
  //returns supply in mV @ 10mV per AD bit read
  unsigned int temp, SupplyVolts;
  byte index;

  analogReference(INTERNAL);
  for (index = 0; index <= 4; index++)                      //sample AD 3 times
  {
    temp = analogRead(SupplyAD);
    SupplyVolts = SupplyVolts + temp;
  }
  SupplyVolts = ((SupplyVolts / 5) * ADMultiplier);
  return SupplyVolts;
}


void print_SupplyVoltage()
{
  //get and display supply volts on terminal or monitor
  Serial.print(F("Supply Volts "));
  Serial.print(read_SupplyVoltage());
  Serial.println(F("mV"));
}


void setup()
{
 
  Serial.begin(38400);                       //setup Serial console ouput
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  Serial.println();

  pinMode(LED1, OUTPUT);                   //for PCB LED
  pinMode(13, OUTPUT);                     //for Pro Mini LED, Pin13

  read_SupplyVoltage();                    //dummy read, first read can be inaccurate

}
