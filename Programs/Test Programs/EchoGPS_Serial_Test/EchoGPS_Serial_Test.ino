#define programname "EchoGPS_Serial_Test"
#define programversion "V1.0"
#define dateproduced "151117"
#define aurthorname "Stuart Robinson"


/*
*******************************************************************************************************************************
Easy Build LoRaTracker Programs for Arduino

Copyright of the author Stuart Robinson - 15/11/17

http://www.LoRaTracker.uk

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
free from errors.
  
To Do:
  
*******************************************************************************************************************************
*/

/*
*******************************************************************************************************************************
  The purpose of this program is to check that a Generic Serial GPS is working. Characters are read from the GPS
  and sent to the Serial monitor at 115200 baud.
*******************************************************************************************************************************
*/


#include <Arduino.h>
#include "Locator2_Board_Definitions.h"     //select board type here
#include "Program_Definitions.h"            //definitions for programs

const unsigned int GPSBaud = 9600;          //baud rate of GPS
const byte GPS_Reply_Size = 12;             //size of GPS reply buffer

#include <NeoSWSerial.h>                    //https://github.com/SlashDevin/NeoSWSerial  
NeoSWSerial GPSserial(GPSRX, GPSTX);        //this library is more relaible at GPS init than software serial

#include "Generic_SerialGPS.h"


void loop()                    
{
  while (GPSserial.available() > 0)
  Serial.write(GPSserial.read());
}


void led_Flash(unsigned int flashes, unsigned int delaymS)
{
  unsigned int index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


void setup()
{
  Serial.begin(115200);                      //connect at 115200 so we can read the GPS fast enough and also spit it out
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));

  pinMode(LED1, OUTPUT);                     //for Watchdog pulse input
  led_Flash(5, 100);

  pinMode(GPSPOWER, OUTPUT);                //setup pin for GPS Power Control, in case its in use
  GPS_On(DoGPSPowerSwitch);                 //this will power the GPSon
  GPS_Setup();

}


