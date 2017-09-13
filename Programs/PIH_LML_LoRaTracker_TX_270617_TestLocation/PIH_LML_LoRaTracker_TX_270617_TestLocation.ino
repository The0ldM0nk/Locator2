#define programname "PIH LML LoRaTracker TX "
#define programversion "V1.2"
#define dateproduced "270617"
#define aurthorname "Stuart Robinson"

#define LoRaTracker_Locator2                 //select board type here
/*
LoRaTracker Programs

Copyright of the author Stuart Robinson

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.


To Do:


Changes:
23/04/16 TXPower and timeout were reversed when calling lora_send
27/06/17 - Added support for latest LoRaTracker board, LoRaTracker_Locator1 (DRF1278F only) 
*/



#define UBLOX                                //UBLOX GPS Needs GLONASS Turned off, needs specific setup
//#define RCPulseMode
//#define Timeout

#define Frequency1 434.400                   //main frequency
#define FreqOffset 0.0                       //adjustment for frequency in Mhz, assumed at room temp
#define TXPower 10                           //transmit power in dBm, 10 = 10dBm = 10mW
#define ThisNode '1'                         //Node number as ASCII character for this device

#define MinsToLost 15                        //minutes before lost mode automatically engaged.
#define TXDelaySecs 27                       //delay in seconds between position transmissions, needed to ensure duty cycle limits kept, normally 10%
#define TXLostDelaySecs 20                   //delay in seconds between lost mode transmissions
#define Output_len_max 126                   //max length of outgoing packet

#include <Arduino.h>
#include <SPI.h>
#include "Tracker_Definitions.h"
#include "LoRa.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>


TinyGPSPlus gps;                             //create the TinyGPS++ object
SoftwareSerial ss(GPSRX, GPSTX);             //create the serial connection to the GPS device
#include "GPS.h"


//Program constants
const int pulseerrorlimit = 100; 	           //number of RC error pulses needed to trigger lost condition, 255 max
const int holddifference = 30;		           //if differance between two RC pulses is less than this, then hold is assumed
const int RCpulseshort = 750; 		           //in uS, lower than this RC pulse is assumed not to be valid
const int RCpulselong = 2500; 		           //in uS, higher than this RC pulse is assumed not to be valid
const unsigned long GPSerrorLimit = 129000;  //number of times around loop with no characters before an error is flagged, around 5seconds
const byte inc_on_error = 5;                 //number to increase pulse error count on a fail
const byte dec_on_OK = 10;                   //number to decrease pulse error count on a pass


//Program Variables
unsigned int UpMins;                         //Calculation Minutes since startup
unsigned int RCPulseLen;                     //length of RC pulse
unsigned int pulsewidthlast = 1500; 	       //measured width in uS of last RC pulse read, used for hold check
unsigned int GPSFixes = 0;                   //count of number of valid GPS Fixes
unsigned long int GPSErrors = 0;             //to be able to check if GPS Still working
int pulseerrorcount = 50;  	                 //number of RC pulse errors recorded, start at 50
int SupplyVolts;                             //to keep local supply volts
byte firstGPSfixflag = 0;                    //used to indicates the first GPS fix





void loop()
{
  int i;
  while (ss.available() > 0)
    gps.encode(ss.read());

  if (gps.location.isUpdated() && gps.altitude.isUpdated())
  {
    ss.end();
    GPSFixes++;
    Serial.print(GPSFixes);
    Serial.print(F(" Fix Updated "));
    
    if (firstGPSfixflag == 0)
    {
      firstGPSfixflag = 1;
      for (i = 0; i <= 9; i++)
      {
        lora_Tone(1500, 75, 2);                  //transmit an FM tone, 1500hz, 75ms, 2dBm
        delay(50);
      }
      delay(1000);
    }
    
    //SendLocation(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
    SendLocation(51.48230, -3.18136, 120);
    checkforlost();
    ss.begin(GPSBaud);                            //startup soft serial for GPS again
    tx_Delay(TXDelaySecs);
  }

  GPSErrors++;

  if (GPSErrors > GPSerrorLimit)                  //equivalent to around a 10 second delay
  {
    GPS_Error();
  }
}


void tx_Delay(unsigned int lTXDelaySecs)
{
  //delay transmissions in order to meet regulations on duty cycle
  Serial.print(F("Duty Cycle Delay "));
  Serial.print(lTXDelaySecs);
  Serial.println(F(" Seconds"));
  lTXDelaySecs = lTXDelaySecs * 1000;
  delay(lTXDelaySecs);
}


void GPS_Error()
{
  //process the GPS error
  ss.end();
  if (gps.charsProcessed() < 100)
  {
    Serial.println(F("No GPS Found"));
    SendGPSError();
    systemerror();
  }

  if ((GPSErrors > GPSerrorLimit) & (GPSFixes == 0))
  {
    GPSErrors = 0;
    Serial.println(F("No GPS Fix"));
    SendGPSNoFix();
  }

  if ((GPSErrors > 129000) & (GPSFixes > 0))
  {
    GPSErrors = 0;
    //ss.end();
    Serial.print(F("GPS Error  "));
    SendGPSError();                                    //dont have to send this, but useful for diagnostics
    delay(500);
    SendLocation(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
    checkforlost();
  }
  ss.begin(GPSBaud);                                   //startup soft serial for GPS again
}



int GetSupplyVolts()
{
  //relies on 1V1 internal reference and 91K & 11K resistor divider
  //returns supply in mV @ 10mV per AD bit read
  int temp;
  byte i;
  SupplyVolts = 0;
  
  analogReference(INTERNAL);
  for (i = 0; i <= 2; i++)                      //sample AD 3 times
  {
    temp = analogRead(SupplyAD);
    SupplyVolts = SupplyVolts + temp;
  }
  SupplyVolts = (int) ((SupplyVolts / 3) * ADMultiplier);
  return SupplyVolts;
}


void DisplaySupplyVolts()
{
  //get and display supply volts on terminal or monitor
  Serial.print(F("TX Supply Volts "));
  Serial.print((int) GetSupplyVolts());
  Serial.println(F("mV"));
}


void SendSavedLocation()
{
  //transmit the location saved in EEPROM
  float Tlat;
  float Tlng;
  float Talt;

  Serial.print(F("Saved Location "));
  EEPROM.get(0x10, Tlat);
  Serial.print(Tlat, 5);
  Serial.print(F(" "));
  EEPROM.get(0x18, Tlng);
  Serial.print(Tlng, 5);
  Serial.print(F(" "));
  EEPROM.get(0x20, Talt);
  Serial.println(Talt, 0);
  SendLocation(Tlat, Tlng, Talt);
}


void resetcheck()
{
  //get number of resets from EEPROM and increment
  int lvar1;

  EEPROM.get(0, lvar1);
  lvar1++;
  EEPROM.put(0, lvar1);
  Serial.print(F("Reset Count "));
  Serial.println(lvar1);
}


void systemerror()
{
  //indicates a sytem error, probably with GPS
  digitalWrite(PLED1, HIGH);
  lora_Tone(500, 4000, 10);                           //transmit an FM tone, 500hz, 4000ms
  digitalWrite(PLED1, LOW);

  while (1)
  {
    digitalWrite(PLED1, HIGH);
    delay(50);
    digitalWrite(PLED1, LOW);
    delay(50);
  }
}


void  saveLocation()
{
  //store current GPS location in EEPROM
  EEPROM.put(0x10, gps.location.lat());
  EEPROM.put(0x18, gps.location.lng());
  EEPROM.put(0x20, gps.altitude.meters());
}


void  clearLocation()
{
  //clear stored GPS location in EEPROM
  float i = 0;

  EEPROM.put(0x10, i);
  EEPROM.put(0x18, i);
  EEPROM.put(0x20, i);
}


void PrintPayload(byte lCount)
{
  //print the received payload to serial monitor
  int i;

  for (i = 0; i <= lCount; i++)
  {
    Serial.write(lora_TXBUFF[i]);
  }
}


int BuildShortPayload(char *lora_TXBUFF)
{
  //we have a location to send, build the payload in buffer
  int Count;

  char LatString[12], LonString[12], AltString[10];
  dtostrf(gps.location.lat(), 7, 5, LatString);
  dtostrf(gps.location.lng(), 7, 5, LonString);
  dtostrf(gps.altitude.meters(), 0, 0, AltString);
  snprintf(lora_TXBUFF,
           Output_len_max,
           "%s,%s,%s,",
           LatString,
           LonString,
           AltString
          );
  Count = strlen(lora_TXBUFF);                 //how long is the array ?
  return Count;
}


void LostMode()
{
  //sadly all is lost
  unsigned int i;

  Serial.println(F("Lost Mode  "));
  gpsOff();                                    //turn off GPS
  delay(2000);
  SendisLost();                                //alert receiver over lost mode
  delay(3000);

  while (1)
  {
    SendSavedLocation();
    delay(1000);
    FindMeTones();
    i = (TXLostDelaySecs * 1000);
    delay(i);
  }
}


byte islost()
{
  //check to see if tracker is in lost mode
  if (EEPROM.read(0X3FF) == 0xAA)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}


void checkforlost()
{
  //check to see if tracker should go into lost mode

#ifdef Timeout
  checkfortimeout();
#endif

#ifdef RCPulseMode
  checkRCpulse();
#endif
}


void setLost()
{
  //set lost mode, stored in EEPROM
  EEPROM.write(0x3FF, 0XAA);                               //set lost mode
}


void checkRCpulse()
{
  //reads RC pulse and checks for errors and hold
  ss.end();
  RCPulseLen = pulseIn(RCPulse, HIGH);
  Serial.print(F("Servo Pulse "));
  Serial.print(RCPulseLen);
  Serial.println(F("uS"));

  if ( (RCPulseLen < RCpulseshort) ||  (RCPulseLen > RCpulselong) )
  {
    pulseerrorcount = pulseerrorcount + inc_on_error;
    lora_Tone(500, 125, 10);                               //transmit an short pip to flag RC pulse error
  }
  else
  {
    pulseerrorcount = pulseerrorcount - dec_on_OK;
    if  (pulseerrorcount < 0)
    {
      pulseerrorcount = 0;
    }
  }

  if ( (RCPulseLen < (pulsewidthlast - holddifference)) ||  (RCPulseLen > (pulsewidthlast + holddifference)) )
  {
    //nothing to do
  }
  else
  {
    Serial.println(F("Pulse Hold !"));
    pulseerrorcount = pulseerrorcount + 5;
  }

  Serial.print(F("RC Pulse Error Count "));
  Serial.println(pulseerrorcount);
  pulsewidthlast = RCPulseLen;

  if  (pulseerrorcount > pulseerrorlimit)
  {
    Serial.println(F("RC Pulse Fail"));
    saveLocation();
    setLost();
    LostMode();
  }
  ss.begin(GPSBaud);                                     //startup soft serial for GPS again
  delay(1000);
}


void checkfortimeout()
{
  //check if timeout has expired and whether tracker should go to lost
  UpMins = (millis() / 60000);                          //calculate upseconds
  Serial.print(F("UpMins "));
  Serial.println(UpMins);
  if (UpMins >= MinsToLost)
  {
    Serial.println(F("UpMins Timeout "));
    saveLocation();
    setLost();
    LostMode();
  }
}


void FindMeTones()
{
  //transmits the FM search tones, descending power and rising frequency
  Serial.println(F("Send Find Me Tones"));
  digitalWrite(PLED1, HIGH);
  lora_Tone(600, 250, 10);                                //transmit an FM tone, 600hz, 250ms, 10dBm
  digitalWrite(PLED1, LOW);
  delay(200);
  digitalWrite(PLED1, HIGH);
  lora_Tone(1000, 300, 5);
  digitalWrite(PLED1, LOW);
  delay(200);
  digitalWrite(PLED1, HIGH);
  lora_Tone(1600, 600, 2);
  digitalWrite(PLED1, LOW);
  Serial.println();
}


void led_FlashStart()
{
  //flash LED to show tracker is alive
  byte i;

  for (i = 0; i <= 4; i++)
  {
    digitalWrite(PLED1, HIGH);
    delay(100);
    digitalWrite(PLED1, LOW);
    delay(100);
  }
}


void setup_LoRa_for_Data()
{
  //set uop the appropriate LoRa modem settings
  init_LoRaSlow();
}


//***********************************************************************
//Packet transmits
//***********************************************************************

void SendAlive()
{
  //send s power up packet so that RX knows tracker is alive
  int Volts;

  Serial.print(F("Send PowerUp "));
  Volts = GetSupplyVolts();
  Serial.print(Volts);
  Serial.println(F("mV"));
  digitalWrite(PLED1, HIGH);
  lora_TXBUFF[1] = (Volts / 256);                                //MSB of supply volts
  lora_TXBUFF[0] = (Volts - (lora_TXBUFF[1] * 256));             //LSB of supply volts
  setup_LoRa_for_Data();
  lora_Send(0, 1, PowerUp, Broadcast, ThisNode, 10, TXPower);	 //send an alive packet, contains supply volts
  digitalWrite(PLED1, LOW);

}


void SendisLost()
{
  //send an IsLost packet to notify RX that tracker is lost
  GPSErrors = 0;
  setup_LoRa_for_Data();
  lora_TXBUFF[0] = 0;
  lora_Send(0, 0, IsLost, '*', ThisNode, 10, TXPower);	         //send a lost mode message
}


void SendLocation(float Tlat, float Tlng, float Talt)
{
  //send the GPS data location as LoRa
  int Count;

  GPSErrors = 0;
  char LatString[12], LonString[12], AltString[10];
  dtostrf(Tlat, 7, 5, LatString);
  dtostrf(Tlng, 7, 5, LonString);
  dtostrf(Talt, 0, 0, AltString);
  snprintf(lora_TXBUFF,
           Output_len_max,
           "%s,%s,%s,",
           LatString,
           LonString,
           AltString
          );
  Count = strlen(lora_TXBUFF);  //how long is the array ?
  Serial.print(F("LoRa Send "));
  PrintPayload(Count);
  setup_LoRa_for_Data();
  lora_Send(0, Count, LMLPayload, Broadcast, ThisNode, 10, TXPower);   //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
  Serial.println();
}

void SendGPSNoFix()
{
  //send a GPS error, NoFix
  GPSErrors = 0;
  setup_LoRa_for_Data();
  lora_TXBUFF[0] = 0; 
  lora_Send(0, 0, NoFix, Broadcast, ThisNode, 10, TXPower);	       //send a 'No GPS Fix' Message
  delay(1000);
  lora_Tone(1500, 200, 10);                                            //transmit an FM tone, 1000hz, 100ms
}


void SendGPSError()
{
  //send a GPS error, NoGPS found maybe?
  GPSErrors = 0;
  setup_LoRa_for_Data();
  lora_TXBUFF[0] = 0;
  lora_Send(0, 0, NoGPS, Broadcast, ThisNode, 10, TXPower);	       //send a 'No GPS ' Message
}

//***********************************************************************


void setup()
{
  pinMode(GPSPOWER, OUTPUT);	                 //setup pin for GPS Power Control
  gpsOn();                                       //power up GPS to allow time for init

  pinMode(PLED1, OUTPUT);		         //setup pin for PCB LED
  led_FlashStart();

  Serial.begin(9600);                            //setup Serial console ouput
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  Serial.println();

  Serial.println();
  pinMode(RCPulse, INPUT);		         //setup pin for RC Pulse input
  pinMode(lora_PReset, OUTPUT);		         //setup pin for RFM98 reset line
  pinMode (lora_PNSS, OUTPUT);		         //setup pin for RFM98 slave select
  digitalWrite(lora_PNSS, HIGH);

  SPI.begin();				         //initialize SPI
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  init_LoRa();		                              //do the initial LoRa Setup

  digitalWrite(PLED1, HIGH);
  lora_Tone(1000, 1000, 2);                     //transmit an FM tone, 1000hz, 1000ms,2dBm
  digitalWrite(PLED1, LOW);
  delay(1000);

  resetcheck();

  if (islost())                                  //after reset is lost mode still set ?
  {
    Serial.println(F("Lost is Set"));
    EEPROM.write(0x3FF, 0X00);  //Clear lost mode
    Serial.println(F("Lost now Cleared"));
    FindMeTones();                               //sending Find me tones at startup indictaes lost mode set
    EEPROM.write(0x3FF, 0XAA);                   //set lost mode again
    Serial.println(F("Lost is Set"));
  }

  if (islost())
  {
    Serial.println(F("Continue Lost Mode"));
    LostMode();
  }
  else
  {
    clearLocation();                             //clear saved location from EEPROM
  }

  delay(1000);
  digitalWrite(PLED1, HIGH);
  SendSavedLocation();
  digitalWrite(PLED1, LOW);

  delay(5000);

  Serial.println(F("Tracker Starting"));

  SendAlive();                                   //send alive packet which reports supply voltage

  delay(1000);
  Serial.println();
  gpsSetup();
  checkRCpulse();                                //check RC pulse regardless
}

