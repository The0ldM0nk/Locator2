#define programname "PIH LML LoRaTracker RX "
#define programversion "V1.1"
#define dateproduced "270617"
#define aurthorname "Stuart Robinson"

                         
#define LoRaTracker_Locator2          //specify PCB type

/*
LoRaTracker Programs

Copyright of the author Stuart Robinson

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

This receiver program can use an attached GPS. The program will then receive
the GPS co-ordinates of the tracker transmitter, and using the local co-ordinates from the attached
GPS, will calculate and display the distance and direction to the tracker transmitters last known position.

This will drive a Digole serial LCD display or my Nokia 5110 backpack. See the constant definition for 
'endcommand' in the Display.h file for more details. 

Uses SendOnlySoftwareSerial. Copy the SendOnlySoftwareSerial.h and SendOnlySoftwareSerial.cpp files from here;

https://github.com/disq/i2c-gps-nav/tree/master/I2C_GPS_NAV

Create a folder called \SendOnlySoftwareSerial in your Arduino Libraries folder and put the two files there. 



To Do:

Changes:
270617 - Added definitions for LoRaTracker_Locator1 which is Version 4 of the DRF1278F tracker

*/

#define GPS                         //comment out if no local GPS is used, cannot use Digole or Backpack display with Local GPS
#define NMEAUplink                  //comment out if no NMEA uplink conversion is required

//#define Debug                       //Uses test Values for Dist, dir

#define Frequency1 434.400          //main frequency
#define FreqOffset 0.0              //adjustment for frequency in Mhz, assumed at room temp
#define ThisNode '2'

#define PayloadArraySize 20         //number of items to decode from incoming packet
#define GPSerrorLimit 10000         //number of GPS errors before a timeout
//#define messagecol 0                //col where messages start on screen
//#define messagerow 5                //row where mesages start on screen
#define defaultfont 6                //default font, 6 for Nokia 5110 backpack
#define defaultfont 10               //default font, 10 for digole displays and ILI9341 backpack 

#include <Arduino.h>
#include "Tracker_Definitions.h"

#include <SoftwareSerial.h>
SoftwareSerial ss(GPSRX, GPSTX);   //The serial connection to the GPS device

#include <SendOnlySoftwareSerial.h>
SendOnlySoftwareSerial disp (DisplayTX);  // Display Tx pin

#include <SPI.h>
#include <EEPROM.h>

#include "LoRa.h"
#include "Generate_NMEA.h"

//Program Variables
unsigned int Halt;
unsigned int Talt;
unsigned int TXSupplyVolts;
unsigned int RXSupplyVolts;
unsigned int dir;
unsigned int dist;
unsigned int Tresets;
unsigned int TXGPSfix;
unsigned long GPSErrors;
int SupplyVolts;

byte RXGPSfix = 0;
byte remote_fix = 0;          //flag byte to indicate a fix from the tracker is available

float Tlat;                   //define the GPS lat and long variables for tracker
float Tlon;
float Hlat;                   //define the GPS lat and long variables for home (RX)
float Hlon;

String results[PayloadArraySize];
String Outputstring;

#include "Display.h"
#include "Screens.h"


#include <TinyGPS++.h>
TinyGPSPlus gps;              //create the TinyGPS++ object
#include "GPS.h"


#ifdef GPS                    //this is the main loop when a local GPS is connected
void loop()
{
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
    {
      ss.end();
      checkforpacket();
      processGPS();
      ss.begin(GPSBaud);      //re-start soft serial for GPS
    }
  if (millis() > 10000 && gps.charsProcessed() < 10)
  {
    display_Setfont(defaultfont);
    display_SetCurPos(0, 5);
    display_Text("RX GPS ERROR");
    Serial.println(F("RX GPS ERROR"));
    systemerror();
  }
}
#endif

#ifndef GPS                  //this is the main loop when there is no local GPS connected
void loop()
{
  checkforpacket();
  delay(1000);
}
#endif

void processGPS()
{
  byte i;
  byte ltemp;
  Outputstring = "";

  if (gps.location.isUpdated() && gps.altitude.isUpdated())       //check the fix is updated
  {
    RXGPSfix = 1;
    Hlat = gps.location.lat();
    Hlon = gps.location.lng();
    Halt = gps.altitude.meters();
    
    Serial.print("RX GPS FIX");
    
    if (remote_fix)
    {
      //if there has been a fix received from transmitter do the calculations
      distanceto();
      directionto();
    }

    for (i == 0; i <= 20; i++)           //rapid flash LED to indicate local GPS update
    {
      digitalWrite(PLED1, HIGH);
      delay(20);
      digitalWrite(PLED1, LOW);
      delay(20);
    }

    GetSupplyVolts();                    //ensure dispaly update has latest battery voltage
    display_Update();

  }
  else
  {
    RXGPSfix = 0;
    if (TXGPSfix == 0);                  //if we have had a TX location fix, allow the Lat and Lon display to stay on LCD.
    {
      Serial.println(F("No GPS Update"));
      delay(2000);
    }
  }
}


float convertstring(String inString)
{
  //convert the string in the payload buffer to a float
  char buf[20];
  inString.toCharArray(buf, inString.length());
  float val = atof(buf);
  return val;
}


void FillPayloadArray(byte llora_RXStart, byte llora_RXEnd)
{
  //fill the payload array from the CSV data in the packet
  byte i = 0;
  byte j = 0;
  byte lvar1;
  String tempstring = "";

  for (i = llora_RXStart; i <= llora_RXEnd; i++)
  {
    lvar1 = lora_RXBUFF[i];

    if (lvar1 == ',')
    {
      results[j] = tempstring;
      j++;
      tempstring = "";
      if (j > PayloadArraySize)
      {
        Serial.print(F("ERROR To many Fields"));
        Serial.println(j);
        break;
      }
    }
    else
    {
      tempstring = tempstring + char(lvar1);
    }
  }
  Serial.println();
}


void printLatLonAlt()
{
  //print lat lon etc to serial termninal
  Serial.print(F("Tlat "));
  Serial.println(Tlat, 6);
  Serial.print(F("Tlon "));
  Serial.println(Tlon, 6);
  Serial.print(F("Talt "));
  Serial.println(Talt);
  Serial.print(F("Hlat "));
  Serial.println(Hlat, 6);
  Serial.print(F("Hlon "));
  Serial.println(Hlon, 6);
  Serial.print(F("Halt "));
  Serial.println(Halt);
  Serial.print(F("Dist "));
  Serial.println(dist);
  Serial.print(F("Dir "));
  Serial.println(dir);
  Serial.println();
}


void checkforpacket()
{
  //check if a packet has been received
  byte lora_LRegData, lora_Ltemp;

  lora_Ltemp = lora_readRXready();

  if (lora_Ltemp == 0)
  {
    Serial.print(F("NP "));
  }

  if (lora_Ltemp == 64)
  {
    lora_RXOFF();                        //stop more packets comming in
    Serial.println();
    Serial.println();
    digitalWrite(PLED1, HIGH);
    Serial.print(F("PacketRX "));
    lora_ReadPacket();
    lora_RXPKTInfo();                    //print the info on received packet
    Serial.println();
    processPacket();
    digitalWrite(PLED1, LOW);
    lora_RXONLoRa();                     //ready for next
  }

  if (lora_Ltemp == 96)
  {
    Serial.println();
    Serial.println(F("CRC Error"));
    lora_RXONLoRa();                     //ready for next
  }
}


void display_Update()
{
  //this carries out any necessary screen updates
  updatescreen0();
}


void directionto()
{
  //using remote and local lat and long calculate direction in degrees
  dir = (int) TinyGPSPlus::courseTo(Hlat, Hlon, Tlat, Tlon);
  #ifdef Debug
  dir = 77;
  #endif
}


void distanceto()
{
  //using remote and local lat and long calculate distance in metres
  dist = (unsigned long) TinyGPSPlus::distanceBetween(Hlat, Hlon, Tlat, Tlon);
  #ifdef Debug
  dist = 2400;
  #endif

}




void processPacket()
{
  //a packet has been received, work out what to do with it
  byte count, i, j, k;

  lora_RXBuffPrint(0);                                     //print the received packet
  Serial.println();

  if ((lora_RXPacketType == LMLPayload) || (lora_RXPacketType == LMLPayload_Repeated)) //is a short payload, direct or repeated
  {
    remote_fix = 1;
    TXGPSfix = 1;
    FillPayloadArray(lora_RXStart, lora_RXEnd);
    Tlat = convertstring(results[0]);
    Tlon = convertstring(results[1]);
    Talt = results[2].toInt();
    Tresets = results[4].toInt();
    TXGPSfix = results[6].toInt();
#ifdef NMEAUplink                                           //do we need to send NMEA string for Bluetooth connection ?
    count = send_NMEA(lora_TXBUFF, Tlat, Tlon, Talt);       //send position to Bluetooth
#endif
  isPositionUpdate();
  }

  if ((lora_RXPacketType == LongPayload) ) //is a Long HAB payload
  {
    remote_fix = 1;
    TXGPSfix = 1;
    FillPayloadArray(lora_RXStart, lora_RXEnd);
    Tlat = convertstring(results[3]);
    Tlon = convertstring(results[4]);
    Talt = results[5].toInt();
    Tresets = results[9].toInt();
    TXGPSfix = results[12].toInt();
#ifdef NMEAUplink                                           //do we need to send NMEA string for Bluetooth connection ?
    count = send_NMEA(lora_TXBUFF, Tlat, Tlon, Talt);       //send position to Bluetooth
#endif
  isPositionUpdate();
  }



  if (lora_RXPacketType == PowerUp)
  {
    j = lora_RXBUFF[0];
    k = lora_RXBUFF[1];
    TXSupplyVolts = (j + (k * 256));

    Serial.print(F("TX Supply "));
    Serial.print(TXSupplyVolts);
    Serial.println(F("mV"));

    display_Setfont(defaultfont);
    display_SetCurPos(0, 5);
    display_Text("           ");
    display_SetCurPos(0, 5);
    display_Text("TX ");
    display_Printint(TXSupplyVolts);
    display_Text("mV    ");
  }

  if (lora_RXPacketType == NoGPS)
  {
    Serial.println(F("TX GPS ERROR"));
    display_Setfont(defaultfont);
    display_SetCurPos(0, 5);
    display_Text("TX No GPS    ");
    lora_Tone(500, 3000, 10);                                  //transmit an FM tone as error
  }

  if (lora_RXPacketType == NoFix)
  {
    Serial.println(F("No TX GPS Fix"));
    display_Setfont(defaultfont);
    display_SetCurPos(0, 5);
    display_Text("No TX GPS Fix ");
  }

  if (lora_RXPacketType == IsLost)
  {
    Serial.println(F("Tracker Lost"));
    display_Setfont(defaultfont);
    display_SetCurPos(0, 5);
    display_Text("Tracker Lost ");
  }
}

void isPositionUpdate()
{
  if (RXGPSfix == 1)
  {
    distanceto();
    directionto();
  }

  printLatLonAlt();
  GetSupplyVolts();
  display_Update();
}  
  


void systemerror()
{
  //major error such as faulty LoRa device, cannot continue
  while (1)
  {
    digitalWrite(PLED1, HIGH);
    delay(100);
    digitalWrite(PLED1, LOW);
    delay(100);
  }
}


void led_FlashStart()
{
  //flash LED right at start so we can tell board is alive
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
  //set-up LoRa device for data reception
  init_LoRaSlow();
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
  //display supply volts or serial terminal\monitor
  Serial.print(F("Supply Volts "));
  Serial.print((int) GetSupplyVolts());
  Serial.println(F("mV"));
}


void setup()
{
  int i, j;

  pinMode(PLED1, OUTPUT);                      //for PCB LED
  led_FlashStart();

#ifdef GPS
  pinMode(GPSPOWER, OUTPUT);	               //setup pin for GPS Power Control
  gpsOn();
#endif
  disp.begin(9600);                            //startup send only softserial for display 
  Serial.begin(9600);                          //serial console ouput
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  Serial.println();

  DisplaySupplyVolts();

  pinMode(lora_PReset, OUTPUT);                //RFM98 reset line
  digitalWrite(lora_PReset, LOW);              //reset RFM98
  pinMode (lora_PNSS, OUTPUT);                 //set the slave Select opin as an output
  digitalWrite(lora_PNSS, HIGH);

  SPI.begin();                                 //initialize SPI:
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  if (lora_CheckDevice() == 1)
  {
    Serial.println(F("LoRa Error"));
    display_Setfont(0);
    display_SetCurPos(0, 5);
    display_Text("LoRa Error");
    systemerror();
  }

  init_LoRa();			               //do the initial LoRa Setup

  digitalWrite(PLED1, HIGH);
  lora_Tone(1000, 1000, 10);                   //transmit an FM tone, 1000hz, 1000ms
  digitalWrite(PLED1, LOW);
  setup_LoRa_for_Data();
  
  disp.begin(GPSBaud);                           //startup soft serial for GPS and Digole display
  display_Cls();
  display_Setfont(defaultfont);
  display_SetCurPos(0, 5);
  display_Text("RX ");
  display_Printint(GetSupplyVolts());
  display_Text("mV");
  delay(2000);
  
#ifndef GPS
  Serial.println(F("Configured for No Local GPS"));
#endif

#ifdef GPS
  gpsOn();
  gpsSetup();
#endif

  display_Update();
  lora_RXONLoRa();
}


