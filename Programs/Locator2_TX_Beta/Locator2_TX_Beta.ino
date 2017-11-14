//**************************************************************************************************
// Note:
//
// Make changes to this Program file at your peril
//
// Configuration changes should be made in the HAB2_Settings file not here !
//
//**************************************************************************************************

#define programname "Locator2_TX_Beta"
#define aurthorname "Stuart Robinson"

#include <Arduino.h>
#include <avr/pgmspace.h>

#include "Locator2_TX_Settings.h"
#include Board_Definition
#include "Program_Definitions.h"

/*
**************************************************************************************************

  LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
  free from errors.

  To Do:
  Change test packet routine to allow for 17dBm packets
  Add checks for TX and RX packets exceeding buffer size
  Change inc_on_error for RC pulse to be more meaningful
  Check altitude calc is working, this reported 51.51196,-3.19966,64917


  Changes:
  Start with HAB2 program 22754/1154
  Remove FSk RTTY 22066/1148
  Remove AFSK RTTY option 220158/1148
  Remove Fence check 21460/1132
  Remvove internal voltage and temperature 21460/1132
  Remove HAB payload 1742/1008
  Remove Doze mode 17028/ 1008
  Remove mAHr calc 15626/972
  Option for remote control 12822/825
  Add save and read TR Location 13062/835
  Tidy up 12930/831
  Remove sleeps from active (not lost mode)
  Add basic lost timeout and find me tones 12826/835
  Add Bit numbers for current_config byte settings, so lost status is save in memory

  13/11/17 Merge lost and RC pulse check routines from PIH tracker, switch to non-Flash5 setup -  18762/799





**************************************************************************************************
*/

char ramc_RemoteControlNode;
char ramc_ThisNode;

int ramc_CalibrationOffset;

unsigned long ramc_TrackerMode_Frequency;           //frequencies, and other parameters, are copied from memory into RAM.
unsigned long ramc_SearchMode_Frequency;            //this is so that these parameters can be changed in flight and then
unsigned long ramc_CommandMode_Frequency;           //copied into memory so that they survive reset.

byte ramc_TrackerMode_Bandwidth;
byte ramc_TrackerMode_SpreadingFactor;
byte ramc_TrackerMode_CodeRate;
byte ramc_TrackerMode_Power;
byte TRStatus;                                      //used to store current status flag bits

byte ramc_SearchMode_Bandwidth;
byte ramc_SearchMode_SpreadingFactor;
byte ramc_SearchMode_CodeRate;
byte ramc_SearchMode_Power;

byte ramc_CommandMode_Bandwidth;
byte ramc_CommandMode_SpreadingFactor;
byte ramc_CommandMode_CodeRate;
byte ramc_CommandMode_Power;

byte ramc_Current_TXconfig1;                        //sets the config of whats transmitted etc
byte ramc_Cmd_WaitSecs;


unsigned int ramc_Sleepsecs;                        //seconds for sleep at end of TX routine
unsigned int ramc_WaitGPSFixSeconds;                //in flight mode, default time to wait for a fix

unsigned long UPTime = 0;
unsigned long  activeTimeStartmS;


float TRLat;                                        //tracker transmitter co-ordinates
float TRLon;
unsigned int TRAlt;

byte ramc_promiscuous_Mode;


byte ramc_key0;
byte ramc_key1;
byte ramc_key2;
byte ramc_key3;
byte keypress;

unsigned long GPSonTime;
unsigned long GPSFixTime;
boolean GPS_Config_Error;


unsigned int UpMins;                         //how many Minutes after startup till lost mode triggered
unsigned int RCPulseLen;                     //length of RC pulse
unsigned int pulsewidthlast = 1500;          //measured width in uS of last RC pulse read, used for hold check
int pulseerrorcount = 50;                    //number of RC pulse errors recorded, start at 50


#include Board_Definition                            //include previously defined board file
#include Memory_Library                              //include previously defined Memory Library

#include <SPI.h>
#include <LowPower.h>                                //https://github.com/rocketscream/Low-Power

#include <TinyGPS++.h>                               //http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                     //create the TinyGPS++ object
TinyGPSCustom GNGGAFIXQ(gps, "GNGGA", 5);            //custom sentences used to detect possible switch to GLONASS mode

#ifdef USE_SOFTSERIAL_GPS
#include <NeoSWSerial.h>                             //https://github.com/SlashDevin/NeoSWSerial  
NeoSWSerial GPSserial(GPSRX, GPSTX);                 //this library is more relaible at GPS init than software serial
#endif

#include GPS_Library                                 //include previously defined GPS Library 

#include "Voltage_Temperature.h"
#include "LoRa3.h"
#include "Binary2.h"


void loop()
{

  Serial.println();
  Serial.println();
  Serial.println(F("Active "));

  checkforlost();

#ifndef DEBUGNoGPS
  gpsWaitFix(ramc_WaitGPSFixSeconds, SwitchOn, LeaveOn);
#endif

#ifdef Use_Test_Location
  TRLat = TestLatitude;
  TRLon = TestLongitude;
  TRAlt = TestAltitude;
#endif

  print_LocationBinary(TRLat, TRLon, TRAlt);          //print last known location
  Serial.println();

#ifdef AllowRemoteControl
  wait_Command();                                     //wait for incoming command
#endif


  do_Transmissions();

  digitalWrite(lora_NSS, HIGH);                             //take NSS line high, makes sure LoRa device is off

  //tempSleepsecs = (ramc_Sleepsecs * 1000);
  //Serial.print(F("Wait "));
  //Serial.print(ramc_Sleepsecs);
  //Serial.println(F(" Seconds"));
  sleepSecs(ramc_Sleepsecs);
}


/*boolean check_Lost()
  {
  unsigned long activeseconds;
  Serial.println(F("check_Lost()"));

  activeseconds = ((millis() -  activeTimeStartmS) / 1000);
  Serial.print(F("Active seconds = "));
  Serial.println(activeseconds);
  Serial.print(F("LostTimeoutSecs = "));
  Serial.println(LostTimeoutSecs);

  if (activeseconds > LostTimeoutSecs)
  {
    Serial.println(F("Locator Timeout!"));
    set_Lost();
    return true;
  }
  else
  {
    Serial.println(F("No Timeout"));
    return false;
  }
  }


  void set_Lost()
  {
  Serial.println(F("set_Lost()"));
  setStatusByte(TrackerLost, 1)
  }


  void are_Lost()
  {
  Serial.println(F("are_Lost()"));
  Setup_LoRaSearchMode();       //so frequency is correct for search mode
  print_LocationBinary(TRLat, TRLon, TRAlt);
  send_LocationBinary(TRLat, TRLon, TRAlt);
  Serial.println();
  FindMeTones();
  }
*/

void do_Transmissions()
{
  //this is where all the transmisions get sent
  //byte index, Count;

  pulseWDI();
  lora_Setup();                                                      //resets then sets up LoRa device

  Setup_LoRaTrackerMode();
  send_LocationBinary(TRLat, TRLon, TRAlt);
  Serial.println();

}


void print_LocationBinary(float Lat, float Lon, unsigned int Alt)
{
  Serial.print(Lat, 5);
  Serial.print(F(","));
  Serial.print(Lon, 5);
  Serial.print(F(","));
  Serial.print(Alt);
}


void send_LocationBinary(float Lat, float Lon, unsigned int Alt)
{
  Write_Float(0, Lat, lora_TXBUFF);
  Write_Float(4, Lon, lora_TXBUFF);
  Write_Int(8, Alt, lora_TXBUFF);
  Write_Byte(10, TRStatus, lora_TXBUFF);

  digitalWrite(LED1, HIGH);
  Serial.println(F("send_LocationBinary()"));
  lora_Send(0, 10, LocationBinaryPacket, Broadcast, ramc_ThisNode, 10, lora_Power, 0);   //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
  digitalWrite(LED1, LOW);
}


void sleepSecs(unsigned int LNumberSleeps)
{
  unsigned int i;

  Serial.print(F("zz "));
  Serial.println(LNumberSleeps);
  Serial.flush();                                      //let print complete
#ifdef USING_SERIALGPS
  GPSserial.end();                                     //we dont want GPS input interfering with sleep, make sure its off
#endif
  digitalWrite(lora_NSS, HIGH);                        //ensure LoRa Device is off

  for (i = 1; i <= LNumberSleeps; i++)
  {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);    //sleep 1 second
    pulseWDI();
  }

}


void incMemoryULong(unsigned int laddress)
{
  unsigned long val;
  val = Memory_ReadULong(laddress);
  val++;
  Memory_WriteULong(laddress, val);
}


byte readConfigByte(byte bitnum)
{
  return bitRead(ramc_Current_TXconfig1, bitnum);
}


void setConfigByte(byte bitnum, byte bitval)
{
  //program the config byte

  if (bitval == 0)
  {
    bitClear(ramc_Current_TXconfig1, bitnum);
  }
  else
  {
    bitSet(ramc_Current_TXconfig1, bitnum);
  }
  Memory_WriteByte(addr_Default_config1, ramc_Current_TXconfig1);

#ifdef DEBUG
  if (bitval)
  {
    Serial.print(F("Set Config Bit "));
  }
  else
  {
    Serial.print(F("Clear Config Bit "));
  }
  Serial.println(bitnum);

#endif
}


boolean getStatusBit(byte bitnum)
{
  //program the status byte
  byte temp;
  temp = bitRead(TRStatus, bitnum);
  return temp;
}




void setStatusByte(byte bitnum, byte bitval)
{
  //program the status byte

  if (bitval == 0)
  {
    bitClear(TRStatus, bitnum);
  }
  else
  {
    bitSet(TRStatus, bitnum);
  }

#ifdef DEBUG
  if (bitval)
  {
    Serial.print(F("Set Status Bit "));
  }
  else
  {
    Serial.print(F("Clear Status Bit "));
  }

  Serial.println(bitnum);
#endif

  Memory_WriteByte(addr_TRStatus, TRStatus);    //save status byte in memory
  //print_TRStatus();
  //Serial.println(F("Saving TRStatus"));
}


void print_TRStatus()
{
  Serial.print(F("TRStatus "));
  Serial.println(TRStatus, BIN);
}


void pulseWDI()
{
  //if the watchdog is fitted it needs a regular pulse to prevent reset
  //togle the WDI pin twice
  digitalWrite(WDI, !digitalRead(WDI));
  delayMicroseconds(1);
  digitalWrite(WDI, !digitalRead(WDI));
}


boolean gpsWaitFix(unsigned long waitSecs, byte StartState, byte LeaveState)
{
  //waits a specified number of seconds for a fix, returns true for good fix
  //StartState when set to 1 will turn GPS on at routine start
  //LeaveState when set to 0 will turn GPS off at routine end, used perhaps when there is no fix

  unsigned long endwaitmS, millistowait, currentmillis;
  pulseWDI();
  byte GPSByte;

  if (StartState == 1)
  {
    GPS_On(DoGPSPowerSwitch);
    GPSonTime = millis();
  }
  else
  {
    GPS_On(NoGPSPowerSwitch);
  }

  Serial.print(F("Wait Fix "));
  Serial.print(waitSecs);
  Serial.println(F(" Secs"));

  currentmillis = millis();
  millistowait = waitSecs * 1000;
  endwaitmS = currentmillis + millistowait;

  while (millis() < endwaitmS)
  {

    do
    {
      GPSByte = GPS_GetByte();
      if (GPSByte != 0xFF)
      {
        gps.encode(GPSByte);
      }
    }
    while (GPSByte != 0xFF);

    if (gps.location.isUpdated() && gps.altitude.isUpdated())
    {
      Serial.println(F("GPS Fix"));
      TRLat = gps.location.lat();
      TRLon = gps.location.lng();
      TRAlt = (unsigned int) gps.altitude.meters();
      
      //Altitude is used as an unsigned integer, so that the binary payload is as short as possible.
      //However gps.altitude.meters(); can return a negative value which converts to
      //65535 - Altitude, which we dont want. So we will assume any value over 60,000M is zero

      if (TRAlt > 60000)                              //check for roll over to negative altitude
      {
        TRAlt = 0;
      }

      save_TRData();                                   //save location in memory

      if (readConfigByte(GPSHotFix))
      {
        GPS_Off(DoGPSPowerSwitch);
        GPSFixTime = (millis() - GPSonTime);
        //GPSoffTime = millis();
        Serial.print(F("GPS FixTime "));
        Serial.print(GPSFixTime);
        Serial.println(F("mS"));
      }
      else
      {
        GPS_Off(NoGPSPowerSwitch);
      }

      setStatusByte(GPSFix, 1);
      pulseWDI();
      return true;
    }

#ifdef UBLOX
    if (GNGGAFIXQ.age() < 2000)                     //check to see if GLONASS has gone active
    {
      Serial.println(F("GLONASS !"));
      setStatusByte(GLONASSisoutput, 1);
      GPS_SetGPMode();
      GPS_SetCyclicMode();
    }
    else
    {
      setStatusByte(GLONASSisoutput, 0);            //GLONASS not detected
    }
#endif



  }

  //if here then there has been no fix and a timeout
  setStatusByte(GPSFix, 0);                       //set status bit to flag no fix
  Serial.println(F("No Fix"));

  if (LeaveState == 0)
  {
    //no fix and gpsWaitFix called with gpspower to be turned off on exit
    GPS_Off(DoGPSPowerSwitch);
  }
  else
  {
    //no fix but gpsWaitFix called with gpspower to be left on at exit
    GPS_Off(NoGPSPowerSwitch);
  }

  pulseWDI();
  return false;
}


void readSettingsDefaults()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this program uses constants in RAM copied from Memory (EEPROM or FRAM) in the same way as the transmitter.
  //There are some exceptions, where the local programs need to use a setting unique to the particular
  //receiver.
  Serial.println(F("Config Defaults"));
  ramc_TrackerMode_Frequency = TrackerMode_Frequency;
  ramc_TrackerMode_Bandwidth = TrackerMode_Bandwidth;
  ramc_TrackerMode_SpreadingFactor = TrackerMode_SpreadingFactor;
  ramc_TrackerMode_CodeRate = TrackerMode_CodeRate;
  ramc_TrackerMode_Power = TrackerMode_Power;

  ramc_SearchMode_Frequency = SearchMode_Frequency;
  ramc_SearchMode_Bandwidth = SearchMode_Bandwidth;
  ramc_SearchMode_SpreadingFactor = SearchMode_SpreadingFactor;
  ramc_SearchMode_CodeRate = SearchMode_CodeRate;
  ramc_SearchMode_Power = SearchMode_Power;

  ramc_CommandMode_Frequency = CommandMode_Frequency;
  ramc_CommandMode_Bandwidth = CommandMode_Bandwidth;
  ramc_CommandMode_SpreadingFactor = CommandMode_SpreadingFactor;
  ramc_CommandMode_CodeRate = CommandMode_CodeRate;
  ramc_CommandMode_Power = CommandMode_Power;

  ramc_ThisNode = ThisNode;
  ramc_RemoteControlNode = RemoteControlNode;

  ramc_Current_TXconfig1 = Default_config1;
  ramc_Cmd_WaitSecs = Cmd_WaitSecs;
  ramc_WaitGPSFixSeconds = WaitGPSFixSeconds;
  ramc_Sleepsecs = Loop_Sleepsecs;
  ramc_promiscuous_Mode = promiscuous_Mode;


  ramc_key0 = key0;
  ramc_key1 = key1;
  ramc_key2 = key2;
  ramc_key3 = key3;

  ramc_CalibrationOffset = CalibrationOffset;
}


void readSettingsMemory()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this program uses constants in RAM copied from Memory (EEPROM or FRAM) in the same way as the transmitter.
  //There are some exceptions, where the local programs need to use a setting unique to the particular
  //receiver.
  Serial.println(F("Config from Memory"));

  ramc_TrackerMode_Frequency = Memory_ReadULong(addr_TrackerMode_Frequency);
  ramc_TrackerMode_Bandwidth = Memory_ReadByte(addr_TrackerMode_Bandwidth);
  ramc_TrackerMode_SpreadingFactor = Memory_ReadByte(addr_TrackerMode_SpreadingFactor);
  ramc_TrackerMode_CodeRate = Memory_ReadByte(addr_TrackerMode_CodeRate);
  ramc_TrackerMode_Power = Memory_ReadByte(addr_TrackerMode_Power);

  ramc_SearchMode_Frequency = Memory_ReadULong(addr_SearchMode_Frequency);
  ramc_SearchMode_Bandwidth = Memory_ReadByte(addr_SearchMode_Bandwidth);
  ramc_SearchMode_SpreadingFactor = Memory_ReadByte(addr_SearchMode_SpreadingFactor);
  ramc_SearchMode_CodeRate = Memory_ReadByte(addr_SearchMode_CodeRate);
  ramc_SearchMode_Power = Memory_ReadByte(addr_SearchMode_Power);

  ramc_CommandMode_Frequency = Memory_ReadULong(addr_CommandMode_Frequency);
  ramc_CommandMode_Bandwidth = Memory_ReadByte(addr_CommandMode_Bandwidth);
  ramc_CommandMode_SpreadingFactor = Memory_ReadByte(addr_CommandMode_SpreadingFactor);
  ramc_CommandMode_CodeRate = Memory_ReadByte(addr_CommandMode_CodeRate);
  ramc_CommandMode_Power = Memory_ReadByte(addr_CommandMode_Power);

  ramc_ThisNode = Memory_ReadByte(addr_ThisNode);;
  ramc_RemoteControlNode = Memory_ReadByte(addr_RemoteControlNode);;

  ramc_Current_TXconfig1 = Memory_ReadByte(addr_Default_config1);
  ramc_Cmd_WaitSecs = Memory_ReadByte(addr_Cmd_WaitSecs);
  ramc_WaitGPSFixSeconds = Memory_ReadUInt(addr_WaitGPSFixSeconds);
  ramc_Sleepsecs = Memory_ReadUInt(addr_Sleepsecs);
  ramc_promiscuous_Mode = Memory_ReadByte(addr_promiscuous_Mode);

  //retrieve TRStatus form memory so that we can test for Lost stauts
  TRStatus = Memory_ReadByte(addr_TRStatus);

  ramc_key0 = Memory_ReadByte(addr_key0);
  ramc_key1 = Memory_ReadByte(addr_key1);
  ramc_key2 = Memory_ReadByte(addr_key2);
  ramc_key3 = Memory_ReadByte(addr_key3);

  ramc_CalibrationOffset = Memory_ReadInt(addr_CalibrationOffset);

}


void writeSettingsMemory()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this program uses constants in RAM copied from Memory (EEPROM or FRAM)

  Serial.println(F("Write RAM Settings to Memory"));

  Memory_Set(addr_StartConfigData, addr_EndConfigData, 0);          //fill config area with 0

  Memory_WriteULong(addr_TrackerMode_Frequency, ramc_TrackerMode_Frequency);
  Memory_WriteByte(addr_TrackerMode_Bandwidth, ramc_TrackerMode_Bandwidth);
  Memory_WriteByte(addr_TrackerMode_SpreadingFactor, ramc_TrackerMode_SpreadingFactor);
  Memory_WriteByte(addr_TrackerMode_CodeRate, ramc_TrackerMode_CodeRate);
  Memory_WriteByte(addr_TrackerMode_Power, ramc_TrackerMode_Power);

  Memory_WriteULong(addr_SearchMode_Frequency, ramc_SearchMode_Frequency);
  Memory_WriteByte(addr_SearchMode_Bandwidth, ramc_SearchMode_Bandwidth);
  Memory_WriteByte(addr_SearchMode_SpreadingFactor, ramc_SearchMode_SpreadingFactor);
  Memory_WriteByte(addr_SearchMode_CodeRate, ramc_SearchMode_CodeRate);
  Memory_WriteByte(addr_SearchMode_Power, ramc_SearchMode_Power);

  Memory_WriteULong(addr_CommandMode_Frequency, ramc_CommandMode_Frequency);
  Memory_WriteByte(addr_CommandMode_Bandwidth, ramc_CommandMode_Bandwidth);
  Memory_WriteByte(addr_CommandMode_SpreadingFactor, ramc_CommandMode_SpreadingFactor);
  Memory_WriteByte(addr_CommandMode_CodeRate, ramc_CommandMode_CodeRate);
  Memory_WriteByte(addr_CommandMode_Power, ramc_CommandMode_Power);

  Memory_WriteByte(addr_Default_config1, ramc_Current_TXconfig1);
  Memory_WriteByte(addr_RemoteControlNode, ramc_RemoteControlNode);

  Memory_WriteByte(addr_ThisNode, ramc_ThisNode);
  Memory_WriteByte(addr_RemoteControlNode, ramc_RemoteControlNode);

  Memory_WriteByte(addr_Default_config1, ramc_Current_TXconfig1);
  Memory_WriteByte(addr_Cmd_WaitSecs, ramc_Cmd_WaitSecs);
  Memory_WriteUInt(addr_WaitGPSFixSeconds, ramc_WaitGPSFixSeconds);
  Memory_WriteUInt(addr_Sleepsecs, ramc_Sleepsecs);
  Memory_WriteByte(addr_promiscuous_Mode, ramc_promiscuous_Mode);


  Memory_WriteByte(addr_key0, ramc_key0);
  Memory_WriteByte(addr_key1, ramc_key1);
  Memory_WriteByte(addr_key2, ramc_key2);
  Memory_WriteByte(addr_key3, ramc_key3);
  Memory_WriteInt(addr_CalibrationOffset, ramc_CalibrationOffset);
}


void sendTrackerBind()
{
  unsigned int i, j;
  byte msb_CRC, lsb_CRC;
  unsigned int bindCRC;

  saveKeyin_buffer();                          //loads key in bytes 0,1,2,3 of TX buffer

  lora_TXEnd = 4;                              //this is where the bind data starts

  for (i = addr_StartBindData; i <= addr_EndBindData; i++)
  {
    j =  Memory_ReadByte(i);
    lora_TXBUFF[lora_TXEnd++] = j;
  }

  bindCRC = Print_CRC_Bind_Memory();
  msb_CRC = highByte(bindCRC);
  lsb_CRC = lowByte(bindCRC);
  lora_TXBUFF[lora_TXEnd++] = lsb_CRC;
  lora_TXBUFF[lora_TXEnd] = msb_CRC;

#ifdef DEBUG
  Serial.print(F("Bind PacketLen "));
  Serial.println(lora_TXEnd + 4);               //allow for 3 addressing bytes in length, plus 1 for packet starting at [0]
#endif

  lora_Send(0, lora_TXEnd, Bind, ramc_RemoteControlNode, ramc_ThisNode, 10, BindMode_Power, 0);

}


void printNodes()
{
  Serial.print(F("ThisNode "));
  Serial.print(ramc_ThisNode);
  Serial.print(F("  RemoteNode "));
  Serial.println(ramc_RemoteControlNode);
  Serial.println();
}


void Setup_LoRaTrackerMode()
{
  Serial.println(F("Setup_LoRaTrackerMode()"));
  lora_SetFreq(ramc_TrackerMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(ramc_TrackerMode_Bandwidth, ramc_TrackerMode_SpreadingFactor, ramc_TrackerMode_CodeRate, Explicit);  //Setup the LoRa modem parameters for tracker mode
  lora_Power = ramc_TrackerMode_Power;
}


void Setup_LoRaSearchMode()
{
  Serial.println(F("Setup_LoRaSearchMode()"));
  lora_SetFreq(ramc_SearchMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(ramc_SearchMode_Bandwidth, ramc_SearchMode_SpreadingFactor, ramc_SearchMode_CodeRate, Explicit);  //Setup the LoRa modem parameters for search mode
}


void Setup_LoRaCommandMode()
{
  Serial.println(F("Setup_LoRaCommandMode()"));
  lora_SetFreq(ramc_CommandMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(ramc_CommandMode_Bandwidth, ramc_CommandMode_SpreadingFactor, ramc_CommandMode_CodeRate, Explicit);  //Setup the LoRa modem parameters for command mode
  lora_Power = ramc_CommandMode_Power;
}


void Setup_LoRaBindMode()
{
  Serial.println(F("Setup_LoRaBindMode()"));
  lora_SetFreq(BindMode_Frequency, ramc_CalibrationOffset);
  lora_SetModem2(BindMode_Bandwidth, BindMode_SpreadingFactor, BindMode_CodeRate, Explicit); //Setup the LoRa modem parameters for bind mode
  lora_Power = BindMode_Power;
}


void display_current_frequency()
{
  float freq_temp;
  freq_temp = lora_GetFreq();
  Serial.print(F("Frequency "));
  Serial.print(freq_temp, 3);
  Serial.println(F("MHz"));
}


void led_Flash(unsigned int flashes, unsigned int delaymS)
{
  //flash LED to show tracker is alive
  unsigned int index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


//*******************************************************************************************************
// Memory Routines
//*******************************************************************************************************

void Clear_All_Memory()
{
  //clears the whole of memory, normally 1kbyte
  Serial.print(F("Clear All Memory"));
  Memory_Set(addr_StartMemory, addr_EndMemory, 0);
}


void Print_Config_Memory()
{
  //prints the memory used for storing configuration settings
  byte memory_LLoopv1;
  byte memory_LLoopv2;
  unsigned int memory_Laddr = 0;
  byte memory_Ldata;
  //unsigned int CRC;
  Serial.println(F("Config Memory"));
  Serial.print(F("Lcn    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  Serial.println();

  for (memory_LLoopv1 = 0; memory_LLoopv1 <= 15; memory_LLoopv1++)
  {
    Serial.print(F("0x"));
    Serial.print(memory_LLoopv1, HEX);                       //print the register number
    Serial.print(F("0  "));
    for (memory_LLoopv2 = 0; memory_LLoopv2 <= 15; memory_LLoopv2++)
    {
      memory_Ldata = Memory_ReadByte(memory_Laddr);
      if (memory_Ldata < 0x10) {
        Serial.print(F("0"));
      }
      Serial.print(memory_Ldata, HEX);                       //print the register number
      Serial.print(F(" "));
      memory_Laddr++;
    }
    Serial.println();
  }
}



unsigned int Print_CRC_Bind_Memory()
{
  unsigned int returnedCRC = Memory_CRC(addr_StartBindData, addr_EndBindData);
  Serial.print(F("Local BindCRC "));
  Serial.println(returnedCRC, HEX);
  return returnedCRC;
}


void read_TRData()
{
  //read stored tracker location data from memory
  TRLat = Memory_ReadFloat(addr_TRLat);
  TRLon = Memory_ReadFloat(addr_TRLon);
  TRAlt = Memory_ReadUInt(addr_TRAlt);
}


void save_TRData()
{
  //writes the last received tracker location data to memory
  Memory_WriteFloat(addr_TRLat, TRLat);
  Memory_WriteFloat(addr_TRLon, TRLon);
  Memory_WriteUInt(addr_TRAlt, TRAlt);
}


void saveKeyin_buffer()
{
  lora_TXBUFF[0] = key0;       //key used in some packets to reduce chances of a change being applied by accident
  lora_TXBUFF[1] = key1;
  lora_TXBUFF[2] = key2;
  lora_TXBUFF[3] = key3;
}

void send_Command(char cmd)
{
  unsigned int volts;
  volts = read_SupplyVoltage();
  Serial.print(F("Send Cmd "));
  Serial.write(cmd);
  Serial.println();
  Write_Byte(0, lora_PacketSNR, lora_TXBUFF);                         //so that receiver alwsys knows last received SNR
  Write_Byte(1, lora_PacketRSSI, lora_TXBUFF);                        //so that receiver alwsys knows last received RSSI
  Write_UInt(2, volts, lora_TXBUFF);
  Write_Byte(4, TRStatus, lora_TXBUFF);
  digitalWrite(LED1, HIGH);
  lora_Send(0, 4, cmd, Broadcast, ramc_ThisNode, 10, lora_Power, 0);
  digitalWrite(LED1, LOW);
}



//**************************************************************************************************
// Start Remote Contol Routines
//**************************************************************************************************


#ifdef AllowRemoteControl
void wait_Command()
{
  byte index;
  pulseWDI();
  lora_Setup();                                              //resets then sets up LoRa device
  Setup_LoRaCommandMode();                                   //commands can be sent in any mode, make sure this is sent using the right frequency etc
  send_Command(ClearToSendCommand);                          //indicate ready for command

  lora_RXPacketType = 0;                                     //use as flag to tell if anything received during listen
  Listen(Cmd_WaitSecs);                                      //wait for command packet

  if (lora_RXPacketType > 0)
  {
    do
    {
      //there was activity during previous listen
      lora_RXPacketType = 0;

      for (index = 1; index <= Command_Loops; index++)
      {
        Setup_LoRaCommandMode();                              //commands can be sent in any mode, make sure this is sent using the right frequency etc
        send_Command(ClearToSendCommand);
        Listen(Cmd_WaitSecs);
      }

    }  while (lora_RXPacketType > 0);                         //wait until the extended listen exits with no packet received

  }
  else
  {
    Serial.println(F("No RX"));
  }
}

void Listen(unsigned int seconds)
{
  //listen (in seconds) for an incoming packet using the current frequency and LoRa modem settings
  unsigned long tilltime;
  tilltime = (millis() + (seconds * 1000));
  Serial.print(F("Listen "));
  Serial.println(seconds);

  lora_RXONLoRa();

  while (millis() < tilltime)
  {
    checkForPacket();
  }
  lora_RXOFF();                                     //as we have finished listening
}


void checkForPacket()
{
  //check LoRa device to see if a command packet has arrived
  byte lora_Ltemp;

  lora_Ltemp = lora_readRXready();

  if (lora_Ltemp > 0)
  {

    Serial.print(F("RX "));

    if (lora_Ltemp == 64)
    {
      //packet has arrived
      lora_ReadPacket();

#ifdef DEBUG
      Serial.write(lora_RXPacketType);
      Serial.write(lora_RXDestination);
      Serial.write(lora_RXSource);
      Serial.println();
#endif
    }
    else
    {
      //packet arrived with error
      Serial.println(F("Error"));
      lora_RXOFF();
      lora_RXONLoRa();
    }

    if (promiscuous_Mode)                           //can we accept packet from any source
    {
      processPacket();
      lora_RXONLoRa();
    }

    if (!promiscuous_Mode)                          //can we only accepts packet from known node
    {
      if (lora_RXSource == ramc_RemoteControlNode)
      {
        processPacket();
        lora_RXONLoRa();                            //ready for next and clear flags
      }
      else
      {
        Serial.println(F("Rejected"));
        lora_RXOFF();
        Setup_LoRaCommandMode();
        send_Command(NACK);
        lora_RXONLoRa();
      }
    }
  }
}


void processPacket()
{
  //we have a packet so lets decide what to do with it
  byte i, j, ptr;

  if (lora_RXPacketType == Test)
  {
    if (lora_RXBUFF[0] == '0')
    {
      Serial.println(F("Pkt Test"));
      delay(inter_Packet_delay);
      Setup_LoRaCommandMode();
      send_Command(ACK);
      delay(inter_Packet_delay);
      sendTest();
    }
  }

  if (lora_RXPacketType == LinkReport)
  {
    send_Command(ACK);
    delay(inter_Packet_delay);
    Serial.println(F("Link Report"));
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(Info);
  }


  if (lora_RXPacketType == Config0)                   //is it a change config byte request ?
  {
    Serial.println(F("Prog Cfgbyte"));

    i = ((lora_RXBUFF[0] - 48));                      //config byte requests come in as ASCCI, '1' for 1 etc
    j = ((lora_RXBUFF[1] - 48));
    setConfigByte(i, j);
    lora_RXBuffPrint(0);                              //print packet contents as ASCII
    Serial.println();
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(ACK);                                //send the ack
  }

  if (lora_RXPacketType == ResetTracker)              //is it a reset ?
  {
    Serial.println(F("Reset ?"));
    lora_RXBuffPrint(0);                              //print packet contents as ASCII
    Serial.println();

    if ( isKeyValid() )
    {
      Serial.println(F("Valid"));
      delay(inter_Packet_delay);
      Setup_LoRaCommandMode();
      send_Command(ACK);
      Serial.flush();
      sleepsecs(2);
      softReset();
    }
    else
    {
      Serial.println(F("Invalid"));
      delay(inter_Packet_delay);
      Setup_LoRaCommandMode();
      send_Command(NACK);
    }
  }

  if (lora_RXPacketType == WritePacketMemory)
  {
    Serial.println(F("Write Memory"));
    writePacketMemory();
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(ACK);
  }

  if (lora_RXPacketType == INCFreq)
  {
    Serial.println(F("IncOffset 1KHZ"));
    ramc_CalibrationOffset = ramc_CalibrationOffset + 1000;
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(ACK);
    Memory_WriteInt(addr_CalibrationOffset, ramc_CalibrationOffset);

  }

  if (lora_RXPacketType == DECFreq)
  {
    Serial.println(F("DecOffset 1KHZ"));
    ramc_CalibrationOffset = ramc_CalibrationOffset - 1000;
    delay(inter_Packet_delay);
    Setup_LoRaCommandMode();
    send_Command(ACK);
    Memory_WriteInt(addr_CalibrationOffset, ramc_CalibrationOffset);
  }

  if (lora_RXPacketType == Bind)
  {

    if (isKeyValid())                                           //only accept bind request when key is valid
    {

      ptr = 4;                                                  //bind packet has 4 bytes of key
      Serial.println(F("Bind RX"));

      for (i = addr_StartConfigData; i <= addr_EndConfigData; i++)
      {
        j = lora_RXBUFF[ptr++];
        Memory_WriteByte(i, j);
      }

      readSettingsMemory();

#ifdef DEBUG

      Print_Config_Memory();

#endif

      delay(inter_Packet_delay);
      send_Command(ACK);
    }
    else
    {
      Serial.println(F("Key not valid"));
    }

  }
}

boolean isKeyValid()
{
  if ( (lora_RXBUFF[0] == key0) && (lora_RXBUFF[1] == key1)  && (lora_RXBUFF[2] == key2)  && (lora_RXBUFF[3] == key3) )
  {
    return true;
  }
  else
  {
    return false;
  }
}


void softReset()
{
  asm volatile ("  jmp 0");
}




void sendTest()
{
  byte power;
  for (power = 10; power >= 2; power--)
  {
    Setup_LoRaTrackerMode();
    lora_TXBUFF[0] = '0';
    lora_TXBUFF[1] = power + 48;
    lora_TXEnd = 1;

    if (power == 10)
    {
      lora_TXBUFF[0] = '1';
      lora_TXBUFF[1] = '0';
      lora_TXEnd = 2;
    }

    Serial.print(F("Send "));
    Serial.print(power);
    Serial.println(F("dBm"));

    lora_Send(0, 1, Test, Broadcast, ramc_ThisNode, 10, power, 0);     //send the test packet
    sleepsecs(2);
    //delay(2000);
  }
}

void printMemoryChange(byte number)
{
#ifdef DEBUG
  byte index, j;
  Serial.print(F("Memory Change"));
  for (index = 0; index <= number; index++)
  {
    j = lora_RXBUFF[index];
    Serial.print(F(" "));
    Serial.print(j, HEX);
  }
  Serial.println();
#endif
}


void writePacketMemory()
{
  //there is an incoming packet which is a request to write bytes to Memory.
  //the effect is to change stored program definitions and constants
  byte i, j, k, ptr;
  //byte i, j, k, ptr, low, high;
  unsigned int addr_Memory;
  //float tempfloat;

  //packet format is key0, key1, key2, key3, number of bytes to write, address to write to, bytes to write
  //terminate list with 0 bytes to write.

  if (isKeyValid())
  {
    Serial.print(F("Not Valid"));
    return;
  }

  i = lora_RXPacketL - 4;                      //end of packet will be length - 1 for 0 offset and -3 for adddress bytes

  printMemoryChange(i);

  ptr = 4;

  j = lora_RXBUFF[ptr++];

  addr_Memory = Read_Int(5, lora_RXBUFF);     //read address for frequency offset into buffer

  ptr++;
  ptr++;

  Serial.println(F("Write memory "));

  for (i = 1; i <= j; i++)
  {
    Memory_WriteByte(addr_Memory, lora_RXBUFF[ptr]);
    k = lora_RXBUFF[ptr];
    Serial.print(k, HEX);
    Serial.print(F(" "));
    addr_Memory++;
    ptr++;
  }
  readSettingsMemory();
  Setup_LoRaTrackerMode();                     //dummy change so we can see if offset chnages
}
#endif

//**************************************************************************************************
// End Remote Contol Routines
//**************************************************************************************************


//**************************************************************************************************
// Start previous version lost routines
//**************************************************************************************************



void LostMode()
{
  //sadly all is lost

  GPS_Off(DoGPSPowerSwitch);                            //make sure GPS is off
  send_Command(IsLost);
  sleepSecs(1);


  while (1)
  {
    Serial.println(F("Lost Mode"));
    print_LocationBinary(TRLat, TRLon, TRAlt);          //print last known location
    Serial.println();
    Setup_LoRaTrackerMode();
    send_LocationBinary(TRLat, TRLon, TRAlt);
    sleepSecs(1);
    Setup_LoRaSearchMode();
    send_LocationBinary(TRLat, TRLon, TRAlt);
    sleepSecs(1);
    FindMeTones();
    Serial.print(F("Wait "));
    Serial.print(TXLostDelaySecs);
    Serial.println(F(" Seconds"));
    sleepSecs(TXLostDelaySecs);
    Serial.println();
  }
}


byte isLost()
{
  //check to see if tracker is in lost mode
  if (getStatusBit(TrackerLost))
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

#ifdef EnableTimeout
  checkfortimeout();
#endif

#ifdef EnableRCPulseCheck
  checkRCpulse();
#endif
}


void setLost()
{
  //set lost mode, stored in memory
  setStatusByte(TrackerLost, 1);
}


void checkRCpulse()
{
  //reads RC pulse and checks for errors and hold
  //ss.end();
  //GPSserial.end();
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
    setLost();
    LostMode();
  }
  sleepSecs(1);
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
    setLost();
    LostMode();
  }
}


void FindMeTones()
{
  //transmits the FM search tones, descending power and rising frequency
  Serial.println(F("FindMeTones()"));
  digitalWrite(LED1, HIGH);
  lora_Tone(600, 250, 10);                                //transmit an FM tone, 600hz, 250ms, 10dBm
  digitalWrite(LED1, LOW);
  delay(200);
  digitalWrite(LED1, HIGH);
  lora_Tone(1000, 300, 5);
  digitalWrite(LED1, LOW);
  delay(200);
  digitalWrite(LED1, HIGH);
  lora_Tone(1600, 600, 2);
  digitalWrite(LED1, LOW);
}



void setup()
{
  //unsigned long int i;
  unsigned int j;
  activeTimeStartmS = millis();

  pinMode(LED1, OUTPUT);                  //for PCB LED
  pinMode(WDI, OUTPUT);                   //for Watchdog pulse input

  led_Flash(5,100);                       //power on LED flash  

  Serial.begin(38400);                    //Setup Serial console ouput

#ifdef ClearAllMemory
  Clear_All_Memory();
#endif


  Serial.println(F(programname));
  Serial.println(F(aurthorname));

  pinMode(GPSPOWER, OUTPUT);              //in case power switching components are fitted
  GPS_On(DoGPSPowerSwitch);               //this will power the GPSon
  GPSonTime = millis();

#ifdef USING_SERIALGPS
  GPSserial.end();                        //but we dont want soft serial running for now, it interferes with the LoRa device
#endif

  pinMode(lora_NReset, OUTPUT);           //LoRa device reset line
  digitalWrite(lora_NReset, HIGH);

  pinMode (lora_NSS, OUTPUT);             //set the slave select pin as an output:
  digitalWrite(lora_NSS, HIGH);

  SPI.begin();                            //initialize SPI
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));


#ifdef ClearSavedData
  do_ClearSavedData();
#endif

  Serial.print(F("Default_Config1 "));
  Serial.println(Default_config1, BIN);


  j = Memory_ReadUInt(addr_ResetCount);
  j++;
  Memory_WriteUInt(addr_ResetCount, j);
  Serial.print(F("Resets "));
  Serial.println(j);


#ifdef ConfigureDefaults
  readSettingsDefaults();
  writeSettingsMemory();
#endif


#ifdef ConfigureFromMemory
  readSettingsMemory();
#endif

  read_TRData();  //get last saved location
  TRStatus = Memory_ReadByte(addr_TRStatus);    //make sure we have previous TRStatus
  print_TRStatus();

  ramc_ThisNode = ThisNode;

#ifdef DEBUG
  printNodes();
#endif

  lora_Setup();

  if (!lora_CheckDevice())
  {
    led_Flash(100, 50);                                                //long medium speed flash for Lora device error
    Serial.println(F("LoRa Error!"));
  }

#ifdef DEBUG
  display_current_frequency();
#endif

  ramc_CalibrationOffset = Memory_ReadInt(addr_CalibrationOffset);  //get calibration offset for this tracker
  Serial.print(F("Cal Offset "));
  Serial.println(ramc_CalibrationOffset);

#ifdef DEBUG
  lora_Print();
#endif

  Serial.println();
  print_SupplyVoltage();
  Serial.println();

  j = read_SupplyVoltage();                     //get supply mV
  Write_Int(0, j, lora_TXBUFF);                 //write to first two bytes of buffer
  Write_Byte(2, ramc_Current_TXconfig1, lora_TXBUFF);  //add the current config byte

  Setup_LoRaTrackerMode();
  send_Command(PowerUp);                                //send power up command, includes supply mV and config, on tracker settings
  sleepSecs(1);

  if (isLost())                                  //after reset is lost mode still set ?
  {
    Serial.println(F("Lost is Set - Checking for clear"));
    bitClear(TRStatus, TrackerLost);            //Clear lost mode
    Memory_WriteByte(addr_TRStatus, TRStatus);  //save cleared lost mode status byte in memory
    Serial.println(F("Lost Cleared - Waiting for reset"));
    FindMeTones();                              //sending Find me tones at startup indictaes lost mode set
    digitalWrite(LED1, HIGH);                    //put LED high to indicate start of lost clear check
    sleepSecs(lostclear_CheckSeconds);
    setStatusByte(TrackerLost, 1);              //set lost mode again, and save
    Memory_WriteByte(addr_TRStatus, TRStatus);  //save set lost mode status byte in memory
    Serial.println(F("Lost is Set"));
  }

  if (isLost())
  {
    Serial.println(F("Continue Lost Mode"));
    LostMode();
  }

#ifdef SendBind
  if (readConfigByte(TXEnable))   //is TX enabled ?
  {
    Setup_LoRaBindMode();
    sendTrackerBind();
  }
#endif

  Setup_LoRaTrackerMode();                             //so that check tone is at correct frequency

  GPS_Config_Error = false;                            //make sure GPS error flag is cleared

#ifndef DEBUGNoGPS
  GPS_On(DoGPSPowerSwitch);                                 //GPS should have been on for a while by now, so this is just to start soft serial
  GPSonTime = millis();
  GPS_Setup();                                           //GPS should have had plenty of time to initialise by now

  if (GPS_Config_Error)
  {
    Serial.println(F("GPS Error !"));
    Serial.println();
    send_Command(NoGPS);                                    //make sure receiver knows about GPS error
    led_Flash(100, 25);                                     //long very rapid flash for GPS error
  }
  else
  {
#ifdef CheckTone
    if (readConfigByte(TXEnable))                           //is TX enabled - needed because of fence limits
    {
      Serial.println(F("Check Tone"));                      //check tone indicates navigation model 6 set (if checktone enabled!)
      lora_Tone(1000, 3000, 5);                             //Transmit an FM tone, 1000hz, 3000ms, 5dBm
    }
#endif
  }

  digitalWrite(LED1, HIGH);
  setStatusByte(NoGPSTestMode, 0);



  GPSonTime = millis();
  while (!gpsWaitFix(5, DontSwitch, LeaveOn))           //wait for the initial GPS fix, this could take a while, leave GPS powered on
  {

    led_Flash(2, 50);                                     //two short LED flashes to indicate GPS waiting for fix

#ifdef DEBUG
    i = (millis() - GPSonTime) / 1000;
    Serial.print(F("GPS OnTime "));
    Serial.print(i);
    Serial.println(F(" Secs"));
#endif
  }

#endif

#ifndef DEBUGNoGPS
  GPS_On(DoGPSPowerSwitch);
  GPS_SetCyclicMode();                                     //set this regardless of whether hot fix mode is enabled
#endif

  lora_Tone(500, 500, 2);                                  //Transmit an FM tone, 500hz, 500ms, 2dBm
  digitalWrite(LED1, LOW);
  sleepSecs(2);                                            //wait for GPS to shut down

#ifdef DEBUGNoGPS
  setStatusByte(NoGPSTestMode, 1);
#endif

  //read_TRData();

  /*

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
    checkRCpulse();


  */




}











