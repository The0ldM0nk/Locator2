//GPS.h
/*
LoRaTracker Programs

Copyright of the author Stuart Robinson - 11/04/16

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

To do:

*/


void setup_GPMode();
void setup_FlightMode();
void setup_Cyclic();
void send_Config();
void clear_CurrentCFG();
void save_CurrentCFG();


#ifdef NONMEA
void getprocessGPSchar()                 //get and process output from GPS
{
  while (ss.available() > 0)
    gps.encode(ss.read());
}
#endif


void gpsOn()                             //do what is necessary to turn GPS on
{
  digitalWrite(GPSPOWER, LOW);           //turn on GPS Power
}

void gpsOff()                            //do what is necessary to turn GPS off
{
  digitalWrite(GPSPOWER, HIGH);          //turn off GPS power
}


void gpsSetup()
{
  ss.begin(GPSBaud);                     //startup soft serial for GPS

#ifdef UBLOX
  Serial.println(F("Configuring for UBLOX GPS"));
  clear_CurrentCFG();
  delay(250);
  setup_GPMode();
  delay(250);
  setup_FlightMode();
  delay(250);
  setup_Cyclic();
  delay(250);
  ss.println("$PUBX,40,GLL,0,0,0,0,0,0*5C");
  delay(50);
  ss.println("$PUBX,40,ZDA,0,0,0,0,0,0*44");
  delay(50);
  ss.println("$PUBX,40,VTG,0,0,0,0,0,0*5E");
  delay(50);
  ss.println("$PUBX,40,GSV,0,5,0,0,0,0*5C");
  delay(50);
  ss.println("$PUBX,40,GSA,0,0,0,0,0,0*4E");
  delay(50);
  ss.println("$PUBX,40,RMC,0,1,0,0,0,0*46");
  delay(50);
  ss.println("$PUBX,40,GGA,0,1,0,0,0,0*7F");
  delay(50);
  save_CurrentCFG();
  delay(50);
#endif
}


void send_Config(unsigned char *Config, int Length)
{
  //send config string to GPS
  int i;
  for (i = 0; i < Length; i++)
  {
    ss.write(Config[i]);
  }
}


void setup_GPMode()
{
  //turn off GLONASS Mode
  unsigned char setGP[] = {0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x8F, 0xB2};
  send_Config(setGP, sizeof(setGP));
  Serial.println(F("setupGPMode"));
}


void setup_FlightMode()
{
  //configure GPS for flight mode
  unsigned char setFlight[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  send_Config(setFlight, sizeof(setFlight));
  Serial.println(F("setupFlightMode"));
}


void setup_Cyclic()
{
  //put GPS in cyclic power save mode
  unsigned char setCyclic[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 };
  send_Config(setCyclic, sizeof(setCyclic));
  Serial.println(F("setupCyclic"));
}


void clear_CurrentCFG()
{
  //clear current GPS configuration
  unsigned char clearCFG[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x19, 0x98};
  send_Config(clearCFG, sizeof(clearCFG));
  Serial.println(F("ClearCurrentCFG"));
}


void save_CurrentCFG()
{
  //save GPS configuration
  unsigned char saveCFG[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA9};
  send_Config(saveCFG, sizeof(saveCFG));
  Serial.println(F("SaveCurrentCFG"));
}




