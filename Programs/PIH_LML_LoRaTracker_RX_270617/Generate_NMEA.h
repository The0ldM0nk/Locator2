//Generate_NMEA.h
/*
LoRaTracker Programs

Copyright of the author Stuart Robinson - 11/04/16

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

Generates GPGGA and GPRMC sentences from decimal Latitude, Longitude and Altitude values
Sends to Console port

For use with mapping applications such as MemoryMap (PC) and AlpinQuest (Android)

/*
The GPS Bluetooth Mouse application for Android requires;
1. That GPGGA and GPRMC sentances are present
2. That the time field (GPGGA and GPRMC) be present
3. That the date field (GPRMC) be present
4. The sentence checksum can be wrong or missing

For Memory Map (PC) the above applies and the checksum need to be correct.
Memory Map will report an GPS communications error if it does not receive data approx every 5 seconds
The 'data' does not need to be a NMEA string it can be replaced with a CR/LF as a keep alive.

The combination of Bluetooth GPS mouse and Alpinquest on Android devices is not so fussy.

*/

#define Payload_buffer 125              //will be used to generate NMEA output

void print_NMEA_payload(byte lCount)
{
  //prints payload to console port
  int i;

  for (i = 0; i <= lCount; i++)
  {
    Serial.write(lora_TXBUFF[i]);
  }
  Serial.println();
}

float convert_degrees(float decimaldegrees)
{
  int integerdegrees;
  float decimals, answer;
  integerdegrees = (int) decimaldegrees;
  decimals = decimaldegrees - integerdegrees;
  decimals = (decimals * 6) / 10;
  answer = (integerdegrees + decimals) * 100;
  return answer;
}

char Hex(char lchar)
{
  //used in CRC calculation
  char Table[] = "0123456789ABCDEF";
  return Table[lchar];
}

void replaceSpaces(byte lcount)
{
  byte i;
  for (i = 0; i <= lcount; i++)
  {
    if (lora_TXBUFF[i] == ' ')
    {
      lora_TXBUFF[i] = '0';
    }
  }
}

byte addChecksum(byte lcount)
{
  byte i, checksum;

  checksum = 0;

  for (i = 1; i <= lcount; i++)
  {
    checksum = checksum ^ lora_TXBUFF[i];
  }

  lora_TXBUFF[lcount++] = '*';
  lora_TXBUFF[lcount++] = Hex((checksum >> 4) & 15);   //first digit of checksum
  lora_TXBUFF[lcount] = Hex(checksum & 15);            //last digit of checksum
  return lcount;
}


int send_NMEA(char *lora_TXBUFF, float latfloat, float lonfloat, float alt)
{
  int count, i, intalt;
  float latitude, longitude;
  byte checksum;

  char LatString[12], LonString[12], AltString[10];
  char Latquad = 'N', Lonquad = 'E';

  if (latfloat < 0)
  {
    Latquad = 'S';
    latfloat = -latfloat;
  }

  if (lonfloat < 0)
  {
    Lonquad = 'W';
    lonfloat = -lonfloat;
  }

  latitude = convert_degrees(latfloat);
  longitude = convert_degrees(lonfloat);
  intalt = (int) alt;

  dtostrf(latitude, 9, 4, LatString);                        //this will create spaces where leading zeros would be present in NMEA output
  dtostrf(longitude, 10, 4, LonString);
  dtostrf(intalt, 1, 0, AltString);

  memset(lora_TXBUFF, 0, sizeof(lora_TXBUFF));               //clear array

  snprintf(lora_TXBUFF,
           Payload_buffer,
           "$GPGGA,000000.000,%s,%c,%s,%c,1,4,3.16,%s.0,M,53.3,M,,",
           LatString,
           Latquad,
           LonString,
           Lonquad,
           AltString
          );

  count = strlen(lora_TXBUFF);                               //how long is the array ?
  replaceSpaces(count);                                      //replace spaces with 0s
  count = addChecksum(count);                                //checksum adds characters to array, need to pick up new end value
  print_NMEA_payload(count);                                 //print the GPGGA

  memset(lora_TXBUFF, 0, sizeof(lora_TXBUFF));               //clear array

  snprintf(lora_TXBUFF,
           Payload_buffer,
           "$GPRMC,000000.000,A,%s,%c,%s,%c,0.00,0.00,010101,,,A",
           LatString,
           Latquad,
           LonString,
           Lonquad,
           AltString
          );

  count = strlen(lora_TXBUFF);                               //how long is the array ?
  replaceSpaces(count);                                      //replacespaces with 0s
  count = addChecksum(count);                                //checksum adds characters to array, need to pick up new end value
  print_NMEA_payload(count);                                 //print the GPRMC
}





