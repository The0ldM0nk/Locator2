/*
LoRaTracker Programs

Copyright of the author Stuart Robinson - 18/07/17

These programs may be used free of charge for personal, recreational and
educational purposes only.

This program, or parts of it, may not be used for or in connection with
any commercial purpose without the explicit permission of the author
Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the
programs are suitable for the intended purpose and free from errors.

*/

//routines in this file
void updatescreen0();
void updatescreen1();
void writeSNR(byte lcol, byte lrow);
void writePktCount(byte lcol, byte lrow);


void updatescreen0()
{
  //prints the tracker lat, long and alt
  unsigned long ldist;
  byte lvar1;
  Display_Clear();
  Display_SetTextSize(1);
  Display_SetCursor(0, 0);
  disp.print("LAT ");
  disp.print(Tlat, 5);    //5 decimal places is enough, gives location to the nearest approx 10M
  Display_SetCursor(0, 2);
  disp.print("LON ");
  disp.print(Tlon, 5);
  Display_SetCursor(0, 4);
  disp.print("ALT ");
  disp.print(Talt);
  disp.print("M ");
  
  #ifdef GPS
  Display_SetCursor(0, 6);
  disp.print("Dist ");
  if (dist > 9999)
  {
    ldist = dist / 1000;
    disp.print(ldist);
    disp.print("K ");
  }
  else
  {
    disp.print(dist);
    disp.print("M ");
  }

  disp.print("  Dir ");
  disp.print(dir);
  disp.print("d");
  #endif

  writeSNR(0, 7);
  writePktCount(9, 7);
  
}


void updatescreen1()
{
  //prints the tracker lat, long and alt
  unsigned long ldist;
  byte lvar1;
  Display_Clear();
  Display_SetTextSize(2);
  Display_SetCursor(0, 0);
  disp.print(Tlat, 5);    //5 decimal places is enough, gives location to the nearest approx 10M
  Display_SetCursor(0, 2);
  disp.print(Tlon, 5);
  Display_SetCursor(0, 4);
  disp.print(Talt);
  disp.print("M ");
  
  #ifdef GPS
  Display_SetCursor(0, 6);
  if (dist > 9999)
  {
    ldist = dist / 1000;
    disp.print(ldist);
    disp.print("K ");
  }
  else
  {
    disp.print(dist);
    disp.print("M ");
  }

  disp.print(dir);
  disp.print("d");
  #endif
  
 
}


void writeSNR(byte lcol, byte lrow)
{
  byte lvar1;
  char tempchar;
  if (lora_PacketSNR > 127)                 //calculate the SNR
  {
    lvar1 = ((255 - lora_PacketSNR) / 4);
    tempchar = '-';
  }
  else
  {
    lvar1 = (lora_PacketSNR / 4);
    tempchar = '+';
  }

  Display_SetTextSize(1);
  Display_SetCursor(lcol, lrow);
  disp.write(tempchar);                    //now print the SNR
  disp.print(lvar1);
  disp.print("dB ");
}


void writePktCount(byte lcol, byte lrow)
{
  //now print the last two digits of lora_RXpacketCount so we can see if packets are arriving

  Display_SetTextSize(1);
  Display_SetCursor(lcol, lrow);            //cursor to last two cols of bottom line
  disp.print("Pkts ");
  disp.print(lora_RXpacketCount);     //send count 0-99 to LCD
}




