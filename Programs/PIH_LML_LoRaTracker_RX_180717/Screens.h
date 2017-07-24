/*
LoRaTracker Programs

Copyright of the author Stuart Robinson - 11/04/16

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
void writeSNR(byte lcol, byte lrow);
void writePktCount(byte lcol, byte lrow);


void updatescreen0()
{
  //prints the tracker lat, long and alt
  unsigned long ldist;
  byte lvar1;
  String lstring;
  display_Cls();
  display_Setfont(defaultfont);
  display_SetCurPos(0, 0);
  display_Text("LA ");
  display_Printfloat(Tlat, 5);    //5 decimal places is enough, gives location to the nearest approx 10M
  display_SetCurPos(0, 1);
  display_Text("LO ");
  display_Printfloat(Tlon, 5);
  display_SetCurPos(0, 2);
  display_Text("AL ");
  display_Printint(Talt);
  display_Text("M ");
  display_SetCurPos(0, 3);

  if (dist > 9999)
  {
    ldist = dist / 1000;
    display_Printint(ldist);
    display_Text("K ");
  }
  else
  {
    display_Printint(dist);
    display_Text("M ");
  }

  display_Printint(dir);
  display_Text("d");
  display_SetCurPos(13, 1);    //position cursor last char of second line

  if (lora_RXPacketType == 64) //packet type 64 indicates a relay packet
  {
    display_Text("R");
  }

  if (lora_RXPacketType == 60) //packet type 60 indicates a direct packet
  {
    display_Text("D");
  }
  writeSNR(0, 4);
  writePktCount(7, 4);
  
}


void writeSNR(byte lcol, byte lrow)
{
  byte lvar1;
  String lstring;
  if (lora_PacketSNR > 127)                 //calculate the SNR
  {
    lvar1 = ((255 - lora_PacketSNR) / 4);
    lstring = "-";
  }
  else
  {
    lvar1 = (lora_PacketSNR / 4);
    lstring = "+";
  }

  display_Setfont(defaultfont);
  display_SetCurPos(lcol, lrow);
  display_Text(lstring);                    //now print the SNR
  display_Printint(lvar1);
  display_Text("dB ");
}


void writePktCount(byte lcol, byte lrow)
{
  //now print the last two digits of lora_RXpacketCount so we can see if packets are arriving
  if (lora_RXpacketCount > 99)
  {
    lora_RXpacketCount = 0;
  }
  display_Setfont(defaultfont);
  display_SetCurPos(lcol, lrow);            //cursor to last two cols of bottom line
  display_Text("RX ");
  display_Printint(lora_RXpacketCount);     //send count 0-99 to LCD
}




