//Display.h
/*
LoRaTracker Programs

Copyright of the author Stuart Robinson - 11/04/16

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

Normally used with Nokia 5110 display and Arduino backapack, this display is 6 lines of 14 characters.

*/

const byte DisplayDelay = 10;              //delay to allow display backpack time to complete command     
const byte ClearDelay = 10;                //delay to allow display backpack time to clear screen
const byte contrast = 63;                  //Contrast value for this LCD, varies between modules, range noramlly 35-65, not needed for Digole displays
const byte endcommand = 13;                //this character value ends a text write. This can be 0 or 13 for digole display modules, its 13 for my Nokia 5110 Backpack 



void display_Cls()
{
  //clears display
  disp.print("CL");                           //print text
  delay(ClearDelay);
}


void display_Setfont(byte lfont)
{
  //sets the current font number
  disp.print("SF");                           //print text
  disp.write(char(lfont));                    //print data
  delay(DisplayDelay);
}


void display_Text(String Ltext)
{
  //adds text at cursior position
  disp.print("TT");                            //print text
  disp.print(Ltext);                           //print text
  disp.write(endcommand);
  delay(DisplayDelay);
}


void display_SetCurPos(byte lcol, byte lrow)
{
  //Sets the current cursor position
  disp.print("TP");                            //print text
  disp.write(char(lcol));                      //print data
  disp.write(char(lrow));                      //print data
  delay(DisplayDelay);
}


void display_SetContrast(byte lcont)
{
  //Sets the display contrast
  disp.print("CT");                             //print text
  disp.write(char(lcont));                      //print data
  delay(DisplayDelay);
}


void display_Printint(int Ltemp)
{
  //prints an integer at current cursor position
  disp.print("TT");                             //print text
  disp.print(Ltemp);                            //print number
  disp.write(endcommand);
  delay(DisplayDelay);
}


void display_Printfloat(float Ltemp, byte Lnumdecimals)
{
  //prints a float at current cursor position
  disp.print("TT");                            //print text
  disp.print(Ltemp, Lnumdecimals);             //print number
  disp.write(endcommand);
  delay(DisplayDelay);
}


void display_ClearAndHome(byte lfont)
{
  //clears display, sets the current font number and homes the cursor
  display_SetContrast(contrast);
  display_Cls();
  delay(ClearDelay);
  display_Setfont(lfont);
  display_SetCurPos(0, 0);
  delay(DisplayDelay);
}
