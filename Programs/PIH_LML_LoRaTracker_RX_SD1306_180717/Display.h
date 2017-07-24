//Display.h
/*
LoRaTracker Programs

Copyright of the author Stuart Robinson - 18/07/17

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.


*/

const byte DisplayDelay = 10;              //delay to allow display backpack time to complete command     
const byte ClearDelay = 10;                //delay to allow display backpack time to clear screen
const byte contrast = 63;                  //Contrast value for this LCD, varies between modules, range noramlly 35-65, not needed for Digole displays
const byte endcommand = 13;                //this character value ends a text write. This can be 0 or 13 for digole display modules, its 13 for my Nokia 5110 Backpack 


void Display_Update()
{
 //defined for compatibility
}

//void Display_Clear()
void Display_Clear()
{
  //clears display
  ss.print("CL");                           //print text
  delay(ClearDelay);
}

//void Display_SetTextSize(byte textsize)
void Display_SetTextSize(byte lfont)
{
  //sets the current font number
  ss.print("SF");                           //print text
  ss.write(char(lfont));                    //print data
  delay(DisplayDelay);
}

//disp.print()
void disp.print(String Ltext)
{
  //adds text at cursior position
  ss.print("TT");                            //print text
  ss.print(Ltext);                           //print text
  ss.write(endcommand);
  delay(DisplayDelay);
}

//void Display_SetCursor(byte col, byte row)
void Display_SetCursor(byte lcol, byte lrow)
{
  //Sets the current cursor position
  ss.print("TP");                            //print text
  ss.write(char(lcol));                      //print data
  ss.write(char(lrow));                      //print data
  delay(DisplayDelay);
}

//void Display_SetContrast(byte contrast)
void display_SetContrast(byte lcont)
{
  //Sets the display contrast
  ss.print("CT");                             //print text
  ss.write(char(lcont));                      //print data
  delay(DisplayDelay);
}


void disp.print(int Ltemp)
{
  //prints an integer at current cursor position
  ss.print("TT");                             //print text
  ss.print(Ltemp);                            //print number
  ss.write(endcommand);
  delay(DisplayDelay);
}


void disp.print(float Ltemp, byte Lnumdecimals)
{
  //prints a float at current cursor position
  ss.print("TT");                            //print text
  ss.print(Ltemp, Lnumdecimals);             //print number
  ss.write(endcommand);
  delay(DisplayDelay);
}


void display_ClearAndHome(byte lfont)
{
  //clears display, sets the current font number and homes the cursor
  display_SetContrast(contrast);
  Display_Clear();
  delay(ClearDelay);
  Display_SetTextSize(lfont);
  Display_SetCursor(0, 0);
  delay(DisplayDelay);
}


//void Display_Setup()

