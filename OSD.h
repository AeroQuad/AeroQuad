/*
  AeroQuad v2.4.1 - June 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
 
/* 
   Special thanks to Alamo for contributing this capability! You will need to upload the correct character set to the MAX7456 first.  Please see:
   http://aeroquad.com/showthread.php?2942-OSD-implementation-using-MAX7456
   
   This class provides a basic on-screen display (OSD) for flying the AeroQuad in FPV. It can display
   battery voltage, altitude, compass heading, a flight clock, a centre reticle and an attitude indicator. Display elements will appear
   if defined below or the corresponding sensor is defined in AeroQuad.pde.
   
   The user must connect a MAX7456 OSD chip to the appropriate header pins on their Arduino. These pins are
   marked 'OSD' on the AeroQuad Shield v2. If the chip is not connected properly, this code may hang.
   If using the SparkFun MAX7456 breakout board, you should add a 10kOhm pull up resistor between 5V and the reset pin.
   You will need to update the character memory of your MAX7456 or you'll see garbage on screen. See the AQ forum thread.
   
   It's a good idea to use an external 5V linear regulator, such as an LM7805/LM317 or similar to power to the MAX7456. The MAX7456 can draw up to 100mA @ 5V
    - this would result in an extra ~0.7W being dissipated across the Arduino onboard regulator (for a 3S battery). That's a lot of power for a little regulator with minimal heatsinking!
*/

/*********************** User configuration section ***************************************/
#define ShowReticle            //Displays a reticle in the centre of the screen. 
#define ShowFlightTimer        //Displays how long the motors have been armed for since the Arduino was last reset
#define ShowAttitudeIndicator
#define ShowCallSign
//#define feet                   //Comment this line out for altitude measured in metres, uncomment it for feet

//Choose your video standard:
//#define NTSC
#define PAL

//You can configure positioning of various display elements below. #defines for elements which will not be displayed, can be ignored.
//The MAX7456 overlays characters in a grid 30 characters wide, 16/13 high (PAL/NTSC respectively). The row/column defines below
// correspond to positions in the grid of characters, with the origin at the top left. 0-origin indexing is used - ie row 0, col 0
// is the highest, leftmost character on the screen while row 15, col 29 is the bottom right (for PAL).
//Display elements start at the position you give and print to the right. They will wrap around to the next row if there are too few
// columns remaining on the row you specify.

//Battery voltage - 5 characters long
#define VOLTAGE_ROW 2
#define VOLTAGE_COL 1
//Compass reading - 5 characters long
#define COMPASS_ROW 1
#define COMPASS_COL 13
//Altitude reading - up to 8 characters long (32768 max)
#define ALTITUDE_ROW 1
#define ALTITUDE_COL 1
//Flight timer - 6 characters long
#define TIMER_ROW 1
#define TIMER_COL 23
//Callsign
#define CALLSIGN_ROW 2
#define CALLSIGN_COL 23
#ifdef ShowCallSign
byte *callsign = (byte*)"OH2FXR";
#endif

//Juice monitor, two battery config
#define JUICE_ROW 2
#define JUICE_COL 1
#define JUICE_MAXROWS 3 // limit the number of batteries shown....

/********************** End of user configuration section ********************************/

//OSD pins on AQ v2 shield:
#define CS 22 //SS_OSD
#define DOUT 51 //MOSI
#define DIN 50 //MISO
#define SCK 52 //SCLK

//MAX7456 register write addresses - see datasheet for lots of info
#define DMM   0x04 //Display memory mode register - for choosing 16bit/8bit write mode, clearing display memory, enabling auto-increment
#define DMAH  0x05 //Holds MSB of display memory address, for setting location of a character on display
#define DMAL  0x06 //Holds remaining 8 bits of display memory address - 480 characters displayed -> 9 bits req'd for addressing
#define DMDI  0x07 //Display memory data in - character address or attribute byte, depending on 8b/16b mode and DMAH[1]
#define VM0   0x00 //Video mode 0 register - for choosing, NTSC/PAL, sync mode, OSD on/off, reset, VOUT on/off
#define VM1   0x01 //Video mode 1 register - nothing very interesting in this one
#define RB0   0x10 //Row 0 brightness register - 15 more follow sequentially (ending at 0x1F)
#define STAT  0xA2 //Status register read address

//MAX7456 commands - provided in datasheet.
#define CLEAR_display 0x04
#define CLEAR_display_vert 0x06
#define END_string 0xff

#define WHITE_level_90 0x02
#define PAL_DETECTED 0

#if defined(AUTO_VIDEO_STANDARD) && (defined(NTSC) || defined(PAL))
#undef PAL
#undef NTSC
#endif

#if defined(NTSC)
  #define MAX_screen_size 390
  #define MAX_screen_rows 13
  #define ENABLE_display 0x08
  #define ENABLE_display_vert 0x0c
  #define MAX7456_reset 0x02
  #define DISABLE_display 0x00
#elif defined(PAL)
// all VM0 commands need bit 6 set to indicate PAL
  #define MAX_screen_size 480
  #define MAX_screen_rows 16
  #define ENABLE_display 0x48
  #define ENABLE_display_vert 0x4c
  #define MAX7456_reset 0x42
  #define DISABLE_display 0x40
#endif

//configuration for AI
#define LINE_ROW_0 0x80                //character address of a character with a horizontal line in row 0. Other rows follow this one
#define AI_MAX_PITCH_ANGLE (PI/4)      //bounds of scale used for displaying pitch. When pitch is >= |this number|, the pitch lines will be at top or bottom of bounding box
#define ROLL_L1_COL 10                 //column which the leftmost roll line will be printed in
#define ROLL_L2_COL 12
#define ROLL_R1_COL 17
#define ROLL_R2_COL 19                 //column which the rightmost roll line will be printed in
#define PITCH_L_COL 7
#define PITCH_R_COL 22
#define AI_DISPLAY_RECT_HEIGHT 9       //Height of rectangle bounding AI. Should be odd so that there is an equal space above/below the centre reticle

#define RETICLE_ROW (MAX_screen_rows/2)//centre row - don't change this
#define RETICLE_COL 14                 //reticle will be in this col, and col to the right

#define AI_TOP_PIXEL ((RETICLE_ROW - AI_DISPLAY_RECT_HEIGHT/2)*18)
#define AI_BOTTOM_PIXEL ((RETICLE_ROW + AI_DISPLAY_RECT_HEIGHT/2)*18)

// OSD profiling
#define OSD_PROFILE


#ifdef MAX7456_OSD

byte clear;

class OSD {
private:

  unsigned long prevUpdate; //armed time when last update occurred
  unsigned long prevTime; //previous time since start when OSD.update() ran
  unsigned long armedTime; //time motors have spent armed
  byte          prevFlightMode; // previous flightmode for reticle update

#ifdef OSD_PROFILE
  unsigned long prof_min,prof_max,prof_this,prof_start;
  unsigned long pt_start,pt_this,pt_min[8],pt_max[8];
  byte pt_task, prof_cnt;
  #define PROF_TASK_START { pt_start=micros(); }
  #define PROF_TASK_END { pt_this=micros()-pt_start; if (pt_this<pt_min[pt_task]) pt_min[pt_task]=pt_this; if (pt_this>pt_max[pt_task]) pt_max[pt_task]=pt_this; pt_task++; }
#else
  #define PROF_TASK_START
  #define PROF_TASK_END
#endif
  
#if defined(AUTO_VIDEO_STANDARD)
  unsigned MAX_screen_size;
  unsigned MAX_screen_rows;
  byte ENABLE_display;
  byte ENABLE_display_vert;
  byte MAX7456_reset;
  byte DISABLE_display;
#endif

public:
  OSD (void) {
    prevUpdate = 0;
    armedTime = 0;
    prevTime = currentTime;
#ifdef OSD_PROFILE
    prof_min=999999;
    prof_max=0;
    prof_cnt=0;
    for (int i=0; i<8; i++) { 
      pt_min[i]=999999;
      pt_max[i]=0;
    }
#endif
  }


  void initialize( void ) {
    int i;
    pinMode( CS, OUTPUT );
    pinMode( 53, OUTPUT ); //Default CS pin - needs to be output or SPI peripheral will behave as a slave instead of master
    digitalWrite( CS, HIGH );

    pinMode( DOUT, OUTPUT );
    pinMode( DIN, INPUT );
    pinMode( SCK, OUTPUT );

    // SPCR = 01010000
    //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
    //sample on leading edge of clk,system clock/4 rate (fastest)
    SPCR = (1 << SPE) | (1 << MSTR);
    clear = SPSR;
    clear = SPDR;
    delay( 10 ); 
    
    #if defined(AUTO_VIDEO_STANDARD)
    detectVideoStandard();
    #endif

    //Soft reset the MAX7456 - clear display memory
    spi_select();
    spi_write_nowait( VM0 ); //Writing to VM0 register, 1st write must not wait data ready!!!
    spi_write( MAX7456_reset ); //...reset bit
    spi_deselect();
    delay( 1 ); //Only takes ~100us typically

    //Set white level to 90% for all rows
    spi_select();
    for( i = 0; i < MAX_screen_rows; i++ ) {
      spi_write( RB0 + i );
      spi_write( WHITE_level_90 );
    }

    //ensure device is enabled
    spi_write( VM0 );
    spi_write( ENABLE_display );
    delay(100);
    //finished writing
    spi_deselect();  
    
    initDisplays(); //Print initial values to screen
  }

private:
  //Performs an 8-bit SPI transfer operation
  void spi_wait() {
    while ( !(SPSR & (1 << SPIF)) ) { }; //Wait until transmission done
  }

  void spi_select() {
//    spi_wait();
    digitalWrite(CS, LOW);
  }

  void spi_deselect() {
    spi_wait();
    digitalWrite(CS, HIGH);
  }

  //Performs an 8-bit SPI transfer operation
  void spi_write( char data ) {
    spi_wait();
    SPDR = data; //transfer data with hardware SPI
  }

  void spi_write_nowait( char data ) {
    SPDR = data; //transfer data with hardware SPI
  }

  char spi_read( char data ) {
    spi_write(data);
    spi_wait();
    return SPDR;
  }
    
  void initDisplays() {
    #ifdef ShowReticle
      byte buf[2];
      buf[0] = 0x01;
      buf[1] = 0x02;
      writeChars( buf, 2, 0, RETICLE_ROW, RETICLE_COL ); //write 2 chars to row (middle), column 14
      prevFlightMode=ACRO;
    #endif

    #ifdef ShowCallSign
      writeChars( callsign, strlen((char*)callsign), 0, CALLSIGN_ROW, CALLSIGN_COL );
    #endif

    #ifdef ShowFlightTimer
    updateTimer();
    #endif
    
    #ifdef AltitudeHold
    updateAltitude();
    #endif
    
    #ifdef HeadingMagHold
    updateHdg();
    #endif
    
    #ifdef BatteryMonitor
    updateVoltage();
    #endif
    
    #ifdef JuicMonitor
    updateJuice();
    #endif
  }
  
  
  //Clears (ie sets to be transparent) a column 'col' of some characters - clears centreRow+-offset
  void clearCol(unsigned col, unsigned centreRow, unsigned offset) {
    int i = 0;
    spi_select();
    spi_write( DMM );
    spi_write( 0x00 ); //16bit transfer, transparent BG
    
    for( i = ((centreRow-offset >= 0) ? centreRow-offset : 0) ; (i <= (centreRow + offset)) && (i <= MAX_screen_rows); i++ ) {
      spi_write( DMAH );
      spi_write( ( (i*30+col) > 0xff ) ? 0x01 :0x00 );
      spi_write( DMAL );
      spi_write( ( (i*30+col) > 0xff ) ? (byte)(i*30+col-0xff-1) : (byte)(i*30+col) );
      spi_write( DMDI );
      spi_write( 0x00 );
    }
    spi_deselect();
  }
  
  
  //Writes 'len' character address bytes to the display memory corresponding to row y, column x
  //Uses autoincrement mode so will wrap around to next row if 'len' is greater than the remaining
  //columns in row y
  void writeChars( byte* buf, unsigned len, byte blink, unsigned y, unsigned x ) {
    spi_select();
    
    //don't disable display before writing as this makes the entire screen flicker, instead of just the character modified
    spi_write( DMM );
    spi_write( (blink) ? 0x11 : 0x01 ); //16bit transfer, transparent BG, autoincrement mode
    //send starting display memory address (position of text)
    spi_write( DMAH );
    spi_write( ( (y*30+x) > 0xff ) ? 0x01 :0x00 );
    spi_write( DMAL );
    spi_write( ( (y*30+x) > 0xff ) ? (byte)(y*30+x-0xff-1) : (byte)(y*30+x) );
    
    //write out data
    for ( int i = 0; i < len; i++ ) {
      spi_write( DMDI );
      spi_write( buf[i] );
    }
    
    //Send escape 11111111 to exit autoincrement mode
    spi_write( DMDI );
    spi_write( END_string );

    //finished writing
    spi_deselect();
  }
  
#if defined(AUTO_VIDEO_STANDARD)
  void detectVideoStandard() {
    byte result;
    
    //this section isn't working yet - trying to get contents of STAT returns 0x00, when it should be 0x01 for PAL or 0x02 for NTSC
    Serial.println("Polling STAT");
    spi_select();
    spi_write( STAT );
    result = spi_read();
    spi_deselect();
    Serial.print("STAT= ");
    Serial.println((unsigned)result);
    
    if( (result & (0x01)) != 0 ) {
      //PAL signal detected on video in
      MAX_screen_size = 480;
      MAX_screen_rows = 16;
      ENABLE_display = 0x48;
      ENABLE_display_vert = 0x4c;
      MAX7456_reset = 0x42;
      DISABLE_display = 0x40;
    }
    else {
      //default to NTSC - most AQ users will use this, and author uses PAL so will notice bugs in autodetect!
      MAX_screen_size = 390;
      MAX_screen_rows = 13;
      ENABLE_display = 0x08;
      ENABLE_display_vert = 0x0c;
      MAX7456_reset = 0x02;
      DISABLE_display = 0x00;
    }
  }
#endif

#ifdef BattMonitor
  int prevVoltage;
  void updateVoltage(void) {
    int currentVoltage = batteryMonitor.getData()*10;
    if (currentVoltage!=prevVoltage) {
      char buf[6];
      snprintf(buf,6,"\004%2d.%1d",
        currentVoltage/10,currentVoltage%10);
      writeChars( buf, 5, 0, VOLTAGE_ROW, VOLTAGE_COL );
    }
  }
#endif

#ifdef JuicMonitor
  byte current_battery;
  void updateJuice(void) {
    byte buf[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // Sxx.xVxxx.xA
    current_battery = (current_battery+1) % min(JUICE_MAXROWS,juiceMonitor.getNB());     
    if (juiceMonitor.isI(current_battery)) {
      unsigned _u = (unsigned)(10.0 * juiceMonitor.getU(current_battery));
      unsigned _i = (unsigned)(10.0 * juiceMonitor.getI(current_battery));
      snprintf((char*)buf,20,"%c%2u.%1uV%3u.%1uA%4u\020",
               juiceMonitor.getOSDsym(current_battery),
               _u/10,_u%10,_i/10,_i%10,
               (unsigned)juiceMonitor.getC(current_battery));
    } else {
      unsigned _u = 10.0 * juiceMonitor.getU(current_battery);
      snprintf((char*)buf,20,"%c%2u.%1uV",
               juiceMonitor.getOSDsym(current_battery),_u/10,_u%10);
    }
    writeChars( buf, 19, (juiceMonitor.getA(current_battery) != OK), JUICE_ROW+current_battery, JUICE_COL );
  }
#endif

#ifdef AltitudeHold
  int lastAltitude;
  void updateAltitude(void) {
    #ifdef feet
    int currentAltitude = (int)(altitude.getData()/0.3048);
    #else
    int currentAltitude = (int)altitude.getData();
    #endif
    if (currentAltitude != lastAltitude) {
      byte buf[8];
      snprintf((char*)buf,8,"%c%d",(ON==altitudeHold)?0x09:0x08,(int)currentAltitude);
      writeChars( buf, strlen((char*)buf)+1, 0, ALTITUDE_ROW, ALTITUDE_COL );
      //write the null terminator - this will clear extra columns, so 10->9 doesn't become 90 on screen
      lastAltitude=currentAltitude;
    }
  }
#endif

#ifdef HeadingMagHold
  unsigned lastHdg;
  void updateHdg(void) {
    unsigned currentHdg = (unsigned) flightAngle->getDegreesHeading(YAW);
    if (currentHdg!=lastHdg) {
      lastHdg=currentHdg;
      byte buf[6];
      snprintf((char*)buf,6,"\006%03u\007",currentHdg);
      writeChars( buf, 5, 0, COMPASS_ROW, COMPASS_COL );
    }
  }
#endif

#ifdef ShowFlightTimer
void updateTimer(void) {
  if( (armed == true) ) {
    armedTime += ( currentTime-prevTime );
  }
  prevTime = currentTime;
  unsigned armedTimeSecs = armedTime/1000000 ;
  if (armedTimeSecs!=prevUpdate) {
    prevUpdate=armedTimeSecs;
    char timerAscii[7]; //clock symbol, two chars for mins, colon, two chars for secs, null terminator
    snprintf(timerAscii,7,"\005%02u:%02u",
      (armedTimeSecs / 60) % 100,
      armedTimeSecs % 60);
    writeChars( (byte*) timerAscii, 6, 0, TIMER_ROW, TIMER_COL );
  }
}
#endif

#ifdef ShowReticle
void updateReticle(void) {
  if (prevFlightMode!=flightMode) {
    byte buf[2];
    buf[0] = (ACRO==flightMode)?0x01:0x11;
    buf[1] = (ACRO==flightMode)?0x02:0x12;
    writeChars( buf, 2, 0, RETICLE_ROW, RETICLE_COL ); //write 2 chars to row (middle), column 14
    prevFlightMode=flightMode;
  }
}
#endif

#ifdef ShowAttitudeIndicator
void updateAI( void ) {
  float roll = flightAngle->getData(ROLL);
  float pitch = flightAngle->getData(PITCH);

  unsigned centreRow = RETICLE_ROW*18 + 10;  //pixel row which corresponds to an angle of zero pitch - same row as centre reticle
  int pitchPixelRow = constrain( (int)centreRow + (int)( (pitch/AI_MAX_PITCH_ANGLE)*(centreRow-AI_TOP_PIXEL) ), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );  //centre + proportion of full scale
  
  byte pitchLine = LINE_ROW_0 + (pitchPixelRow % 18);
  byte empty = 0x00;
  
  //write pitch lines, clear spaces above/below pitch line so that old lines don't remain
  clearCol( PITCH_L_COL, RETICLE_ROW, AI_DISPLAY_RECT_HEIGHT/2 );
  writeChars( &pitchLine, 1, 0, pitchPixelRow/18, PITCH_L_COL );
  clearCol( PITCH_R_COL, RETICLE_ROW, AI_DISPLAY_RECT_HEIGHT/2 );
  writeChars( &pitchLine, 1, 0, pitchPixelRow/18, PITCH_R_COL );
  
  //calculating which row (in pixels) each roll line should be on. We know the desired angle between the 'line' displayed by the two
  //  roll lines, plus the distance between the centre of reticle and centre of each roll line. so we can take tan(roll) to find the vertical offset
  int distFar = (ROLL_R2_COL - (RETICLE_COL + 1))*12 + 6; //horizontal pixels between centre of reticle and centre of far angle line
  int distNear = (ROLL_R1_COL- (RETICLE_COL + 1))*12 + 6;
  int farRightRow = constrain( centreRow - (int)(((float)distFar)*tan(roll)), AI_TOP_PIXEL, AI_BOTTOM_PIXEL ); //row of far right angle line, in pixels from top
  int nearRightRow = constrain( centreRow - (int)(((float)distNear)*tan(roll)), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );
  int farLeftRow = constrain( centreRow - (farRightRow - centreRow), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );
  int nearLeftRow = constrain( centreRow - (nearRightRow - centreRow), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );
  
  //converting pixel offsets to character addresses
  byte nearRightRollLine = LINE_ROW_0 + (nearRightRow % 18);
  byte farRightRollLine = LINE_ROW_0 + (farRightRow % 18);
  byte nearLeftRollLine = LINE_ROW_0 + (nearLeftRow % 18);
  byte farLeftRollLine = LINE_ROW_0 + (farLeftRow % 18);
  
  //clearing old lines, writing new ones to screen
  clearCol( ROLL_L1_COL, RETICLE_ROW, AI_DISPLAY_RECT_HEIGHT/2 );
  writeChars( &farLeftRollLine, 1, 0, farLeftRow/18, ROLL_L1_COL );
  clearCol( ROLL_L2_COL, RETICLE_ROW, AI_DISPLAY_RECT_HEIGHT/2 );
  writeChars( &nearLeftRollLine, 1, 0, nearLeftRow/18, ROLL_L2_COL );
  clearCol( ROLL_R1_COL, RETICLE_ROW, AI_DISPLAY_RECT_HEIGHT/2 );
  writeChars( &nearRightRollLine, 1, 0, nearRightRow/18, ROLL_R1_COL );
  clearCol( ROLL_R2_COL, RETICLE_ROW, AI_DISPLAY_RECT_HEIGHT/2 );
  writeChars( &farRightRollLine, 1, 0, farRightRow/18, ROLL_R2_COL );
}
#endif

public:
  //updates to display memory can make text flicker - we want to minimise # updates
  void update(void) {
#ifdef OSD_PROFILE
    prof_start=micros();
    pt_task=0;
#endif
    #ifdef ShowReticle
    PROF_TASK_START
    updateReticle();
    PROF_TASK_END
    #endif

    #ifdef BattMonitor
    PROF_TASK_START
    updateVoltage();
    PROF_TASK_END
    #endif
    
    #ifdef JuicMonitor
    PROF_TASK_START
    updateJuice();
    PROF_TASK_END
    #endif
    
    #ifdef AltitudeHold
    PROF_TASK_START
    updateAltitude();
    PROF_TASK_END
    #endif
    
    #ifdef HeadingMagHold
    PROF_TASK_START
    updateHdg();
    PROF_TASK_END
    #endif
    
    #ifdef ShowFlightTimer
    PROF_TASK_START
    updateTimer();
    PROF_TASK_END
    #endif
    
    #ifdef ShowAttitudeIndicator
    PROF_TASK_START
    updateAI();
    PROF_TASK_END
    #endif
#ifdef OSD_PROFILE
    prof_this=micros()-prof_start;
    if (prof_this<prof_min) prof_min=prof_this;
    if (prof_this>prof_max) prof_max=prof_this;
    if ((prof_cnt&15)==15) {
      char buf[20];
      snprintf(buf,20,"%u:%u:%u   ",(unsigned)prof_min,(unsigned)prof_this,(unsigned)prof_max);
      writeChars( (byte*)buf, strlen(buf), 0, 13, 1 );
    }
    if (prof_cnt%15==7) {
      pt_task=(prof_cnt>>4)&7;
      char buf[20];
      snprintf(buf,20,"T%u:%u:%u   ",(unsigned)pt_task,(unsigned)pt_min[pt_task],(unsigned)pt_max[pt_task]);
      writeChars( (byte*)buf, strlen(buf), 0, 14, 1 );
    }
    prof_cnt++;    
#endif
  }

};

#endif

