// Code to copy a MCM font file (converted for C by mcm2h.pl) to the Arduino + Max7456 OSD
//
// MAX7456_font Sketch
//
// http://www.maxim-ic.com/tools/evkit/index.cfm?EVKit=558
// max7456 evaluation kit software

#define DATAOUT 51 //MOSI
#define DATAIN  50 //MISO 
#define SPICLOCK  52 //SCLK
#define MAX7456SELECT 22 //22-SS

//MAX7456 opcodes
#define VM0_reg   0x00
#define VM1_reg   0x01
#define DMM_reg   0x04
#define DMAH_reg  0x05
#define DMAL_reg  0x06
#define DMDI_reg  0x07
#define CMM_reg   0x08
#define CMAH_reg  0x09
#define CMAL_reg  0x0A
#define CMDI_reg  0x0B
#define STAT_reg  0xA0

//MAX7456 commands
#define CLEAR_display 0x04
#define CLEAR_display_vert 0x06
#define END_string 0xff
#define WRITE_nvr 0xa0
// with NTSC
#define ENABLE_display 0x08
#define ENABLE_display_vert 0x0c
#define MAX7456_reset 0x02
#define DISABLE_display 0x00

// with PAL
// all VM0_reg commands need bit 6 set
//#define ENABLE_display 0x48
//#define ENABLE_display_vert 0x4c
//#define MAX7456_reset 0x42
//#define DISABLE_display 0x40

#define WHITE_level_80 0x03
#define WHITE_level_90 0x02
#define WHITE_level_100 0x01
#define WHITE_level_120 0x00

#define MAX_font_rom 0xff
#define STATUS_reg_nvr_busy 0x20
#define NVM_ram_size 0x36

// with NTSC
#define MAX_screen_rows 0x0d //13

// with PAL
//#define MAX_screen_rows 0x10 //16

volatile int  incomingByte;

#include "font.h"

//////////////////////////////////////////////////////////////
void setup()
{
  byte spi_junk;
  int x;
  Serial.begin(9600);
  Serial.flush();

  pinMode( 53, OUTPUT );
  pinMode(MAX7456SELECT,OUTPUT);
  digitalWrite(MAX7456SELECT,HIGH); //disable device

  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK,OUTPUT);

  // SPCR = 01010000
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //sample on leading edge of clk,system clock/4 rate (4 meg)
  SPCR = (1<<SPE)|(1<<MSTR);
  spi_junk=SPSR;
  spi_junk=SPDR;
  delay(250);

  reset_max7456();

  incomingByte = 0;

  //display all 256 internal MAX7456 characters
  show_font();  

  Serial.println("Ready for commands: D - download font");
  Serial.println("MAX7456>");
  delay(100);  
}

void reset_max7456()
{
  // force soft reset on Max7456
  digitalWrite(MAX7456SELECT,LOW);
  spi_write(VM0_reg);
  spi_write(MAX7456_reset);
  digitalWrite(MAX7456SELECT,HIGH);
  delay(500);

  // set all rows to same character white level, 90%
  digitalWrite(MAX7456SELECT,LOW);
  for (byte x = 0; x < MAX_screen_rows; x++)
  {
    spi_write(x + 0x10);
    spi_write(WHITE_level_90);
  }

  // make sure the Max7456 is enabled
  spi_write(VM0_reg);
  spi_write(ENABLE_display);
  digitalWrite(MAX7456SELECT,HIGH);
}

void transfer_fontdata()
{    
  Serial.println("Downloading font to MAX7456 NVM, this may take a while...");
  for (byte i=0;i<255;i++) {
    write_NVM(i,fontdata+64*i); // one char is 64 bytes on data (54 used)
    Serial.print("\r ");
    Serial.print(i+1);
    Serial.print("/256 written  ");
  }

  // force soft reset on Max7456
  digitalWrite(MAX7456SELECT,LOW);
  spi_write(VM0_reg);
  spi_write(MAX7456_reset);
  digitalWrite(MAX7456SELECT,HIGH);
  delay(500);

  Serial.println("");
  Serial.println("Done with font download");
  Serial.println("MAX7456>");
}
//////////////////////////////////////////////////////////////
void loop()
{
  byte x;
  
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    incomingByte = Serial.read();

    switch(incomingByte) // wait for commands
    {
      case 'D': // download font
        transfer_fontdata();
      break;
      case 'r': // reset
        reset_max7456();
      break;
      case 's': // show charset
        show_font();
      break;
      default:
        Serial.println("invalid command");
      break;
    }
    Serial.println("MAX7456>");
  }
  
}

//////////////////////////////////////////////////////////////
//Performs an 8-bit SPI transfer operation
void spi_wait() {
  while ( !(SPSR & (1 << SPIF)) ) { }; //Wait until transmission done
}

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

//////////////////////////////////////////////////////////////
void show_font() //show all chars on 24 wide grid
{
  int x;
  byte char_address_hi, char_address_lo;

  char_address_hi = 0;
  char_address_lo = 60; // start on third line
 //Serial.println("write_new_screen");   

  // clear the screen
  digitalWrite(MAX7456SELECT,LOW);
  spi_write(DMM_reg);
  spi_write(CLEAR_display);
  digitalWrite(MAX7456SELECT,HIGH);

  // disable display
  digitalWrite(MAX7456SELECT,LOW);
  spi_write(VM0_reg); 
  spi_write(DISABLE_display);

  spi_write(DMM_reg); //dmm
  spi_write(0x01); //16 bit trans w/o background, autoincrement

  spi_write(DMAH_reg); // set start address high
  spi_write(char_address_hi);

  spi_write(DMAL_reg); // set start address low
  spi_write(char_address_lo);

  // show all characters on screen
  for (x = 0;x<254;x++) {
    spi_write(DMDI_reg);
    spi_write(x);
  }

  spi_write(DMDI_reg);
  spi_write(END_string);

  spi_write(VM0_reg); // turn on screen next vertical
  spi_write(ENABLE_display_vert);
  digitalWrite(MAX7456SELECT,HIGH);
}

//////////////////////////////////////////////////////////////
void write_NVM(byte ch, const byte *data)
{
  byte x;
  byte char_address_hi, char_address_lo;

  // disable display
  digitalWrite(MAX7456SELECT,LOW);
  spi_write(VM0_reg); 
  spi_write(DISABLE_display);

  spi_write(CMAH_reg); // set start address high
  spi_write(ch);

  for(x = 0; x < NVM_ram_size; x++) // write out 54 (out of 64) bytes of character to shadow ram
  {
    spi_write(CMAL_reg); // set start address low
    spi_write(x);
    spi_write(CMDI_reg);
    spi_write(data[x]);
  }

  // transfer a 54 bytes from shadow ram to NVM
  // spi_write(CMM_reg);
  // spi_write(WRITE_nvr);
  
  // wait until bit 5 in the status register returns to 0 (12ms)
  while ((spi_read(STAT_reg) & STATUS_reg_nvr_busy) != 0x00);

  spi_write(VM0_reg); // turn on screen next vertical
  spi_write(ENABLE_display_vert);
  digitalWrite(MAX7456SELECT,HIGH);  
}
