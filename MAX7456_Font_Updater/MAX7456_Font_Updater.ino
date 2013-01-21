// Code to copy a MCM font file (converted for C by mcm2h.pl) to the Arduino + Max7456 OSD
//
// MAX7456_font Sketch
//
// http://www.maxim-ic.com/tools/evkit/index.cfm?EVKit=558
// max7456 evaluation kit software
#define SPI_DATAOUT 51 //MOSI
#define SPI_DATAIN  50 //MISO 
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

#define READ_REG  0x80

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

byte charbuf[54]; // for NVM_read

#include <avr/pgmspace.h>
#include <MAX7456_Font.h>


//////////////////////////////////////////////////////////////
void setup()
{
  volatile byte spi_junk;
  Serial.begin(9600);
  Serial.flush();
  pinMode( 53, OUTPUT );
  pinMode(MAX7456SELECT,OUTPUT);
  digitalWrite(MAX7456SELECT,HIGH); //disable device

  pinMode(SPI_DATAOUT, OUTPUT);
  pinMode(SPI_DATAIN, INPUT);
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
  Serial.print("Embedded font is ");
  Serial.print(sizeof(fontdata));
  Serial.println("bytes long.");
  Serial.println("MAX7456>");
}

void reset_max7456()
{
  // force soft reset on Max7456
  digitalWrite(MAX7456SELECT,LOW);
  spi_writereg(VM0_reg,MAX7456_reset);
  digitalWrite(MAX7456SELECT,HIGH);
  delay(500);

  // set all rows to same character white level, 90%
  digitalWrite(MAX7456SELECT,LOW);
  for (byte x = 0; x < MAX_screen_rows; x++)
  {
    spi_writereg(x + 0x10,WHITE_level_90);
  }

  // make sure the Max7456 is enabled
  spi_writereg(VM0_reg,ENABLE_display);
  digitalWrite(MAX7456SELECT,HIGH);
}

//////////////////////////////////////////////////////////////
void loop()
{
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
      case '?': // read status
        digitalWrite(MAX7456SELECT,LOW);
        Serial.print("STAT=");
        Serial.println((int)spi_readreg(STAT_reg));
        digitalWrite(MAX7456SELECT,HIGH);
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
byte spi_transfer( byte data ) {
  SPDR = data; //transfer data with hardware SPI
  while ( !(SPSR & _BV(SPIF)) ) ;
  return SPDR;
}

void spi_writereg(byte r, byte d) {
  spi_transfer(r);
  spi_transfer(d);
}  

byte spi_readreg(byte r) {
  spi_transfer(r);
  return spi_transfer(0);
}  

//////////////////////////////////////////////////////////////
void show_font() //show all chars on 24 wide grid
{
  unsigned short x;

  // clear the screen
  digitalWrite(MAX7456SELECT,LOW);
  spi_writereg(DMM_reg,CLEAR_display);
  digitalWrite(MAX7456SELECT,HIGH);
  delay(1); // clearing display takes 20uS so wait some...
  // disable display
  digitalWrite(MAX7456SELECT,LOW);
  // spi_writereg(VM0_reg,DISABLE_display); 

  spi_writereg(DMM_reg,0x01); //16 bit trans w/o background, autoincrement
  spi_writereg(DMAH_reg,0); // set start address high
  spi_writereg(DMAL_reg,33); // set start address low (line 1 col 3 (0 based)

  // show all characters on screen (actually 0-254)
  for (x = 0;x<255;x++) {
    spi_writereg(DMDI_reg,(byte)x);
    if ((x%24)==23) {
      for (byte i=0;i<6;i++) spi_writereg(DMDI_reg,0);
    }
  }

  spi_writereg(DMDI_reg,END_string);

  // spi_writereg(VM0_reg,ENABLE_display_vert);  // turn on screen next vertical
  digitalWrite(MAX7456SELECT,HIGH);
}

void transfer_fontdata()
{
  if (sizeof(fontdata)!=16384) {
    Serial.println("ERROR: fontdata with invalid size, aborting!!!");
    return;
  }
  Serial.println("Downloading font to MAX7456 NVM, this may take a while...");
  for (int ch=0; ch<256; ch++) {
    Serial.print((int)ch);
    write_NVM_character(ch,fontdata+64*ch,1);
    Serial.println(" OK");
    delay(30);
  }
  // force soft reset on Max7456
  reset_max7456();
  show_font();
  
  Serial.println("");
  Serial.println("Done with font download");
  Serial.println("MAX7456>");
}

void wait_NVM() {
  while (spi_readreg(STAT_reg)&STATUS_reg_nvr_busy) ;
}

void write_NVM_character(byte ch, const byte* addr, byte progmem)
{
  byte x;
  // disable display
  digitalWrite(MAX7456SELECT,LOW);
  spi_writereg(VM0_reg,DISABLE_display);
  spi_writereg(CMAH_reg,ch);  // set start address high
  for(x = 0; x < NVM_ram_size; x++) // write out 54 (out of 64) bytes of character to shadow ram
  {
    spi_writereg(CMAL_reg,x); // set start address low
    if (progmem) {
      spi_writereg(CMDI_reg,pgm_read_byte_near(addr+x));
    } else { 
      spi_writereg(CMDI_reg,*(addr+x));
    }      
  }
  // transfer a 54 bytes from shadow ram to NVM
  spi_writereg(CMM_reg,WRITE_nvr);
  wait_NVM(); // NVM should be busy around 12ms
  spi_writereg(VM0_reg,ENABLE_display_vert);
  digitalWrite(MAX7456SELECT,HIGH);  
}


