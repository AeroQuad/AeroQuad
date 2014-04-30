
#define METRIC 0
#define IMPERIAL 1

//Analog input defines
const uint16_t voltagePin=0;
const uint16_t vidvoltagePin=2;
const uint16_t amperagePin=1;
const uint16_t rssiPin=3;
const uint16_t temperaturePin=6;            // Temperature pin 6 for original Rushduino Board V1.2
const uint8_t rssiSample=30;

//General use variables
int tenthSec=0;
int TempBlinkAlarm=0;                       // Temporary for blink alarm 
int BlinkAlarm=0;                           // This is turning on and off at selected freq. (alarm)
int Blink10hz=0;                            // This is turning on and off at 10hz
int lastCallSign=0;                         // callsign_timer
uint8_t rssiTimer=0;
uint8_t accCalibrationTimer=0;
uint8_t magCalibrationTimer=0;
uint8_t eepromWriteTimer=0;

unsigned int allSec=0;

// Config status and cursor location
uint8_t ROW=10;
uint8_t COL=4;
uint8_t configPage=MINPAGE;
uint8_t configMode=0;
uint8_t fontMode = 0;
uint8_t fontData[54];
uint8_t nextCharToRequest;
uint8_t lastCharToRequest;
uint8_t retransmitQueue;

// Mode bits
uint32_t mode_armed;
uint32_t mode_stable;
uint32_t mode_horizon;
uint32_t mode_baro;
uint32_t mode_mag;
uint32_t mode_gpshome;
uint32_t mode_gpshold;
uint32_t mode_osd_switch;

// Settings Locations
enum Setting_ {
  S_CHECK_,		// used for check
  S_RSSIMIN,
  S_RSSIMAX,
  S_RSSI_ALARM,
  S_MWRSSI,
  S_PWMRSSI,
  S_VOLTAGEMIN,
  S_BATCELLS,
  S_DIVIDERRATIO,
  S_MAINVOLTAGE_VBAT,
  S_VIDDIVIDERRATIO,
  S_VIDVOLTAGE_VBAT,
  S_TEMPERATUREMAX,
  S_BOARDTYPE,
  S_DISPLAYGPS,
  S_COORDINATES,
  S_HEADING360,
  S_UNITSYSTEM,
  S_VIDEOSIGNALTYPE,
  S_RESETSTATISTICS,
  S_ENABLEADC,
  S_USE_BOXNAMES,
  S_BLINKINGHZ,    // selectable alarm blink freq
  S_MWAMPERAGE,
  S_AMPOFFSET,
  S_CLIMB_RATE_ALARM,
  
  S_CS0,      // 10 callsign char locations
  S_CS1,
  S_CS2,
  S_CS3,
  S_CS4,
  S_CS5,
  S_CS6,
  S_CS7,
  S_CS8,
  S_CS9,
  // EEPROM_SETTINGS must be last for H/W settings!
  EEPROM_SETTINGS,  //33
  
// Screen item Locations
// ********* EEProm enum data position and display On/Off option for all items on screen ****************
// Enum valid for both PAL/NTSC  
  L_GPS_NUMSATPOSITIONROW,    //34
  L_GPS_NUMSATPOSITIONCOL,
  L_GPS_NUMSATPOSITIONDSPL,
  L_GPS_DIRECTIONTOHOMEPOSROW,
  L_GPS_DIRECTIONTOHOMEPOSCOL,
  L_GPS_DIRECTIONTOHOMEPOSDSPL,
  L_GPS_DISTANCETOHOMEPOSROW,
  L_GPS_DISTANCETOHOMEPOSCOL,
  L_GPS_DISTANCETOHOMEPOSDSPL,
  L_SPEEDPOSITIONROW,
  L_SPEEDPOSITIONCOL,
  L_SPEEDPOSITIONDSPL,
  L_GPS_ANGLETOHOMEPOSROW,
  L_GPS_ANGLETOHOMEPOSCOL,
  L_GPS_ANGLETOHOMEPOSDSPL,
  L_MW_GPS_ALTPOSITIONROW,
  L_MW_GPS_ALTPOSITIONCOL,
  L_MW_GPS_ALTPOSITIONDSPL,
  L_SENSORPOSITIONROW,
  L_SENSORPOSITIONCOL,
  L_SENSORPOSITIONDSPL,
  L_MW_HEADINGPOSITIONROW,
  L_MW_HEADINGPOSITIONCOL,
  L_MW_HEADINGPOSITIONDSPL,
  L_MW_HEADINGGRAPHPOSROW,
  L_MW_HEADINGGRAPHPOSCOL,
  L_MW_HEADINGGRAPHPOSDSPL,
  L_TEMPERATUREPOSROW,
  L_TEMPERATUREPOSCOL,
  L_TEMPERATUREPOSDSPL,

  L_MW_ALTITUDEPOSITIONROW,
  L_MW_ALTITUDEPOSITIONCOL,
  L_MW_ALTITUDEPOSITIONDSPL,
  L_CLIMBRATEPOSITIONROW,
  L_CLIMBRATEPOSITIONCOL,
  L_CLIMBRATEPOSITIONDSPL,
  L_HORIZONPOSITIONROW,
  L_HORIZONPOSITIONCOL,
  L_HORIZONPOSITIONDSPL,
  L_HORIZONSIDEREFROW,
  L_HORIZONSIDEREFCOL,
  L_HORIZONSIDEREFDSPL,
  L_HORIZONCENTERREFROW,
  L_HORIZONCENTERREFCOL,
  L_HORIZONCENTERREFDSPL,  
    
  L_CURRENTTHROTTLEPOSITIONROW,
  L_CURRENTTHROTTLEPOSITIONCOL,
  L_CURRENTTHROTTLEPOSITIONDSPL,
  L_FLYTIMEPOSITIONROW,
  L_FLYTIMEPOSITIONCOL,
  L_FLYTIMEPOSITIONDSPL,
  L_ONTIMEPOSITIONROW,
  L_ONTIMEPOSITIONCOL,
  L_ONTIMEPOSITIONDSPL,
  L_MOTORARMEDPOSITIONROW,
  L_MOTORARMEDPOSITIONCOL,
  L_MOTORARMEDPOSITIONDSPL,
  L_MW_GPS_LATPOSITIONROW,
  L_MW_GPS_LATPOSITIONCOL,
  L_MW_GPS_LATPOSITIONDSPL,
  L_MW_GPS_LONPOSITIONROW,
  L_MW_GPS_LONPOSITIONCOL,
  L_MW_GPS_LONPOSITIONDSPL,
  L_RSSIPOSITIONROW,
  L_RSSIPOSITIONCOL,
  L_RSSIPOSITIONDSPL,
  L_VOLTAGEPOSITIONROW,
  L_VOLTAGEPOSITIONCOL,
  L_VOLTAGEPOSITIONDSPL,  
  L_MAINBATLEVEVOLUTIONROW,
  L_MAINBATLEVEVOLUTIONCOL,
  L_MAINBATLEVEVOLUTIONDSPL,  
  L_VIDVOLTAGEPOSITIONROW,
  L_VIDVOLTAGEPOSITIONCOL,
  L_VIDVOLTAGEPOSITIONDSPL,
  L_AMPERAGEPOSITIONROW,
  L_AMPERAGEPOSITIONCOL,
  L_AMPERAGEPOSITIONDSPL,
  L_PMETERSUMPOSITIONROW,
  L_PMETERSUMPOSITIONCOL,
  L_PMETERSUMPOSITIONDSPL,
  L_CALLSIGNPOSITIONROW,
  L_CALLSIGNPOSITIONCOL,
  L_CALLSIGNPOSITIONDSPL,
  // EEPROM_ITEM_LOCATION must be last for Items location!
  EEPROM_ITEM_LOCATION
};

uint8_t Settings[EEPROM_ITEM_LOCATION];

// For Settings Defaults
uint8_t EEPROM_DEFAULT[EEPROM_SETTINGS] = {
1,   // used for check              0

0,   // S_RSSIMIN                   1
255, // S_RSSIMAX                   2
60,  //S_RSSI_ALARM                 3
1,   // S_MWRSSI                    4
0,   // S_PWMRSSI                   5
105, // S_VOLTAGEMIN                6
3,   // S_BATCELLS                  7
100, // S_DIVIDERRATIO              8
1,   // S_MAINVOLTAGE_VBAT          9
100, // S_VIDDIVIDERRATIO           10
0,   // S_VIDVOLTAGE_VBAT           11 
90,  // S_TEMPERATUREMAX            12
1,   // S_BOARDTYPE                 13
1,   // S_DISPLAYGPS                14
1,   // S_COORDINATES               15
1,   // S_HEADING360                16
0,   // S_UNITSYSTEM                17
1,   // S_VIDEOSIGNALTYPE           18
0,   // S_RESETSTATISTICS           19
1,   // S_ENABLEADC                 20
0,   // S_USE_BOXNAMES              21
5,   // S_BLINKINGHZ,               22   // 10=1Hz, 9=1.1Hz, 8=1,25Hz, 7=1.4Hz, 6=1.6Hz, 5=2Hz, 4=2,5Hz, 3=3,3Hz, 2=5Hz, 1=10Hz
0,   //S_MWAMPERAGE                 23
0,   //S_AMPOFFSET,                 24
2,   //S_CLIMB_RATE_ALARM           25


0,   // S_CS0,                      26  // 10 callsign char locations
0,   // S_CS1,
0,   // S_CS2,
0,   // S_CS3,
0,   // S_CS4,
0,   // S_CS5,
0,   // S_CS6,
0,   // S_CS7,
0,   // S_CS8,
0,   // S_CS9,                      35
};


// PAL item position Defaults
uint8_t EEPROM_PAL_DEFAULT[EEPROM_ITEM_LOCATION-EEPROM_SETTINGS] = {
// ROW= Row position on screen (255= no action)
// COL= Column position on screen (255= no action)
// DSPL= Display item on screen
4,   // L_GPS_NUMSATPOSITIONROW LINE02+2
2,   // L_GPS_NUMSATPOSITIONCOL
1,   // L_GPS_NUMSATPOSITIONDSPL
3,   // L_GPS_DIRECTIONTOHOMEPOSROW LINE03+14
14,  // L_GPS_DIRECTIONTOHOMEPOSCOL
1,   // L_GPS_DIRECTIONTOHOMEPOSDSPL
2,   // L_GPS_DISTANCETOHOMEPOSROW LINE02+24
24,  // L_GPS_DISTANCETOHOMEPOSCOL
1,   // L_GPS_DISTANCETOHOMEPOSDSPL
3,   // L_SPEEDPOSITIONROW LINE03+24
24,  // L_SPEEDPOSITIONCOL
1,   // L_SPEEDPOSITIONDSPL
4,   // L_GPS_ANGLETOHOMEPOSROW LINE04+12
12,  // L_GPS_ANGLETOHOMEPOSCOL
1,   // L_GPS_ANGLETOHOMEPOSDSPL
4,   // L_MW_GPS_ALTPOSITIONROW LINE04+24
24,  // L_MW_GPS_ALTPOSITIONCOL
1,   // L_MW_GPS_ALTPOSITIONDSPL
2,   // L_SENSORPOSITIONROW LINE03+2
2,   // L_SENSORPOSITIONCOL
1,   // L_SENSORPOSITIONDSPL
2,   // L_MW_HEADINGPOSITIONROW LINE02+19
19,  // L_MW_HEADINGPOSITIONCOL
1,   // L_MW_HEADINGPOSITIONDSPL
2,   // L_MW_HEADINGGRAPHPOSROW LINE02+10
10,  // L_MW_HEADINGGRAPHPOSCOL
1,   // L_MW_HEADINGGRAPHPOSDSPL
11,  // L_TEMPERATUREPOSROW LINE11+2
2,   // L_TEMPERATUREPOSCOL
0,   // L_TEMPERATUREPOSDSPL

8,   // L_MW_ALTITUDEPOSITIONROW LINE08+2
2,   // L_MW_ALTITUDEPOSITIONCOL
1,   // L_MW_ALTITUDEPOSITIONDSPL
8,   // L_CLIMBRATEPOSITIONROW LINE08+24
24,  // L_CLIMBRATEPOSITIONCOL
1,   // L_CLIMBRATEPOSITIONDSPL
6,   // L_HORIZONPOSITIONROW LINE06+8
8,   // L_HORIZONPOSITIONCOL
1,   // L_HORIZONPOSITIONDSPL
255, // L_HORIZONSIDEREFROW,
255, // L_HORIZONSIDEREFCOL,
0,   // L_HORIZONSIDEREFDSPL,
255, // L_HORIZONCENTERREFROW,
255, // L_HORIZONCENTERREFCOL,
1,   // L_HORIZONCENTERREFDSPL,  
  
14,   // L_CURRENTTHROTTLEPOSITIONROW LINE14+22
22,   // L_CURRENTTHROTTLEPOSITIONCOL
1,    // L_CURRENTTHROTTLEPOSITIONDSPL
15,   // L_FLYTIMEPOSITIONROW LINE15+22
22,   // L_FLYTIMEPOSITIONCOL
1,    // L_FLYTIMEPOSITIONDSPL
15,   // L_ONTIMEPOSITIONROW LINE15+22
22,   // L_ONTIMEPOSITIONCOL
1,    // L_ONTIMEPOSITIONDSPL
14,   // L_MOTORARMEDPOSITIONROW LINE14+11
11,   // L_MOTORARMEDPOSITIONCOL
1,    // L_MOTORARMEDPOSITIONDSPL
12,   // L_MW_GPS_LATPOSITIONROW  LINE12+2
2,    // L_MW_GPS_LATPOSITIONCOL
1,    // L_MW_GPS_LATPOSITIONDSPL
12,   // L_MW_GPS_LONPOSITIONROW  LINE12+15
18,   // L_MW_GPS_LONPOSITIONCOL
1,    // L_MW_GPS_LONPOSITIONDSPL
14,   // L_RSSIPOSITIONROW LINE14+2
2,    // L_RSSIPOSITIONCOL
1,    // L_RSSIPOSITIONDSPL
15,   // L_VOLTAGEPOSITIONROW LINE15+3
3,    // L_VOLTAGEPOSITIONCOL
1,    // L_VOLTAGEPOSITIONDSPL
255,  // L_MAINBATLEVEVOLUTIONROW,
255,  // L_MAINBATLEVEVOLUTIONCOL,
1,    // L_MAINBATLEVEVOLUTIONDSPL,  
13,   // L_VIDVOLTAGEPOSITIONROW LINE13+3
3,    // L_VIDVOLTAGEPOSITIONCOL
0,    // L_VIDVOLTAGEPOSITIONDSPL
15,   // L_AMPERAGEPOSITIONROW LINE15+10
10,   // L_AMPERAGEPOSITIONCOL
1,    // L_AMPERAGEPOSITIONDSPL
15,   // L_PMETERSUMPOSITIONROW LINE15+16
16,   // L_PMETERSUMPOSITIONCOL
1,    // L_PMETERSUMPOSITIONDSPL
14,   // L_CALLSIGNPOSITIONROW LINE14+10
10,   // L_CALLSIGNPOSITIONCOL
0,    // L_CALLSIGNPOSITIONDSPL
};


// NTSC item position Defaults
uint8_t EEPROM_NTSC_DEFAULT[EEPROM_ITEM_LOCATION-EEPROM_SETTINGS] = {
// ROW= Row position on screen (255= no action)
// COL= Column position on screen (255= no action)
// DSPL= Display item on screen
4,   // L_GPS_NUMSATPOSITIONROW LINE02+2
2,   // L_GPS_NUMSATPOSITIONCOL
1,   // L_GPS_NUMSATPOSITIONDSPL
3,   // L_GPS_DIRECTIONTOHOMEPOSROW LINE03+14
14,  // L_GPS_DIRECTIONTOHOMEPOSCOL
1,   // L_GPS_DIRECTIONTOHOMEPOSDSPL
2,   // L_GPS_DISTANCETOHOMEPOSROW LINE02+24
24,  // L_GPS_DISTANCETOHOMEPOSCOL
1,   // L_GPS_DISTANCETOHOMEPOSDSPL
3,   // L_SPEEDPOSITIONROW LINE03+24
24,  // L_SPEEDPOSITIONCOL
1,   // L_SPEEDPOSITIONDSPL
4,   // L_GPS_ANGLETOHOMEPOSROW LINE04+12
12,  // L_GPS_ANGLETOHOMEPOSCOL
1,   // L_GPS_ANGLETOHOMEPOSDSPL
4,   // L_MW_GPS_ALTPOSITIONROW LINE04+24
24,  // L_MW_GPS_ALTPOSITIONCOL
1,   // L_MW_GPS_ALTPOSITIONDSPL
2,   // L_SENSORPOSITIONROW LINE03+2
2,   // L_SENSORPOSITIONCOL
1,   // L_SENSORPOSITIONDSPL
2,   // L_MW_HEADINGPOSITIONROW LINE02+19
19,  // L_MW_HEADINGPOSITIONCOL
1,   // L_MW_HEADINGPOSITIONDSPL
2,   // L_MW_HEADINGGRAPHPOSROW LINE02+10
10,  // L_MW_HEADINGGRAPHPOSCOL
1,   // L_MW_HEADINGGRAPHPOSDSPL
9,   // L_TEMPERATUREPOSROW LINE11+2
2,   // L_TEMPERATUREPOSCOL
0,   // L_TEMPERATUREPOSDSPL

7,   // L_MW_ALTITUDEPOSITIONROW LINE08+2
2,   // L_MW_ALTITUDEPOSITIONCOL
1,   // L_MW_ALTITUDEPOSITIONDSPL
7,   // L_CLIMBRATEPOSITIONROW LINE08+24
24,  // L_CLIMBRATEPOSITIONCOL
1,   // L_CLIMBRATEPOSITIONDSPL
5,   // L_HORIZONPOSITIONROW LINE06+8
8,   // L_HORIZONPOSITIONCOL
1,   // L_HORIZONPOSITIONDSPL
255, // L_HORIZONSIDEREFROW,
255, // L_HORIZONSIDEREFCOL,
0,   // L_HORIZONSIDEREFDSPL,
255, // L_HORIZONCENTERREFROW,
255, // L_HORIZONCENTERREFCOL,
1,   // L_HORIZONCENTERREFDSPL,  
  
12,   // L_CURRENTTHROTTLEPOSITIONROW LINE14+22
22,   // L_CURRENTTHROTTLEPOSITIONCOL
1,    // L_CURRENTTHROTTLEPOSITIONDSPL
13,   // L_FLYTIMEPOSITIONROW LINE15+22
22,   // L_FLYTIMEPOSITIONCOL
1,    // L_FLYTIMEPOSITIONDSPL
13,   // L_ONTIMEPOSITIONROW LINE15+22
22,   // L_ONTIMEPOSITIONCOL
1,    // L_ONTIMEPOSITIONDSPL
12,   // L_MOTORARMEDPOSITIONROW LINE14+11
11,   // L_MOTORARMEDPOSITIONCOL
1,    // L_MOTORARMEDPOSITIONDSPL
10,   // L_MW_GPS_LATPOSITIONROW  LINE12+2
2,    // L_MW_GPS_LATPOSITIONCOL
1,    // L_MW_GPS_LATPOSITIONDSPL
10,   // L_MW_GPS_LONPOSITIONROW  LINE12+15
18,   // L_MW_GPS_LONPOSITIONCOL
1,    // L_MW_GPS_LONPOSITIONDSPL
12,   // L_RSSIPOSITIONROW LINE14+2
2,    // L_RSSIPOSITIONCOL
1,    // L_RSSIPOSITIONDSPL
13,   // L_VOLTAGEPOSITIONROW LINE15+3
3,    // L_VOLTAGEPOSITIONCOL
1,    // L_VOLTAGEPOSITIONDSPL
255,  // L_MAINBATLEVEVOLUTIONROW,
255,  // L_MAINBATLEVEVOLUTIONCOL,
1,    // L_MAINBATLEVEVOLUTIONDSPL,  
11,   // L_VIDVOLTAGEPOSITIONROW LINE13+3
3,    // L_VIDVOLTAGEPOSITIONCOL
0,    // L_VIDVOLTAGEPOSITIONDSPL
13,   // L_AMPERAGEPOSITIONROW LINE15+10
10,   // L_AMPERAGEPOSITIONCOL
1,    // L_AMPERAGEPOSITIONDSPL
13,   // L_PMETERSUMPOSITIONROW LINE15+16
16,   // L_PMETERSUMPOSITIONCOL
1,    // L_PMETERSUMPOSITIONDSPL
12,   // L_CALLSIGNPOSITIONROW LINE14+10
10,   // L_CALLSIGNPOSITIONCOL
0,    // L_CALLSIGNPOSITIONDSPL
};


static uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];

static uint8_t rcRate8,rcExpo8;
static uint8_t rollPitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint8_t thrMid8;
static uint8_t thrExpo8;


static uint16_t  MwAccSmooth[3]={0,0,0};       // Those will hold Accelerator data
int32_t  MwAltitude=0;                         // This hold barometric value


int MwAngle[2]={0,0};           // Those will hold Accelerator Angle
static uint16_t MwRcData[8]={   // This hold receiver pulse signal
  1500,1500,1500,1500,1500,1500,1500,1500} ;

uint16_t  MwSensorPresent=0;
uint32_t  MwSensorActive=0;
uint8_t MwVersion=0;
uint8_t MwVBat=0;
int16_t MwVario=0;
uint8_t armed=0;
uint8_t previousarmedstatus=0;  // for statistics after disarming
int16_t GPS_distanceToHome=0;
uint8_t GPS_fix=0;
int32_t GPS_latitude;
int32_t GPS_longitude;
int16_t GPS_altitude;
uint16_t GPS_speed=0;
int16_t GPS_directionToHome=0;
uint8_t GPS_numSat=0;
int16_t I2CError=0;
uint16_t cycleTime=0;
uint16_t pMeterSum=0;
uint16_t MwRssi=0;
uint16_t MWAmperage=0;

//For Current Throttle
int LowT = 1100;
int HighT = 1900;

// For Time
uint16_t onTime=0;
uint16_t flyTime=0;

// For Heading
const char headGraph[] PROGMEM = {
  0x1a,0x1d,0x1c,0x1d,0x19,0x1d,0x1c,0x1d,0x1b,0x1d,0x1c,0x1d,0x18,0x1d,0x1c,0x1d,0x1a,0x1d,0x1c,0x1d,0x19,0x1d,0x1c,0x1d,0x1b};
static int16_t MwHeading=0;

// For Amperage
float amperage = 0;                // its the real value x10
float amperagesum = 0;

// Rssi
int rssi =0;
int rssiADC=0;
int rssiMin;
int rssiMax;
int rssi_Int=0;


// For Voltage
uint16_t voltage=0;                      // its the value x10
uint16_t vidvoltage=0;                   // its the value x10

// For temperature
int16_t temperature=0;                  // temperature in degrees Centigrade


// For Statistics
uint16_t speedMAX=0;
int8_t temperMAX=0;
int16_t altitudeMAX=0;
int16_t distanceMAX=0;
float trip=0;
uint16_t flyingTime=0; 


// ---------------------------------------------------------------------------------------
// Defines imported from Multiwii Serial Protocol MultiWii_shared svn r1337
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4
// End of imported defines from Multiwii Serial Protocol MultiWii_shared svn r1333
// ---------------------------------------------------------------------------------------

// Private MSP for use with the GUI
#define MSP_OSD                  220   //in message          starts epprom send to OSD GUI
// Subcommands
#define OSD_NULL                 0
#define OSD_READ_CMD             1
#define OSD_WRITE_CMD            2
#define OSD_GET_FONT             3
#define OSD_SERIAL_SPEED         4
#define OSD_RESET                5
// End private MSP for use with the GUI

const char disarmed_text[] PROGMEM = "DISARMED";
const char armed_text[] PROGMEM = " ARMED";

// For Intro
const char message0[] PROGMEM = "KV_OSD_TEAM_2.2";
const char message5[] PROGMEM = "MW VERSION:";
const char message6[] PROGMEM = "MENU:THRT MIDDLE";
const char message7[] PROGMEM = "YAW RIGHT";
const char message8[] PROGMEM = "PITCH FULL";
const char message9[] PROGMEM = "UNIQUE ID:";         // Call Sign on the beginning of the transmission   

// For Config menu common
const char configMsgON[] PROGMEM = "ON";
const char configMsgOFF[] PROGMEM = "OFF";
const char configMsgNoAct[] PROGMEM = "--";
const char configMsgEXIT[] PROGMEM = "EXIT";
const char configMsgSAVE[] PROGMEM = "SAVE-EXIT";
const char configMsgPGS[] PROGMEM = "<PAGE>";
const char configMsgNTSC[] PROGMEM = "NTSC";
const char configMsgPAL[] PROGMEM = "PAL";

// For Config pages
//-----------------------------------------------------------Page1
const char configMsg10[] PROGMEM = "1/9 PID CONFIG";
const char configMsg11[] PROGMEM = "ROLL";
const char configMsg12[] PROGMEM = "PITCH";
const char configMsg13[] PROGMEM = "YAW";
const char configMsg14[] PROGMEM = "ALT";
const char configMsg15[] PROGMEM = "GPS";
const char configMsg16[] PROGMEM = "LEVEL";
const char configMsg17[] PROGMEM = "MAG";
//-----------------------------------------------------------Page2
const char configMsg20[] PROGMEM = "2/9 RC TUNING";
const char configMsg21[] PROGMEM = "RC RATE";
const char configMsg22[] PROGMEM = "EXPONENTIAL";
const char configMsg23[] PROGMEM = "ROLL PITCH RATE";
const char configMsg24[] PROGMEM = "YAW RATE";
const char configMsg25[] PROGMEM = "THROTTLE PID ATT";
const char configMsg26[] PROGMEM = "MWCYCLE TIME";
const char configMsg27[] PROGMEM = "MWI2C ERRORS";
//-----------------------------------------------------------Page3
const char configMsg30[] PROGMEM = "3/9 SUPPLY & ALARM";
const char configMsg31[] PROGMEM = "VOLTAGE ALARM";
const char configMsg32[] PROGMEM = "SET TEMP ALARM";
const char configMsg33[] PROGMEM = "BLINKING FREQ";
//-----------------------------------------------------------Page4
const char configMsg40[] PROGMEM = "4/9 RSSI";
const char configMsg41[] PROGMEM = "ACTUAL RSSIADC";
const char configMsg42[] PROGMEM = "ACTUAL RSSI";
const char configMsg43[] PROGMEM = "SET RSSI MIN";
const char configMsg44[] PROGMEM = "SET RSSI MAX";
//-----------------------------------------------------------Page5
const char configMsg50[] PROGMEM = "5/9 CALIBRATION";
const char configMsg51[] PROGMEM = "ACC CALIBRATION";
const char configMsg52[] PROGMEM = "ACC ROLL";
const char configMsg53[] PROGMEM = "ACC PITCH";
const char configMsg54[] PROGMEM = "ACC Z";
const char configMsg55[] PROGMEM = "MAG CALIBRATION";
const char configMsg56[] PROGMEM = "HEADING";
const char configMsg57[] PROGMEM = "MW EEPROM WRITE";
//-----------------------------------------------------------Page6
const char configMsg60[] PROGMEM = "6/9 GPS";
const char configMsg61[] PROGMEM = "DISPLAY GPS DATA";
const char configMsg62[] PROGMEM = "GPS COORDINATES";
const char configMsg63[] PROGMEM = "CALLSIGN";
//-----------------------------------------------------------Page7
const char configMsg70[] PROGMEM = "7/9 ADV SETUP";
const char configMsg71[] PROGMEM = "RESET STATISTICS";
const char configMsg72[] PROGMEM = "HEADING 0-360";
const char configMsg73[] PROGMEM = "UNIT SYSTEM";
const char configMsg74[] PROGMEM = "METRIC";
const char configMsg75[] PROGMEM = "IMPERL";
const char configMsg76[] PROGMEM = "VIDEO SYSTEM";
//-----------------------------------------------------------Page8
const char configMsg80[] PROGMEM = "8/9 SCREEN ITEM POS";
const char configMsg81[] PROGMEM = "ITEM      DSP LINE COL";
const char configMsg82[] PROGMEM = "DEFAULT-EXIT";
//-----------------------------------------------------------Page9
const char configMsg90[] PROGMEM = "9/9 STATISTICS";
const char configMsg91[] PROGMEM = "TRIP";
const char configMsg92[] PROGMEM = "MAX DISTANCE";
const char configMsg93[] PROGMEM = "MAX ALTITUDE";
const char configMsg94[] PROGMEM = "MAX SPEED";
const char configMsg95[] PROGMEM = "FLYING TIME";
const char configMsg96[] PROGMEM = "AMPS DRAINED";
const char configMsg97[] PROGMEM = "MAX TEMP";


// Variables for items pos change on screen
//-----------------------------------------------------------
int8_t screenitemselect=0; // pointer for item text strings
int8_t screen_pos_item_pointer=EEPROM_SETTINGS+1;  // 0; // pointer for first item display/row/col positions
#define MAXSCREENITEMS 27

// Strings for item select on screen
//-----------------------------------------------------------
const char screen_item_00[] PROGMEM = "NUM SAT";
const char screen_item_01[] PROGMEM = "DIR TO HOME";
const char screen_item_02[] PROGMEM = "DIST TO HOME";
const char screen_item_03[] PROGMEM = "GPS SPEED";
const char screen_item_04[] PROGMEM = "ANGLE TO HOM";
const char screen_item_05[] PROGMEM = "GPS ALTITUDE";
const char screen_item_06[] PROGMEM = "SENSORS";
const char screen_item_07[] PROGMEM = "HEADING";
const char screen_item_08[] PROGMEM = "HEAD GRAPH";
const char screen_item_09[] PROGMEM = "TEMPERATURE";
const char screen_item_10[] PROGMEM = "BARO ALTIT";
const char screen_item_11[] PROGMEM = "CLIMB RATE";
const char screen_item_12[] PROGMEM = "HORIZON";
const char screen_item_13[] PROGMEM = "AH SIDE REF";
const char screen_item_14[] PROGMEM = "AH CENTR REF";
const char screen_item_15[] PROGMEM = "THROTTLE";
const char screen_item_16[] PROGMEM = "FLY TIME";
const char screen_item_17[] PROGMEM = "ON TIME";
const char screen_item_18[] PROGMEM = "ARMED INDIC";
const char screen_item_19[] PROGMEM = "GPS LATIT";
const char screen_item_20[] PROGMEM = "GPS LONGIT";
const char screen_item_21[] PROGMEM = "RSSI";
const char screen_item_22[] PROGMEM = "MAIN BATT";
const char screen_item_23[] PROGMEM = "MAIN BAT EVO";
const char screen_item_24[] PROGMEM = "VIDEO BATT";
const char screen_item_25[] PROGMEM = "AMPERAGE";
const char screen_item_26[] PROGMEM = "MA/H CONSUM";  
const char screen_item_27[] PROGMEM = "CALLSIGN";    

PROGMEM const char *item_table[] =
{   
screen_item_00,
screen_item_01,
screen_item_02,
screen_item_03,
screen_item_04,
screen_item_05,
screen_item_06,
screen_item_07,
screen_item_08,
screen_item_09,
screen_item_10,
screen_item_11,
screen_item_12,
screen_item_13,
screen_item_14,
screen_item_15,
screen_item_16,
screen_item_17,
screen_item_18,
screen_item_19,
screen_item_20,
screen_item_21,
screen_item_22,
screen_item_23,
screen_item_24,
screen_item_25,
screen_item_26,
screen_item_27,
};


// POSITION OF EACH CHARACTER OR LOGO IN THE MAX7456
const unsigned char speedUnitAdd[2] ={
  0x8c,0x8e} ;                               // [0][0] and [0][1] = Km/h   [1][0] and [1][1] = Mph
const unsigned char speedUnitAdd1[2] ={
  0x8d,0x8f} ;
const unsigned char temperatureUnitAdd[2] = {
  0x0e,0x0d};

const char MultiWiiLogoL1Add[17] PROGMEM = {
  0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xdb,0xdc,0xdd,0xde,0};
const char MultiWiiLogoL2Add[17] PROGMEM = {
  0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xeb,0xec,0xed,0xee,0};
const char MultiWiiLogoL3Add[17] PROGMEM = {
  0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0};

const unsigned char MwAltitudeAdd[2]={
  0xa7,0xa8};
const unsigned char MwClimbRateAdd[2]={
  0x9f,0x99};
const unsigned char GPS_distanceToHomeAdd[2]={
  0x8a,0x7a,};
  const unsigned char GPS_distanceToHomeAdd1[2]={
  0x8b,0x8b};
const unsigned char MwGPSAltPositionAdd[2]={
  0xa7,0xa8};
const unsigned char MwGPSAltPositionAdd1[2]={
  0xa3,0xa3};
const char KVTeamVersionPosition = 35;


#define REQ_MSP_IDENT     (1 <<  0)
#define REQ_MSP_STATUS    (1 <<  1)
#define REQ_MSP_RAW_IMU   (1 <<  2)
#define REQ_MSP_RC        (1 <<  3)
#define REQ_MSP_RAW_GPS   (1 <<  4)
#define REQ_MSP_COMP_GPS  (1 <<  5)
#define REQ_MSP_ATTITUDE  (1 <<  6)
#define REQ_MSP_ALTITUDE  (1 <<  7)
#define REQ_MSP_ANALOG    (1 <<  8)
#define REQ_MSP_RC_TUNING (1 <<  9)
#define REQ_MSP_PID       (1 << 10)
#define REQ_MSP_BOX       (1 << 11)
#define REQ_MSP_FONT      (1 << 12)
