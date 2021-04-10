/***************************************************
  Name
    Defines.h
    
  Description
    Global definitions

  Author
    Paulo Almeida
    falmeida.paulo@gmail.com
    https://www.youtube.com/channel/UC-kUryZXKOTKsRH2tHRS1NA

  Updates
 ****************************************************/
//#define TEST_MODE
#define DEBUG_SPEED     115200
#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port

//https://github.com/ThingPulse/esp8266-plane-spotter-color
//http://open.mapquestapi.com/staticmap/v4/getmap?key=r19I8UVBfwIkmE4EZR9S6yMR43eMiRDZ&type=map&scalebar=false&size=320,240&zoom=16&center=-3.7784381,-38.4887794
//http://maps.googleapis.com/maps/api/staticmap?key=AIzaSyBw0G8jCBry0IATNmysuyPd2fBblndS3jU&center=-3.7784381,-38.4887794&zoom=15&size=320x240&format=jpg-baseline&maptype=satellite

#define debug     Serial
#define telem     Serial1
#define sbus_port Serial2

//#include <ILI9341_t3n.h>
#include <SdFat.h>

//#define USE_SBUS2

#ifdef USE_SBUS
#include <SBUS.h>
SBUS sbus(sbus_port);
#endif

/***************************************************************************************
** TFT Defines
***************************************************************************************/
// Buffer TFT
//DMAMEM uint16_t frame_buffer1[ILI9341_TFTWIDTH][ILI9341_TFTHEIGHT];
//uint16_t * scr = (uint16_t*)&frame_buffer1[0][0];

DMAMEM uint16_t frame_buffer1[ILI9341_TFTWIDTH * ILI9341_TFTHEIGHT];
DMAMEM uint16_t frame_buffer2[ILI9341_TFTWIDTH * ILI9341_TFTHEIGHT];
uint16_t * screen = (uint16_t*)&frame_buffer1[0];
uint16_t * screen2 = (uint16_t*)&frame_buffer2[0];

#define BUFFPIXEL 40

//uint16_t  *frame_buffer;
//uint16_t  *scr;
//frame_buffer = (uint16_t *)malloc((ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT)+32);
//scr = (uint16_t*) (((uintptr_t)frame_buffer + 32) & ~ ((uintptr_t) (31)));

/*
  TFT #1 - PFD       ---->              Teensy 4.0
  ------------------------------------------------------
1 - VCC  ----------------------------- 3V3
2 - GND  ----------------------------- GND
3 - CS   ----------------------------- #10 CS   (PFD_CS)
4 - RST  ----------------------------- #32      (PFD_RST)
5 - DC   ----------------------------- #09      (PFD_DC)
6 - MOSI ----------------------------- #11 MOSI (TFT_MOSI)
7 - SCK  ----------------------------- #13 SCK  (TFT_SCK)
8 - LED  ----------------------------- 3V3 - RESISTOR
9 - MISO ----------------------------- #12 MISO (TFT_MISO)

  TFT #2 - MFD       ---->              Teensy 4.0
  ------------------------------------------------------
1 - VCC  ----------------------------- 3V3
2 - GND  ----------------------------- GND
3 - CS   ----------------------------- #16 A2   (MFD_CS)
4 - RST  ----------------------------- #15 A1   (MFD_RST)
5 - DC   ----------------------------- #14 A0   (MFD_DC)
6 - MOSI ----------------------------- #11 MOSI (TFT_MOSI)
7 - SCK  ----------------------------- #13 SCK  (TFT_SCK)
8 - LED  ----------------------------- 3V3 - RESISTOR
9 - MISO ----------------------------- #12 MISO (TFT_MISO)

*/

#define PFD_DC      9   // OUT1C
#define PFD_CS     10   // CS MQRS
#define PFD_RST    32   // OUT1B

#define MFD_DC     14   // A0 TX3 SPDIF_OUT
#define MFD_RST    15   // A1 RX3 SPDIF_IN
#define MFD_CS     16   // A2 RX3 SCL1

#define TFT_MOSI   MOSI
#define TFT_MISO   MISO
#define TFT_SCK    SCK

//ILI9341_t3n pfd = ILI9341_t3n(PFD_CS, PFD_DC, PFD_RST, TFT_MOSI, TFT_SCK, TFT_MISO);
ILI9341_t3n pfd = ILI9341_t3n(PFD_CS, PFD_DC, PFD_RST);
#define USE_MFD
#ifdef USE_MFD
//ILI9341_t3n mfd = ILI9341_t3n(MFD_CS, MFD_DC, MFD_RST, TFT_MOSI, TFT_SCK, TFT_MISO);
ILI9341_t3n mfd = ILI9341_t3n(MFD_CS, MFD_DC, MFD_RST);
#endif

#define DEGREE_SYMBOL       9   // Degree symbol font 5x7
#define TFT_WIDTH           (uint16_t)320 //ILI9341_TFTWIDTH
#define TFT_HEIGHT          (uint16_t)240 //ILI9341_TFTHEIGHT 
#define ROWS_TO_WRITE       (uint16_t)20  //lines read from flash
#define XMIN                (int32_t)0                // Left horizon
#define XMAX                (int32_t)(TFT_WIDTH)      // Right horizon
#define YMIN                (int32_t)20               // Top horizon 
#define YMAX                (int32_t)(TFT_HEIGHT -10) // Bot horizon
#define H_ROWS              (uint16_t)220
#define H_COLS              (uint16_t)TFT_WIDTH
#define X_PIVOT             (uint16_t)H_COLS / 2
#define Y_PIVOT             (uint16_t)88

const uint16_t SKY_COLOR        = pfd.color565(0, 76, 255);
const uint16_t GND_COLOR        = pfd.color565(100, 50, 20);
const uint16_t TFT_TRANSPARENT  = ILI9341_MAGENTA;
const uint16_t TFT_OPAQUE       = pfd.color565(0, 36, 0);
const uint16_t MFD_BACKCOLOR    = pfd.color565(13, 16, 13);
const uint16_t COLOR_DARKGREEN  = pfd.color565(0,120,0);
const uint16_t DARK_ORANGE      = pfd.color565(255, 150, 0);

const int16_t HI_LIMIT_OF_CLIMB = 400;
const int16_t LO_LIMIT_OF_CLIMB = -400;

#ifdef TEST_MODE
  int d = 0;
  uint32_t refreshTime = 0;       // time for next update
  #define LOOP_PERIOD 35 // Display updates every 35 ms
  #ifdef USE_ENCODER
    int oldPosition  = -999;
    int newPosition;
    int tickEnc = 0;
    int minPosition = 1;
    int maxPosition = 360;
    boolean reload = true;
  #endif
#endif

#define MAX_TANK_SIZE             450                 // Fuel tank max volume 5000ml
#define MIN_TANK_SIZE             MAX_TANK_SIZE * 0.2

#define NUMBER_OF_CHANNELS 18

/***************************************************************************************
** SD/Flash chip Defines
***************************************************************************************/
const int FLASH_CHIP_SELECT = 6; // digital pin for flash chip CS pin
static boolean flash_chip_OK = false;
static boolean sdCard_OK = false;
//SdFatSdioEX SD;
SdFat SD;

/***************************************************************************************
** SYSTEM Defines
***************************************************************************************/
const float                 mag_decl = -19.33333; // 29/04/2019 - 19º20'W http://www.magnetic-declination.com/#
float                       converts = 3.6;       // convert m/s to km/h
float                       converth = 1.0;       // disatance to m
boolean                     flight_director = false;
int16_t                     to_heading = 0;
int16_t                     to_course = 0;
boolean                     tick_half_second;
static uint32_t             time_boot_ms = 0;
static uint8_t              apm_mav_type = 0;
static uint8_t              flight_mode = 0;      // Navigation mode from RC AC2 = CH5, APM = CH8
static uint16_t             rpm_value = 0;
static boolean              mavbeat = 0;
static uint8_t              apm_mav_system;
static uint8_t              apm_mav_component;
static uint8_t              osd_nav_mode = 0;
static float                alt_prev = 0;             // previous altitude
static uint8_t              alt_cnt = 0;              // counter for stable osd_alt
boolean                     failSafe;
boolean                     lostFrame;

static int maxval = 0;
static int minval = 9999;

/***************************************************************************************
** BITMAP Header Struct
** Save point values
***************************************************************************************/
typedef struct
{
  uint16_t signature;  // 02
  uint32_t fileSize;   // 04
  uint32_t reserved;   // 04
  uint32_t imageoffset;// 04
  uint32_t headerSize; // 04
  int32_t bmpWidth;    // 04
  int32_t bmpHeight;   // 04
  uint16_t planes;     // 02
  uint16_t bmpDepth;   // 02
  uint32_t compression;// 04
  uint32_t ImageSize;  // 04
  uint32_t XpixelsPerM;// 04
  uint32_t YpixelsPerM;// 04
  uint32_t Colors;     // 04
  uint32_t Important;  // 04
}__attribute__((__packed__))bitmap_header_t;


/***************************************************************************************
** POINT Struct
** Save point values
***************************************************************************************/
typedef struct {
  int                   x = 0;                  // [rad] Roll angle (-pi..+pi)
  int                   y = 0;                  // [rad] Pitch angle (-pi..+pi)
} __attribute__((__packed__))POINT;

/***************************************************************************************
** Channel Struct
** Save RX channels values
***************************************************************************************/
typedef struct {
  uint16_t value = 0;
}__attribute__((__packed__))CHANNEL;

/*
typedef struct{
 float                  roll_rate = 0;              //< Desired roll rate in radians per second
 float                  pitch_rate =0;              //< Desired pitch rate in radians per second
 float                  yaw_rate = 0;               //< Desired yaw rate in radians per second
 float                  thrust = 0;                 //< Collective thrust, normalized to 0 .. 1
} __attribute__((__packed__))TO_TARGET;
*/
/***************************************************************************************
** AHRS Struct
** Save AHRS values
***************************************************************************************/
typedef struct {
  float                   roll = 0;                   // [rad] Roll angle (-pi..+pi)
  float                   pitch = 0;                  // [rad] Pitch angle (-pi..+pi)
  float                   yaw = 0;                    // [rad] Yaw angle (-pi..+pi)
  float                   rollspeed;                  //  Roll angular speed (rad/s)
  float                   pitchspeed;                 //  Pitch angular speed (rad/s)
  float                   yawspeed;                   //  Yaw angular speed (rad/s)
  //TO_TARGET               target;
  float                   sinRoll = sin(roll * DEG_TO_RAD);                // Seno angle Roll
  float                   cosRoll = cos(roll * DEG_TO_RAD);                // Cosine angle Roll
} __attribute__((__packed__))AHRS;

/***************************************************************************************
** WAYPOINT Struct
** Save geographic coordenate
***************************************************************************************/
typedef struct {
  float                   lat ;
  float                   lon;
  float                   alt;
} __attribute__((__packed__))WAYPOINT;

/***************************************************************************************
** WIND Struct
** Save WIND values
***************************************************************************************/
typedef struct {
  float                   direction ;
  float                   speed;
  float                   speed_z;
} __attribute__((__packed__))WIND;

/***************************************************************************************
** HOME Struct
** Save HOME values
***************************************************************************************/
typedef struct {
  boolean                 got_home = false;           // tels if got home position or not
  WAYPOINT                position; 
  uint32_t                distance = 0;               // distance from home
  int16_t                 direction;                  // {Arrow direction pointing to home (1-16 to CW loop)}
  float                   glide = 0;
  uint32_t                ete = 0;                    // etimated time enroute in seconds
} __attribute__((__packed__))HOME;

/***************************************************************************************
** NAV Struct
** Save NAV data values
***************************************************************************************/
typedef struct {
  AHRS                    ahrs;                       // [deg] Current desired roll/pitch/yaw
  WAYPOINT                position;
  float                   alt_error = 0;              // Current altitude error in meters
  float                   aspd_error = 0;             // Current airspeed error in meters/second
  float                   xtrack_error = 0;           // Current crosstrack error on x-y plane in meters
  int16_t                 nav_bearing = 0;            // [deg] Current desired heading
  int16_t                 target_bearing = 0;         // [deg] Bearing to current waypoint/target
  uint32_t                wp_dist = 0;                // [m] Distance to active waypoint
  uint8_t                 wp_number = 0;              // Current waypoint number
  uint32_t                ete = 0;                    // etimated time enroute in seconds
} __attribute__((__packed__))NAV;

#define                   GPS_FIX_TYPE_NO_GPS     0 //No GPS connected
#define                   GPS_FIX_TYPE_NO_FIX     1 //No position information, GPS is connected
#define                   GPS_FIX_TYPE_2D_FIX     2 //2D position
#define                   GPS_FIX_TYPE_3D_FIX     3 //3D position
#define                   GPS_FIX_TYPE_DGPS       4 //DGPS/SBAS aided 3D position
#define                   GPS_FIX_TYPE_RTK_FLOAT  5 //RTK float, 3D position
#define                   GPS_FIX_TYPE_RTK_FIXED  6 //RTK Fixed, 3D position
#define                   GPS_FIX_TYPE_STATIC     7 //Static fixed, typically used for base stations
#define                   GPS_FIX_TYPE_PPP        8 //PPP, 3D position.

/***************************************************************************************
** APM Struct
** Save APM values
***************************************************************************************/
typedef struct {
  bool                    connected = false;
  uint16_t                hb_count = 0;               // heart beat count
  float                   vbatA = 0;                  // Battery A voltage in milivolt
  float                   ibatA = 0;                  // Battery A current
  int8_t                  remainingA = 0;             // 0 to 100 <=> 0 to 1000
  bool                    motor_armed = 0;
  /* MAVLINK_MSG_ID_GPS_RAW_INT */
  WAYPOINT                position;
  uint8_t                 fix_type = GPS_FIX_TYPE_NO_GPS;   // GPS lock 0-1=no fix, 2=2D, 3=3D
  uint8_t                 satellites_visible = 0;     // number of satelites
  uint64_t                time_usec = 0;              // [us] Timestamp (UNIX Epoch time or time since system boot).
  /* MAVLINK_MSG_ID_VFR_HUD 74 */
  float                   airspeed = 0;               // [m/s] Current indicated airspeed (IAS).
  float                   groundspeed = 0;            // [m/s] Current ground speed.
  float                   alt = 0;                    // [m] Current altitude (MSL).
  float                   relative_alt = 0;           // [m] Current altitide (AGL)
  float                   climb = 0;                  // [m/s] Current climb rate.
  int16_t                 heading = 0;                // [deg] Current heading in compass units (0-360, 0=north).
  uint16_t                throttle = 0;               // [%] Current throttle setting (0 to 100).
  float                   temperature = 0;
  AHRS                    ahrs;
} __attribute__((__packed__))APM;

APM             apm;
NAV             wp;
HOME            home;
WIND            wind;
CHANNEL         raw_channel[NUMBER_OF_CHANNELS];
CHANNEL         sbus_channel[NUMBER_OF_CHANNELS];
