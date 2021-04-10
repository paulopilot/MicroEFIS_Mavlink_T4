/***************************************************
  Name
    MicroPFD_MavLink_T4
    
  Description
    Garmin 1000 Flight Deck Emulator to use RC model Cessna 182 Gold Edition TopFlite.
    The system use Mavlink telemetry protocol.

  Features
    - Complete AHRS simulation
    - Show temperature from MAVlink
    - Show pitch,row,yaw from MAVlink
    - Show engine RPM from sensor 
    - Show fuel flow by sensor   (thaks https://www.rc-thoughts.com/jeti-fuel-sensor/)
      https://www.conrad.com/p/bt-bio-tech-fch-m-pp-lc-low-flow-flowmeter-corrosive-chemicals-npn-155374
    - 
    
  Hardware
    1x Teensy 4.0
    2x ILI9341
    1x Custom PCB

  Author
    Paulo Almeida
    falmeida.paulo@gmail.com
    https://www.youtube.com/channel/UC-kUryZXKOTKsRH2tHRS1NA

  Licence MIT
    Copyright (c) 2021 Paulo Almeida

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

  Updates
    10/04/2021 - Licence MIT
    08/04/2021 - Bug fix Flight Directro indicator
    
  
 ****************************************************/
// <SoftEgg>

#include <SPI.h>
#include <SerialFlash.h>
#ifdef USE_ENCODER
  #include <Encoder.h>
#endif

#define USE_THREAD
#ifdef USE_THREAD
  #include <Thread.h>
#endif
//#include <TeensyThreads.h>

#include <FreqMeasure.h>

#include <ILI9341_t3n.h>

#include "SmallText.h"
#include "Gauge.h"
#include "Sprite.h"

SmallText *rpmLabel = new SmallText();
SmallText *hdgLabel = new SmallText();
SmallText *hdgValue = new SmallText();
SmallText *crsLabel = new SmallText();
SmallText *crsValue = new SmallText();

SmallText *fuelFlowText     = new SmallText();
SmallText *fuelTankText     = new SmallText();
SmallText *throttleText     = new SmallText();
SmallText *engineTempText   = new SmallText();
SmallText *batteryText      = new SmallText();

Gauge     *rpmGauge = new Gauge();

Sprite    *fuelIndicator    = new Sprite();
Sprite    *fuelFlowIndicator = new Sprite();
Sprite    *throttleIndicator = new Sprite();
Sprite    *engineTempIndicator = new Sprite();
//Sprite    *batteryIndicator = new Sprite();

#include "Defines.h"
#include "Functions.h"
#include "MavlinkFunctions.h"

#ifdef USE_SBUF2
  #include "SBUS2.h"
  SBUS2 sbus(sbus_port);
#endif

#ifdef USE_THREAD
Thread::Task tasks[] = {
  THREADTASK(drawEFIS, 54 ,true),
  THREADTASK(setHomeVars,250,true),
  THREADTASK(updateMFD, 500,true),
  //THREADTASK(logData,1000,true),
  //THREADTASK(updateTime,1000,true),
  //THREADTASK(debug,100,true),
  //THREADTASK(handle_Messages, 10, true),
  THREADEND
};
#endif

/*
 * Only teste 
 */
#ifdef USE_ENCODER
Encoder myEnc(22, 23);
const int pwPin =  21;
#endif

/***************************************************************************************
** Function name:           setup()
** Description:             Initial configuration
***************************************************************************************/
void setup() {
  // put your setup code here, to run once:
  debug.begin(DEBUG_SPEED);
  telem.begin(TELEMETRY_SPEED);
#ifdef USE_ENCODER  
  pinMode(pwPin, OUTPUT);
  digitalWrite(pwPin, HIGH);
#endif  

  pfd.begin();
  pfd.setRotation(1);
  pfd.fillScreen(ILI9341_BLACK);
  pfd.setTextColor(ILI9341_WHITE);
  pfd.setTextSize(1);
  pfd.println("INITIALIZING SYSTEM");

  #ifdef USE_MFD  
  mfd.begin();
  mfd.setRotation(1);
  mfd.fillScreen(ILI9341_BLACK);
  mfd.setTextColor(ILI9341_WHITE);
  mfd.setTextSize(1);
  mfd.println("INITIALIZING SYSTEM");
  #endif

  setDarkColor(pfd.color565(255, 200, 100), 80);

  debug.printf("Serial Flash....");
  flash_chip_OK = SerialFlash.begin(FLASH_CHIP_SELECT);
  if(flash_chip_OK){
    debug.printf("OK\n");  
  }else{
    debug.printf("FAIL\n");  
  }

  debug.printf("SD Card.........");
  if (!SD.begin()) {
    sdCard_OK = false;
    debug.printf("FAIL\n"); 
    SD.initErrorPrint();
  }else{
    debug.printf("OK\n"); 
    sdCard_OK = false;
  }

  delay(2000);
  mfd.fillScreenVGradient(ILI9341_NAVY, ILI9341_BLACK);
  pfd.setFrameBuffer(screen);
  pfd.useFrameBuffer(true);
  mfd.setFrameBuffer(screen2);
  mfd.useFrameBuffer(true);
  drawImage("I0001.RAW" , &mfd);

  pfd.updateScreen();
  mfd.updateScreen();
  //pfd.updateScreenAsync();
  //mfd.updateScreenAsync();
  //pfd.waitUpdateAsyncComplete();
  //mfd.waitUpdateAsyncComplete();
  //delay(3000);

  rpmLabel->init(&mfd, 16, 50, ILI9341_WHITE, ILI9341_BLACK);
  hdgLabel->init(&pfd, 106,133, ILI9341_WHITE, ILI9341_BLACK);
  hdgValue->init(&pfd, 120,133, ILI9341_CYAN, ILI9341_BLACK);
  crsLabel->init(&pfd, 188,133, ILI9341_WHITE, ILI9341_BLACK);
  crsValue->init(&pfd, 202,133, ILI9341_GREEN, ILI9341_BLACK);
  //rpmGauge->init(&mfd, 23, 38, 15, 0, 10000, 0, ILI9341_WHITE, ILI9341_BLACK);

  rpmGauge->init(&mfd, 23, 38, 15, 100, 1950, 0, ILI9341_WHITE, ILI9341_BLACK);
  
  drawScreen(&pfd);   // Change frame buffer before show into display
  drawScreen(&mfd, 1);   // Change frame buffer before show into display
  drawMap(&mfd, GND_COLOR, 2);

  throttleIndicator->initSprite(&mfd, 3, 63, 0 , 100);
  throttleIndicator->showSprite();
  throttleText->init(&mfd, 17, 72);

  engineTempIndicator->initSprite(&mfd, 3, 121, 60 , 300);
  engineTempIndicator->showSprite();
  engineTempText->init(&mfd, 15, 130);

  fuelFlowIndicator->initSprite(&mfd, 3, 140, 0 , 100);
  fuelFlowIndicator->showSprite();
  fuelFlowText->init(&mfd, 17, 149);

  fuelIndicator->initSprite(&mfd, 3, 159, 0 , MAX_TANK_SIZE, true);
  fuelIndicator->showSprite();
  fuelTankText->init(&mfd, 17, 168);

  batteryText->init(&mfd, 15, 192);

#ifdef USE_SBUS
  sbus.begin();
#endif

#ifdef USE_THREAD  
  Thread::load_tasks(tasks);
#endif

  analogWriteFrequency(18, (7000.0f / 60.0f));
  analogWrite(18, 128); // short pulses at 150,0 Hz = 9000RPM
  FreqMeasure.begin();

  //threads.addThread(drawEFIS);

}  // End Setup()


double sum=0;
int count=0;
/***************************************************************************************
** Function name:           loop()
** Description:             Main loop
***************************************************************************************/
void loop() {
  
  char buf[10];
  uint32_t now = micros();
  
#ifdef USE_SBUS
  if(sbus.read(&sbus_channel[0].value, &failSafe, &lostFrame)){
    // write the SBUS packet to an SBUS compatible servo
    //sbus.write(&sbus_channel[0].value);
    if (!lostFrame){
      //debug.printf("Channel 4=%d\n", sbus_channel[4-1].value);
      rpmGauge->value((int32_t)sbus_channel[4-1].value);
      //sprintf (buf, "%04d", (int)sbus_channel[4-1].value);
      //rpmLabel->printText(buf);
    }
  }
  //rpmGauge->value((int32_t)sbus_channel[4-1].value);
  //sprintf (buf, "%04d", (int)sbus_channel[4-1].value);
  //rpmLabel->printText(buf);
#endif

#ifdef USE_THREAD
  Thread::run(now);
#endif
  
  if (FreqMeasure.available()) {
    // average several reading together
    sum = sum + FreqMeasure.read();
    count = count + 1;
    if (count > 10) {
      float frequency = FreqMeasure.countToFrequency(sum / count);
      rpm_value = (uint32_t)(frequency * 60.0f);
      //debug.printf("Frequency=%f\tRPM=%d\n", frequency, rpm_value);
      sum = 0;
      count = 0;
    }
  }
 

#ifdef TEST_MODE
  apm.fix_type = GPS_FIX_TYPE_3D_FIX;
  apm_mav_type = 1;
  flight_mode = 10; // AUTO
  // SBFZ
  home.got_home = true;
  home.position.alt = 25;
  home.position.lat = -3.77583;
  home.position.lon = -38.5322;
  // SBRF
  apm.position.lat =  -8.12638;
  apm.position.lon =  -34.9227;

#ifdef USE_ENCODER
  newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    if (newPosition > maxPosition) {
      if (reload) {
        newPosition = minPosition;
      } else {
        newPosition = maxPosition;
      }
      myEnc.write(newPosition);
    }
    if (newPosition < minPosition) {
      if (reload) {
        newPosition = maxPosition;
      } else{
        newPosition = minPosition;
      }
      myEnc.write(newPosition);
    }
    oldPosition = newPosition;
    //debug.printf("Normalize: %d\n", normalizeAngle(newPosition, 360));
  }
#endif
  
  //wp.target_bearing = Course to waypoint
  //apm.heading       = Veihcle direction
  //apm.ahrs.roll     = Veihcle roll
  if (refreshTime <= millis()) {
    refreshTime = millis() + LOOP_PERIOD;
    d += 4; if (d >= 360) d = 0;
    wp.target_bearing = 127;
    wp.wp_dist = 36000;     // Distance to WP 72000m
    wp.wp_number = 1;
    apm.heading = 90 + 90 * sin((d + 0) * 0.0174532925);
    apm.climb = 4 * sin((d + 30) * 0.0174532925);
    apm.ahrs.pitch = 7 * sin((d + 60) * 0.0174532925);
    apm.ahrs.roll = 15 * sin((d + 120) * 0.0174532925);
    apm.airspeed = 60 + 60 * sin((d + 180) * 0.0174532925);
    apm.alt = 100 + 100 * sin((d + 180) * 0.0174532925);
    //wp.nav_bearing = 30 * sin((d + 0) * 0.0174532925);
    //apm.heading = newPosition;
    //apm.climb = (float)(newPosition / 100.0f);
    //apm.ahrs.pitch = newPosition;
    //wp.nav_bearing = newPosition; //
    //apm.groundspeed = (float)newPosition;
  }
#endif
} // End loop()

#define SERIAL_EVENT
#ifdef SERIAL_EVENT
void serialEvent1() {
  handle_Messages();
} //serialEvent1
#endif
