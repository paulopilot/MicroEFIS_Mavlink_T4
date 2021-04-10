/***************************************************
  Name
    Functions.h
    
  Description
    General Functions

  Author
    Paulo Almeida
    falmeida.paulo@gmail.com
    https://www.youtube.com/channel/UC-kUryZXKOTKsRH2tHRS1NA

  Updates
 ****************************************************/
#include "icons.h"
#include "Images.h"

/***************************************************************************************
** Function name:           normalizeAngle
** Description:             Add angle to value and normalize value 0..360
***************************************************************************************/
int16_t normalizeAngle(int16_t angle, int16_t value) {
  int16_t normal = angle + value;
  while(normal > 360) normal -= 360;
  return normal;
}

/***************************************************************************************
** Function name:           fillRect
** Description:             Draw and fill rectangle on buffer image
***************************************************************************************/
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  //x+=_originx;
  //y+=_originy;

  // Rectangular clipping (drawChar w/big text requires this)
  if((x >= TFT_WIDTH) || (y >= TFT_HEIGHT)) return;
  if (((x+w) <= 0) || ((y+h) <= 0)) return;
  if(x < 0) {  w -= (0-x); x = 0;  }
  if(y < 0) {  h -= (0 - y); y = 0;  }
  if((x + w - 1) >= TFT_WIDTH)  w = TFT_WIDTH  - x;
  if((y + h - 1) >= TFT_HEIGHT) h = TFT_HEIGHT - y;

  if ((x&1) || (w&1)) {
    uint16_t * pfbPixel_row = &screen[ y*TFT_WIDTH + x];
    for (;h>0; h--) {
      uint16_t * pfbPixel = pfbPixel_row;
      for (int i = 0 ;i < w; i++) {
        *pfbPixel++ = color;
      }
      pfbPixel_row += TFT_WIDTH;
    }
  } else {
    // Horizontal is even number so try 32 bit writes instead
    uint32_t color32 = (color << 16) | color;
    uint32_t * pfbPixel_row = (uint32_t *)((uint16_t*)&screen[ y*TFT_WIDTH + x]);
    w = w/2;  // only iterate half the times
    for (;h>0; h--) {
      uint32_t * pfbPixel = pfbPixel_row;
      for (int i = 0 ;i < w; i++) {
        *pfbPixel++ = color32;
      }
      pfbPixel_row += (TFT_WIDTH/2);
    }
  }
}

/***************************************************************************************
** Function name:           drawImageSD
** Description:             Draw raw image from SD
***************************************************************************************/
int drawImage(char *filename, ILI9341_t3n *tft = NULL, int16_t x = 0, int16_t y = 0, int16_t rawWidth = 320, int16_t rawHeight = 240){
  
  if (flash_chip_OK) {
    SerialFlashFile  rawFile;
    // Open file
    rawFile = SerialFlash.open(filename);

    uint16_t * flashBuffer = malloc ((rawWidth*ROWS_TO_WRITE) * sizeof(uint16_t));

    // Read flash, convert to colour and display
    for (uint32_t row = 0; row < (TFT_HEIGHT/ROWS_TO_WRITE); row++) {
      rawFile.read(flashBuffer, ((rawWidth*2)*ROWS_TO_WRITE));
      tft->writeRect(x, y+(row*ROWS_TO_WRITE), rawWidth, ROWS_TO_WRITE, flashBuffer);
    }
    // Close the file
    rawFile.close();
    // Clear buffer
    free(flashBuffer);
    return true;
  }else{
    if (sdCard_OK) {
      File file;
      if (!file.open(filename, O_READ)) {
        int rd = 0;
        int bytesread;
        const uint32_t framesize = 153600;
        uint16_t * p = screen;
        rd = framesize;

        do {
          bytesread = file.read(p, rd);
          if (bytesread <= 0) {
            break;
          }
          p += bytesread / sizeof(uint16_t);
          rd -= bytesread;
          if (tft != NULL) {
            tft->writeRect(x, y, rawWidth, rawHeight, screen);
          }
        }while (bytesread > 0 && rd > 0);
        file.close();
      }else{
        SD.errorHalt("open failed");
        return 0;
      }
      return true;
    } else {
      return false;
    }
  }
}

/***************************************************************************************
** Function name:           drawBitmap
** Description:             Draw bitmap image from SD
***************************************************************************************/
void drawBitmap(char *filename, ILI9341_t3n *tft = NULL, int16_t x = 0, int16_t y = 0) {

  uint32_t bmpWidth, bmpHeight;   // W+H in pixels
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  flip    = true;        // BMP is stored bottom-to-top
  uint32_t w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();
  uint16_t awColors[320];  // hold colors for one row at a time...

  if((x >= tft->width()) || (y >= tft->height())) return;

  // Open requested file on SD card
  if (sdCard_OK) {
    File     bmpFile;
    if (!(bmpFile = SD.open(filename))) {
      debug.printf("File not found\n");
      return;
    }
    // Read bitmap header
    bitmap_header_t bitmap_header;
    byte  *buff = (unsigned char *) &bitmap_header.signature;
    bmpFile.read(buff, sizeof(bitmap_header_t));

    if(bitmap_header.signature == 0x4D42) { // BMP signature
      bmpWidth  = bitmap_header.bmpWidth;
      bmpHeight = bitmap_header.bmpHeight;
      // # planes      -- must be '1'
      // # compression -- must be '0'
      // # bmpDepth    -- must be '24'
      if((bitmap_header.planes == 1) && (bitmap_header.bmpDepth == 24) && (bitmap_header.compression == 0)) {
        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bitmap_header.bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bitmap_header.bmpWidth;
        h = bmpHeight;

        if((x+w-1) >= tft->width())  w = tft->width()  - x;
        if((y+h-1) >= tft->height()) h = tft->height() - y;

        // Set TFT address window to clipped image bounds
        tft->setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) { // For each scanline...
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bitmap_header.imageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bitmap_header.imageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }
          for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            awColors[col] = tft->color565(r,g,b);
          } // end pixel
          tft->writeRect(x, row + y, w, 1, awColors);
        }
      }else{
        debug.printf("BMP format not recognized.\n");
      }
    }else{
      debug.printf("File is not a bitmap.\n");
    }
    bmpFile.close();
  }
}

/***************************************************************************************
** Function name:           setDarkColor
** Description:             Set Dark Color
***************************************************************************************/
uint16_t setDarkColor(uint16_t color, int percent){
  uint8_t r, g ,b;
  float perc = (float)percent / 100.0f;
  pfd.color565toRGB(color, r, g, b);
  r = (uint8_t)((float)r * perc);
  g = (uint8_t)((float)g * perc);
  b = (uint8_t)((float)b * perc);
  return pfd.color565(r, g, b);
}

/***************************************************************************************
** Function name:           drawIcon
** Description:             Draw icon
***************************************************************************************/
void drawIcon(const unsigned short* image, uint16_t x, uint16_t y, uint16_t index, uint16_t color = ILI9341_WHITE, uint16_t bgcolor = TFT_TRANSPARENT){
  uint16_t idx = index * icon_S;
  uint16_t ix = 0;
  for (uint32_t i = idx; i < (idx + icon_S); i++) {
    uint32_t x0 = (ix % icon_W) + x;
    uint32_t y0 = (ix / icon_W) + y;
    uint32_t poss = (y0 * H_COLS) + x0;
    uint16_t pix_color = pgm_read_word(&image[i]);
    if (pix_color == TFT_TRANSPARENT){ 
      if (bgcolor == TFT_TRANSPARENT){
        pix_color = screen[poss];
      }else{
        pix_color = bgcolor;
      }
    }else{
      if (color != ILI9341_WHITE) pix_color = color;
    }
    screen[poss] = pix_color;
    ix++;
  }
} //drawIcon

/***************************************************************************************
** Function name:           drawMap
** Description:             Draw map
***************************************************************************************/
void drawMap(ILI9341_t3n *tft, uint16_t back_color = GND_COLOR, uint16_t scale = 1){
  if (scale > 4) scale = 4;
  tft->writeRect(47, 19, mapa_W, mapa_H, Mapa);
  //tft->fillRect(47, 19, 273, 211, back_color);
  //tft->drawFastHLine(47, 124, 273, setDarkColor(back_color, 120));
  //tft->drawFastVLine(183, 19, 211, setDarkColor(back_color, 120));
  //tft->drawCircle(183,124,104, setDarkColor(back_color, 120));
  //tft->drawCircle(183,124, 52, setDarkColor(back_color, 120));

  //tft->fillRect(181, 19, 5, 5, back_color);
  //tft->fillRect(181, 70, 5, 5, back_color);
  //tft->fillRect(181, 174, 5, 5, back_color);
  //tft->fillRect(181, 225, 5, 5, back_color);
  //tft->writeRect(182,  19, small_num_W, small_num_H, small_num + (2 * 15 * scale));
  //tft->writeRect(182, 225, small_num_W, small_num_H, small_num + (2 * 15 * scale));

  //tft->writeRect(182,  70, small_num_W, small_num_H, small_num + (1 * 15 * scale));
  //tft->writeRect(182, 174, small_num_W, small_num_H, small_num + (1 * 15 * scale));
} //drawMap


/***************************************************************************************
** Function name:           drawArc
** Description:             Draw arc center x and y ray r start angle to end angle and color
***************************************************************************************/
void drawArc(int x, int y, int r, int startAngle, int endAngle, uint16_t color) {
  /* original code from Henning Karlsen (http://www.rinkydinkelectronics.com)
    This library is free software; you can redistribute it and/or
    modify it under the terms of the CC BY-NC-SA 3.0 license.
    Please see the included documents for further information.
  */
  uint16_t cx, cy;
  uint32_t poss;
  startAngle -= 90;
  endAngle   -= 90;

  if (startAngle != endAngle) {
    for (int d = startAngle + 1; d < endAngle + 1; d++) {
      cx = x + cos(d * DEG_TO_RAD) * r;
      cy = y + sin(d * DEG_TO_RAD) * r;
      poss = cy * H_COLS + cx;
      screen[poss] = color;
      //pfd.drawPixel(cx, cy, color);
    }
  } else {
    cx = x + cos(startAngle * DEG_TO_RAD) * r;
    cy = y + sin(startAngle * DEG_TO_RAD) * r;
    poss = cy * H_COLS + cx;
    screen[poss] = color;
  }
}
/***************************************************************************************
** Function name:           drawScreen
** Description:             Draw main screen
***************************************************************************************/
void drawScreen(ILI9341_t3n *tft, uint8_t device = 0 ){
  // Top screen
  tft->setTextColor(ILI9341_WHITE);
  tft->fillScreen(ILI9341_BLACK);
  tft->drawFastHLine(0, 18, 320, ILI9341_DARKGREY);
  tft->drawFastVLine(66, 0, 17, ILI9341_DARKGREY);
  tft->drawFastVLine(252, 0, 17, ILI9341_DARKGREY);

  tft->drawFastHLine(0, 230, 320, ILI9341_DARKGREY);
  if (device == 1) {
    // Arc RPM gauge
    tft->drawArc(23, 42, 21, -120, 48, ILI9341_DARKGREY);
    tft->drawArc(23, 42, 20, -120, 48, ILI9341_DARKGREY);
    tft->drawArc(23, 42, 19, -120, 48, ILI9341_DARKGREY);

    tft->drawArc(23, 42, 21, 49, 95, ILI9341_GREEN);
    tft->drawArc(23, 42, 20, 49, 95, ILI9341_GREEN);
    tft->drawArc(23, 42, 19, 49, 95, ILI9341_GREEN);

    tft->drawArc(23, 42, 21, 96, 120, ILI9341_RED);
    tft->drawArc(23, 42, 20, 96, 120, ILI9341_RED);
    tft->drawArc(23, 42, 19, 96, 120, ILI9341_RED);
    rpmLabel->printText("RPM");

    
    // Throttle 1 liner gauge
    tft->drawFastVLine(2, 66, 4, ILI9341_DARKGREY); // |_________________|
    tft->drawFastHLine(3, 70, 41, ILI9341_DARKGREY);
    tft->drawFastVLine(44, 66, 4, ILI9341_DARKGREY);
    tft->drawFastHLine(4, 68, 39, COLOR_DARKGREEN);
    tft->drawFastHLine(4, 69, 39, ILI9341_GREEN);

    // Throttle 2 liner gauge
    tft->drawFastVLine(2, 85, 4, ILI9341_DARKGREY); // |_________________|
    tft->drawFastHLine(3, 89, 41, ILI9341_DARKGREY);
    tft->drawFastVLine(44, 85, 4, ILI9341_DARKGREY);
    tft->drawFastHLine(4, 87, 39, COLOR_DARKGREEN);
    tft->drawFastHLine(4, 88, 39, ILI9341_GREEN);

    // Temp liner gauge
    tft->drawFastVLine(2, 104, 4, ILI9341_DARKGREY); // |_________________|
    tft->drawFastHLine(3, 108, 41, ILI9341_DARKGREY);
    tft->drawFastVLine(44, 104, 4, ILI9341_DARKGREY);
    tft->drawFastHLine(4, 106, 39, COLOR_DARKGREEN);
    tft->drawFastHLine(4, 107, 39, ILI9341_GREEN);

    // EGT liner gauge
    tft->drawFastVLine(2, 123, 4, ILI9341_DARKGREY); // |__|__|__|__|__|__|
    tft->drawFastHLine(3, 127, 41, ILI9341_DARKGREY);
    tft->drawFastVLine(44, 123, 4, ILI9341_DARKGREY);
    for (uint16_t col = 8; col < 44; col+=6) {
      tft->drawFastVLine(col, 125, 2, ILI9341_DARKGREY);
    }

    // Fuel Flow liner gauge
    tft->drawFastVLine(2, 142, 4, ILI9341_DARKGREY); // |_________________|
    tft->drawFastHLine(3, 146, 41, ILI9341_DARKGREY);
    tft->drawFastVLine(44, 142, 4, ILI9341_DARKGREY);
    tft->drawFastHLine(4, 144, 39, COLOR_DARKGREEN);
    tft->drawFastHLine(4, 145, 39, ILI9341_GREEN);

    // Fuel quanty liner gauge
    tft->drawFastVLine(2, 161, 5, ILI9341_DARKGREY); // |-----------------|
    tft->drawFastVLine(44, 161, 5, ILI9341_DARKGREY);
    tft->fillRect(4, 162, 4, 3,ILI9341_RED);
    tft->fillRect(8, 162, 7, 3,ILI9341_YELLOW);
    tft->fillRect(15, 162, 29, 3,ILI9341_DARKGREEN);
    
    // Battery liner gauge
    tft->drawFastVLine(2, 199, 4, ILI9341_DARKGREY); // |_________________|
    tft->drawFastHLine(3, 203, 41, ILI9341_DARKGREY);
    tft->drawFastVLine(44, 199, 4, ILI9341_DARKGREY);
    
    tft->fillRect(47, 19, 273, 211, GND_COLOR);
  } 

  tft->drawFastHLine(66, 8, 186, ILI9341_DARKGREY);
  tft->setCursor(0,0);
  tft->print(F("COM1 129.00"));
  tft->setCursor(254,0);
  tft->print(F("NAV1 114.10"));
  tft->setCursor(0,10);
  tft->print(F("COM2 133.00"));
  tft->setCursor(254,10);
  tft->print(F("NAV2 109.30"));

  //tft->setCursor(160,0);
  //tft->setTextColor(ILI9341_CYAN);
  //tft->print(F("BRG"));
  
  //tft->setCursor(160,10);
  //tft->setTextColor(ILI9341_ORANGE);
  //tft->print(F("BRG"));
  
  for (uint16_t col = 10; col < 296; col+=26) {
    tft->drawFastVLine(col, 230, 10, ILI9341_DARKGREY);
  }
  //tft->drawFastVLine(270, 230, 10, ILI9341_DARKGREY);
  
  tft->setCursor(12, TFT_HEIGHT -8);
  tft->print(F("MODE"));
  tft->setCursor(222, TFT_HEIGHT -8);
  tft->print(F("GPS"));

  if (device == 0) {
    drawIcon(icons, 68, 10, ICO_HOME);
    drawIcon(icons, 176, 10, ICO_DIRECT, ILI9341_CYAN);
    drawIcon(icons, 206, 10, ICO_TIME);
    drawIcon(icons, 68, 00, ICO_WAYPOINT);
    drawIcon(icons, 109, 00, ICO_EAST);
    drawIcon(icons, 176, 0, ICO_DIRECT, ILI9341_ORANGE);
    drawIcon(icons, 206, 0, ICO_TIME);
    //drawIcon(icons, 228, 240-8, ICO_GPS);
  }
} // drawScreen

/***************************************************************************************
** Function name:           rotateImage
** Description:             Push a rotated copy of the Sprite to TFT screen
*************************************************************************************x*/
void rotateImage(const unsigned short* image, POINT origin, POINT pivot, uint16_t w, uint16_t h, float angle, uint16_t fColor = TFT_OPAQUE, uint16_t tColor = TFT_TRANSPARENT) {
  //float midX, midY;
  //float deltaX, deltaY;
  //int rotX, rotY;
  //int i, j;

  float sa = sin(angle * DEG_TO_RAD);
  float ca = cos(angle * DEG_TO_RAD);
  uint16_t midw = w/2;
  uint16_t midh = h/2;

  ///////////////////////////
  for (uint32_t i = 0; i < (w * h); i++) {
    int x1 = (i % w) + (origin.x - midw);
    int y1 = (i / w) + (origin.y - midh);

    int x0 = ca * (x1 - pivot.x) - sa * (y1 - pivot.y) + pivot.x;
    int y0 = sa * (x1 - pivot.x) + ca * (y1 - pivot.y) + pivot.y;

    //int x0 = ca * (x1 - pivot.x) - sa * (y1 - pivot.y -1) + pivot.x;
    //int y0 = sa * (x1 - pivot.x) + ca * (y1 - pivot.y -1) + pivot.y -1;
      
    int poss = (y0 * H_COLS) + x0;

    uint16_t pixel;
    pixel = pgm_read_word(&image[i]);

    if (pixel != tColor) {
      if (pixel == TFT_OPAQUE) {
        pixel = setDarkColor(screen[poss], 60);
      }
      if (fColor == TFT_OPAQUE)
        screen[poss] = pixel;
      else
        screen[poss] = fColor;
    }
  }

  /*

  midX = width / 2.0f;
  midY = height / 2.0f;
  
  float sa = sin(-angle * DEG_TO_RAD);
  float ca = cos(-angle * DEG_TO_RAD);

  for (i = 0; i < width; i++){
    for (j = 0; j < height; j++) {      
     
      deltaX = i - midX;
      deltaY = j - midY;

      rotX = ca * (origin.x - pivot.x) - sa * (origin.y - pivot.y) + pivot.x;
      rotY = sa * (origin.x - pivot.x) + ca * (origin.y - pivot.y) + pivot.y;
      
      rotX = (int)(midX + deltaX * sa + deltaY * ca);
      rotY = (int)(midY + deltaX * ca - deltaY * sa);
      
      if (rotX >= 0 && rotX < width && rotY >= 0 && rotY < height) {
        //buff[(j * width + i)] = pgm_read_word(&image[(rotX * width + rotY)]);
        uint32_t poss = (j * 320 + (origin.y * 320) + (i + origin.x));
        uint16_t pixel = pgm_read_word(&image[(rotX * width + rotY)]);
        if (pixel != tColor) {
          if (fColor == TFT_OPAQUE)
            screen[poss] = pixel;
          else
            screen[poss] = fColor;
        }
      }
    }
  }
  */
}

/***************************************************************************************
** Function name:           drawImage
***************************************************************************************/
void drawImage(const unsigned short* image, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fColor = TFT_OPAQUE, uint16_t tColor = TFT_TRANSPARENT) {

  for (uint32_t i = 0; i < (w * h); i++) {
    uint32_t x0 = (i % w) + x;
    uint32_t y0 = (i / w) + y;
    uint32_t poss = (y0 * H_COLS) + x0;

    uint16_t pixel;
    pixel = pgm_read_word(&image[i]);

    if (pixel != tColor) {
      if (pixel == TFT_OPAQUE) {
        pixel = setDarkColor(screen[poss], 60);
      }
      if (fColor == TFT_OPAQUE)
        screen[poss] = pixel;
      else
        screen[poss] = fColor;
    }
  }
} //drawImage

/***************************************************************************************
** Function name:           drawScale
***************************************************************************************/
void drawScale(const unsigned short* image, float value, uint16_t x, uint16_t y, uint16_t w, uint16_t h, float factor, uint16_t imgH) {

  uint32_t ix = ((imgH - h) - (uint32_t)(value * factor))*w;

  for (uint32_t i = 0; i < (w * h); i++) {
    uint32_t x0 = (i % w) + x;
    uint32_t y0 = (i / w) + y;
    uint32_t posj = ix + i;
    uint32_t poss = y0 * H_COLS + x0;
    uint16_t color = pgm_read_word(&image[posj]);
    if (color == TFT_OPAQUE) {
      color = setDarkColor(screen[poss], 60);
    }
    screen[poss] = color;
  }
} //drawScale

/***************************************************************************************
** Function name:           drawPitchScale
***************************************************************************************/
void drawPitchScale(const unsigned short* image, uint16_t x, uint16_t y, uint16_t w, uint16_t h, float factor, AHRS *ahrs) {  

  float pitch = ahrs->pitch;
  float roll = ahrs->roll;
  
  if (pitch < -80) pitch = -80;
  if (pitch > 80) pitch = 80;
  pitch = pitch + 80;
  int32_t ix = (int32_t)(pitch * factor)*w;

  int32_t px = x + w/2;
  int32_t py = y + h/2;

  for (uint32_t i = 0; i < (w * h); i++) {
    int32_t x0 = (i % w) + x;
    int32_t y0 = (i / w) + y;

    int32_t x1 = ahrs->cosRoll * (x0 - px) - ahrs->sinRoll * (y0 - py) + px;
    int32_t y1 = ahrs->sinRoll * (x0 - px) + ahrs->cosRoll * (y0 - py) + py;

    if (x1 < 0) x1=0;
    if (x1 > 320) x1=320;
    if (y1 < 0) y1=0;
    if (y1 > 240) y1=240;
    
    int32_t posj = ix + i;
    int32_t poss = y1 * H_COLS + x1;
    uint16_t color = pgm_read_word(&image[posj]);
    if (color != TFT_TRANSPARENT) {
      screen[poss] = color;
    }
  }
} //drawPitchScale

/***************************************************************************************
** Function name:           drawBackGround
***************************************************************************************/
void drawBackGround(AHRS *ahrs) {
  int32_t oldyf1 = -1;
  int32_t oldyf2 = -1;
  float roll = -ahrs->roll;
  float pitch = -ahrs->pitch * 4.4;
  if (pitch > 180) pitch = 180;
  if (pitch < -180) pitch = -180;
  
  float ta = tan(roll * DEG_TO_RAD);
  
  int32_t xc = X_PIVOT;
  if (pitch < -60) pitch = -60;
  if (pitch > 120) pitch = 120;
  int32_t yc = pitch + Y_PIVOT;
  if (yc < YMIN) yc = YMIN;
  if (yc > YMAX) yc = YMAX;
  int32_t x1 = XMIN;
  int32_t y1 = yc;
  int32_t x2 = XMAX;
  int32_t y2 = yc;
  int32_t cadj = xc - x1;
  int32_t co = ta * cadj;
  int32_t yf1 = y1 - co;
  int32_t yf2 = y2 + co;

  if ((yf1 >= YMIN) && (yf1 <= YMAX)){
    oldyf1 = yf1; 
  } else {
    if (yf1 < YMIN) oldyf1 = YMIN;
    if (yf1 > YMAX) oldyf1 = YMAX;
  }
  if ((yf2 >= YMIN) && (yf2 <= YMAX)){
    oldyf2 = yf2; 
  } else {
    if (yf2 < YMIN) oldyf2 = YMIN;
    if (yf2 > YMAX) oldyf2 = YMAX;
  }
  if ((yf1 > YMIN) && (yf2 > YMIN) && (yf1 < YMAX) && (yf2 < YMAX)) {
    if (yf1 == yf2) {
      pfd.fillRect(x1, YMIN, x2, oldyf2 - YMIN, SKY_COLOR);
      pfd.fillRect(x1, y2, x2, YMAX - oldyf1, GND_COLOR);
    }else{
      if (yf1 > yf2) {
        pfd.fillRect(x1, YMIN, x2, oldyf2 - YMIN, SKY_COLOR);
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2, x1,oldyf2, SKY_COLOR);
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2+1, x2,oldyf1, GND_COLOR);
        pfd.fillRect(x1, oldyf1, x2, YMAX - oldyf1, GND_COLOR);
      }else{
        pfd.fillRect(x1, YMIN, x2, oldyf1 - YMIN, SKY_COLOR);
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2, x2,oldyf1, SKY_COLOR);
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2, x1,oldyf2, GND_COLOR);
        pfd.fillRect(x1, oldyf2, x2, YMAX - oldyf2, GND_COLOR);
      }
    }
    pfd.drawLine(x1,oldyf1, x2,oldyf2,ILI9341_LIGHTGREY);
  } else {
    if (yf1 <= YMIN) {
      if (yf1 > 0){
        co = YMIN - yf1;
      }else{
        co = YMIN + abs(yf1);
      }
      x1 = co / ta;
    }
     if (yf1 > YMAX) {
      co = yf1 - YMAX;
      x1 = co / abs(ta);
    }
 
    if (yf2 <= YMIN) {
      if (yf2 > 0){
        co = YMIN - yf2;
      }else{
        co = YMIN + abs(yf2);
      }
      x2 = XMAX - (co / abs(ta));
    }
     if (yf2 > YMAX) {
      co = yf2 - YMAX;
      x2 = XMAX - (co / abs(ta));
    }
      if (ta > 0 ) {
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2, x2,oldyf1, SKY_COLOR);
        pfd.fillRect(XMIN, YMIN, x2, oldyf1 - YMIN, SKY_COLOR);
        pfd.fillRect(x2, YMIN, XMAX - x2, YMAX - YMIN, SKY_COLOR);
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2, x1,oldyf2, GND_COLOR);
        pfd.fillRect(XMIN, YMIN, x1, YMAX - YMIN, GND_COLOR);
        pfd.fillRect(x1, oldyf2, XMAX - x1, YMAX - oldyf2, GND_COLOR);
      }else{
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2, x2,oldyf1, GND_COLOR);
        pfd.fillRect(x2, oldyf2, XMAX - x2, YMAX - YMIN, GND_COLOR);
        pfd.fillRect(x1, oldyf1, x2, YMAX - oldyf1 , GND_COLOR);
        pfd.fillTriangle(x1,oldyf1, x2, oldyf2, x1,oldyf2, SKY_COLOR);
        pfd.fillRect(XMIN, YMIN, x1, YMAX - YMIN, SKY_COLOR);
        pfd.fillRect(XMIN, YMIN, x2, oldyf2 - YMIN, SKY_COLOR);
      }
      pfd.drawLine(x1, oldyf1, x2, oldyf2,ILI9341_LIGHTGREY);
  }
}

/***************************************************************************************
** Function name:           drawCompas
** Description:             Draw compass
***************************************************************************************/
void drawCompass(int16_t heading, int16_t course = 0, int16_t heading_to = 0, int16_t bearing_to = 0){

  int16_t angle = heading;

  // Draw compas marks
  drawImage(compass_background, X_PIVOT - compass_background_W/2, 182-compass_background_H/2, compass_background_W, compass_background_H);
  for (int16_t da=0; da < 360; da+=5) {
    if (da % 10)
      pfd.drawLineByAngle(X_PIVOT+1, 182, normalizeAngle(-angle, da), 43, 2, ILI9341_LIGHTGREY);
    else
      pfd.drawLineByAngle(X_PIVOT+1, 182, normalizeAngle(-angle, da), 41, 4, ILI9341_LIGHTGREY);
  }

  // Draw compass external marks   
  pfd.drawLineByAngle(X_PIVOT+1, 182,  18, 48, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT+1, 182,  45, 48, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT+1, 182,  90, 48, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT+1, 182, 135, 48, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT+1, 182, 225, 48, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT+1, 182, 270, 48, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT+1, 182, 315, 48, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT+1, 182, -18, 48, 4, ILI9341_WHITE);

  POINT pivot = {X_PIVOT, 182};
  
  // Draw compass values
  POINT origin = {X_PIVOT, 146};
  rotateImage(nsew + 0, origin, pivot, nsew_W, nsew_H, normalizeAngle(-angle, 0));
  rotateImage(nsew + 35, origin, pivot, nsew_W, nsew_H, normalizeAngle(-angle, 90));
  rotateImage(nsew + 70, origin, pivot, nsew_W, nsew_H, normalizeAngle(-angle, 180));
  rotateImage(nsew + 105, origin, pivot, nsew_W, nsew_H, normalizeAngle(-angle, 270));

  rotateImage(small_num + (3 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 30));
  rotateImage(small_num + (6 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 60));
  
  rotateImage(small_num + (1 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 120 -4));
  rotateImage(small_num + (2 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 120 +4));
  
  rotateImage(small_num + (1 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 150 -4));
  rotateImage(small_num + (5 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 150 +4));

  rotateImage(small_num + (2 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 210 -4));
  rotateImage(small_num + (1 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 210 +4));

  rotateImage(small_num + (2 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 240 -4));
  rotateImage(small_num + (4 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 240 +4));

  rotateImage(small_num + (3 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 300 -4));
  rotateImage(small_num + (0 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 300 +4));

  rotateImage(small_num + (3 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 330 -4));
  rotateImage(small_num + (3 * 15), origin, pivot, small_num_W, small_num_H, normalizeAngle(-angle, 330 +4));

  // Put Heading
  //pfd.drawRect(TFT_WIDTH/2 - 12, 125, 23, 11, ILI9341_WHITE);
  pfd.fillRect(TFT_WIDTH/2 - 10, 123, 21, 9, ILI9341_BLACK);
  pfd.setTextColor(ILI9341_WHITE);
  pfd.setCursor(TFT_WIDTH/2 - 8 ,124);
  if (angle == 0) angle = 360;
  pfd.printf("%03d", (int)angle);

  drawImage(triangle_dn, X_PIVOT-2, 134, triangle_W, triangle_H);
  origin = {X_PIVOT, 139};
  rotateImage(bug, origin, pivot, bug_W, bug_H, -angle + heading_to, ILI9341_CYAN);
  
  origin = {X_PIVOT, 182};
  
  // Show direction to home
  rotateImage(arrow1, origin, pivot, arrow_W, arrow_H, -angle + bearing_to);
  
  if ((wp.wp_number > 0) || (flight_director)) {
    rotateImage(arrow, origin, pivot, arrow_W, arrow_H, -angle + course, DARK_ORANGE);  
    rotateImage(lateral, origin, pivot, lateral_W, lateral_H, -angle + course, ILI9341_LIGHTGREY);
  }
  drawImage(plane, X_PIVOT - plane_W/2, 182-plane_H/2, plane_W, plane_H);
}

/***************************************************************************************
** Function name:           drawRollScale
** Description:             Draw rolling scale
***************************************************************************************/
void drawRollScale(float roll) {

  if (roll < -60) roll = -60;
  if (roll > 60) roll = 60;
  drawArc(X_PIVOT, Y_PIVOT, 59, -60 + roll, 60 + roll, ILI9341_WHITE);
  //drawArc(TFT_WIDTH/2, 88, 60, -60 + apm.ahrs.roll, 60 + apm.ahrs.roll, ILI9341_WHITE);

  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll - 10, 59, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll - 20, 59, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll - 30, 59, 7, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll - 45, 59, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll - 60, 59, 7, ILI9341_WHITE);
    
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll + 10, 59, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll + 20, 59, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll + 30, 59, 7, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll + 45, 59, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll + 60, 59, 7, ILI9341_WHITE);

  POINT origin;
  POINT pivot;

  origin = {X_PIVOT, 24};
  pivot = {X_PIVOT, Y_PIVOT};
  rotateImage(triangle_dn, origin, pivot, triangle_W, triangle_H, roll);
}

/***************************************************************************************
** Function name:           updateTime
** Description:             Draw time from MavLink
***************************************************************************************/
void updateTime(){
  // Time to WP
  uint32_t ss = time_boot_ms / 1000;
  uint32_t hh = ss / 3600;
  ss -= hh * 3600;
  uint32_t mm = ss / 60;
  ss -= mm * 60;
  if (hh > 59) hh = 0;
  if (mm > 59) mm = 0;
  if (ss > 59) ss = 0;

  pfd.setTextColor(ILI9341_WHITE);
  pfd.setCursor(272,TFT_HEIGHT -18);
  pfd.printf("%02d:%02d:%02d", hh, mm, ss);

  pfd.setCursor(2, TFT_HEIGHT -18);
  pfd.printf("T:%3.0f%c", apm.temperature, DEGREE_SYMBOL); // Degree symbol
}

/***************************************************************************************
** Function name:           flightMode
** Description:             Put flight mode in display
***************************************************************************************/
void flightMode(uint16_t x, uint16_t y, uint16_t color){
    uint8_t *mode_str = "";
    if (apm_mav_type == 2){ //ArduCopter MultiRotor or ArduCopter Heli
        if (flight_mode < 0)        {mode_str = ""; flight_director = false;}
        else if (flight_mode == 0)  {mode_str = " STABILIZED ";  flight_director = false;}//Stabilize: hold level position
        else if (flight_mode == 1)  {mode_str = "    ACRO    ";  flight_director = false;}//Acrobatic: rate control
        else if (flight_mode == 2)  {mode_str = "  ALT HOLD  ";  flight_director = false;}//Altitude Hold: auto control
        else if (flight_mode == 3)  {mode_str = "    AUTO    ";  flight_director = true;}//Auto: auto control
        else if (flight_mode == 4)  {mode_str = "   GUIDED   ";  flight_director = true;}//Guided: auto control
        else if (flight_mode == 5)  {mode_str = "   LOITER   ";  flight_director = true;}//Loiter: hold a single location
        else if (flight_mode == 6)  {mode_str = "     RTL    ";  flight_director = true;}//Return to Launch: auto control
        else if (flight_mode == 7)  {mode_str = "   CIRCLE   ";  flight_director = true;}//Circle: auto control
        else if (flight_mode == 8)  {mode_str = "  POSITION  ";  flight_director = true;}//Position: auto control
        else if (flight_mode == 9)  {mode_str = "  LANDING   ";  flight_director = true;}//Land:: auto control
        else if (flight_mode == 10) {mode_str = "  OF LOITER ";  flight_director = true;}//OF_Loiter: hold a single location using optical flow sensor
        else if (flight_mode == 11) {mode_str = "    DRIFT   ";  flight_director = true;}//Drift mode: 
        else if (flight_mode == 13) {mode_str = "    SPORT   ";  flight_director = false;}//Sport: earth frame rate control
        else if (flight_mode == 14) {mode_str = "    FLIP    ";  flight_director = false;}//Flip: flip the vehicle on the roll axis
        else if (flight_mode == 15) {mode_str = "  AUTOTUNE  ";  flight_director = false;}//Auto Tune: autotune the vehicle's roll and pitch gains
        else if (flight_mode == 16) {mode_str = "   HYBRID   ";  flight_director = false;}//Hybrid: position hold with manual override
    } else if(apm_mav_type == 1){ //ArduPlane
        if (flight_mode < 0)        {mode_str = ""; flight_director = false;}
        else if (flight_mode == 0)  {mode_str = "   MANUAL   "; flight_director = false;}//Manual
        else if (flight_mode == 1)  {mode_str = "   CIRCLE   "; flight_director = false;} //Circle
        else if (flight_mode == 2)  {mode_str = " STABILIZED "; flight_director = true;} //Stabilize
        else if (flight_mode == 3)  {mode_str = "  TRAINING  "; flight_director = true;} //Training
        else if (flight_mode == 4)  {mode_str = "    ACRO    "; flight_director = false;}//Acro
        else if (flight_mode == 5)  {mode_str = "    FBWA    "; flight_director = true;}//Fly_By_Wire_A
        else if (flight_mode == 6)  {mode_str = "    FBWB    "; flight_director = true;}//Fly_By_Wire_B
        else if (flight_mode == 7)  {mode_str = "   CRUISE   "; flight_director = true;}//Cruise
        else if (flight_mode == 8)  {mode_str = "  AUTOTUNE  "; flight_director = false;}//Auto Tune
        else if (flight_mode == 10) {mode_str = "    AUTO    "; flight_director = true;}//Auto
        else if (flight_mode == 11) {mode_str = "    RTL     "; flight_director = true;}//Return to Launch
        else if (flight_mode == 12) {mode_str = "   LOITER   "; flight_director = true;}//Loiter
        else if (flight_mode == 15) {mode_str = "   GUIDED   "; flight_director = true;}//Guided
        else if (flight_mode == 16) {mode_str = "INITIALIZING"; flight_director = false;}//Initializing
    }
    pfd.setCursor(x,y);
    pfd.setTextColor(color, ILI9341_BLACK);
    pfd.printf("%s", mode_str);

    mfd.setCursor(x,y);
    mfd.setTextColor(color, ILI9341_BLACK);
    mfd.printf("%s", mode_str);
}

/***************************************************************************************
** Function name:           drawEFIS
** Description:             Draw EFIS
***************************************************************************************/
void drawEFIS() {
  char buf[10];
  //long temp_rotina = millis();
  //pfd.refreshOnce();
  //pfd.updateScreen();
  //pfd.updateScreenAsync();
  //pfd.waitUpdateAsyncComplete();
  
  //apm.ahrs.roll = 30;
  //static float vari = 1;
  //apm.alt = apm.alt + vari;
  //if (apm.alt > 250) vari = -1; 
  //if (apm.alt < 0) vari = 1; 
  //uint32_t tempo = micros();

  drawBackGround(&apm.ahrs);
  drawPitchScale(pitch_scale, 136, 42, pitch_scale_W, 80, 4.4, &apm.ahrs);
  drawScale(speed_scale_km, apm.airspeed, 59, 41, speed_scale_km_W, 104, 4, 614);  // 0 - 125
  if (apm.alt > 250.0){
    debug.printf(F("Error Alt limit: %fm\n"), apm.alt);
    apm.alt = 250.0;
  }
  drawScale(altitude_scale, apm.alt, 236, 41, altitude_scale_W, 104, 2, 614);
  drawImage(alt_bug, 236, 32, alt_bug_W, alt_bug_H);
  drawIcon(icons, 238, 33, ICO_BUG, ILI9341_BLACK);
  drawImage(vbar, (H_COLS /2) - (vbar_W / 2), 80, vbar_W, vbar_H);

  // Substituido pelo rotateImage
  //drawImage(vbar1, (H_COLS /2) - (vbar1_W / 2), 83 + vbar1_H, vbar1_W, vbar1_H);
  POINT origin = {X_PIVOT, Y_PIVOT + vbar1_H/2 +1};
  POINT pivot = {X_PIVOT, Y_PIVOT};

  drawImage(leftbox, 59, 78, bugbox_W, bugbox_H);
  drawImage(rightbox, 238, 78, bugbox_W, bugbox_H);
  drawImage(vs_scale, 236 + altitude_scale_W + 1, 45, vs_scale_W, vs_scale_H);
   
  //pfd.drawRect(58,40,27,106, ILI9341_WHITE);
  //pfd.drawRect(235,40,28,106, ILI9341_WHITE);

  to_heading = wp.nav_bearing;
  //to_course = (int16_t)wp_target_bearing_rotate_int;
  to_course = (int16_t)wp.target_bearing;
  //drawCompass(int16_t heading, int16_t course = 0, int16_t heading_to = 0, int16_t bearing_to = 0){
  drawCompass((int16_t)apm.heading, to_course, to_heading, home.direction);
  //drawCompass((int16_t)apm.heading);
  
  if (flight_director) {
    int16_t flight_director_angle;
    if (to_heading < 0){
      to_heading = to_heading + 360;
    }
    
    flight_director_angle =  (float)(apm.heading - wp.nav_bearing);

    if (flight_director_angle > 180) flight_director_angle = flight_director_angle - 360;
    if (flight_director_angle > 30) flight_director_angle = 30;  
    if (flight_director_angle < -30) flight_director_angle = -30;

    POINT origin2 = {X_PIVOT, Y_PIVOT + vbar1_H/2 +1 - apm.ahrs.pitch};
    //debug.printf("wp.nav_bearing: %d\tto_heading: %d\tapm.heading: %d\tflight_director_angle: %d\n", wp.nav_bearing, to_heading, apm.heading, flight_director_angle);
    rotateImage(vbar1, origin2, pivot, vbar1_W, vbar1_H, (float)(-flight_director_angle/2)); // Deve ser inserido angulo de curva
  }

  //int16_t climb = (int16_t)((apm.climb * 100.0f) / 5); // m/s -> cm/s -> pixel/s  // using scale max 2 m/s
  int16_t climb = (int16_t)(apm.climb * 100.0f); // m/s -> cm/s -> pixel/s    // using scale max 4 m/s
  if (climb > HI_LIMIT_OF_CLIMB) climb = HI_LIMIT_OF_CLIMB;
  if (climb < LO_LIMIT_OF_CLIMB) climb = LO_LIMIT_OF_CLIMB;
  drawImage(vs_bug, 236 + altitude_scale_W + 1, -(climb / 10) + 85, vs_bug_W, vs_bug_H);     //  45 < y < 125
  pfd.setCursor(236 + altitude_scale_W + 2, -(climb / 10) + 86);
  pfd.setTextColor(ILI9341_WHITE);
  pfd.printf("%3.0f", fabs(apm.climb));
  drawIcon(icons, 236 + altitude_scale_W + 1, 85, ICO_BUG, ILI9341_MAGENTA); //  46 < y < 126

  drawRollScale(apm.ahrs.roll);
  drawImage(triangle_up, 158, 31, triangle_W, triangle_H);

  // Put Aispeed and Altitude
  pfd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  pfd.setCursor(60,85);
  pfd.printf("%3.0f", apm.airspeed);
  float valUnit = abs(apm.airspeed);
  if (valUnit >= 100){
    valUnit = fmod(valUnit,100.0);
  }
  if (valUnit >= 10){
    valUnit = fmod(valUnit,10.0);
  }
  drawScale(num_scale_V, valUnit, 72, 80, num_scale_V_W, 17, 10, num_scale_V_H);
    
  pfd.setCursor(243,85);
  pfd.printf("%3.0f", apm.alt);
  valUnit = abs(apm.alt);
  if (valUnit >= 100){
    valUnit = fmod(valUnit,100.0);
  }
  if (valUnit >= 10){
    valUnit = fmod(valUnit,10.0);
  }
  drawScale(num_scale_V, valUnit, 255, 80, num_scale_V_W, 17, 10, num_scale_V_H);

  //  PITCH > 30º OR PITCH < -20º  Hide fields
  //  -15º < PITCH < 25º Show fields
  if ((apm.ahrs.pitch > -15.0) && (apm.ahrs.pitch < 25.0)) {
    
    // GroundSpeed
    fillRect(59,145,25,9, ILI9341_BLACK);
    pfd.setTextColor(ILI9341_WHITE);
    pfd.setCursor(60,146);
    pfd.printf("%3.0f", apm.groundspeed);
    drawIcon(icons, 78, 146, ICO_GS, ILI9341_CYAN);

    // Field QNH
    fillRect(236,145,27,9, ILI9341_BLACK);

    // HDG
    pfd.fillRect(105,132,28,7,ILI9341_BLACK);
    hdgLabel->printText("HDG ");
    if (wp.nav_bearing < 0) {
      sprintf (buf, "%03d", 360 + (int)wp.nav_bearing);
    }else{
      sprintf (buf, "%03d", (int)wp.nav_bearing);
    }
    hdgValue->printText(buf);

    // CRS
    pfd.fillRect(187,132,28,7,ILI9341_BLACK);
    crsLabel->printText("CRS ");
    sprintf (buf, "%03d", (int)home.direction);
    crsValue->printText(buf);
     
    // Next Waypoint Information
    pfd.setCursor(77,00);
    pfd.setTextColor(DARK_ORANGE, ILI9341_BLACK);
    pfd.printf("%3dWP", (int)wp.wp_number);
    pfd.setCursor(118,0);
    pfd.fillRect(118,0,42,7,ILI9341_BLACK);
    if (wp.wp_dist > 1000)
      pfd.printf("%3.1fKm", (float)(wp.wp_dist / 1000.0));
    else
      pfd.printf("%3dm", (int)wp.wp_dist);
    
    // Bearing to WP
    //wp_target_bearing_rotate_int = (float)(wp.target_bearing - apm.heading)+360.0;
    //if (wp_target_bearing_rotate_int > 360.0) wp_target_bearing_rotate_int -= 360.0;
    //drawIcon(icons, 178, 10, dirIconIndex(wp.nav_bearing));
    //drawIcon(icons, 178, 10, dirIconIndex((int16_t)wp_target_bearing_rotate_int));
    pfd.setCursor(186,00);
    pfd.setTextColor(DARK_ORANGE, ILI9341_BLACK);
    if (wp.target_bearing < 0) {
      pfd.printf("%03d", 360 + (int)wp.target_bearing);
    }else{
      pfd.printf("%03d", (int)wp.target_bearing);
    }
  
    // Altitute to WP
    pfd.setCursor(244, 33);
    pfd.setTextColor(ILI9341_BLACK, 0x279E);
    pfd.printf("%03d", (int)wp.alt_error);
      
    // Time to WP
    if (apm.groundspeed >= 2) {
      uint32_t ete = wp.ete;
      uint16_t hh = ete / 3600;
      ete %= 3600;
      uint16_t mm = ete / 60;
      uint16_t ss = ete % 60;
      pfd.setTextColor(DARK_ORANGE, ILI9341_BLACK);
      pfd.setCursor(216,0);
      if (hh > 0) {
        if (hh > 23) {
          uint16_t dd = hh / 24;
          hh -= 24;
          if (hh > 99) hh = 99;
          pfd.printf("%02dd%02dh", (int)dd,(int)hh);
        } else {
          pfd.printf("%02d:%02dh", (int)hh,(int)mm);
        }
      } else {
        pfd.printf("%02d:%02dm", mm,ss);
      }

      // Time to Home
      ete = home.ete;
      hh = ete / 3600;
      ete %= 3600;
      mm = ete / 60;
      ss = ete % 60;
      pfd.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
      pfd.setCursor(216,10);
      if (hh > 0) {
        if (hh > 23) {
          uint16_t dd = hh / 24;
          hh -= 24;
          if (hh > 99) hh = 99;
          pfd.printf("%02dd%02dh", (int)dd,(int)hh);
        } else {
          pfd.printf("%02d:%02dh", (int)hh,(int)mm);
        }
      } else {
        pfd.printf("%02d:%02dm", mm,ss);
      }
    }else{
      home.ete = 0;
      wp.ete = 0;
      pfd.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
      pfd.setCursor(216,0);
      pfd.printf("  :  m");
      pfd.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
      pfd.setCursor(216,10);
      pfd.printf("  :  m");
    }
    

    // Home information
    pfd.setCursor(77,10);
    pfd.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    pfd.fillRect(77,10,42,07,ILI9341_BLACK);
    if (home.distance > 1000)
      pfd.printf("%3.1fKm", (float)(home.distance / 1000.0));
    else
      pfd.printf("%3dm", (int)home.distance);
    // Direction
    pfd.setCursor(186,10);
    pfd.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    pfd.printf("%03d", home.direction);
    // Draw heading to Home
    //drawIcon(icons, 178, 0, dirIconIndex(home.direction), ILI9341_CYAN, ILI9341_BLACK);
    // Time to Home
  }
  
  // Motor ARMED
  if(apm.motor_armed) {
    pfd.fillRect(167, TFT_HEIGHT -9, 25, 9, ILI9341_RED);
    pfd.setCursor(170, TFT_HEIGHT -8);
    pfd.setTextColor(ILI9341_WHITE);
    pfd.print(F("ARM"));
  }else{
    pfd.fillRect(167, TFT_HEIGHT -9, 25, 9, ILI9341_DARKGREY);
    pfd.setCursor(170, TFT_HEIGHT -8);
    pfd.setTextColor(ILI9341_BLACK);
    pfd.print(F("DIS"));
  }

  // Show GPS status information
  pfd.setCursor(248,240-8);
  pfd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  
  mfd.setCursor(248,240-8);
  mfd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  
  sprintf (buf, "%2d", (int)apm.satellites_visible);
  pfd.printf(buf);
  mfd.printf(buf);
  
  // Draw small bargraph sat visible
  uint8_t s = 7;
  while(s > apm.satellites_visible) {
    pfd.drawFastVLine(260+s,238-s,s+1,ILI9341_BLACK);
    mfd.drawFastVLine(260+s,238-s,s+1,ILI9341_BLACK);
    s--;
  };
  for(s=0; s < apm.satellites_visible; s++) {
    if (apm.satellites_visible < 9) {
      pfd.drawFastVLine(260+s,238-s,s+1,ILI9341_GREEN);
      mfd.drawFastVLine(260+s,238-s,s+1,ILI9341_GREEN);
    }
  }
  pfd.setCursor(275,240-8);
  if (tick_half_second) {
    pfd.setTextColor(ILI9341_WHITE, ILI9341_RED);
    mfd.setTextColor(ILI9341_WHITE, ILI9341_RED);
  }else{
    pfd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    mfd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  }
  switch(apm.fix_type){
    case GPS_FIX_TYPE_NO_FIX:
      sprintf (buf, "%s", F("NO FIX"));
      break;
    case GPS_FIX_TYPE_2D_FIX:
      pfd.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
      mfd.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
      sprintf (buf, "%s", F("  2D  "));
      break;
    case GPS_FIX_TYPE_3D_FIX:
    case GPS_FIX_TYPE_DGPS:
    case GPS_FIX_TYPE_RTK_FLOAT:
    case GPS_FIX_TYPE_RTK_FIXED:
    case GPS_FIX_TYPE_STATIC:
    case GPS_FIX_TYPE_PPP:
      pfd.setTextColor(ILI9341_BLACK, ILI9341_GREEN);
      mfd.setTextColor(ILI9341_BLACK, ILI9341_GREEN);
      sprintf (buf, "%s", F("  3D  "));
      break;
    default:
      sprintf (buf, "%s", F("NO GPS"));
      break;
  }

  pfd.print(buf);
  mfd.print(buf);
  
  // Temperature and Time fields
  fillRect(0, 220, 40, 10, ILI9341_BLACK);
  fillRect(270, 220, 50, 10, ILI9341_BLACK);
  updateTime();

  flightMode(40, 240-8, ILI9341_GREEN);
 
  //pfd.updateScreen();
  pfd.updateScreenAsync();
  //pfd.waitUpdateAsyncComplete();

  //sprintf (buf, "%04d", (int)rpm_value);
  //rpmGauge->value((int32_t)rpm_value);
  //rpmLabel->printText(buf);

  //tempo = micros() - tempo;
  //debug.printf("Tempo %dus\n", tempo);

  //debug.printf("Draw Efis : %d ms\n", int(millis() - temp_rotina));
}

/***************************************************************************************
** Function name:           setHomeVars
** Description:             Home Distance and Direction Calculation
***************************************************************************************/
void setHomeVars(void)
{
  tick_half_second = !tick_half_second;
  
  if(home.got_home == false && apm.fix_type > GPS_FIX_TYPE_NO_FIX){
    home.position.lat = apm.position.lat;
    home.position.lon = apm.position.lon;
    //home.position.alt = apm.alt;
    home.got_home = true;
  } else if(home.got_home){
    // JRChange: home.position.alt: check for stable apm.alt (must be stable for 25*120ms = 3s)
    if(apm.alt < 25){
      if(fabs(alt_prev - apm.alt) > 0.5){
        alt_cnt = 0;
        alt_prev = apm.alt;
      } else {
        if(++alt_cnt >= 25){
          home.position.alt = apm.alt;  // take this stable apm.alt as home.position.alt
        }
      }
    }

    float lat1 = apm.position.lat * DEG_TO_RAD;
    float lon1 = apm.position.lon * DEG_TO_RAD;
    float lat2 = home.position.lat * DEG_TO_RAD;
    float lon2 = home.position.lon * DEG_TO_RAD;
    float difLat = lat2 - lat1;
    float difLon = lon2 - lon1;

    float a = sin(difLat / 2) * sin(difLat / 2) + cos(lat1) * cos(lat2) * sin(difLon / 2) * sin(difLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    home.distance = (uint32_t)(6378137.0 * c);

    float radDist = home.distance;// * DEG_TO_RAD;

    if(sin(difLon)<0)
      home.direction = (uint16_t)(acos((sin(lat2)-sin(lat1)*cos(radDist))/(sin(radDist)*cos(lat1))));
    else
      home.direction = (uint16_t)(2 * M_PI-acos((sin(lat2)-sin(lat1)*cos(radDist))/(sin(radDist)*cos(lat1))));    

    float y = sin(difLon) * cos(lat2);
    float x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(difLon);
    home.direction = (int16_t)(atan2(y, x) * RAD_TO_DEG - mag_decl);
    if (home.direction < 0) home.direction += 360;

    // Calcule ETE in seconds
    home.ete = ((home.distance / 1000) / (apm.groundspeed * converts)) * 3600 ;
    //debug.printf("Distance:%03d Crs:%03d Dir:%03d GS:%d ETE:%d\n", home.distance, (int)bearing, (int)home.direction, (int)(apm.groundspeed * converts), (int)home.ete);
    // Calcule ETE in seconds to WP
    wp.ete = ((wp.wp_dist / 1000) / (apm.groundspeed * converts)) * 3600 ;
  }
}

/***************************************************************************************
** Function name:           updateMFD
** Description:             Show information into MFD display
***************************************************************************************/
void updateMFD(){
#ifdef USE_MFD
  char buf[10];
  // Show voltage
  uint16_t barcolor;

  
  if (apm.remainingA <= 25) // 
  {
    mfd.fillRect(2, 205, 43, 9, ILI9341_WHITE);
    mfd.setTextColor(ILI9341_RED, ILI9341_WHITE);
    barcolor = ILI9341_YELLOW;
  } else {
    mfd.fillRect(2, 205, 43, 9, MFD_BACKCOLOR);
    mfd.setTextColor(ILI9341_WHITE, MFD_BACKCOLOR);
    barcolor = ILI9341_GREEN;
  }
  mfd.setCursor(12, 206);
  mfd.printf("%02.1fV", apm.vbatA);
  sprintf (buf, "%3d%%", (int)apm.remainingA);
  batteryText->printText(buf);
  
  // Battery percent Bargraph
  uint16_t lineW = (uint16_t)round((float)apm.remainingA * 0.4);
  //mfd.drawRect(2, 200, 42, 4, ILI9341_WHITE);
  //mfd.drawFastVLine(2, 199, 4, ILI9341_DARKGREY); // |________________|
  //mfd.drawFastHLine(3, 203, 41, ILI9341_DARKGREY);
  //mfd.drawFastVLine(44, 199, 4, ILI9341_DARKGREY);
  mfd.fillRect(3, 200, 40, 3, MFD_BACKCOLOR);
  mfd.fillRect(3, 201, lineW, 2, barcolor);

  // Update Throttle
  //apm.throttle = 50;
  //debug.printf("THR: %d\n", (int)apm.throttle);
  
  throttleIndicator->moveSprite((uint16_t)apm.throttle);
  sprintf (buf, "%3d%%", apm.throttle);
  throttleText->printText(buf);
  
  engineTempIndicator->moveSprite((uint16_t)apm.temperature);
  sprintf (buf, "%3.0f", apm.temperature);
  engineTempText->printText(buf);
  
  fuelFlowIndicator->moveSprite(0);
  sprintf (buf, "%3d", 0);
  fuelFlowText->printText(buf);
  
  fuelIndicator->moveSprite(MAX_TANK_SIZE);
  sprintf (buf, "%3d", MAX_TANK_SIZE);
  fuelTankText->printText(buf);
  //batteryIndicator->moveSprite(0);
  //mfd.updateScreen();
  //mfd.updateScreenAsync();
  //mfd.waitUpdateAsyncComplete();
#endif  
}
