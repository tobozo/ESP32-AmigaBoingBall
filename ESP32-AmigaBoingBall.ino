/*

  ESP32 AmigaBoingBall - A port of the famous Amiga Boing Ball Demo
  ported from https://github.com/niklasekstrom/boing_ball_python/
  Source: https://github.com/tobozo/ESP32-AmigaBoingBall

  MIT License

  Copyright (c) 2019 tobozo

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  -----------------------------------------------------------------------------

*/

#define M5STACK
//#define ODROIDGO
//#define DDUINO32XS
//#define LOLIND32PRO
//#define WROVER_KIT

//#define USE_NEOPIXEL
//#define USE_ESPI

#include "assets.h"
#include "TinyRaytracer.h"

#if defined(WROVER_KIT)

#elif defined(ODROIDGO)
  #define ROTATION 3
#elif defined(M5STACK)
  #define TFT_eSPI_BUILTIN
  #include <M5Stack.h>
  M5Display tft;
  #define BUZZER 25
#elif defined(LOLIND32PRO)
  // driver select : ST7735_DRIVER / ST7735_GREENTAB
  #define ROTATION 3
#elif defined(DDUINO32XS)
  #define USE_NEOPIXEL
  #define BUZZER 33
#elif defined(INSERT_YOUR_BOARD)
  // and your setting
#endif


#ifndef TFT_eSPI_BUILTIN
  #include <TFT_eSPI.h>
  TFT_eSPI tft;
#endif


TFT_eSprite sprite = TFT_eSprite(&tft);
#include <JPEGDecoder.h>


uint32_t sprite_jpegRender(int xpos, int ypos) {
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;
  bool swapBytes = sprite.getSwapBytes();
  sprite.setSwapBytes(true);
  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = min(mcu_w, max_x % mcu_w);
  uint32_t min_h = min(mcu_h, max_y % mcu_h);
    // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;
  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();
  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;
  // Fetch data from the file, decode and display
  while (JpegDec.read()) {    // While there is more data in the file
    pImg = JpegDec.pImage ;   // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)
    // Calculate coordinates of top left corner of current MCU
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;
    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;
    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;
    // copy pixels into a contiguous block
    if (win_w != mcu_w) {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++) {
        p += mcu_w;
        for (int w = 0; w < win_w; w++) {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }
    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;
    // draw image MCU block only if it will fit on the screen
    if (( mcu_x + win_w ) <= sprite.width() && ( mcu_y + win_h ) <= sprite.height())
      sprite.pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    else if ( (mcu_y + win_h) >= sprite.height())
      JpegDec.abort(); // Image has run off bottom of screen so abort decoding
  }
  sprite.setSwapBytes(swapBytes);
  return millis() - drawTime;
}


uint32_t tft_jpegRender(int xpos, int ypos) {
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;
  bool swapBytes = tft.getSwapBytes();
  tft.setSwapBytes(true);
  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = min(mcu_w, max_x % mcu_w);
  uint32_t min_h = min(mcu_h, max_y % mcu_h);
    // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;
  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();
  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;
  // Fetch data from the file, decode and display
  while (JpegDec.read()) {    // While there is more data in the file
    pImg = JpegDec.pImage ;   // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)
    // Calculate coordinates of top left corner of current MCU
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;
    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;
    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;
    // copy pixels into a contiguous block
    if (win_w != mcu_w) {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++) {
        p += mcu_w;
        for (int w = 0; w < win_w; w++) {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }
    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;
    // draw image MCU block only if it will fit on the screen
    if (( mcu_x + win_w ) <= tft.width() && ( mcu_y + win_h ) <= tft.height())
      tft.pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    else if ( (mcu_y + win_h) >= tft.height())
      JpegDec.abort(); // Image has run off bottom of screen so abort decoding
  }
  tft.setSwapBytes(swapBytes);
  //showTime(millis() - drawTime); // These lines are for sketch testing only
  return millis() - drawTime;
}

void sprite_drawJpg(uint16_t x, uint16_t y, const uint8_t * jpg_data, size_t jpg_len, uint16_t maxWidth, uint16_t maxHeight) {
  boolean decoded = JpegDec.decodeArray(jpg_data, jpg_len);
  if (decoded) {
    sprite_jpegRender(x, y);
  } else {
    Serial.println("Jpeg file format not supported!");
  }
}

void tft_drawJpg(uint16_t x, uint16_t y, const uint8_t * jpg_data, size_t jpg_len, uint16_t maxWidth, uint16_t maxHeight) {
  boolean decoded = JpegDec.decodeArray(jpg_data, jpg_len);
  if (decoded) {
    tft_jpegRender(x, y);
  } else {
    Serial.println("Jpeg file format not supported!");
  }
}


TFT_eSprite ball = TFT_eSprite(&tft);
TFT_eSprite shadow = TFT_eSprite(&tft);
TFT_eSprite grid = TFT_eSprite(&tft);
TFT_eSprite bgImage = TFT_eSprite(&tft);


#ifdef USE_NEOPIXEL
  #define PIXEL_PIN    25    // Digital IO pin connected to the NeoPixels.
  #define PIXEL_COUNT 1
  #include <Adafruit_NeoPixel.h>
  Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif

// Color definitions
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

static bool buzz_wall = false;
static bool buzz_floor = false;
bool hasPsram = false;

#include "AmigaRulez.h"


#ifdef USE_NEOPIXEL
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
#endif

#ifdef BUZZER
void buzzWall() {
  for(int i=0;i<500;i++) {
    digitalWrite(BUZZER,HIGH);
    delayMicroseconds(140);//Change this could adjust voice
    digitalWrite(BUZZER,LOW);
    delayMicroseconds(5);
  }
}
void buzzFloor() {
  for(int i=0;i<500;i++) {
    digitalWrite(BUZZER,HIGH);
    delayMicroseconds(175);//Change this could adjust voice
    digitalWrite(BUZZER,LOW);
    delayMicroseconds(5);
  }
}
static void buzzTask(void * param=NULL) {
  while(1) {
    if( buzz_wall == true ) {
      buzz_wall = false;
      buzzWall();
    }
    if( buzz_floor == true ) {
      buzz_floor = false;
      buzzFloor();
    }
    vTaskDelay(1);
  }
}
#endif


static void animTask(void * param=NULL) {
  while(1) {
    AmigaBall.animate(100, false);
    vTaskDelay(1);
  }
}


void setup() {
  Serial.begin(115200);
  tft.begin();
  #ifdef ROTATION
    tft.setRotation( ROTATION );
  #endif
  amigaBallConfig.Width = tft.width();
  amigaBallConfig.Height = tft.height();
  amigaBallConfig.Wires = 8;
  //amigaBallConfig.XPos = 40;
  //amigaBallConfig.YPos = 20;
  amigaBallConfig.ScaleRatio = 5; // bigger value means smaller ball

  hasPsram = psramInit();

  if( hasPsram ) {
    Serial.println("PRSRAM Detected !");
  } else {
    Serial.println("No PRSRAM");
  }
  

  AmigaBall.init( amigaBallConfig );

  xTaskCreatePinnedToCore(animTask, "animTask", 25000, NULL, 5, NULL, 1); /* last = Task Core */

  #ifdef USE_NEOPIXEL
    // turn that damn neopixel off!!
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    colorWipe(strip.Color(0, 0, 0), 50);
  #endif
  #ifdef BUZZER
    // turn bouncy sounds on
    pinMode(BUZZER,OUTPUT);
    xTaskCreatePinnedToCore(buzzTask, "buzzTask", 2000, NULL, 4, NULL, 0); /* last = Task Core */
  #endif

}


void loop() {
  vTaskSuspend(NULL);
}
