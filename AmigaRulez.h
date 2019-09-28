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


struct color {
  float r;
  float g;
  float b;
};


// checkboard.y = -1e-3
Vec3f BallCoords( 3.2, -2, -18 );
color BallDefaultColor =  {0.4, 0.4, 0.3 };
float BallRadius = 2.;

Light Projector1(Vec3f(-10, 20,  20), 1.5);
Light Projector2(Vec3f( 5, 50, -25), 1.8);
Light Projector3(Vec3f( 5, 20,  30), 0.7);


Vec2f BallCoords2DMin( tft.width(), tft.height());
Vec2f BallCoords2DMax( -tft.width(), -tft.height() );

Vec2f FloorCoords2DMin( tft.width(), tft.height());
Vec2f FloorCoords2DMax( -tft.width(), -tft.height() );

void setBounds(Vec2f &Coords2DMin, Vec2f &Coords2DMax, int i, int j) {
  if ( Coords2DMin.x > i ) Coords2DMin.x = i;
  if ( Coords2DMin.y > j ) Coords2DMin.y = j;
  if ( Coords2DMax.x < i ) Coords2DMax.x = i;
  if ( Coords2DMax.y < j ) Coords2DMax.y = j;
}


float cot(float x) {
  return cos(x)/sin(x);
}

float getSphereRadius( Vec3f &coords, float radius, float fov) {
  //approximate radius on screen[CLIP SPACE] = world radius * cot(fov / 2) / Z
  return radius * cot( fov / 1 ) / coords.z;
}

float project2D(float fov, float d, float r) {
  return 1.0 / tan(fov) * r / sqrt(d * d - r * r);
}


bool debugray = false;

uint16_t getrenderedpixel(uint16_t i, uint16_t j, uint16_t width, uint16_t height, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, float fov) {
  float dir_x =  (i + 0.5) -  width/2.;
  float dir_y = -(j + 0.5) + height/2.;    // this flips the image at the same time
  float dir_z = -height/(2.*tan(fov/2.));
  rayhitsphere = false;
  rayhitfloor = false;
  Vec3f rgb = cast_ray(Vec3f(0,0,0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
  char r = (char)(255 * max(0.f, min(1.f, rgb[0])));
  char g = (char)(255 * max(0.f, min(1.f, rgb[1])));
  char b = (char)(255 * max(0.f, min(1.f, rgb[2])));

  uint32_t bufferindex = i+j*width;
  
  rgbBuffer[bufferindex+0] = r;
  rgbBuffer[bufferindex+1] = g;
  rgbBuffer[bufferindex+2] = b;
  
  //uint16_t pixelcolor = tft.color565(r, g, b);
  return tft.color565(r, g, b);
/*
  if( rayhitfloor && rayhitsphere ) {
    rayhitsphere = false;
    rayhitfloor = false;
    if(debugray) return TFT_GREEN;
  }
  
  if( rayhitfloor ) {
    rayhitsphere = false;
    if(debugray) return TFT_YELLOW;
  }
  if( rayhitsphere ) {
    rayhitfloor = false;
    if(debugray) return TFT_ORANGE;
  }
  return tft.color565(r, g, b);*/
}



void renderHLine(int32_t x, int32_t y, int32_t l, uint16_t width, uint16_t height, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, float fov) {
  for(uint32_t i=x; i<x+l ;i++) {
      uint16_t pixelcolor = getrenderedpixel(i, y, width, height, spheres, lights, fov);
      imgBuffer[i+y*width] = pixelcolor;
      tft.drawPixel(i, y, pixelcolor);
  }
}

void renderCircle(int32_t x0, int32_t y0, int32_t r, uint16_t width, uint16_t height, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, float fov) {
  int32_t  x  = 0;
  int32_t  dx = 1;
  int32_t  dy = r+r;
  int32_t  p  = -(r>>1);

  renderHLine(x0 - r, y0, dy+1, width, height, spheres, lights, fov);
  while(x<r){

    if(p>=0) {
      dy-=2;
      p-=dy;
      r--;
    }
    dx+=2;
    p+=dx;
    x++;
    renderHLine(x0 - r, y0 + x, 2 * r+1, width, height, spheres, lights, fov);
    renderHLine(x0 - r, y0 - x, 2 * r+1, width, height, spheres, lights, fov);
    renderHLine(x0 - x, y0 + r, 2 * x+1, width, height, spheres, lights, fov);
    renderHLine(x0 - x, y0 - r, 2 * x+1, width, height, spheres, lights, fov);
  }
}


void render(uint16_t posx, uint16_t posy, uint16_t width, uint16_t height, int16_t offsetx, int16_t offsety, uint16_t offsetw, uint16_t offseth, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, float fov=M_PI/2) {
  // yay ! thanks to @atanisoft https://gitter.im/espressif/arduino-esp32?at=5c474edc8ce4bb25b8f1ed95
  uint32_t pos = 0;
  uint32_t pos16 = 0;
  uint16_t jstart = 0, jend = height;
  uint16_t istart = 0, iend = width;

  if( offseth > 0 ) {
    jstart = offsety;
    jend = offsety + offseth;
  }
  if( offsetw > 0 ) {
    istart = offsetx;
    iend = offsetx + offsetw;
  }

  for (size_t j = jstart; j<jend; j++) { // actual rendering loop
    for (size_t i = istart; i<iend; i++) {
      uint16_t pixelcolor = getrenderedpixel(i, j, width, height, spheres, lights, fov);
      if(debugray) {
        if( rayhitsphere ) {
          setBounds(BallCoords2DMin, BallCoords2DMax, i, j);
        }
        if( rayhitfloor ) {
          setBounds(FloorCoords2DMin, FloorCoords2DMax, i, j);
        }
      }
      //imgBuffer[++pos16] = pixelcolor;
      uint32_t bufferindex = i+j*width;
      imgBuffer[bufferindex] = pixelcolor;
      tft.drawPixel(i+posx, j+posy, pixelcolor);
    }
  }
}


void readBufferRect( uint16_t *sourceBuffer, TFT_eSprite &destSprite, uint16_t x, uint16_t y, uint16_t width, uint16_t height ) {
  Serial.printf("ReadRectBuffer( x=%d, y=%d, width=%d, height=%d\n", x, y, width, height);
  //uint16_t *destBuffer = ( uint16_t* )ps_calloc( width*height, sizeof( uint16_t ) );
  uint32_t imgindex = 0;
  destSprite.createSprite( width, height );
  uint16_t *destBuffer = (uint16_t*)sprite.frameBuffer(1);
  for( uint16_t j=y; j<y+height; j++ ) {
    for( uint16_t i=x; i<x+width; i++ ) {
      Serial.printf("%04x ", i, j, sourceBuffer[i+j*width]);
      destBuffer[imgindex] = sourceBuffer[i+j*width];
      imgindex++;
    }
    Serial.println();
  }
  //destSprite.createSprite( width, height );
  destSprite.pushImage(0, 0, width, height, destBuffer);
  //free(destBuffer);
}



bool renderball = true;
bool renderfloor = true;
bool renderbg = true;

void raytracecircle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, float fxpos, float fypos, float fradius, float fov) {
  // reset rgbBuffer
  memset( rgbBuffer, 0, 320*240*3 );
  Material      BallMaterial(1.0, Vec4f(0.8,  0.2, 0.0, 0.0), Vec3f(BallDefaultColor.r, BallDefaultColor.g, BallDefaultColor.b),   50.);
  std::vector<Sphere> spheres;
  spheres.push_back(Sphere(Vec3f(BallCoords.x,                BallCoords.y,                BallCoords.z), BallRadius, BallMaterial));
  std::vector<Light>  lights;
  lights.push_back( Projector1 );
  lights.push_back( Projector2 );
  lights.push_back( Projector3 );
  renderCircle(fxpos, fypos, fradius, width, height, spheres, lights, fov);
}

void raytracebox(uint16_t x, uint16_t y, uint16_t width, uint16_t height, int16_t offsetx=0, int16_t offsety=0, uint16_t offsetw=0, uint16_t offseth=0, float fov=0.5) {
  // reset rgbBuffer
  memset( rgbBuffer, 0, 320*240*3 );
  Material      BallMaterial(1.0, Vec4f(0.8,  0.2, 0.0, 0.0), Vec3f(BallDefaultColor.r, BallDefaultColor.g, BallDefaultColor.b),   50.);
  //Material      glass(1.5, Vec4f(0.0,  0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8),  125.);
  //Material red_rubber(1.0, Vec4f(0.9,  0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1),   10.);
  //Material     mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);
  std::vector<Sphere> spheres;
  if( renderball ) {
    spheres.push_back(Sphere(Vec3f(BallCoords.x,                BallCoords.y,                BallCoords.z), BallRadius, BallMaterial));
  }
  //spheres.push_back(Sphere(Vec3f(BallCoords.x+BallRadius*2, BallCoords.y,                BallCoords.z), BallRadius, BallMaterial));
  //spheres.push_back(Sphere(Vec3f(BallCoords.x,                BallCoords.y+BallRadius*2, BallCoords.z), BallRadius, BallMaterial));
  //spheres.push_back(Sphere(Vec3f(BallCoords.x+BallRadius*2, BallCoords.y+BallRadius*2, BallCoords.z), BallRadius, BallMaterial));
  std::vector<Light>  lights;
  lights.push_back( Projector1 );
  lights.push_back( Projector2 );
  lights.push_back( Projector3 );
  render(x, y, width, height, offsetx, offsety, offsetw, offseth, spheres, lights, fov);
}


extern bool hasPsram;

struct AmigaBallConfig {
  long Framelength = 20;
  byte Wires = 7; // 0 = no wireframes
  uint16_t BGColor = tft.color565(0xa9, 0xa9, 0xa9);
  uint16_t GridColor =  tft.color565(0xac, 0x00, 0xac);
  uint16_t ShadowColor = tft.color565(0x66, 0x66, 0x66);
  uint16_t YPos = 0;
  uint16_t XPos = 0;
  uint16_t Width = tft.width();
  uint16_t Height = tft.height();
  uint16_t ScaleRatio = 5; // ball size will have this/nth of the window Height, bigger value means smaller ball
} amigaBallConfig;


class AmigaRulez {
  public:

    struct Points {
      float x = 0.00;
      float y = 0.00;
    };

    Points points[10][10];

    float deg2rad   = PI/180.0;
    float EIGTH_PI = PI/8.0; // 22.5 deg
    float QUARTER_PI = PI/4.0; // 45 deg
    //float HALF_PI = PI/2.0; // 90 deg
    //float TWO_PI     = PI*2;
    float Phase     = 0.0;
    float velocityX = 2.1;
    float velocityY = 0.07;
    float angleY    = 0.0;
    
    float PhaseVelocity;
    float perspective[4];
    float XtoYratio;
    float YtoXratio;
    float TiltRad;

    bool AnimationDone;
    bool isMovingRight;
    bool isMovingUp = false;
    bool hasPsram = false;
    
    byte Wires;
    byte bytecounter = 0;

    int BounceMargin;

    long Framelength;
    long startedTick = millis();
    long lastTick    = millis();
    long processTicks = 0;

    float variableScale = Scale;
    float oldScale = Scale;
    float ScaleAmplitude = 8;
    float MaxScaleAmplitude;
    float AmplitudeFactor = 4;
    float TiltDeg = 17; // 17 degrees tilting to the right
    float LeftBoundary;
    float RightBoundary;
    float ShadowYPos = 0.00;
    float XPos;
    float YPos;
    float Width;
    float Height;
    
    float VCentering;
    float Scale;
    float YPosAmplitude;
    uint16_t ScaleRatio;
    uint16_t BGColor;
    uint16_t GridColor;
    uint16_t ShadowColor;
    uint16_t *bgData = NULL;
    float lastPositionX;
    float lastPositionY;
    float positionX;
    float positionY;

    int spriteWidth;
    int spriteHeight;
    int spriteCenterX;
    int spriteCenterY;

    void init( AmigaBallConfig config = amigaBallConfig ) {
      BGColor     = config.BGColor;
      GridColor   = config.GridColor;
      Framelength = config.Framelength;//33; // millis
      ScaleRatio  = config.ScaleRatio;
      ShadowColor = config.ShadowColor;
      XPos   = config.XPos;
      YPos   = config.YPos;
      Width  = config.Width;
      Height = config.Height;
      Wires  = config.Wires;
      hasPsram = psramInit();
      setupValues();
      
      tft.fillRect(XPos, YPos, Width, Height, BGColor);

      if( Wires > 0 ) {


        Serial.println("Amigaball init");
      
        shadow.createSprite( spriteWidth / 2, spriteHeight / 8 );
        shadow.fillSprite( BGColor );
        shadow.fillEllipse( shadow.width()/2, shadow.height()/2, shadow.width()/2-4, shadow.height()/2-2, ShadowColor );
        ShadowYPos += shadow.height()/2;

        if( hasPsram ) {

          uint16_t bgImageWidth  = Width;
          uint16_t bgImageHeight = Height - ( ( Height*2 ) / Wires );
          grid.createSprite( Width, Height );
          bgImage.createSprite( Width, Height );

          Serial.println("Created Sprite");

          envmap_width = 1280;
          envmap_height = 640;

          tinyRayTracerInit();

          Serial.println("Raytracer init'd");
          // calculate ball and horizon coords
          float fradius  = project2D(0.5, fabs(BallCoords.z)-1.0, BallRadius);
          float fcenterx = project2D(0.5, fabs(BallCoords.z)-1.0, BallCoords.x);
          float fcentery = project2D(0.5, fabs(BallCoords.z)-1.0, BallCoords.y);
          float checkerhorizon = project2D(0.5, fabs(30.0)-1.0, -4.0); // checkerboard y = -4, 10 < z < 30
          float fxpos = (fcenterx*Height)+Width/2;
          float fypos = (-fcentery*Height)+Height/2;
          // translate to screen
          checkerhorizon = (-checkerhorizon*Height)+Height/2;;
          fradius *= Height;
          // sphere bounds
          float foffx, foffy, fheight, fwidth;
          foffx = fxpos-fradius;
          foffy = fypos-fradius;
          fwidth = 1.0+fradius*2.;

          if( fypos + fradius > checkerhorizon ) {
            fheight = checkerhorizon - foffy;
            /* box h boundings = [ fypos-radius - checkerhorizon ] */
          } else {
            fheight = fwidth;
            /* box h boundings = [ fypos-radius - fypos+radius  ] */
          }
          
          Serial.printf("Sphere centerx (%F): %F, translated: %F\n", BallCoords.x, fcenterx, fxpos );
          Serial.printf("Sphere centery (%F): %F, translated: %F\n", BallCoords.y, fcentery, fypos );
          Serial.printf("Sphere radius: %F, * %F = %F \n", fradius, Height, fradius);
          Serial.printf("Checkboard horizon y : %F\n", checkerhorizon);

/*
          tft.drawCircle(fxpos, fypos, fradius, GREEN);
          tft.drawRect( fxpos-fradius, fypos-fradius, fradius*2.0, fradius*2.0, TFT_PURPLE );
          tft.drawRect( fxpos-fradius, fypos-fradius, fradius*2.0, checkerhorizon-(fypos-fradius), TFT_ORANGE );
          tft.drawRect( 0, checkerhorizon, Width, Height-checkerhorizon, TFT_BLUE  );*/

          sprite.createSprite( envmap_width, envmap_height );
          sprite_drawJpg(0, 0, envmap_1280x640_q4_jpeg, envmap_1280x640_q4_jpeg_len, envmap_width, envmap_height);

          Serial.println("Created BG Sprite");

          bgBuffer = (uint16_t*)sprite.frameBuffer(1);

          renderball = false;
          renderfloor = false;
          renderbg = true;
          raytracebox(XPos, YPos, Width, checkerhorizon);

          TFT_eSprite Sprite_Background = TFT_eSprite(&tft);
          Sprite_Background.createSprite( Width, checkerhorizon );
          Sprite_Background.pushImage(0, 0, Width, checkerhorizon, imgBuffer);

          
          renderball = true;
          renderfloor = true;

          raytracecircle(XPos, YPos, Width, Height, fxpos, fypos, fradius, 0.5);
          TFT_eSprite Sprite_Ball = TFT_eSprite ( &tft );
          readBufferRect( imgBuffer, Sprite_Ball, fxpos-fradius, fypos-fradius, 1.0+fradius*2.0, 1.0+checkerhorizon-(fypos-fradius) );
          
          raytracebox(XPos, YPos, Width, Height, 0, checkerhorizon, Width, Height-checkerhorizon);

          TFT_eSprite Sprite_Floor = TFT_eSprite ( &tft );
          readBufferRect( imgBuffer, Sprite_Floor, 0, checkerhorizon, Width, Height-checkerhorizon );
          
          bgImage.pushImage(0, 0, Width, Height, imgBuffer);

          bgImage.drawCircle( fxpos, fypos, fradius, GREEN );
          bgImage.drawRect( fxpos-fradius, fypos-fradius, 1.0+fradius*2.0, 1.0+fradius*2.0, TFT_PURPLE );
          bgImage.drawRect( fxpos-fradius, fypos-fradius, 1.0+fradius*2.0, 1.0+checkerhorizon-(fypos-fradius), TFT_ORANGE );
          bgImage.drawRect( 0, checkerhorizon, Width, Height-checkerhorizon, TFT_BLUE  );

/*
          struct ballSpriteArray {
            TFT_eSprite sprite =  TFT_eSprite(&tft);
          };

          ballSpriteArray BallSpriteArray[60];
          
          float hwratio = Height / Width;

          float minY = -4 + BallRadius; // -4 = floor y
          float minX = minY / hwratio;
          float maxX = -minX;
          float maxY = -minY;
          float spanY = maxY - minY;
          float spanX = maxX - minX;
          
          Vec2f BounceSpanBoundsMin( minX, minY );
          Vec2f BounceSpanBoundsMax( maxX, maxY );

          for(uint8_t i=0;i<60;i++) {
            BallSpriteArray[i];
          }
          */


          drawGrid(bgImage, XPos, YPos, Width, Height, Width, Height);
          grid.pushImage(0, 0, bgImage.width(), bgImage.height(), (uint16_t*)bgImage.frameBuffer(1) );
          drawGrid(grid, XPos, YPos, Width, Height, Width, Height);
          grid.pushSprite( XPos, YPos );
          grid.deleteSprite();
          
        } else {

          grid.createSprite( Width/2, Height/2 ); // don't overflow heap
          
          grid.fillSprite(BGColor);
          drawGrid(grid, XPos, YPos, Width, Height, grid.width(), grid.height());
          grid.pushSprite( XPos, YPos, BGColor );
    
          grid.fillSprite(BGColor);
          drawGrid(grid, XPos+Width/2, YPos, Width, Height, grid.width(), grid.height());
          grid.pushSprite( XPos+Width/2, YPos, BGColor );
    
          grid.fillSprite(BGColor);
          drawGrid(grid, XPos, YPos+Height/2, Width, Height, grid.width(), grid.height());
          grid.pushSprite( XPos, YPos+Height/2, BGColor );
    
          grid.fillSprite(BGColor);
          drawGrid(grid, XPos+Width/2, YPos+Height/2, Width, Height, grid.width(), grid.height());
          grid.pushSprite( XPos+Width/2, YPos+Height/2, BGColor );
    
          grid.deleteSprite();
        }

      }

      ball.createSprite(spriteWidth, spriteHeight);
      ball.fillSprite(BGColor);

    }


    void setupValues() {
      Scale = Height/ScaleRatio;// 
      ScaleAmplitude = Scale/ AmplitudeFactor; // ball diameter will vary on this
      MaxScaleAmplitude = Scale + ScaleAmplitude;

      spriteWidth  = (MaxScaleAmplitude + AmplitudeFactor )*2;
      spriteHeight = (MaxScaleAmplitude + AmplitudeFactor )*2;

      spriteCenterX = spriteWidth/2 + spriteWidth%2;
      spriteCenterY = spriteHeight/2 - spriteHeight%2;
     
      YPosAmplitude = (Height-(Scale+ScaleAmplitude))/2; // ball will bounce on this span pixels
      VCentering = YPos + (Height-1) - (MaxScaleAmplitude + AmplitudeFactor);// -(YPosAmplitude/2 + Scale + ScaleAmplitude);
      
      BounceMargin = AmplitudeFactor*2+Scale+ScaleAmplitude; // 135
      LeftBoundary = XPos + BounceMargin;
      RightBoundary = XPos + Width - BounceMargin;
     
      TiltRad = TiltDeg * deg2rad;
      lastPositionX = 0;
      lastPositionY = 0;
      PhaseVelocity = 2.5 * deg2rad;
      positionX = XPos + Width/2;
      isMovingRight = true;

      ShadowYPos = YPos + ( (Height / Wires) * (Wires-1) ) /*- 20*/;
      
    }

    float getLat(float phase, int i) {
      if(i == 0) {
        return -HALF_PI;
      } else if(i == 9) {
        return HALF_PI;
      } else {
        return -HALF_PI + phase + (i-1) * EIGTH_PI;
      }
    }

    void calcPoints(float phase) {
      float sin_lat[10] = {0};// = {}
      for(int i=0;i<10;i++) {
        float lat = getLat(phase, i);
        sin_lat[i] = sin( lat );
      }
      for(int j=0;j<9;j++) {
        float lon = -HALF_PI + j * EIGTH_PI;
        float _y = sin( lon );
        float _l = cos( lon );
        for(int i=0;i<10;i++) {
          float _x = sin_lat[i] * _l;
          points[i][j].x = _x;
          points[i][j].y = _y;
        }
      }
    }

    void tiltSphere(float ang) {
      float st = sin( ang );
      float ct = cos( ang );
      for( int i=0; i<10; i++) {
        for( int j=0; j<9; j++) {
          float _x = points[i][j].x * ct - points[i][j].y * st;
          float _y = points[i][j].x * st + points[i][j].y * ct;
          points[i][j].x = _x;
          points[i][j].y = _y;
        }
      }
    }

    float scaleTranslate(float s, float tx, float ty) {
      for( int i=0; i<10; i++) {
        for( int j=0; j<9; j++ ) {
          float _x = points[i][j].x * s + tx;
          float _y = points[i][j].y * s + ty;
          points[i][j].x = _x;
          points[i][j].y = _y;
        }
      }
    }

    void transform(float s, float tx, float ty) {
      tiltSphere( TiltRad );
      scaleTranslate( s, tx, ty );
    }

    void fillTiles(bool alter) {
      for( int j=0; j<8; j++ ) {
        for( int i=0; i<9; i++) {
          uint16_t color = alter ? RED : WHITE;
          ball.fillTriangle(points[i][j].x,     points[i][j].y,     points[i+1][j].x, points[i+1][j].y, points[i+1][j+1].x, points[i+1][j+1].y, color);
          ball.fillTriangle(points[i+1][j+1].x, points[i+1][j+1].y, points[i][j+1].x, points[i][j+1].y, points[i][j].x,     points[i][j].y, color);
          alter = !alter;
        }
      }
    }

    void drawGrid(TFT_eSprite sprite, int x, int y, int width, int height, int spanX, int spanY) {

      int center = width / 2;
      int vspace = ( height*2 ) / Wires;
      int vpos   = height - vspace - y + YPos;
      float stepX  = width / Wires;
      float stepY  = height / Wires;
      int i, x2, centerdiff;

      if( hasPsram ) {
          sprite.pushImage(-x+XPos, -y+YPos, bgImage.width(), bgImage.height(), (uint16_t*)bgImage.frameBuffer(0));
      } else {
        for (uint16_t nPosY = 0; nPosY < height-vspace; nPosY++) {
          uint16_t nColTmp = tft.color565(nPosY % 256, 255 - (nPosY % 256), 0 );
          //sprite.drawFastHLine(0, nPosY-y, YPos+spanX, nColTmp);
          for(uint16_t nPosX = 0; nPosX < width; nPosX+=stepX/4) {
            nColTmp = tft.color565(nPosY % 256, 255 - (nPosY % 256), 255- (nPosX/2) % 256 );
            sprite.drawFastHLine(nPosX-x+XPos, nPosY-y+YPos, stepX, nColTmp);
          }
        }

        sprite.fillRect(0, vpos, width, vspace, BGColor);

        for( i=0; i<width; i+=stepX ) {
          centerdiff = abs(center - i);
          if(i==center) {
            x2 = i;
          } else {
            if( i < center ) {
              x2 = i - centerdiff*2;
            } else { // i > center
              x2 = i + centerdiff*2;
            }
          }
          sprite.drawFastVLine(XPos+i-x, 0, vpos, GridColor);
          sprite.drawLine(XPos+i-x, vpos, XPos+x2-x, YPos+vpos+vspace, GridColor);
        }
        
        for( i=0; i<height-vspace+YPos; i+=stepY ) {
          sprite.drawFastHLine(0, i-y, YPos+spanX, GridColor);
        }
        float powa = 32;
        while(powa>1) {
          powa /=2;
          sprite.drawFastHLine(0, vpos+(vspace/powa), YPos+spanX, GridColor);  
        }
      }
    }


    void drawBall(float phase, float scale, float oldscale, float x, float y) {
      calcPoints( fmod(phase, EIGTH_PI) );
      transform(scale, x, y);
      fillTiles(phase >= EIGTH_PI);
    }


    void animate( long duration = 5000, bool clearAfter = true ) {

      AnimationDone = false;
      startedTick = millis();
      lastTick = millis();
      processTicks = 0;

      while( !AnimationDone ) {
        lastTick = millis();
        if( isMovingRight ) {
          Phase = fmod( Phase + ( QUARTER_PI - PhaseVelocity ), QUARTER_PI );
          positionX += velocityX;
        } else {
          Phase = fmod( Phase + PhaseVelocity, QUARTER_PI );
          positionX -= velocityX;
        }
        if ( positionX >= RightBoundary ) {
          isMovingRight = false;
          buzz_wall = true;
        } else if( positionX < LeftBoundary ) {
          isMovingRight = true;
          buzz_wall = true;
        }
        angleY = fmod( angleY + velocityY, TWO_PI );
        float absCosAngleY = fabs( cos( angleY ) );
        variableScale = Scale + ScaleAmplitude * absCosAngleY;
        positionY = VCentering - YPosAmplitude * absCosAngleY;

        int trend = positionY - lastPositionY;
        
        if( !isMovingUp && trend < 0) {
          buzz_floor = true;
        }

        if( positionY < lastPositionY ) {
          isMovingUp = true;
        } else {
          isMovingUp = false;
        }
        
        ball.fillSprite(BGColor); // Note: Sprite is filled with black when created
        if( Wires > 0 ) {
          if( hasPsram ) {
            shadow.fillSprite( TFT_TRANSPARENT );
          } else {
            shadow.fillSprite( BGColor );            
          }
          int r1 = (shadow.width()/2-4)* (1-.5*absCosAngleY);
          int r2 = (shadow.height()/2-2)* (1-.5*absCosAngleY);
          drawGrid( shadow, positionX-spriteCenterX + shadow.width()/2, ShadowYPos, Width, Height, Width,       Height );
          shadow.fillEllipse( shadow.width()/2, shadow.height()/2, r1, r2, ShadowColor );
          drawGrid( ball,   positionX-spriteCenterX,                    positionY-spriteCenterY,    Width, Height, spriteWidth, spriteHeight );
          shadow.pushSprite( positionX-spriteCenterX + shadow.width()/2, ShadowYPos );
        }
        drawBall( Phase, variableScale, oldScale, spriteCenterX, spriteCenterY );
        ball.pushSprite( positionX-spriteCenterX, positionY-spriteCenterY, TFT_TRANSPARENT );

        oldScale = variableScale;
        lastPositionX = positionX;
        lastPositionY = positionY;
        processTicks = millis() - lastTick;
        if( processTicks < Framelength ) {
          delay( Framelength - processTicks );
        }
        if( millis() - startedTick > duration ) {
          if( clearAfter ) {
            tft.fillRect( XPos, YPos, Width, Height, BGColor );
          }
          AnimationDone = true;
        }
      }
    }

};


AmigaRulez AmigaBall;
