#include "regmap.h"
#include "neopixel.h"
volatile uint32_t pixelColorArray[NUM_PIXELS_MAX+1];

Adafruit_NeoPixel_ZeroDMA pixels(NUM_PIXELS_MAX+1, PIN_NEOPIXEL, NEO_GRB);

void pixelBegin(){
    pixels.begin();
    *numPixels=3;
    pixels.setBrightness(*pixelBrightness);
}

void pixelUpdateConfig(){
  pixels.setBrightness(*pixelBrightness);
}

void pixelShow(){
  for (int i=1; i<=*numPixels; i++){
    pixels.setPixelColor(i, pixelColorArray[i]); //note that we start with  1, leaving index 0 for internal neopixel
  }
  pixels.show();
}

void intPixelUpdate(uint32_t color){
  pixels.setPixelColor(0,color);
  pixels.show();
}
