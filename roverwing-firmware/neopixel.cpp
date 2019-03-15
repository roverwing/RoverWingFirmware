#include "regmap.h"
#include "neopixel.h"
volatile uint32_t pixelColorArray[NUM_PIXELS_MAX+1];

Adafruit_NeoPixel_ZeroDMA pixels(NUM_PIXELS_MAX+1, PIN_NEOPIXEL, NEO_GRB);

void setupPixels(){
    pixels.begin();
    pixels.setBrightness(*pixelBrightness);
}



void updatePixels(){
  pixels.setBrightness(*pixelBrightness);
  for (int i=1; i<=*numPixels; i++){
    pixels.setPixelColor(i, pixelColorArray[i]); //note that we start with  1, leaving index 0 for internal neopixel
  }
  pixels.show();
}

void updateIntPixel(uint32_t color){
  pixels.setPixelColor(0,color);
  pixels.show();
}
