#include "regmap.h"
#include "neopixel.h"


Adafruit_NeoPixel_ZeroDMA pixels(NUM_PIXELS_MAX+1, PIN_NEOPIXEL, NEO_GRB);

void setupPixels(){
    pixels.begin();
    pixels.setBrightness(*pixelBrightness);
}



void updatePixels(){
  pixels.setBrightness(*pixelBrightness);
  for (int i=0; i<*numPixels; i++){
    pixels.setPixelColor(i+1, pixelColors[i]); //note that we shift i by 1, leaving index 0 for internal neopixel
  }
  pixels.show();
}

void updateIntPixel(uint32_t color){
  pixels.setPixelColor(0,color);
  pixels.show();
}
