#ifndef _ROVERWING_NEOPIXEL_H
#define _ROVERWING_NEOPIXEL_H

#include <Arduino.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>
#define NUM_PIXELS_MAX  25
#define RED    0x00FF0000
#define GREEN  0x0000FF00
#define BLUE   0x000000FF
#define YELLOW 0x00FFFF00
#define OFF    0x00000000
#define RGBcolor(R,G,B) ((R<<16)|(G<<8)|B)
void setupPixels();
void updatePixels();
void updateIntPixel(uint32_t c);
#endif
