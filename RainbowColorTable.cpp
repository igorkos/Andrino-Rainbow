/*
 * Copyright 2020 Igor Kosulin.
 * All rights reserved.
 */
 
#include <Adafruit_NeoPixel.h>  // NeoPixel library from Adafruit
#include <ArduinoLog.h>

#include "RainbowColorTable.h"

RainbowColorTable::RainbowColorTable(int pix) :
  pixels(pix),
  lights(new uint8_t[pixels + 1]) {

  Log.trace(F("Initializing Rainbow table..." CR));
  //Calculate red color cos based table
  int wave = int(pixels / 3);
  for (int k = 0; k < wave; k++)
  {
    float a = k * (PI / wave);
    float cv = (cos( a ) + 1) / 2;
    lights[k] = int(cv * 255);
    lights[pixels - k] = int(cv * 255);
  }

  for (int k = wave; k < pixels - wave; k++)
  {
    lights[k] = 0;
  }

  Log.verbose(F(CR "LED table:" CR));
  for (int k = 0; k < pixels; k++)
  {
    Log.plotter(F("[%d:%x],"), k, lights[k]);
  }
  Log.verbose(F(CR));

  SHIFT_GREEN = int(2 * pixels / 3);
  SHIFT_BLUE = int(pixels / 3);
}

uint32_t RainbowColorTable::operator[](const int n) const {
  return led_rainbow_color(n, false);
}

uint32_t RainbowColorTable::led_rainbow_color(int base, bool log) {
  int red = base;
  if (red >= pixels) {
    red = red - pixels;
  }
  int green = red + SHIFT_GREEN;
  if (green >= pixels) {
    green = green - pixels;
  }
  int blue = red + SHIFT_BLUE;
  if (blue >= pixels) {
    blue = blue - pixels;
  }
  if ( log ) {
    Log.plotter(F(" %d"), red);
    Log.plotter(F(" %d"), lights[red]);
    Log.plotter(F(" %d"), green);
    Log.plotter(F(" %d"), lights[green]);
    Log.plotter(F(" %d"), blue);
    Log.plotter(F(" %d "), lights[blue]);
    Log.plotter(F(CR));
  }
  return Adafruit_NeoPixel::Color(lights[red], lights[green], lights[blue]);
}
