#ifndef __INC_LED_RUNNER_H
#define __INC_LED_RUNNER_H

#include <Adafruit_NeoPixel.h>  // NeoPixel library from Adafruit
#include "RainbowColorTable.h"

class LedRunner {

  public:
    LedRunner(int pixels, int pin);

    bool update_leds(float x);

  private:
    int PIXELPIN;  // Arduino pin connected to strip
    int NUMPIXELS; // Total number of RGB LEDs on strip

    float ANGLE_STEP; //Angle step
    float ANGLE_NOISE; //Angle Noise for skip colors update
    int STATIC_SPPED; //Rainbow run speed

    bool static_run = false;
    float static_angle = 0;
    
    float last_angle = 0;
    unsigned long last_run = 0;


    float last_static_angle = 0;
    unsigned long hold = 0;
    bool up = true;

    // When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
    Adafruit_NeoPixel pixels;

    //Rainbow color table
    RainbowColorTable lights;

    bool isStatic(float x);
    bool isNeedUpdate(float x);
    float angleToShow(float angle);
    void update_led_rainbow(float x);
};

#endif
