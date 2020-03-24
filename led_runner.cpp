// Rainbow runner class
// Igor Kosulin
//===============================================
#include <ArduinoLog.h>
#include "led_runner.h"

#define ANGLE          60.0
#define HOLD_DELAY     10000
// ================================================================
// ===                TESTING parameters                        ===
// ================================================================
#ifdef DEBUG
int number_of_led_runs = 0;
#endif
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
LedRunner::LedRunner(int pix, int pin) :
  NUMPIXELS(pix),
  PIXELPIN(pin),
  pixels(Adafruit_NeoPixel(pix, pin, NEO_GRB  + NEO_KHZ800)),
  lights(RainbowColorTable(pix)),
  ANGLE_STEP(float(ANGLE / pix)),
  ANGLE_NOISE(ANGLE_STEP),
  STATIC_SPPED(int(5000 / pix))
{
  pixels.begin();
  pixels.show(); // Initialize all pixels to 'off'
}

bool LedRunner::isStatic(float x) {
  unsigned long time = millis();
  //If new angle is in Noise range start calculating hold
  if ( abs(last_angle - x) <= ANGLE_NOISE ) {
    //Log.verbose(F("LED: noise %F - %F " CR), last_angle, x);
    if (!static_run && ( time - hold > HOLD_DELAY )) {
      Log.verbose(F("LED: start static run" CR));
      static_run = true;
      static_angle = last_angle;
    }
  } else {
    Log.verbose(F("LED: stop static run" CR));
    static_run = false;
    hold = time;
  }
  
  return static_run;
}

bool LedRunner::isNeedUpdate(float x) {
  if (!isStatic(x)) {
    //Log.verbose(F("LED: need update" CR));
    return true;
  }

  unsigned long now = millis();
  if (now - last_run > STATIC_SPPED) {
    //Log.verbose(F("LED: need update static run" CR));
    last_run = now;
    return true;
  }
  return false;
}

float LedRunner::angleToShow(float angle) {
  if (static_run) {
    angle = static_angle;
    if ( up ) {
      angle += ANGLE_STEP;
      if ( angle > ANGLE) {
       // Log.verbose(F("LED:  angle down" CR));
        angle = ANGLE;
        up = false;
      }
    } else {
      angle -= ANGLE_STEP;
      if ( angle < 0 ) {
        //Log.verbose(F("LED:  angle up" CR));
        angle = 0;
        up = true;
      }
    }
    static_angle = angle;
  } else {
    last_angle = angle;
  }
  
  return angle;
}

bool LedRunner::update_leds(float x) {
  //X Angle is in range -(ANGLE/2) to (ANGLE/2)
  //convert to 0 - ANGLE range
  float angle = x + ANGLE / 2;
  if( angle > ANGLE ) angle = ANGLE;
  if( angle < 0 ) angle = 0;
  if (isNeedUpdate(angle)) {
    update_led_rainbow(angleToShow(angle));
    return true;
  }
  return false;
}

void LedRunner::update_led_rainbow(float angle) {
  pixels.clear();

  int red_shift = (int) lround(angle / ANGLE_STEP);
  Log.plotter(F(" %F "), angle);

  for (int i = 0; i < NUMPIXELS; i++) {
    int red = red_shift + i;
    pixels.setPixelColor(i, lights[red]);//
  }
  // Turn the device on
  pixels.show();
}
