/*
 * Copyright 2020 Igor Kosulin.
 * All rights reserved.
 */


#ifndef __INC_RAINBOW_COLOR_TABLE_H
#define __INC_RAINBOW_COLOR_TABLE_H

/*
 * Rainbow LED Color table 
 */
class RainbowColorTable {

  public:
    RainbowColorTable(int pix);

    uint32_t operator[](const int n) const;
  private:
    int pixels;      //Number of pixels on table
    uint8_t* lights; //COS based color table

    int SHIFT_GREEN; //Light table Green color shift
    int SHIFT_BLUE; //Light table Blue color shift

    uint32_t led_rainbow_color(int base, bool log = false);

};

#endif
