/*
 * Klasse für den Sharp GP2Y0A02
 * 
 * Abstandsermittelung durch Wertetabelle und abschnittsweise lineare Interpolation
 */

#include <arduino.h>

#ifndef SHARP_H
#define SHARP_H

//#define DEBUG_IR


// Wertetabelle: cm - V
static unsigned int dist[] = {130, // 0,5V
                              63,  // 1,0V
                              40,  // 1,5V
                              30,  // 2,0V
                              21}; // 2,5V

class Sharp {
  unsigned int pin;

  public:
  Sharp(int pin);
  unsigned int get_distance();
};


#endif

