/*
 * Klasse für den Sharp GP2Y0A02
 *
 * Abstandsermittelung durch Wertetabelle und abschnittsweise lineare Interpolation
 */

#include <arduino.h>

#ifndef SHARP_IR_H
#define SHARP_IR_H

// Wertetabelle: cm - V
static unsigned int dist[] = {130, // 0,5V
                              63,  // 1,0V
                              40,  // 1,5V
                              30,  // 2,0V
                              21   // 2,5V
                             }; 

class SharpIR {
    unsigned int pin;

  public:
    SharpIR(int pin);
    unsigned int get_distance();
};


#endif

