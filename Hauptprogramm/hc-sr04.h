/*
 * Klasse für den HC-SR04 Ultraschallsensor
 *
 * Abstandsermittelung durch Schallgeschwindigkeit
 */

#include <arduino.h>

#ifndef HC_SR04_H
#define HC_SR04_H


class HC_SR04 {
    unsigned int trigger;
    unsigned int echo;

    unsigned int mean_arr[20];
    int mean_iterator;
    unsigned int mean_sum;

  public:
    HC_SR04(int trigger, int echo);
    unsigned int get_distance();
};


#endif

