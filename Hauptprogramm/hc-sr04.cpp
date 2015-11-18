/*
 * Klasse für den HC-SR04 Ultraschallsensor
 * 
 * Abstandsermittelung durch Schallgeschwindigkeit
 */
 
#include "hc-sr04.h"


HC_SR04::HC_SR04(int trigger, int echo) {
  this->trigger = trigger;
  this->echo    = echo;
}

unsigned int HC_SR04::get_distance() {
  // send pulse
  digitalWrite(trigger, LOW);
  delayMicroseconds(5);

  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigger, LOW);

  // get echo pulse lenght
  unsigned long duration = pulseIn(echo, HIGH, 25000);

  // calculate distance
  duration /= 2;
  long distance = duration / 29.1;

  if (distance >= 200 || distance <= 0) {
    return 0;
  } else {
    return distance;
  } 

}

