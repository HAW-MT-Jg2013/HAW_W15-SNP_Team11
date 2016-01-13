/*
 * Klasse für den HC-SR04 Ultraschallsensor
 *
 * Abstandsermittelung durch Schallgeschwindigkeit
 */

#include "hc-sr04.h"


HC_SR04::HC_SR04(int trigger, int echo) {
  this->trigger = trigger;
  this->echo    = echo;

  mean_iterator = 0;
  mean_sum = 0;
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
  int distance = duration / 29.1;

  // mean
  mean_sum -= mean_arr[mean_iterator];
  mean_arr[mean_iterator] = distance;
  mean_sum += mean_arr[mean_iterator];
  mean_iterator++;
  if (mean_iterator >= 20) {
    mean_iterator = 0;
  }
  distance = mean_sum / 20.0;

  if (distance >= 200 || distance <= 0) {
    return 0;
  } else {
    return distance;
  }

}

