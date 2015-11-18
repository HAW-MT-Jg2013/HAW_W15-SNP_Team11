/*
 * Klasse für den Sharp GP2Y0A02
 * 
 * Abstandsermittelung durch Wertetabelle und abschnittsweise lineare Interpolation
 */
 
#include "sharp_IR.h"


SharpIR::SharpIR(int pin) {
  this->pin = pin;
}

unsigned int SharpIR::get_distance() {

  float volts = analogRead(pin)*5/1024.0;

  int key_l = volts * 2; // rechte Ecke der Linearisierung
  int key_r = key_l - 1; // linke  Ecke der Linearisierung

  int dist_l, dist_r;

  if (key_l < 0) {
    dist_l = 300;
  }else if (key_l > 4) {
    dist_l = 10;
  }else {
    dist_l = dist[key_l];
  }
  
  if (key_r < 0) {
    dist_r = 300;
  }else if (key_r > 4) {
    dist_r = 10;
  }else {
    dist_r = dist[key_r];
  }

#ifdef DEBUG_IR
  Serial.print("\t\t"); Serial.print(volts);Serial.print("V");
  Serial.print("\t B:"); Serial.print(dist_r);
  Serial.print(" -\t"); Serial.print(dist_l);
#endif 

  return dist_r -  (dist_r-dist_l) * (volts-key_l/2.0)/0.5;
}

