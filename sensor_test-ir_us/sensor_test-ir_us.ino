// Sensorik WS15 - HAW-Luftschiff
// Team 11: Beyerstedt, Schmid, Friedrich

#include "sharp.h"


// --- PINS ---
#define US_TRIG   7  // an HC-SR04 Trig
#define US_ECHO   6  // an HC-SR04 Echo

#define IR1_VAL   A2

#define MOT1_A    5
#define MOT1_B    9
#define MOTS_EN   12
#define MOT2_A  
#define MOT2_B  


// --- OPTIONEN ---
//#define DEBUG_IR


// --- VARIABLEN ---
long duration=0;
long distance=0;

Sharp IR_1 = Sharp(IR1_VAL);


void setup(void) {
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  pinMode(IR1_VAL, INPUT);

  pinMode(MOT1_A, OUTPUT);
  pinMode(MOT1_B, OUTPUT);
  pinMode(MOTS_EN, OUTPUT);
  digitalWrite(MOTS_EN, HIGH);
  //pinMode(MOT2_A, OUTPUT);
  //pinMode(MOT2_B, OUTPUT);

  Serial.begin(9600);
}


void loop() {
  /*
   * Ultraschall
   */
  digitalWrite(US_TRIG, LOW);  
  delayMicroseconds(5); 
 
  digitalWrite(US_TRIG, HIGH);  
  delayMicroseconds(10);
  
  digitalWrite(US_TRIG, LOW);
  duration = pulseIn(US_ECHO, HIGH, 15000); // US_ECHO-Zeit messen
  
  // US_ECHO-Zeit halbieren (weil hin und zurück, der doppelte Weg ist)
  duration = (duration/2); 
  // Zeit des Schalls durch Luft in Zentimeter umrechnen
  distance = duration / 29.1;

  Serial.print("US "); Serial.print(distance);

  delay(20);


  /*
   * SHARP
   */
  int ir_distance = IR_1.get_distance();
  Serial.print("\t");Serial.print("IR1 "); Serial.print(ir_distance);

 
  /*
   * Höhenregelung
   */
  int power = 50-ir_distance;
  if (power < 0) {
    power = -power;
    power = power*4;
    if (power > 255) {
      power = 255;
    }
    analogWrite(MOT1_A, power);
    digitalWrite(MOT1_B, LOW);
  }else {
    power = power*4;
    if (power > 255) {
      power = 255;
    }
    analogWrite(MOT1_B, power);
    digitalWrite(MOT1_A, LOW);
  }


  /*
   * ende
   */
  Serial.print("\n");
  delay(100);
}

