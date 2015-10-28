// Sensorik WS15 - HAW-Luftschiff
// Team 11: Beyerstedt, Schmid, Friedrich

#include "sharp.h"


// --- PINS ---
#define US_TRIG   8  // an HC-SR04 Trig
#define US_ECHO   7  // an HC-SR04 Echo

#define IR1_VAL   A0
#define IR2_VAL   A1

#define MOTS_EN   12
#define MOT1_A    5
#define MOT1_B    6
#define MOT2_A    9
#define MOT2_B    10
#define MOT3_A    11
#define MOT3_B    13

#define IMU_SCL   2
#define IMU_SDA   3
#define STATUS_LED 4


// --- OPTIONEN ---
//#define DEBUG_IR


// --- VARIABLEN ---
unsigned long duration=0;
long distance=0;

Sharp IR_1 = Sharp(IR1_VAL);
Sharp IR_2 = Sharp(IR2_VAL);


void setup(void) {
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  pinMode(IR1_VAL, INPUT);
  pinMode(IR2_VAL, INPUT);

  pinMode(MOT1_A, OUTPUT);
  pinMode(MOT1_B, OUTPUT);
  pinMode(MOTS_EN, OUTPUT);
  digitalWrite(MOTS_EN, HIGH);
  pinMode(MOT2_A, OUTPUT);
  pinMode(MOT2_B, OUTPUT);
  pinMode(MOT3_A, OUTPUT);
  pinMode(MOT3_B, OUTPUT);

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
  duration = pulseIn(US_ECHO, HIGH, 25000); // US_ECHO-Zeit messen
  
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
  
  int ir2_dist    = IR_2.get_distance();
  Serial.print("\t");Serial.print("IR2 "); Serial.print(ir2_dist);

 
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
   * Motor Test
   */
  analogWrite(MOT2_A, 125);
  digitalWrite(MOT2_B, LOW);

  digitalWrite(MOT3_A, LOW);
  analogWrite(MOT3_B, 125);
   
  

  /*
   * ende
   */
  Serial.print("\n");
  delay(100);
}

