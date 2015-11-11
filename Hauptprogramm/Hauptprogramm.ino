// Sensorik WS15 - HAW-Luftschiff
// Team 11: Beyerstedt, Schmid, Friedrich

#include <L3G.h>
#include <LSM303.h>
#include <Wire.h>
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


//--- OPTIONEN ---

// IMU
#define OUTPUTMODE_C  //for corrected data, comment for uncorrectet data of gyro (with drift)

//#define PRINT_DCM      //Will print the whole direction cosine matrix
//#define PRINT_ANALOGS  //Will print the analog raw data
//#define PRINT_EULER    //Will print the Euler angles Roll, Pitch and Yaw

// Allgemein
//#define DEBUG_IR
#define DEBUG_IMU

#include "IMU_lib.h"


// --- GLOBALE VARIABLEN ---
unsigned int us_distance = 0;
unsigned int ir1_distance = 0;
unsigned int ir2_distance = 0;

Sharp IR_1 = Sharp(IR1_VAL);
Sharp IR_2 = Sharp(IR2_VAL);

enum Status {START = 1, GERADEAUS, TREPPE, BARRIKADE, LANDUNG};
static enum Status S = START;


void setup() {
  pinMode(MOT1_A, OUTPUT);
  pinMode(MOT1_B, OUTPUT);
  pinMode(MOT2_A, OUTPUT);
  pinMode(MOT2_B, OUTPUT);
  pinMode(MOT3_A, OUTPUT);
  pinMode(MOT3_B, OUTPUT);
  pinMode(MOTS_EN, OUTPUT);
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  pinMode(IR1_VAL, INPUT);
  pinMode(IR2_VAL, INPUT);
  pinMode(STATUS_LED, OUTPUT);

  digitalWrite(MOTS_EN, HIGH);

  Serial.begin(9600);

  I2C_Init();
  digitalWrite(STATUS_LED, LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for (int i = 0 ; i < 32 ; i++) { // We take some readings... /////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!HIER LÄUFT WAS FALSCH!!!!!!!!!!!!!! //TODO
    Read_Gyro();
    Read_Accel();
    for (int y = 0; y < 6; y++) // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
  }

  for (int y = 0; y < 6; y++) {
    AN_OFFSET[y] = AN_OFFSET[y] / 32;
  }

  AN_OFFSET[5] -= GRAVITY * SENSOR_SIGN[5];

  //Serial.println("Offset:");
  for (int y = 0; y < 6; y++) {
    Serial.println(AN_OFFSET[y]);
  }

  delay(2000);
  digitalWrite(STATUS_LED, HIGH);

  timer = millis();
  delay(20);
  counter = 0;
}

void Hoehenregelung(unsigned int distance, int offset, int Kp) {
  int power = offset - distance;
  if (power < 0) {
    power = -power;
    power = power * Kp;
    if (power > 255) {
      power = 255;
    }
    analogWrite(MOT1_A, power);
    digitalWrite(MOT1_B, LOW);
  } else {
    power = power * Kp;
    if (power > 255) {
      power = 255;
    }
    analogWrite(MOT1_B, power);
    digitalWrite(MOT1_A, LOW);
  }
}

void IMU_Zeug() {
  if ((millis() - timer) >= 20) { // IMU runs at 50Hz
    counter++;
    timer_old = timer;
    timer = millis();
    if (timer > timer_old)
      G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;

    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer

    if (counter > 5) { // Read compass data at 10Hz... (5 loop runs)
      counter = 0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }

    // Calculations...
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***
#ifdef DEBUG_IMU
    printdata();
#endif
  }
}

unsigned int US_read_and_print() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(5);

  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);

  digitalWrite(US_TRIG, LOW);
  unsigned long duration = pulseIn(US_ECHO, HIGH, 25000); // US_ECHO-Zeit messen

  // US_ECHO-Zeit halbieren (weil hin und zurÃ¼ck, der doppelte Weg ist)
  duration /= 2;
  // Zeit des Schalls durch Luft in Zentimeter umrechnen
  long distance = duration / 29.1;

  Serial.print("US "); Serial.print(distance);
  if (distance >= 200 || distance <= 0) {
    Serial.print("\t Out of range");
  }

  return distance;
}


void loop() {

  //IMU_Zeug();
  us_distance = US_read_and_print();
  ir1_distance = IR_1.get_distance();
  ir2_distance    = IR_2.get_distance();
  Serial.print("\t"); Serial.print("IR1 "); Serial.print(ir1_distance);
  Serial.print("\t"); Serial.print("IR2 "); Serial.println(ir2_distance);
  Hoehenregelung(us_distance, 50, 4);

  switch (S) {
    case START:
      {

        break;
      }
    case GERADEAUS:
      {
        //Motor volle Pulle an
        break;
      }
    case TREPPE:
      {

        break;
      }
    case BARRIKADE:
      {

        break;
      }
    case LANDUNG:
      {

        break;
      }
    default:
      {

        break;
      }
  }
}


