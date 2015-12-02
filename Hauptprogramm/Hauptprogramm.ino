// Sensorik WS15 - HAW-Luftschiff
// Team 11: Beyerstedt, Schmid, Friedrich

#include <L3G.h>
#include <LSM303.h>
#include <Wire.h>
#include "sharp_IR.h"
#include "hc-sr04.h"


// --- PINS ---
#define US_TRIG   8  // an HC-SR04 Trig
#define US_ECHO   7  // an HC-SR04 Echo

#define IR1_VAL   A0
#define IR2_VAL   A1

#define MOTS_EN   12
#define MOT1_A    5   // links
#define MOT1_B    6
#define MOT2_A    9   // rechts
#define MOT2_B    10
#define MOT3_A    11  // unten
#define MOT3_B    13

#define IMU_SCL   2
#define IMU_SDA   3
#define STATUS_LED 4


//--- OPTIONEN ---

// IMU
#define OUTPUTMODE_C  //for corrected data, comment for uncorrectet data of gyro (with drift)

//#define PRINT_DCM      //Will print the whole direction cosine matrix
//#define PRINT_ANALOGS  //Will print the analog raw data
#define PRINT_EULER    //Will print the Euler angles Roll, Pitch and Yaw

// Allgemein
//#define DEBUG_IR
//#define DEBUG_IMU
#define DEBUG_SENSORS

#include "IMU_lib.h"


// --- OBJEKTE ---
SharpIR IR_V = SharpIR(IR1_VAL);
SharpIR IR_U = SharpIR(IR2_VAL);
HC_SR04 US_L = HC_SR04(US_TRIG, US_ECHO);


// --- GLOBALE VARIABLEN ---
unsigned int abst_links = 0;
unsigned int abst_vorne = 0;
unsigned int hoehe = 0;
int drehung = 0;

typedef enum main_st {START = 1, GERADEAUS, TREPPE, BARRIKADE, ABSTIEG, LANDUNG};
main_st abschnitt = START;
typedef enum treppe_st {GERADE1, KURVE1, GERADE2, KURVE2, GERADE3};
treppe_st treppe_abschnitt = GERADE1;


// I-Regler
unsigned long I_lastTime = millis();
int I_interval = 100;
long I_sum = 0;
#define I_MAX 1000
#define K_I 10
#define K_P 2


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
  while (!Serial);

  I2C_Init();
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();
  delay(20);

  for (int i = 0 ; i < 32 ; i++) { // We take some readings...
    Read_Gyro();
    Read_Accel();
    for (int y = 0; y < 6; y++) { // Cumulate values
      AN_OFFSET[y] += AN[y];
    }
    delay(20);
  }
  for (int y = 0; y < 6; y++) {
    AN_OFFSET[y] = AN_OFFSET[y] / 32;
  }
  AN_OFFSET[5] -= GRAVITY * SENSOR_SIGN[5];

  Serial.print("Offset:");
  for (int y = 0; y < 6; y++) {
    Serial.print(AN_OFFSET[y]); Serial.print("\t");
  }
  Serial.print("\n");

  delay(2000);

  timer = millis();
  delay(20);
  counter = 0;
}

void Motor(int power, int motNr) {
  int pinA, pinB;

  if (motNr == 1) {
    pinA = MOT1_A;
    pinB = MOT1_B;
  } else if (motNr == 2) {
    pinA = MOT2_A;
    pinB = MOT2_B;
  } else if (motNr == 3) {
    pinA = MOT3_A;
    pinB = MOT3_B;
  }
  
  if (power < 0) {
    power = -power;
    if (power > 255) {
      power = 255;
    }
    analogWrite(pinA, power);
    digitalWrite(pinB, LOW);
  } else {
    if (power > 255) {
      power = 255;
    }
    analogWrite(pinB, power);
    digitalWrite(pinA, LOW);
  }
}

void Hoehenregelung(unsigned int distance, int offset) {
  int diff = offset - distance;
  
  if ( (millis() - I_lastTime) > I_interval ) {
    I_lastTime = millis();

    I_sum += diff;
    if(I_sum > I_MAX) {
      I_sum = I_MAX;
    }else if (I_sum < -I_MAX) {
      I_sum = -I_MAX;
    }
  }

  int power = (diff * K_P) + (K_I * I_sum/I_interval);

  Motor(power, 3);
}



void IMU_Berechnungen() {
  if ((millis() - timer) >= 20) { // IMU runs at 50Hz
    counter++;
    timer_old = timer;
    timer = millis();
    if (timer > timer_old) {
      G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    } else {
      G_Dt = 0;
    }

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


void loop() {

  IMU_Berechnungen();

  hoehe = IR_U.get_distance();
  abst_links = US_L.get_distance();
  abst_vorne = IR_V.get_distance();

#ifdef DEBUG_SENSORS
  Serial.print("\t"); Serial.print("US1 "); Serial.print(abst_links);
  Serial.print("\t"); Serial.print("IR1 "); Serial.print(abst_vorne);
  Serial.print("\t"); Serial.print("IR2 "); Serial.println(hoehe);
#endif
  


  switch (abschnitt) {
    case START:
      {
        // Höhe voll an

        if (hoehe > 80) {
          abschnitt = GERADEAUS;
        }
        break;
      }
    case GERADEAUS:
      {
        // Höhenregelung an
        Hoehenregelung(hoehe, 100);
        //analogWrite(MOT1_A, 50);
        //digitalWrite(MOT1_B, LOW);
        

        // vorwärts

        if (0) { // if Treppe erkannt
          abschnitt = TREPPE;
        }
        break;
      }
    case TREPPE:
      {
        // Höhenregelung an
        Hoehenregelung(hoehe, 100);

        switch (treppe_abschnitt) {
          case GERADE1:
          case GERADE2:
            {
              // alles fuer Geradeausflug
              // TODO Seitenabstands-Regelung


              if (treppe_abschnitt == GERADE1 && abst_vorne < 150) {
                treppe_abschnitt = KURVE1;
                drehung = 0;
              } else if (treppe_abschnitt == GERADE2 && abst_vorne < 150) {
                treppe_abschnitt = KURVE2;
                drehung = 0;
              }
              break;
            }
          case KURVE1:
          case KURVE2:
            {
              // TODO 90° drehen

              if (treppe_abschnitt == KURVE1 && drehung > 80) {
                treppe_abschnitt = GERADE2;
              } else if (treppe_abschnitt == KURVE2 && drehung > 80) {
                treppe_abschnitt = GERADE3;
              }
              break;
            }
          case GERADE3:
            {
              // alles fuer Geradeausflug
              // TODO Seitenabstands-Regelung

              if (abst_links == 0) {
                abschnitt = BARRIKADE;
              }

              break;
            }
          default:
            {
            }
        }
        break;
      }
    case BARRIKADE:
      {
        // Höhe leicht erhöhen
        Hoehenregelung(hoehe, 120);

        if (hoehe > 140) { // Höhe sehr groß - über Abgrund TODO
          abschnitt = ABSTIEG;
        }
        break;
      }
    case ABSTIEG:
      {
        // TODO mit Motor nach unten fliegen

        if (hoehe < 100) {
          abschnitt = LANDUNG;
        }
        break;
      }
    case LANDUNG:
      {
        // TODO sanfte Landung

        break;
      }
    default:
      {

        break;
      }
  }
}
