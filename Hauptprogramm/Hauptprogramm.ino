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
#define SERIAL

// IMU
//#define OUTPUTMODE_C   //for corrected data, comment for uncorrectet data of gyro (with drift)
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


#include "functions.h"


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

#ifdef SERIAL
  Serial.begin(9600);
  while (!Serial);
#endif

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


void loop() {
  IMU_Berechnungen();

  hoehe = IR_U.get_distance();
  abst_links = US_L.get_distance();
  abst_vorne = IR_V.get_distance();

#ifdef DEBUG_SENSORS
  Serial.print("\t"); Serial.print("US1 "); Serial.print(abst_links);
  Serial.print("\t"); Serial.print("IR1 "); Serial.print(abst_vorne);
  Serial.print("\t"); Serial.print("IR2 "); Serial.print(hoehe);
  //Serial.print("\t"); Serial.print("MAG ");
  //Serial.print(c_magnetom_x); Serial.print (","); Serial.print(c_magnetom_y); Serial.print (","); Serial.print(c_magnetom_z);
  Serial.print("\n");
#endif


  switch (abschnitt) {
    case START:
      {
        //Motor(255, 3);

        if (hoehe > 80) {
          abschnitt = GERADEAUS;
#ifdef SERIAL Serial.println(" -- to state GERADEAUS -- ");
#endif
        }
        break;
      }
    case GERADEAUS:
      {
        HeightControl(hoehe, 100);
        HeadingControl(yaw, 0, 0); // TODO parameters

        if (0) { // if Treppe erkannt
          abschnitt = TREPPE;
#ifdef SERIAL Serial.println(" -- to state TREPPE -- ");
#endif
        }
        break;
      }
    case TREPPE:
      {
        // Höhenregelung an
        HeightControl(hoehe, 100);

        switch (treppe_abschnitt) {
          case GERADE1:
          case GERADE2:
            {
              // alles fuer Geradeausflug
              // TODO Seitenabstands-Regelung


              if (treppe_abschnitt == GERADE1 && abst_vorne < 150) {
                treppe_abschnitt = KURVE1;
                drehung = 0;
#ifdef SERIAL Serial.println(" -- to state KURVE1 -- ");
#endif
              } else if (treppe_abschnitt == GERADE2 && abst_vorne < 150) {
                treppe_abschnitt = KURVE2;
                drehung = 0;
#ifdef SERIAL Serial.println(" -- to state KURVE2 -- ");
#endif
              }
              break;
            }
          case KURVE1:
          case KURVE2:
            {
              // TODO 90° drehen

              if (treppe_abschnitt == KURVE1 && drehung > 80) {
                treppe_abschnitt = GERADE2;
#ifdef SERIAL
                Serial.println(" -- to state GERADE2 -- ");
#endif
              } else if (treppe_abschnitt == KURVE2 && drehung > 80) {
                treppe_abschnitt = GERADE3;
#ifdef SERIAL Serial.println(" -- to state GERADE3 -- ");
#endif
              }
              break;
            }
          case GERADE3:
            {
              // alles fuer Geradeausflug
              // TODO Seitenabstands-Regelung

              if (abst_links == 0) {
                abschnitt = BARRIKADE;
#ifdef SERIAL Serial.println(" -- to state BARRIKADE -- ");
#endif
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
        HeightControl(hoehe, 120);

        if (hoehe > 140) { // Höhe sehr groß - über Abgrund TODO
          abschnitt = ABSTIEG;
#ifdef SERIAL Serial.println(" -- to state ABSTIEG -- ");
#endif
        }
        break;
      }
    case ABSTIEG:
      {
        // TODO mit Motor nach unten fliegen

        if (hoehe < 100) {
          abschnitt = LANDUNG;
#ifdef SERIAL Serial.println(" -- to state LANDUNG -- ");
#endif
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

