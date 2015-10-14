// Sensorik WS15 - HAW-Luftschiff
// Team 11: Beyerstedt, Schmid, Friedrich

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

enum Status {
  START=1, GERADEAUS, TREPPE, BARRIKADE, LANDUNG};
static enum Status S = START;

// --- OPTIONEN ---
//#define DEBUG_IR

// --- VARIABLEN ---
long duration=0;
long distance=0;

void setup(){
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

  digitalWrite(MOTS_EN, HIGH);

  Serial.begin(9600);
}

void Hoehenregelung(){
  
  
}


void loop(){

  Hoehenregelung();

  switch(S) {
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


