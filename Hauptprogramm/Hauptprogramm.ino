// Sensorik WS15 - HAW-Luftschiff
// Team 11: Beyerstedt, Schmid, Friedrich


//--- IMU-KRAM ---
// Uncomment the below line to use this axis definition: 
// X axis pointing forward
// Y axis pointing to the right 
// and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {
  1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition: 
// X axis pointing forward
// Y axis pointing to the left 
// and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

#include <Wire.h>

// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={
  0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {
  0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {
  0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {
  0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {
  0,0,0};//Omega Proportional correction
float Omega_I[3]= {
  0,0,0};//Omega Integrator
float Omega[3]= {
  0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {
  0,0,0}; 
float errorYaw[3]= {
  0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0    }
  ,{
    0,1,0    }
  ,{
    0,0,1    }
}; 
float Update_Matrix[3][3]={
  {
    0,1,2  }
  ,{
    3,4,5  }
  ,{
    6,7,8  }
}; //Gyros here

float Temporary_Matrix[3][3]={
  {
    0,0,0    }
  ,{
    0,0,0    }
  ,{
    0,0,0    }
};

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
  pinMode(STATUS_LED, OUTPUT);

  digitalWrite(MOTS_EN, HIGH);

  Serial.begin(9600);

  I2C_Init();
  digitalWrite(STATUS_LED,LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for(int i=0;i<32;i++)    // We take some readings...
  {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
  }

  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;

  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
    Serial.println(AN_OFFSET[y]);

  delay(2000);
  digitalWrite(STATUS_LED,HIGH);

  timer=millis();
  delay(20);
  counter=0;
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



