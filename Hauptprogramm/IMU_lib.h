/*
 * Hilfsfunktionen für die Pololu AltIMU-10 v3
 * 
 * Hilfsfunktionen
 */


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

// Uncomment the below line to use this axis definition:
// X axis pointing forward, Y axis right, Z down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
//int SENSOR_SIGN[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// Uncomment the below line to use this axis definition:
// X axis pointing forward, Y axis left, Z up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

float G_Dt = 0.02;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer = 0; //general purpuse timer
long timer_old;
long timer24 = 0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6] = {0, 0, 0, 0, 0, 0}; //Array that stores the Offset of the sensors

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

float Accel_Vector[3] = {0, 0, 0}; //Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; //Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; //Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; //Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; //Omega Integrator
float Omega[3] = {0, 0, 0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3] = { 0, 0, 0 };
float errorYaw[3] = {0, 0, 0};

unsigned int counter = 0;
byte gyro_sat = 0;

float DCM_Matrix[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};

float Update_Matrix[3][3] = {
  {0, 1, 2},
  {3, 4, 5},
  {6, 7, 8}
}; //Gyros here

float Temporary_Matrix[3][3] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};



// ---------------I2C-------------
L3G gyro;
LSM303 compass;

void I2C_Init() {
  Wire.begin();
}

void Gyro_Init() {
  gyro.init();
  gyro.enableDefault();
  gyro.writeReg(L3G::CTRL_REG4, 0x20); // 2000 dps full scale
  gyro.writeReg(L3G::CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
}

void Read_Gyro() {
  gyro.read();

  AN[0] = gyro.g.x;
  AN[1] = gyro.g.y;
  AN[2] = gyro.g.z;
  gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
  gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
  gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

void Accel_Init() {
  compass.init();
  compass.enableDefault();
  switch (compass.getDeviceType()) {
    case LSM303::device_D:
      compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::device_DLHC:
      compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      compass.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
}

// Reads x,y and z accelerometer registers
void Read_Accel() {
  compass.readAcc();

  AN[3] = compass.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  AN[4] = compass.a.y >> 4;
  AN[5] = compass.a.z >> 4;
  accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
  accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
  accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}

void Compass_Init() {
  // doesn't need to do anything because Accel_Init() should have already called compass.enableDefault()
}

void Read_Compass() {
  compass.readMag();

  magnetom_x = SENSOR_SIGN[6] * compass.m.x;
  magnetom_y = SENSOR_SIGN[7] * compass.m.y;
  magnetom_z = SENSOR_SIGN[8] * compass.m.z;
}

// -----------COMPASSKRAM----------

void Compass_Heading() {
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;

  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);

  // adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
  c_magnetom_x = (float)(magnetom_x - SENSOR_SIGN[6] * M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6] * 0.5;
  c_magnetom_y = (float)(magnetom_y - SENSOR_SIGN[7] * M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7] * 0.5;
  c_magnetom_z = (float)(magnetom_z - SENSOR_SIGN[8] * M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8] * 0.5;

  // Tilt compensated Magnetic filed X:
  MAG_X = c_magnetom_x * cos_pitch + c_magnetom_y * sin_roll * sin_pitch + c_magnetom_z * cos_roll * sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = c_magnetom_y * cos_roll - c_magnetom_z * sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-MAG_Y, MAG_X);
}

// -----------VEKTORKRAM-----------
//Computes the dot product of two vectors
float Vector_Dot_Product(float vector1[3], float vector2[3]) {
  float op = 0;

  for (int c = 0; c < 3; c++) {
    op += vector1[c] * vector2[c];
  }

  return op;
}

//Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3]) {
  vectorOut[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  vectorOut[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  vectorOut[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

//Multiply the vector by a scalar.
void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2) {
  for (int c = 0; c < 3; c++) {
    vectorOut[c] = vectorIn[c] * scale2;
  }
}

void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]) {
  for (int c = 0; c < 3; c++) {
    vectorOut[c] = vectorIn1[c] + vectorIn2[c];
  }
}


//-----------------------MATRIXKRAM--------------------------
//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!).
void Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3]) {
  float op[3];
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      for (int w = 0; w < 3; w++) {
        op[w] = a[x][w] * b[w][y];
      }
      mat[x][y] = 0;
      mat[x][y] = op[0] + op[1] + op[2];

      float test = mat[x][y];
    }
  }
}

