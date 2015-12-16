
// Parameter Regler Hoehe
unsigned long H_Ilasttime = millis();
int H_Iinterval = 100;
long H_Isum = 0;
#define H_Imax 1000
#define H_Ki 10
#define H_Kp 10

// Parameter Regler Geradeaus (Yaw)
#define Y_Kp 5

// Parameter Regler Seitenabstand
unsigned long S_Ilasttime = millis();
int S_Iinterval = 100;
long S_Isum = 0;
#define S_Imax 1000
#define S_Ki 10
#define S_Kp 10


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

void HeightControl(unsigned int distance, int offset) {
  int diff = offset - distance;

  if ( (millis() - H_Ilasttime) > H_Iinterval ) {
    H_Ilasttime = millis();

    H_Isum += diff;
    if (H_Isum > H_Imax) {
      H_Isum = H_Imax;
    } else if (H_Isum < -H_Imax) {
      H_Isum = -H_Imax;
    }
  }

  int power = (diff * H_Kp) + (H_Ki * H_Isum / H_Iinterval);

  Motor(power, 3);
}

void HeadingControl(float actual, float desired, int baseThrust) {
  actual = ToDeg(actual);
  desired = ToDeg(desired);
  float diff = actual - desired;
  if (diff > 180) {
    diff -= 360;
  } else if (diff < -180) {
    diff += 360;
  }

  int delta = diff * Y_Kp;

  Motor(baseThrust - delta, 1);
  Motor(baseThrust + delta, 2);
}

void SideDistanceControl(int actual, int desired, int baseThrust) {
  int diff = desired - actual;

  if ( (millis() - S_Ilasttime) > S_Iinterval ) {
    S_Ilasttime = millis();

    S_Isum += diff;
    if (S_Isum > S_Imax) {
      S_Isum = S_Imax;
    } else if (S_Isum < -S_Imax) {
      S_Isum = -S_Imax;
    }
  }

  int delta = (diff * S_Kp) + (S_Ki * S_Isum / S_Iinterval);

  Motor(baseThrust - delta, 1);
  Motor(baseThrust + delta, 2);
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

