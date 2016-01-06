
// Parameter Regler Hoehe
unsigned long H_Ilasttime = millis();
int H_Iinterval = 20;
int H_Dinterval = 20;
long H_Isum = 0;
int H_Dlastdiff = 0;
int H_Doutput = 0;
#define H_Imax 1000
#define H_Ki 1
#define H_Kp 10
#define H_Kd 10

// Parameter Regler Geradeaus (Yaw)
unsigned long Y_Ilasttime = millis();
int Y_Iinterval = 20;
int Y_Dinterval = 20;
long Y_Isum = 0;
int Y_Dlastdiff = 0;
int Y_Doutput = 0;
#define Y_Imax 1000
#define Y_Ki 0
#define Y_Kp 1
#define Y_Kd 10

// Parameter Regler Seitenabstand
unsigned long S_Ilasttime = millis();
int S_Iinterval = 10;
long S_Isum = 0;
#define S_Imax 1000
#define S_Ki 1
#define S_Kp 1


// LED status blink
unsigned long LED1_lasttime1 = millis();
unsigned long LED1_lasttime2 = millis();
int LED1_counter = 0;
unsigned long LED2_lasttime1 = millis();
unsigned long LED2_lasttime2 = millis();
int LED2_counter = 0;

// Motoren
#define M1_MIN_POW 45
#define M2_MIN_POW 45
#define M3_MIN_POW 35


void Motor(int power, int motNr) {
  int pinA, pinB, minPower;

  if (motNr == 1) {
    pinA = MOT1_A;
    pinB = MOT1_B;
    minPower = M1_MIN_POW;
  } else if (motNr == 2) {
    pinA = MOT2_A;
    pinB = MOT2_B;
    minPower = M2_MIN_POW;
  } else if (motNr == 3) {
    pinA = MOT3_A;
    pinB = MOT3_B;
    minPower = M3_MIN_POW;
  }

  if (power < 0) {
    power = -power + minPower;
    if (power > 255) {
      power = 255;
    }
    analogWrite(pinA, power);
    digitalWrite(pinB, LOW);
  } else {
    power = power + minPower;
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

    // I-Regler
    H_Isum += diff;
    if (H_Isum > H_Imax) {
      H_Isum = H_Imax;
    } else if (H_Isum < -H_Imax) {
      H_Isum = -H_Imax;
    }

    // D-Regler
    H_Doutput = H_Kd * (diff - H_Dlastdiff) / H_Dinterval;
    H_Dlastdiff = diff;
  }

  int power = (diff * H_Kp) + (H_Ki * H_Isum / H_Iinterval) + H_Doutput;
  if (power < 0) {
    power = 0;
  }
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

  if ( (millis() - Y_Ilasttime) > Y_Iinterval ) {
    Y_Ilasttime = millis();

    // I-Regler
    Y_Isum += diff;
    if (Y_Isum > Y_Imax) {
      Y_Isum = Y_Imax;
    } else if (Y_Isum < -Y_Imax) {
      Y_Isum = -Y_Imax;
    }

    // D-Regler
    Y_Doutput = Y_Kd * (diff - Y_Dlastdiff) / Y_Dinterval;
    Y_Dlastdiff = diff;
  }

  int delta = (diff * Y_Kp) + (Y_Ki * Y_Isum / Y_Iinterval) + Y_Doutput;

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

  Motor(baseThrust + delta, 1);
  Motor(baseThrust - delta, 2);
}


void LED_BlinkMain(int blinkAnzahl) {
  if (LED1_counter < blinkAnzahl * 2) {
    if ((millis() - LED1_lasttime1) > 150) {
      digitalWrite(LED_R1, !digitalRead(LED_R1));
      LED1_lasttime1 = millis();
      LED1_counter++;
    }
  } else {
    if ((millis() - LED1_lasttime2) > 3000) {
      digitalWrite(LED_R1, LOW);
      LED1_lasttime2 = millis();
      LED1_counter = 0;
    }
  }
}


void LED_StateStairs(int blinkAnzahl) {
  if (LED2_counter < blinkAnzahl * 2) {
    if ((millis() - LED2_lasttime1) > 150) {
      digitalWrite(LED_R1, !digitalRead(LED_R2));
      LED2_lasttime1 = millis();
      LED2_counter++;
    }
  } else {
    if ((millis() - LED2_lasttime2) > 3000) {
      digitalWrite(LED_R2, LOW);
      LED2_lasttime2 = millis();
      LED2_counter = 0;
    }
  }
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

