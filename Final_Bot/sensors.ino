
// Function Name: setupEncoder()
// Input: none
// Output: none
// Logic: Configure encoder pins and set up hardware interrupts for pulse counting.
// Example Call: setupEncoder();
void setupEncoder() {
    Serial.begin(9600);
  // put your setup code here, to run once:
  //------Encoder HW Interrupt setup-----//
  pinMode(8, INPUT_PULLUP);
  digitalWrite(8, HIGH);
  // attachInterrupt(digitalPinToInterrupt(2), mot_rencoder, RISING);

  pinMode(3, INPUT_PULLUP);
  digitalWrite(3, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), mot_rencoder_left, RISING);
  // ---------------------------------------------- ///

  //------Encoder HW Interrupt setup-----//
  pinMode(7, INPUT_PULLUP);  //10
  digitalWrite(7, HIGH);
  // attachInterrupt(digitalPinToInterrupt(2), mot_rencoder, RISING);

  pinMode(2, INPUT_PULLUP);  //11
  digitalWrite(2, HIGH);
  attachInterrupt(digitalPinToInterrupt(2), mot_rencoder_right, RISING);
}

// Function Name: mot_rencoder_left
// Input: None
// Output: None
// Logic: Interrupt Service Routine (ISR) for the left motor encoder. Increments or decrements the wheel pulse count based on the encoder signal.
// Example Call: Triggered automatically by an interrupt.


void mot_rencoder_left() {
  // if (digitalRead(encodPinBR) == HIGH) {
  if (digitalRead(encodPinBL) > digitalRead(encodPinAL)) {
    wheel_pulse_count_left = wheel_pulse_count_left + 1;
  } 
  else {
    wheel_pulse_count_left = wheel_pulse_count_left - 1;
  }
}

void mot_rencoder_right() {
  // if (digitalRead(encodPinBR) == HIGH) {
  if (digitalRead(encodPinBR) > digitalRead(encodPinAR)) {
    wheel_pulse_count_right = wheel_pulse_count_right - 1;
  }
  else {
    wheel_pulse_count_right = wheel_pulse_count_right + 1;
  }
}


// Tanays IMU Read Function
void read_imu(){
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + 1.68; // AccErrorX ~(1.98) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.35; // AccErrorY ~(-0.21)

  // === Read gyroscope data === //
  previousTimeT = currentTimeT;        // Previous time is stored before the actual time read
  currentTimeT = millis();            // Current time actual time read
  elapsedTime = (currentTimeT - previousTimeT) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Correct the outputs with the calculated error values
  GyroX = GyroX + 3.52; // GyroErrorX ~(-1.72)
  GyroY = GyroY + 1.46; // GyroErrorY ~(-0.73)
  GyroZ = GyroZ + 0.08; // GyroErrorZ ~ (-0.03)

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  roll = (0.98 * gyroAngleX) + (0.02 * accAngleX);
  pitch = (0.98 * gyroAngleY) + (0.02 * accAngleY);

  gyroAngleX = roll;     //corrects the gyroAngleX and y else senser reading gyroAnglex and y starts to drift
  gyroAngleY = pitch;

  roll -= roll_offset;

  // Print the values on the serial monitor
  // Serial.print(roll); Serial.print(" ");
  //Serial.print("/");
  // Serial.print(pitch);                       
  //Serial.print("/");
  //Serial.println(yaw);
}