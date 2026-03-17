#include "constants.h"


void setup() {
  initializeMotors();
  setupEncoder();

  Serial.begin(9600); // For debugging
  bluetooth.begin(9600);     // For HC-05 (default baud rate is 9600)
  Serial.println("Staring Serial Print");
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmissio
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

void loop() {
  read_imu();
  //   float pidOutput = computePID(pitch); // Compute PID output
  float pidOutput = controller_simple_why_complicate_life();
  //bluetooth_send(pitch);

  
  controlMotors(pidOutput); // Control motors based on PID output

  // Debugging
  // Serial.print("tilt Angle: ");
  // Serial.print(tilt_angle);
  // Serial.print(" | roll Angle: ");
  // Serial.print(roll);
  // Serial.print(" | yaw Angle: ");
  // Serial.print(yaw);
  //Serial.print(" | pitch Angle: ");
  //Serial.println(pitch);
  //Serial.print(" | tilt error, PID Output: ");
  //Serial.print( wheel_pulse_count_left);
  ///Serial.print(" ; ");
  //Serial.println(wheel_pulse_count_right);

  //Serial.println("Tilt Error: " + String(tilt_error_print, 2) + " pitch: " + String(pitch, 2) );
 // Serial.println(pidOutput);

  unsigned long currentTime = millis(); // Get the current time in milliseconds
  float elapsedTime = currentTime / 1000.0; // Convert to seconds
  Serial.print(elapsedTime); // Log elapsed time
  Serial.print(",");        // CSV separator
  Serial.print(pidOutput);
  Serial.print(",");        // CSV separator
  Serial.print(pitch);

}



