void bluetooth_send(float data){
  // Check for data from Serial Monitor
  
   
    bluetooth.print(data); // Send data to Bluetooth
    //Serial.print("Sent to Bluetooth: ");
    //Serial.println(serialData);
  
}
// Function Name: initializeMotors()
// Input: none
// Output: none
// Logic: Initialize motor pins as outputs and set initial motor speeds using enable pins.
// Example Call: initializeMotors();
void initializeMotors() {
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(inputPin1, OUTPUT);
  pinMode(inputPin2, OUTPUT);
  pinMode(inputPin3, OUTPUT);
  pinMode(inputPin4, OUTPUT);
  analogWrite(EN1, 90); // Set initial motor speed
  analogWrite(EN2, 90);
}

