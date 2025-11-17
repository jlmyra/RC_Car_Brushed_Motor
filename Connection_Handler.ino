//*****************************************************************************
//*************************CONNECTION HANDLER**********************************
//*****************************************************************************
// This module handles PS3 controller connection events including:
// - Connection status monitoring
// - Connection event callbacks
// - Safety shutdown on disconnect

//*******************PS3 Controller Connection Event***********************
void onConnection() {
  if (Ps3.isConnected()) {
    Serial.println();
    Serial.println();
    Serial.println("********************************");
    Serial.println("*** PS3 Controller Connected ***");
    Serial.println("********************************");
    Serial.println();
    Serial.println();
  }
}

//*******************PS3 Controller Disconnect Handler***********************
// This function is called when the controller disconnects
// It ensures all motors are safely stopped
void onDisconnect() {
  Serial.println();
  Serial.println("**********************************");
  Serial.println("*** PS3 Controller Disconnected ***");
  Serial.println("**********************************");
  Serial.println();

  // Safety: Stop all motors immediately
  targetMotorSpeed = 0;
  currentMotorSpeed = 0;
  digitalWrite(driveMotorDirection, LOW);
  ledcWrite(driveMotorChannel, 0);

  // Stop winch motors
  digitalWrite(winchPWMChannel_1, LOW);
  digitalWrite(winchPWMChannel_2, LOW);
}

//*********************END Connection Handler Module*********************
