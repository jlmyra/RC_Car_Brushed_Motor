//*****************************************************************************
//*************************CONNECTION HANDLER**********************************
//*****************************************************************************
// This module handles PS3 controller connection events including:
// - Connection status monitoring
// - Connection event callbacks
// - Steering sweep animation on connection
// - Safety shutdown on disconnect

// External references to steering servo
extern Servo steerServo;
extern int servoMin;
extern int servoMax;

//*******************Steering Sweep Animation***********************
// Visual confirmation of controller connection - sweeps steering left-right-center
void steeringSweepAnimation() {
  int center = (servoMin + servoMax) / 2;

  Serial.println("→ Performing steering sweep test...");

  // Start from center
  steerServo.writeMicroseconds(center);
  delay(200);

  // Sweep left
  steerServo.writeMicroseconds(servoMin);
  delay(400);

  // Sweep right
  steerServo.writeMicroseconds(servoMax);
  delay(400);

  // Return to center
  steerServo.writeMicroseconds(center);
  delay(200);

  Serial.println("✓ Steering sweep complete - ready to drive!");
}

//*******************PS3 Controller Connection Event***********************
void onConnection() {
  if (Ps3.isConnected()) {
    Serial.println();
    Serial.println();
    Serial.println("********************************");
    Serial.println("*** PS3 Controller Connected ***");
    Serial.println("********************************");
    Serial.println();

    // Visual confirmation via steering sweep
    steeringSweepAnimation();

    Serial.println();
  }
}

//*******************PS3 Controller Disconnect Handler***********************
// Note: Safety shutdown on disconnect is handled in the main loop
// by checking Ps3.isConnected() which returns false when disconnected
// This causes the loop to return early, and the motor ramping naturally
// stops updating. For immediate stop on disconnect, consider using
// Ps3.attachOnDisconnect() in the future.

//*********************END Connection Handler Module*********************
