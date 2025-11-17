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
// Note: Safety shutdown on disconnect is handled in the main loop
// by checking Ps3.isConnected() which returns false when disconnected
// This causes the loop to return early, and the motor ramping naturally
// stops updating. For immediate stop on disconnect, consider using
// Ps3.attachOnDisconnect() in the future.

//*********************END Connection Handler Module*********************
