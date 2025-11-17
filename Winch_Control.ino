//*****************************************************************************
//*****************************WINCH CONTROL***********************************
//*****************************************************************************
// This module handles all winch motor control including:
// - Winch motor PWM pin definitions and variables
// - Winch unwind (up button) and rewind (down button) control
// - Speed control for winch motor

// External references to PWM settings from Motor_Control.ino
extern uint32_t freq;       // PWM Frequency
extern uint8_t resolution;  // PWM Resolution

//****************Winch Motor PWM Pin Definitions******************************/
uint8_t winchPWMChannel_1 = 4;
uint8_t winchPWMChannel_2 = 2;

int winchDirection_1 = 26; // GPIO26 - Winch Unwind A1A
int winchDirection_2 = 18; // GPIO18 - Winch Rewind A1B

//*******************Winch Setup Function***********************
void setupWinchControl() {
  pinMode(winchDirection_1, OUTPUT);
  pinMode(winchDirection_2, OUTPUT);

  ledcSetup(winchPWMChannel_1, freq, resolution);
  ledcAttachPin(winchDirection_1, winchPWMChannel_1);

  ledcSetup(winchPWMChannel_2, freq, resolution);
  ledcAttachPin(winchDirection_2, winchPWMChannel_2);
}

//*******************Winch Motor Control Event Handler***********************
// Up button = Unwind winch
// Down button = Rewind winch
// Winch speed is controlled by button pressure (analog value) * 0.8

void handleWinchControl() {
  // Handle UP button - UNWIND winch
  if((Ps3.event.analog_changed.button.up)) {
    Serial.print("Pressing the up button: ");
    digitalWrite(winchPWMChannel_1, LOW);
    ledcWrite(winchPWMChannel_2, Ps3.data.analog.button.up * .8);
    Serial.println(Ps3.data.analog.button.up, DEC);

    // Stop winch when button released
    if (Ps3.data.analog.button.up < 1) {
      digitalWrite(winchPWMChannel_1, LOW);
      digitalWrite(winchPWMChannel_2, LOW);
    }
  }

  // Handle DOWN button - REWIND winch
  if((Ps3.event.analog_changed.button.down)) {
    Serial.print("Pressing the down button: ");
    digitalWrite(winchPWMChannel_2, LOW);
    ledcWrite(winchPWMChannel_1, Ps3.data.analog.button.down * .8);
    Serial.println(Ps3.data.analog.button.down, DEC);

    // Stop winch when button released
    if (Ps3.data.analog.button.down < 1) {
      digitalWrite(winchPWMChannel_1, LOW);
      digitalWrite(winchPWMChannel_2, LOW);
    }
  }
}

//*********************END Winch Control Module*********************
