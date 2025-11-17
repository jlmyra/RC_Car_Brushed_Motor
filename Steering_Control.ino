//*****************************************************************************
//****************************STEERING CONTROL*********************************
//*****************************************************************************
// This module handles all steering servo control including:
// - Steering servo pin definitions and variables
// - Servo setup and calibration
// - Right joystick control for steering
// Note: ESP32Servo.h is included in main file

//****************Steering Servo Variables******************************/
Servo steerServo;  // Create servo object to control a servo
                   // 2 servo objects can be created on the ESP32
int servoPin = 4;  // GPIO4

// ServoMin and ServoMax can be used to 'align' the front wheels so that they are centered when the joystick position
// is zeroed. On my vehicles the range is 800 between wheels with different range endpoints. Your vehicle may
// require different endpoints to get the wheels centered.
int servoMin = MV_servoMin; // Drivers Side  range 800
int servoMax = MV_servoMax; // Passenger side

//*******************Steering Setup Function***********************
void setupSteeringControl() {
  // Allow allocation of timers (called in main setup)
  // ESP32PWM::allocateTimer() calls are in main setup

  steerServo.setPeriodHertz(300);    // Standard 50 hz servo
  steerServo.attach(servoPin, servoMin, servoMax); // Attaches the servo on pin 4 to the servo object

  // ServoMin and ServoMax can be used to 'align' the front wheels so that they are centered when the joystick position
  // is zeroed. On my vehicles the range is 800 between wheels with different range endpoints. Your vehicle may
  // require different endpoints to get the wheels centered.
}

//*******************Right Joystick - Steering Servo Control Event Handler***********************
//  =============================================================
//  |         Forward       |                                   |
//  |          -128         |                                   |
//  |           /\          |               /\                  |
//  |        <  ly  >       | Left -128  <  rx  > +128 Right    |
//  |           \/          |               \/                  |
//  |          +128         |                                   |
//  |        Reverse        |                                   |
//  |                       |                                   |
//  |      LEFT JOYSTICK    |         RIGHT JOYSTICK            |
//  |         Motion        |            Steering               |
//  =============================================================

void handleSteeringControl() {
  // Handle right joystick movement for steering (positive values)
  if((Ps3.event.analog_changed.stick.rx) > 2) {
    int joystick_Pos;
    int x_position = Ps3.data.analog.stick.rx;   // Uses joystick x position for steering left/right
    joystick_Pos = map(x_position, -128, 128, servoMin, servoMax); // Map joystick values to servo values
    steerServo.write(joystick_Pos);

    Serial.print("Moved the right stick:");
    Serial.print(" x="); Serial.print(Ps3.data.analog.stick.rx, DEC);
    Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ry, DEC);
    Serial.println();
    Serial.print("x_Position=  "); Serial.print(x_position);
    Serial.print("  joystick_Pos= "); Serial.println(joystick_Pos);
  }

  // Handle right joystick movement for steering (negative values)
  if((Ps3.event.analog_changed.stick.rx) < -2) {
    int joystick_Pos;
    int x_position = Ps3.data.analog.stick.rx;   // Uses joystick x position for steering left/right
    joystick_Pos = map(x_position, -128, 128, servoMin, servoMax); // Map joystick values to servo values
    steerServo.write(joystick_Pos);

    Serial.print("Moved the right stick:");
    Serial.print(" x="); Serial.print(Ps3.data.analog.stick.rx, DEC); // Serial.print DECimal
    Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ry, DEC);
    Serial.println();
    Serial.print("x_Position=  "); Serial.print(x_position);
    Serial.print("  joystick_Pos= "); Serial.println(joystick_Pos);
  }
}

//*********************END Steering Control Module*********************
