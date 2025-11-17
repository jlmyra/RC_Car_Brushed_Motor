//*****************************************************************************
//****************************STEERING CONTROL*********************************
//*****************************************************************************
// This module handles all steering servo control including:
// - Steering servo pin definitions and variables
// - Servo setup and calibration
// - Right joystick control for steering
// - Exponential steering response for better precision
// Note: ESP32Servo.h is included in main file

//****************Debug Macros******************************/
#if MV_DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

//****************Steering Servo Variables******************************/
Servo steerServo;  // Create servo object to control a servo
                   // 2 servo objects can be created on the ESP32
int servoPin = 4;  // GPIO4

// ServoMin and ServoMax can be used to 'align' the front wheels so that they are centered when the joystick position
// is zeroed. On my vehicles the range is 800 between wheels with different range endpoints. Your vehicle may
// require different endpoints to get the wheels centered.
int servoMin = MV_servoMin; // Drivers Side  range 800
int servoMax = MV_servoMax; // Passenger side

// Steering expo for smoother control
float steeringExpo = MV_STEERING_EXPO;

//*******************Exponential Curve Function for Steering***********************
// Applies exponential curve to steering input for more precise center control
// input: -1.0 to 1.0, expo: 0.0 (linear) to 1.0 (very exponential)
// Returns: curved value from -1.0 to 1.0
float applySteeringExpo(float input, float expo) {
  if (expo == 0.0) return input; // Linear, no expo

  float sign = (input >= 0) ? 1.0 : -1.0;
  float absInput = abs(input);

  // Exponential curve: output = input^(1+expo) for more precision at center
  float curved = pow(absInput, 1.0 + expo);

  return sign * curved;
}

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
  // Handle right joystick movement for steering (both positive and negative values)
  if(abs(Ps3.event.analog_changed.stick.rx) > 2) {
    int x_position = Ps3.data.analog.stick.rx;   // Uses joystick x position for steering left/right

    // Apply exponential curve to steering input for better precision
    float normalizedInput = x_position / 128.0; // Convert to -1.0 to 1.0
    float curvedInput = applySteeringExpo(normalizedInput, steeringExpo);
    int curvedPosition = (int)(curvedInput * 128.0); // Convert back to -128 to 128

    int joystick_Pos = map(curvedPosition, -128, 128, servoMin, servoMax); // Map joystick values to servo values
    steerServo.write(joystick_Pos);

    DEBUG_PRINT("Steering: x="); DEBUG_PRINT(x_position);
    DEBUG_PRINT(" curved="); DEBUG_PRINT(curvedPosition);
    DEBUG_PRINT(" servo="); DEBUG_PRINTLN(joystick_Pos);
  }
}

//*********************END Steering Control Module*********************
