//*****************************************************************************
//*****************************MOTOR CONTROL***********************************
//*****************************************************************************
// This module handles all drive motor control including:
// - Motor PWM pin definitions and variables
// - Motor speed ramping for smooth acceleration/deceleration
// - Forward/reverse/stop control via left joystick
// - Nitro and normal speed modes

//****************Motor PWM Pin Definitions******************************/
uint32_t freq = 30000;  //PWM Frequency
uint8_t driveMotorChannel = 14; //Changed line 96 espservo.h to 2 servos max leaving channels 2-16 available for motors
uint8_t resolution = 8; //8 bit resolution PWM
int driveMotorPin = 25; //Cytron white wire (pwm) GPIO25
int driveMotorDirection = 27; //Cytron yellow wire (DIR) GPIO27

//*********************Motor Speed Variables***********************/
int motorSpeed = 0;
int motorSpeedSlow = 0;
float nitroSpeed = 1;
float normalSpeed = 0.6;
int joystickPos = 0;

//*********************Motor Smoothing Variables***********************/
int currentMotorSpeed = 0;      // Current actual motor speed (smoothed)
int targetMotorSpeed = 0;       // Target motor speed from joystick
int targetDirection = LOW;      // Target motor direction (LOW=forward, HIGH=reverse)
int currentDirection = LOW;     // Current motor direction

// Ramping rates - configured in Model_Variables.h per vehicle
// Higher values = faster/more aggressive, Lower values = smoother/gentler
int accelerationRate = MV_accelerationRate;    // Speed increase per update cycle
int decelerationRate = MV_decelerationRate;    // Speed decrease per update cycle (usually faster than accel)

unsigned long lastRampUpdate = 0;
unsigned long rampUpdateInterval = MV_rampUpdateInterval; // Update interval in ms (default: 10ms = 100Hz)

//*******************Motor Setup Function***********************
void setupMotorControl() {
  pinMode(driveMotorPin, OUTPUT);
  pinMode(driveMotorDirection, OUTPUT);

  ledcSetup(driveMotorChannel, freq, resolution);
  ledcAttachPin(driveMotorPin, driveMotorChannel); // GPIO25
}

//*******************Motor Ramping Function for Smooth Acceleration/Deceleration***********************
void updateMotorRamping() {
  unsigned long currentMillis = millis();

  // Only update at specified interval (default 10ms)
  if (currentMillis - lastRampUpdate < rampUpdateInterval) {
    return;
  }
  lastRampUpdate = currentMillis;

  // Handle direction changes - need to ramp down to zero before changing direction
  if (targetDirection != currentDirection && currentMotorSpeed > 0) {
    // Ramp down to zero before direction change
    if (currentMotorSpeed > decelerationRate) {
      currentMotorSpeed -= decelerationRate;
    } else {
      currentMotorSpeed = 0;
      currentDirection = targetDirection; // Change direction when stopped
    }
  }
  // Normal ramping when direction matches or motor is stopped
  else {
    currentDirection = targetDirection;

    // Ramp speed up or down toward target
    if (currentMotorSpeed < targetMotorSpeed) {
      // Accelerating
      currentMotorSpeed += accelerationRate;
      if (currentMotorSpeed > targetMotorSpeed) {
        currentMotorSpeed = targetMotorSpeed; // Don't overshoot
      }
    }
    else if (currentMotorSpeed > targetMotorSpeed) {
      // Decelerating
      currentMotorSpeed -= decelerationRate;
      if (currentMotorSpeed < targetMotorSpeed) {
        currentMotorSpeed = targetMotorSpeed; // Don't undershoot
      }
    }
  }

  // Apply smoothed speed to motor
  digitalWrite(driveMotorDirection, currentDirection);
  ledcWrite(driveMotorChannel, currentMotorSpeed);
}

//*****************Left Joystick - Motor Speed & Direction Event Handler***********************
//Pushing the joystick forward creates negative values from 0 to -128
//Those values get mapped to 2^8 bits or 0 to -256.
//When the joystickPos values go negative or less than zero we want to ramp up the motor speed and
//create forward motion.
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

//*******************Update Motor Speed Based on Joystick and L1 Button***********************
// This function recalculates target speed based on current joystick position and L1 state
void updateMotorSpeed() {
  joystickPos = (Ps3.data.analog.stick.ly);
  motorSpeed = map(joystickPos, -128, 128, -256, 256);

  // Determine speed multiplier based on L1 button (nitro/turbo)
  float speedMultiplier = Ps3.data.button.l1 ? nitroSpeed : normalSpeed;

  //***FORWARD***
  if(joystickPos < -2) {
    motorSpeed = -motorSpeed; // Change sign of motorSpeed
    targetDirection = LOW; // Cytron MD-13S DIR pin LOW for forward
    targetMotorSpeed = (motorSpeed * speedMultiplier);

    Serial.print("FORWARD ");
    Serial.print(Ps3.data.button.l1 ? "NITRO: " : "NORMAL: ");
    Serial.print("  targetSpeed="); Serial.print(targetMotorSpeed);
    Serial.print("  joystickPos="); Serial.println(joystickPos);
  }
  //***REVERSE***
  else if(joystickPos > 2) {
    targetDirection = HIGH; // Cytron MD-13S DIR pin HIGH to reverse motor direction
    targetMotorSpeed = (motorSpeed * speedMultiplier);

    Serial.print("REVERSE ");
    Serial.print(Ps3.data.button.l1 ? "NITRO: " : "NORMAL: ");
    Serial.print("  targetSpeed="); Serial.print(targetMotorSpeed);
    Serial.print("  joystickPos="); Serial.println(joystickPos);
  }
  //***STOP***
  else if(joystickPos > -2 && joystickPos < 2) {
    motorSpeed = 0;
    targetMotorSpeed = 0; // Smoothly decelerate to zero

    Serial.print("STOP:  targetSpeed="); Serial.print(targetMotorSpeed);
    Serial.print("  joystickPos="); Serial.println(joystickPos);
  }
}

void handleMotorControl() {
  // Update motor speed when joystick changes
  if((abs(Ps3.event.analog_changed.stick.ly) > 2)) {
    updateMotorSpeed();
  }

  // IMPORTANT: Update motor speed when L1 button is pressed or released
  // This allows turbo/nitro mode to engage at ANY speed, not just from stop
  if(Ps3.event.button_down.l1 || Ps3.event.button_up.l1) {
    Serial.print("L1 button ");
    Serial.println(Ps3.data.button.l1 ? "PRESSED - NITRO MODE ENGAGED" : "RELEASED - NORMAL MODE");
    updateMotorSpeed(); // Recalculate speed with new multiplier
  }
}

//*********************END Motor Control Module*********************
