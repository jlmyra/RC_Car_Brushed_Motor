//*****************************************************************************
//*******************RC CAR BRUSHED MOTOR CONTROLLER - MAIN*******************
//*****************************************************************************
// ESP32-based RC car controller with PS3 gamepad support
// Version 2.0 - Modular architecture with performance and safety optimizations
//
// Modules:
// - Motor_Control.ino: Drive motor control with smooth ramping
// - Steering_Control.ino: Servo-based steering control
// - Winch_Control.ino: Winch motor control for up/down
// - Battery_Monitor.ino: Battery voltage monitoring and status display
// - Connection_Handler.ino: PS3 controller connection management
// - Model_Variables.h: Model-specific configuration (JEEP/LANDY/BUGGY)
//
// Author: Enhanced with Claude Code - November 2025
//*****************************************************************************

#include <Ps3Controller.h>
#include <ESP32Servo.h>
#include "esp_adc_cal.h"
#include "Model_Variables.h"

//****************Connection Watchdog Timer******************************/
unsigned long lastEventTime = 0;
unsigned long connectionTimeout = MV_CONNECTION_TIMEOUT;
bool watchdogTriggered = false;

// External references for emergency stop
extern int targetMotorSpeed;
extern int currentMotorSpeed;

//*****************************************************************************
//*********************************SETUP***************************************
//*****************************************************************************
void setup() {
  Serial.begin(250000);

  // Initialize PS3 Controller
  if (!Ps3.begin("b8:27:eb:37:85:b9")) { // This is unique to my controllers - you will have to change this for yours
    Serial.println("Initialization failed.");
    return;
  }
  else {
    Serial.println();
    Serial.println();
    Serial.println("***********************************************************");
    Serial.println("***Initialization finished. Ready to pair PS3 controller***");
    Serial.println("***********************************************************");
    Serial.println();
    Serial.println();
  }

  // Attach event handlers
  Ps3.attach(onEvent);
  Ps3.attachOnConnect(onConnection);

  // Allow allocation of all timers for PWM
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Initialize all modules
  setupBatteryMonitor();   // Battery_Monitor.ino
  setupMotorControl();     // Motor_Control.ino
  setupSteeringControl();  // Steering_Control.ino
  setupWinchControl();     // Winch_Control.ino

  // Initialize watchdog timer
  lastEventTime = millis();

  Serial.println();
  Serial.println("✓ All systems initialized");
  Serial.println("✓ Safety features active:");
  Serial.println("  - Connection watchdog enabled");
  Serial.println("  - Motor speed bounds checking");
  Serial.println("  - Battery voltage protection");
  Serial.println();
}

//*****************************************************************************
//***********************************LOOP**************************************
//*****************************************************************************
void loop() {
  // Check if PS3 controller is connected
  if(!Ps3.isConnected()) {
    watchdogTriggered = false; // Reset watchdog on disconnect
    return;
  }

  // SAFETY: Connection watchdog - emergency stop if no events received
  unsigned long currentTime = millis();
  if (currentTime - lastEventTime > connectionTimeout) {
    if (!watchdogTriggered) {
      // Emergency stop all motors
      targetMotorSpeed = 0;
      currentMotorSpeed = 0;
      watchdogTriggered = true;
      Serial.println("⚠️ WATCHDOG TRIGGERED - NO CONTROLLER EVENTS - EMERGENCY STOP");
    }
    // Don't update motors while watchdog is active
    return;
  } else {
    watchdogTriggered = false;
  }

  // Update all control systems
  updateMotorRamping();     // Motor_Control.ino - Smooth motor speed transitions
  computeBatteryVoltage();  // Battery_Monitor.ino - Monitor battery and update status LEDs
}

//*****************************************************************************
//******************************EVENT HANDLER**********************************
//*****************************************************************************
// This function is called whenever a PS3 controller event occurs
// It dispatches events to the appropriate control modules
void onEvent() {
  // Update watchdog timer - we received an event from controller
  lastEventTime = millis();

  handleMotorControl();     // Motor_Control.ino - Handle drive motor joystick input
  handleSteeringControl();  // Steering_Control.ino - Handle steering joystick input
  handleWinchControl();     // Winch_Control.ino - Handle winch button input
}

//*****************************************************************************
//***********************************END***************************************
//*****************************************************************************
