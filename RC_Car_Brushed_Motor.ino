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
#include "esp_adc_cal.h"
#include "Model_Variables.h"

//****************Global ESP32 PWM Object******************************/
ESP32PWM pwm;

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
}

//*****************************************************************************
//***********************************LOOP**************************************
//*****************************************************************************
void loop() {
  // Check if PS3 controller is connected
  if(!Ps3.isConnected()) {
    return;
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
  handleMotorControl();     // Motor_Control.ino - Handle drive motor joystick input
  handleSteeringControl();  // Steering_Control.ino - Handle steering joystick input
  handleWinchControl();     // Winch_Control.ino - Handle winch button input
}

//*****************************************************************************
//***********************************END***************************************
//*****************************************************************************
