

//***JEEP Variables***

//Debug Mode (set to 0 for production to eliminate Serial output overhead)
#define MV_DEBUG_MODE 1;  // 1 = Debug output enabled, 0 = No debug output

//Connection Watchdog (safety timeout)
#define MV_CONNECTION_TIMEOUT 500;  // Milliseconds - emergency stop if no events

//Motor Speed Limits
#define MV_SPEED_LIMIT_NORMAL 255;  // Maximum motor speed in normal mode (0-255)
#define MV_SPEED_LIMIT_CRAWL 128;   // Maximum motor speed in crawl mode (0-255)

//Steering Expo (0.0 = linear, higher = more exponential response)
#define MV_STEERING_EXPO 0.3;  // 0.0-1.0, recommended: 0.2-0.5

//Battery Protection
#define MV_LOW_VOLTAGE_CUTOFF 6.5;     // Volts - limit power below this voltage
#define MV_LOW_VOLTAGE_POWER_LIMIT 0.5; // Multiply motor speed by this when low

//Speed Profiles
#define MV_SPEED_CRAWL 0.3;   // Crawl mode - precise control
#define MV_SPEED_NORMAL 0.6;  // Normal mode - balanced
#define MV_SPEED_SPORT 0.8;   // Sport mode - spirited
#define MV_SPEED_RACE 1.0;    // Race mode - full power

//Motor Expo (0.0 = linear, higher = more exponential throttle response)
#define MV_MOTOR_EXPO 0.2;  // 0.0-1.0, recommended: 0.1-0.3

//Steering Variables:
#define MV_servoMin 1100;
#define MV_servoMax 2075;

//Voltage Divider Resistors:
#define MV_R1 101600; //Voltage divider R1
#define MV_R2 41600; //Voltage Divider R2

//Battery Voltage Correction Factor
#define MV_batCorrFactor 0.0;

//Motor Ramping Rates (adjust for desired acceleration/deceleration feel)
//Higher values = faster/more aggressive, Lower values = smoother/gentler
#define MV_accelerationRate 3;    // Speed increase per update cycle (default: 3)
#define MV_decelerationRate 5;    // Speed decrease per update cycle (default: 5)
#define MV_rampUpdateInterval 10; // Update interval in milliseconds (default: 10ms = 100Hz)


//***LANDY Variables***
/*
//Steering Variables:
#define MV_servoMin 1125;
#define MV_servoMax 1925;

//Voltage Divider Resistors:
#define MV_R1 102800; //Voltage divider R1
#define MV_R2 41700; //Voltage Divider R2

//Battery Voltage Correction Factor
#define MV_batCorrFactor 0.0;

//Motor Ramping Rates (adjust for desired acceleration/deceleration feel)
//Higher values = faster/more aggressive, Lower values = smoother/gentler
#define MV_accelerationRate 3;    // Speed increase per update cycle (default: 3)
#define MV_decelerationRate 5;    // Speed decrease per update cycle (default: 5)
#define MV_rampUpdateInterval 10; // Update interval in milliseconds (default: 10ms = 100Hz)
*/

//***BUGGY Variables***
/*
//Voltage Divider Resistors:
#define MV_R1 101600; //Voltage divider R1
#define MV_R2 41600; //Voltage Divider R2

//Steering Variables:
#define MV_servoMin 1225;
#define MV_servoMax 2025;

//Battery Voltage Correction Factor
#define MV_batCorrFactor 0.00;

//Motor Ramping Rates (adjust for desired acceleration/deceleration feel)
//Higher values = faster/more aggressive, Lower values = smoother/gentler
#define MV_accelerationRate 3;    // Speed increase per update cycle (default: 3)
#define MV_decelerationRate 5;    // Speed decrease per update cycle (default: 5)
#define MV_rampUpdateInterval 10; // Update interval in milliseconds (default: 10ms = 100Hz)
*/
