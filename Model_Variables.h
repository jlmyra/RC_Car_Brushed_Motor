

//***JEEP Variables***

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
