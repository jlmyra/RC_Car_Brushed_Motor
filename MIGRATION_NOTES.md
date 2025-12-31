# ESP32 RC Car - Migration & Troubleshooting Notes

**Project:** RC Car Brushed Motor Controller
**Board:** MH ET LIVE ESP32MiniKit
**Date:** December 2025
**Author:** Enhanced with Claude Code

---

## Current Working Configuration ✅

| Component | Version | Status |
|-----------|---------|--------|
| **ESP32 Arduino Core** | 2.0.17 | ✅ Recommended |
| **ESP32Servo Library** | 3.0.9 | ✅ Compatible |
| **PS3Controller Library** | Latest | ✅ Working |
| **Compilation** | Success | ✅ No errors |
| **Runtime** | Stable | ✅ Fully functional |

---

## Critical Issues Discovered & Fixed

### 1. Servo Frequency Configuration (CRITICAL - Hardware Damage Risk!)

**Problem:**
```cpp
steerServo.setPeriodHertz(300);  // ❌ WRONG - 6x too fast!
```

**Impact:**
- Servo receives PWM pulses 6x faster than designed (300 Hz vs 50 Hz)
- Servo constantly fights itself trying to adjust
- Excessive current draw (can exceed 2A)
- Overheating
- **BEC (Battery Elimination Circuit) damage/failure**
- Potential permanent servo damage

**Solution:**
```cpp
steerServo.setPeriodHertz(50);   // ✅ CORRECT - Standard RC servo frequency
```

**Why 50 Hz?**
- RC servos expect 50 Hz (20ms period)
- PWM pulse width: 1000-2000µs determines position
- This is an industry standard for hobby servos

---

### 2. Servo Write Method Mismatch

**Problem:**
```cpp
int joystick_Pos = map(curvedPosition, -128, 128, servoMin, servoMax); // Returns 1225-2025
steerServo.write(joystick_Pos);  // ❌ Treats 1225 as DEGREES, not microseconds!
```

**Impact:**
- `write()` expects degrees (0-180)
- Passing microseconds (1225-2025) as degrees causes invalid servo positions
- Servo sends corrupted PWM signals
- Interference with motor control
- Servo doesn't respond to joystick input

**Solution Option A - Use writeMicroseconds():**
```cpp
int joystick_Pos = map(curvedPosition, -128, 128, servoMin, servoMax); // 1225-2025µs
steerServo.writeMicroseconds(joystick_Pos);  // ✅ CORRECT
```

**Solution Option B - Convert to degrees (Current Implementation):**
```cpp
int joystick_Pos = map(curvedPosition, -128, 128, servoMin, servoMax); // 1225-2025µs
steerServo.write(joystick_Pos);  // Works when attach() uses microsecond calibration
```

**Note:** When using `attach(pin, min_us, max_us)`, the library internally handles microsecond-to-degree conversion, so `write()` can accept the mapped values directly.

---

### 3. ESP32 Arduino Core Breaking Changes (2.x → 3.x)

**The Problem:**
ESP32 Arduino Core 3.0+ completely removed the LEDC PWM API used in version 2.x.

#### API Comparison

| Function | Core 2.x | Core 3.x |
|----------|----------|----------|
| **Setup PWM** | `ledcSetup(channel, freq, resolution)` | Removed ❌ |
| **Attach Pin** | `ledcAttachPin(pin, channel)` | Removed ❌ |
| **Write PWM** | `ledcWrite(channel, duty)` | `ledcWrite(pin, duty)` ⚠️ Changed |
| **New Function** | N/A | `ledcAttach(pin, freq, resolution)` ✅ New |

#### Code Migration Example

**ESP32 Core 2.x (Working):**
```cpp
void setupMotorControl() {
  pinMode(driveMotorPin, OUTPUT);
  pinMode(driveMotorDirection, OUTPUT);

  ledcSetup(driveMotorChannel, freq, resolution);      // Configure channel 14
  ledcAttachPin(driveMotorPin, driveMotorChannel);     // Attach GPIO25 to channel 14
}

void updateMotorRamping() {
  ledcWrite(driveMotorChannel, finalMotorSpeed);       // Write to channel
}
```

**ESP32 Core 3.x (Attempted - Has Issues):**
```cpp
void setupMotorControl() {
  pinMode(driveMotorPin, OUTPUT);
  pinMode(driveMotorDirection, OUTPUT);

  ledcAttach(driveMotorPin, freq, resolution);         // Auto-assigns channel
  ledcWrite(driveMotorPin, 0);                         // Initialize to zero
}

void updateMotorRamping() {
  ledcWrite(driveMotorPin, finalMotorSpeed);           // Write to pin directly
}
```

---

### 4. PWM Channel Conflicts

**The Issue:**
- ESP32 has 16 PWM channels (0-15)
- ESP32Servo library reserves channels (typically 0-1 for servos)
- Motor/Winch LEDC functions must use different channels
- ESP32 Core 3.x auto-allocation can cause conflicts

**Core 2.x Solution (Manual Channel Assignment):**
```cpp
// In Motor_Control.ino
uint8_t driveMotorChannel = 14;  // Explicitly use channel 14

// In Winch_Control.ino
uint8_t winchPWMChannel_1 = 4;   // Explicitly use channel 4
uint8_t winchPWMChannel_2 = 2;   // Explicitly use channel 2
```

**Core 3.x Challenge:**
- Channels are auto-assigned
- Need to ensure servo library claims channels first
- Initialize servo BEFORE motor/winch to avoid conflicts

---

## Why Stay on ESP32 Core 2.0.17?

### ✅ Advantages of Core 2.0.17
1. **Proven Stability** - Code works perfectly, tested and verified
2. **No Migration Required** - Save development time
3. **Explicit Channel Control** - Predictable PWM channel assignment
4. **Compatible with ESP32Servo 3.0.9** - Latest servo library works fine
5. **No Breaking Changes** - Future updates within 2.x are backward compatible

### ❌ Disadvantages of Core 3.x Migration
1. **API Breaking Changes** - All LEDC code must be rewritten
2. **Channel Conflict Risk** - Auto-allocation less predictable
3. **Testing Required** - Need to verify all PWM functions work correctly
4. **No Critical Features** - Core 3.x doesn't add features needed for this project
5. **Compilation Errors** - Old API functions completely removed

### Core 3.x Benefits (Not Critical for This Project)
- Improved WiFi performance
- Better BLE support
- USB-OTG support
- Updated ESP-IDF base (4.4.x → 5.x)

**None of these features are used in this RC car controller.**

---

## Migration Path to Core 3.x (If Needed in Future)

### Step 1: Update Code

**Motor_Control.ino:**
```cpp
// BEFORE (Core 2.x):
ledcSetup(driveMotorChannel, freq, resolution);
ledcAttachPin(driveMotorPin, driveMotorChannel);

// AFTER (Core 3.x):
ledcAttach(driveMotorPin, freq, resolution);

// Update all ledcWrite calls:
ledcWrite(driveMotorPin, finalMotorSpeed);  // Use pin, not channel
```

**Winch_Control.ino:**
```cpp
// BEFORE (Core 2.x):
ledcSetup(winchPWMChannel_1, freq, resolution);
ledcAttachPin(winchDirection_1, winchPWMChannel_1);

// AFTER (Core 3.x):
ledcAttach(winchDirection_1, freq, resolution);

// Update all ledcWrite calls:
ledcWrite(winchDirection_1, speed);  // Use pin, not channel
```

### Step 2: Manage Initialization Order

**RC_Car_Brushed_Motor.ino:**
```cpp
void setup() {
  // ... PS3 initialization ...

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // IMPORTANT: Initialize servo FIRST
  setupSteeringControl();  // Reserves PWM channels 0-1

  // Then initialize motor/winch (will use higher channels)
  setupBatteryMonitor();
  setupMotorControl();
  setupWinchControl();
}
```

### Step 3: Remove Channel Variables (Optional)

Since Core 3.x auto-assigns channels, you can remove:
```cpp
uint8_t driveMotorChannel = 14;    // No longer needed
uint8_t winchPWMChannel_1 = 4;     // No longer needed
uint8_t winchPWMChannel_2 = 2;     // No longer needed
```

### Step 4: Testing Checklist

- [ ] Code compiles without errors
- [ ] Servo responds correctly to right joystick
- [ ] Servo doesn't overheat (should be cool to touch)
- [ ] Drive motor responds to left joystick
- [ ] Winch responds to up/down buttons
- [ ] No motor activation when moving steering
- [ ] L1 turbo mode works
- [ ] Battery monitoring functions
- [ ] No abnormal current draw
- [ ] Extended runtime test (10+ minutes)

---

## Hardware Specifications

### Servo Configuration
```cpp
// Steering Servo
int servoPin = 4;              // GPIO4
int servoMin = 1225;           // Microseconds (full left)
int servoMax = 2025;           // Microseconds (full right)
int servoCenter = 1625;        // Calculated: (servoMin + servoMax) / 2
steerServo.setPeriodHertz(50); // Standard 50 Hz (20ms period)
```

### Motor Driver (Cytron MD-13S)
```cpp
int driveMotorPin = 25;        // GPIO25 - PWM signal (white wire)
int driveMotorDirection = 27;  // GPIO27 - Direction (yellow wire)
uint32_t freq = 30000;         // 30 kHz PWM frequency
uint8_t resolution = 8;        // 8-bit resolution (0-255)
```

### Winch Motor (Dual Channel H-Bridge)
```cpp
int winchDirection_1 = 26;     // GPIO26 - A1A (Unwind)
int winchDirection_2 = 18;     // GPIO18 - A1B (Rewind)
uint32_t freq = 30000;         // 30 kHz PWM frequency
uint8_t resolution = 8;        // 8-bit resolution
```

---

## Troubleshooting Guide

### Symptom: Steering servo doesn't respond

**Check:**
1. Is servo frequency set to 50 Hz? ✓
2. Is servo getting power (5V)? ✓
3. Is GPIO4 connected correctly? ✓
4. Does debug output show steering events? ✓
5. Is `write()` or `writeMicroseconds()` being used correctly? ✓

**Debug Code:**
```cpp
void handleSteeringControl() {
  if(abs(Ps3.event.analog_changed.stick.rx) > 2) {
    Serial.print("Steering event: rx=");
    Serial.print(Ps3.data.analog.stick.rx);
    Serial.print(" servo_us=");
    Serial.println(joystick_Pos);

    steerServo.write(joystick_Pos);
  }
}
```

### Symptom: Servo overheating / high current draw

**Most likely cause:** Frequency set too high (300 Hz instead of 50 Hz)

**Fix:**
```cpp
steerServo.setPeriodHertz(50);  // Must be 50 Hz for RC servos
```

### Symptom: Motor activates when moving steering joystick

**Root cause:** Corrupted servo PWM signals causing electrical interference

**Check:**
1. Servo frequency = 50 Hz ✓
2. Using correct write method (degrees or microseconds) ✓
3. Servo and motor have separate power/ground connections ✓

### Symptom: Code won't compile - "ledcSetup not declared"

**Cause:** Using ESP32 Core 3.x with Core 2.x code

**Solution 1 (Recommended):** Downgrade to ESP32 Core 2.0.17
**Solution 2:** Update all LEDC code to Core 3.x API (see migration guide above)

---

## Performance Optimizations

### Motor Ramping (Smooth Acceleration/Deceleration)
```cpp
int accelerationRate = 3;      // Speed increase per 10ms cycle
int decelerationRate = 5;      // Speed decrease per 10ms cycle
unsigned long rampUpdateInterval = 10; // Update every 10ms (100 Hz)
```

**Tuning:**
- Higher `accelerationRate` = more aggressive acceleration
- Higher `decelerationRate` = faster braking
- Lower `rampUpdateInterval` = smoother but more CPU usage

### Exponential Throttle Curves
```cpp
float motorExpo = 0.2;         // Motor throttle expo (0.0 = linear, 1.0 = very exponential)
float steeringExpo = 0.3;      // Steering expo
```

**Effect:**
- Expo > 0 gives more precision at center/low values
- Expo = 0 gives linear response
- Recommended range: 0.1 - 0.5

---

## Safety Features

### Connection Watchdog
```cpp
unsigned long connectionTimeout = 500;  // 500ms - emergency stop if no events
```

### Battery Protection
```cpp
#define MV_LOW_VOLTAGE_CUTOFF 6.5         // Volts
#define MV_LOW_VOLTAGE_POWER_LIMIT 0.5    // 50% power when low
```

### Speed Bounds Checking
```cpp
targetMotorSpeed = constrain(targetMotorSpeed, 0, 255);   // Before ramping
currentMotorSpeed = constrain(currentMotorSpeed, 0, 255); // After ramping
finalMotorSpeed = constrain(finalMotorSpeed, 0, 255);     // Before output
```

---

## Quick Reference

### Downgrade to ESP32 Core 2.0.17
1. Arduino IDE → Tools → Board → Boards Manager
2. Search "ESP32"
3. Find "esp32 by Espressif Systems"
4. Click dropdown → Select **2.0.17**
5. Click "Install"
6. Restart Arduino IDE

### Verify Installation
```
Tools → Board → ESP32 Arduino → MH ET LIVE ESP32MiniKit
Tools → Get Board Info → Should show Core 2.0.17
```

### Library Versions
```
Sketch → Include Library → Manage Libraries
Search: "ESP32Servo" → Should show v3.0.9 installed
Search: "PS3 Controller Host" → Latest version
```

---

## Contact & Support

**Original Code:** Based on community RC car projects
**Enhanced:** December 2025 with Claude Code
**GitHub Issues:** [Your repository URL]

---

## Changelog

### December 2025 - Migration Analysis
- ✅ Fixed servo frequency (300 Hz → 50 Hz)
- ✅ Fixed servo write method compatibility
- ✅ Documented ESP32 Core 2.x vs 3.x differences
- ✅ Established stable configuration (Core 2.0.17)
- ✅ Created comprehensive migration notes

### Future Enhancements (Optional)
- [ ] Migrate to ESP32 Core 3.x when needed
- [ ] Add support for multiple speed profiles via button combinations
- [ ] Implement telemetry data logging
- [ ] Add OTA (Over-The-Air) update capability

---

**Last Updated:** December 30, 2025
**Status:** ✅ Stable and fully functional on ESP32 Core 2.0.17
