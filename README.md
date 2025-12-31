# ESP32 RC Car Brushed Motor Controller

ESP32-based RC car controller with PS3 gamepad support, featuring modular architecture, advanced safety features, and performance optimizations.

## Version 2.1 - Enhanced Safety & Code Quality (December 2025)

### 🆕 What's New in v2.1

#### 🔴 Critical Safety Enhancements

**1. PS3 Controller Disconnect Protection**
- **Emergency stop on disconnect**: All motors immediately stop when controller connection is lost
- **Prevents runaway vehicle**: Ensures safe shutdown if connection drops unexpectedly
- **Clear status messages**: Serial monitor displays disconnect warnings
- **Hardware protection**: Direct GPIO writes ensure motors stop even if main loop is blocked

**2. Servo Position Validation**
- **Added `constrain()` to steering output**: Prevents servo from receiving out-of-range values
- **Protects mechanical components**: Ensures servo stays within safe calibrated endpoints
- **Prevents servo damage**: Guards against map() function edge cases

**3. Enhanced Startup Validation**
- **Professional startup banner**: Clean, formatted initialization messages
- **PS3 initialization verification**: System halts with clear error if controller init fails
- **Version display**: Shows firmware version on startup (v2.1)
- **Hardware checks**: Validates critical systems before allowing operation

**4. Connection Watchdog** (from v2.0)
- Monitors controller connectivity with 500ms timeout
- Automatically stops motors if no controller events received
- Prevents runaway vehicle if PS3 connection drops during operation
- Updates activity timer on every controller event

**5. Motor Speed Bounds Checking** (from v2.0)
- Multiple `constrain()` checks limit motor speed to valid 0-255 range
- Prevents integer overflow and undefined behavior
- Ensures motor controller receives valid PWM values
- Protects hardware from invalid speed commands

#### 🎯 New Features

**Steering Sweep Animation on Connection**
- **Visual confirmation**: Wheels automatically sweep left-right-center when controller connects
- **System verification**: Confirms steering servo is responding correctly
- **User feedback**: Provides tactile confirmation that system is ready to drive
- **Timing**: Center (200ms) → Full left (400ms) → Full right (400ms) → Center (200ms)
- **Serial output**: Shows sweep progress in serial monitor

#### 🛠️ Code Quality Improvements

**1. Eliminated Magic Numbers**
- **New constants in Model_Variables.h**:
  - `MV_JOYSTICK_DEADZONE` - Centralized joystick deadzone threshold (replaces hardcoded `2`)
  - `MV_WINCH_SPEED_FACTOR` - Winch speed multiplier (replaces hardcoded `0.8`)
- **Easier tuning**: Change values in one place instead of searching through code
- **Better maintainability**: Clear, descriptive constant names
- **Consistent behavior**: Ensures all modules use same thresholds

**2. Improved Code Organization**
- All safety-critical code clearly marked with `// SAFETY:` comments
- Consistent formatting across all modules
- Enhanced documentation in comments
- Version tracking in main file header

**3. External Reference Management**
- Proper `extern` declarations for cross-module communication
- Emergency stop can access motor control variables safely
- Clean separation of concerns between modules

### ⚙️ Configuration & Setup

#### Hardware Requirements
- **Board**: MH ET LIVE ESP32MiniKit (or compatible ESP32)
- **Motor Controller**: Cytron MD-13S (or compatible brushed motor controller)
- **Servo**: Standard RC servo (50 Hz PWM)
- **Controller**: PS3 DualShock 3 Gamepad
- **Power**: 2S LiPo (7.4V nominal) or equivalent

#### Software Requirements
- **ESP32 Core**: 2.0.17 (recommended - see MIGRATION_NOTES.md)
- **ESP32Servo Library**: 3.0.9
- **Ps3Controller Library**: Latest version
- **Arduino IDE**: 1.8.x or 2.x

#### Configuration File: Model_Variables.h

All tunable parameters are centralized in `Model_Variables.h`:

```cpp
//Debug Mode
#define MV_DEBUG_MODE 1  // 1 = Debug output enabled, 0 = No debug output

//Connection Watchdog
#define MV_CONNECTION_TIMEOUT 500  // Milliseconds - emergency stop if no events

//Motor Speed Limits
#define MV_SPEED_LIMIT_NORMAL 255  // Maximum motor speed in normal mode
#define MV_SPEED_LIMIT_CRAWL 128   // Maximum motor speed in crawl mode

//Steering Expo (0.0 = linear, higher = more exponential)
#define MV_STEERING_EXPO 0.3  // 0.0-1.0, recommended: 0.2-0.5

//Battery Protection
#define MV_LOW_VOLTAGE_CUTOFF 6.5     // Volts - limit power below this
#define MV_LOW_VOLTAGE_POWER_LIMIT 0.5 // Multiply motor speed by this when low

//Speed Profiles
#define MV_SPEED_CRAWL 0.3   // Crawl mode - precise control
#define MV_SPEED_NORMAL 0.6  // Normal mode - balanced
#define MV_SPEED_SPORT 0.8   // Sport mode - spirited
#define MV_SPEED_RACE 1.0    // Race mode - full power (L1 button)

//Motor Expo
#define MV_MOTOR_EXPO 0.2  // 0.0-1.0, recommended: 0.1-0.3

//Steering Variables
#define MV_servoMin 1225  // Your calibrated LEFT endpoint
#define MV_servoMax 2025  // Your calibrated RIGHT endpoint

//Motor Ramping Rates
#define MV_accelerationRate 3    // Speed increase per cycle (higher = faster)
#define MV_decelerationRate 5    // Speed decrease per cycle
#define MV_rampUpdateInterval 10 // Update interval in ms (10ms = 100Hz)

//Joystick Deadzone
#define MV_JOYSTICK_DEADZONE 2   // Ignore joystick values within ±2

//Winch Control
#define MV_WINCH_SPEED_FACTOR 0.8  // Winch button pressure multiplier
```

### 📋 Module Architecture

The code is organized into modular `.ino` files:

1. **RC_Car_Brushed_Motor.ino** - Main setup, loop, event dispatcher
2. **Motor_Control.ino** - Drive motor PWM control with smooth ramping
3. **Steering_Control.ino** - Servo-based steering with exponential response
4. **Winch_Control.ino** - Winch motor control for up/down operation
5. **Battery_Monitor.ino** - Battery voltage monitoring and LED status
6. **Connection_Handler.ino** - PS3 connection events and steering sweep
7. **Model_Variables.h** - Vehicle-specific configuration constants

### 🎮 PS3 Controller Mapping

```
╔═══════════════════════════════════════════════════╗
║  LEFT JOYSTICK          RIGHT JOYSTICK            ║
║                                                   ║
║       Forward                                     ║
║        -128                                       ║
║         /\                      /\                ║
║      <  ly  >            Left < rx > Right        ║
║         \/               -128     +128            ║
║        +128                                       ║
║       Reverse                                     ║
║                                                   ║
║  Motion Control         Steering Control         ║
╚═══════════════════════════════════════════════════╝

BUTTONS:
  L1 - Nitro/Turbo Mode (activates RACE speed profile)
  D-Pad UP - Winch Unwind
  D-Pad DOWN - Winch Rewind
```

### 🔧 Steering Calibration

Use the included **Steering_Calibration.ino** tool to find your servo's precise endpoints:

1. Upload `Steering_Calibration.ino` to your ESP32
2. Open Serial Monitor (115200 baud)
3. Use interactive commands to find left and right endpoints
4. Update `MV_servoMin` and `MV_servoMax` in `Model_Variables.h`

See [README_CALIBRATION.md](README_CALIBRATION.md) for detailed calibration guide.

### 📊 Performance & Safety Features

#### Real-Time Performance
- **Motor Update Rate**: 100 Hz (10ms intervals)
- **Battery Monitoring**: 500 samples/sec averaged every 1 second
- **Controller Watchdog**: 500ms timeout for safety
- **Servo Frequency**: 50 Hz (standard RC servo)
- **Motor PWM Frequency**: 30 kHz (smooth, silent operation)

#### Safety Systems
1. ✅ **Connection watchdog** - Emergency stop if no controller events
2. ✅ **Disconnect protection** - Immediate motor shutdown on disconnect
3. ✅ **Bounds checking** - Motor speed and servo position validation
4. ✅ **Battery protection** - Reduced power below voltage threshold
5. ✅ **Startup validation** - System halts if initialization fails
6. ✅ **Direction change safety** - Smooth ramping through zero before reversing

### 🚀 Startup Sequence

When you power on the system:

```
╔════════════════════════════════════════════════╗
║  ESP32 RC CAR CONTROLLER v2.1                 ║
║  Initializing systems...                      ║
╚════════════════════════════════════════════════╝

Initializing PS3 controller... ✓ OK

***********************************************************
***Initialization finished. Ready to pair PS3 controller***
***********************************************************

✓ All systems initialized
✓ Safety features active:
  - Connection watchdog enabled
  - Motor speed bounds checking
  - Battery voltage protection
```

When PS3 controller connects:

```
********************************
*** PS3 Controller Connected ***
********************************

→ Performing steering sweep test...
✓ Steering sweep complete - ready to drive!
```

### 📚 Documentation

- **MIGRATION_NOTES.md** - ESP32 Core 2.x vs 3.x compatibility guide
- **README_CALIBRATION.md** - Steering servo calibration tool guide
- **Citations.ino** - Code attribution and references

### 🔄 Upgrading from v2.0

1. Replace all `.ino` files with v2.1 versions
2. Update `Model_Variables.h` with new constants (if not already present):
   - `MV_JOYSTICK_DEADZONE`
   - `MV_WINCH_SPEED_FACTOR`
3. No wiring changes required
4. Upload to ESP32
5. Optionally run steering calibration tool if servo endpoints changed

### ⚡ Version 2.0 Performance Optimizations (Retained in v2.1)

**1. ADC Calibration Optimization (500x Improvement)**
- Moved ADC characterization to one-time initialization in setup
- **Before**: Calibration executed 500 times per second
- **After**: Calibration executed once at startup
- **Impact**: 99.8% reduction in calibration overhead

**2. Debug Mode Macros**
- `DEBUG_MODE` flag for conditional compilation
- `DEBUG_PRINT()` and `DEBUG_PRINTLN()` macros
- Set `MV_DEBUG_MODE 0` for production to eliminate debug overhead
- Significant performance gain when disabled

**3. Simplified Control Logic**
- Single ternary operator for L1 nitro logic
- Consolidated steering control blocks
- Reduced code duplication by ~50%
- More efficient compiled code

**4. Named Constants**
- All magic numbers replaced with descriptive constants
- Centralized configuration in Model_Variables.h
- Easier to tune and maintain

### 🐛 Bug Fixes (v2.0-2.1)

- ✅ Fixed winch control - proper motor stop when button released
- ✅ Fixed unreachable digitalWrite calls in winch control
- ✅ Removed unused variables and dead code
- ✅ Fixed servo frequency (was 300 Hz, now correctly 50 Hz)
- ✅ Added proper external references for cross-module communication

### 🎯 Future Enhancement Ideas

- [ ] PID control for even smoother motor response
- [ ] Telemetry data logging (speed, battery history, position tracking)
- [ ] Support for different motor controller types
- [ ] Over-the-air (OTA) firmware updates
- [ ] IMU integration for stability control and tilt detection
- [ ] Multi-vehicle support with selectable profiles
- [ ] Configurable speed curves (exponential vs linear)
- [ ] Web interface for configuration and monitoring
- [ ] EEPROM storage of calibration values

### 🙏 Credits

- **Original Code**: Based on community ESP32 RC car projects
- **Enhancements**: Implemented with Claude Code (Anthropic)
- **Date**: December 2025
- **License**: Open Source

### 🔗 Related Projects

- ESP32 Arduino Core: https://github.com/espressif/arduino-esp32
- ESP32Servo Library: https://github.com/madhephaestus/ESP32Servo
- Ps3Controller Library: https://github.com/jvpernis/esp32-ps3

---

**Version History:**
- **v2.1** (December 2025) - Enhanced safety features and code quality
- **v2.0** (November 2025) - Performance optimizations and modular architecture
- **v1.0** (Earlier) - Initial implementation

*For technical support, see documentation files or raise an issue.*
