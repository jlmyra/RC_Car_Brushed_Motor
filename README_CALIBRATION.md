# Steering Servo Calibration Tool

A standalone Arduino sketch for calibrating RC servo steering endpoints on ESP32-based RC car controllers.

## Overview

This tool helps you find the precise microsecond pulse width values for your steering servo's left and right endpoints, ensuring perfect wheel alignment and centering. No PS3 controller required!

## Features

- ✅ **Interactive Serial Interface** - Easy-to-use menu system
- ✅ **Real-time Adjustment** - See servo movement as you type
- ✅ **Multiple Input Methods** - Direct values, fine/coarse adjustments
- ✅ **Visual Position Indicator** - Progress bar shows current position
- ✅ **Safety Limits** - Prevents damage from excessive pulse widths
- ✅ **Test Sweep Function** - Verify full range of motion
- ✅ **Auto Center Calculation** - Computes perfect center point
- ✅ **Code Generation** - Copy/paste ready values for your config

## Hardware Requirements

- ESP32 development board (MH ET LIVE ESP32MiniKit or compatible)
- RC servo connected to GPIO 4
- USB cable for serial communication
- 5V power supply for servo (BEC or separate power)

## Software Requirements

- Arduino IDE
- ESP32 Board Support (2.0.17 recommended)
- ESP32Servo library (v3.0.5+)

## Installation

1. **Install ESP32Servo Library**
   ```
   Arduino IDE → Sketch → Include Library → Manage Libraries
   Search: "ESP32Servo" → Install latest version (3.0.9)
   ```

2. **Open Calibration Sketch**
   ```
   File → Open → Steering_Calibration.ino
   ```

3. **Select Board**
   ```
   Tools → Board → ESP32 Arduino → MH ET LIVE ESP32MiniKit
   ```

4. **Upload**
   ```
   Sketch → Upload
   ```

## Usage Guide

### Step 1: Initial Setup

1. Connect your ESP32 to computer via USB
2. Upload the `Steering_Calibration.ino` sketch
3. Open Serial Monitor: `Tools → Serial Monitor`
4. Set baud rate to **115200**
5. Press reset on ESP32 if needed

You should see:
```
╔════════════════════════════════════════════════════════════╗
║     STEERING SERVO CALIBRATION TOOL                       ║
║     ESP32 RC Car Controller                               ║
╚════════════════════════════════════════════════════════════╝

✓ Servo initialized on GPIO 4
```

### Step 2: Find Left Endpoint

Move the servo until the wheels point **full left**:

**Option A - Direct Entry:**
```
> 1200
```
Type the microsecond value and press Enter

**Option B - Fine Adjustment:**
```
> +        (increase by 10µs)
> -        (decrease by 10µs)
```

**Option C - Coarse Adjustment:**
```
> >        (increase by 50µs, use Shift + .)
> <        (decrease by 50µs, use Shift + ,)
```

When wheels are at full left:
```
> l
✓ LEFT endpoint saved: 1225µs
```

### Step 3: Find Right Endpoint

Move the servo until the wheels point **full right**:

```
> 2000
```

When wheels are at full right:
```
> r
✓ RIGHT endpoint saved: 2025µs
```

### Step 4: Verify Calibration

**Show Summary:**
```
> s

╔════════════════════════════════════════════════╗
║        CALIBRATION SUMMARY                     ║
╠════════════════════════════════════════════════╣
║ LEFT endpoint:   1225 µs  ✓                   ║
║ RIGHT endpoint:  2025 µs  ✓                   ║
╠════════════════════════════════════════════════╣
║ Calculated Center: 1625 µs                     ║
║ Total Range:       800 µs                      ║
╠════════════════════════════════════════════════╣
║ Add these to Model_Variables.h:                ║
╠════════════════════════════════════════════════╣
║ #define MV_servoMin 1225                       ║
║ #define MV_servoMax 2025                       ║
╚════════════════════════════════════════════════╝
```

**Test Sweep:**
```
> t

Testing sweep between saved endpoints...
→ Center (1625µs)
→ Full LEFT (1225µs)
→ Center (1625µs)
→ Full RIGHT (2025µs)
→ Center (1625µs)
✓ Test complete
```

Watch your wheels move through the full range to verify correct calibration.

### Step 5: Update Configuration

1. Open `Model_Variables.h` in your main RC car project
2. Find the servo configuration section:
   ```cpp
   //Steering Variables:
   #define MV_servoMin 1225
   #define MV_servoMax 2025
   ```
3. Replace with your calibrated values
4. Save the file
5. Upload your main RC car sketch

## Command Reference

### Position Control

| Command | Description | Example |
|---------|-------------|---------|
| `1500` | Set exact position (any number) | `1500` → 1500µs |
| `+` | Increase by 10µs | Progressive fine adjustment |
| `-` | Decrease by 10µs | Progressive fine adjustment |
| `>` | Increase by 50µs (Shift + .) | Quick coarse adjustment |
| `<` | Decrease by 50µs (Shift + ,) | Quick coarse adjustment |
| `c` | Go to calculated center | Centers between endpoints |

### Calibration Commands

| Command | Description |
|---------|-------------|
| `l` | Save current as LEFT endpoint |
| `r` | Save current as RIGHT endpoint |
| `s` | Show calibration summary |
| `t` | Test sweep between endpoints |

### Other Commands

| Command | Description |
|---------|-------------|
| `h` or `?` | Show help menu |

## Visual Position Indicator

The tool displays a visual representation of the servo position:

```
┌─────────────────────────────────────────────┐
│ Current Position: 1625 µs  [··········█··········]
│ Range: 500µs (min) ← → 2500µs (max)
└─────────────────────────────────────────────┘
```

- `█` = Current position
- `|` = Center point (1500µs)
- `·` = Available range

## Safety Guidelines

### ⚠️ CRITICAL SAFETY WARNINGS

1. **Don't Force the Servo**
   - Never exceed the mechanical limits of your servo
   - If you hear grinding, STOP IMMEDIATELY
   - Reduce the pulse width if servo is struggling

2. **Typical Servo Ranges**
   - Standard RC servos: 1000µs - 2000µs
   - High-end servos: 600µs - 2400µs
   - Cheap servos: May have narrower range

3. **Protection Limits**
   - Tool enforces: 500µs - 2500µs (absolute limits)
   - Start conservative, expand range gradually
   - Don't push to absolute limits unless needed

4. **Power Requirements**
   - Servos draw high current during movement
   - Ensure adequate power supply (BEC or separate)
   - USB power alone may not be sufficient

5. **Mechanical Stops**
   - Steering mechanism may limit range before servo
   - Stop when you feel resistance
   - Don't rely on pulse width alone

## Troubleshooting

### Servo doesn't move

**Check:**
- Is servo connected to GPIO 4?
- Is servo getting 5V power?
- Is servo ground connected to ESP32 ground?
- Try Serial Monitor restart (close/reopen)

**Try:**
```
> 1500      # Standard center position
```

### Servo jitters or makes noise

**Cause:** Insufficient power supply

**Solution:**
- Use external 5V BEC rated for servo current
- Don't power servo from ESP32 3.3V pin
- Add 100µF capacitor near servo power pins

### Wheels don't center properly

**Cause:** Servo arm not properly aligned

**Solution:**
1. Remove servo arm
2. Set servo to calculated center (`c` command)
3. Reattach arm pointing straight
4. Recalibrate endpoints if needed

### Range seems too narrow

**Check:**
- Mechanical linkages may be limiting range
- Verify servo arm length and mounting
- Check for binding in steering mechanism
- May need to adjust servo horn position

### Values keep changing

**Cause:** Servo hitting mechanical stops

**Solution:**
- Reduce range slightly (10-20µs from each end)
- This prevents servo from fighting against stops
- Improves reliability and reduces wear

## Understanding Servo Pulse Widths

### How Servo Control Works

RC servos use PWM (Pulse Width Modulation) at 50 Hz (20ms period):

```
Pulse Width:
1000µs = Full Left
1500µs = Center
2000µs = Full Right

Timing:
|<- 1500µs ->|<- 18500µs low ->|
|____/‾‾‾‾‾‾\_______|_______|    = 20ms total (50 Hz)
```

### Why Calibration Matters

1. **Servo Variation**
   - Different servos have different ranges
   - Manufacturing tolerances affect endpoints
   - Age and wear change characteristics

2. **Mechanical Setup**
   - Servo arm position affects throw
   - Linkage geometry changes effective range
   - Wheel alignment depends on endpoints

3. **Centering Accuracy**
   - Perfect center = (left + right) / 2
   - Off-center = steering pulls to one side
   - Drift affects straight-line tracking

## Example Calibration Session

```
> 1500                          # Start at typical center
> +                             # Adjust right
> +
> +
> 1650                          # Jump to specific value
> l                             # Save as LEFT
✓ LEFT endpoint saved: 1650µs

> 2200                          # Try right side
> -                             # Too far, back off
> -
> 2100                          # Just right
> r                             # Save as RIGHT
✓ RIGHT endpoint saved: 2100µs

> s                             # Show summary
╔════════════════════════════════════════════════╗
║ LEFT endpoint:   1650 µs  ✓                   ║
║ RIGHT endpoint:  2100 µs  ✓                   ║
║ Calculated Center: 1875 µs                     ║
║ Total Range:       450 µs                      ║
╚════════════════════════════════════════════════╝

> t                             # Test sweep
→ Full LEFT (1650µs)
→ Full RIGHT (2100µs)
→ Center (1875µs)
✓ Test complete

> s                             # Get code to copy
Add these to Model_Variables.h:
#define MV_servoMin 1650
#define MV_servoMax 2100
```

## Tips & Best Practices

### 1. Start Conservative
- Begin near standard values (1000-2000µs)
- Expand range gradually
- Stop at first sign of mechanical resistance

### 2. Fine-tune Center
- Use `c` command to check calculated center
- Wheels should point straight ahead
- If not, adjust endpoints symmetrically

### 3. Account for Expo Curve
- Main code applies exponential curve to inputs
- Center position is most important
- Endpoints should allow full mechanical range

### 4. Document Your Values
- Write down calibrated values
- Note which servo/vehicle they're for
- Keep backup of working configurations

### 5. Periodic Recalibration
- Servos wear over time
- Gears can develop slop
- Recalibrate every 6-12 months or if drift occurs

## Integration with Main Code

After calibration, your `Model_Variables.h` should contain:

```cpp
//Steering Variables:
#define MV_servoMin 1225   // Your calibrated LEFT value
#define MV_servoMax 2025   // Your calibrated RIGHT value
```

The main RC car code uses these values:
```cpp
// In Steering_Control.ino
steerServo.attach(servoPin, servoMin, servoMax);

// Maps joystick (-128 to +128) to servo range
int joystick_Pos = map(curvedPosition, -128, 128, servoMin, servoMax);
steerServo.write(joystick_Pos);
```

## Technical Details

### GPIO Configuration
- **Pin:** GPIO 4 (matches main project)
- **PWM Frequency:** 50 Hz (standard RC servo)
- **Timer:** Uses ESP32PWM library timers 0-3

### Pulse Width Ranges
- **Safety Limits:** 500µs - 2500µs (enforced by tool)
- **Standard Servos:** 1000µs - 2000µs (1ms - 2ms)
- **Wide Range:** Some servos support 600µs - 2400µs

### Memory Usage
- **Flash:** ~15 KB
- **RAM:** <1 KB
- **No EEPROM:** Values not saved on power cycle

## Version History

### v1.0 (December 2025)
- Initial release
- Interactive serial interface
- Visual position indicator
- Test sweep function
- Code generation output

## Credits

- **Author:** Claude Code
- **Project:** ESP32 RC Car Brushed Motor Controller
- **License:** Open Source
- **Date:** December 2025

## Related Documentation

- [MIGRATION_NOTES.md](MIGRATION_NOTES.md) - ESP32 Core migration guide
- [Model_Variables.h](Model_Variables.h) - Configuration file
- [Steering_Control.ino](Steering_Control.ino) - Main steering code

## Support

If you encounter issues:

1. Check wiring connections
2. Verify power supply adequacy
3. Review safety guidelines
4. Consult troubleshooting section
5. Check ESP32Servo library version

## Contributing

Suggestions for improvement:
- EEPROM storage of calibration values
- Multiple servo profiles
- Automated endpoint detection
- Graphical calibration interface

---

**⚠️ Remember:** Always prioritize safety! If anything feels wrong, stop immediately and reassess.

**Happy Calibrating! 🎮🚗**
