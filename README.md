# 3-Axis Robot Arm Controller

Minimalist control software for a 3-axis robot arm using Arduino Mega 2560.

## Hardware Setup

- 3 NEMA 17 stepper motors (base, shoulder, slide)
- 1 servo at the end effector for level positioning
- 3 limit switches for homing
- Arduino Mega 2560

### Pin Connections

#### Motors
- Base: STEP=22, DIR=23, ENABLE=24, LIMIT=25
- Shoulder: STEP=26, DIR=27, ENABLE=28, LIMIT=29
- Slide: STEP=30, DIR=31, ENABLE=32, LIMIT=33
- Servo: PIN=9

## Commands

### Movement
- `base X` - Move base to X degrees
- `shoulder X` - Move shoulder to X degrees
- `slide X` - Move slide to X mm

### Homing
- `home base` - Home base axis
- `home shoulder` - Home shoulder axis
- `home slide` - Home slide axis
- `home all` - Home all axes

### Servo Control
- `servo X` - Set servo to X degrees (0-180)
- `servo on` - Enable servo auto-leveling
- `servo off` - Disable and detach servo

### System
- `status` - Show current position
- `stop` - Emergency stop
- `resume` - Resume after emergency stop

## Notes

- Motors are disabled when not in motion to prevent overheating
- The arm may fall due to gravity when motors are disabled
- Always home before using precise movements
- Servo auto-leveling uses the formula: servo_angle = 90 - shoulder_angle
- If arm doesn't move, check if it's in emergency stop mode

## Mechanical Considerations

Since the motors disable after movement to prevent overheating, you'll need a mechanical solution to prevent the arm from falling:

1. **Counterweights**: Add counterweights to balance the arm
2. **Friction joints**: Design joints with enough friction to hold position without power
3. **External brakes**: Add mechanical brakes that can be manually engaged
4. **Worm gears**: Replace motor gearing with worm gears that can't be back-driven

These mechanical solutions are more reliable and energy-efficient than keeping motors powered.