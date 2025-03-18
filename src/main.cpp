#include <AccelStepper.h>
#include <MAX6675.h>  // Add library for K-type thermocouple

// Pin definitions (unchanged)
const int V_LIMIT_SWITCH_PIN = 18;
const int V_STEP_PIN = 22;
const int V_DIR_PIN = 24;
const int V_ENABLE_PIN = 26;

const int H_LIMIT_SWITCH_PIN = 19;
const int H_STEP_PIN = 23;
const int H_DIR_PIN = 25;
const int H_ENABLE_PIN = 27;

// Thermocouple pins (new)
const int THERMO_SCK_PIN = 6;  // Serial clock
const int THERMO_CS_PIN = 5;   // Chip select
const int THERMO_SO_PIN = 7;   // Serial out

// Motor and mechanical parameters (unchanged)
const float STEPS_PER_MM = 50.0;
const int STEPS_PER_REVOLUTION = 400;
const int GEARBOX_RATIO = 1;
const int MAX_SPEED = 800;
const int ACCELERATION = 800;

// Rotary encoder pins (unchanged)
const int ENCODER_CLK_PIN = 2;
const int ENCODER_DT_PIN = 21;
const int ENCODER_SW_PIN = 4;

// Timing constants (unchanged)
const unsigned long DEBOUNCE_DELAY = 10;
const unsigned long REPORT_INTERVAL = 500;
const unsigned long LIMIT_SWITCH_DEBOUNCE = 50;

// Grid parameters (new)
const float X_MIN = 90.0;    // Start X position (mm)
const float X_MAX = 110.0;   // End X position (mm)
const float Y_MIN = 150.0;    // Start Y position (mm)
const float Y_MAX = 200.0;   // End Y position (mm)
const float STEP_SIZE = 1.0; // Step size (mm)

// Thermocouple Constants
#define TC_PIN A0          // set to ADC pin used
#define AREF 3.3           // set to AREF, typically board voltage like 3.3 or 5.0
#define ADC_RESOLUTION 10

// System state variables (existing + new)
volatile int encoderPosition = 0;   // Encoder position
int lastEncoderPosition = 0;        // Last encoder position for comparison
volatile int lastEncoderCLK;        // Last state of encoder CLK pin
bool controllingVertical = true;    // Flag to switch between vertical/horizontal control
unsigned long lastReportTime = 0;   // Timestamp for position reporting
bool isScanning = false;     // Flag to indicate scanning mode (new)
float currentX = X_MIN;      // Current X position during scan (new)
float currentY = Y_MIN;      // Current Y position during scan (new)



// MotorSystem class (mostly unchanged, ISR adjusted)
class MotorSystem {
public:
  const int limitSwitchPin;
  const int enablePin;
  const int directionPin;
  const float stepsPerMM;
  const bool invertSwitchLogic;
  bool limitSwitchTriggered;
  AccelStepper stepper;
  bool isHomed;
  volatile bool limitHit = false;  // Per-instance limit hit flag

  // Constructor made public
  MotorSystem(int dirPin, int stepPin, int enablePin, int limitPin, float stepsPerMM, bool invertSwitch = false)
    : stepper(AccelStepper::DRIVER, stepPin, dirPin),
      limitSwitchPin(limitPin),
      enablePin(enablePin),
      directionPin(dirPin),
      stepsPerMM(stepsPerMM),
      isHomed(false),
      limitSwitchTriggered(false),
      invertSwitchLogic(invertSwitch) {
    pinMode(limitSwitchPin, INPUT_PULLUP);
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW);
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(ACCELERATION);
  }

  // Public methods
  void enable(bool enabled) {
    digitalWrite(enablePin, enabled ? LOW : HIGH);
  }

  float getCurrentPositionMM() {
    return stepper.currentPosition() / stepsPerMM;
  }

  void setTargetPositionMM(float mm) {
    stepper.moveTo(mm * stepsPerMM);
  }

  bool spinMotor(int steps, bool safetyOn = false) {
    limitHit = false;  // Reset limit hit flag
    digitalWrite(directionPin, (steps > 0) ? LOW : HIGH);
    stepper.move(steps);

    while (stepper.distanceToGo() != 0) {
      stepper.run();
      if (safetyOn && limitHit) {
        return false;  // Stop if limit is hit and safety is on
      }
    }
    return true;
  }

  bool moveToHome() {
    Serial.println("Initial move away from limit");
    stepper.setMaxSpeed(MAX_SPEED / 2);
    stepper.setAcceleration(ACCELERATION / 2);
    stepper.move(500); // Move away to unpress switch (HIGH to LOW)
    while (stepper.distanceToGo() != 0) stepper.run();

    Serial.println("Fast move to limit");
    limitHit = false;
    stepper.setMaxSpeed(MAX_SPEED / 2);
    stepper.move(-1000000); // Toward switch
    while (!limitHit) stepper.run(); // ISR stops on FALLING
    stepper.stop();

    Serial.println("Slow backup");
    stepper.setMaxSpeed(MAX_SPEED / 16);
    stepper.move(2 * stepsPerMM);
    while (stepper.distanceToGo() != 0) stepper.run();

    Serial.println("Slow move to limit");
    stepper.setMaxSpeed(MAX_SPEED / 16);
    limitHit = false;
    stepper.move(-1000000);
    while (!limitHit) stepper.run();
    stepper.stop();
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(MAX_SPEED);
    isHomed = true;
    Serial.println("Calibration complete!");
    return true;
  }

  // ISR handler for limit switch
  void limitISR() {
    stepper.stop();
    stepper.setCurrentPosition(0);
    limitHit = true;
  }
};

// Initialize systems (unchanged)
MotorSystem verticalSystem(V_DIR_PIN, V_STEP_PIN, V_ENABLE_PIN, V_LIMIT_SWITCH_PIN, STEPS_PER_MM);
MotorSystem horizontalSystem(H_DIR_PIN, H_STEP_PIN, H_ENABLE_PIN, H_LIMIT_SWITCH_PIN, STEPS_PER_MM);

// Thermocouple object (new)
MAX6675 thermocouple(THERMO_SCK_PIN, THERMO_CS_PIN, THERMO_SO_PIN);

// **Interrupt Service Routines**
void vLimitISR() {
  static unsigned long lastTime = 0;
  if (millis() - lastTime > 100 && digitalRead(V_LIMIT_SWITCH_PIN) == LOW) {
    verticalSystem.limitISR();
    lastTime = millis();
  }
}

void hLimitISR() {
  static unsigned long lastTime = 0;
  if (millis() - lastTime > 100 && digitalRead(H_LIMIT_SWITCH_PIN) == LOW) {
    horizontalSystem.limitISR();
    lastTime = millis();
  }
}

// Encoder ISR (unchanged)
void encoderISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > DEBOUNCE_DELAY) {
    int currentCLK = digitalRead(ENCODER_CLK_PIN);
    if (currentCLK != lastEncoderCLK) {
      if (digitalRead(ENCODER_DT_PIN) != currentCLK) {
        encoderPosition++;
      } else {
        encoderPosition--;
      }
      lastEncoderCLK = currentCLK;
    }
    lastInterruptTime = interruptTime;
  }
}

// Process serial commands (enhanced with scan and stop)
void processSerialCommand(String command) {
  command.trim();
  if (command.length() < 2 && command != "scan" && command != "stop") return;
  if (command == "scan") {
    if (!verticalSystem.isHomed || !horizontalSystem.isHomed) {
      Serial.println(F("Error: Axes not homed."));
      return;
    }
    isScanning = true;
    currentX = X_MIN;
    currentY = Y_MIN;
    horizontalSystem.enable(true);
    verticalSystem.enable(true);
    horizontalSystem.setTargetPositionMM(currentX);
    verticalSystem.setTargetPositionMM(currentY);
    Serial.println(F("Scan started"));
    Serial.println(F("X,Y,Temperature"));
  } else if (command == "stop") {
    isScanning = false;
    horizontalSystem.enable(false);
    verticalSystem.enable(false);
    Serial.println(F("Scan stopped"));
  } else {
    char axis = command.charAt(0);
    float position = command.substring(1).toFloat();
    switch (tolower(axis)) {
      case 'x':
        horizontalSystem.enable(true);
        horizontalSystem.setTargetPositionMM(position);
        Serial.print(F("Moving X to ")); Serial.println(position);
        break;
      case 'y':
        verticalSystem.enable(true);
        verticalSystem.setTargetPositionMM(position);
        Serial.print(F("Moving Y to ")); Serial.println(position);
        break;
      case 'h':
        if (command.equals("home")) {
          Serial.println(F("Homing..."));
          verticalSystem.enable(true);
          horizontalSystem.enable(true);
          if (verticalSystem.moveToHome() && horizontalSystem.moveToHome()) {
            Serial.println(F("Homing complete"));
          } else {
            Serial.println(F("Homing failed"));
          }
        }
        break;
    }
  }
}

double getTemp() {
  // double total = 0;
  // delay(5000);
  // for (int i = 0; i < NUM_SAMPLES; i++) {
  //   double reading = analogRead(TC_PIN);
  //   double voltage = reading * (AREF / 1023.0);
  //   double temp = (voltage - 1.25) / 0.005;
  //   total += temp;
  //   delay(10); // Short delay to allow ADC to settle
  // }
  // double avgTemp = total / NUM_SAMPLES;
  // return avgTemp + 91.0; // Adjust offset based on calibration
  return thermocouple.readCelsius();
}

void reportPosition() {
  if (!isScanning && millis() - lastReportTime >= REPORT_INTERVAL) {
    Serial.print(F("Position (mm) - X:"));
    Serial.print(horizontalSystem.getCurrentPositionMM(), 2);
    Serial.print(F(" Y:"));
    Serial.print(verticalSystem.getCurrentPositionMM(), 2);
    Serial.print(F(" | Steps - X:"));
    Serial.print(horizontalSystem.stepper.currentPosition());
    Serial.print(F(" Y:"));
    Serial.println(verticalSystem.stepper.currentPosition());
    Serial.println(getTemp());
    lastReportTime = millis();
  }
}

#define NUM_SAMPLES 10



void setup() {
  // Configure encoder pins (unchanged)
  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);

  pinMode(V_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(H_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  // Attach limit switch interrupts
  attachInterrupt(digitalPinToInterrupt(V_LIMIT_SWITCH_PIN), vLimitISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H_LIMIT_SWITCH_PIN), hLimitISR, CHANGE);

  // Attach encoder interrupt (unchanged)
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), encoderISR, CHANGE);
  lastEncoderCLK = digitalRead(ENCODER_CLK_PIN);

  Serial.begin(9600);
  Serial.println(F("2D Platform Controller"));
  Serial.println(F("Commands:"));
  Serial.println(F("  'x<position>' - Move X axis to position in mm"));
  Serial.println(F("  'y<position>' - Move Y axis to position in mm"));
  Serial.println(F("  'home' - Home both axes"));
  Serial.println(F("  'scan' - Start thermal mapping"));  // New command
  Serial.println(F("  'stop' - Stop scanning"));          // New command
  Serial.println(F("Press encoder button to switch axis control"));


  // // Initial homing (unchanged)
  // if (verticalSystem.moveToHome() && horizontalSystem.moveToHome()) {
  //   Serial.println(F("Initial homing complete"));
  // } else {
  //   Serial.println(F("Initial homing failed"));
  // }

  // Allow thermocouple to stabilize (new)
  delay(500);
}

void loop() {
  // if (Serial.available()) {
  //   String command = Serial.readStringUntil('\n');
  //   processSerialCommand(command);
  // }

  // if (digitalRead(ENCODER_SW_PIN) == LOW) {
  //   unsigned long debounceStart = millis();
  //   while (millis() - debounceStart < DEBOUNCE_DELAY && digitalRead(ENCODER_SW_PIN) == LOW);
  //   if (digitalRead(ENCODER_SW_PIN) == LOW) {
  //     controllingVertical = !controllingVertical;
  //     Serial.print(F("Now controlling "));
  //     Serial.println(controllingVertical ? F("vertical") : F("horizontal"));
  //     encoderPosition = lastEncoderPosition = 0;
  //   }
  // }

  // if (encoderPosition != lastEncoderPosition) {
  //   int steps = (encoderPosition - lastEncoderPosition) * 10;
  //   if (controllingVertical) verticalSystem.stepper.move(steps);
  //   else horizontalSystem.stepper.move(steps);
  //   lastEncoderPosition = encoderPosition;
  // }

  // verticalSystem.stepper.run();
  // horizontalSystem.stepper.run();

  // if (isScanning) {
  //   if (horizontalSystem.stepper.distanceToGo() == 0 && verticalSystem.stepper.distanceToGo() == 0) {
  //     // float temp = thermocouple.readCelsius(); // Use real reading
  //     float temp = getTemp();
  //     Serial.print(currentX, 2);
  //     Serial.print(",");
  //     Serial.print(currentY, 2);
  //     Serial.print(",");
  //     Serial.println(temp, 2);
  //     currentX += STEP_SIZE;
  //     if (currentX > X_MAX) {
  //       currentX = X_MIN;
  //       currentY += STEP_SIZE;
  //       if (currentY > Y_MAX) {
  //         isScanning = false;
  //         Serial.println(F("Scan completed"));
  //       } else {
  //         verticalSystem.setTargetPositionMM(currentY);
  //       }
  //     }
  //     horizontalSystem.setTargetPositionMM(currentX);
  //   }
  // }

  // reportPosition();
  reportPosition();
  Serial.println(getTemp());
  delay(10);
}