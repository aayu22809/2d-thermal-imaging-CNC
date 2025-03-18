#include <AccelStepper.h>
#include <MAX6675.h>

// Pin Definitions
const int V_LIMIT_SWITCH_PIN = 18;
const int V_STEP_PIN = 22;
const int V_DIR_PIN = 24;
const int V_ENABLE_PIN = 26;

const int H_LIMIT_SWITCH_PIN = 19;
const int H_STEP_PIN = 23;
const int H_DIR_PIN = 25;
const int H_ENABLE_PIN = 27;

// Thermocouple Pins (SPI)
const int THERMO_SCK_PIN = 6;
const int THERMO_CS_PIN = 5;
const int THERMO_SO_PIN = 7;

// Motor Parameters
const float STEPS_PER_MM = 50.0;
const int MAX_SPEED = 800;
const int ACCELERATION = 800;

// Grid Limits
const float X_MIN = 90.0;
const float X_MAX = 110.0;
const float Y_MIN = 150.0;
const float Y_MAX = 200.0;
const float STEP_SIZE = 1.0;

// System State Variables
volatile bool vLimitTriggered = false;
volatile bool hLimitTriggered = false;
bool isScanning = false;
float currentX = X_MIN;
float currentY = Y_MIN;
unsigned long lastReportTime = 0;

// Thermocouple Object
MAX6675 thermocouple(THERMO_SCK_PIN, THERMO_CS_PIN, THERMO_SO_PIN);

// MotorSystem Class
class MotorSystem {
public:
    const int limitSwitchPin;
    const int enablePin;
    const int directionPin;
    const float stepsPerMM;
    bool isHomed;
    AccelStepper stepper;

    MotorSystem(int dirPin, int stepPin, int enablePin, int limitPin, float stepsPerMM)
      : stepper(AccelStepper::DRIVER, stepPin, dirPin),
        limitSwitchPin(limitPin),
        enablePin(enablePin),
        directionPin(dirPin),
        stepsPerMM(stepsPerMM),
        isHomed(false) {
        pinMode(limitSwitchPin, INPUT_PULLUP);
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, LOW);
        stepper.setMaxSpeed(MAX_SPEED);
        stepper.setAcceleration(ACCELERATION);
    }

    void enable(bool enabled) {
        digitalWrite(enablePin, enabled ? LOW : HIGH);
    }

    float getCurrentPositionMM() {
        return stepper.currentPosition() / stepsPerMM;
    }

    void setTargetPositionMM(float mm) {
        stepper.moveTo(mm * stepsPerMM);
    }

    bool moveToHome() {
        Serial.println("Starting homing...");

        unsigned long startTime = millis();
        stepper.setMaxSpeed(MAX_SPEED / 2);
        stepper.setAcceleration(ACCELERATION / 2);
        stepper.move(500);
        while (stepper.distanceToGo() != 0) {
            stepper.run();
            if (millis() - startTime > 5000) {
                Serial.println("Homing timeout (move away).");
                return false;
            }
        }

        Serial.println("Moving to limit...");
        startTime = millis();
        stepper.setMaxSpeed(MAX_SPEED / 2);
        stepper.move(-1000000);
        while (!digitalRead(limitSwitchPin) == LOW) {
            stepper.run();
            if (millis() - startTime > 5000) {
                Serial.println("Homing timeout (limit switch).");
                return false;
            }
        }
        stepper.stop();
        stepper.setCurrentPosition(0);
        stepper.setMaxSpeed(MAX_SPEED);
        isHomed = true;
        Serial.println("Homing complete.");
        return true;
    }
};

// Motor Objects
MotorSystem verticalSystem(V_DIR_PIN, V_STEP_PIN, V_ENABLE_PIN, V_LIMIT_SWITCH_PIN, STEPS_PER_MM);
MotorSystem horizontalSystem(H_DIR_PIN, H_STEP_PIN, H_ENABLE_PIN, H_LIMIT_SWITCH_PIN, STEPS_PER_MM);

// Limit Switch Interrupts
void vLimitISR() { vLimitTriggered = true; }
void hLimitISR() { hLimitTriggered = true; }

// Temperature Reading (Corrected for MAX6675)
double getTemp() {
    return thermocouple.readCelsius();
}

// Command Processor
void processSerialCommand(String command) {
    command.trim();
    command.toLowerCase();

    if (command == "scan") {
        if (!verticalSystem.isHomed || !horizontalSystem.isHomed) {
            Serial.println("Error: Axes not homed.");
            return;
        }
        isScanning = true;
        Serial.println("Scan started: X,Y,Temperature");
        return;
    }

    if (command == "stop") {
        isScanning = false;
        verticalSystem.enable(false);
        horizontalSystem.enable(false);
        Serial.println("Scan stopped.");
        return;
    }

    char axis = command.charAt(0);
    float position = command.substring(1).toFloat();

    if (!verticalSystem.isHomed || !horizontalSystem.isHomed) {
        Serial.println("Error: System not homed.");
        return;
    }

    if (axis == 'x') {
        horizontalSystem.enable(true);
        horizontalSystem.setTargetPositionMM(position);
        Serial.print("Moving X to "); Serial.println(position);
    } else if (axis == 'y') {
        verticalSystem.enable(true);
        verticalSystem.setTargetPositionMM(position);
        Serial.print("Moving Y to "); Serial.println(position);
    } else if (command == "home") {
        Serial.println("Homing...");
        if (verticalSystem.moveToHome() && horizontalSystem.moveToHome()) {
            Serial.println("Homing complete.");
        } else {
            Serial.println("Homing failed.");
        }
    }
}

// Position Reporting (Every 1s)
void reportPosition() {
    if (millis() - lastReportTime >= 1000) {
        Serial.print("X: "); Serial.print(horizontalSystem.getCurrentPositionMM(), 2);
        Serial.print(" Y: "); Serial.print(verticalSystem.getCurrentPositionMM(), 2);
        Serial.print(" | Temp: "); Serial.println(getTemp());
        lastReportTime = millis();
    }
}

// Setup Function
void setup() {
    Serial.begin(9600);
    Serial.println("2D Platform Controller Ready.");

    pinMode(V_LIMIT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(H_LIMIT_SWITCH_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(V_LIMIT_SWITCH_PIN), vLimitISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(H_LIMIT_SWITCH_PIN), hLimitISR, FALLING);

    Serial.println("Homing system...");
    if (verticalSystem.moveToHome() && horizontalSystem.moveToHome()) {
        Serial.println("Homing complete.");
    } else {
        Serial.println("Homing failed.");
    }
}

// Main Loop
void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processSerialCommand(command);
    }

    if (vLimitTriggered) {
        Serial.println("Vertical limit switch hit!");
        verticalSystem.moveToHome();
        vLimitTriggered = false;
    }

    if (hLimitTriggered) {
        Serial.println("Horizontal limit switch hit!");
        horizontalSystem.moveToHome();
        hLimitTriggered = false;
    }

    verticalSystem.stepper.run();
    horizontalSystem.stepper.run();
    reportPosition();
}
