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

// Grid Limits & Step Size
const float X_MIN = 90.0;
const float X_MAX = 110.0;
const float Y_MIN = 150.0;
const float Y_MAX = 200.0;
const float STEP_SIZE = 1.0;

// System State Variables
bool isScanning = false;
float currentX = X_MIN;
float currentY = Y_MIN;
unsigned long lastReportTime = 0;

// Thermocouple Object
MAX6675 thermocouple(THERMO_SCK_PIN, THERMO_CS_PIN, THERMO_SO_PIN);

// MotorSystem Class
class MotorSystem {
public:
    AccelStepper stepper;
    bool isHomed;

    MotorSystem(int dirPin, int stepPin, int enablePin, int limitPin)
      : stepper(AccelStepper::DRIVER, stepPin, dirPin),
        limitSwitchPin(limitPin),
        enablePin(enablePin),
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

    void moveToPosition(float position) {
        enable(true);  // Enable motor before moving
        stepper.moveTo(position * STEPS_PER_MM);
        while (stepper.distanceToGo() != 0) {
            stepper.run();
        }
        enable(false);  // Disable motor when done

        if (stepper.currentPosition() != 0) isHomed = false;
    }


    bool home() {
        enable(true);  // Enable before homing
        stepper.setMaxSpeed(MAX_SPEED / 2.);
        stepper.move(-1000000);
        while (digitalRead(limitSwitchPin) == HIGH) stepper.run();
        stepper.stop();
        stepper.setCurrentPosition(0);
        isHomed = true;
        enable(false);  // Disable motor after homing
        return true;
    }

private:
    const int limitSwitchPin;
    const int enablePin;
};

// Motor Objects
MotorSystem verticalSystem(V_DIR_PIN, V_STEP_PIN, V_ENABLE_PIN, V_LIMIT_SWITCH_PIN);
MotorSystem horizontalSystem(H_DIR_PIN, H_STEP_PIN, H_ENABLE_PIN, H_LIMIT_SWITCH_PIN);



// Start Grid Scan
void startScan(float xMin, float xMax, float yMin, float yMax, float stepSize) {
    if (!verticalSystem.isHomed || !horizontalSystem.isHomed) {
        Serial.println("Error: Axes not homed.");
        return;
    }

    Serial.println("<SCAN_START>");
    Serial.print("Start X: "); Serial.println(xMin);
    Serial.print("End X: "); Serial.println(xMax);
    Serial.print("Start Y: "); Serial.println(yMin);
    Serial.print("End Y: "); Serial.println(yMax);
    Serial.print("Resolution (mm/cell): "); Serial.println(stepSize);
    Serial.println("X,Y,Temperature");

    isScanning = true;

    for (float y = yMin; y <= yMax; y += stepSize) {
        verticalSystem.moveToPosition(y);

        for (float x = xMin; x <= xMax; x += stepSize) {
            horizontalSystem.moveToPosition(x);
            float temp = thermocouple.readCelsius();
            Serial.print(x, 2);
            Serial.print(",");
            Serial.print(y, 2);
            Serial.print(",");
            Serial.println(temp, 2);
        }
    }

    isScanning = false;
    Serial.println("<SCAN_END>");
}

// Process Serial Commands
void processSerialCommand(String command) {
    command.trim();
    command.toLowerCase();

    if (command == "scan") {
        startScan(X_MIN, X_MAX, Y_MIN, Y_MAX, STEP_SIZE);
    } else if (command == "home") {
        Serial.println("Homing...");
        verticalSystem.home();
        horizontalSystem.home();
        Serial.println("Homing complete.");
    } else if (command.startsWith("x=")) {
        sscanf(command.c_str(), "x=%f", &currentX);
        Serial.print("Moving to X: ");
        Serial.println(currentX);
        verticalSystem.moveToPosition(currentX);
    } else if (command.startsWith("y=")) {
        sscanf(command.c_str(), "y=%f", &currentY);
        Serial.print("Moving to Y: ");
        Serial.println(currentY);
        horizontalSystem.moveToPosition(currentY);
    } else {
        Serial.println("Invalid command.");
    }
}

// Setup Function
void setup() {
    Serial.begin(9600);
    Serial.println("2D Platform Controller Ready.");
}

// Main Loop
void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processSerialCommand(command);
    }
}
