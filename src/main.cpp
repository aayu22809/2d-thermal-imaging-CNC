#include <AccelStepper.h>
#include <MAX6675.h>

// Plasma POI temperature
#define PLASMA_POI_TEMPERATURE 35.0

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
const float X_MIN = 0.0;
const float X_MAX = 100.0;
const float Y_MIN = 0.0;
const float Y_MAX = 100.0;

// System State Variables
bool isScanning = false;
bool liveMonitoring = false;
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
    bool isNormallyClosed;

    MotorSystem(int dirPin, int stepPin, int enablePin, int limitPin, bool normallyClosed = true)
      : stepper(AccelStepper::DRIVER, stepPin, dirPin),
        limitSwitchPin(limitPin),
        enablePin(enablePin),
        isHomed(false),
        isLimitHit(false),
        isNormallyClosed(normallyClosed)
    {
        pinMode(limitSwitchPin, INPUT_PULLUP);
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, LOW);
        stepper.setMaxSpeed(MAX_SPEED);
        stepper.setAcceleration(ACCELERATION);
    }

    void enable(bool enabled) const
    {
        digitalWrite(enablePin, enabled ? LOW : HIGH);
    }

    void moveToPosition(float position) {
        if (position < X_MIN || position < Y_MIN || position > X_MAX || position > Y_MAX) {
            Serial.println("Movement out of bounds! Please stay within the defined area.");
            return;  // Prevent the movement
        }

        enable(true);  // Enable motor before moving
        stepper.moveTo(position * STEPS_PER_MM);
        while (stepper.distanceToGo() != 0) {
            stepper.run();
        }
        enable(false);  // Disable motor when done
    }

    bool home() {
        isHomed = false;
        isLimitHit = false;  // Reset limit hit flag

        enable(true);  // Enable the motor before homing
        stepper.setMaxSpeed(MAX_SPEED / 2.);
        stepper.move(-1000000);  // Move motor towards the limit switch

        unsigned long startTime = millis();
        while (digitalRead(limitSwitchPin) == (isNormallyClosed ? LOW : HIGH) && millis() - startTime < 5000) { // While the limit switch is not pressed
            stepper.run();
        }
        if (digitalRead(limitSwitchPin) == (isNormallyClosed ? HIGH : LOW) ) {  // If limit switch is pressed
            stepper.stop();
            stepper.setCurrentPosition(0);  // Set current position to 0

            stepper.setMaxSpeed(MAX_SPEED / 4.);
            stepper.move(1000000);  // Move motor away from the limit switch

            startTime = millis();
            while (digitalRead(limitSwitchPin) == (isNormallyClosed ? LOW : HIGH) && millis() - startTime < 5000) { // While the limit switch is not pressed
                stepper.run();
            }

            if (digitalRead(limitSwitchPin) == (isNormallyClosed ? HIGH : LOW) ) {  // If limit switch is pressed
                stepper.stop();
                stepper.setCurrentPosition(0);  // Set current position to 0
                isHomed = true;
                enable(false);  // Disable motor
                stepper.setMaxSpeed(MAX_SPEED); // Restore max speed
                Serial.println("Homing complete.");
            }
        }

        if (!isHomed) {
            stepper.stop();
            enable(false);  // Disable motor
            Serial.println("Homing failed: Limit switch not pressed.");
        }

        return isHomed;
    }

private:
    const int limitSwitchPin;
    const int enablePin;
    bool isLimitHit;
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
            if (temp > PLASMA_POI_TEMPERATURE) { // If the temperature is above the plasma POI, slow down the motors
                horizontalSystem.stepper.setMaxSpeed(MAX_SPEED / 2.);
                verticalSystem.stepper.setMaxSpeed(MAX_SPEED / 2.);
            } else {
                verticalSystem.stepper.setMaxSpeed(MAX_SPEED);
                horizontalSystem.stepper.setMaxSpeed(MAX_SPEED);
            }
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
        float xMin, yMin, xMax, yMax, stepSize;
        if (sscanf(command.c_str(), "scan (%f,%f) -> (%f,%f), %f", &xMin, &yMin, &xMax, &yMax, &stepSize) == 5) {
            startScan(xMin, xMax, yMin, yMax, stepSize);
        }
    } else if (command == "home") {
        Serial.println("Homing...");
        verticalSystem.home();
        horizontalSystem.home();
        Serial.println("Homing complete.");
    } else if (command.startsWith("x")) {
        sscanf(command.c_str(), "x %f", &currentX);
        Serial.print("Moving to X: ");
        Serial.println(currentX);
        verticalSystem.moveToPosition(currentX);
    } else if (command.startsWith("y")) {
        sscanf(command.c_str(), "y=%f", &currentY);
        Serial.print("Moving to Y: ");
        Serial.println(currentY);
        horizontalSystem.moveToPosition(currentY);
    } else if (command.startsWith("monitor")) {
        sscanf(command.c_str(), "monitor %d", &liveMonitoring);
    }
    else {
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
    if (liveMonitoring) {
        if (isScanning) {
            Serial.println("Scanning...");
        } else {
            char buffer[100];
            sprintf(buffer, "X: %.2f, Y: %.2f, Temperature: %.2f", currentX, currentY, thermocouple.readCelsius());
            Serial.println(buffer);
        }
    }
}
