#include <AccelStepper.h>
#include <MAX6675.h>

// Plasma POI temperature
#define PLASMA_POI_TEMPERATURE 35.0

// Pin Definitions
constexpr int V_LIMIT_SWITCH_PIN = 18;
constexpr int V_STEP_PIN = 22;
constexpr int V_DIR_PIN = 24;
constexpr int V_ENABLE_PIN = 26;

constexpr int H_LIMIT_SWITCH_PIN = 19;
constexpr int H_STEP_PIN = 23;
constexpr int H_DIR_PIN = 25;
constexpr int H_ENABLE_PIN = 27;

// Thermocouple Pins (SPI)
constexpr int THERMO_SCK_PIN = 6;
constexpr int THERMO_CS_PIN = 5;
constexpr int THERMO_SO_PIN = 7;

// Motor Parameters
constexpr float STEPS_PER_MM = 50.0;
constexpr int MAX_SPEED = 800;
constexpr int ACCELERATION = 800;

// Grid Limits & Step Size
constexpr float X_MIN = 0.0;
constexpr float X_MAX = 100.0;
constexpr float Y_MIN = 0.0;
constexpr float Y_MAX = 100.0;

// System State Variables
bool isScanning = false;
bool liveMonitoring = false;
float currentX = X_MIN;
float currentY = Y_MIN;
unsigned long lastReportTime = 0;
constexpr int MAX_HOMING_DISTANCE = 1000000;


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

    boolean moveToPosition(float position) {
        if (position < X_MIN || position < Y_MIN || position > X_MAX || position > Y_MAX) {
            Serial.println("Error: Movement out of bounds!");
            return false;  // Prevent the movement
        }

        enable(true);  // Enable motor before moving
        stepper.moveTo(position * STEPS_PER_MM);
        while (stepper.distanceToGo() != 0) {
            stepper.run();
        }
        enable(false);  // Disable motor when done

        if (stepper.distanceToGo() == 0 || stepper.currentPosition() != position * STEPS_PER_MM) {
            return true;
        }
        return false;
    }

    bool home() {
        isHomed = false;
        isLimitHit = false;  // Reset limit hit flag

        enable(true);  // Enable the motor before homing
        stepper.setMaxSpeed(MAX_SPEED / 2.);
        stepper.move(-MAX_HOMING_DISTANCE);  // Move motor towards the limit switch

        unsigned long startTime = millis();
        while (digitalRead(limitSwitchPin) == (isNormallyClosed ? LOW : HIGH) && millis() - startTime < 5000) { // While the limit switch is not pressed
            stepper.run();
        }
        if (digitalRead(limitSwitchPin) == (isNormallyClosed ? HIGH : LOW) ) {  // If limit switch is pressed
            stepper.stop();
            stepper.setCurrentPosition(0);  // Set current position to 0

            stepper.setMaxSpeed(MAX_SPEED / 4.);
            stepper.move(MAX_HOMING_DISTANCE);  // Move motor away from the limit switch

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
                Serial.println("Info: Homing complete.");
            }
        }
        if (!isHomed) {
            stepper.stop();
            enable(false);  // Disable motor
            Serial.println("Error: Homing failed.");
        }

        return isHomed;
    }

    void moveToPosition(float position, float max_speed)
    {
        int current_speed = stepper.maxSpeed();
        stepper.setMaxSpeed(max_speed);
        moveToPosition(position);
        stepper.setMaxSpeed(current_speed);
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

/*
 These commands follow the G-code syntax
     G0          Rapid move (move to X, Y at a specified feed rate)
     G1          Linear move (move to X, Y at a specified feed rate)
     G28         Home all axes
     M105        Report current temperature
     M3 / M5     Start/stop scanning
 */
void processSerialCommand(String command) {
    command.trim();
    command.toLowerCase();


    // G-code Parsing
    if (command.startsWith("g") || command.startsWith("m")) {
        float x = currentX, y = currentY, f = MAX_SPEED; // Default to current position and max speed
        bool hasX = false, hasY = false, hasF = false;

        // Extract parameters (e.g. "X10.5 Y20.0 F500")
        for (int i = 1; i < command.length(); i++) {
            if (command[i] == 'x' && i + 1 < command.length()) {
                int startIdx = i + 1;
                while (i + 1 < command.length() && (isdigit(command[i + 1]) || command[i + 1] == '.')) {
                    i++;
                }
                x = command.substring(startIdx, i + 1).toFloat();
                hasX = true;
            } else if (command[i] == 'y' && i + 1 < command.length()) {
                int startIdx = i + 1;
                while (i + 1 < command.length() && (isdigit(command[i + 1]) || command[i + 1] == '.')) {
                    i++;
                }
                y = command.substring(startIdx, i + 1).toFloat();
                hasY = true;
            } else if (command[i] == 'f' && i + 1 < command.length()) {
                int startIdx = i + 1;
                while (i + 1 < command.length() && (isdigit(command[i + 1]) || command[i + 1] == '.')) {
                    i++;
                }
                f = command.substring(startIdx, i + 1).toFloat();
                hasF = true;
            }
        }

        if (command.startsWith("g0") || command.startsWith("g1")) {
            // Move to position with specified feed rate (F)
            if (hasX) {
                horizontalSystem.moveToPosition(x, hasF ? f : MAX_SPEED);
                currentX = x;
            }
            if (hasY) {
                verticalSystem.moveToPosition(y, hasF ? f : MAX_SPEED);
                currentY = y;
            }
            Serial.println("ok");
        } else if (command == "g28") {
            verticalSystem.home();
            horizontalSystem.home();
            currentX = 0;
            currentY = 0;
            Serial.println("ok");
        } else if (command == "m105") {
            float temp = thermocouple.readCelsius();
            Serial.print("T:");
            Serial.println(temp, 2);
            Serial.println("ok");
        } else if (command == "m3") {
            if (command.length() > 4) {
                float xMin, yMin, xMax, yMax, stepSize;
                if (sscanf(command.c_str(), "m3 (%f,%f) -> (%f,%f), %f", &xMin, &yMin, &xMax, &yMax, &stepSize) == 5) {
                    startScan(xMin, xMax, yMin, yMax, stepSize);
                }
            }
            // Start a default scan (customize as needed)
            startScan(X_MIN, X_MAX, Y_MIN, Y_MAX, 1.0); // Example: full-size, 1mm step
            Serial.println("ok");
        } else if (command == "m5") {
            isScanning = false; // Stop scan
            Serial.println("ok");
        } else {
            Serial.println("Error: Unknown G-code.");
        }
    }


    // Existing Custom Commands
    else if (command.startsWith("scan")) {
        float xMin, yMin, xMax, yMax, stepSize;
        if (sscanf(command.c_str(), "scan (%f,%f) -> (%f,%f) res=%f", &xMin, &yMin, &xMax, &yMax, &stepSize) == 5) {
            startScan(xMin, xMax, yMin, yMax, stepSize);
        } else {
            Serial.println("Error: Invalid scan format.");
        }
    } else if (command == "home") {
        verticalSystem.home();
        horizontalSystem.home();
        currentX = 0;
        currentY = 0;
    } else if (command.startsWith("x")) {
        int tempX;
        if (sscanf(command.c_str(), "x %f", &tempX) == 1) {
            if (horizontalSystem.moveToPosition(tempX)) {
                currentX = tempX;
            }
        }
    } else if (command.startsWith("y")) {
        int tempY;
        if (sscanf(command.c_str(), "y %f", &tempY) == 1) {
            if (verticalSystem.moveToPosition(tempY)) {
                currentY = tempY;
            }
        }
    } else if (command.startsWith("monitor")) {
        sscanf(command.c_str(), "monitor %d", &liveMonitoring);
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
