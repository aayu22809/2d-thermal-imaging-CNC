#include <AccelStepper.h>
#include <MAX6675.h>

// Plasma POI temperature
#define PLASMA_POI_TEMPERATURE 35.0

// Pin Definitions
constexpr int V_LIMIT_SWITCH_PIN = 18; // No longer necessary to have on comms, can move back to digital.
constexpr int V_STEP_PIN = 22;
constexpr int V_DIR_PIN = 24;
constexpr int V_ENABLE_PIN = 26;

constexpr int H_LIMIT_SWITCH_PIN = 19; // No longer necessary to have on comms, can move back to digital.
constexpr int H_STEP_PIN = 23;
constexpr int H_DIR_PIN = 25;
constexpr int H_ENABLE_PIN = 27;

// Thermocouple Pins (SPI)
constexpr int THERMO_SCK_PIN = 6;
constexpr int THERMO_CS_PIN = 5;
constexpr int THERMO_SO_PIN = 7;
constexpr int AD8495_PIN = A0;  // Analog pin for AD8495 breakout

// Motor Parameters
constexpr float STEPS_PER_MM = 50.0;
constexpr float STEPS_PER_MM_X = 50.0;  // Keep original for X-axis
constexpr float STEPS_PER_MM_Y = 50.0;  // Adjust this value for Y-axis
constexpr int MAX_SPEED = 800;
constexpr int ACCELERATION = 800;

// Grid Limits & Step Size
constexpr float X_MIN = 3;
constexpr float X_MAX = 450.0;
constexpr float Y_MIN = 5;
constexpr float Y_MAX = 205.0;

// System State Variables
bool isScanning = false;
bool liveMonitoring = false;
float currentX = X_MIN;
float currentY = Y_MIN;
unsigned long lastReportTime = 0;
constexpr int MAX_HOMING_DISTANCE = 10000;


// Thermocouple Object
MAX6675 thermocouple(THERMO_SCK_PIN, THERMO_CS_PIN, THERMO_SO_PIN);

// Define axis types
enum AxisType {
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
    AXIS_A  // For future rotational axis
};

// MotorSystem Class
// Fix the member initialization order in the constructor
class MotorSystem {
public:
    AccelStepper stepper;
    bool isHomed;
    bool isNormallyClosed;
    float stepsPerMm;
    AxisType axisType;
    float minLimit;
    float maxLimit;

private:
    const int limitSwitchPin;
    const int enablePin;
    bool isLimitHit;
    static constexpr unsigned long HOMING_TIMEOUT_MS = 15000;
    static constexpr unsigned long MOVEMENT_TIMEOUT_MS = 5000;

    // Check if limit switch is triggered
    bool isLimitSwitchTriggered() const {
        return digitalRead(limitSwitchPin) == (isNormallyClosed ? HIGH : LOW);
    }
    
    // Wait for limit switch with timeout
    bool waitForLimitSwitch(unsigned long timeoutMs) {
        unsigned long startTime = millis();
        while (!isLimitSwitchTriggered() && (millis() - startTime < timeoutMs)) {
            stepper.run();
        }
        return isLimitSwitchTriggered();
    }
    
    // Log position error
    void logPositionError(float position) const {
        Serial.print("Error: Movement out of bounds for ");
        Serial.print(getAxisName());
        Serial.print(" axis! (");
        Serial.print(position);
        Serial.print(" not in range ");
        Serial.print(minLimit);
        Serial.print("-");
        Serial.print(maxLimit);
        Serial.println(")");
    }

public:
    MotorSystem(int dirPin, int stepPin, int enablePin, int limitPin, 
                AxisType axis, float minPos, float maxPos, 
                float stepsPerMm = STEPS_PER_MM, bool normallyClosed = true)
      : stepper(AccelStepper::DRIVER, stepPin, dirPin),
        isHomed(false),
        isNormallyClosed(normallyClosed),
        stepsPerMm(stepsPerMm),
        axisType(axis),
        minLimit(minPos),
        maxLimit(maxPos),
        limitSwitchPin(limitPin),
        enablePin(enablePin),
        isLimitHit(false)
    {
        pinMode(limitSwitchPin, INPUT_PULLUP);
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, LOW);
        stepper.setMaxSpeed(MAX_SPEED);
        stepper.setAcceleration(ACCELERATION);
    }

    // Enable or disable the motor
    void enable(bool enabled) const {
        digitalWrite(enablePin, enabled ? LOW : HIGH);
    }

    // Move to a specific position with timeout
    boolean moveToPosition(float position) {
        // Check bounds based on this motor's limits
        if (position < minLimit || position > maxLimit) {
            logPositionError(position);
            return false;  // Prevent the movement
        }

        enable(true);  // Enable motor before moving
        stepper.moveTo(position * stepsPerMm);
        
        // Add timeout to prevent motor from being enabled too long
        unsigned long startTime = millis();
        while (stepper.distanceToGo() != 0 && (millis() - startTime < MOVEMENT_TIMEOUT_MS)) {
            stepper.run();
        }
        
        // Always disable motor when done
        enable(false);

        return (stepper.distanceToGo() == 0);
    }

    // Move to position with specified speed
    void moveToPosition(float position, float max_speed) {
        int current_speed = stepper.maxSpeed();
        stepper.setMaxSpeed(max_speed);
        moveToPosition(position);
        stepper.setMaxSpeed(current_speed);
    }

    // Helper method to get axis name for debugging
    const char* getAxisName() const {
        switch (axisType) {
            case AXIS_X: return "X";
            case AXIS_Y: return "Y";
            case AXIS_Z: return "Z";
            case AXIS_A: return "A";
            default: return "Unknown";
        }
    }

    // First phase of homing - move to limit switch
    bool homeFirstPhase() {
        stepper.setMaxSpeed(MAX_SPEED / 2.);
        stepper.move(-MAX_HOMING_DISTANCE);  // Move motor towards the limit switch
        
        if (waitForLimitSwitch(HOMING_TIMEOUT_MS)) {
            stepper.stop();
            stepper.setCurrentPosition(0);  // Set current position to 0
            return true;
        }
        
        stepper.stop();
        return false;
    }
    
    // Second phase of homing - move away from limit switch
    bool homeSecondPhase() {
        stepper.setMaxSpeed(MAX_SPEED / 4.);
        stepper.move(MAX_HOMING_DISTANCE);  // Move motor away from the limit switch
        
        if (waitForLimitSwitch(MOVEMENT_TIMEOUT_MS)) {
            stepper.stop();
            stepper.setCurrentPosition(0);  // Set current position to 0
            return true;
        }
        
        stepper.stop();
        return false;
    }

    // Complete homing sequence
    bool home() {
        isHomed = false;
        isLimitHit = false;  // Reset limit hit flag
        enable(true);  // Enable the motor before homing
        
        if (!homeFirstPhase()) {
            enable(false);  // Disable motor
            Serial.println("Error: Homing failed in first phase.");
            return false;
        }
        
        if (!homeSecondPhase()) {
            enable(false);  // Disable motor
            Serial.println("Error: Homing failed in second phase.");
            return false;
        }
        
        isHomed = true;
        enable(false);  // Disable motor
        stepper.setMaxSpeed(MAX_SPEED); // Restore max speed
        Serial.println("Info: Homing complete.");
        return true;
    }
};

// Motor Objects
MotorSystem verticalSystem(V_DIR_PIN, V_STEP_PIN, V_ENABLE_PIN, V_LIMIT_SWITCH_PIN, 
                          AXIS_Y, Y_MIN, Y_MAX, STEPS_PER_MM_Y);
MotorSystem horizontalSystem(H_DIR_PIN, H_STEP_PIN, H_ENABLE_PIN, H_LIMIT_SWITCH_PIN, 
                            AXIS_X, X_MIN, X_MAX, STEPS_PER_MM_X);


// Temperature Sensor Class
class TemperatureSensor {
private:
    // AD8495 parameters
    static constexpr float VOLTAGE_OFFSET = 1.25;  // Voltage offset at 0°C (in volts)
    static constexpr float SCALE_FACTOR = 0.005;   // Scale factor (5mV/°C)
    static constexpr float AREF_VOLTAGE = 5.0;     // Arduino reference voltage (typically 5V)
    static constexpr int ADC_RESOLUTION = 10;      // ADC resolution (10-bit for most Arduinos)
    
    const int sensorPin;
    
    // Convert ADC reading to voltage
    float convertToVoltage(int rawAdc) const {
        return rawAdc * (AREF_VOLTAGE / (pow(2, ADC_RESOLUTION) - 1));
    }
    
    // Convert voltage to temperature
    float convertToTemperature(float voltage) const {
        return (voltage - VOLTAGE_OFFSET) / SCALE_FACTOR;
    }
    
public:
    explicit TemperatureSensor(int pin) : sensorPin(pin) {}
    
    // Get temperature reading from AD8495 breakout
    float readTemperature() const {
        int rawValue = analogRead(sensorPin);
        float voltage = convertToVoltage(rawValue);
        float temperature = convertToTemperature(voltage);
        return temperature;
    }
};

// Create temperature sensor instance
TemperatureSensor temperatureSensor(AD8495_PIN);


// Scanner Class to handle grid scanning operations
class Scanner {
private:
    // Adjust motor speed based on temperature
    static void adjustMotorSpeed(float temperature) {
        if (temperature > PLASMA_POI_TEMPERATURE) {
            horizontalSystem.stepper.setMaxSpeed(MAX_SPEED / 2.);
            verticalSystem.stepper.setMaxSpeed(MAX_SPEED / 2.);
        } else {
            verticalSystem.stepper.setMaxSpeed(MAX_SPEED);
            horizontalSystem.stepper.setMaxSpeed(MAX_SPEED);
        }
    }
    
    // Log temperature data for current position
    static void logPositionData(float x, float y, float temperature) {
        Serial.print(x, 2);
        Serial.print(",");
        Serial.print(y, 2);
        Serial.print(",");
        Serial.println(temperature, 2);
    }
    
    // Scan a single row from left to right
    static void scanRowLeftToRight(float xStart, float xEnd, float y, float stepSize) {
        for (float x = xStart; x <= xEnd; x += stepSize) {
            horizontalSystem.moveToPosition(x);
            delay(10); // Small delay for motor cooling and stabilization
            
            float temp = temperatureSensor.readTemperature();
            adjustMotorSpeed(temp);
            logPositionData(x, y, temp);
        }
    }
    
    // Scan a single row from right to left
    static void scanRowRightToLeft(float xStart, float xEnd, float y, float stepSize) {
        for (float x = xEnd; x >= xStart; x -= stepSize) {
            horizontalSystem.moveToPosition(x);
            delay(10); // Small delay for motor cooling and stabilization
            
            float temp = temperatureSensor.readTemperature();
            adjustMotorSpeed(temp);
            logPositionData(x, y, temp);
        }
    }
    
    // Print scan parameters
    static void printScanParameters(float xMin, float xMax, float yMin, float yMax, float stepSize) {
        Serial.println("<SCAN_START>");
        Serial.print("Start X: "); Serial.println(xMin);
        Serial.print("End X: "); Serial.println(xMax);
        Serial.print("Start Y: "); Serial.println(yMin);
        Serial.print("End Y: "); Serial.println(yMax);
        Serial.print("Resolution (mm/cell): "); Serial.println(stepSize);
        Serial.println("X,Y,Temperature");
    }

public:
    // Start a grid scan
    static void startScan(float xMin, float xMax, float yMin, float yMax, float stepSize) {
        // Check if axes are homed
        if (!verticalSystem.isHomed || !horizontalSystem.isHomed) {
            Serial.println("Error: Axes not homed. Please run G28 or 'home' command first.");
            return;
        }

        // Print scan parameters
        printScanParameters(xMin, xMax, yMin, yMax, stepSize);
        
        // Move to starting position one axis at a time with complete movement
        // First move X axis and wait for it to complete
        horizontalSystem.enable(true);
        horizontalSystem.stepper.moveTo(xMin * horizontalSystem.stepsPerMm);
        while (horizontalSystem.stepper.distanceToGo() != 0) {
            horizontalSystem.stepper.run();
        }
        horizontalSystem.enable(false);
        delay(100);
        
        // Then move Y axis and wait for it to complete
        verticalSystem.enable(true);
        verticalSystem.stepper.moveTo(yMin * verticalSystem.stepsPerMm);
        while (verticalSystem.stepper.distanceToGo() != 0) {
            verticalSystem.stepper.run();
        }
        verticalSystem.enable(false);
        delay(100);

        isScanning = true;
        bool scanLeftToRight = true;
        
        // Scan row by row
        for (float y = yMin; y <= yMax; y += stepSize) {
            // Move to the start of the row
            verticalSystem.enable(true);
            verticalSystem.stepper.moveTo(y * verticalSystem.stepsPerMm);
            while (verticalSystem.stepper.distanceToGo() != 0) {
                verticalSystem.stepper.run();
            }
            verticalSystem.enable(false);
            delay(50); // Allow Y motor to cool and stabilize
            
            // Scan in alternating directions
            if (scanLeftToRight) {
                scanRowLeftToRight(xMin, xMax, y, stepSize);
            } else {
                scanRowRightToLeft(xMin, xMax, y, stepSize);
            }
            
            // Toggle direction for next row
            scanLeftToRight = !scanLeftToRight;
        }

        // Disable both motors when scan is complete
        horizontalSystem.enable(false);
        verticalSystem.enable(false);
        
        isScanning = false;
        Serial.println("<SCAN_END>");
    }
};

// Start Grid Scan - wrapper function to maintain compatibility
void startScan(float xMin, float xMax, float yMin, float yMax, float stepSize) {
    Scanner::startScan(xMin, xMax, yMin, yMax, stepSize);
}

// Legacy temperature functions - kept for backward compatibility
#define TC_PIN A0          // set to ADC pin used
#define AREF 3.3           // set to AREF, typically board voltage like 3.3 or 5.0
#define ADC_RESOLUTION 10  // set to ADC bit resolution, 10 is default

float reading, voltage, temperature;

float get_voltage(int raw_adc) {
    return raw_adc * (AREF / (pow(2, ADC_RESOLUTION)-1));
}

float get_temperature(float voltage) {
    return (voltage - 1.25) / 0.005 + 89.89;
}

float getCelcius() {
    constexpr float offset = 89.89;
    reading = analogRead(TC_PIN);
    voltage = get_voltage(reading);
    temperature = get_temperature(voltage);
    return temperature + offset;
}

// Get temperature from AD8495 breakout
float getTemperatureFromAD8495() {
    return temperatureSensor.readTemperature();
}

// Command Processor Class to handle all command parsing and execution
class CommandProcessor {
private:
    // Extract movement parameters from G-code
    static void extractMovementParameters(const String& command, float& x, float& y, float& f, 
                                         bool& hasX, bool& hasY, bool& hasF) {
        for (unsigned int i = 1; i < command.length(); i++) {
            if (command[i] == 'x' && i + 1 < command.length()) {
                unsigned int startIdx = i + 1;
                while (i + 1 < command.length() && (isdigit(command[i + 1]) || command[i + 1] == '.')) {
                    i++;
                }
                x = command.substring(startIdx, i + 1).toFloat();
                hasX = true;
            } else if (command[i] == 'y' && i + 1 < command.length()) {
                unsigned int startIdx = i + 1;
                while (i + 1 < command.length() && (isdigit(command[i + 1]) || command[i + 1] == '.')) {
                    i++;
                }
                y = command.substring(startIdx, i + 1).toFloat();
                hasY = true;
            } else if (command[i] == 'f' && i + 1 < command.length()) {
                unsigned int startIdx = i + 1;
                while (i + 1 < command.length() && (isdigit(command[i + 1]) || command[i + 1] == '.')) {
                    i++;
                }
                f = command.substring(startIdx, i + 1).toFloat();
                hasF = true;
            }
        }
    }
    
    // Process G0/G1 movement commands
    static void processMovementCommand(float x, float y, float f, bool hasX, bool hasY, bool hasF) {
        if (hasX) {
            horizontalSystem.moveToPosition(x, hasF ? f : MAX_SPEED);
            currentX = x;
        }
        if (hasY) {
            verticalSystem.moveToPosition(y, hasF ? f : MAX_SPEED);
            currentY = y;
        }
        Serial.println("ok");
    }
    
    // Process G28 homing command
    static void processHomingCommand() {
        verticalSystem.home();
        horizontalSystem.home();
        currentX = 0;
        currentY = 0;
        Serial.println("ok");
    }
    
    // Process M105 temperature report command
    static void processTemperatureCommand() {
        float temp = temperatureSensor.readTemperature();
        Serial.print("T:");
        Serial.println(temp, 2);
        Serial.println("ok");
    }
    
    // Process M3 scan start command
    static void processScanStartCommand(const String& command) {
        Serial.println(command);
        float x1 = 0, y1 = 0, x2 = 0, y2 = 0, res = 0;
        
        // Debug homing status
        Serial.print("Homing status - X: ");
        Serial.print(horizontalSystem.isHomed ? "Homed" : "Not Homed");
        Serial.print(", Y: ");
        Serial.println(verticalSystem.isHomed ? "Homed" : "Not Homed");
        
        // Simple regex-like pattern matching using String functions
        int firstParenOpen = command.indexOf('(');
        int firstParenClose = command.indexOf(')', firstParenOpen);
        int firstComma = command.indexOf(',', firstParenOpen);
        
        int secondParenOpen = command.indexOf('(', firstParenClose);
        int secondParenClose = command.indexOf(')', secondParenOpen);
        int secondComma = command.indexOf(',', secondParenOpen);
        
        int lastComma = command.lastIndexOf(',');
        
        // Extract coordinates if all delimiters are found
        if (firstParenOpen > 0 && firstParenClose > firstParenOpen && firstComma > firstParenOpen && 
            secondParenOpen > 0 && secondParenClose > secondParenOpen && secondComma > secondParenOpen && 
            lastComma > 0) {
            
            // Extract first point (x1,y1)
            x1 = command.substring(firstParenOpen + 1, firstComma).toFloat();
            y1 = command.substring(firstComma + 1, firstParenClose).toFloat();
            
            // Extract second point (x2,y2)
            x2 = command.substring(secondParenOpen + 1, secondComma).toFloat();
            y2 = command.substring(secondComma + 1, secondParenClose).toFloat();
            
            // Extract resolution
            res = command.substring(lastComma + 1).toFloat();
            
            // Check if we got valid numbers
            if (res > 0) {
                Serial.print("Scan: (");
                Serial.print(x1); Serial.print(","); Serial.print(y1);
                Serial.print(") -> (");
                Serial.print(x2); Serial.print(","); Serial.print(y2);
                Serial.print("), Step: "); Serial.println(res);
                
                // Make sure we're passing parameters in the correct order
                startScan(x1, x2, y1, y2, res);
                Serial.println("ok");
                return;
            }
        }
        
        Serial.println("Error: Invalid m3 format. Use: m3 (x1,y1) -> (x2,y2), res");
    }
    
    // Process M5 scan stop command
    static void processScanStopCommand() {
        isScanning = false;
        Serial.println("ok");
    }
    
    // Process custom scan command
    static void processCustomScanCommand(const String& command) {
        float xMin, yMin, xMax, yMax, stepSize;
        if (sscanf(command.c_str(), "scan (%f,%f) -> (%f,%f) res=%f", &xMin, &yMin, &xMax, &yMax, &stepSize) == 5) {
            startScan(xMin, xMax, yMin, yMax, stepSize);
        } else {
            Serial.println("Error: Invalid scan format.");
        }
    }
    
    // Process custom home command
    static void processCustomHomeCommand() {
        verticalSystem.home();
        horizontalSystem.home();
        currentX = 0;
        currentY = 0;
    }
    
    // Process X axis movement command
    static void processXAxisCommand(const String& command) {
        float tempX;
        if (sscanf(command.c_str(), "x %f", &tempX) == 1) {
            if (horizontalSystem.moveToPosition(tempX)) {
                currentX = tempX;
            }
        }
    }
    
    // Process Y axis movement command
    static void processYAxisCommand(const String& command) {
        float tempY;
        if (sscanf(command.c_str(), "y %f", &tempY) == 1) {
            if (verticalSystem.moveToPosition(tempY)) {
                currentY = tempY;
            }
        }
    }
    
    // Process monitor command
    static void processMonitorCommand(const String& command) {
        int monitorValue;
        if (sscanf(command.c_str(), "monitor %d", &monitorValue) == 1) {
            liveMonitoring = (monitorValue != 0);
        }
    }
    
    // Process G-code commands
    static bool processGCode(const String& command) {
        if (!(command.startsWith("g") || command.startsWith("m"))) {
            return false;
        }
        
        float x = currentX, y = currentY, f = MAX_SPEED; // Default to current position and max speed
        bool hasX = false, hasY = false, hasF = false;
        
        // Extract parameters for movement commands
        extractMovementParameters(command, x, y, f, hasX, hasY, hasF);
        
        // Process specific G-code commands
        if (command.startsWith("g0") || command.startsWith("g1")) {
            processMovementCommand(x, y, f, hasX, hasY, hasF);
        } else if (command == "g28") {
            processHomingCommand();
        } else if (command == "m105") {
            processTemperatureCommand();
        } else if (command.startsWith("m3")) {
            processScanStartCommand(command);
        } else if (command == "m5") {
            processScanStopCommand();
        } else {
            Serial.println("Error: Unknown G-code.");
        }
        
        return true;
    }
    
    // Process custom commands
    static void processCustomCommands(const String& command) {
        if (command.startsWith("scan")) {
            processCustomScanCommand(command);
        } else if (command == "home") {
            processCustomHomeCommand();
        } else if (command.startsWith("x")) {
            processXAxisCommand(command);
        } else if (command.startsWith("y")) {
            processYAxisCommand(command);
        } else if (command.startsWith("monitor")) {
            processMonitorCommand(command);
        } else {
            Serial.println("Invalid command.");
        }
    }
    
public:
    /*
     These commands follow the G-code syntax
         G0          Rapid move (move to X, Y at a specified feed rate)
         G1          Linear move (move to X, Y at a specified feed rate)
         G28         Home all axes
         M105        Report current temperature
         M3 / M5     Start/stop scanning
     */
    static void processCommand(String command) {
        command.trim();
        command.toLowerCase();
        
        // First try to process as G-code
        if (processGCode(command)) {
            return;
        }
        
        // If not G-code, process as custom command
        processCustomCommands(command);
    }
};

// Wrapper function to maintain compatibility
void processSerialCommand(String command) {
    CommandProcessor::processCommand(command);
}
// System Monitor Class to handle status reporting
class SystemMonitor {
private:
    static constexpr unsigned long REPORT_INTERVAL_MS = 1000; // Report every second
    
    // Format and print current position and temperature
    static void reportStatus() {
        char buffer[100];
        // Cast float values to double to match %f format specifier
        sprintf(buffer, "X: %.2f, Y: %.2f, Temperature: %.2f", 
               (double)currentX, (double)currentY, (double)temperatureSensor.readTemperature());
        Serial.println(buffer);
    }
    
public:
    // Update monitoring status
    static void update() {
        if (!liveMonitoring) {
            return;
        }
        
        if (isScanning) {
            Serial.println("Scanning...");
        } else {
            reportStatus();
        }
    }
};

// Setup Function
void setup() {
    Serial.begin(9600);
    Serial.println("2D Platform Controller Ready.");
}

// Main Loop
void loop() {
    // Process any incoming serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        CommandProcessor::processCommand(command);
    }
    
    // Update system monitoring
    SystemMonitor::update();
}
