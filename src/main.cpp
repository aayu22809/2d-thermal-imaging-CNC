#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include <Arduino.h>

// Pin Definitions
// Base Motor
const int BASE_STEP_PIN = 22;
const int BASE_DIR_PIN = 23;
const int BASE_ENABLE_PIN = 24;
const int BASE_LIMIT_PIN = 253;

// Shoulder Motor
const int SHOULDER_STEP_PIN = 25;
const int SHOULDER_DIR_PIN = 26;
const int SHOULDER_ENABLE_PIN = 27;
const int SHOULDER_LIMIT_PIN = 29;

// Magnetic Encoder Pins
const int ENCODER_SDA_PIN = 20; // Default I2C SDA pin on Mega
const int ENCODER_SCL_PIN = 21; // Default I2C SCL pin on Mega

// Slide Motor
const int SLIDE_STEP_PIN = 30;
const int SLIDE_DIR_PIN = 31;
const int SLIDE_ENABLE_PIN = 32;
const int SLIDE_LIMIT_PIN = 33;

// Servo
const int SERVO_PIN = 9;

// Encoder addresses
const int ENCODER_BASE_ADDR = 0x36;    // Default AS5600 address
const int ENCODER_SHOULDER_ADDR = 0x37; // Modified AS5600 address
const int MAGNETIC_ENCODER_ADDR = 0x36; // New magnetic encoder address (may need to be adjusted)

// Configuration
const float STEPS_PER_DEG_BASE = 4.0833;    // (200 * 16 * 10) / 360
const float STEPS_PER_DEG_SHOULDER = 6.88; // (200 * 16 * 20) / 360
const float STEPS_PER_MM_SLIDE = 25.0;     // (200 * 16 * 5) / 40

// Movement limits
const float BASE_MIN = -30.0;
const float BASE_MAX = 135.0;
const float SHOULDER_MIN = -45.0;
const float SHOULDER_MAX = 90.0;
const float SLIDE_MIN = 0.0;
const float SLIDE_MAX = 200.0;

// Motor objects
AccelStepper baseMotor(AccelStepper::DRIVER, BASE_STEP_PIN, BASE_DIR_PIN);
AccelStepper shoulderMotor(AccelStepper::DRIVER, SHOULDER_STEP_PIN, SHOULDER_DIR_PIN);
AccelStepper slideMotor(AccelStepper::DRIVER, SLIDE_STEP_PIN, SLIDE_DIR_PIN);
Servo endEffectorServo;

// Function Declerations
void homeAllAxes();
void reportPosition();
bool runMotors();
void processCommand(String cmd);
void moveBase(float degrees);
void moveShoulder(float degrees);
void moveSlide(float mm);
void homeBase();
void homeShoulder();
void homeSlide();

// System state
bool emergency = false;
float basePos = -4;
float shoulderPos = 0;
float slidePos = 0;
bool baseHomed = false;
bool shoulderHomed = false;
bool slideHomed = false;
bool servoEnabled = false;
unsigned long lastEncoderReadTime = 0;
const unsigned long ENCODER_READ_INTERVAL = 200; // Read encoders every 200ms
float shoulderMagneticEncoderPos = 0; // Store the magnetic encoder position

// Function to read raw angle from AS5600 encoder
float readEncoder(int address) {
  Wire.beginTransmission(address);
  Wire.write(0x0C); // Register for raw angle
  Wire.endTransmission(false);
  Wire.requestFrom(address, 2); // Request 2 bytes
  
  if (Wire.available() >= 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    uint16_t rawAngle = (highByte << 8) | lowByte;
    
    // Convert to degrees (0-360)
    return (rawAngle * 360.0) / 4096.0;
  }
  return -1; // Error reading
}

// Function to read position from the magnetic encoder on shoulder
float readMagneticEncoder() {
  Wire.beginTransmission(MAGNETIC_ENCODER_ADDR);
  Wire.write(0x0C); // Register for raw angle (may need to be adjusted)
  Wire.endTransmission(false);
  Wire.requestFrom(MAGNETIC_ENCODER_ADDR, 2); // Request 2 bytes
  
  if (Wire.available() >= 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    uint16_t rawAngle = (highByte << 8) | lowByte;
    
    // Convert to degrees (0-360)
    return (rawAngle * 360.0) / 4096.0;
  }
  return -1; // Error reading
}

// Check if encoder is present
bool checkEncoder(int address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  return (error == 0);
}

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Initialize I2C
  
  // Configure pins
  pinMode(BASE_ENABLE_PIN, OUTPUT);
  pinMode(SHOULDER_ENABLE_PIN, OUTPUT);
  pinMode(SLIDE_ENABLE_PIN, OUTPUT);
  pinMode(BASE_LIMIT_PIN, INPUT_PULLUP);
  pinMode(SHOULDER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(SLIDE_LIMIT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SDA_PIN, INPUT); // SDA is already configured by Wire.begin()
  pinMode(ENCODER_SCL_PIN, INPUT); // SCL is already configured by Wire.begin()
  
  // Configure motors
  baseMotor.setMaxSpeed(500);
  baseMotor.setAcceleration(200);
  shoulderMotor.setMaxSpeed(400);
  shoulderMotor.setAcceleration(200);
  slideMotor.setMaxSpeed(1000);
  slideMotor.setAcceleration(200);
  
  // Initialize servo
  endEffectorServo.attach(SERVO_PIN);
  endEffectorServo.write(90);
  endEffectorServo.detach();
  
  // Initialize motors - initially disabled
  digitalWrite(BASE_ENABLE_PIN, HIGH);
  digitalWrite(SHOULDER_ENABLE_PIN, HIGH);
  digitalWrite(SLIDE_ENABLE_PIN, HIGH);
  
  // Check if encoders are present
  Serial.println("Robot arm ready");
  Serial.print("Base encoder: ");
  Serial.println(checkEncoder(ENCODER_BASE_ADDR) ? "Connected" : "Not found");
  Serial.print("Shoulder encoder: ");
  Serial.println(checkEncoder(ENCODER_SHOULDER_ADDR) ? "Connected" : "Not found");
  Serial.print("Magnetic shoulder encoder: ");
  Serial.println(checkEncoder(MAGNETIC_ENCODER_ADDR) ? "Connected" : "Not found");
  
  Serial.println("Commands:");
  Serial.println("  base X, shoulder X, slide X - Move to position");
  Serial.println("  home base/shoulder/slide/all - Home axes");
  Serial.println("  encoder - Read encoder values");
  Serial.println("  status - Show system status");
}

void loop() {
  // Run motors if moving
  if (!emergency) {
    baseMotor.run();
    shoulderMotor.run();
    slideMotor.run();
  }
  
  // Update servo if enabled
  if (servoEnabled) {
    float servoAngle = -shoulderPos + 90.0;
    servoAngle = constrain(servoAngle, 0, 180);
    endEffectorServo.write(servoAngle);
  }
  
  // Periodically read encoders for monitoring
  if (millis() - lastEncoderReadTime > ENCODER_READ_INTERVAL) {
    lastEncoderReadTime = millis();
    
    // Read magnetic encoder position
    float newMagneticPos = readMagneticEncoder();
    if (newMagneticPos >= 0) {
      shoulderMagneticEncoderPos = newMagneticPos;
    }
  }
  
  // Check for commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    processCommand(cmd);
  }
}

// Process incoming commands
void processCommand(String cmd) {
  cmd.toLowerCase();
  
  // Emergency commands
  if (cmd == "stop") {
    emergency = true;
    baseMotor.stop();
    shoulderMotor.stop();
    slideMotor.stop();
    Serial.println("EMERGENCY STOP");
    return;
  }
  
  if (cmd == "resume") {
    emergency = false;
    Serial.println("Resumed");
    return;
  }
  
  // Skip other commands in emergency mode
  if (emergency) {
    Serial.println("In emergency stop");
    return;
  }
  
  // Encoder read command
  if (cmd == "encoder") {
    float baseAngle = readEncoder(ENCODER_BASE_ADDR);
    float shoulderAngle = readEncoder(ENCODER_SHOULDER_ADDR);
    float magneticShoulderAngle = readMagneticEncoder();
    
    Serial.print("Base encoder: ");
    if (baseAngle >= 0) {
      Serial.print(baseAngle);
      Serial.println(" degrees");
    } else {
      Serial.println("Error reading");
    }
    
    Serial.print("Shoulder encoder: ");
    if (shoulderAngle >= 0) {
      Serial.print(shoulderAngle);
      Serial.println(" degrees");
    } else {
      Serial.println("Error reading");
    }
    
    Serial.print("Magnetic shoulder encoder: ");
    if (magneticShoulderAngle >= 0) {
      Serial.print(magneticShoulderAngle);
      Serial.println(" degrees");
    } else {
      Serial.println("Error reading");
    }
    
    // Also print the motor position
    Serial.print("Shoulder motor position: ");
    Serial.print(shoulderPos);
    Serial.println(" degrees");
    return;
  }
  
  // Movement commands
  if (cmd.startsWith("base ")) {
    float pos = cmd.substring(5).toFloat();
    moveBase(pos);
  }
  else if (cmd.startsWith("shoulder ")) {
    float pos = cmd.substring(9).toFloat();
    moveShoulder(pos);
  }
  else if (cmd.startsWith("slide ")) {
    float pos = cmd.substring(6).toFloat();
    moveSlide(pos);
  }
  
  // Home commands
  else if (cmd == "home base") {
    homeBase();
  }
  else if (cmd == "home shoulder") {
    homeShoulder();
  }
  else if (cmd == "home slide") {
    homeSlide();
  }
  else if (cmd == "home all") {
    homeBase();
    homeShoulder();
    homeSlide();
  }
  
  // Servo commands
  else if (cmd == "servo on") {
    endEffectorServo.attach(SERVO_PIN);
    servoEnabled = true;
    Serial.println("Servo auto-leveling enabled");
  }
  else if (cmd == "servo off") {
    servoEnabled = false;
    endEffectorServo.detach();
    Serial.println("Servo disabled");
  }
  else if (cmd.startsWith("servo ")) {
    int angle = cmd.substring(6).toInt();
    angle = constrain(angle, 0, 180);
    endEffectorServo.attach(SERVO_PIN);
    endEffectorServo.write(angle);
    Serial.print("Servo set to ");
    Serial.println(angle);
  }
  
  // Status command
  else if (cmd == "status") {
    Serial.print("Base: ");
    Serial.print(basePos);
    Serial.print(" Shoulder: ");
    Serial.print(shoulderPos);
    Serial.print(" Slide: ");
    Serial.println(slidePos);
    
    // Read encoders
    float baseAngle = readEncoder(ENCODER_BASE_ADDR);
    float shoulderAngle = readEncoder(ENCODER_SHOULDER_ADDR);
    float magneticShoulderAngle = readMagneticEncoder();
    
    Serial.print("Base encoder: ");
    if (baseAngle >= BASE_MIN && baseAngle <= BASE_MAX) {
      Serial.print(baseAngle);
      Serial.println(" degrees");
    } else {
      Serial.println("Not available");
    }
    
    Serial.print("Shoulder encoder: ");
    if (shoulderAngle >= SHOULDER_MIN && shoulderAngle <= SHOULDER_MAX) {
      Serial.print(shoulderAngle);
      Serial.println(" degrees");
    } else {
      Serial.println("Not available");
    }
    
    Serial.print("Magnetic shoulder encoder: ");
    if (magneticShoulderAngle >= SHOULDER_MIN && magneticShoulderAngle <= SHOULDER_MAX) {
      Serial.print(magneticShoulderAngle);
      Serial.println(" degrees");
    } else {
      Serial.println("Not available");
    }
  }
}

// Move base to position
void moveBase(float degrees) {
  if (degrees < BASE_MIN || degrees > BASE_MAX) {
    Serial.println("Base position out of range");
    return;
  }
  
  digitalWrite(BASE_ENABLE_PIN, LOW);
  baseMotor.moveTo(degrees * STEPS_PER_DEG_BASE);
  basePos = degrees;
  Serial.print("Moving base to ");
  Serial.println(degrees);
}

// Move shoulder to position
void moveShoulder(float degrees) {
  if (degrees < SHOULDER_MIN || degrees > SHOULDER_MAX) {
    Serial.println("Shoulder position out of range");
    return;
  }
  
  digitalWrite(SHOULDER_ENABLE_PIN, LOW);
  shoulderMotor.moveTo(degrees * STEPS_PER_DEG_SHOULDER);
  shoulderPos = degrees;
  
  // Read magnetic encoder for feedback
  float magneticEncoderPos = readMagneticEncoder();
  Serial.print("Moving shoulder to ");
  Serial.print(degrees);
  Serial.print(" degrees (Magnetic encoder: ");
  Serial.print(magneticEncoderPos);
  Serial.println(" degrees)");
}

// Move slide to position
void moveSlide(float mm) {
  if (mm < SLIDE_MIN || mm > SLIDE_MAX) {
    Serial.println("Slide position out of range");
    return;
  }
  
  digitalWrite(SLIDE_ENABLE_PIN, LOW);
  slideMotor.moveTo(mm * STEPS_PER_MM_SLIDE);
  slidePos = mm;
  Serial.print("Moving slide to ");
  Serial.println(mm);
}

// Home base motor
void homeBase() {
  digitalWrite(BASE_ENABLE_PIN, LOW);
  
  // Move away from limit if already triggered
  if (digitalRead(BASE_LIMIT_PIN) == LOW) {
    baseMotor.setSpeed(200);
    while (digitalRead(BASE_LIMIT_PIN) == LOW) {
      baseMotor.runSpeed();
    }
    baseMotor.stop();
  }
  
  // Move toward limit switch
  baseMotor.setSpeed(-200);
  while (digitalRead(BASE_LIMIT_PIN) == HIGH) {
    baseMotor.runSpeed();
    if (emergency) return;
  }
  
  baseMotor.stop();
  baseMotor.setCurrentPosition(0);
  basePos = 0;
  baseHomed = true;
  
  // Move to home position
  moveBase(5);
}

// Home shoulder motor
void homeShoulder() {
  digitalWrite(SHOULDER_ENABLE_PIN, LOW);
  
  // Move away from limit if already triggered
  if (digitalRead(SHOULDER_LIMIT_PIN) == LOW) {
    shoulderMotor.setSpeed(200);
    while (digitalRead(SHOULDER_LIMIT_PIN) == LOW) {
      shoulderMotor.runSpeed();
    }
    shoulderMotor.stop();
  }
  
  // Move toward limit switch
  shoulderMotor.setSpeed(-200);
  while (digitalRead(SHOULDER_LIMIT_PIN) == HIGH) {
    shoulderMotor.runSpeed();
    if (emergency) return;
  }
  
  shoulderMotor.stop();
  shoulderMotor.setCurrentPosition(0);
  shoulderPos = 0;
  shoulderHomed = true;
  
  // Move to home position
  moveShoulder(5);
}

// Home slide motor
void homeSlide() {
  digitalWrite(SLIDE_ENABLE_PIN, LOW);
  
  // Move away from limit if already triggered
  if (digitalRead(SLIDE_LIMIT_PIN) == LOW) {
    slideMotor.setSpeed(200);
    while (digitalRead(SLIDE_LIMIT_PIN) == LOW) {
      slideMotor.runSpeed();
    }
    slideMotor.stop();
  }
  
  // Move toward limit switch
  slideMotor.setSpeed(-200);
  while (digitalRead(SLIDE_LIMIT_PIN) == HIGH) {
    slideMotor.runSpeed();
    if (emergency) return;
  }
  
  slideMotor.stop();
  slideMotor.setCurrentPosition(0);
  slidePos = 0;
  slideHomed = true;
}