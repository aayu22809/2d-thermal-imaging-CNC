#include <AccelStepper.h>
#include <Servo.h>
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

// Slide Motor
const int SLIDE_STEP_PIN = 30;
const int SLIDE_DIR_PIN = 31;
const int SLIDE_ENABLE_PIN = 32;
const int SLIDE_LIMIT_PIN = 33;

// Servo
const int SERVO_PIN = 9;

// Configuration
const float STEPS_PER_DEG_BASE = 4.0833;    // (200 * 16 * 10) / 360
const float STEPS_PER_DEG_SHOULDER = 6.88;  // (200 * 16 * 20) / 360
const float STEPS_PER_MM_SLIDE = 25.0;      // (200 * 16 * 5) / 40

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

// Function declarations
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

void setup() {
  Serial.begin(9600);
  
  // Configure pins
  pinMode(BASE_ENABLE_PIN, OUTPUT);
  pinMode(SHOULDER_ENABLE_PIN, OUTPUT);
  pinMode(SLIDE_ENABLE_PIN, OUTPUT);
  pinMode(BASE_LIMIT_PIN, INPUT_PULLUP);
  pinMode(SHOULDER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(SLIDE_LIMIT_PIN, INPUT_PULLUP);
  
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
  
  Serial.println("Robot arm ready");
  Serial.println("Commands:");
  Serial.println("  base X, shoulder X, slide X - Move to position");
  Serial.println("  home base/shoulder/slide/all - Home axes");
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
  
  // Movement commands
  if (cmd.startsWith("base ")) {
    moveBase(cmd.substring(5).toFloat());
  }
  else if (cmd.startsWith("shoulder ")) {
    moveShoulder(cmd.substring(9).toFloat());
  }
  else if (cmd.startsWith("slide ")) {
    moveSlide(cmd.substring(6).toFloat());
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
  Serial.print("Moving shoulder to ");
  Serial.println(degrees);
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