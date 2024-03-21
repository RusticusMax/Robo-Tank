#include <Arduino.h>
#include <AccelStepper.h>

// Define some steppers and the pins the will use
#define RIGHT_STEP_PIN 2
#define RIGHT_DIR_PIN 3
#define RIGHT_ENABLE_PIN 4

#define LEFT_STEP_PIN 5
#define LEFT_DIR_PIN 6
#define LEFT_ENABLE_PIN 7

#define LeftFWD LOW
#define LeftREV HIGH
#define RightFWD HIGH
#define RightREV LOW
#define STEP_ENABLE LOW
#define STEP_DISABLE HIGH

#define AUTO_LOOP_CNT 70000
#define AUTO_LOOP_DELAY 1
#define MANUAL_LOOP_CNT 7000
#define MANUAL_LOOP_DELAY 200

#define RUN_MANUAL false

AccelStepper leftTread;
AccelStepper rightTread;

unsigned int i = 0;

// put function declarations here:
void manualRun(unsigned int step_cnt, unsigned int delay);
void manualSetup();
void manualTestDrive();

void autoRun(long step_cnt);
void autoSetup();
void autoSetSpeed(float speed);
void autoSetSpeedR(float speed);
void autoSetSpeedL(float speed);
void autoTestDrive();
void initializeAccelLib();

// Main Code
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.write("Starting up\n");

  if (RUN_MANUAL) {
    manualSetup();
  } else {
    // Auto mode
    initializeAccelLib();
    autoSetup();
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // Singnle start
  if (RUN_MANUAL) {
    manualTestDrive();
  } else {
    autoTestDrive();
  }
  digitalWrite(LED_BUILTIN, LOW);  // Single end
  exit(0);
}

//
// Manual control functions
//
void manualSetup()  {
  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(LEFT_ENABLE_PIN, OUTPUT);

  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_ENABLE_PIN, OUTPUT);

  digitalWrite(LEFT_ENABLE_PIN, STEP_ENABLE);
  digitalWrite(RIGHT_ENABLE_PIN, STEP_ENABLE);
}

void manualTestDrive()  {
  // Go fwd
  digitalWrite(LEFT_DIR_PIN, LeftFWD);
  digitalWrite(RIGHT_DIR_PIN, RightFWD);
  manualRun(MANUAL_LOOP_CNT, MANUAL_LOOP_DELAY);
  delay(1000);

  // Spin in place to left
  digitalWrite(LEFT_DIR_PIN, LeftREV);
  digitalWrite(RIGHT_DIR_PIN, RightFWD);
  manualRun(MANUAL_LOOP_CNT, MANUAL_LOOP_DELAY);
  delay(1000);

// Spin in place to Right
  digitalWrite(LEFT_DIR_PIN, LeftFWD);
  digitalWrite(RIGHT_DIR_PIN, RightREV);
  manualRun(MANUAL_LOOP_CNT, MANUAL_LOOP_DELAY);
  delay(1000);

  // Disable motors... hopefully.
  digitalWrite(RIGHT_ENABLE_PIN, STEP_DISABLE);
  digitalWrite(LEFT_ENABLE_PIN, STEP_DISABLE);
}

void manualRun(unsigned int step_cnt, unsigned int delay) {
  for(unsigned int Li=0; Li < step_cnt; Li++) {
    digitalWrite(LEFT_STEP_PIN, HIGH);
    digitalWrite(LEFT_STEP_PIN, LOW);

    digitalWrite(RIGHT_STEP_PIN, HIGH);
    digitalWrite(RIGHT_STEP_PIN, LOW);

    delayMicroseconds(delay);
  }
}

//
// Auto control functions
//
void autoSetup()  {
  leftTread.setEnablePin(RIGHT_ENABLE_PIN);
  // Direction, step, enable
  leftTread.setPinsInverted(true, false, true); // Invert enable (low = enabled)
  leftTread.enableOutputs();
  leftTread.setMinPulseWidth(20);

  rightTread.setEnablePin(LEFT_ENABLE_PIN);
  rightTread.setPinsInverted(false, false, true);
  rightTread.enableOutputs();
  rightTread.setMinPulseWidth(20);

  leftTread.setMaxSpeed(10000.0);
  leftTread.setAcceleration(500.0);
  rightTread.setMaxSpeed(10000.0);
  rightTread.setAcceleration(500.0);
  Serial.write("Auto setup done\n");
}

void autoSetSpeed(float speed) {
  leftTread.setSpeed(speed);
  rightTread.setSpeed(speed);
  Serial.write("Auto set speed done");
}

void autoSetSpeedR(float speed) {
  rightTread.setSpeed(speed);
  Serial.write("Auto set speed done");
}

void autoSetSpeedL(float speed) {
  leftTread.setSpeed(speed);
  Serial.write("Auto set speed done");
}


void autoTestDrive() {
  digitalWrite(LED_BUILTIN, HIGH);
  
  autoSetSpeed(3000.0);
  autoRun(AUTO_LOOP_CNT);
  delay(1000);

  autoSetSpeedL(-3000.0);
  autoRun(AUTO_LOOP_CNT);
  delay(1000);

  autoSetSpeedL(3000.0);
  autoSetSpeedR(-3000.0);
  autoRun(AUTO_LOOP_CNT);

  digitalWrite(LED_BUILTIN, LOW);
  leftTread.disableOutputs();
  rightTread.disableOutputs();
}

void autoRun(long loop_cnt) {
  long i = 0;
  while(i < loop_cnt) {
    i++;
    leftTread.runSpeed();
    rightTread.runSpeed();
  }
}

void initializeAccelLib() {
  leftTread = AccelStepper(AccelStepper::DRIVER, RIGHT_STEP_PIN, RIGHT_DIR_PIN);
  rightTread = AccelStepper(AccelStepper::DRIVER, LEFT_STEP_PIN, LEFT_DIR_PIN);
}