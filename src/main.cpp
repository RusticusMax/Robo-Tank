#include <Arduino.h>
#include <AccelStepper.h>

// Define some steppers and the pins the will use
#define LeftStep 2
#define LeftDir 3
#define LeftEnable 4

#define RightStep 5
#define RightDir 6
#define RightEnable 7

#define LeftFWD HIGH
#define LeftREV LOW
#define RightFWD LOW
#define RightREV HIGH

#define LOOP_CNT 100000
#define LOOP_DELAY 1

AccelStepper Left(AccelStepper::DRIVER, RightStep, RightDir);
AccelStepper Right(AccelStepper::DRIVER, LeftStep, LeftDir);

int i = 0;

// put function declarations here:
void runBothManual(int loop_cnt, int delay);
void manualSetup();
void manualTestDrive();

void setup() {
  manualSetup();
}

void loop() {

  //manualTestDrive();
  //exit(0);

  if(i < LOOP_CNT){
    digitalWrite(LED_BUILTIN, HIGH);
    Left.setMaxSpeed(3000.0);
    Left.setAcceleration(1000.0);
    Right.setMaxSpeed(3000.0);
    Right.setAcceleration(1000.0);
    Left.setSpeed(1500.0);
    i++;
    //delay(LOOP_DELAY);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    Left.setSpeed(0);
    
  }
  Left.run();
}

void runBothManual(int loop_cnt, int delay) {
  for(int Li=0; Li < loop_cnt; Li++) {
    digitalWrite(LeftStep, HIGH);
    digitalWrite(RightStep, HIGH);

    digitalWrite(LeftStep, LOW);
    digitalWrite(RightStep, LOW);
    delayMicroseconds(delay);
  }
}

void manualSetup()  {
  pinMode(LeftStep, OUTPUT);
  pinMode(LeftDir, OUTPUT);
  pinMode(LeftEnable, OUTPUT);
  pinMode(RightStep, OUTPUT);
  pinMode(RightDir, OUTPUT);
  pinMode(RightEnable, OUTPUT);

  digitalWrite(LeftEnable, HIGH);
  digitalWrite(LeftDir, LOW);

  digitalWrite(RightEnable, HIGH);
  digitalWrite(RightDir, HIGH);

  // Left.setEnablePin(RightEnable);
  // Left.enableOutputs();
  // Right.setEnablePin(LeftEnable);
  // Right.enableOutputs();
  pinMode(LED_BUILTIN, OUTPUT);
}

void manualTestDrive()  {
  digitalWrite(LED_BUILTIN, HIGH);
  // Go fwd
  digitalWrite(LeftDir, LeftFWD);
  digitalWrite(RightDir, RightFWD);
  runBothManual(LOOP_CNT, LOOP_DELAY);
  delay(1000);

  // Spin in place to left
  digitalWrite(LeftDir, LeftREV);
  digitalWrite(RightDir, RightFWD);
  runBothManual(LOOP_CNT, LOOP_DELAY);
  delay(1000);

// Spin in place to Right
  digitalWrite(LeftDir, LeftFWD);
  digitalWrite(RightDir, RightREV);
  runBothManual(LOOP_CNT, LOOP_DELAY);
  delay(1000);

  // Disable motors... hopefully.
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(RightEnable, LOW);
  digitalWrite(LeftEnable, LOW);
}
