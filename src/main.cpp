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

#define RUN_MANUAL 1
#define RUN_AUTO 2
#define RUN_SERIAL 3
int run_mode = RUN_SERIAL;

AccelStepper leftTread;
AccelStepper rightTread;
unsigned long time_to_run = 0; // time to run the motors
//unsigned long Speed = 0; // Speed in mm a second... someday ;) 
unsigned long Steps = 0; // steps to run
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
unsigned long mmToSteps(unsigned long mm);
void moveMm(unsigned long mm);
void stepIfTime(unsigned long speed);
void stepMotors();

// Main Code
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.print("Starting up\n");

  switch (run_mode) {
    case RUN_MANUAL:
      manualSetup();
      break;
    case RUN_AUTO:
      initializeAccelLib();
      autoSetup();
      break;
    case RUN_SERIAL:
      manualSetup();
      Serial.print("Seral mode\n");
      Serial.print("Enter command: w\n");
      break;
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // Singnle start

  switch (run_mode) {
    case RUN_MANUAL:
      manualTestDrive();
      break;
    case RUN_AUTO:
      autoTestDrive();
      break;
    case RUN_SERIAL:
      while (true) {
        if (Serial.available() > 0) {
          char inChar = Serial.read();
          Serial.print("Received: (");
          Serial.print(inChar);
          Serial.print(")\n");
          switch (inChar) {
            case 'w':
              Serial.print("Forward\n");
              digitalWrite(LEFT_ENABLE_PIN, STEP_ENABLE);
              digitalWrite(RIGHT_ENABLE_PIN, STEP_ENABLE);
              digitalWrite(LEFT_DIR_PIN, LeftFWD);
              digitalWrite(RIGHT_DIR_PIN, RightFWD);
              
              for(i=0; i < 1000; i++) {
                Serial.print("by hand\n");
                stepMotors();
                delay(1);
              } 
              Serial.print("moveMm()\n");
              moveMm(10);
              break;
            default:
              Serial.print("Invalid command\n");
              break;
          }
        }
        stepIfTime(100);
        delay(1);
      }
    default:
      Serial.print("Invalid run mode?!?\n");
      break;
  }
  digitalWrite(LED_BUILTIN, LOW);  // Signal end
  exit(0);
}

//
// Basic manual control functions
//

void stepIfTime(unsigned long speed) {
  if (speed > 0 && Steps > 0) { // If speed is 0, or steps are 0, No stepping
    if(time_to_run == 0) {  // If time to run is 0, set it to now + speed
      time_to_run = millis() + speed;
    }
    if (millis() > time_to_run) {
      Serial.print("Stepping: (");
      Serial.print(Steps);
      Serial.print(")\n");
      stepMotors();
      time_to_run = millis() + speed;
      Steps--;
    }
  }
}

void moveMm(unsigned long mm) {
  Steps = mmToSteps(mm);
  Serial.print("Set Steps to: ");
  Serial.print(Steps, DEC);
  Serial.print("\n");
}

unsigned long mmToSteps(unsigned long mm) {
  return(mm * 10);
}

void stepMotors(){
  // Serial.print("Stepping motors\n");
  digitalWrite(LEFT_ENABLE_PIN, STEP_ENABLE);
  digitalWrite(RIGHT_ENABLE_PIN, STEP_ENABLE);
  digitalWrite(LEFT_DIR_PIN, LeftFWD);
  digitalWrite(RIGHT_DIR_PIN, RightFWD);

  digitalWrite(LEFT_STEP_PIN, HIGH);
  digitalWrite(RIGHT_STEP_PIN, HIGH);

  digitalWrite(LEFT_STEP_PIN, LOW);
  digitalWrite(RIGHT_STEP_PIN, LOW);
}



//
// Manual control functions
//
void manualSetup()  {
  // Set up the servo pins
  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(LEFT_ENABLE_PIN, OUTPUT);

  // Set up the servo pins
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_ENABLE_PIN, OUTPUT);

  // Enable the motors
  digitalWrite(LEFT_ENABLE_PIN, STEP_ENABLE);
  digitalWrite(RIGHT_ENABLE_PIN, STEP_ENABLE);
}

void manualTestDrive()  {
  // Go fwd
  digitalWrite(LEFT_DIR_PIN, LeftFWD);
  digitalWrite(RIGHT_DIR_PIN, RightFWD);
  manualRun(MANUAL_LOOP_CNT, MANUAL_LOOP_DELAY);
  delay(1000);

//   // Spin in place to left
//   digitalWrite(LEFT_DIR_PIN, LeftREV);
//   digitalWrite(RIGHT_DIR_PIN, RightFWD);
//   manualRun(MANUAL_LOOP_CNT, MANUAL_LOOP_DELAY);
//   delay(1000);

// // Spin in place to Right
//   digitalWrite(LEFT_DIR_PIN, LeftFWD);
//   digitalWrite(RIGHT_DIR_PIN, RightREV);
//   manualRun(MANUAL_LOOP_CNT, MANUAL_LOOP_DELAY);
//   delay(1000);

//   // Disable motors... hopefully.
//   digitalWrite(RIGHT_ENABLE_PIN, STEP_DISABLE);
//   digitalWrite(LEFT_ENABLE_PIN, STEP_DISABLE);
}

// pulse pins high/low with delay between pulses to step motors
void manualRun(unsigned int step_cnt, unsigned int delay) {
  for(unsigned int Li=0; Li < step_cnt; Li++) {
    stepMotors();
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
  Serial.print("Auto setup done\n");
}

void autoSetSpeed(float speed) {
  leftTread.setSpeed(speed);
  rightTread.setSpeed(speed);
  Serial.print("Auto set speed done");
}

void autoSetSpeedR(float speed) {
  rightTread.setSpeed(speed);
  Serial.print("Auto set speed done");
}

void autoSetSpeedL(float speed) {
  leftTread.setSpeed(speed);
  Serial.print("Auto set speed done");
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
//
// End of auto control functions
//


// case 's':
//           autoSetSpeed(-3000.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'a':
//           autoSetSpeedL(-3000.0);
//           autoSetSpeedR(3000.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'd':
//           autoSetSpeedL(3000.0);
//           autoSetSpeedR(-3000.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'q':
//           autoSetSpeed(0.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'e':
//           autoSetSpeed(0.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'z':
//           autoSetSpeed(0.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'x':
//           autoSetSpeed(0.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'r':
//           autoSetSpeed(3000.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'f':
//           autoSetSpeed(-3000.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 't':
//           autoSetSpeedL(-3000.0);
//           autoSetSpeedR(3000.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'g':
//           autoSetSpeedL(3000.0);
//           autoSetSpeedR(-3000.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'y':
//           autoSetSpeed(0.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'h':
//           autoSetSpeed(0.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'u':
//           autoSetSpeed(0.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'j':
//           autoSetSpeed(0.0);
//           autoRun(AUTO_LOOP_CNT);
//           break;
//         case 'i':
//           autoSetSpeed(3000.0);