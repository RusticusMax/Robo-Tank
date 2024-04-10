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
unsigned long Speed = 10; // Speed in mm a second... someday ;) 
unsigned long StepDelay = 0; // Delay between steps
float DistanceMM = 10; // distance to run
unsigned long Steps = 0; // steps to run

unsigned int i = 0;
bool executeMove = false;

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
unsigned long mmToSteps(float mm);
unsigned long mmToDelay(float mm);
int setDistanceMm(float mm);
void stepIfTime(unsigned long speed);
void stepMotors();
void byHand();
float getParam();
char scanChar();

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
      Serial.print("Enter command:\n");
      Serial.print("f = Forward\n");
      Serial.print("b = Back\n");
      Serial.print("r = Right\n");
      Serial.print("l = Left\n");
      Serial.print("e = Engage Motors\n");
      Serial.print("d = Disengage Motors\n");
      Serial.print("sxx = Set Speed\n");
      Serial.print("mxx = Set Distance in mm\n");
      Serial.print("axx = Set Absolute Steps directly\n");
      Serial.print("t = Test\n");
      break;
  }
}

void loop() {
  char inChar = 0;
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
        //if (Serial.available() > 0) {
        if((inChar = scanChar()) != 0) { // If there is a character in the buffer, read it and act on it
          // char inChar = Serial.read();
          Serial.print("Received: (");
          Serial.print(inChar, HEX);
          Serial.print(")\n");
          switch (inChar) {  //fbrldesmta  abdeflmrst
            case 'f':
              Serial.print("Forward\n");
              digitalWrite(LEFT_DIR_PIN, LeftFWD);
              digitalWrite(RIGHT_DIR_PIN, RightFWD);
              executeMove = true;
              break;
            case 'b':
              Serial.print("Backward\n");
              digitalWrite(LEFT_DIR_PIN, LeftREV);
              digitalWrite(RIGHT_DIR_PIN, RightREV);
              executeMove = true;
              break;
            case 'r':
              Serial.print("Right\n");
              digitalWrite(LEFT_DIR_PIN, LeftFWD);
              digitalWrite(RIGHT_DIR_PIN, RightREV);
              executeMove = true;
              break;
            case 'l':
              Serial.print("Left\n");
              digitalWrite(LEFT_DIR_PIN, LeftREV);
              digitalWrite(RIGHT_DIR_PIN, RightFWD);
              executeMove = true;
              break;
            case 'd':
              Serial.print("DisableMotors\n");
              digitalWrite(LEFT_ENABLE_PIN, STEP_DISABLE);
              digitalWrite(RIGHT_ENABLE_PIN, STEP_DISABLE);
              break;
            case 'e':
              Serial.print("Enable Motors\n");
              digitalWrite(LEFT_ENABLE_PIN, STEP_ENABLE);
              digitalWrite(RIGHT_ENABLE_PIN, STEP_ENABLE);
              break;
            case 's':
              Serial.print("Set Speed\n");
              Speed = getParam();
              StepDelay = mmToDelay(Speed);
              Serial.print("Speed Set: (");
              Serial.print(Speed);
              Serial.print(")\n");
              break;
            case 'm':
              Serial.print("Set Distance\n");
              DistanceMM = getParam();
              Steps = setDistanceMm(DistanceMM);
              Serial.print("Distance Set: (");
              Serial.print(DistanceMM);
              Serial.print(")\n");
              break;
            case 'a':
              Serial.print("Set Absolute Steps\n");
              Steps = getParam();
              Serial.print("Steps Set: (");
              Serial.print(Steps);
              Serial.print(")\n");
              break;
            case 't':
              Serial.print("Test Routine\n");
              Serial.print("drive by hand\n");
              digitalWrite(LEFT_DIR_PIN, LeftFWD);
              digitalWrite(RIGHT_DIR_PIN, RightFWD);
              byHand();
            default:
              Serial.print("Invalid command\n");
              break;
          }
        }
        while(executeMove)  {
          stepIfTime(Speed);
        }
      }
    default:
      Serial.print("Invalid run mode?!?\n");
      break;
  }
  digitalWrite(LED_BUILTIN, LOW);  // Signal end
  exit(0);
}

char scanChar() {
  char inChar = 0;
  if (Serial.available() > 0) {
    inChar = Serial.read();
  }
  return inChar;
}

void byHand() {
  for(i=0; i < 1000; i++) {
    Serial.print("by hand\n");
    stepMotors();
    delay(1);
  }
}

// Eat all characters from the serial buffer and return them as a single number
// Ignore all non-numeric characters
float getParam() {
  float param = 0;
  // float temp = 0;
  // bool postDecimal = false; // After decimal point change how we handle the number
  String inString = "";
  char inChar;

  //delay(1); // let the buffer fill up ()
  //while (Serial.available() > 0) {
  while((inChar = scanChar()) != 0) {
    //inChar = Serial.read();
    inString.concat(inChar);
  }
  param = inString.toFloat();

  Serial.print("Param: ");
  Serial.print(param);
  Serial.print("\n");
  return param;
}

//
// Basic manual control functions
//

void stepIfTime(unsigned long speed) {
  if (executeMove && StepDelay > 0 && Steps > 0) { // If speed is 0, or steps are 0, No stepping
    // if(time_to_run == 0) {  // If time to run is 0, set it to now + speed
    //   time_to_run = millis() + speed;
    // }
    if (micros() > time_to_run) {
      // Serial.print("Stepping: (");
      // Serial.print(Steps);
      // Serial.print(")\n");
      stepMotors();
      time_to_run = micros() + StepDelay;
      Steps--;
      if(Steps == 0) {
        executeMove = false;
        Serial.print("Move complete\n");
      }
    }
  }
}

int setDistanceMm(float mm) {
  return (mmToSteps(mm));
}

unsigned long mmToSteps(float mm) {
  float tmp;
  tmp = mm * 22.64150943396;
  return (unsigned long)(tmp + 0.5);  // round up
}

// mm per second to delay between steps
// 200 steps per rev, 1.8 degrees per step, 38mm per rev (wheel diameter)
// ideally 5ms per step is 1 rev per second = 38mm per second
// 38mm/200steps = 0.19mm per step
unsigned long mmToDelay(float mm) { 
  float tmp;
  tmp = 1000000.0/(mm/0.19);  // Delay in microseconds
  Serial.print("Delay: (");
  Serial.print(tmp);
  Serial.print(")\n");
  return (unsigned long)(tmp + 0.5);  // round up
}

void stepMotors(){
  // Serial.print("Stepping motors\n");
  // digitalWrite(LEFT_ENABLE_PIN, STEP_ENABLE);
  // digitalWrite(RIGHT_ENABLE_PIN, STEP_ENABLE);
  // digitalWrite(LEFT_DIR_PIN, LeftFWD);
  // digitalWrite(RIGHT_DIR_PIN, RightFWD);

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