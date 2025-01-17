#include <Arduino.h>
#include <limits.h>

// Define steppers and the pins the will use
#define RIGHT_STEP_PIN 2
#define RIGHT_DIR_PIN 3
#define RIGHT_ENABLE_PIN 4

#define LEFT_STEP_PIN 5
#define LEFT_DIR_PIN 6
#define LEFT_ENABLE_PIN 7

// Make logical names for states
#define LeftFWD LOW
#define LeftREV HIGH
#define RightFWD HIGH
#define RightREV LOW
#define STEP_ENABLE LOW
#define STEP_DISABLE HIGH

#define RIGHT_MOTOR 1
#define LEFT_MOTOR 2

#define MANUAL_LOOP_CNT 7000
#define MANUAL_LOOP_DELAY 200

#define WHEEL_DIST 4 // this is for pivot (emperical) // actual distance 222.3 // in inches 8.75"

//float Speed = 50.0; // Speed in mm a second
float LSpeed = 50, RSpeed = 50.0; // Speed in mm a second
//unsigned long StepDelayMs = 0; // Delay between steps. Derived from Speed
unsigned long RStepDelayMs = 0, LStepDelayMs = 0; // Delay between steps. Derived from Speed
unsigned long LastStepTime = 0; // time of last step.  Set as counter for next step
unsigned long RLastStepTime = 0, LLastStepTime = 0; // time of last step.  Set as counter for next step
//float DistanceMM = 10; // distance to run
float RDistanceMM = 10, LDistanceMM = 10; // distance to run
//unsigned long Steps = 0; // steps to run. Derived from DistanceMM
unsigned long RSteps = 0, LSteps = 0; // steps to run. Derived from DistanceMM
// String Playback[] = {
//   "s50 m80 r", 
//   "s1000 m500 b",
//   };
//int PlaybackIndex = -1;   // The predefined string to playback.  -1 is not playing back
//int PlaybackStep = 0;     // The current step in the playback string
// float PivotAngle = 0; // Angle to pivot with differential drive

// unsigned int i = 0;
// bool executeMove = false; // Flag to process the timeToStep function to run the motors
bool RexecuteMove = false; // Flag to process the timeToStep function to run the motors
bool LexecuteMove = false; // Flag to process the timeToStep function to run the motors

// Twist message Tlx,ly,lz,ax,ay,az
struct Twist {
  float lx;
  float ly;
  float lz;
  float ax;
  float ay;
  float az;
};

// put function declarations here:
void manualRun(unsigned int step_cnt, unsigned int delay);
void manualSetup();
void manualTestDrive();

unsigned long mmToSteps(float mm);
unsigned long mmASecToMsDelay(float mm);
int setDistanceMm(float mm);
void stepIfTime();
void stepMotors(int motor);
float getParam();
char scanChar();
float degToDiffDistance(float deg);
void prepToMove();
void stopAllMotors();
void processTwist(struct Twist twist);
void startMove(int motor);

// Main Code
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  // Serial.print("Starting up\n");

  manualSetup();
  // Serial.print("Seral mode\n");
  // Serial.print("Enter command:\n");
  // Serial.print("f = Set Forward\n");
  // Serial.print("b = Set Backward\n");
  // Serial.print("r = Set Right\n");
  // Serial.print("l = Set Left\n");
  // Serial.print("px.x = Set pivot in degrees\n");
  // Serial.print("g = Go based in set speed/distance/direction\n");
  // Serial.print("e = Engage Motors\n");
  // Serial.print("d = Disengage Motors\n");
  // Serial.print("sx.x = Set Speed in mm/sec\n");
  // Serial.print("Sx.x = Adjust Speed in mm/sec\n");
  // Serial.print("mx.x = Set Distance in mm\n");
  // Serial.print("axx = Set Absolute Steps directly\n");
  // Serial.print("Px = Playback preset command sequence\n");

  // set defaults
  // StepDelayMs = mmASecToMsDelay(Speed);
  RStepDelayMs = mmASecToMsDelay(RSpeed);
  LStepDelayMs = mmASecToMsDelay(LSpeed);

  // Steps = setDistanceMm(DistanceMM);
  RSteps = setDistanceMm(RDistanceMM);
  LSteps = setDistanceMm(LDistanceMM);
}

void loop() {
  char inChar = 0;
  struct Twist twist = {0,0,0,0,0,0};
  float deltaSpeed = 0; // Tmp for computing relitive speed

  if((inChar = scanChar()) != 0) { // If there is a character in the buffer, read it and act on it
    // Serial.print("Received: (");
    // Serial.print(inChar, HEX);
    // Serial.print(")\n");
    delay(2); // wait for the rest of the message to arrive (10 char@115200 = 0.87ms?)
    switch (inChar) {  //fbrldesmaPpwST  abdeflmprswPST
      case 'f':
        Serial.print("Forward\n");
        digitalWrite(LEFT_DIR_PIN, LeftFWD);
        digitalWrite(RIGHT_DIR_PIN, RightFWD);
        startMove(LEFT_MOTOR + RIGHT_MOTOR);
        break;
      case 'b':
        Serial.print("Backward\n");
        digitalWrite(LEFT_DIR_PIN, LeftREV);
        digitalWrite(RIGHT_DIR_PIN, RightREV);
        startMove(LEFT_MOTOR + RIGHT_MOTOR);
        break;
      case 'r':
        Serial.print("Right\n");
        digitalWrite(LEFT_DIR_PIN, LeftFWD);
        digitalWrite(RIGHT_DIR_PIN, RightREV);
        startMove(RIGHT_MOTOR);
        break;
      case 'l':
        Serial.print("Left\n");
        digitalWrite(LEFT_DIR_PIN, LeftREV);
        digitalWrite(RIGHT_DIR_PIN, RightFWD);
        startMove(LEFT_MOTOR);
        break;
      // case 'p': // Pivot right by n degrees
      //   PivotAngle = getParam();
      //   DistanceMM = degToDiffDistance(PivotAngle);
      //   Steps = setDistanceMm(DistanceMM);
      //   digitalWrite(LEFT_DIR_PIN, LeftFWD);
      //   digitalWrite(RIGHT_DIR_PIN, RightREV);
      //   Serial.print("Pivot Set: (");
      //   Serial.print(PivotAngle);
      //   Serial.print(")\n");
      //   break;
      case 'g':
        Serial.print("Go\n");
        prepToMove();
        startMove(LEFT_MOTOR + RIGHT_MOTOR);
        break;
      case 'h':
        Serial.print("Halt\n");
        stopAllMotors();
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
        RSpeed = getParam();
        LSpeed = RSpeed;
        RStepDelayMs = mmASecToMsDelay(RSpeed);
        LStepDelayMs = RStepDelayMs;
        Serial.print("Speed Set: (R:");
        Serial.print(RSpeed);
        Serial.print(" - L:");
        Serial.print(LSpeed);
        Serial.print(")\n");
        break;
      case 'S':
        Serial.print("Set Relitive Speed\n");
        deltaSpeed = getParam();
        RSpeed += deltaSpeed;
        LSpeed += deltaSpeed;
        RStepDelayMs = mmASecToMsDelay(RSpeed);
        LStepDelayMs = mmASecToMsDelay(LSpeed);
        Serial.print("Speed Set: (R:");
        Serial.print(RSpeed);
        Serial.print(" - L:");
        Serial.print(LSpeed);
        Serial.print(")\n");
        break;
      case 'm':
        Serial.print("Set Distance\n");
        RDistanceMM = getParam();
        LDistanceMM = RDistanceMM;
        RSteps = setDistanceMm(RDistanceMM);
        LSteps = setDistanceMm(LDistanceMM);
        Serial.print("Distance Set: (R:");
        Serial.print(RDistanceMM);
        Serial.print(" - L:");
        Serial.print(LDistanceMM);
        Serial.print(")\n");
        break;
      case 'a':
        Serial.print("Set Absolute Steps\n");
        RSteps = getParam();
        LSteps = RSteps;
        Serial.print("Steps Set: (R:");
        Serial.print(RSteps);
        Serial.print(" - L:");
        Serial.print(LSteps);
        Serial.print(")\n");
        break;
      // case 'P':
      //   PlaybackIndex = (int)getParam();
      //   Serial.print("Received: (");
      //   Serial.print(inChar, HEX);
      //   Serial.print(")\n");
      //   if(PlaybackIndex >= 0 && PlaybackIndex < (int)sizeof(Playback)) {
      //     Serial.print("Playback: (");
      //     Serial.print(Playback[PlaybackIndex]);
      //     Serial.print(")\n");
      //     PlaybackStep = 0;
      //   } else {
      //     Serial.print("Invalid playback\n");
      //     PlaybackIndex = -1;
      //   }
      //   break;
      case 'w':
        Serial.print("Wait\n");
        delay(getParam());
        break;
      case 'T': // Twist message Tlx,ly,lz,ax,ay,az
        // Serial.print("Twist\n");
        twist.lx = getParam();
        twist.ly = getParam();
        twist.lz = getParam();
        twist.ax = getParam();
        twist.ay = getParam();
        twist.az = getParam();
        processTwist(twist);  // Set global RSpeed and LSpeed for motor control
        prepToMove();         // Set up the motor control variables, and set state to move next step
        startMove(LEFT_MOTOR + RIGHT_MOTOR);
        Serial.print("Twisting: R(");
        Serial.print(RSpeed);
        Serial.print("), L(");
        Serial.print(LSpeed); 
        Serial.print(") * (");
        Serial.print(twist.lx);
        Serial.print(", ");
        Serial.print(twist.ly);
        Serial.print(", ");
        Serial.print(twist.lz);
        Serial.print(", ");
        Serial.print(twist.ax);
        Serial.print(", ");
        Serial.print(twist.ay);
        Serial.print(", ");
        Serial.print(twist.az);
        Serial.print(")\n");
        break;
      default:
        Serial.print("Invalid command: (");
        Serial.print(inChar, HEX);
        Serial.print(")  (");
        Serial.print(inChar);
        Serial.print(")\n");
        break;
    }
  }
  stepIfTime();
}


void startMove(int motor) {
  //if((motor | RIGHT_MOTOR) == RIGHT_MOTOR) {
    RexecuteMove = true;
  //}
  //if((motor | LEFT_MOTOR) == LEFT_MOTOR) {
    LexecuteMove = true;
  //}
}

// Turn a twist message into motor speeds
void processTwist(struct Twist twist)  {
  float rsp, lsp;

  rsp = (twist.az * WHEEL_DIST) /2 + twist.lx;
  lsp = twist.lx * 2 - rsp;
  RSpeed = rsp;
  LSpeed = lsp;
}

// Prepare internal states for executing a move command
void prepToMove() {
  digitalWrite(LEFT_ENABLE_PIN, STEP_ENABLE);
  digitalWrite(RIGHT_ENABLE_PIN, STEP_ENABLE);
  // Steps = setDistanceMm(DistanceMM);
  RSteps = setDistanceMm(RDistanceMM);
  LSteps = setDistanceMm(LDistanceMM);

  // StepDelayMs = mmASecToMsDelay(Speed);
  RStepDelayMs = mmASecToMsDelay(RSpeed);
  LStepDelayMs = mmASecToMsDelay(LSpeed);

  if(RSpeed < 0) {
    digitalWrite(RIGHT_DIR_PIN, RightREV);
  } else {
    digitalWrite(RIGHT_DIR_PIN, RightFWD);
  }
  if(LSpeed < 0) {
    digitalWrite(LEFT_DIR_PIN, LeftREV);
  } else {
    digitalWrite(LEFT_DIR_PIN, LeftFWD);
  }
}

void stopAllMotors() {
  // executeMove = false;
  RexecuteMove = false;
  LexecuteMove = false;
}

// Scan input for char nonblocking 
// Also allows for playback of recorded commands
char scanChar() {
  char inChar = 0;
  // if(PlaybackIndex >= 0) {
  //   if(PlaybackStep < (int)Playback[PlaybackIndex].length()) {
  //     inChar = Playback[PlaybackIndex].charAt(PlaybackStep++);
  //   } else {
  //     PlaybackIndex = -1;
  //     PlaybackStep = 0;
  //   }
  // } else 
  if (Serial.available() > 0) {
    inChar = Serial.read();
  }
  return inChar;
}

// Convert degrees to distance in mm for differential drive
// how far do treads have to move in opposite directions to turn the robot N degress
float degToDiffDistance(float deg) {
  return deg * 1.7777778; // measured
}
// Eat all characters from the serial buffer and return them as a single number
// Ignore all non-numeric characters
float getParam() {
  float param = 0;
  String inString = "";
  char inChar;

  while(((inChar = scanChar()) >= '0' && inChar <= '9') || inChar == '.' || inChar == '-') {
    inString.concat(inChar);
  }
  param = inString.toFloat();

  // Serial.print("Param: ");
  // Serial.print(param);
  // Serial.print("\n");
  return param;
}

//
// Basic control functions
//
void stepIfTime() {
  // unsigned long curMicros = micros(); // time to run the motors
  // if (RexecuteMove || LexecuteMove) {
  //   Serial.print("StepIfTime\n");
  //   Serial.print("RexecuteMove: ");
  //   Serial.print(RexecuteMove);
  //   Serial.print(" - LexecuteMove: ");
  //   Serial.print(LexecuteMove);
  //   Serial.print("\n");
  //   Serial.print("RStepDelayMs: ");
  //   Serial.print(RStepDelayMs);
  //   Serial.print(" - LStepDelayMs: ");
  //   Serial.print(LStepDelayMs);
  //   Serial.print("\n");
  //   Serial.print("RSteps: ");
  //   Serial.print(RSteps);
  //   Serial.print(" - LSteps: ");
  //   Serial.print(LSteps);
  //   Serial.print("\n");
  // }
  if (RexecuteMove && (RStepDelayMs > 0) && (RSteps > 0)) { // If speed is 0, or steps are 0, No stepping
    if ((RStepDelayMs != ULONG_MAX) && ((micros() - RLastStepTime) >= RStepDelayMs)) {
      stepMotors(RIGHT_MOTOR);
      RLastStepTime = micros();
      // Steps--;
      // if(Steps == 0) {
      //   executeMove = false;
      //   Serial.print("Move complete\n");
      // }
    }
  }
  if (LexecuteMove && LStepDelayMs > 0 && LSteps > 0) { // If speed is 0, or steps are 0, No stepping
    // Deal with wraparound of micros()
    if ((LStepDelayMs != ULONG_MAX) && ((micros() - LLastStepTime) >= LStepDelayMs)) {
      stepMotors(LEFT_MOTOR);
      LLastStepTime = micros();
      // Steps--;
      // if(Steps == 0) {
      //   executeMove = false;
      //   Serial.print("Move complete\n");
      // }
    }
  }
  // if (executeMove && StepDelayMs > 0 && Steps > 0) { // If speed is 0, or steps are 0, No stepping
  //   // Deal with wraparound of micros()
  //   if (micros() - LastStepTime >= StepDelayMs) {
  //     stepMotors(0);
  //     LastStepTime = micros();
  //     // Steps--;
  //     // if(Steps == 0) {
  //     //   executeMove = false;
  //     //   Serial.print("Move complete\n");
  //     // }
  //   }
  // }
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
//
  // 200 steps per revolution 
  // 38mm per rev
  // 26.3158 revs in a meter
  // 5,263.16 steps per meter
  // 1/5,263.16 =  0.00019 seconds per step = 1 meter per sec
  // 19us delay per step = 1 meter per sec
  // Delay = 0.00019 / meters per sec
unsigned long MetersaSecToMicroSecDelay(float MpS) {
  float delayCnt = 0;

  if (MpS == 0) {
    delayCnt = ULONG_MAX;  // minimum speed 1 second per step (Gives us a chance to see the motor is on, incase it's not supposed to be)
  } else {
    MpS = abs(MpS); // handle negative speeds (we handle direction in the motor control)
    delayCnt = 0.00019/MpS;  // Delay in seconds for speed requested (in meters per second)
  }

  return (unsigned long)delayCnt;
}

unsigned long mmASecToMsDelay(float mm) { 
  float delayCnt = 0;
  
  if (mm == 0) {
    delayCnt = ULONG_MAX;  // minimum speed 1 second per step (Gives us a chance to see the motor is on, incase it's not supposed to be)
  } else {
    mm = abs(mm); // handle negative speeds
    delayCnt = 1000000.0/(mm/0.04);  // Delay in microseconds
    // delayCnt = 0.00019/m;  // Delay in seconds for speed requested (in meters per second)
    delayCnt += 0.5; // round up when we cast to unisgned long
  }
  // Serial.print("Delay: (");
  // Serial.print(delayCnt);
  // Serial.print(")\n");
  return (unsigned long)delayCnt;  // round up
}

void stepMotors(int motor) {
  //if((motor | RIGHT_MOTOR) == RIGHT_MOTOR) {
    digitalWrite(RIGHT_STEP_PIN, HIGH);
    digitalWrite(RIGHT_STEP_PIN, LOW);
  //}
  //if((motor | LEFT_MOTOR) == LEFT_MOTOR) {
    digitalWrite(LEFT_STEP_PIN, HIGH);
    digitalWrite(LEFT_STEP_PIN, LOW);
  //}
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
}

// pulse pins high/low with delay between pulses to step motors
void manualRun(unsigned int step_cnt, unsigned int delay) {
  for(unsigned int Li=0; Li < step_cnt; Li++) {
    stepMotors(RIGHT_MOTOR + LEFT_MOTOR);
    delayMicroseconds(delay);
  }
}
