#include <Arduino.h>

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

#define MANUAL_LOOP_CNT 7000
#define MANUAL_LOOP_DELAY 200

float Speed = 50.0; // Speed in mm a second
unsigned long StepDelayMs = 0; // Delay between steps. Derived from Speed
unsigned long LastStepTime = 0; // time of last step.  Set as counter for next step
float DistanceMM = 10; // distance to run
unsigned long Steps = 0; // steps to run. Derived from DistanceMM
String Playback[] = {
  "s50 m80 r", 
  "s1000 m500 b",
  };
int PlaybackIndex = -1;   // The predefined string to playback.  -1 is not playing back
int PlaybackStep = 0;     // The current step in the playback string
float PivotAngle = 0; // Angle to pivot with differential drive

// unsigned int i = 0;
bool executeMove = false; // Flag to process the timeToStep function to run the motors

// put function declarations here:
void manualRun(unsigned int step_cnt, unsigned int delay);
void manualSetup();
void manualTestDrive();

unsigned long mmToSteps(float mm);
unsigned long mmASecToMsDelay(float mm);
int setDistanceMm(float mm);
void stepIfTime(unsigned long speed);
void stepMotors();
float getParam();
char scanChar();
float degToDiffDistance(float deg);
void prepToMove();
void stopAllMotors();

// Main Code
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.print("Starting up\n");

  manualSetup();
  Serial.print("Seral mode\n");
  Serial.print("Enter command:\n");
  Serial.print("f = Set Forward\n");
  Serial.print("b = Set Backward\n");
  Serial.print("r = Set Right\n");
  Serial.print("l = Set Left\n");
  Serial.print("px.x = Set pivot in degrees\n");
  Serial.print("g = Go based in set speed/distance/direction\n");
  Serial.print("e = Engage Motors\n");
  Serial.print("d = Disengage Motors\n");
  Serial.print("sx.x = Set Speed in mm/sec\n");
  Serial.print("mx.x = Set Distance in mm\n");
  Serial.print("axx = Set Absolute Steps directly\n");
  Serial.print("Px = Playback preset command sequence\n");
}

void loop() {
  char inChar = 0;

  if((inChar = scanChar()) != 0) { // If there is a character in the buffer, read it and act on it
    Serial.print("Received: (");
    Serial.print(inChar, HEX);
    Serial.print(")\n");
    switch (inChar) {  //fbrldesmaPp  abdeflmprsP
      case 'f':
        Serial.print("Forward\n");
        digitalWrite(LEFT_DIR_PIN, LeftFWD);
        digitalWrite(RIGHT_DIR_PIN, RightFWD);
        break;
      case 'b':
        Serial.print("Backward\n");
        digitalWrite(LEFT_DIR_PIN, LeftREV);
        digitalWrite(RIGHT_DIR_PIN, RightREV);
        break;
      case 'r':
        Serial.print("Right\n");
        digitalWrite(LEFT_DIR_PIN, LeftFWD);
        digitalWrite(RIGHT_DIR_PIN, RightREV);
        break;
      case 'l':
        Serial.print("Left\n");
        digitalWrite(LEFT_DIR_PIN, LeftREV);
        digitalWrite(RIGHT_DIR_PIN, RightFWD);
        break;
      case 'p': // Pivot right by n degrees
        PivotAngle = getParam();
        DistanceMM = degToDiffDistance(PivotAngle);
        Steps = setDistanceMm(DistanceMM);
        digitalWrite(LEFT_DIR_PIN, LeftFWD);
        digitalWrite(RIGHT_DIR_PIN, RightREV);
        Serial.print("Pivot Set: (");
        Serial.print(PivotAngle);
        Serial.print(")\n");
        break;
      case 'g':
        Serial.print("Go\n");
        prepToMove();
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
        Speed = getParam();
        StepDelayMs = mmASecToMsDelay(Speed);
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
      case 'P':
        //if((inChar = scanChar()) != 0) { // see which playback to run
        PlaybackIndex = (int)getParam();
        Serial.print("Received: (");
        Serial.print(inChar, HEX);
        Serial.print(")\n");
        if(PlaybackIndex >= 0 && PlaybackIndex < (int)sizeof(Playback)) {
          Serial.print("Playback: (");
          Serial.print(Playback[PlaybackIndex]);
          Serial.print(")\n");
          PlaybackStep = 0;
        } else {
          Serial.print("Invalid playback\n");
          PlaybackIndex = -1;
        }
        //}
        break;
      case 'w':
        Serial.print("Wait\n");
        delay(getParam());
        break;
      default:
        Serial.print("Invalid command\n");
        break;
    }
  }
  stepIfTime(Speed);
}

void prepToMove() {
  Steps = setDistanceMm(DistanceMM);
  StepDelayMs = mmASecToMsDelay(Speed);
  executeMove = true;
}

void stopAllMotors() {
  // Steps = 0;
  // StepDelayMs = 0;
  executeMove = false;
}

// Scan input for char nonblocking 
// Also allows for playback of recorded commands
char scanChar() {
  char inChar = 0;
  if(PlaybackIndex >= 0) {
    if(PlaybackStep < (int)Playback[PlaybackIndex].length()) {
      inChar = Playback[PlaybackIndex].charAt(PlaybackStep++);
    } else {
      PlaybackIndex = -1;
      PlaybackStep = 0;
    }
  } else if (Serial.available() > 0) {
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

  while(((inChar = scanChar()) >= '0' && inChar <= '9') || inChar == '.') {
    inString.concat(inChar);
  }
  param = inString.toFloat();

  Serial.print("Param: ");
  Serial.print(param);
  Serial.print("\n");
  return param;
}

//
// Basic control functions
//
void stepIfTime(unsigned long speed) {
  // unsigned long curMicros = micros(); // time to run the motors

  if (executeMove && StepDelayMs > 0 && Steps > 0) { // If speed is 0, or steps are 0, No stepping
    // Deal with wraparound of micros()
    if (micros() - LastStepTime >= StepDelayMs) {
      stepMotors();
      LastStepTime = micros();
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
unsigned long mmASecToMsDelay(float mm) { 
  float tmp;
  tmp = 1000000.0/(mm/0.04);  // Delay in microseconds
  Serial.print("Delay: (");
  Serial.print(tmp);
  Serial.print(")\n");
  return (unsigned long)(tmp + 0.5);  // round up
}

void stepMotors(){
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
}

// pulse pins high/low with delay between pulses to step motors
void manualRun(unsigned int step_cnt, unsigned int delay) {
  for(unsigned int Li=0; Li < step_cnt; Li++) {
    stepMotors();
    delayMicroseconds(delay);
  }
}
