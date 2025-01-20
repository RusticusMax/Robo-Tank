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

#define WHEEL_DIST 0.25 // This is a guess (need it in meters) // actual distance 222.3 // in inches 8.75"

float LSpeed = 0.0, RSpeed = 0.0; // Speed in Meters a second
bool RspeedNonZero = false, LspeedNonZero = false;
unsigned long RStepDelayMicroS = 0, LStepDelayMicroS = 0; // Delay in microsec between steps. Derived from Speed
unsigned long RLastStepTime = 0, LLastStepTime = 0; // time of last step.  Set as counter for next step
unsigned long RStepCnt = 0, LStepCnt = 0; // steps to run for testing step distance.

// Twist message Tlx,ly,lz,ax,ay,az
struct Twist {
  float lx;
  float ly;
  float lz;
  float ax;
  float ay;
  float az;
};

struct xspeed {
	float lspeed;		// Left tread speed
	float rspeed;		// Right tread speed
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
struct xspeed processTwist(struct Twist twist);

// Main Code
void setup() {
	pinMode(LED_BUILTIN, OUTPUT);     // Turn LED as sign of life
	Serial.begin(115200);             // Set comm baudrate @ 115200
	
	initMotors();                     // Initialize motors
}

void loop() {
  char inChar = 0;
  struct Twist twist = {0,0,0,0,0,0};
	struct xspeed tread_speed;

  if((inChar = scanChar()) != 0) { // If there is a character in the buffer, read it and act on it
    // Serial.print("Received: (");
    // Serial.print(inChar, HEX);
    // Serial.print(")\n");
    delay(2); // wait for the rest of the message to arrive (10 char@115200 = 0.87ms?)
    switch (inChar) { 
      case 'h':	// Halt
        stopAllMotors();
        break;
      case 'd':	// disable motors
        Serial.print("DisableMotors\n");
        digitalWrite(LEFT_ENABLE_PIN, STEP_DISABLE);
        digitalWrite(RIGHT_ENABLE_PIN, STEP_DISABLE);
        break;
      case 'e':	// enable motors
        digitalWrite(LEFT_ENABLE_PIN, STEP_ENABLE);
        digitalWrite(RIGHT_ENABLE_PIN, STEP_ENABLE);
        break;
      case 'T': // Twist message Tlx,ly,lz,ax,ay,az
        // Serial.print("Twist\n");
        twist.lx = getParam();
        twist.ly = getParam();
        twist.lz = getParam();
        twist.ax = getParam();
        twist.ay = getParam();
        twist.az = getParam();
        tread_speed = processTwist(twist);  // Set global RSpeed and LSpeed for motor control
				setSpeed(tread_speed.lspeed, tread_speed.rspeed);
        break;
			case 'D':	// Set step delay directly (when we let the sender do the calculations)
				LStepDelayMicroS = getParam();
				RStepDelayMicroS = getParam();
				LSpeed = 0;	// indicates that we set delay directly (not used at present, so that's fine)
				RSpeed = 0;
				break;
			case 'C':	// Set step counts (for testing of step length)
				LStepCnt = getParam();
				RStepCnt = getParam();
				LStepDelayMicroS = getParam();
				RStepDelayMicroS = getParam();
				LSpeed = 0;	// indicates that we set delay directly (not used at present, so that's fine)
				RSpeed = 0;
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

// Turn a twist message into motor speeds in meters per second
struct xspeed processTwist(struct Twist twist)  {
  float rsp, lsp;
	struct xspeed tread_speed;

  tread_speed.lspeed = (twist.az * WHEEL_DIST) /2 + twist.lx;
  tread_speed.rspeed = twist.lx * 2 - rsp;
	return tread_speed;
}

//Set speed and calculate step delay
void setSpeed(float lspeed, float rspeed) {
	LSpeed = lspeed;
	RSpeed = rspeed;
	LStepDelayMicroS = MetersaSecToMicroSecDelay(LSpeed);
	RStepDelayMicroS = MetersaSecToMicroSecDelay(RSpeed);
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

// Scan input for char nonblocking (returns 0 if no char) 
char scanChar() {
  char inChar = 0;
  if (Serial.available() > 0) {
    inChar = Serial.read();
  }
  return inChar;
}

// Eat all characters from the serial buffer and return them as a single number
// all non-numeric (excluding '.', and '-',) characters are terminators
float getParam() {
  float param = 0;
  String inString = "";
  char inChar;

  while(((inChar = scanChar()) >= '0' && inChar <= '9') || inChar == '.' || inChar == '-') {
    inString.concat(inChar);
  }
  param = inString.toFloat();
  return param;
}

//
// Basic control functions
//
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
// How many microseconds to delay between steps to get the speed requested
unsigned long MetersaSecToMicroSecDelay(float MeterspS) {
  float delayCnt = 0;

  if (MeterspS == 0) {
    delayCnt = 0;  // minimum speed 1 second per step (Gives us a chance to see the motor is on, incase it's not supposed to be)
  } else {
    MeterspS = abs(MeterspS); // handle negative speeds (we handle direction in the motor control)
    delayCnt = 0.00019/MeterspS;  // Delay in seconds for speed requested (in meters per second)
  }

  return (unsigned long)delayCnt;
}

// Check globals LastStepTime and StepDelayMicroS to see if we need to step
// if currentTime is > LastStepTime + StepDelayMicroS then step (but handle micros() wraparound)
void stepIfTime() {
	unsigned long currentMicros = micros();

  if (RStepDelayMicroS > 0) { // We have a step delay see of we passed it
    if ((currentMicros - RLastStepTime) >= RStepDelayMicroS) {
      stepMotors(RIGHT_MOTOR);	// Take a Step...
      RLastStepTime = currentMicros;	// ...and reset the time marker
			// step count used for calibration (how long is a step?)  This code can be remove for operation
			if(RStepCnt > 0) {
				RStepCnt--;
				if(RStepCnt == 0) {
					stopRMotor();
				}
			}
    }
  }
  if (LStepDelayMicroS > 0) { // If speed is 0, or steps are 0, No stepping
    // Deal with wraparound of micros()
    if ((currentMicros - LLastStepTime) >= LStepDelayMicroS)	{
      stepMotors(LEFT_MOTOR);
      LLastStepTime = currentMicros;
			// step count used for calibration (how long is a step?)  This code can be remove for operation
			if(LStepCnt > 0) {
				LStepCnt--;
				if(LStepCnt == 0) {
					stopLMotor();
				}
			}
    }
  }
}

void stepMotors(int motor) {
  //if((motor & RIGHT_MOTOR) == RIGHT_MOTOR) {
    digitalWrite(RIGHT_STEP_PIN, HIGH);
    digitalWrite(RIGHT_STEP_PIN, LOW);
  //}
  //if((motor & LEFT_MOTOR) == LEFT_MOTOR) {
    digitalWrite(LEFT_STEP_PIN, HIGH);
    digitalWrite(LEFT_STEP_PIN, LOW);
  //}
}

// Stop all motors and set speed/delay to zero
void stopAllMotors() {
	// Set delay and speed to 0
	stopRMotor();
	stopLMotor();
}

void stopRMotor() {
	RSpeed = 0;
	RStepDelayMicroS = 0;
	digitalWrite(RIGHT_STEP_PIN, LOW);
}

void stopLMotor() {
	LSpeed = 0;
	LStepDelayMicroS = 0;
	digitalWrite(LEFT_STEP_PIN, LOW);
}

//
// Initialize motors
//
void initMotors()  {
  // Disable the motors first
  pinMode(LEFT_ENABLE_PIN, OUTPUT);
	pinMode(RIGHT_ENABLE_PIN, OUTPUT);
	disableMotors();

	// Setup and init left stepper
	pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  digitalWrite(LEFT_STEP_PIN, LOW);

	// Setup and init right stepper
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
	digitalWrite(RIGHT_STEP_PIN, LOW);

	enableMotors();
}

void enableMotors() {
  // Enable the motors
  digitalWrite(LEFT_ENABLE_PIN, STEP_ENABLE);
  digitalWrite(RIGHT_ENABLE_PIN, STEP_ENABLE);
}

void disableMotors() {
  // Enable the motors
  digitalWrite(LEFT_ENABLE_PIN, STEP_DISABLE);
  digitalWrite(RIGHT_ENABLE_PIN, STEP_DISABLE);
}
