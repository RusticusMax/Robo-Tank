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
#define LeftFWD HIGH
#define LeftREV LOW
#define RightFWD LOW
#define RightREV HIGH
#define STEP_ENABLE LOW
#define STEP_DISABLE HIGH

#define RIGHT_MOTOR 1
#define LEFT_MOTOR 2

#define WHEEL_DIST 0.222 // This is  in meters // actual distance 222.3 // in inches 8.75"

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
unsigned long MetersaSecToMicroSecDelay(float MeterspS);
void stepIfTime();
void stepMotors(int motor);
float getParam();
char scanChar();
void initMotors();
void enableMotors();
void disableMotors();
void setSpeed(float lspeed, float rspeed);
void stopAllMotors();
void stopRMotor();
void stopLMotor();
struct xspeed processTwist(struct Twist twist);
bool interactiveMode = false;

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
      case 'v':	// Version
        Serial.print("Version 1.5\n");
        break;
      case 'i': // Interactive mode
        interactiveMode = !interactiveMode;
        Serial.print("Interactive mode: ");
        Serial.print((interactiveMode ? "On" : "Off"));
        Serial.print("\n");
        break;
      case 'h':	// Halt
        if(interactiveMode) Serial.print("Halt all Motors\n");
        stopAllMotors();
        break;
      case 'd':	// disable motors
        if(interactiveMode) Serial.print("DisableMotors\n");
        digitalWrite(LEFT_ENABLE_PIN, STEP_DISABLE);
        digitalWrite(RIGHT_ENABLE_PIN, STEP_DISABLE);
        break;
      case 'e':	// enable motors
        if(interactiveMode) Serial.print("EnableMotors\n");
        digitalWrite(LEFT_ENABLE_PIN, STEP_ENABLE);
        digitalWrite(RIGHT_ENABLE_PIN, STEP_ENABLE);
        break;
      case 'T': // Twist message Tlx,ly,lz,ax,ay,az
        if(interactiveMode) Serial.print("Twist\n");
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
        if(interactiveMode) Serial.print("Set motordelay in mirosecs\n");
				LStepDelayMicroS = getParam();
				RStepDelayMicroS = getParam();
				LSpeed = 0;	// indicates that we set delay directly (not used at present, so that's fine)
				RSpeed = 0;
        if(interactiveMode) {
          Serial.print("params: (");
          Serial.print(LStepDelayMicroS, DEC);
          Serial.print(")  (");
          Serial.print(RStepDelayMicroS, DEC);
          Serial.print(")\n");
        }
        RLastStepTime = micros();
        LLastStepTime = RLastStepTime;
				break;
			case 'C':	// Set step counts (for testing of step length)
        if(interactiveMode) Serial.print("Set step counts\n");
				LStepCnt = getParam();
				RStepCnt = getParam();
				LStepDelayMicroS = getParam();
				RStepDelayMicroS = getParam();
        setDirection(RIGHT_MOTOR, (RStepCnt < 0) : RightREV ? RightFWD); // Set direction for R motor
        setDirection(LEFT_MOTOR, (LStepCnt < 0) : LeftREV ? LeftFWD); // Set direction for L motor
        RLastStepTime = micros(); // Slop TODO: clear up motor initization is done by seting speed or step count
        LLastStepTime = RLastStepTime;
				LSpeed = 0;	// indicates that we set delay directly (not used at present, so that's fine)
				RSpeed = 0;
				break;
      default:
        if(interactiveMode) Serial.print("Invalid command: (");
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
	struct xspeed tread_speed;

  tread_speed.rspeed = (twist.az * WHEEL_DIST) /2 + twist.lx;
  tread_speed.lspeed = twist.lx * 2 - tread_speed.rspeed;
	return tread_speed;
}

//Set speed and calculate step delay
void setSpeed(float lspeed, float rspeed) {
	LSpeed = lspeed;
	RSpeed = rspeed;
	LStepDelayMicroS = MetersaSecToMicroSecDelay(LSpeed);
	RStepDelayMicroS = MetersaSecToMicroSecDelay(RSpeed);
  Serial.print("setSpeed: ");
  Serial.print(LSpeed, 3);
  Serial.print(", ");
  Serial.print(RSpeed, 3);
  Serial.print(", ");
  Serial.print(LStepDelayMicroS, DEC);
  Serial.print(", ");
  Serial.print(RStepDelayMicroS, DEC);
  Serial.print("\n");
  // Set direction for both motors
  setDirection(RIGHT_MOTOR, (RSpeed < 0) : RightREV ? RightFWD); // Set direction for both motors
  setDirection(LEFT_MOTOR, (LSpeed < 0) : LeftREV ? LeftFWD); // Set direction for both motors
	// if(RSpeed < 0) {
  //   digitalWrite(RIGHT_DIR_PIN, RightREV);
  // } else {
  //   digitalWrite(RIGHT_DIR_PIN, RightFWD);
  // }
  // if(LSpeed < 0) {
  //   digitalWrite(LEFT_DIR_PIN, LeftREV);
  // } else {
  //   digitalWrite(LEFT_DIR_PIN, LeftFWD);
  // }
}

// Set Direction of the motors
void setDirection(int motor, int dir) {
  if(motor & RIGHT_MOTOR) {
    digitalWrite(RIGHT_DIR_PIN, dir);
  }
  if(motor & LEFT_MOTOR) {
    digitalWrite(LEFT_DIR_PIN, dir);
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
    // WRONG
    // 200 steps per revolution 
    // 38mm per rev
    // 26.3158 revs in a meter
    // 5,263.16 steps per meter
    // 1/5,263.16 =  0.00019 seconds per step = 1 meter per sec
    // 19us delay per step = 1 meter per sec
    // Delay = 0.00019 / meters per sec\
    //
    //ALmost right, but not quite
    // 200 steps per revolution
    // 135mm per rev
    // 1481.48 steps in a meter
    // 675 us per step = 1 meter per sec
    // Delay = 675 / meters per sec = us delay
// 2000 step test = 52.75inches = 1339.85mm
// 200 steps per revolution
// 1339.85/10(revs) = 133.985mm per rev
// 1339.85mm / 2000 steps = 0.669925mm per step
// 1000mm / 0.669925mm = 1492.7 steps per meter
// 1492.7 steps per meter = 0.000669925 seconds per step = 1 meter per sec
// 669.93 us per step = 1 meter per sec
// Delay = 669.93 / meters per sec = us delay 
// How many microseconds to delay between steps to get the speed requested
unsigned long MetersaSecToMicroSecDelay(float MeterspS) {
  float delayCnt = 0;

  if (MeterspS == 0) {
    delayCnt = 0;  // minimum speed 1 second per step (Gives us a chance to see the motor is on, incase it's not supposed to be)
  } else {
    MeterspS = abs(MeterspS); // handle negative speeds (we handle direction in the motor control)
    delayCnt = 669.93/MeterspS;  // Delay in seconds for speed requested (in meters per second)
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
			if((RSpeed == 0) && (RStepCnt > 0)) {
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
			if((LSpeed == 0) && (LStepCnt > 0)) {
				LStepCnt--;
				if(LStepCnt == 0) {
					stopLMotor();
				}
			}
    }
  }
}

void stepMotors(int motor) {
  if(motor & RIGHT_MOTOR) {
    digitalWrite(RIGHT_STEP_PIN, HIGH);
    digitalWrite(RIGHT_STEP_PIN, LOW);
  }
  if(motor & LEFT_MOTOR) {
    digitalWrite(LEFT_STEP_PIN, HIGH);
    digitalWrite(LEFT_STEP_PIN, LOW);
  }
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
