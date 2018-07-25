#include <ParaSerialWrite.h> // ParaSerialWrite library for parallel UART transmission

#define _DEBUG 1
#define _SERIAL 9600 // bandwidth or 0 means turned off

const int debugPin4            = 13; // debug during development
const int debugPin3            = 12; // debug during development
const int debugPin2            = 11; // debug during development
const int debugPin1            = 10; // debug during development
const int inputSelector        =  7; // hijack gyro sensor communication
const int hallRightBluePin     =  5; // hall effect direction check
const int hallLeftBluePin      =  4; // hall effect direction check
const int hallRightYellowPin   =  3; // hall effect triggering
const int hallLeftYellowPin    =  2; // hall effect triggering

signed int leftAcceleration = 111; // left wheel speed
signed int rightAcceleration = 111; // right wheel speed
bool leftClockwise = false;
bool rightClockwise = false;


unsigned long lastLeftIncrCalculated = 0;
unsigned int leftIncrements = 0;
unsigned long leftCyclePerSec = 0;
unsigned long lastRightIncrCalculated = 0;
unsigned int rightIncrements = 0;
unsigned long rightCyclePerSec = 0;

char shifter = 'D'; // [P]ark or [D]rive

ParaSerialWrite mySerial(2); // register 2 lower ports of PORTB for parallel UART transmission
const int framesInMessage = 10; // vary by models. how many frames transfer before ~500us pause

enum controlOptions { bySerial, byI2C }; // control by serial input or pull up buttons
int control = bySerial; // control set to button
char c = 'x'; //last received input by serial. x = neutral

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
void debugPulse(int pin, uint16_t count)
{
  bool pinStatus = digitalRead(pin);
  while (count--)
  {
    digitalWrite(pin, 1 ^ pinStatus);
    digitalWrite(pin, pinStatus);
  }
}

/***
 * Transmits imitated UART signal, just like gyro sensor.
 * Uses ParaSerialWrite library what can do multiple UART transmission with
 * same frame size and baud-rate
 */
void writeCurrentSpeed() {
  if (digitalRead(inputSelector) == HIGH) {
    mySerial.write(~0, ~0);
  } else {
    //0xC00 2 stop bits and <<1 for the start
    const uint16_t startSignal = 0xFC00 | (256 << 1);
    const uint16_t driveEndSignal = 0xFC00 | (85 << 1);
    const uint16_t parkEndSignal = 0xFC00 | (170 << 1);

    static unsigned int currentFrame = 1;
    static uint16_t endSignal = parkEndSignal;
    static uint16_t leftAccelerationLowFrame;
    static uint16_t leftAccelerationHighFrame;
    static uint16_t rightAccelerationLowFrame;
    static uint16_t rightAccelerationHighFrame;

    switch (currentFrame) {
      case 1:
        mySerial.write(~startSignal, ~startSignal);
        leftAccelerationLowFrame = 0xFC00 | ((leftAcceleration & 0xFF) << 1);
        leftAccelerationHighFrame = 0xFC00 | (((leftAcceleration >> 8) & 0xFF) << 1);
        rightAccelerationLowFrame = 0xFC00 | ((-rightAcceleration & 0xFF) << 1);
        rightAccelerationHighFrame = 0xFC00 | (((-rightAcceleration >> 8) & 0xFF) << 1);
        endSignal = (shifter == 'D') ? driveEndSignal : parkEndSignal ;
        currentFrame++;
        break;
      case 2:
      case 4:
        mySerial.write(~leftAccelerationLowFrame, ~rightAccelerationLowFrame);
        currentFrame++;
        break;
      case 3:
      case 5:
        mySerial.write(~leftAccelerationHighFrame, ~rightAccelerationHighFrame);
        currentFrame++;
        break;
      case 6:
        mySerial.write(~endSignal, ~endSignal);
        currentFrame = 1;
        break;
    }
  }
}

/***
 * Reads control from serial port.
 * Development and debugging purpose only.
 */
void listenSerialControl() {
  if (Serial.available() > 0) {
    c=Serial.read();
    if(c == ' ') {
      leftAcceleration=0;
      rightAcceleration=0;
    } else if(c == 'q') {
      leftAcceleration += 50;
    } else if(c == 'a') {
      leftAcceleration -= 50;
    } else if(c == 'w') {
      rightAcceleration += 50;
    }  else if(c == 's') {
      rightAcceleration -= 50;
    }
  }
}

/***
 * Selects control.
 */
void listenControl() {
  if(control == bySerial){
    listenSerialControl();
  } else if (control == byI2C) {
    // tbd
  }
}

/***
  * Calculates last 4 averages and sets to leftCyclePerSec and rightCyclePerSec
  */
void calculateIncInPeriod() {
  static unsigned long timestamp = 0;
  static unsigned int chpsL1 = 0, chpsL2 = 0, chpsL3 = 0, chpsL4 = 0;
  static unsigned int chpsR1 = 0, chpsR2 = 0, chpsR3 = 0, chpsR4 = 0;
  unsigned long timeElipsed = 0;
  if (timestamp) {
    chpsL4 = chpsL3;
    chpsL3 = chpsL2;
    chpsL2 = chpsL1;
    timeElipsed = millis() - timestamp;
    chpsL1 =  (1000 * (long)leftIncrements) / (long)timeElipsed;
    leftCyclePerSec = (chpsL1 + chpsL2 + chpsL3 + chpsL4) / 4;
    chpsR4 = chpsR3;
    chpsR3 = chpsR2;
    chpsR2 = chpsR1;
    timeElipsed = millis() - timestamp;
    chpsR1 =  (1000 * (long)rightIncrements) / (long)timeElipsed;
    rightCyclePerSec = (chpsR1 + chpsR2 + chpsR3 + chpsR4) / 4;
  }
  leftIncrements = 0;
  rightIncrements = 0;
  timestamp = millis();
}

/***
  * Increments changes on left hall effect sensor and detects direction.
  */
void hallLeftEnc() {
  static unsigned long timestamp = 0;
  debugPulse(debugPin1, 1);
  bool hallLeftYellowState = digitalRead(hallLeftYellowPin);
  if (!hallLeftYellowState && (timestamp+500) < micros()) {
    if (leftCyclePerSec < 15) //only detect direction under one rotation/s
      leftClockwise = digitalRead(hallLeftBluePin);
    leftIncrements++;
    timestamp = micros();
  }
}

/***
  * Increments changes on right hall effect sensor and detects direction.
  */
void hallRightEnc() {
  static unsigned long timestamp = 0;
  debugPulse(debugPin1, 1);
  bool hallRightYellowState = digitalRead(hallRightYellowPin);
  if (!hallRightYellowState && (timestamp+500) < micros()) {
    if (rightCyclePerSec < 15) //only detect direction under one rotation/s
      rightClockwise = digitalRead(hallRightBluePin);
    rightIncrements++;
    timestamp = micros();
  }
}

void debugInfo() {
  Serial.print("LCPS: ");
  Serial.print(leftCyclePerSec);
  Serial.print("; RCPS: ");
  Serial.println(rightCyclePerSec);
  //Serial.println(millis());
}

void setup() {

  pinMode(debugPin1, OUTPUT);
  pinMode(debugPin2, OUTPUT);
  pinMode(debugPin3, OUTPUT);
  pinMode(debugPin4, OUTPUT);
  pinMode(hallLeftYellowPin, INPUT);
  pinMode(hallLeftBluePin, INPUT);
  pinMode(hallRightYellowPin, INPUT);
  pinMode(hallRightBluePin, INPUT);
  pinMode(inputSelector, OUTPUT);
  digitalWrite(inputSelector, HIGH); // by start no hijack

  attachInterrupt(digitalPinToInterrupt(hallLeftYellowPin),hallLeftEnc,FALLING);
  attachInterrupt(digitalPinToInterrupt(hallRightYellowPin),hallRightEnc,FALLING);

  mySerial.begin(26275, 11); // hover board UART baud-rate and frame size
  if (_SERIAL) {
    Serial.begin(_SERIAL); // opens serial port, sets data rate to 9600 bps
    Serial.println("CarOne");
  }

  // TODO: need to be controlled from outside
  // digitalWrite(inputSelector, LOW);
}

void loop() {
  static unsigned long pmTransmisson =  0;
  static unsigned long pmCalculateIncInPeriod =  0;
  static unsigned long pmListenControl =  0;
  static unsigned long pmDebugInfo =  0;
  static unsigned int transmissionPeriod = 450;
  static unsigned int transmissionCounter = 0;
  unsigned long currentMicros = micros();

  // Short 450 us frame cycles (time between 2 start bit) happen 10/9 times
  // Long 875 us cycles (time between 2 start bit) happen 10/1 times
  // transmissions, just like gyro sensor does
  if (currentMicros - pmTransmisson >= transmissionPeriod) {
    pmTransmisson = currentMicros;
    writeCurrentSpeed();
    transmissionPeriod = (++transmissionCounter % framesInMessage == 0)
      ? 875 : 450;
    currentMicros = micros();
  }

  // calculate increments in every 1/4 sec
  if (currentMicros - pmCalculateIncInPeriod >= 250000) {
    pmCalculateIncInPeriod = currentMicros;
    calculateIncInPeriod();
    currentMicros = micros();
  }

  // delay 50 milliseconds between control samples
  if (currentMicros - pmListenControl >= 50000) {
    pmListenControl = currentMicros;
    listenControl();
    currentMicros = micros();
  }

  // delay 1s between debug info dump
  if (_SERIAL && _DEBUG) {
    if (currentMicros - pmDebugInfo >= 2000000) {
      pmDebugInfo = currentMicros;
      debugInfo();
      currentMicros = micros();
    }
  }
}

