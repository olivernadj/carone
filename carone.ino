#include <ParaSerialWrite.h> // ParaSerialWrite library for parallel UART transmission
#include <pt.h>              // protothread library

#define _DEBUG 1
#define _SERIAL 9600 // bandwidth or 0 means turned off

const int debugPin            = 6;
const int inputSelector       = 7; // hijack gyro sensor communication
const int hallLeftYellowPin   = 2;
const int hallLeftBluePin     = 4;
const int hallRightYellowPin   = 3;
const int hallRightBluePin     = 5;

ParaSerialWrite mySerial(2); // register 2 lower ports of PORTB for parallel UART transmission
signed int leftSpeed = 0; // left wheel speed
signed int rightSpeed = 0; // right wheel speed
bool leftClockwise = false;
bool rightClockwise = false;

unsigned long lastLeftIncrCalculated = 0;
unsigned int leftIncrements = 0;
unsigned long leftCyclePerSec = 0;


const int maxSpeed = 350; // absolute maximum speed
const int maxManeuveringSpeed = 150; // maximum maneuvering speed
const int acceleration = 100; // x unit acceleration / second^2
const int framesInMessage = 10; // vary by models. how many frames transfer before ~500us pause

enum controlOptions { bySerial, byI2C }; // control by serial input or pull up buttons
int control = bySerial; // control set to button
char c = 'x'; //last received input by serial. x = neutral

static struct pt ptTx, ptLC, pIncr, ptDebug; // each protothread needs one of these


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
  if (digitalRead(inputSelector) == LOW) {
    mySerial.write(0, 0);
  } else {
    static unsigned int currentFrame = 1;
    //0xC00 2 stop bits and <<1 for the start
    uint16_t startSignal = 0xFC00 | (256 << 1);
    uint16_t endSignal = 0xFC00 | (85 << 1);
    if (leftSpeed == 0 && rightSpeed == 0) {
      endSignal = 0xFC00 | (170 << 1);
    }

    for(int i=0; i<framesInMessage; i++){
      switch (currentFrame) {
        case 1:
          mySerial.write(startSignal, startSignal);
          break;
        case 2:
          mySerial.write(
            0xFC00 | ((leftSpeed & 0xFF) << 1),
            0xFC00 | ((-rightSpeed & 0xFF) << 1)
          );
          break;
        case 3:
          mySerial.write(
            0xFC00 | (((leftSpeed >> 8) & 0xFF) << 1),
            0xFC00 | (((-rightSpeed >> 8) & 0xFF) << 1)
          );
          break;
        case 4:
          mySerial.write(
            0xFC00 | ((leftSpeed & 0xFF) << 1),
            0xFC00 | ((-rightSpeed & 0xFF) << 1)
          );
          break;
        case 5:
          mySerial.write(
            0xFC00 | (((leftSpeed >> 8) & 0xFF) << 1),
            0xFC00 | (((-rightSpeed >> 8) & 0xFF) << 1)
          );
          break;
        case 6:
          mySerial.write(endSignal, endSignal);
          break;
      }
      currentFrame++;
      if (currentFrame > 6) currentFrame = 1;
      //_delay_loop_2(150); // delays between frames
      // we need delay here to let interruptions happen
    }
  }
}

/***
 * A protothread helper function.
 * Calls writeCurrentSpeed after 'interval' microseconds passed.
 */
static int ptTransmisson(struct pt *pt, unsigned long interval) {
  static unsigned long timestamp = 0;
  if (timestamp > micros()) timestamp = micros();
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, micros() - timestamp > interval );
    timestamp = micros();
    writeCurrentSpeed();
  }
  PT_END(pt);
}

/***
 * Reads control from serial port.
 * Development and debugging purpose only.
 */
void listenSerialControl() {
  if (Serial.available() > 0) {
    c=Serial.read();
    if(c == ' ') {
      leftSpeed=0;
      rightSpeed=0;
    } else if(c == 'q') {
      leftSpeed += 50;
    } else if(c == 'a') {
      leftSpeed -= 50;
    } else if(c == 'w') {
      rightSpeed += 50;
    }  else if(c == 's') {
      rightSpeed -= 50;
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
 * A protothread helper function.
 * Calls listenControl after 'interval' microseconds passed.
 */
static int ptListenControl(struct pt *pt, unsigned long interval) {
  static unsigned long timestamp = 0;
  if (timestamp > micros()) timestamp = micros();
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, micros() - timestamp > interval );
    timestamp = micros();
    listenControl();
  }
  PT_END(pt);
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
  }
  leftIncrements = 0;
  timestamp = millis();
}

/***
 * A protothread helper function.
 * Calls calculateIncInPeriod after 'interval' microseconds passed.
 */
static int ptCalculateIncInPeriod(struct pt *pt, unsigned long interval) {
  static unsigned long timestamp = 0;
  if (timestamp > micros()) timestamp = micros();
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, micros() - timestamp > interval );
    timestamp = micros();
    calculateIncInPeriod();
  }
  PT_END(pt);
}

/***
  * Increments changes on left hall effect sensor and detects direction.
  */
void hallLeftEnc() {
  static unsigned long timestamp = 0;
  debugPulse(debugPin, 1);
  bool hallLeftYellowState = digitalRead(hallLeftYellowPin);
  if (!hallLeftYellowState && (timestamp+500) < micros()) {
    if (leftCyclePerSec < 15) //only detect direction under one rotation/s
      leftClockwise = digitalRead(hallLeftBluePin);
    leftIncrements++;
    timestamp = micros();
  }
}

// to be implemented
void hallRightEnc() {
}

void debugInfo() {
  Serial.print("LCPS: ");
  Serial.print(leftCyclePerSec);
  Serial.print("; RCPS: ");
  Serial.println("TBI");
  //Serial.println(millis());
}

/***
 * A protothread helper function.
 * Calls debugInfo after 'interval' microseconds passed.
 */
static int ptDebugInfo(struct pt *pt, unsigned long interval) {
  static unsigned long timestamp = 0;
  if (timestamp > micros()) timestamp = micros();
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, micros() - timestamp > interval );
    timestamp = micros();
    debugInfo();
  }
  PT_END(pt);
}

void setup() {
  PT_INIT(&ptTx);
  PT_INIT(&pIncr);
  PT_INIT(&ptLC);
  PT_INIT(&ptDebug);

  pinMode(inputSelector, OUTPUT);
  pinMode(debugPin, OUTPUT);
  pinMode(hallLeftYellowPin, INPUT);
  pinMode(hallLeftBluePin, INPUT);
  pinMode(hallRightYellowPin, INPUT);
  pinMode(hallRightBluePin, INPUT);
  digitalWrite(inputSelector, LOW); // by start no hijack

  attachInterrupt(digitalPinToInterrupt(hallLeftYellowPin),hallLeftEnc,FALLING);
  attachInterrupt(digitalPinToInterrupt(hallRightYellowPin),hallRightEnc,FALLING);

  mySerial.begin(26275, 13); // hover board UART baud-rate and frame size
  if (_SERIAL) {
    Serial.begin(_SERIAL); // opens serial port, sets data rate to 9600 bps
    Serial.println("CarOne");
  }

  // TODO: need to be controlled from outside
  digitalWrite(inputSelector, HIGH);
}

void loop() { //schedule the protothreads by calling them infinitely
  // 2900ms frame transmission and 300ms in between
  // transmissions, just like gyro sensor does
  ptTransmisson(&ptTx, 5000);
  // calculate increments in every 1/4 sec
  ptCalculateIncInPeriod(&pIncr, 250000);
  // delay 50 milliseconds between control samples
  ptListenControl(&ptLC, 50000);
  // delay 1s between debug info dump
  if (_SERIAL && _DEBUG) ptDebugInfo(&ptDebug, 2000000);
}

