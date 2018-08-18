#include <SoftSerialParallelWrite.h> // SoftSerialParallelWrite library for parallel UART transmission

#define _DEBUG 0
#define _GYRO_SERIAL 26275
#define _GYRO_FRAMES 10 // number of frames in one batch
// Example:
//   Short 450 us frame cycles (time between 2 start bit) happen 10/9 times
//   Long 875 us cycles (time between 2 start bit) happen 10/1 times
#define _GYRO_FRAME_LENGTH 450 // frame length
#define _GYRO_FRAMES_LENGTH 875 // frame length + delay between batch

#define _SERIAL_CTRL 9600 // bandwidth or 0 means turned off
#define _I2C_CTRL 100000 // bandwidth or 0 means turned off
#define _I2C_BUS_ADDRESS 8

#if _I2C_CTRL
#include <Wire.h>
#endif

#if _DEBUG && _I2C_CTRL
the_debug_use_the_same_pins_as_I2C_therefore_both_cannot_be_used;
#endif

const int debugPin2           =    A5; // debug during development
const int debugPin1           =    A4; // debug during development
const int inputSelector       =     7; // hijack gyro sensor communication
const int rightHallBluePin    =     5; // hall effect direction check
const int leftHallBluePin     =     4; // hall effect direction check
const int rightHallYellowPin  =     3; // hall effect triggering
const int leftHallYellowPin   =     2; // hall effect triggering
const int maxAcceleration     =  2900; // measured value from gyro board at ~30 degree

volatile unsigned long leftHallInterrupted = 0; //0 means not otherwise micro time stamp
volatile unsigned long rightHallInterrupted = 0; //0 means not otherwise micro time stamp
unsigned int leftIncrements = 0;
unsigned long leftCyclePerSec = 0;
unsigned int rightIncrements = 0;
unsigned long rightCyclePerSec = 0;
// although ccw status logically is a boolean, we need to avoid hall check error
// with software solution. Increasing of the status on each sampling absorb the
// occasional sampling errors.
signed int leftClockwise = 0; // ccw: -3, -2, -1; cw: 0, 1, 2 
signed int rightClockwise = 0; 

SoftSerialParallelWrite mySerial(2); // register 2 lower ports of PORTB for parallel UART transmission

volatile byte i2cReg = 0xFF;
char command = 'n'; //last received input by serial. n is no acceleration
char shifter = 'P'; // [P]ark or [D]rive
signed int leftAcceleration = 0;
signed int rightAcceleration = 0;

/***
 * Debugging
 *
 * This function generates a brief pulse
 * for debugging or measuring on an oscilloscope.
 */
#if _DEBUG
inline void debugPulse(uint8_t pin, uint16_t count)
{
  bool pinStatus = digitalRead(pin);
  while (count--)
  {
    digitalWrite(pin, 1 ^ pinStatus);
    digitalWrite(pin, pinStatus);
  }
}
#else
inline void debugPulse(uint8_t, uint16_t) {}
#endif

/***
 * Transmits imitated UART signal, just like gyro sensor.
 * Uses SoftSerialParallelWrite library what can do multiple UART transmission with
 * same frame size and baud-rate
 *
 * ATTENTION: the logical high and low has been inverted (to be align with NOT
 * gate on logic level shift from 5v to 3.3v) 
 */
void writeCurrentSpeed() {
  if (digitalRead(inputSelector) == HIGH) {
    mySerial.write(~0, ~0);
  } else {
    //0xC00 2 stop bits and <<1 for the start
    const uint16_t startSignal = 0xFC00 | (256 << 1);
    const uint16_t driveEndSignal = 0xFC00 | (85 << 1);
    const uint16_t parkEndSignal = 0xFC00 | (170 << 1);

    static unsigned int currentFrame = 6;
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
        rightAccelerationLowFrame = 0xFC00 | ((rightAcceleration & 0xFF) << 1);
        rightAccelerationHighFrame = 0xFC00 | (((rightAcceleration >> 8) & 0xFF) << 1);
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
  * Calculates last 4 averages of hall effect cycles 
  * and sets to leftCyclePerSec and rightCyclePerSec
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
void leftHallInc() {
  debugPulse(debugPin1, 2);
  bool leftHallYellowState = digitalRead(leftHallYellowPin);
  if (!leftHallYellowState) {
    debugPulse(debugPin1, 3);
    if (leftCyclePerSec < 45) {//only detect direction under 3 rotation/s
      debugPulse(debugPin1, 4);
      if (digitalRead(leftHallBluePin)) {
        if (leftClockwise < 2) leftClockwise++;
      } else {
        if (leftClockwise > -3) leftClockwise--;        
      }
    }
    leftIncrements++;
  }
}

/***
  * Increments changes on right hall effect sensor and detects direction.
  */
void rightHallInc() {
  debugPulse(debugPin1, 2);
  bool rightHallYellowState = digitalRead(rightHallYellowPin);
  if (!rightHallYellowState) {
    debugPulse(debugPin1, 3);
    if (rightCyclePerSec < 45) {//only detect direction under 3 rotation/s
      debugPulse(debugPin1, 4);
      if (!digitalRead(rightHallBluePin)) {
        if (rightClockwise < 2) rightClockwise++;
      } else {
        if (rightClockwise > -3) rightClockwise--;        
      }
    }
    rightIncrements++;
  }
}

/***
  * Handles interrupt.
  */
void leftHallInterrupt() {
  debugPulse(debugPin1, 1);
  leftHallInterrupted = micros() + 500;
}

/***
  * Handles interrupt.
  */
void rightHallInterrupt() {
  debugPulse(debugPin1, 1);
  rightHallInterrupted = micros() + 500;
}

/***
 * Reads control from serial port.
 * For debugging purpose mostly.
 */
#if _SERIAL_CTRL
inline void listenSerialControl() {
  int parsedInt = 0;
  if (Serial.available() > 0) {
    command=Serial.read();
    if (Serial.available() > 0) parsedInt = Serial.parseInt(); 
    switch (command) {
      case 'n':
        leftAcceleration=0;
        rightAcceleration=0;
        break;
      case 'q':
        if (abs(parsedInt) <= maxAcceleration) leftAcceleration = parsedInt;
        break;
      case 'w':
        if (abs(parsedInt) <= maxAcceleration) rightAcceleration = parsedInt;
        break;
      case '1':
        digitalWrite(inputSelector, LOW); // turned on
        break;
      case '0':
        digitalWrite(inputSelector, HIGH); //turned off
        break;
      case 'd':
      case 'D':
        shifter = 'D'; // drive
        break;
      case 'p':
      case 'P':
        shifter = 'P'; // park
        break;
    }
  }
}
#else
inline void listenSerialControl() {}
#endif

/***
 * Print status info on hardware serial connection for mostly debugging purpose.
 */
#if _SERIAL_CTRL
inline void serialInfo() {
  Serial.print(command);
  Serial.print(";");
  Serial.print(!digitalRead(inputSelector));
  Serial.print(";");
  Serial.print(shifter);
  Serial.print(";");
  if (leftClockwise < 0) Serial.print("-");
  Serial.print(leftCyclePerSec);
  Serial.print(";");
  if (rightClockwise < 0) Serial.print("-");
  Serial.print(rightCyclePerSec);
  Serial.print(";");
  Serial.print(leftAcceleration);
  Serial.print(";");
  Serial.print(rightAcceleration);
  Serial.println(";");
}
#else
inline void serialInfo() {}
#endif

/***
 * Initializes hardware serial connection for mostly debugging purpose.
 */
#if _SERIAL_CTRL
inline void serialInit() {
  Serial.begin(_SERIAL_CTRL); // opens serial port
  Serial.setTimeout(10); // 10ms should be enough to receive few chars
  Serial.println("\n         _    _\n         \\`../ |o_..__\n          (*)______(*).>\n\n");
  Serial.println("Hi there, CarOne is ready to drive.");
  Serial.println("Commands:");
  Serial.println("\t1\t Hijack on");
  Serial.println("\t0\t Hijack off");
  Serial.println("\tp\t Park");
  Serial.println("\td\t Drive");
  Serial.println("\tn\t Reset acceleration");
  Serial.println("\tq\t Set left side acceleration min: -2900, max: 2900");
  Serial.println("\tw\t Set left side acceleration min: -2900, max: 2900");
  Serial.print("\nLast Command; Hijack; Shifter; Left Cycle Per Sec; ");
  Serial.println("Right Cycle Per Sec; Left Acceleration; Right Acceleration;");
}
#else
inline void serialInit() {}
#endif

/***
 * Receives I2C data from master, what can use for control or set the reading
 * intention.
 */
#if _I2C_CTRL
inline void i2cReceiveEvent(int howMany) {
  signed int si;
  if (howMany) {
    i2cReg = Wire.read();
    // similarly to I2C address the least significant predict the intention of R/W
    // therefore all write or set register numbers are even. 
    switch (i2cReg) {
      case B00000000: //turn off
        digitalWrite(inputSelector, HIGH); //HIGH => off
        break;
      case B00000010: // turn on
        digitalWrite(inputSelector, LOW); // LOW => on
        break;
      case B00000100: // park
        shifter = 'P';
        break;
      case B00000110: // drive
        shifter = 'D';
        break;
      case B00001000: // zero acceleration
        leftAcceleration=0;
        rightAcceleration=0;
        break;
      case B00001010: // left acceleration
        if (howMany >= 3) {
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          leftAcceleration = si;
        }
        break;
      case B00001110: // right acceleration
        if (howMany >= 3) {
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          rightAcceleration = si;
        }
        break;
      case B00010000: // left and right acceleration
        if (howMany >= 5) {
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          leftAcceleration = si;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          rightAcceleration = si;
        }
        break;
    }
    //prepare for read back the recent status
    i2cReg |= B00000001;
  }
}
#else
inline void i2cReceiveEvent(int) {}
#endif

/***
 * Responses to I2C request from master.
 */
#if _I2C_CTRL
inline void i2cRequestEvent() {
  byte buffer[5] = {i2cReg, 0, 0, 0, 0};
  signed int si;
  // similarly to I2C address the least significant predict the intention of R/W
  // therefore all read register numbers are odd. 
  switch (i2cReg) {
    case B00000001: //turn off
    case B00000011: //turn on
      buffer[1] = byte(!digitalRead(inputSelector));
      Wire.write(buffer, 2);
      break;
    case B00000101: // park
    case B00000111: // drive
      buffer[1] = shifter;
      Wire.write(buffer, 2);
      break;
    case B00001001: // zero acceleration
    case B00010001: // left and right acceleration
      buffer[1] = byte((leftAcceleration >> 8) & 0x00FF);
      buffer[2] = byte(leftAcceleration & 0x00FF);
      buffer[3] = byte((rightAcceleration >> 8) & 0x00FF);
      buffer[4] = byte(rightAcceleration & 0x00FF);
      Wire.write(buffer, 5);
      break;
    case B00001011: // left acceleration
      buffer[1] = byte((leftAcceleration >> 8) & 0x00FF);
      buffer[2] = byte(leftAcceleration & 0x00FF);
      Wire.write(buffer, 3);
      break;
    case B00001111: // right acceleration
      buffer[1] = byte((rightAcceleration >> 8) & 0x00FF);
      buffer[2] = byte(rightAcceleration & 0x00FF);
      Wire.write(buffer, 3);
      break;
    case B00010011: // left speed
      si = (leftClockwise < 0) ? -1 * leftCyclePerSec : leftCyclePerSec;
      buffer[1] = byte((si >> 8) & 0x00FF);
      buffer[2] = byte(si & 0x00FF);
      Wire.write(buffer, 3);
      break;
    case B00010101: // right speed
      si = (rightClockwise < 0) ? -1 * rightCyclePerSec : rightCyclePerSec;
      buffer[1] = byte((si >> 8) & 0x00FF);
      buffer[2] = byte(si & 0x00FF);
      Wire.write(buffer, 3);
      break;
    default:
      si = (leftClockwise < 0) ? -1 * leftCyclePerSec : leftCyclePerSec;
      buffer[1] = byte((si >> 8) & 0x00FF);
      buffer[2] = byte(si & 0x00FF);
      si = (rightClockwise < 0) ? -1 * rightCyclePerSec : rightCyclePerSec;
      buffer[3] = byte((si >> 8) & 0x00FF);
      buffer[4] = byte(si & 0x00FF);
      Wire.write(buffer, 5);
  }
}
#else
inline void i2cRequestEvent() {}
#endif

/***
 * Initializes I2C wire connection for control.
 */
#if _SERIAL_CTRL
inline void wireInit() {
  Wire.begin(_I2C_BUS_ADDRESS); // join i2c bus
  Wire.onRequest(i2cRequestEvent); // register event
  Wire.onReceive(i2cReceiveEvent); // register event
}
#else
inline void wireInit() {}
#endif

/***
  * arduino setup
  */
void setup() {
  if (_DEBUG) {
    pinMode(debugPin1, OUTPUT);
    pinMode(debugPin2, OUTPUT);
  } 
  pinMode(leftHallYellowPin, INPUT);
  pinMode(leftHallBluePin, INPUT);
  pinMode(rightHallYellowPin, INPUT);
  pinMode(rightHallBluePin, INPUT);
  pinMode(inputSelector, OUTPUT);
  digitalWrite(inputSelector, HIGH); // by start no hijack
  attachInterrupt(digitalPinToInterrupt(leftHallYellowPin),leftHallInterrupt,FALLING);
  attachInterrupt(digitalPinToInterrupt(rightHallYellowPin),rightHallInterrupt,FALLING);
  mySerial.begin(_GYRO_SERIAL, 11); // hover board UART baud-rate and frame size
  serialInit();
  wireInit();
}

/***
 * The loop function is responsible for (time) even handling.
 * The checks are ordered by importance. Maximum one hander function can be 
 * called in each loop. E.g. serial write will not be delayed with multiple
 * handlers in one loop.
 */
void loop() {
  static unsigned long pmTx =  0;
  static unsigned long pmCalculateIncInPeriod =  0;
  static unsigned long pmListenControl =  0;
  static unsigned long pmSerialInfo =  0;
  static unsigned int txLength = _GYRO_FRAME_LENGTH;
  static unsigned int txCounter = 0;
  unsigned long currentMicros = micros();
  signed long microsTillNextTx = 0;

  // transmissions, just like gyro sensor does 
  if (currentMicros - pmTx >= txLength) {
    pmTx = currentMicros;
    writeCurrentSpeed();
    txLength = (++txCounter % _GYRO_FRAMES == 0)
      ? _GYRO_FRAMES_LENGTH : _GYRO_FRAME_LENGTH;
    return;
  } else {
    microsTillNextTx = pmTx + txLength - currentMicros;
  }

  if (leftHallInterrupted 
      && leftHallInterrupted < currentMicros 
      && microsTillNextTx > 50) { //it takes ~50us on 16MHz
    leftHallInc();
    leftHallInterrupted = 0;
    return;
  }

  if (rightHallInterrupted 
      && rightHallInterrupted < currentMicros
      && microsTillNextTx > 50) {  //it takes ~50us on 16MHz
    rightHallInc();
    rightHallInterrupted = 0;
    return;
  }

  if (currentMicros - pmCalculateIncInPeriod >= 250000
      && microsTillNextTx > 100) { //it takes ~100us on 16MHz
    pmCalculateIncInPeriod = currentMicros;
    calculateIncInPeriod();
    return;
  }

  if (_SERIAL_CTRL) {
    if (microsTillNextTx > 50) {
      listenSerialControl();
    }
    if (currentMicros - pmSerialInfo >= 2000000
      && microsTillNextTx > 200) { //it takes ~420us on 16MHz
      pmSerialInfo = currentMicros;
      serialInfo(); 
      return;
    }
  }
}

