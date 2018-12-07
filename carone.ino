#include <SoftSerialParallelWrite.h> // SoftSerialParallelWrite library for parallel UART transmission.

// A4 and A5 can be used for debug purpose only if not used for I2C. The debug signal can be read by digital analyzer or oscilloscope.
#define _DEBUG 0

// Baud-rate, frame size and timing for UART communication between gyro-daughterboard and motherboard .
#define _GYRO_SERIAL 26275 //Baud-rate.
#define _GYRO_FRAMES 10 // Number of frames in one batch.
#define _GYRO_FRAME_LENGTH 450 // Frame length. UART transmission omits stop bits to save time for ATmega328 core. Otherwise poor core, have to spend all the resources to deliver UARD bytes.
#define _GYRO_FRAMES_LENGTH 875 // Frame length + delay between batch.
// Example:
//   Short 450 us frame cycles (time between 2 start bit) happen 10/9 times.
//   Long 875 us cycles (time between 2 start bit) happen 10/1 times.

// Optional Arduino serial for debugging and controlling.
#define _SERIAL_CTRL 9600 // bandwidth or 0 means disabled.
#define _SERIAL_TIMEOUT 5 // Time limits for reading serial inputs in millis. Unfortunately sometimes 5 millis not enough, but with longer time limit there would be not enough time between UART frames.
#define _SERIAL_INFO 1 // Serial var dump is time expensive. Not recommended in production.

// Optional I2C settings. NOTICE: it uses the same pins - A4, A5 like signal debug .
#define _I2C_CTRL 1 // 1 means enabled or 0 means disabled.
#define _I2C_BUS_ADDRESS 8

#if _I2C_CTRL
#include <Wire.h> // I2C Wire library.
#endif
#if _DEBUG && _I2C_CTRL // Throw compile error if both pin debug and I2C enabled.
the_debug_use_the_same_pins_as_I2C_therefore_both_cannot_be_used;
#endif

// ===========+=== SETTINGS FOR DRIVING MECHANISM ============================//
// This settings are basically fine tuning to an instance of hover board.
// Here you can change the driving dynamic, sensors logic, etc.   
//
// Default directing mode
#define _DEFAULT_DIRECTION 0 // 0:acceleration, 1:target speed.
// Measured output from gyro board at ~30 degree.
#define _MAX_GYRO_ACCELERATION 2900
// Maximum allowed speed before the safety parking more activated.
// The hover board tend to loose control above that speed.
#define _MAX_SPEED 40 // Halt changes per sec. (15 changes = 1 wheel rotation).
#define _AUTOCRUISE_ACC 160 // Applied max acceleration in auto-cruise function.
#define _AUTOCRUISE_ACC_STEPS 7 // The acceleration increases in each loop with steps till max or till other condition meet. 
#define _AUTOCRUISE_START_ACC_SPEED 5 // Under this speed the acceleration will goes up to _AUTOCRUISE_ACC. 
#define _AUTOCRUISE_TARGET_TOLERANCE 3 // 0 acceleration if the actual and target speed differ less than this.
#define _AUTOCRUISE_ACC_DIFF_MULTIPLICATOR 3 // The acceleration also depends on the difference of the target and actual speed. This value limit acceleration to x times speed difference.  
// Settings for hall sensor of the wheel.
#define _HALL_DIRECTION_DETECT_LIMIT 30 // Detects direction (cw,ccw) under this hall changes/s limit.
// Although ccw status logically is a boolean, we need to avoid hall check errors. The implemented software solution is to incrementally enforcing the status on each sampling. This method absorbs occasional sampling errors.
#define _HALL_CLOCKWISE_PROBABILITY_CW 6 // Maximum cw enforcement. Example: ccw: -3, -2, -1; cw: 0, 1, 2
#define _HALL_CLOCKWISE_PROBABILITY_CCW -7 // Maximum cw enforcement. Example: ccw: -3, -2, -1; cw: 0, 1, 2
#define _HALL_INTERRUP_GRACE_PERIOD 500 // Interrupt handler will wait x micros before read pin status. 

// =========== END OF SETTINGS FOR DRIVING MECHANISM =========================//


const int debugPin2            =    A5; // Used for Debug only if _DEBUG enabled
const int debugPin1            =    A4; // Used for Debug only if _DEBUG enabled
const int inputSelector        =     7; // Hijack gyro sensor communication. It controls the source for multiplexer (gyro-daughterboard VS CarOne).
const int rightHallBluePin     =     5; // Hall effect direction check. 
const int leftHallBluePin      =     4; // Same as above, but for the right wheel.
const int rightHallYellowPin   =     3; // Hall effect triggering, as interrupt.
const int leftHallYellowPin    =     2; // Same as above, but for the right wheel

volatile unsigned long leftHallInterrupted = 0; //0 means no interruption, otherwise interruption happened at a micro time stamp + _HALL_INTERRUP_GRACE_PERIOD
volatile unsigned long rightHallInterrupted = 0; //Same as above, but for the right wheel.
unsigned int leftIncrements = 0; //Hall effect changes since last calculation
unsigned int rightIncrements = 0; // Same as above, but for the right wheel
unsigned long leftCyclePerSec = 0; //Hall effect cycles per second
unsigned long rightCyclePerSec = 0; // Same as above, but for the right wheel
signed int leftClockwise = 0; // Example: ccw: -3, -2, -1; cw: 0, 1, 2
signed int rightClockwise = 0; // Example: ccw: -3, -2, -1; cw: 0, 1, 2

SoftSerialParallelWrite mySerial(2); // register 2 lower ports of PORTB for parallel UART transmission

volatile byte i2cReg = 0xFF; // Storage for I2C internal register address. 
char command = 'n'; //last received input by serial. n is no acceleration
char shifter = 'P'; // [P]ark or [D]rive
signed int leftAcceleration = 0;
signed int rightAcceleration = 0;
signed int leftTargetSpeed = 0;
signed int rightTargetSpeed = 0;
enum directions {Acceleration, Autocruise} directedby = _DEFAULT_DIRECTION;

signed int leftDirectionChanged = 0;
signed int rightDirectionChanged = 0;

/***
 * Debugging
 *
 * This function generates a brief pulse
 * for debugging or measuring on an oscilloscope.
 */
#if _DEBUG
inline void debugPulse(uint8_t pin, uint16_t count) {
  bool pinStatus = digitalRead(pin);
  while (count--) {
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
        mySerial.write(~rightAccelerationLowFrame, ~leftAccelerationLowFrame);
        currentFrame++;
        break;
      case 3:
      case 5:
        mySerial.write(~rightAccelerationHighFrame, ~leftAccelerationHighFrame);
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
  * Calculates necessary acceleration to catch up with target speed
  */
void calculateAutocruise() {
  if (shifter == 'D') {
    signed int lCps, rCps, lCpsDiff, rCpsDiff;
    lCps = (leftClockwise < 0) ? -1 * int(leftCyclePerSec) : int(leftCyclePerSec); 
    lCpsDiff = leftTargetSpeed - lCps;
    if (abs(lCpsDiff) < _AUTOCRUISE_TARGET_TOLERANCE) {
      leftAcceleration = 0; 
    } else if (lCpsDiff > 0) {
      if (leftAcceleration < _AUTOCRUISE_ACC) leftAcceleration += _AUTOCRUISE_ACC_STEPS;
      else leftAcceleration = _AUTOCRUISE_ACC;
      if (lCps > _AUTOCRUISE_START_ACC_SPEED && leftAcceleration > lCpsDiff * _AUTOCRUISE_ACC_DIFF_MULTIPLICATOR) 
        leftAcceleration = lCpsDiff * _AUTOCRUISE_ACC_DIFF_MULTIPLICATOR;
    } else {
      if (leftAcceleration > -_AUTOCRUISE_ACC) leftAcceleration -= _AUTOCRUISE_ACC_STEPS;
      else leftAcceleration = -_AUTOCRUISE_ACC;
      if (lCps < -_AUTOCRUISE_START_ACC_SPEED && leftAcceleration < lCpsDiff * _AUTOCRUISE_ACC_DIFF_MULTIPLICATOR) 
        leftAcceleration = lCpsDiff * _AUTOCRUISE_ACC_DIFF_MULTIPLICATOR;
    }
    rCps = (rightClockwise < 0) ? -1 * int(rightCyclePerSec) : int(rightCyclePerSec); 
    rCpsDiff = rightTargetSpeed - rCps;
    if (abs(rCpsDiff) < _AUTOCRUISE_TARGET_TOLERANCE) {
      rightAcceleration = 0; 
    } else if (rCpsDiff > 0) {
      if (rightAcceleration < _AUTOCRUISE_ACC) rightAcceleration += _AUTOCRUISE_ACC_STEPS;
      else rightAcceleration = _AUTOCRUISE_ACC;
      if (rCps > _AUTOCRUISE_START_ACC_SPEED && rightAcceleration > rCpsDiff * _AUTOCRUISE_ACC_DIFF_MULTIPLICATOR) 
        rightAcceleration = rCpsDiff * _AUTOCRUISE_ACC_DIFF_MULTIPLICATOR;
    } else {
      if (rightAcceleration > -_AUTOCRUISE_ACC) rightAcceleration -= _AUTOCRUISE_ACC_STEPS;
      else rightAcceleration = -_AUTOCRUISE_ACC;
      if (rCps < -_AUTOCRUISE_START_ACC_SPEED && rightAcceleration < rCpsDiff * _AUTOCRUISE_ACC_DIFF_MULTIPLICATOR) 
        rightAcceleration = rCpsDiff * _AUTOCRUISE_ACC_DIFF_MULTIPLICATOR;
    }
  } else {
    leftAcceleration = 0;
    rightAcceleration = 0;
  }
}

/***
  * Increments changes on left hall effect sensor and detects direction.
  */
void leftHallInc() {
  debugPulse(debugPin1, 2);
  bool leftHallYellowState = digitalRead(leftHallYellowPin);
  if (!leftHallYellowState) {
    debugPulse(debugPin1, 3);
    if (leftCyclePerSec < _HALL_DIRECTION_DETECT_LIMIT) {//only detect direction under X hall changes/s
      debugPulse(debugPin1, 4);
      if (digitalRead(leftHallBluePin)) {
        if (leftClockwise == -1) leftDirectionChanged++;
        if (leftClockwise < _HALL_CLOCKWISE_PROBABILITY_CW) leftClockwise++;
      } else {
        if (leftClockwise == 0) leftDirectionChanged++;
        if (leftClockwise > _HALL_CLOCKWISE_PROBABILITY_CCW) leftClockwise--;
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
    if (rightCyclePerSec < _HALL_DIRECTION_DETECT_LIMIT) {//only detect direction under X hall changes/s
      debugPulse(debugPin1, 4);
      if (!digitalRead(rightHallBluePin)) {
        if (rightClockwise == -1) rightDirectionChanged++;
        if (rightClockwise < _HALL_CLOCKWISE_PROBABILITY_CW) rightClockwise++;
      } else {
        if (rightClockwise == 0) rightDirectionChanged++;
        if (rightClockwise > _HALL_CLOCKWISE_PROBABILITY_CCW) rightClockwise--;
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
  leftHallInterrupted = micros() + _HALL_INTERRUP_GRACE_PERIOD;
}

/***
  * Handles interrupt.
  */
void rightHallInterrupt() {
  debugPulse(debugPin1, 1);
  rightHallInterrupted = micros() + _HALL_INTERRUP_GRACE_PERIOD;
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
        directedby = Acceleration;
        leftAcceleration=0;
        rightAcceleration=0;
        break;
      case 'm':
        directedby = Autocruise;
        leftTargetSpeed=0;
        rightTargetSpeed=0;
        break;
      case 'q':
        directedby = Acceleration;
        if (abs(parsedInt) <= _MAX_GYRO_ACCELERATION) leftAcceleration = parsedInt;
        break;
      case 'w':
        directedby = Acceleration;
        if (abs(parsedInt) <= _MAX_GYRO_ACCELERATION) rightAcceleration = parsedInt;
        break;
      case 'a':
        directedby = Autocruise;
        if (abs(parsedInt) <= _MAX_SPEED) leftTargetSpeed = parsedInt;
        break;
      case 's':
        directedby = Autocruise;
        if (abs(parsedInt) <= _MAX_SPEED) rightTargetSpeed = parsedInt;
        break;
      case 'i':
        digitalWrite(inputSelector, LOW); // turned on
        break;
      case 'o':
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
      case 'l':
        serialInfo();
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
  Serial.print(directedby);
  Serial.print(";cps:");
  if (leftClockwise < 0) Serial.print("-");
  Serial.print(leftCyclePerSec);
  Serial.print(";");
  if (rightClockwise < 0) Serial.print("-");
  Serial.print(rightCyclePerSec);
  Serial.print(";acc:");
  Serial.print(leftAcceleration);
  Serial.print(";");
  Serial.print(rightAcceleration);  
  Serial.print(";ts:");
  Serial.print(leftTargetSpeed);
  Serial.print(";");
  Serial.print(rightTargetSpeed);
  Serial.print(";dc:");
  Serial.print(leftDirectionChanged);
  Serial.print(";");
  Serial.print(rightDirectionChanged);
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
  Serial.setTimeout(_SERIAL_TIMEOUT); // 5ms should be enough to receive few chars
  if (_SERIAL_INFO) {
    Serial.println("\n         _    _\n         \\`../ |o_..__\n          (*)______(*).>\n\n");
    Serial.println("Hi there, CarOne is ready to drive.");
    Serial.println("Commands:");
    Serial.println("\t1\t Hijack on");
    Serial.println("\t0\t Hijack off");
    Serial.println("\tp\t Park");
    Serial.println("\td\t Drive");
    Serial.println("\tn\t Reset acceleration");
    Serial.println("\tq\t Set left side acceleration min: -2900, max: 2900");
    Serial.println("\tw\t Set right side acceleration min: -2900, max: 2900");
    Serial.println("\tm\t Reset target speed");
    Serial.println("\ta\t Set left target speed min: -80, max: 80");
    Serial.println("\ts\t Set right target speed min: -80, max: 80");
    Serial.print("\n1) Last Command; 2) Hijack; 3) Shifter; 4) Driected by; 5) Left Cycle Per Sec; ");
    Serial.print("6) Left Cycle Per Sec; 7) Right Cycle Per Sec; 8) Left Acceleration;");
    Serial.println("9) Right Acceleration; 10) Left target speed; 11) Right target speed;");
  }
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
        directedby = Acceleration;
        leftAcceleration=0;
        rightAcceleration=0;
        break;
      case B00001010: // left acceleration
        if (howMany >= 3) {
          directedby = Acceleration;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          leftAcceleration = si;
        }
        break;
      case B00001110: // right acceleration
        if (howMany >= 3) {
          directedby = Acceleration;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          rightAcceleration = si;
        }
        break;
      case B00010000: // left and right acceleration
        if (howMany >= 5) {
          directedby = Acceleration;
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
      case B00011000: // left target speed
        if (howMany >= 3) {
          directedby = Autocruise;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          leftTargetSpeed = si;
        }
        break;
      case B00011010: // right target speed
        if (howMany >= 3) {
          directedby = Autocruise;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          rightTargetSpeed = si;
        }
        break;
      case B00011100: // left and right target speed
        if (howMany >= 5) {
          directedby = Autocruise;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          leftTargetSpeed = si;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          rightTargetSpeed = si;
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
    case B00010111: // left and right speed
      si = (leftClockwise < 0) ? -1 * leftCyclePerSec : leftCyclePerSec;
      buffer[1] = byte((si >> 8) & 0x00FF);
      buffer[2] = byte(si & 0x00FF);
      si = (rightClockwise < 0) ? -1 * rightCyclePerSec : rightCyclePerSec;
      buffer[3] = byte((si >> 8) & 0x00FF);
      buffer[4] = byte(si & 0x00FF);
      Wire.write(buffer, 5);
    case B00011001: // left target speed
      buffer[1] = byte((leftTargetSpeed >> 8) & 0x00FF);
      buffer[2] = byte(leftTargetSpeed & 0x00FF);
      Wire.write(buffer, 3);
      break;
    case B00011011: // right target speed
      buffer[1] = byte((rightTargetSpeed >> 8) & 0x00FF);
      buffer[2] = byte(rightTargetSpeed & 0x00FF);
      Wire.write(buffer, 3);
      break;
    case B00011101: // left and right target speed
      buffer[1] = byte((leftTargetSpeed >> 8) & 0x00FF);
      buffer[2] = byte(leftTargetSpeed & 0x00FF);
      buffer[3] = byte((rightTargetSpeed >> 8) & 0x00FF);
      buffer[4] = byte(rightTargetSpeed & 0x00FF);
      Wire.write(buffer, 5);
      break;
  }
}
#else
inline void i2cRequestEvent() {}
#endif

/***
 * Initializes I2C wire connection for control.
 */
#if _I2C_CTRL
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
  static unsigned long pmAutocruise =  0;
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

  if (directedby == Autocruise && currentMicros - pmAutocruise >= 250000
      && microsTillNextTx > 100) { // it takes ~20us on 16MHz
    pmAutocruise = currentMicros;
    calculateAutocruise();
    return;
  }

  if (shifter == 'D' && (leftCyclePerSec > _MAX_SPEED || rightCyclePerSec > _MAX_SPEED)) {
    shifter = 'P';    
    if (_SERIAL_CTRL && _SERIAL_INFO) {
      Serial.print("[P] forced; exceed _MAX_SPEED:");
      Serial.print(_MAX_SPEED);
      Serial.print(";lCps");
      Serial.print(leftCyclePerSec);
      Serial.print(";rCps");
      Serial.print(rightCyclePerSec);
      Serial.println(";");
    }
  }

  if (shifter == 'D' && (leftCyclePerSec + rightCyclePerSec) > 40 && (leftClockwise*rightClockwise) > 0) {
    shifter = 'P';    
    if (_SERIAL_CTRL && _SERIAL_INFO) {
      Serial.print("[P] forced; direction detection error:");
      Serial.print(leftClockwise*rightClockwise);
      Serial.print(";lCw");
      Serial.print(leftClockwise);
      Serial.print(";rCcw");
      Serial.print(rightClockwise);
      Serial.print(";lCps");
      Serial.print(leftCyclePerSec);
      Serial.print(";rCps");
      Serial.print(rightCyclePerSec);
      Serial.println(";");
    }
  }

  if (_SERIAL_CTRL) {
    if (microsTillNextTx > 50) {
      listenSerialControl();
    }
    if (_SERIAL_INFO && currentMicros - pmSerialInfo >= 2000000
      && microsTillNextTx > 200) { //it takes ~420us on 16MHz
      pmSerialInfo = currentMicros;
      serialInfo();
      return;
    }
  }
}
