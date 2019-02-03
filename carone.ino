#include <SoftSerialParallelWrite.h> // SoftSerialParallelWrite library for parallel UART transmission.
#include <Wire.h> // I2C Wire library.

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
#define _I2C_CTRL 1//1 // 1 means enabled or 0 means disabled.
#define _I2C_BUS_ADDRESS 8

#if _DEBUG && _I2C_CTRL // Throw compile error if both pin debug and I2C enabled.
the_debug_use_the_same_pins_as_I2C_therefore_both_cannot_be_used;
#endif

// ===========+=== SETTINGS FOR DRIVING MECHANISM ============================//
// This settings are basically fine tuning to an instance of hover board.
// Here you can change the driving dynamic, sensors logic, etc.
//
// Default directing mode
#define _DEFAULT_DIRECTION 0 // 0:acceleration, 1:target speed.
// Measured output from gyro board at ~30 degree horizontal angle is 2900.
#define _MAX_GYRO_ACCELERATION 180
// Maximum allowed speed before the safety parking mode activated.
// The hover board tend to loose control above that speed.
#define _MAX_SPEED 142 // Max controllable speed in cm/s. More about units in README.md
#define _SAFETY_SPEED_LIMIT 150 // Force parking mode above that speed. 
#define _AUTOCRUISE_SINGLE_START_FORCE 80 // Initial force. With 0 initial force the acceleration is sluggish. Single mode means only one wheel is working and the acceleration force related to speed is linear.
#define _AUTOCRUISE_SINGLE_FORCE_INC 5 // Force adjusting increments or decrements.
#define _AUTOCRUISE_DUAL_START_FORCE 80 // Initial force. Dual mode means both wheels are working and the acceleration happens exponentially, therefor need to converge to zero. Zero force means keep velocity.
#define _AUTOCRUISE_DUAL_FORCE_INC 5 // Force adjusting increments or decrements. Only used for acceleration from 0 speed.
// Settings for hall sensor of the wheel.
#define _HALL_DIRECTION_DETECT_LIMIT 120 // Detects direction (cw,ccw) under this wheel cm/s limit.
// Although ccw status logically is a boolean, we need to avoid hall check errors. The implemented software solution is to incrementally enforcing the status on each sampling. This method absorbs occasional sampling errors.
#define _HALL_CLOCKWISE_PROBABILITY_CW 2 // Maximum cw enforcement. Example: ccw: -3, -2, -1; cw: 0, 1, 2
#define _HALL_CLOCKWISE_PROBABILITY_CCW -3 // Maximum cw enforcement. Example: ccw: -3, -2, -1; cw: 0, 1, 2

// =========== END OF SETTINGS FOR DRIVING MECHANISM =========================//

SoftSerialParallelWrite  mySerial(2); // register 2 lower ports of PORTB for parallel UART transmission
const int debugPin2            =  A5; // Used for Debug only if _DEBUG enabled
const int debugPin1            =  A4; // Used for Debug only if _DEBUG enabled
const int inputSelector        =   7; // Hijack gyro sensor communication. It controls the source for multiplexer (gyro-daughterboard VS CarOne).
const int rightHallBluePin     =  A0; // Hall effect direction check.
const int leftHallBluePin      =  A1; // Same as above, but for the left wheel.
const int rightHallYellowPin   =  A2; // Hall effect triggering, as interrupt.
const int leftHallYellowPin    =  A3; // Same as above, but for the left wheel.
const int shmittTriggerUpper   = 450; // (1025 / 5) * 2.2V
const int shmittTriggerMiddle  = 337; // (1025 / 5) * 1.65V
const int shmittTriggerLower   = 225; // (1025 / 5) * 1.1V

uint8_t rightHallBlueState;
uint8_t leftHallBlueState;
uint8_t rightHallYellowState;
uint8_t leftHallYellowState;

signed long leftLastRealInterrupt  = 0; // when did the last hall interrupt happened.
signed long rightLastRealInterrupt = 0; // Same as above, but for the right wheel.
signed int leftActualCmPS          = 0; // Measured Absolute wheel speed in cm/s. More about units in README.md
signed int rightActualCmPS         = 0; // Same as above, but for the right wheel
signed int leftForce               = 0; // Accelerating force. A wired unit what gyro extension board sends to motherboard based on its horizontal angle.
signed int rightForce              = 0; // Same as above, but for the right side
signed int leftTargetCmPS          = 0; // Target speed (wheel RMP) in auto-cruise direction mode
signed int rightTargetCmPS         = 0; // Same as above, but for the right wheel
signed int leftClockwise           = 0; // Example: ccw: -3, -2, -1; cw: 0, 1, 2
signed int rightClockwise          = 0; // Example: ccw: -3, -2, -1; cw: 0, 1, 2

char command = 'n'; //last received input by serial. n is no acceleration
char shifter = 'P'; // [P]ark or [D]rive
bool leftDrive = false; // Used for single wheel control
bool rightDrive = false; // Same as above

enum directions {Acceleration, Autocruise} directedby = _DEFAULT_DIRECTION;

volatile byte i2cReg = 0xFF; // Storage for I2C internal register address.

/**
 * Debugging
 *
 * This function generates a brief pulse
 * for debugging or measuring on an oscilloscope.
 * Example:
 *   debugPulse(debugPin1, 1); // expected signal on pin1: __|__
 *   debugPulse(debugPin2, 3); // expected signal on pin2: __|_|_|__
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

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // Macro Set bit in IO port port.
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit)) // Macro Clear bit in IO port port.

/**
 * Trigger an analog read background task for given pin.
 */
bool analogReadStart(uint8_t pin) {
  if (ADCSRA & (1<<ADSC)) {
    return false; // return false if ADC is busy
  }
  if (pin >= 14) pin -= 14; // allow for channel or pin numbers
  // set the analog reference (high two bits of ADMUX) and select the
  // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
  // to 0 (the default).
  ADMUX = (1<<REFS0)| (pin & 0x07);
  // start the conversion
  sbi(ADCSRA, ADSC);
  return true;
}

/**
 * Reads the result from analog measurement done in background.
 */
int analogReadResult() {
  uint8_t low, high;
  // ADSC is cleared when the conversion finishes
  while (ADCSRA & (1<<ADSC));
  // we have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  low  = ADCL;
  high = ADCH;
  // combine the two bytes
  return (high << 8) | low;
}

/**
 * Transmits imitated UART signal, just like gyro sensor does.
 * Uses SoftSerialParallelWrite library what enables to do multiple UART
 * transmission with same frame size and baud-rate
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
    static uint16_t leftEndSignal = parkEndSignal;
    static uint16_t rightEndSignal = parkEndSignal;
    static uint16_t leftForceLowFrame;
    static uint16_t leftForceHighFrame;
    static uint16_t rightForceLowFrame;
    static uint16_t rightForceHighFrame;

    switch (currentFrame) {
      case 1:
        mySerial.write(~startSignal, ~startSignal);
        leftForceLowFrame = 0xFC00 | ((-leftForce & 0xFF) << 1); // Apply invert logic
        leftForceHighFrame = 0xFC00 | (((-leftForce >> 8) & 0xFF) << 1); // Apply invert logic
        rightForceLowFrame = 0xFC00 | ((rightForce & 0xFF) << 1);
        rightForceHighFrame = 0xFC00 | (((rightForce >> 8) & 0xFF) << 1);
        if (shifter == 'D') {
          if (directedby == Autocruise) {
            leftEndSignal = (leftDrive) ? driveEndSignal : parkEndSignal ;
            rightEndSignal = (rightDrive) ? driveEndSignal : parkEndSignal ;
          } else {
            leftEndSignal = driveEndSignal;
            rightEndSignal = driveEndSignal;
          }
        } else {
          leftEndSignal = parkEndSignal;
          rightEndSignal = parkEndSignal;
        }
        currentFrame++;
        break;
      case 2:
      case 4:
        mySerial.write(~rightForceLowFrame, ~leftForceLowFrame);
        currentFrame++;
        break;
      case 3:
      case 5:
        mySerial.write(~rightForceHighFrame, ~leftForceHighFrame);
        currentFrame++;
        break;
      case 6:
        mySerial.write(~rightEndSignal, ~leftEndSignal);
        currentFrame = 1;
        break;
    }
  }
}

/**
  * Calculates necessary acceleration to catch up with target speed
  *
  */
void calculateAutocruise() {
  //debugPulse(debugPin1, 1);
  signed int leftRecentCmPS  = 0;
  signed int rightRecentCmPS = 0;
  static bool onMyWay = false;
  if (shifter != 'D') {
    leftForce = 0;
    rightForce = 0;
    onMyWay = false;
    leftDrive = false;
    rightDrive = false;
  } else if (!onMyWay && (leftTargetCmPS == 0 || rightTargetCmPS == 0)) { // singe side mode
    if (rightDrive == false && leftTargetCmPS != 0) {
      if (leftDrive == false) {
        // always start the drive mode with 0 force
        leftDrive = true;
        leftForce = 0;
      } else if (leftForce == 0) {
        leftForce = (leftTargetCmPS < 0 ? -_AUTOCRUISE_SINGLE_START_FORCE : _AUTOCRUISE_SINGLE_START_FORCE); // initial force
      } else {
        if (leftTargetCmPS > leftActualCmPS) {
          leftForce += _AUTOCRUISE_SINGLE_FORCE_INC; // need to be fine tuned
        } else if(leftTargetCmPS < leftActualCmPS) {
          leftForce -= _AUTOCRUISE_SINGLE_FORCE_INC;
        }
      }
    } else {
      if (leftForce == 0) {
        leftDrive = false;
      } else {
        leftForce = 0;
      }
    }
    if (leftDrive == false && rightTargetCmPS != 0) {
      if (rightDrive == false) {
        rightDrive = true;
        rightForce = 0;
      } else if (rightForce == 0) {
        rightForce = (rightTargetCmPS < 0 ? -_AUTOCRUISE_SINGLE_START_FORCE : _AUTOCRUISE_SINGLE_START_FORCE); // initial force
      } else {
        if (rightTargetCmPS > rightActualCmPS) {
          rightForce += _AUTOCRUISE_SINGLE_FORCE_INC;
        } else if(rightTargetCmPS < rightActualCmPS) {
          rightForce -= _AUTOCRUISE_SINGLE_FORCE_INC;
        }
      }
    } else {
      if (rightForce == 0) {
        rightDrive = false;
      } else {
        rightForce = 0;
      }
    }
  } else if (onMyWay || (leftTargetCmPS != 0 && rightTargetCmPS != 0)) {
    onMyWay = true;
    leftDrive = true;
    rightDrive = true;
    if (leftTargetCmPS != 0 && leftActualCmPS == 0 && leftForce == 0) {
      leftForce = (leftTargetCmPS < 0 ? -_AUTOCRUISE_DUAL_START_FORCE : _AUTOCRUISE_DUAL_START_FORCE); // initial force
    } else if (leftTargetCmPS != 0 && abs(leftActualCmPS) <= 5) {
      if (leftTargetCmPS > 0) {
        leftForce += _AUTOCRUISE_DUAL_FORCE_INC; // need to be fine tuned
      } else {
        leftForce -= _AUTOCRUISE_DUAL_FORCE_INC;
      }
    } else {
      leftForce = leftTargetCmPS - leftActualCmPS;
    }
    if (rightTargetCmPS != 0 && rightActualCmPS == 0 && rightForce == 0) {
      rightForce = (rightTargetCmPS < 0 ? -_AUTOCRUISE_DUAL_START_FORCE : _AUTOCRUISE_DUAL_START_FORCE); // initial force
    } else if (rightTargetCmPS != 0 && abs(rightActualCmPS) <= 5) {
      if (rightTargetCmPS > 0) {
        rightForce += _AUTOCRUISE_DUAL_FORCE_INC; // need to be fine tuned
      } else {
        rightForce -= _AUTOCRUISE_DUAL_FORCE_INC;
      }
    } else {
      rightForce = rightTargetCmPS - rightActualCmPS;
    }
    if (leftTargetCmPS > 0 && rightTargetCmPS < 0) {
      leftForce += 20;
      rightForce -= 20;
    } else if (leftTargetCmPS > 0 && rightTargetCmPS == 0) {
      leftForce += 20;
    } else if (leftTargetCmPS == 0 && rightTargetCmPS > 0) {
      rightForce += 20;
    } else if (leftTargetCmPS < 0 && rightTargetCmPS > 0) {
      leftForce -= 20;
      rightForce += 20;
    }
  }
  if (leftForce > _MAX_GYRO_ACCELERATION) leftForce = _MAX_GYRO_ACCELERATION;
  if (leftForce < -_MAX_GYRO_ACCELERATION) leftForce = -_MAX_GYRO_ACCELERATION;
  if (rightForce > _MAX_GYRO_ACCELERATION) rightForce = _MAX_GYRO_ACCELERATION;
  if (rightForce < -_MAX_GYRO_ACCELERATION) rightForce = -_MAX_GYRO_ACCELERATION;
  leftRecentCmPS = leftActualCmPS;
  rightRecentCmPS = rightActualCmPS;
  //debugPulse(debugPin1, 1);
}

/**
 * Sets the speed to 0 if no hall interruption for a given period.
 * Applies safety brake when unleashed. - "Normally" the hoverboard looses 
 * control above 140 cm/s and accelerates to max speed uncontrollable  
 */
void speedCheck() {
  signed long currentMillis = millis();
  if (currentMillis - leftLastRealInterrupt > 890) {
    leftActualCmPS = 0;
  }
  if (currentMillis - rightLastRealInterrupt > 890) {
    rightActualCmPS = 0;
  }
  static bool safetyReady = false;
  if (shifter == 'D' && (abs(leftActualCmPS) > _SAFETY_SPEED_LIMIT || abs(rightActualCmPS) > _SAFETY_SPEED_LIMIT)) {
    if (safetyReady) { // apply safety break only for 2 consecutive speed violation
      shifter = 'P';
      if (_SERIAL_CTRL && _SERIAL_INFO) {
        Serial.print("[P] forced; exceed _SAFETY_SPEED_LIMIT:");
        Serial.print(_SAFETY_SPEED_LIMIT);
        Serial.print(";l");
        Serial.print(leftActualCmPS);
        Serial.print(";r");
        Serial.print(rightActualCmPS);
        Serial.println("cm/s;");
      }
    } else {
      safetyReady = true;  // 1st violation gracefully ignored
    }
  } else {
    safetyReady = false;
  }
}

/**
 * Implements Schmitt Trigger on analog pins and calculates the wheel speed.
 * Although analogRead() function takes 100 microseconds it can be scaled down
 * and even put it into a background task. 
 * @see http://yaab-arduino.blogspot.com/2015/02/fast-sampling-from-analog-input.html
 * @see 
 */
void schmittTrigger() {
  static bool pinPointer = true;
  static bool leftRecentHallBlueState;
  static bool rightRecentHallBlueState;
  bool stateChanged = false;
  int adcResult;
  int elipsedMillis;
  int absSpeed = 0;
  signed long currentMillis;
  adcResult = analogReadResult();
  if (pinPointer) { // leftHallYellowPin
    if (leftHallYellowState == HIGH && adcResult < shmittTriggerLower) {
      leftHallYellowState = LOW;
      stateChanged = true;
    } else if (leftHallYellowState == LOW && adcResult > shmittTriggerUpper) {
      leftHallYellowState = HIGH;
      stateChanged = true;
    }
    if (stateChanged) {
      // digitalWrite(debugPin2, leftHallYellowState);
      if (abs(leftActualCmPS) < _HALL_DIRECTION_DETECT_LIMIT) analogReadStart(leftHallBluePin);
      if (leftHallYellowState == LOW) {
        currentMillis = millis();
        elipsedMillis = currentMillis - leftLastRealInterrupt;
        if (elipsedMillis < 11) {
          absSpeed = 356;
        } else if (elipsedMillis < 890) {
          absSpeed = 3560 / elipsedMillis;
        }
      }
      if (abs(leftActualCmPS) < _HALL_DIRECTION_DETECT_LIMIT)  {
        adcResult = analogReadResult();
        if (adcResult < shmittTriggerMiddle) {
          leftHallBlueState = LOW;
        } else {
          leftHallBlueState = HIGH;
        }
        if (leftHallYellowState == LOW && leftRecentHallBlueState != leftHallBlueState) {
          if (absSpeed < _HALL_DIRECTION_DETECT_LIMIT) {//only detect direction under X wheel RPM
            if (!leftHallBlueState) {
              if (absSpeed < 5) leftClockwise = 0;
              else if (leftClockwise < _HALL_CLOCKWISE_PROBABILITY_CW) leftClockwise++;
            } else {
              if (absSpeed < 5) leftClockwise = -1;
              else if (leftClockwise > _HALL_CLOCKWISE_PROBABILITY_CCW) leftClockwise--;
            }
          }
          leftLastRealInterrupt = currentMillis;
          leftActualCmPS = (leftClockwise < 0 ? -1 : 1) * absSpeed;
        }
        leftRecentHallBlueState = leftHallBlueState;
      } else if(leftHallYellowState == LOW) {
        leftLastRealInterrupt = currentMillis;
        leftActualCmPS = (leftClockwise < 0 ? -1 : 1) * absSpeed;        
      }
    }
    analogReadStart(rightHallYellowPin);
  } else { // rightHallYellowPin
    if (rightHallYellowState == HIGH && adcResult < shmittTriggerLower) {
      rightHallYellowState = LOW;
      stateChanged = true;
    } else if (rightHallYellowState == LOW && adcResult > shmittTriggerUpper) {
      rightHallYellowState = HIGH;
      stateChanged = true;
    }
    if (stateChanged) {
      // digitalWrite(debugPin2, rightHallYellowState);
      if (abs(rightActualCmPS) < _HALL_DIRECTION_DETECT_LIMIT) analogReadStart(rightHallBluePin);
      if (rightHallYellowState == LOW) {
        currentMillis = millis();
        elipsedMillis = currentMillis - rightLastRealInterrupt;
        if (elipsedMillis < 11) {
          absSpeed = 356;
        } else if (elipsedMillis < 890) {
          absSpeed = 3560 / elipsedMillis;
        }
      }
      if (abs(rightActualCmPS) < _HALL_DIRECTION_DETECT_LIMIT)  {
        adcResult = analogReadResult();
        if (adcResult < shmittTriggerMiddle) {
          rightHallBlueState = LOW;
        } else {
          rightHallBlueState = HIGH;
        }
        if (rightHallYellowState == LOW && rightRecentHallBlueState != rightHallBlueState) {
          if (absSpeed < _HALL_DIRECTION_DETECT_LIMIT) {//only detect direction under X wheel RPM
            if (!rightHallBlueState) {
              if (absSpeed < 5) rightClockwise = 0;
              else if (rightClockwise < _HALL_CLOCKWISE_PROBABILITY_CW) rightClockwise++;
            } else {
              if (absSpeed < 5) rightClockwise = -1;
              else if (rightClockwise > _HALL_CLOCKWISE_PROBABILITY_CCW) rightClockwise--;
            }
          }
          rightLastRealInterrupt = currentMillis;
          rightActualCmPS = (rightClockwise < 0 ? -1 : 1) * absSpeed;
        }
        rightRecentHallBlueState = rightHallBlueState;
      } else if(rightHallYellowState == LOW) {
        rightLastRealInterrupt = currentMillis;
        rightActualCmPS = (rightClockwise < 0 ? -1 : 1) * absSpeed;        
      }
    }
    analogReadStart(leftHallYellowPin);
  }
  pinPointer = !pinPointer;
}

/**
 * Reads control from serial port.
 * For debugging purpose mostly.
 */
#if _SERIAL_CTRL
inline void listenSerialControl() {
  char ch;
  static char c;
  static String toParse = "";
  int parsedInt = 0;
  while (Serial.available() > 0) {
    ch=Serial.read();
    if (ch == ';') {
      if (toParse.length() > 0) {
        parsedInt = toParse.toInt();
        toParse = "";
      } else {
        parsedInt = 0;
      }
      command = c;
      switch (command) {
        case 'n':
          directedby = Acceleration;
          leftForce=0;
          rightForce=0;
          break;
        case 'm':
          directedby = Autocruise;
          leftTargetCmPS=0;
          rightTargetCmPS=0;
          break;
        case 'q':
          directedby = Acceleration;
          if (abs(parsedInt) <= _MAX_GYRO_ACCELERATION) leftForce = parsedInt;
          break;
        case 'w':
          directedby = Acceleration;
          if (abs(parsedInt) <= _MAX_GYRO_ACCELERATION) rightForce = parsedInt;
          break;
        case 'a':
          directedby = Autocruise;
          if (abs(parsedInt) <= _MAX_SPEED) leftTargetCmPS = parsedInt;
          break;
        case 's':
          directedby = Autocruise;
          if (abs(parsedInt) <= _MAX_SPEED) rightTargetCmPS = parsedInt;
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
    } else if (isDigit(ch) || ch == '-') {
      toParse += ch;
    } else if (isAlpha(ch)) {
      c = ch;
    }
  }  
}
#else
inline void listenSerialControl() {}
#endif

/**
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
  Serial.print(";cm/s:");
  Serial.print(leftActualCmPS);
  Serial.print(";");
  Serial.print(rightActualCmPS);
  Serial.print(";F:");
  Serial.print(leftForce);
  Serial.print(";");
  Serial.print(rightForce);
  Serial.print(";ts:");
  Serial.print(leftTargetCmPS);
  Serial.print(";");
  Serial.print(rightTargetCmPS);
  Serial.println(";");
}
#else
inline void serialInfo() {}
#endif

/**
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
    Serial.print("\n1) Last Command; 2) Hijack; 3) Shifter; 4) Directed by; ");
    Serial.print("5) Left cm/s; 6) R. cm/s; 7) Left Force; ");
    Serial.println("8) R. F.; 9) Left target cm/s; 10) R. t. cm/s;");
  }
}
#else
inline void serialInit() {}
#endif

/**
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
        leftForce=0;
        rightForce=0;
        break;
      case B00001010: // left acceleration
        if (howMany >= 3) {
          directedby = Acceleration;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          leftForce = si;
        }
        break;
      case B00001110: // right acceleration
        if (howMany >= 3) {
          directedby = Acceleration;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          rightForce = si;
        }
        break;
      case B00010000: // left and right acceleration
        if (howMany >= 5) {
          directedby = Acceleration;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          leftForce = si;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          rightForce = si;
        }
        break;
      case B00011000: // left target speed
        if (howMany >= 3) {
          directedby = Autocruise;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          leftTargetCmPS = si;
        }
        break;
      case B00011010: // right target speed
        if (howMany >= 3) {
          directedby = Autocruise;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          rightTargetCmPS = si;
        }
        break;
      case B00011100: // left and right target speed
        if (howMany >= 5) {
          directedby = Autocruise;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          leftTargetCmPS = si;
          si = Wire.read();  // receive high byte (overwrites previous reading)
          si = si << 8;    // shift high byte to be high 8 bits
          si |= Wire.read(); // receive low byte as lower 8 bits
          rightTargetCmPS = si;
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

/**
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
      buffer[1] = byte((leftForce >> 8) & 0x00FF);
      buffer[2] = byte(leftForce & 0x00FF);
      buffer[3] = byte((rightForce >> 8) & 0x00FF);
      buffer[4] = byte(rightForce & 0x00FF);
      Wire.write(buffer, 5);
      break;
    case B00001011: // left acceleration
      buffer[1] = byte((leftForce >> 8) & 0x00FF);
      buffer[2] = byte(leftForce & 0x00FF);
      Wire.write(buffer, 3);
      break;
    case B00001111: // right acceleration
      buffer[1] = byte((rightForce >> 8) & 0x00FF);
      buffer[2] = byte(rightForce & 0x00FF);
      Wire.write(buffer, 3);
      break;
    case B00010011: // left speed
      si = (leftClockwise < 0) ? -1 * leftActualCmPS : leftActualCmPS;
      buffer[1] = byte((si >> 8) & 0x00FF);
      buffer[2] = byte(si & 0x00FF);
      Wire.write(buffer, 3);
      break;
    case B00010101: // right speed
      si = (rightClockwise < 0) ? -1 * rightActualCmPS : rightActualCmPS;
      buffer[1] = byte((si >> 8) & 0x00FF);
      buffer[2] = byte(si & 0x00FF);
      Wire.write(buffer, 3);
      break;
    case B00010111: // left and right speed
      si = (leftClockwise < 0) ? -1 * leftActualCmPS : leftActualCmPS;
      buffer[1] = byte((si >> 8) & 0x00FF);
      buffer[2] = byte(si & 0x00FF);
      si = (rightClockwise < 0) ? -1 * rightActualCmPS : rightActualCmPS;
      buffer[3] = byte((si >> 8) & 0x00FF);
      buffer[4] = byte(si & 0x00FF);
      Wire.write(buffer, 5);
    case B00011001: // left target speed
      buffer[1] = byte((leftTargetCmPS >> 8) & 0x00FF);
      buffer[2] = byte(leftTargetCmPS & 0x00FF);
      Wire.write(buffer, 3);
      break;
    case B00011011: // right target speed
      buffer[1] = byte((rightTargetCmPS >> 8) & 0x00FF);
      buffer[2] = byte(rightTargetCmPS & 0x00FF);
      Wire.write(buffer, 3);
      break;
    case B00011101: // left and right target speed
      buffer[1] = byte((leftTargetCmPS >> 8) & 0x00FF);
      buffer[2] = byte(leftTargetCmPS & 0x00FF);
      buffer[3] = byte((rightTargetCmPS >> 8) & 0x00FF);
      buffer[4] = byte(rightTargetCmPS & 0x00FF);
      Wire.write(buffer, 5);
      break;
  }
}
#else
inline void i2cRequestEvent() {}
#endif

/**
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

/**
  * arduino setup
  */
void setup() {
  if (_DEBUG) {
    pinMode(debugPin1, OUTPUT);
    pinMode(debugPin2, OUTPUT);
  }
  pinMode(inputSelector, OUTPUT);
  digitalWrite(inputSelector, HIGH); // by start no hijack
  // AREF = AVcc. Default reference voltage(5V in case of Arduino Uno).  
  ADMUX = (1<<REFS0);
  // ADC Enable and prescaler of 128
  // 16000000/128 = 125000
  ADCSRA = (1<<ADEN)|(1<<ADPS2);
  mySerial.begin(_GYRO_SERIAL, 11); // hover board UART baud-rate and frame size
  serialInit();
  wireInit();
}

/**
 * The loop function is responsible for (time) even handling.
 * The checks are ordered by importance. Maximum one hander function can be
 * called in each loop. E.g. serial write will not be delayed with multiple
 * handlers in one loop.
 */
void loop() {
  static unsigned long pmTx =  0;
  static unsigned long pmSchmittTrigger =  0;
  static unsigned long pmSpeedCheck =  0;
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

  if (currentMicros - pmSchmittTrigger >= 380
      && microsTillNextTx > 25) {  //it takes ~16us-37us on 16MHz
    schmittTrigger();
    pmSchmittTrigger = currentMicros;
    return;
  }

  if (currentMicros - pmSpeedCheck >= 200000
      && microsTillNextTx > 25) {
    pmSpeedCheck = currentMicros;
    debugPulse(debugPin1, 1);
    speedCheck();
    debugPulse(debugPin1, 2);
    return;
  }

  if (directedby == Autocruise && currentMicros - pmAutocruise >= 150000
      && microsTillNextTx > 25) { // it takes ~20us on 16MHz
    pmAutocruise = currentMicros;
    debugPulse(debugPin2, 1);
    calculateAutocruise();
    debugPulse(debugPin2, 2);
    return;
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
