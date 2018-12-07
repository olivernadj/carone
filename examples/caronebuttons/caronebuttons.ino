#include <Wire.h>

#define _I2C_BUS_ADDRESS 8
#define _MAX_FW_SPEED 20
#define _MAX_BW_SPEED 15
#define _MAX_IDDLE_TIME 0 // after x millis will go to park mode
#define _MAX_BREAK_GREACE 500 // for x millis stays break mode

signed int leftCyclePerSec   = 0; //hall effect cycles. 
signed int rightCyclePerSec  = 0; //hall effect cycles. 

signed int leftTargetSpeed = 0;
signed int rightTargetSpeed = 0;

byte i2cBuffer [32]; //i2c read or write buffer

const int driveLed = 13;
const int L1Pin    = 11; // digital pin connected to switch output
const int L2Pin    = 10; // digital pin connected to switch output
const int R2Pin    =  9; // digital pin connected to switch output
const int R1Pin    =  8; // digital pin connected to switch output

bool drive         = false;
bool driveChanged  = false;


/***
  * @return 0 success, 2 address NACK, 3 data NACK, 4 other TWI error
  */
byte i2cRead(byte reg, uint8_t quantity) {
  byte ack, b;
  ack = i2cWrite(reg, 0);
  if (ack) return ack;
  if (quantity) {
    Wire.requestFrom(uint8_t(_I2C_BUS_ADDRESS), quantity);
    for (int i = 0; i < quantity; i++) {
      if (Wire.available()) {
        i2cBuffer[i] = Wire.read();  
      } else {
        return 3;
      }
    }
  }
  return 0;
}

/***
  * @return 0 success, 2 address NACK, 3 data NACK, 4 other TWI error
  */
byte i2cWrite(byte reg, uint8_t quantity) {
  Wire.beginTransmission(uint8_t(_I2C_BUS_ADDRESS));
  Wire.write(reg);
  if (quantity) Wire.write(i2cBuffer, quantity);
  return Wire.endTransmission();
}

void serialInfo() {
  byte sw = !digitalRead(L1Pin) << 3 | !digitalRead(L2Pin) << 2 | !digitalRead(R2Pin) << 1 | !digitalRead(R1Pin);
  Serial.print(sw, BIN);
  Serial.print("Lts:");
  Serial.print(leftTargetSpeed);
  Serial.print(";Rts:");
  Serial.print(rightTargetSpeed);
  Serial.print(";Ls:");
  Serial.print(leftCyclePerSec);
  Serial.print(";Rs:");
  Serial.print(rightCyclePerSec);
  Serial.print(";D:");
  Serial.print(drive);
  Serial.println(";");
}

void speedCheck() {
  i2cRead(B00010111, 5);
  leftCyclePerSec = i2cBuffer[1];
  leftCyclePerSec = leftCyclePerSec << 8;
  leftCyclePerSec |= i2cBuffer[2];
  rightCyclePerSec = i2cBuffer[3];
  rightCyclePerSec = rightCyclePerSec << 8;
  rightCyclePerSec |= i2cBuffer[4];
}

void txSpeed() {
  static signed int leftLastSpeed = 0;
  static signed int rightLastSpeed = 0;
  if (leftLastSpeed != leftTargetSpeed || rightLastSpeed != rightTargetSpeed) {
    i2cBuffer[0] = (leftTargetSpeed >> 8) & 0xFF;
    i2cBuffer[1] = leftTargetSpeed & 0xFF;
    i2cBuffer[2] = (rightTargetSpeed >> 8) & 0xFF;
    i2cBuffer[3] = rightTargetSpeed & 0xFF;
    i2cWrite(B00011100, 4);
    leftLastSpeed = leftTargetSpeed;
    rightLastSpeed = rightTargetSpeed;
  }
}

void buttonCheck() {
  static unsigned long tsParkMode = 0;
  static unsigned long tsGraceBrake = 0;
  byte sw = !digitalRead(L1Pin) << 3 | !digitalRead(L2Pin) << 2 | !digitalRead(R2Pin) << 1 | !digitalRead(R1Pin);
  if (tsGraceBrake < millis()) {
    switch (sw) {
      case B0000:
        leftTargetSpeed = 0;
        rightTargetSpeed = 0;
        if (drive && tsParkMode < millis()) {
          drive = false;
          driveChanged = true;
        }
        break;
      case B0001: // right only go forward
        leftTargetSpeed = 0;
        rightTargetSpeed = _MAX_FW_SPEED;
        tsParkMode = millis() + _MAX_IDDLE_TIME;
        if (!drive) {
          drive = true;
          driveChanged = true;
        }
        break;
      case B0010: // right only go backward
        leftTargetSpeed = 0;
        rightTargetSpeed = -_MAX_BW_SPEED;
        tsParkMode = millis() + _MAX_IDDLE_TIME;
        if (!drive) {
          drive = true;
          driveChanged = true;
        }
        break;
      case B0100: // left only goes backward
        leftTargetSpeed = _MAX_BW_SPEED;
        rightTargetSpeed = 0;
        tsParkMode = millis() + _MAX_IDDLE_TIME;
        if (!drive) {
          drive = true;
          driveChanged = true;
        }
        break;
      case B0101: // left goes backward, right goes forward
        leftTargetSpeed = _MAX_BW_SPEED;
        rightTargetSpeed = _MAX_FW_SPEED;
        tsParkMode = millis() + _MAX_IDDLE_TIME;
        if (!drive) {
          drive = true;
          driveChanged = true;
        }
        break;
      case B0110: // left and right go backward
        leftTargetSpeed = _MAX_BW_SPEED;
        rightTargetSpeed = -_MAX_BW_SPEED;
        tsParkMode = millis() + _MAX_IDDLE_TIME;
        if (!drive) {
          drive = true;
          driveChanged = true;
        }
        break;
      case B1001: // left and right go forward
        leftTargetSpeed = -_MAX_FW_SPEED;
        rightTargetSpeed = _MAX_FW_SPEED;
        tsParkMode = millis() + _MAX_IDDLE_TIME;
        if (!drive) {
          drive = true;
          driveChanged = true;
        }
        break;
      case B1010: // left goes forward, right goes backward
        leftTargetSpeed = -_MAX_FW_SPEED;
        rightTargetSpeed = -_MAX_BW_SPEED;
        tsParkMode = millis() + _MAX_IDDLE_TIME;
        if (!drive) {
          drive = true;
          driveChanged = true;
        }
        break;
      case B1000: // left only goes forward
        leftTargetSpeed = -_MAX_FW_SPEED;
        rightTargetSpeed = 0;
        tsParkMode = millis() + _MAX_IDDLE_TIME;
        if (!drive) {
          drive = true;
          driveChanged = true;
        }
        break;
      default:
        rightTargetSpeed = 0;
        leftTargetSpeed = 0;
        if (drive) {
          drive = false;
          driveChanged = true;
        }
        tsGraceBrake = millis() + _MAX_BREAK_GREACE;
    }
  }
}

void setup() {
  pinMode(L1Pin, INPUT_PULLUP);
  pinMode(L2Pin, INPUT_PULLUP);
  pinMode(R2Pin, INPUT_PULLUP);
  pinMode(R1Pin, INPUT_PULLUP);
  pinMode(driveLed, OUTPUT);
  digitalWrite(driveLed, HIGH);
  delay(500);
  digitalWrite(driveLed, LOW);
  delay(500);
  Serial.begin(9600);
  Serial.println("Hi control!");
  digitalWrite(driveLed, HIGH);
  delay(500);
  digitalWrite(driveLed, LOW);
  delay(500);
  Wire.begin();        // join i2c bus (address optional for master)  
}


void loop() {
  static unsigned long pmTx =  0;
  static unsigned long pmSpeedCheck =  0;
  static unsigned long pmButtonCheck =  0;
  static unsigned long pmSerialInfo =  0;
  static unsigned long pmHijack =  0;
  unsigned long currentMicros = micros();

  if (currentMicros - pmButtonCheck >= 100000) {
    pmButtonCheck = currentMicros;
    buttonCheck();
    return;
  }
  if (currentMicros - pmSpeedCheck >= 250000) {
    pmSpeedCheck = currentMicros;
    speedCheck();
    return;
  }
  if (currentMicros - pmTx >= 100000) {
    pmTx = currentMicros;
    txSpeed();
    return;
  }
  if (currentMicros - pmSerialInfo >= 2000000) {
    pmSerialInfo = currentMicros;
    serialInfo(); 
    return;
  }
  if (driveChanged) {
    if (drive) {
      i2cWrite(B110, 0);
      digitalWrite(driveLed, HIGH);
    } else {
      i2cWrite(B100, 0);
      digitalWrite(driveLed, LOW);
    }
    driveChanged = false;
    return;
  }
  if (currentMicros - pmHijack >= 5000000) {
    // make sure hijack will happen and stay
    pmHijack = currentMicros;
    i2cWrite(B10, 0);
    return;
  }
}


