#include <Wire.h>

#define _I2C_BUS_ADDRESS 8
#define _MAX_SPEED 40
#define _MAX_ACCELERATION 100
#define _MAX_IDDLE_TIME 2500 // after x millis will go to park mode


signed int leftCyclePerSec   = 0; //hall effect cycles. 
signed int rightCyclePerSec  = 0; //hall effect cycles. 
signed int leftAcceleration  = 0;
signed int rightAcceleration = 0;

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
  Serial.print("La:");
  Serial.print(leftAcceleration);
  Serial.print(";Ra:");
  Serial.print(rightAcceleration);
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
  if (leftCyclePerSec > 2 * _MAX_SPEED) {
    leftCyclePerSec = 2 * _MAX_SPEED;
  } else if (leftCyclePerSec < 2 * -_MAX_SPEED) {
    leftCyclePerSec = 2 * -_MAX_SPEED;
  }
  rightCyclePerSec = i2cBuffer[3];
  rightCyclePerSec = rightCyclePerSec << 8;
  rightCyclePerSec |= i2cBuffer[4];
  if (rightCyclePerSec > 2 * _MAX_SPEED) {
    rightCyclePerSec = 2 * _MAX_SPEED;
  } else if (rightCyclePerSec < 2 * -_MAX_SPEED) {
    rightCyclePerSec = 2 * -_MAX_SPEED;
  }
}

void txAcceleration() {
  i2cBuffer[0] = (leftAcceleration >> 8) & 0xFF;
  i2cBuffer[1] = leftAcceleration & 0xFF;
  i2cBuffer[2] = (rightAcceleration >> 8) & 0xFF;
  i2cBuffer[3] = rightAcceleration & 0xFF;
  i2cWrite(B00010000, 4);
}

void buttonCheck() {
  static unsigned long ts2ParkMode = 0;
  bool L1PinState = digitalRead(L1Pin) == LOW;
  bool L2PinState = digitalRead(L2Pin) == LOW;
  bool R2PinState = digitalRead(R2Pin) == LOW;
  bool R1PinState = digitalRead(R1Pin) == LOW;

  signed int leftSpeed   = -leftCyclePerSec; 
  signed int rightSpeed  = rightCyclePerSec;
  signed int centerSpeed = (leftSpeed + rightSpeed) / 2;

  float leftAcc  = 0;
  float rightAcc = 0;

  if (L2PinState || R2PinState) { // Park mode
    if (drive) {
      drive = false;
      driveChanged = true;
    }
  } else {
    if (L1PinState || R1PinState) { //acceleration happen
      if (!drive) {
        drive = true;
        driveChanged = true;
      }
      ts2ParkMode = millis() + _MAX_IDDLE_TIME;
    } else {
      if (drive && ts2ParkMode < millis()) {
        drive = false;
        driveChanged = true;
      } 
    }
    if (L1PinState) {
      leftAcc = (float)_MAX_ACCELERATION * ((float)_MAX_SPEED - (float)leftSpeed) / (float)_MAX_SPEED;
    } else if (R1PinState) {
      leftAcc = 0;
    } else {
      leftAcc = (float)_MAX_ACCELERATION * (0.0f - (float)leftSpeed) / (float)_MAX_SPEED;
    }
    if (R1PinState) {
      rightAcc = (float)_MAX_ACCELERATION * ((float)_MAX_SPEED - (float)rightSpeed) / (float)_MAX_SPEED;
    } else if (L1PinState) {
      rightAcc = 0;
    } else {
      rightAcc = (float)_MAX_ACCELERATION * (0.0f - (float)rightSpeed) / (float)_MAX_SPEED;
    }
    leftAcc = -leftAcc;
  }
  leftAcceleration = (int)leftAcc;
  rightAcceleration = (int)rightAcc;
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
  if (currentMicros - pmTx >= 250000) {
    pmTx = currentMicros;
    txAcceleration();
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


