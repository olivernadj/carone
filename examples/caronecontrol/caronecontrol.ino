#include <Wire.h>

#define _I2C_BUS_ADDRESS 8

unsigned long leftCyclePerSec = 0;
unsigned long rightCyclePerSec = 0;

byte i2cBuffer [32]; //i2c read or write buffer

const int hijackOn = 13;
const int HJ_pin = 3; // digital pin connected to switch output
const int GS_pin = 2; // digital pin connected to switch output
const int X_pin = 0; // analog pin connected to X output
const int Y_pin = 1; // analog pin connected to Y output

int X_neutral;
int Y_neutral;

char shifter = 'P'; // [P]ark or [D]rive
bool hijack = false;
bool gearShiftChanged = false;
bool hijackChanged = false;


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
  int Y, X, L, R;
  Y = (Y_neutral - analogRead(Y_pin)) / 3;
  X = (X_neutral - analogRead(X_pin)) / 4;
  L = Y - X;
  R = Y + X;
  Serial.print(";X:");
  Serial.print(X);
  Serial.print(";Y:");
  Serial.print(Y);
  Serial.print(";L:");
  Serial.print(L);
  Serial.print(";R:");
  Serial.print(R);
  Serial.println(";");
}

void txAcceleration() {
  int Y, X, L, R;
  Y = (Y_neutral - analogRead(Y_pin)) / 3;
  X = (X_neutral - analogRead(X_pin)) / 4;
  L = Y - X;
  R = Y + X;
  i2cBuffer[0] = (L >> 8) & 0xFF;
  i2cBuffer[1] = L & 0xFF;
  i2cBuffer[2] = (-R >> 8) & 0xFF;
  i2cBuffer[3] = -R & 0xFF;
  i2cWrite(B00010000, 4);
}

void gearShiftInterrupt() {
  static unsigned long timestamp = 0;
  bool buttonState = digitalRead(GS_pin);
  if (!buttonState && (timestamp+500) < millis()) {
    gearShiftChanged = true;
  }
  timestamp = millis();
}

void hijackInterrupt() {
  static unsigned long timestamp = 0;
  bool buttonState = digitalRead(HJ_pin);
  if (!buttonState && (timestamp+500) < millis()) {
    hijackChanged = true;
  }
  timestamp = millis();
}

void setup() {
  pinMode(GS_pin, INPUT_PULLUP);
  pinMode(HJ_pin, INPUT_PULLUP);
  pinMode(hijackOn, OUTPUT);
  digitalWrite(hijackOn, LOW);
  attachInterrupt(digitalPinToInterrupt(GS_pin),gearShiftInterrupt,FALLING);
  attachInterrupt(digitalPinToInterrupt(HJ_pin),hijackInterrupt,FALLING);
  Y_neutral = analogRead(Y_pin);
  X_neutral = analogRead(X_pin);
  Wire.begin();        // join i2c bus (address optional for master)  
  Serial.begin(9600);
}


void loop() {
  static unsigned long pmTx =  0;
  static unsigned long pmSerialInfo =  0;
  unsigned long currentMicros = micros();

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
  if (gearShiftChanged) {
    if (shifter == 'P') {
      shifter = 'D';
      i2cWrite(B110, 0);
    } else {
      shifter = 'P';
      i2cWrite(B100, 0);
    }
    Serial.println(shifter);
    gearShiftChanged = false;
    return;
  }
  if (hijackChanged) {
    if (hijack) {
      hijack = false;
      i2cWrite(0, 0);
      digitalWrite(hijackOn, LOW);
    } else {
      hijack = true;
      i2cWrite(B10, 0);
      digitalWrite(hijackOn, HIGH);
    }
    Serial.println(hijack);
    hijackChanged = false;
    return;
  }
}


