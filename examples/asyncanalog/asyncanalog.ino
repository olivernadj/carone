long t, t0;
int lastSample = 0;

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


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

  return (bool) ADCSRA & (1<<ADSC);
}

bool analogReadHasFinish() {
  return ! ((bool) ADCSRA & (1<<ADSC));
}

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


const int debugPin                  =  6;
const int rightHallBlueDebugPin     =  5;
const int leftHallBlueDebugPin      =  4;
const int rightHallYellowDebugPin   =  3;
const int leftHallYellowDebugPin    =  2;


const int rightHallBluePin     =  A3; // Hall effect direction check.
const int leftHallBluePin      =  A2; // Same as above, but for the left wheel.
const int rightHallYellowPin   =  A1; // Hall effect triggering, as interrupt.
const int leftHallYellowPin    =  A0; // Same as above, but for the left wheel

const int shmittTriggerUpper =  450; // (1025 / 5) * 2.2V
const int shmittTriggerLower =  225; // (1025 / 5) * 1.1V

uint8_t rightHallBlueState;
uint8_t leftHallBlueState;
uint8_t rightHallYellowState;
uint8_t leftHallYellowState;

void debugPulse(uint8_t pin, uint16_t count) {
  bool pinStatus = digitalRead(pin);
  while (count--) {
    digitalWrite(pin, 1 ^ pinStatus);
    digitalWrite(pin, pinStatus);
  }
}

void leftHallYellowChanging() {
  debugPulse(debugPin, 1);
  // Serial.println("leftHallYellowChanged");
  // Serial.print("rightHallBlueState: ");
  // Serial.println(rightHallBlueState);
  // Serial.print("leftHallBlueState: ");
  // Serial.println(leftHallBlueState);
  // Serial.print("rightHallYellowState: ");
  // Serial.println(rightHallYellowState);
  // Serial.print("leftHallYellowState: ");
  // Serial.println(leftHallYellowState);
  digitalWrite(rightHallBlueDebugPin, rightHallBlueState);
  digitalWrite(leftHallBlueDebugPin, leftHallBlueState);
  digitalWrite(rightHallYellowDebugPin, rightHallYellowState);
  digitalWrite(leftHallYellowDebugPin, leftHallYellowState);
}

void rightHallYellowChanging() {
  // Serial.println("rightHallYellowChanged");
  // Serial.print("rightHallBlueState: ");
  // Serial.println(rightHallBlueState);
  // Serial.print("leftHallBlueState: ");
  // Serial.println(leftHallBlueState);
  // Serial.print("rightHallYellowState: ");
  // Serial.println(rightHallYellowState);
  // Serial.print("leftHallYellowState: ");
  // Serial.println(leftHallYellowState);
  digitalWrite(rightHallBlueDebugPin, rightHallBlueState);
  digitalWrite(leftHallBlueDebugPin, leftHallBlueState);
  digitalWrite(rightHallYellowDebugPin, rightHallYellowState);
  digitalWrite(leftHallYellowDebugPin, leftHallYellowState);
}

void nothingChanging() {
  // Serial.println("nothingChanged");
  // Serial.print("rightHallBlueState: ");
  // Serial.println(rightHallBlueState);
  // Serial.print("leftHallBlueState: ");
  // Serial.println(leftHallBlueState);
  // Serial.print("rightHallYellowState: ");
  // Serial.println(rightHallYellowState);
  // Serial.print("leftHallYellowState: ");
  // Serial.println(leftHallYellowState);
}



void schmittTrigger() {
  static uint8_t pinPointer = 0;
  int adcResult;
  if (ADCSRA & (1<<ADSC)) {
    return; // ADSC is not cleared yet.
  }
  debugPulse(debugPin, 1);
  adcResult = analogReadResult();
  // Serial.print(adcResult);
  if (pinPointer == B00) { // leftHallYellowPin
    // Serial.print("B00;");
    if (leftHallYellowState == HIGH && adcResult < shmittTriggerLower) {
      leftHallYellowState = LOW;
      leftHallYellowChanging();
    } else if (leftHallYellowState == LOW && adcResult > shmittTriggerUpper) {
      leftHallYellowState = HIGH;
      leftHallYellowChanging();
    }
    analogReadStart(rightHallYellowPin);
  } else if (pinPointer == B01) { // rightHallYellowPin
    // Serial.print("B01;");
    if (rightHallYellowState == HIGH && adcResult < shmittTriggerLower) {
      rightHallYellowState = LOW;
      rightHallYellowChanging();
    } else if (rightHallYellowState == LOW && adcResult > shmittTriggerUpper) {
      rightHallYellowState = HIGH;
      rightHallYellowChanging();
    }
    analogReadStart(leftHallBluePin);
  } else if (pinPointer == B10) { // leftHallBluePin
    // Serial.print("B10;");
    if (leftHallBlueState == HIGH && adcResult < shmittTriggerLower) {
      leftHallBlueState = LOW;
    } else if (leftHallBlueState == LOW && adcResult > shmittTriggerUpper) {
      leftHallBlueState = HIGH;
    }
    analogReadStart(rightHallBluePin);
  } else if (pinPointer == B11) { // rightHallBluePin
    // Serial.print("B11;");
    if (rightHallBlueState == HIGH && adcResult < shmittTriggerLower) {
      rightHallBlueState = LOW;
    } else if (rightHallBlueState == LOW && adcResult > shmittTriggerUpper) {
      rightHallBlueState = HIGH;
    }
    analogReadStart(leftHallYellowPin);
  }
  pinPointer = ++pinPointer & B11;
}

void setup() {
  Serial.begin(9600);
  pinMode(debugPin, OUTPUT);
  pinMode(rightHallBlueDebugPin, OUTPUT);
  pinMode(leftHallBlueDebugPin, OUTPUT);
  pinMode(rightHallYellowDebugPin, OUTPUT);
  pinMode(leftHallYellowDebugPin, OUTPUT);
 
  // AREF = AVcc. Default reference voltage(5V in case of Arduino Uno).  
  ADMUX = (1<<REFS0);
 
  // ADC Enable and prescaler of 128
  // 16000000/128 = 125000
  ADCSRA = (1<<ADEN)|(1<<ADPS2);
}

void loop() {
  static unsigned long pmSchmittTrigger =  0;
  static unsigned long pmNothingChanging =  0;
  unsigned long currentMicros = micros();
  if (currentMicros - pmSchmittTrigger >= 115) { //460us enough to sample 3 data point without a change in max speed
    pmSchmittTrigger = currentMicros;
    //debugPulse(debugPin, 1);
    schmittTrigger(); //it takes ~17us on 16MHz
    //debugPulse(debugPin, 2);
    return;
  }
  if (currentMicros - pmNothingChanging >= 2000000) {
    pmNothingChanging = currentMicros;
    //nothingChanging();
    return;
  }
}
