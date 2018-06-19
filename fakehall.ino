#include <pt.h>  // protothread library

static struct pt pt1, pt2, pt3, pt4, pt5; // each protothread needs one of these

#define _SERIAL_DEBUG 9600 // bandwidth or 0 means turned off

const int ledPin = 13;

const int hallYellowPin = 12;
const int hallBluePin = 11;
const int hallGreenPin = 10;

const int waveGenPin = 9;
const int waveMaskSWPin = 8;

const int rotaryEncSWPin = 7;
const int rotaryEncDTPin = 6;
const int rotaryEncCLKPin = 2;

const int maxSpeed = 100; // basically abs(100)

volatile int speed = 0;

int maskSWLastState;
int rotarySWLastState;

bool applyMask = true; // managed by switch
bool maskStateHigh = true;

long blinkLenght = 1000000;  
volatile long hallSignLength = 1000000;  

bool hallYellowState = true;
bool hallBlueState = true;
bool hallGreenState = false;

/***
  * Periodically checks the states of the switch buttons.
  */
void checkSwitches() {
  int maskSWState;
  int rotarySWState;
  maskSWState = digitalRead(waveMaskSWPin);
  rotarySWState = digitalRead(rotaryEncSWPin);
  if (maskSWState != maskSWLastState && !maskSWState) {
    applyMask ^= 1;
  }
  if (rotarySWState != rotarySWLastState && !rotarySWState) {
    speed = 0;
  }
  maskSWLastState = maskSWState;
  rotarySWLastState = rotarySWState;
}

/***
  * A protothread helper function calls checkSwitches after 'interval' microseconds passed
  */
static int ptCheckSwitches(struct pt *pt, unsigned long interval) {
  static unsigned long timestamp = 0;
  if (timestamp > micros()) timestamp = micros();
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, micros() - timestamp > interval );
    timestamp = micros();
    checkSwitches();
  }
  PT_END(pt);
}

/***
  * Hall effect signal generator
  */
void hallSingGen() {
  if (speed > 0) {
    if (hallYellowState == hallBlueState) {
      hallYellowState ^= 1;
    } else if (hallBlueState == hallGreenState) {
      hallBlueState ^= 1;
    } else if (hallGreenState == hallYellowState) {
      hallGreenState ^= 1;
    }
  } else if (speed < 0) {
    if (hallYellowState == hallBlueState) {
      hallBlueState ^= 1;
    } else if (hallBlueState == hallGreenState) {
      hallGreenState ^= 1;
    } else if (hallGreenState == hallYellowState) {
      hallYellowState ^= 1;
    }
  }
  if (!applyMask || maskStateHigh) {
    digitalWrite(hallYellowPin, hallYellowState);
    digitalWrite(hallBluePin, hallBlueState);
    digitalWrite(hallGreenPin, hallGreenState);
  } else {      
    digitalWrite(hallYellowPin, LOW);
    digitalWrite(hallBluePin, LOW);
    digitalWrite(hallGreenPin, LOW);
  }
}

/***
  * A protothread helper function calls hallSingGen after 'interval' microseconds passed
  */
static int ptHallSingGen(struct pt *pt, unsigned long interval) {
  static unsigned long timestamp = 0;
  if (timestamp > micros()) timestamp = micros();
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, micros() - timestamp > interval );
    timestamp = micros();
    hallSingGen();
  }
  PT_END(pt);
}

/***
  * Simple speed indicators
  */
void blink() {
  bool ledstate = digitalRead(ledPin);
  ledstate ^= 1;
  digitalWrite(ledPin, ledstate);
  blinkLenght = (long)1000000 - (long)abs(speed) * 9000;
}

/***
 * A protothread helper function calls blink after 'interval' microseconds passed
 */
static int ptBlink(struct pt *pt, unsigned long interval) {
  static unsigned long timestamp = 0;
  if (timestamp > micros()) timestamp = micros();
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, micros() - timestamp > interval );
    timestamp = micros();
    blink();
  }
  PT_END(pt);
}

/***
  * The bottom flattened sine wave signal generator
  */
void waveGen() {
  static int phase = 0;
  maskStateHigh = ++phase % 3 == 0;
  digitalWrite(waveGenPin, maskStateHigh);
  if (applyMask) { 
    if (maskStateHigh) {
      digitalWrite(hallYellowPin, hallYellowState);
      digitalWrite(hallBluePin, hallBlueState);
      digitalWrite(hallGreenPin, hallGreenState);
    } else {      
      digitalWrite(hallYellowPin, LOW);
      digitalWrite(hallBluePin, LOW);
      digitalWrite(hallGreenPin, LOW);
    }
  }    
}

/***
  * A protothread helper function calls waveGen after 'interval' microseconds passed
  */
static int ptWaveGen(struct pt *pt, unsigned long interval) {
  static unsigned long timestamp = 0;
  if (timestamp > micros()) timestamp = micros();
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, micros() - timestamp > interval );
    timestamp = micros();
    waveGen();
  }
  PT_END(pt);
}

/***
  * Well tested rotary encoder.
  */
void readRotaryEnc() {
  static unsigned long timestamp = 0;
  bool rotaryCLKState = digitalRead(rotaryEncCLKPin);
  if (!rotaryCLKState && (timestamp+500) < micros()) {
    if (digitalRead(rotaryEncDTPin)) {
      if (speed > -maxSpeed) speed --;
    } else {
      if (speed < maxSpeed) speed ++;
    }
    if (speed != 0) hallSignLength = (long)1000000 / (abs(speed) * 6);
    else hallSignLength = 1000000;
  }
  timestamp = micros(); 
}

/***
 * Serial debugger about frequency, speed and settings
 */
void debugInfo() {
  Serial.print("f [Hz]: ");
  Serial.println(speed);
  Serial.print("t [us]: ");
  Serial.println(hallSignLength);
  Serial.print("M: ");
  Serial.println(applyMask);
}
/***
  * A protothread helper function calls debugInfo after 'interval' microseconds passed
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
  PT_INIT(&pt1);
  PT_INIT(&pt2);
  PT_INIT(&pt3);  
  PT_INIT(&pt4);
  PT_INIT(&pt5);

  if (_SERIAL_DEBUG) {
    Serial.begin(9600);
    Serial.println("Hello FakeHall!");
  }

  pinMode(rotaryEncCLKPin, INPUT);
  pinMode(rotaryEncDTPin, INPUT);
  attachInterrupt (digitalPinToInterrupt(rotaryEncCLKPin),readRotaryEnc,FALLING);

  pinMode(ledPin, OUTPUT);
  pinMode(waveGenPin, OUTPUT);

  pinMode(hallYellowPin, OUTPUT);
  pinMode(hallBluePin, OUTPUT);
  pinMode(hallGreenPin, OUTPUT);

  pinMode(waveMaskSWPin, INPUT_PULLUP);
  pinMode(rotaryEncSWPin, INPUT_PULLUP);
  maskSWLastState = digitalRead(waveMaskSWPin);
  rotarySWLastState = digitalRead(rotaryEncSWPin);
}

void loop () {
  if (_SERIAL_DEBUG) ptDebugInfo(&pt1, 2000000);
  ptWaveGen(&pt2, 6667); //150Hz
  ptHallSingGen(&pt3, hallSignLength);
  ptBlink(&pt4, blinkLenght);
  ptCheckSwitches(&pt5, 5000);
}
