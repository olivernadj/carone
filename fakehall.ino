#include <pt.h>              // protothread library

static struct pt pt1, pt2, pt3, pt4, pt5; // each protothread needs one of these

#define _DEBUG 1

const int ledPin = 13;

const int hallYellowPin = 12;
const int hallBluePin = 11;
const int hallGreenPin = 10;

const int waveGenPin = 9;
const int waveMaskSWPin = 8;

const int rotaryEncSWPin = 7;
const int rotaryEncDTPin = 6;
const int rotaryEncCLKPin = 2;

const int maxSpeed = 100; // abs(100)

int speed = 0;

int maskSWLastState;
int rotarySWLastState;


bool applyMask = true;

long blinkLenght = 1000000;  
volatile long hallSignLength = 1000000;  

bool hallYellowState = true;
bool hallBlueState = true;
bool hallGreenState = false;


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

void hallSingGen() {
  if (speed > 0) {
    if (hallYellowState == hallBlueState) {
      hallYellowState ^= 1;
      digitalWrite(hallYellowPin, hallYellowState);
    } else if (hallBlueState == hallGreenState) {
      hallBlueState ^= 1;
      digitalWrite(hallBluePin, hallBlueState);
    } else if (hallGreenState == hallYellowState) {
      hallGreenState ^= 1;
      digitalWrite(hallGreenPin, hallGreenState);
    }
  } else if (speed < 0) {
    if (hallYellowState == hallBlueState) {
      hallBlueState ^= 1;
      digitalWrite(hallBluePin, hallBlueState);
    } else if (hallBlueState == hallGreenState) {
      hallGreenState ^= 1;
      digitalWrite(hallGreenPin, hallGreenState);
    } else if (hallGreenState == hallYellowState) {
      hallYellowState ^= 1;
      digitalWrite(hallYellowPin, hallYellowState);
    }
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

void blink() {
  boolean ledstate = digitalRead(ledPin); // get LED state
  ledstate ^= 1;   // toggle LED state using xor
  digitalWrite(ledPin, ledstate); // write inversed state back
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

void waveGen() {
  static int phase = 0;
  digitalWrite(waveGenPin, (++phase % 3 == 0));
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

void readRotaryEnc() {
  if (digitalRead(rotaryEncDTPin)) {
    if (speed > -maxSpeed) speed --;
  } else {
    if (speed < maxSpeed) speed ++;
  }
  if (speed != 0) hallSignLength = (long)1000000 / (abs(speed) * 6);
  else hallSignLength = 1000000;
}


void debugInfo() {
  Serial.print("Speed: ");
  Serial.println(speed);
  Serial.print("Bink length: ");
  Serial.println(blinkLenght);
  Serial.print("Hall sign length: ");
  Serial.println(hallSignLength);
  Serial.print("Mask the sign: ");
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

  Serial.begin(9600);
  Serial.println("Hello FakeHall!");

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
  ptDebugInfo(&pt1, 2000000);
  //ptReadRotaryEnc(&pt2, 1000);
  //The bottom flattened sine wave signal generator
  ptWaveGen(&pt2, 6667); //150Hz
  ptHallSingGen(&pt3, hallSignLength);
  ptBlink(&pt4, blinkLenght);
  ptCheckSwitches(&pt5, 5000);
}
