#include <ParaSerialWrite.h> // ParaSerialWrite library for parallel UART transmission
#include <pt.h>              // protothread library

const int ledPin              = 13; //debug purpose
const int inputSelector       = 7; // hijack gyro sensor communication 
const int leftForwardButton   = 6; // user control panel buttons
const int leftBackwardButton  = 5;
const int rightForwardButton  = 4;
const int rightBackwardButton = 3;

ParaSerialWrite mySerial(2); // register 2 lower ports of PORTB for parallel UART transmission
signed int leftSpeed = 0; // left wheel speed
signed int rightSpeed = 0; // right wheel speed
const int maxSpeed = 300; // absolute maximum speed
const int maxManeuveringSpeed = 100; // maximum maneuvering speed
const int acceleration = 50; // x unit acceleration / second^2

enum controlOptions { bySerial, byButton }; // control by serial input or pull up buttons
int control = byButton; // control set to button
char c = 'x'; //last received input by serial. x = neutral

static struct pt ptTx, ptLC; // each protothread needs one of these

/***
 * Transmits imitated UART signal, just like gyro sensor.
 * Uses ParaSerialWrite library what can do multiple UART transmission with 
 * same frame size and baud-rate
 */
void writeCurrentSpeed() {
  if (digitalRead(inputSelector) == LOW) {
    mySerial.write(0, 0);
  } else {
    //0xC00 2 stop bits and <<1 for the start
    uint16_t startSignal = 0xFC00 | (256 << 1); 
    uint16_t endSignal = 0xFC00 | (85 << 1);    
    mySerial.write(startSignal, startSignal);
    _delay_loop_2(150); // delays between frames
    mySerial.write(
      0xFC00 | ((leftSpeed & 0xFF) << 1),
      0xFC00 | ((-rightSpeed & 0xFF) << 1)
    );
    _delay_loop_2(150);
    mySerial.write(
      0xFC00 | (((leftSpeed >> 8) & 0xFF) << 1),
      0xFC00 | (((-rightSpeed >> 8) & 0xFF) << 1)
    );
    _delay_loop_2(150);
    mySerial.write(
      0xFC00 | ((leftSpeed & 0xFF) << 1),
      0xFC00 | ((-rightSpeed & 0xFF) << 1)
    );
    _delay_loop_2(150);
    mySerial.write(
      0xFC00 | (((leftSpeed >> 8) & 0xFF) << 1),
      0xFC00 | (((-rightSpeed >> 8) & 0xFF) << 1)
    );
    _delay_loop_2(150);
    mySerial.write(endSignal, endSignal);
    _delay_loop_2(150);
  }
}

/***
 * A protothread helper function calls writeCurrentSpeed after 'interval' microseconds passed
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
 * Checks car control panel button status and calculates the speed of the wheels.
 */
void listenButtonControl() {
  static unsigned long timestamp = 0;
  unsigned int currentAcceleration;
  unsigned int elapsedTime;
  int maxContextualSpeed;
  int leftForwardState;
  int leftBackwardState;
  int rightForwardState;
  int rightBackwardState;
  leftForwardState   = digitalRead(leftForwardButton);
  leftBackwardState  = digitalRead(leftBackwardButton);
  rightForwardState  = digitalRead(rightForwardButton);
  rightBackwardState = digitalRead(rightBackwardButton);
  if (timestamp > millis() || timestamp == 0) {
    timestamp = millis();
  }
  elapsedTime = millis() - timestamp;
  currentAcceleration = elapsedTime * acceleration / 1000;
  elapsedTime = 1000 * currentAcceleration / acceleration; // elapsed and counted
  timestamp += elapsedTime; // increment only with the counted value 
  if (currentAcceleration > 0) {
    if (leftForwardState == LOW && rightForwardState == LOW ) {
      maxContextualSpeed = maxSpeed; // straight forward
    } else {
      maxContextualSpeed = maxManeuveringSpeed; // turn or backward
    }
    if (leftForwardState == LOW) {
      if (leftSpeed < maxContextualSpeed) { // accelerate till contextual max speed
        leftSpeed += currentAcceleration;
        if (leftSpeed > maxContextualSpeed) leftSpeed = maxContextualSpeed;
      } else if (leftSpeed > maxContextualSpeed) { // slow down to contextual max speed
        leftSpeed -= currentAcceleration;
        if (leftSpeed < maxContextualSpeed) leftSpeed = maxContextualSpeed;        
      }
    } else if (leftBackwardState == LOW) { // backward accelerate till contextual max speed
      if (-leftSpeed < maxContextualSpeed) {
        leftSpeed -= currentAcceleration;
        if (-leftSpeed > maxContextualSpeed) leftSpeed = -maxContextualSpeed;
      }
    } else if (leftSpeed > 0) { // slow down to 0
        leftSpeed -= currentAcceleration;
        if (leftSpeed < 0) leftSpeed = 0;   
    } else if (leftSpeed < 0) { // slow down to 0
        leftSpeed += currentAcceleration;
        if (leftSpeed > 0) leftSpeed = 0;   
    }
    if (rightForwardState == LOW) {
      if (rightSpeed < maxContextualSpeed) { // accelerate till contextual max speed
        rightSpeed += currentAcceleration;
        if (rightSpeed > maxContextualSpeed) rightSpeed = maxContextualSpeed;
      } else if (rightSpeed > maxContextualSpeed) { // slow down to contextual max speed
        rightSpeed -= currentAcceleration;
        if (rightSpeed < maxContextualSpeed) rightSpeed = maxContextualSpeed;        
      }
    } else if (rightBackwardState == LOW) { // backward accelerate till contextual max speed
      if (-rightSpeed < maxContextualSpeed) {
        rightSpeed -= currentAcceleration;
        if (-rightSpeed > maxContextualSpeed) rightSpeed = -maxContextualSpeed;
      }
    } else if (rightSpeed > 0) { // slow down to 0
        rightSpeed -= currentAcceleration;
        if (rightSpeed < 0) rightSpeed = 0;   
    } else if (rightSpeed < 0) { // slow down to 0
        rightSpeed += currentAcceleration;
        if (rightSpeed > 0) rightSpeed = 0;   
    }
  }
}

/***
 * Applies safety speed check and trigger the hijack switch.
 */
int speedCheck() {
  static signed int lastLeftSpeed = 0; // left wheel speed
  static signed int lastRightSpeed = 0; // right wheel speed
  if (leftSpeed != lastLeftSpeed || rightSpeed != lastRightSpeed) {
    lastLeftSpeed = leftSpeed;
    lastRightSpeed = rightSpeed;
  }
  // since it is a vehicle controller better to double check the speed.
  if (abs(leftSpeed) > maxSpeed || abs(rightSpeed) > maxSpeed) {
    Serial.println("Speed limit exceeded. Emergency break activated.");
    leftSpeed = 0;
    rightSpeed = 0;
  }
  if (digitalRead(inputSelector) == LOW && (leftSpeed || rightSpeed)) {
    digitalWrite(inputSelector, HIGH);
  }
}

/***
 * Selects control and apply safety speed check.
 */
void listenControl() {
  if(control == bySerial){
    listenSerialControl();
  } else if (control == byButton) {
    listenButtonControl();
  }
  speedCheck(); // hijack switch, safety check
}

/***
 * A protothread helper function calls listenControl after 'interval' microseconds passed
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

void setup() {
  pinMode(inputSelector, OUTPUT);
  pinMode(leftForwardButton, INPUT_PULLUP);
  pinMode(leftBackwardButton, INPUT_PULLUP);
  pinMode(rightForwardButton, INPUT_PULLUP);
  pinMode(rightBackwardButton, INPUT_PULLUP);
  digitalWrite(inputSelector, LOW); // by start no hijack
  mySerial.begin(26315, 12); // hover board UART baud-rate and frame size 
  Serial.begin(115200); // opens serial port, sets data rate to 9600 bps
  Serial.println("Hello CarOne");
}

void loop() {
  //schedule the two protothreads by calling them infinitely
  ptTransmisson(&ptTx, 3100); // 2900ms frame transmission and 300ms in between 
                              // transmissions, just like gyro sensor does
  ptListenControl(&ptLC, 50000); // delay 50 milliseconds between control samples
}

