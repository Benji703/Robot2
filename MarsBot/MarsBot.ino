#include "wifi.h"

//Ultrasonic sensor
int trigPin = A1;            //the used trig pin
int echoPin = A5;            //the used echo pin
int distance;               //to store the value

//IR Sensors
int leftSens = A3;
int turnSens = A2;
int rightSens = A4;

//Multiplex ports
int a = 10;
int b = 7;
int c = 4;
int com = 9;

//ShiftRegister Pins
int latchPin = A0; //ST_CP or RCLK ()
int clockPin = 2; //SH_CP or SRCLK ()
int dataPin = 5; //DS or SER ()
int clearPin = 3; //MR or SRCLR (Blue)

int lightState = 0;

//Instructions
int maxInstructions = 25;
int instructions[25] = {2,4,2,3,2,3,2,2,2,3,2,2,1,2,2,4,2,2,2,4,2,4,2,3,2};

//Car controls
int rightSpeed = 3; 
int leftSpeed = 11;

int rightDir = 12;
int leftDir = 13;

boolean isActive = true;
boolean isPaused = false;
boolean isStopped = false;

int standardSpeedRight = 150;
int standardSpeedLeft = 150;
int turningSpeed = 150;
int correctionSpeed = 0;

int lightLimit = 700;

boolean yellowOn = false;

void setup() {
  
  //enables serial
  Serial.begin(9600); 
  setupWifi();

  //Notify server that device is online
  sendUDPMessage(serverIPAddress, serverPort, "0");

  
  //pinMode(trigPin, OUTPUT); //sets pin as OUTPUT
  //pinMode(echoPin, INPUT);  //sets pin as INPUT

  pinMode(leftSens, INPUT);
  pinMode(rightSens, INPUT);
  pinMode(turnSens, INPUT);

  /* For Multiplexer
  pinMode(a, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(c, OUTPUT);
  pinMode(com, INPUT_PULLUP);
  */
  
  //Set pins to output so you can control the shift register
  /*
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clearPin, OUTPUT);

  digitalWrite(clearPin, HIGH);
  digitalWrite(latchPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(dataPin, LOW);

  clearRegister();
  */
  
        
}

void loop() {
  
  checkStart();
  if (isActive) {
    activeState();
  } else {
    analogWrite(rightSpeed, 0);
    analogWrite(leftSpeed, 0);
  }  
}

void activeState() {
  while (isActive) {
      //sendUDPMessage(serverIPAddress, serverPort, String(42));
      int message = listenForUDPMessage();

      
      if (message != NULL) {
        runInstruction(message);
      }
      delay(100);
    }
    Serial.println("Full stop");
    redLEDOn();
    isActive = false;
}

void pauseState() {
  yellowLEDOn();
  stopEngines();
  while (true) {
    changeMultiplexChannel(HIGH, LOW, LOW);
    if (digitalRead(com)) {
      delay(300);
      break;
    }

    changeMultiplexChannel(HIGH, HIGH, HIGH);
    if (digitalRead(com)) {
      delay(300);
      isStopped = true;
      break;
    }
  }
  greenLEDOn();
}

void runInstruction(int instruction){
  switch (instruction) {
    case 1:
      Serial.println("U-Turn");
      //uTurn();
      break;
    case 2:
      Serial.println("Forward");
      //forward();
      break;
    case 3:
      Serial.println("Left");
      //leftTurn();
      break;
    case 4:
      Serial.println("Right");
      //rightTurn();
      break;
  }
}

//function - returns the distance
int getDistance() {
  //sends out a trigger sound
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //returns the received echo in centimeter
  return pulseIn(echoPin, HIGH) * 0.034 / 2;
}

void obstacleCheck(){
  yellowOn = false;
  distance = getDistance();
  while(distance <= 30){
    distance = getDistance();
    //Stops robot activity
    stopEngines();
    if (!yellowOn) {
      yellowOn = true;
      yellowLEDOn(); 
    }
  } 
  
}

void checkStart() {

  //Start
  changeMultiplexChannel(HIGH, LOW, LOW);
  if (digitalRead(com)) {
    delay(300);
    isActive = true;
  }
}

void checkStopPause() {
  //Stop
  changeMultiplexChannel(HIGH, HIGH, HIGH);
  if (digitalRead(com)) {
    delay(300);
    isStopped = true;
  }
  //Pause
  changeMultiplexChannel(HIGH, LOW, LOW);
  if (digitalRead(com)) {
    delay(300);
    pauseState();
  }
}

void forward() {
  forwardLEDOn();
  while(analogRead(leftSens) > lightLimit && analogRead(rightSens) > lightLimit){
    thrust(standardSpeedLeft, standardSpeedRight);
  }
  
  //While we are not at an intersection...
  while(analogRead(leftSens) < lightLimit || analogRead(rightSens) < lightLimit){
    //Left and right sensors are white
    checkStopPause();
    if(isStopped){
      break;
    }
    
    while (analogRead(leftSens) < lightLimit && analogRead(rightSens) < lightLimit) {
      Serial.println("Straight");
      checkStopPause();
      if(!isStopped){
        thrust(standardSpeedLeft, standardSpeedRight);
      }else{
        break;
      }
    }

    //Left is black -> Correct torwards the left by speeding up on right wheel
    while (analogRead(leftSens) > lightLimit && analogRead(rightSens) < lightLimit ) {
      Serial.println("Right");
      checkStopPause();
      if(!isStopped){
        thrust(correctionSpeed, standardSpeedLeft);
      }else{
        break;
      }
    }

    //Right is black -> Correct torwards the right by speeding up on left wheel
    while (analogRead(leftSens) < lightLimit && analogRead(rightSens) > lightLimit) {
      Serial.println("Left");
      checkStopPause();
      if(!isStopped){
        thrust(standardSpeedLeft, correctionSpeed);
      }else{
        break;
      }
    }
  }

  stopEngines();
}

void stopEngines() {
  digitalWrite(rightDir, HIGH);
  digitalWrite(leftDir, HIGH);
  analogWrite(rightSpeed, 0);
  analogWrite(leftSpeed, 0);
}

void thrust(int leftWheel, int rightWheel) {

  obstacleCheck();
  if (yellowOn) {
      yellowOn = false;
      forwardLEDOn(); 
    }
  //Set the wheels direction -> Forward
  digitalWrite(rightDir, HIGH);
  digitalWrite(leftDir, HIGH);
  //Set the speed of both wheels corresponding to the two arguments
  analogWrite(rightSpeed, rightWheel);
  analogWrite(leftSpeed, leftWheel);  
}

void leftDirection(){
  obstacleCheck();
  if (yellowOn) {
      yellowOn = false;
      leftLEDOn(); 
    }
  digitalWrite(rightDir, HIGH);
  analogWrite(rightSpeed, standardSpeedRight);
  digitalWrite(leftSpeed, LOW);
}

void rightDirection(){
  obstacleCheck();
  if (yellowOn) {
      yellowOn = false;
      rightLEDOn(); 
    }
  digitalWrite(leftDir, HIGH);
  analogWrite(leftSpeed, standardSpeedLeft);
  digitalWrite(rightSpeed, LOW);
}

void leftTurn() {
  leftLEDOn();
  //Turn left while turnSensor is white
  while(analogRead(turnSens) < lightLimit){
    checkStopPause();
    if(!isStopped){
      leftDirection(); 
    }else{
      break;
    }
  }
  //Keep turning while black
  while(analogRead(turnSens) > lightLimit){
    checkStopPause();
    if(!isStopped){
      leftDirection();
    }else{
      break;
    }
  }
  //When turnSensor is white again, stop
  analogWrite(rightSpeed,0);
  digitalWrite(rightDir,HIGH);
  digitalWrite(leftDir,HIGH);
}

void rightTurn() {
  rightLEDOn();
  //Turn right while turnSensor is white
  while(analogRead(turnSens) < lightLimit){
    checkStopPause();
    if(!isStopped){
      rightDirection();
    }else{
      break;
    }
  }
  //Keep turning while black
  while(analogRead(turnSens) > lightLimit){
    delay(100);
    checkStopPause();
    if(!isStopped){
      rightDirection();
    }else{
      break;
    }
  }
  //Keep turning while sensor is, again, white
  while(analogRead(turnSens) < lightLimit){
    checkStopPause();
    if(isStopped){
      rightDirection();
    }else{
      break;
    }
  }
  //When turnSensor is white again, stop
  analogWrite(leftSpeed,0);
  digitalWrite(rightDir,HIGH);
  digitalWrite(leftDir,HIGH);
}

void uTurnDirection(){
  obstacleCheck();
  if (yellowOn) {
      yellowOn = false;
      uTurnLEDOn(); 
    }
  digitalWrite(rightDir, HIGH);
  digitalWrite(leftDir, LOW);
  analogWrite(rightSpeed, standardSpeedRight);
  analogWrite(leftSpeed, standardSpeedLeft);
}

void uTurn() {
  uTurnLEDOn();
  //Turn left while white
  while(analogRead(turnSens) < lightLimit){
    checkStopPause();
    if(!isStopped){
      uTurnDirection();
    }else{
      break;
    }
  }
  //Keep turning while black
  while(analogRead(turnSens) > lightLimit){
    checkStopPause();
    if(!isStopped){
      uTurnDirection();
    }else{
      break;
    }
  }
  //Keep keep turning while white
  while(analogRead(turnSens) < lightLimit){
    checkStopPause();
    if(!isStopped){
      uTurnDirection();
    }else{
      break;
    }
  }
  //Keep turning while black
  while(analogRead(turnSens) > lightLimit){
    checkStopPause();
    if(!isStopped){
      uTurnDirection();
    }else{
      break;
    }
  }
  //When turnSensor is white again, stop
  analogWrite(rightSpeed,0);
  analogWrite(leftSpeed,0);
  digitalWrite(rightDir,HIGH);
  digitalWrite(leftDir,HIGH);
}

void changeMultiplexChannel(int c_val, int b_val, int a_val) {
    digitalWrite(a, a_val);
    digitalWrite(b, b_val);
    digitalWrite(c, c_val);
}

void printSensor(){
  Serial.print("Left:");
  Serial.println(analogRead(leftSens));
  Serial.print("Right:");
  Serial.println(analogRead(rightSens));
  Serial.print("Turn:");
  Serial.println(analogRead(turnSens));
  delay(2000);
}

void greenLEDOn() {
  clearRegister();

  digitalWrite(clearPin, HIGH);
  digitalWrite(clearPin, LOW);
  digitalWrite(clearPin, HIGH);

  latchToggle();

  delay(100);
  
  digitalWrite(dataPin, HIGH);
  clockToggle();
  digitalWrite(dataPin, LOW);
  clockToggle();
  clockToggle();
  clockToggle();
  latchToggle();
}

void yellowLEDOn() {
  clearRegister();

  digitalWrite(clearPin, HIGH);
  digitalWrite(clearPin, LOW);
  digitalWrite(clearPin, HIGH);

  latchToggle();

  delay(100);
  
  digitalWrite(dataPin, HIGH);
  clockToggle();
  digitalWrite(dataPin, LOW);
  clockToggle();
  clockToggle();
  latchToggle();
}

void clearRegister() {
  //Clears the 8-bit register
  digitalWrite(clearPin, HIGH);
  digitalWrite(clearPin, LOW);
  digitalWrite(clearPin, HIGH);

  //Commits the empty register
  digitalWrite(latchPin, HIGH);
  digitalWrite(latchPin, LOW);
  digitalWrite(latchPin, HIGH);
}

void forwardLEDOn(){
  clearRegister();
  
  digitalWrite(clearPin, HIGH);
  digitalWrite(clearPin, LOW);
  digitalWrite(clearPin, HIGH);

  latchToggle();

  delay(100);

  digitalWrite(dataPin, HIGH);
  clockToggle();
  digitalWrite(dataPin, LOW);
  digitalWrite(dataPin, HIGH);
  clockToggle();
  digitalWrite(dataPin, LOW);
  clockToggle();
  clockToggle();
  clockToggle();
  
  latchToggle();

}

void uTurnLEDOn(){
  clearRegister();

  digitalWrite(clearPin, HIGH);
  digitalWrite(clearPin, LOW);
  digitalWrite(clearPin, HIGH);

  latchToggle();

  delay(100);

  digitalWrite(dataPin, HIGH);
  clockToggle();
  digitalWrite(dataPin, LOW);

  clockToggle();
  clockToggle();
  clockToggle();

  digitalWrite(dataPin, HIGH);
  clockToggle();
  digitalWrite(dataPin, LOW);

  clockToggle();
  clockToggle();
  clockToggle();
  
  latchToggle();
}

void leftLEDOn(){
  clearRegister();

  digitalWrite(clearPin, HIGH);
  digitalWrite(clearPin, LOW);
  digitalWrite(clearPin, HIGH);

  latchToggle();

  delay(100);

  digitalWrite(dataPin, HIGH);
  clockToggle();
  digitalWrite(dataPin, LOW);

  clockToggle();
  clockToggle();

  digitalWrite(dataPin, HIGH);
  clockToggle();
  digitalWrite(dataPin, LOW);

  clockToggle();
  clockToggle();
  clockToggle();
  
  latchToggle();
}

void rightLEDOn(){
  clearRegister();

  digitalWrite(clearPin, HIGH);
  digitalWrite(clearPin, LOW);
  digitalWrite(clearPin, HIGH);

  latchToggle();

  delay(100);

  digitalWrite(dataPin, HIGH);
  clockToggle();
  digitalWrite(dataPin, LOW);

  clockToggle();

  digitalWrite(dataPin, HIGH);
  clockToggle();
  digitalWrite(dataPin, LOW);

  clockToggle();
  clockToggle();
  clockToggle();
  
  latchToggle();
}

void redLEDOn() {
  clearRegister();

  digitalWrite(clearPin, HIGH);
  digitalWrite(clearPin, LOW);
  digitalWrite(clearPin, HIGH);

  latchToggle();

  delay(100);
  
  digitalWrite(dataPin, HIGH);
  clockToggle();
  digitalWrite(dataPin, LOW);
  clockToggle();
  latchToggle();
}

//Shift register and LED methods
void latchToggle() {
  digitalWrite(latchPin, HIGH);
  digitalWrite(latchPin, LOW);
}

void clockToggle() {
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
}

void customDelay(unsigned long delayTime) {

  unsigned long delayStart = millis();
  while (millis() < (delayStart + delayTime)) {
    //Do checks while delay...
    }
}
