#include "wifi.h"
#include <MotorDriver.h>

//IR Sensors
int leftSens = A0;
int turnSens = A2;
int rightSens = A1;




//Car controls
MotorDriver m;

int rightMotor = 3;
int leftMotor = 4;

boolean isActive = true;
boolean isPaused = false;
boolean isStopped = false;

int standardSpeedRight = 150;
int standardSpeedLeft = 150;
int turningSpeed = 150;
int correctionSpeed = 0;

//Change to positive value when using line sensors
int lightLimit = -700;

boolean yellowOn = false;

void setup() {
  
  //enables serial
  Serial.begin(9600); 
  setupWifi();

  //Notify server that device is online
  sendUDPMessage(serverIPAddress, serverPort, "0");

  pinMode(leftSens, INPUT);
  pinMode(rightSens, INPUT);
  pinMode(turnSens, INPUT);

  m.motor(rightMotor,RELEASE,0);  
  m.motor(leftMotor,RELEASE,0);  
}

void loop() {
  

  if (isActive) {
    //Send isReady message to server
    sendUDPMessage(serverIPAddress, serverPort, "1");
    activeState();
  } else {
    stopEngines();
  }  
}

void activeState() {
  while (isActive) {
      int message = listenForUDPMessage();
      
      if (message != NULL) {
        //Send isNotReady message to server
        sendUDPMessage(serverIPAddress, serverPort, "0");
        runInstruction(message);
      }
      delay(100);
    }
    Serial.println("Full stop");
    isActive = false;
}


void runInstruction(int instruction){
  switch (instruction) {
    case 1:
      Serial.println("U-Turn");
      uTurn();
      break;
    case 2:
      Serial.println("Forward");
      forward();
      break;
    case 3:
      Serial.println("Left");
      leftTurn();
      break;
    case 4:
      Serial.println("Right");
      rightTurn();
      break;
  }

  //Send isReady message to server
  sendUDPMessage(serverIPAddress, serverPort, "1");
}



void forward() {
  while(analogRead(leftSens) > lightLimit && analogRead(rightSens) > lightLimit){
    thrust(standardSpeedLeft, standardSpeedRight);
  }
  
  //While we are not at an intersection...
  while(analogRead(leftSens) < lightLimit || analogRead(rightSens) < lightLimit){
    //Left and right sensors are white
    if(isStopped){
      break;
    }
    
    while (analogRead(leftSens) < lightLimit && analogRead(rightSens) < lightLimit) {
      Serial.println("Straight");
      if(!isStopped){
        thrust(standardSpeedLeft, standardSpeedRight);
      }else{
        break;
      }
    }

    //Left is black -> Correct torwards the left by speeding up on right wheel
    while (analogRead(leftSens) > lightLimit && analogRead(rightSens) < lightLimit ) {
      Serial.println("Right");
      if(!isStopped){
        thrust(correctionSpeed, standardSpeedLeft);
      }else{
        break;
      }
    }

    //Right is black -> Correct torwards the right by speeding up on left wheel
    while (analogRead(leftSens) < lightLimit && analogRead(rightSens) > lightLimit) {
      Serial.println("Left");
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
  m.motor(rightMotor,RELEASE,0);
  m.motor(leftMotor,RELEASE,0);
  m.motor(rightMotor,FORWARD,0);
  m.motor(leftMotor,FORWARD,0);
}

void thrust(int leftWheel, int rightWheel) {

  //Set the wheels direction -> Forward
  //Set the speed of both wheels corresponding to the two arguments
  m.motor(rightMotor,FORWARD,rightWheel);
  m.motor(leftMotor,FORWARD,leftWheel);
  
}

void leftDirection(){
  m.motor(rightMotor,FORWARD,standardSpeedRight);
  m.motor(leftMotor,BACKWARD,standardSpeedLeft);
}

void rightDirection(){
  m.motor(leftMotor,FORWARD,standardSpeedLeft);
  m.motor(rightMotor,BACKWARD,standardSpeedRight);
}

void leftTurn() {
  //Turn left while turnSensor is white
  while(analogRead(turnSens) < lightLimit){

    if(!isStopped){
      leftDirection(); 
    }else{
      break;
    }
  }
  //Keep turning while black
  while(analogRead(turnSens) > lightLimit){

    if(!isStopped){
      leftDirection();
    }else{
      break;
    }
  }
  //When turnSensor is white again, stop
  m.motor(rightMotor,RELEASE,0);
  m.motor(rightMotor,FORWARD,0);
  m.motor(leftMotor,FORWARD,0);
}

void rightTurn() {

  //Turn right while turnSensor is white
  while(analogRead(turnSens) < lightLimit){

    if(!isStopped){
      rightDirection();
    }else{
      break;
    }
  }
  //Keep turning while black
  while(analogRead(turnSens) > lightLimit){
    delay(100);

    if(!isStopped){
      rightDirection();
    }else{
      break;
    }
  }
  //Keep turning while sensor is, again, white
  while(analogRead(turnSens) < lightLimit){

    if(isStopped){
      rightDirection();
    }else{
      break;
    }
  }
  //When turnSensor is white again, stop
  stopEngines();
}

void uTurnDirection(){

  m.motor(rightMotor,FORWARD,standardSpeedRight);
  m.motor(leftMotor,BACKWARD,standardSpeedLeft);
}

void uTurn() {

  //Turn left while white
  while(analogRead(turnSens) < lightLimit){
    if(!isStopped){
      uTurnDirection();
    }else{
      break;
    }
  }
  //Keep turning while black
  while(analogRead(turnSens) > lightLimit){
    if(!isStopped){
      uTurnDirection();
    }else{
      break;
    }
  }
  //Keep keep turning while white
  while(analogRead(turnSens) < lightLimit){
    if(!isStopped){
      uTurnDirection();
    }else{
      break;
    }
  }
  //Keep turning while black
  while(analogRead(turnSens) > lightLimit){
    if(!isStopped){
      uTurnDirection();
    }else{
      break;
    }
  }
  //When turnSensor is white again, stop
  stopEngines();
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

void customDelay(unsigned long delayTime) {

  unsigned long delayStart = millis();
  while (millis() < (delayStart + delayTime)) {
    //Do checks while delay...
    }
}
