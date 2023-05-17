#include "wifi.h"
#include <MotorDriver.h>

//IR Sensors
int leftSens = A0;
int rightSens = A1;
int turnSens = A2;


//Car controls
MotorDriver m;

int rightMotor = 3;
int leftMotor = 4;

boolean isActive = true;
boolean isPaused = false;
boolean isStopped = false;

int standardSpeedRight = 190;
int standardSpeedLeft = 190;
int turningSpeed = 190;
int correctionSpeed = 0;

//Change to positive value when using line sensors
int lightLimit = 250;

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

  m.motor(rightMotor,BACKWARD,0);  
  m.motor(leftMotor,BACKWARD,0);  
}

void loop() {
  

  if (isActive) {
    //Send isReady message to server
    //sendUDPMessage(serverIPAddress, serverPort, "1");
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
        Serial.println("Message is not null");
        //sendUDPMessage(serverIPAddress, serverPort, "0");
        runInstruction(message);
        Serial.println(message);
      }
      Serial.println("No message received");
      delay(1000);
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
      Serial.println("Stop forward");
      
      break;
    case 3:
      Serial.println("Left");
      leftTurn();
      break;
    case 4:
      Serial.println("Right");
      rightTurn();
  }

  //Send isReady message to server
  sendUDPMessage(serverIPAddress, serverPort, "1");
}



void forward() {
  while(analogRead(leftSens) < lightLimit && analogRead(rightSens) < lightLimit){
    thrust(standardSpeedLeft, standardSpeedRight);
    
  }
  
  //While we are not at an intersection...
  while(analogRead(leftSens) > lightLimit || analogRead(rightSens) > lightLimit){
    //Left and right sensors are white
    
    
    while (analogRead(leftSens) > lightLimit && analogRead(rightSens) > lightLimit) {
      //Serial.println("Straight");
      
      thrust(standardSpeedLeft, standardSpeedRight);
    }

    //Left is black -> Correct torwards the left by speeding up on right wheel
    while (analogRead(leftSens) < lightLimit && analogRead(rightSens) > lightLimit ) {
      //Serial.println("Right");
      thrust(correctionSpeed, standardSpeedLeft);
    }

    //Right is black -> Correct torwards the right by speeding up on left wheel
    while (analogRead(leftSens) > lightLimit && analogRead(rightSens) < lightLimit) {
      //Serial.println("Left");
      thrust(standardSpeedLeft, correctionSpeed);

    }
  }

  stopEngines();
}

void stopEngines() {
  m.motor(rightMotor,BACKWARD,0);
  m.motor(leftMotor,BACKWARD,0);
}

void thrust(int leftWheel, int rightWheel) {

  //Set the wheels direction -> Forward
  //Set the speed of both wheels corresponding to the two arguments
  m.motor(rightMotor,BACKWARD,rightWheel);
  m.motor(leftMotor,BACKWARD,leftWheel);
  
}

void leftDirection(){
  m.motor(rightMotor,BACKWARD,standardSpeedRight);
  m.motor(leftMotor,FORWARD,0);
}

void rightDirection(){
  m.motor(leftMotor,BACKWARD,standardSpeedLeft);
  m.motor(rightMotor,FORWARD,0);
}

void leftTurn() {
  //Turn left while turnSensor is white
  while(analogRead(turnSens) > lightLimit){
    leftDirection(); 
  }
  //Keep turning while black
  while(analogRead(turnSens) < lightLimit){
    leftDirection();
  }
  //When turnSensor is white again, stop
  stopEngines();
}

void rightTurn() {

  //Turn right while turnSensor is white
  while(analogRead(turnSens) > lightLimit){
    rightDirection();
  }
  //Keep turning while black
  while(analogRead(turnSens) < lightLimit){
    delay(100);
    rightDirection();
  }
  //Keep turning while sensor is, again, white
  while(analogRead(turnSens) > lightLimit){
    rightDirection();
  }
  //When turnSensor is white again, stop
  stopEngines();
}

void uTurnDirection(){

  m.motor(rightMotor,BACKWARD,standardSpeedRight);
  m.motor(leftMotor,FORWARD,standardSpeedLeft);
}

void uTurn() {

  //Turn left while white
  while(analogRead(turnSens) > lightLimit){
    uTurnDirection();
  }
  //Keep turning while black
  while(analogRead(turnSens) < lightLimit){
    uTurnDirection();
  }
  //Keep keep turning while white
  while(analogRead(turnSens) > lightLimit){
    uTurnDirection();
  }
  //Keep turning while black
  while(analogRead(turnSens) < lightLimit){
    uTurnDirection();
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
