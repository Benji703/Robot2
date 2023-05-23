#include "wifi.h"
#include <MotorDriver.h>

//IR Sensors
int leftSens = A0;
int rightSens = A1;
int turnSens = A2;

// Project states
int state = 0;

const int M_WAITING_FOR_INPUT = 1;

const int M_FORWARD = 10;
const int M_LEFT = 11;
const int M_RIGHT = 12;
const int M_UTURN = 13;


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
int correctionSpeed = 1;

//Change to positive value when using line sensors
int lightLimit = 250;

int r_var = 0;

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

  state = 0;
}

void(* resetFunc) (void) = 0;

void loop() {
  r_var = (r_var + 1) % 42;
  switch (state) {
    case 0:
      //Send isReady message to server
      sendUDPMessage(serverIPAddress, serverPort, "1");
      state = M_WAITING_FOR_INPUT;
      break;

    case M_WAITING_FOR_INPUT:
      waitForInput();
      break;

    case M_FORWARD:
      thrust(standardSpeedLeft, standardSpeedRight);
      break;
      
    case M_LEFT:
      break;

      
    case M_RIGHT:
      break;

      
    case M_UTURN:
      break;

    default:
      return;

  }

/*
  if (isActive) {
    //Send isReady message to server
    //sendUDPMessage(serverIPAddress, serverPort, "1");
    activeState();
  } else {
    stopEngines();
  }  
  */
}

void waitForInput() {
  int message = listenForUDPMessage();
      
  if (message == NULL) {
    Serial.print("No message");
    Serial.println(r_var);
    delay(1000);
    return;
  }

  Serial.print("Received message ");
  Serial.println(message);

  runInstruction(message);
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
      delay(100 );
    }
    Serial.println("Full stop");
    isActive = false;
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
      //Serial.println("Stop forward");
      break;
    case 3:
      Serial.println("Left");
      //leftTurn();
      break;
    case 4:
      Serial.println("Right");
      //rightTurn();
  }

  //resetFunc();
  //delay(100);
  //sendUDPMessage(serverIPAddress, serverPort, "0");
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

void thrustNew(int unused1, int unused2) {

  //Set the wheels direction -> Forward
  //Set the speed of both wheels corresponding to the two arguments
  m.motor(rightMotor,BACKWARD,standardSpeedRight);
  m.motor(leftMotor,BACKWARD,standardSpeedLeft);
  
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
