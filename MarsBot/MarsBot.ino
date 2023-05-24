#include "wifi.h"
#include <MotorDriver.h>

//IR Sensors
int leftSens = A0;
int rightSens = A1;
int turnSens = A2;

const int LEFT = 1;
const int RIGHT = 2;
const int TURN = 3;

// Project states
int state = 0;

const int M_WAITING_FOR_INPUT = 1;

const int M_FORWARD = 10;
const int M_LEFT = 11;
const int M_RIGHT = 12;
const int M_UTURN = 13;


int state_step = 0;
const int F_LOOK_FOR_WHITE = 10;
const int F_NORMAL_DRIVE = 11;

const int L_ONE = 20;
const int L_TWO = 21;
const int L_THREE = 22;

const int R_ONE = 30;
const int R_TWO = 31;
const int R_THREE = 32;


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
  printState();
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
  
  
  printState();

    if (WiFi.status() != WL_CONNECTED) {
      status = WiFi.begin(ssid, pass);
    }

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
      forwardNew();
      break;
      
    case M_LEFT:
      break;

      
    case M_RIGHT:
    rightTurnNew();
      break;

      
    case M_UTURN:
      break;

    default:
      return;

  }
  sendUDPMessage(serverIPAddress, serverPort, "0");
}

void printState() {
  Serial.print("State: ");
  Serial.print(state);
  Serial.print(".");
  Serial.println(state_step);
  Serial.print("WiFi: ");
  Serial.println(WiFi.status());
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
      state_step = F_LOOK_FOR_WHITE;
      state = M_FORWARD;
      //forward();
      //Serial.println("Stop forward");
      break;
    case 3:
      Serial.println("Left");
      //leftTurn();
      break;
    case 4:
      Serial.println("Right");
      state_step = R_ONE;
      state = M_RIGHT;
      //rightTurn();
  }

  //resetFunc();
  //delay(100);
  //sendUDPMessage(serverIPAddress, serverPort, "0");
}




bool isBothBlack() {
  return isBlack(LEFT) && isBlack(RIGHT);
}

bool isBlack(int sensor) {
  switch (sensor) {
    case LEFT:
      return analogRead(leftSens) < lightLimit;
    case RIGHT:
      return analogRead(rightSens) < lightLimit;
    case TURN: 
      return analogRead(turnSens) < lightLimit;
  }
  return false;
}



void forwardNew() {
  // Check the forward driving state
  switch (state_step) {

    // Initialy, both sensors are on black, as they stop here on an intersection
    // We should therefore move forward untill at least one of them are on white
    case F_LOOK_FOR_WHITE:
      if (!isBothBlack()) {
        state_step = F_NORMAL_DRIVE;
        return;
      }
      thrust(standardSpeedLeft, standardSpeedRight);
      break;
    
    // Once we initially have passed both black, we start driving normally
    // If one sensor hit black, we correct the motors
    // If both hits black, we hit a new intersection
    case F_NORMAL_DRIVE: 
      // Check if both sensors are on black, meaning a intersection was reached
      if (isBothBlack()) {
        state_step = 0;
        state = M_WAITING_FOR_INPUT;
        //stopEngines();
        return;
      }

      // Check if one of them is on the black line, and therefore should turn a bit
      if (isBlack(LEFT)) {
        thrust(correctionSpeed, standardSpeedLeft);
        return;
      } else if (isBlack(RIGHT)) {
        thrust(standardSpeedLeft, correctionSpeed);
        return;
      }

      // Normal driving, full thrust on both motors
      thrust(standardSpeedLeft, standardSpeedRight);
      break;
  }

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

  Serial.print("Forward: ");
  Serial.print(leftWheel);
  Serial.print(" | ");
  Serial.println(rightWheel);
  
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

void rightTurnNew() {
  switch (state_step) {
    // STEP 1: Sensor is white, turn until sensor hits black
    case R_ONE:
      if (isBlack(TURN)) {
        state_step = R_TWO;
      }
      break;
      
    // STEP 2: Sensor is black, turn until white is hit again
    case R_TWO:
      if (!isBlack(TURN)) {
        state_step = R_THREE;
      }
      break;

    // STEP 3: Sensor is white, turn until black is hit once more
    case R_THREE:
      if (isBlack(TURN)) {
        state_step = 0;
        state = M_WAITING_FOR_INPUT;
        stopEngines();
        return;
      }
      break;
  }

  thrust(standardSpeedLeft, 0);
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
