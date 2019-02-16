////////////////////////////////////////////////////////////////////////////////
/*
Mouse.cpp
C++ code for controlling a line following robot.
Inductors are used as pickup coils to sense an induced voltage from a wire
(20kHz, 1Vpp). An additional optical sensor is used to look ahead, and
determine if the robot is travelling straight; if so, the motor speed is
increased.
PID control is used to control the robots positioning (PID.h library)

Author: Alex Robb, University of Bath
Date: 16/2/19
*/
////////////////////////////////////////////////////////////////////////////////
//Includes and libraries
#include <Arduino.h>

#include "pinAssignment.h"

#include "PID.h"
////////////////////////////////////////////////////////////////////////////////
//PID setup
const int baseSpeed = 100;
const int boostSpeed = 200;

//Setpoint for inductor voltage
const double setpoint = 0;
//Result of adding both inductor inputs
double totalError;

//Create PID object   (input, setpoint, kp, ki, kd, sampleTime)
PID sensorPID(totalError, setpoint, 1, 0, 0, 100);

//Output of PID calculation, added/subtracted from motor speed
double motorChange;

////////////////////////////////////////////////////////////////////////////////
//Setup: run once
void setup() {
  totalError = 0;
  motorChange = 0;

  Serial.begin(9600);
}
////////////////////////////////////////////////////////////////////////////////
//overLine: function determines when travelling straight, and how long for
bool overLine(){
  unsigned long startStraight = 0;
  bool straight = false;

  if (!straight && analogRead(lineSensPin) == HIGH){
    startStraight = millis();
    straight = true;
  }
  else if (analogRead(lineSensPin) == LOW){
    straight = false;
  }

  int straightTime = millis() - startStraight;

  if (straightTime >= 1000 && straight){
    return true;
  }
  else {
    return false;
  }
}
////////////////////////////////////////////////////////////////////////////////
//setMotorSpeeds: sets the speed of both left/right motors
void setMotorSpeeds(){
  int motorLeftSpeed;
  int motorRightSpeed;

  if(overLine()){
    motorLeftSpeed = boostSpeed - motorChange;
    motorRightSpeed = boostSpeed + motorChange;
  }
  else{
    motorLeftSpeed = baseSpeed - motorChange;
    motorRightSpeed = baseSpeed + motorChange;
  }

  analogWrite(motorLeftPin, motorLeftSpeed);
  analogWrite(motorRightPin, motorRightSpeed);
}
////////////////////////////////////////////////////////////////////////////////
//loop: run repeatedly
void loop() {

  int inductorLeftValue = analogRead(inductorLeftPin);
  int inductorRightValue = analogRead(inductorRightPin);

  totalError = inductorLeftValue - inductorRightValue;

  motorChange = sensorPID.computePID();

  setMotorSpeeds();
}
