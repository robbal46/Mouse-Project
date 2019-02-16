///////////////////////////////////////////////////////////////////////////////
/*
Mouse.cpp
C++ code for controlling a line following robot.
Inductors are used as pickup coils to sense an induced voltage from a wire.
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
int baseSpeed = 100;
int boostSpeed = 200;

//Setpoint for inductor voltage
const int setpoint = 0;
//Result of adding both inductor inputs
double totalError = 0;

//Create PID object   (input, setpoint, kp, ki, kd)
PID sensorPID(totalError, setpoint, 0, 0, 0);

//Output of PID calculation, added/subtracted from motor speed
double motorChange = 0;

////////////////////////////////////////////////////////////////////////////////
//Setup: run once
void setup() {
  Serial.begin(9600);
}

////////////////////////////////////////////////////////////////////////////////
//overLine: function determines when travelling straight, and how long for
int startStraight = 0;
int stopStraight = 0;
bool straight = false;

int overLine(){
    if(analogRead(!straight && lineSensPin) == HIGH){
    startStraight = millis();
    straight = true;
  }
  else if(straight && analogRead(lineSensPin) == LOW){
    stopStraight = millis();
    straight = false;
  }

  int straightTime = stopStraight - startStraight;

  return straightTime;
}
///////////////////////////////////////////////////////////////////////////////
//setMotorSpeeds: sets the speed of both left/right motors
void setMotorSpeeds(){
  double motorLeftSpeed;
  double motorRightSpeed;

  if(overLine() >= 1000){
    motorLeftSpeed = baseSpeed - motorChange;
    motorRightSpeed = baseSpeed + motorChange;
  }
  else{
    motorLeftSpeed = boostSpeed - motorChange;
    motorRightSpeed = boostSpeed + motorChange;
  }

  analogWrite(motorLeftPin, motorLeftSpeed);
  analogWrite(motorRightPin, motorRightSpeed);
}
///////////////////////////////////////////////////////////////////////////////
//loop: run repeatedly
void loop() {

  int inductorLeftValue = analogRead(inductorLeftPin);
  int inductorRightValue = analogRead(inductorRightPin);

  totalError = inductorLeftValue - inductorRightValue;

  motorChange = sensorPID.computePID();

  setMotorSpeeds();
}
