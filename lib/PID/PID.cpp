//Source file for PID.h library

//Includes
#include "Arduino.h"

#include "PID.h"

//PID object (inputm setpoint, kp, ki, kd)
PID::PID(int uInput, int uSetpoint, float uKp, float uKi, float uKd){

  //Set variables to user inputs
  input = uInput;
  setpoint = uSetpoint;

  //PID constants
  kp = uKp;
  ki = uKi;
  kd = uKd;

  //Default sampling time set at 0.1s
  sampleTime = 100;

  //Initialise last calculation variables
  lastTime = millis() - sampleTime;
  lastError = 0;
  errIntegral = 0;
}

//computePID: returns the output of the PID controller sum
double PID::computePID(){

  double PIDsum;
  currentTime = millis();
  timeChange = currentTime - lastTime;

  //If sampling time has passed
  if (timeChange >= sampleTime){

    //PID calculations
    error = setpoint - input;
    errIntegral += error * timeChange;
    errDeriv = (error - lastError) / timeChange;

    PIDsum = (kp * error) + (ki * errIntegral) + (kd * errDeriv);

    //Set last variables to current at end of function
    lastError = error;
    lastTime = currentTime;
  }
  return PIDsum;
}

//setConstants: tube the PID constants whilst script runs
void PID::setConstants(float uKp, float uKi, float uKd){

  kp = uKp;
  ki = uKi;
  kd = uKd;
}

//setSampleTime: set the sampling time whilst script runs
void PID::setSampleTime(int uSampleTime){

  sampleTime = uSampleTime;
}
