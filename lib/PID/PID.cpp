#include "Arduino.h"

#include "PID.h"

PID::PID(int uInput, int uSetpoint, float uKp, float uKi, float uKd, int uSampleTime){

  //Set variables to user inputs
  input = uInput;
  setpoint = uSetpoint;
  sampleTime = uSampleTime;

  //PID constants
  kp = uKp;
  ki = uKi;
  kd = uKd;

  lastTime = millis() - sampleTime;
  lastError = 0;
  errIntegral = 0;
}

double PID::computePID(){

  double PIDsum;
  currentTime = millis();
  timeChange = currentTime - lastTime;

  if (timeChange >= sampleTime){

    error = setpoint - input;
    errIntegral += error * timeChange;
    errDeriv = (error - lastError) / timeChange;

    PIDsum = (kp * error) + (ki * errIntegral) + (kd * errDeriv);

    lastError = error;
    lastTime = currentTime;
  }
  return PIDsum;
}
