#include "Arduino.h"

#include "PID.h"

PID::PID(double uInput, double uSetpoint, double uKp, double uKi, double uKd){

  //Set variables to user inputs
  input = uInput;
  setpoint = uSetpoint;

  //PID constants
  kp = uKp;
  ki = uKi;
  kd = uKd;

  //PID sampling time (ms)
  sampleTime = 100;

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
