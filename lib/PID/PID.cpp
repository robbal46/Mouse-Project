#include "Arduino.h"

#include "PID.h"

PID::PID(double uInput, double uSetpoint, double uKp, double uKi, double uKd){

  input = uInput;
  setpoint = uSetpoint;

  kp = uKp;
  ki = uKi;
  kd = uKd;

  //Set PID sampling time to 100ms
  sampleTime = 100;

  lastTime = millis() - sampleTime;
  lastError = 0;
  errIntegral = 0;
}

double PID::computePID(){

  currentTime = millis();
  timeChange = currentTime - lastTime;


  if(timeChange >= sampleTime){

    error = setpoint - input;
    errIntegral += error * timeChange;
    errDeriv = (error - lastError) / timeChange;

    double PIDsum = (kp * error) + (ki * errIntegral) + (kd * errDeriv);

    lastError = error;
    lastTime = currentTime;

  return PIDsum;
  }
}
