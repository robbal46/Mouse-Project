////////////////////////////////////////////////////////////////////////////////
/*
Mouse.ino
C++ code for controlling a line following robot.
Inductors are used as pickup coils to sense an induced voltage from a wire
(20kHz, 1A pp). An additional optical sensor is used to look ahead, and
determine if the robot is travelling straight; if so, the motor speed is
increased.
PID control is used to control the robots positioning (PID.h library)

Author: Alex Robb, University of Bath
Date: 17/2/19
*/
////////////////////////////////////////////////////////////////////////////////
//Includes and libraries

#include <PID_v1.h>

#define motorLeftPin 5
#define motorRightPin 6

#define inductorLeftPin A0
#define inductorRightPin A1

#define lineSensPin 4

////////////////////////////////////////////////////////////////////////////////
//PID setup
const int baseSpeed = 70;
const int boostSpeed = 150;

//Setpoint for inductor voltage
double setpoint;
//Result of adding both inductor inputs
double totalError;

//Output of PID calculation, added/subtracted from motor speed
double motorChange;

//Varibles to store values from inductor
int inductorLeftValue;
int inductorRightValue;

double distanceLeft;
double distanceRight;

//Create PID object   (input, setpoint, kp, ki, kd, sampleTime)
PID sensorPID(&totalError, &motorChange, &setpoint, 0.7
, 0, 0.06, DIRECT);

////////////////////////////////////////////////////////////////////////////////
//Setup: run once
void setup() {

  //Set pimodes to input/output
  pinMode(inductorLeftPin, INPUT);
  pinMode(inductorRightPin, INPUT);
  pinMode(motorLeftPin, OUTPUT);
  pinMode(motorRightPin, OUTPUT);

  //Initialise the inductor value variables
  inductorLeftValue = analogRead(inductorLeftPin);
  inductorRightValue = analogRead(inductorRightPin);

  distanceLeft = 131.7*exp(-0.002438*inductorLeftValue);
  distanceRight = 133.4*exp(-0.00269*inductorRightValue);

  //totalError = inductorLeftValue - inductorRightValue;
  totalError = distanceLeft - distanceRight;

  //Initialis setpoint variable as 0
  setpoint = totalError;

  sensorPID.SetOutputLimits(-255, 255);

  sensorPID.SetMode(AUTOMATIC);

  Serial.begin(9600);
}
////////////////////////////////////////////////////////////////////////////////
//overLine: function determines when travelling straight, and how long for
bool overLine(){
  unsigned long startStraight = 0;
  bool straight = false;

  if (!straight && totalError<40){
    startStraight = millis();
    straight = true;
  }
  else if (totalError>40){
    straight = false;
  }
  Serial.println(straight);
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

  bool goingStraight = overLine();
  if(goingStraight){
    motorLeftSpeed = boostSpeed - motorChange;
    motorRightSpeed = boostSpeed + motorChange;
    sensorPID.SetTunings(0.3, 0, 0);
  }
  else{
    motorLeftSpeed = baseSpeed - motorChange;
    motorRightSpeed = baseSpeed + motorChange;
    sensorPID.SetTunings(0.7,0,0.06);
  }
  //Serial.println(motorLeftSpeed);
  analogWrite(motorLeftPin, motorLeftSpeed);
//  Serial.print(motorLeftSpeed);
//  Serial.print("   ");
//  Serial.println(motorRightSpeed);
  analogWrite(motorRightPin, motorRightSpeed);
 
}
////////////////////////////////////////////////////////////////////////////////
//loop: run repeatedly
void loop() {

  inductorLeftValue = analogRead(inductorLeftPin);
  distanceLeft = 131.7*exp(-0.002438*inductorLeftValue);
  
  inductorRightValue = analogRead(inductorRightPin);
  distanceRight = 133.4*exp(-0.00269*inductorRightValue);

//  Serial.print(inductorLeftValue);
//  Serial.print("   ");
//  Serial.println(inductorRightValue);

//  Serial.print(distanceLeft);
//  Serial.print("   ");
//  Serial.println(distanceRight);
  

  totalError = distanceLeft - distanceRight;
  //totalError = inductorLeftValue - inductorRightValue;
  //Serial.println(totalError);

  sensorPID.Compute();
  //Serial.println(motorChange);

  setMotorSpeeds();
}
