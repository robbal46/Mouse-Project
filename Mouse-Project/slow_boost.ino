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

#define motorLeftPin 10
#define motorRightPin 11

#define inductorLeftPin A0
#define inductorRightPin A1

////////////////////////////////////////////////////////////////////////////////
//PID setup
int baseSpeed;
const int boostSpeed = 200;

//Setpoint for inductor voltage
double setpoint;
//Result of adding both inductor inputs
double totalError;

//Output of PID calculation, added/subtracted from motor speed
double motorChange;

//Varibles to store values from inductor
int inductorLeftValue;
int inductorRightValue;

double distanceError;

bool straight;

//Create PID object   (input, setpoint, kp, ki, kd, sampleTime)
PID sensorPID(&distanceError, &motorChange, &setpoint, 2.7
, 0, 0.3, DIRECT);

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

//  distanceLeft = 1067*exp(-0.02772*inductorLeftValue);
//  distanceRight = 1108*exp(-0.02705*inductorRightValue);

  //totalError = inductorLeftValue - inductorRightValue;
  totalError = inductorLeftValue - inductorRightValue;
  distanceError = (0.05244*totalError) + (3.013);

  straight = false;

  //Initialise setpoint variable as current position error
  //Position the inductors equidistant from the wire, and turn arduino power on
  setpoint = distanceError;

  sensorPID.SetOutputLimits(-255, 255);

  sensorPID.SetMode(AUTOMATIC);

  sensorPID.SetSampleTime(10);

  Serial.begin(9600);
}
////////////////////////////////////////////////////////////////////////////////
//overLine: function determines when travelling straight, and how long for
//bool overLine(){
//  unsigned long startStraight;
//  unsigned long straightTime;
//
//  if (!straight && abs(totalError)<=10){
//    startStraight = millis();
//    straight = true;
//  }
//  else if (abs(totalError)>10){
//    //straightTime = 0;
//    straight = false;
//  }
//  else{
//  }
//
//  if (straight == true){
//     straightTime = millis() - startStraight;
//  }
//  else{
//    straightTime = 0;
//  }
//  //.print(straightTime);
//  //Serial.print("   ");
//  
//  if (straightTime >= 4000 && straight){
//    return true;
//  }
//  else {
//    return false;
//  }
//}
////////////////////////////////////////////////////////////////////////////////
//setMotorSpeeds: sets the speed of both left/right motors
void setMotorSpeeds(){
  int motorLeftSpeed;
  int motorRightSpeed;
  
  baseSpeed = map(abs(distanceError), 0, 30, 70, 20);
  //Serial.println(baseSpeed); 
 // bool onStraight = overLine();
  if(false){
    motorLeftSpeed = boostSpeed - motorChange;
    motorRightSpeed = boostSpeed + motorChange;
    //sensorPID.SetTunings(0.3,0,0);
  }
  else{
    motorLeftSpeed = baseSpeed - motorChange;
    motorRightSpeed = baseSpeed + motorChange;
    //sensorPID.SetTunings(1,0,0.1);
  }

  //Serial.println(motorLeftSpeed);
  analogWrite(motorLeftPin, motorLeftSpeed);
//  Serial.print(motorLeftSpeed);
//  Serial.print("   ");
//  Serial.println(motorRightSpeed);
  analogWrite(motorRightPin, motorRightSpeed);
  //Serial.println(onStraight);
 
 
}
////////////////////////////////////////////////////////////////////////////////
//loop: run repeatedly
void loop() {

  inductorLeftValue = analogRead(inductorLeftPin);  
  inductorRightValue = analogRead(inductorRightPin);

  
//
//Serial.print(inductorLeftValue);
//Serial.print("   ");
//Serial.println(inductorRightValue);

//  Serial.print(distanceLeft);
//  Serial.print("   ");
//  Serial.println(distanceRight);
  
  //totalError = distanceLeft - distanceRight;
  totalError = inductorLeftValue - inductorRightValue;
  distanceError = (0.05244*totalError) + (3.013);
  
  //constrain(totalError, -60, 70);
  //totalError = inductorLeftValue - inductorRightValue;
  Serial.println(distanceError);

  sensorPID.Compute();
  //Serial.println(motorChange);

  setMotorSpeeds();
}
