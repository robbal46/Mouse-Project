//////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////
//Includes and libraries

#include <PID_v1.h>

#define motorLPin 5
#define motorRPin 6
#define brakeLPin 8
#define brakeRpin 9

#define inductorLPin A0
#define inductorRPin A1
#define inductorFPin A2

///////////////////////////////////////////////////////////////////////
//PID setup
int baseSpeed = 70;
int slowSpeed = 55;
const int boostSpeed = 200;

//Setpoint for inductor voltage
double setpoint;
//Result of adding both inductor inputs
double totalError;

//Output of PID calculation, added/subtracted from motor speed
double motorChange;

//Varibles to store values from inductor
int inductorLValue;
int inductorRValue;
int inductorFValue;

double distanceError;

bool straight;
bool speedUp;
 
double kp = 12;
double ki = 0;
double kd = 0.5;

//Create PID object   (input, setpoint, kp, ki, kd, sampleTime)
PID sensorPID(&distanceError, &motorChange, &setpoint, kp, ki, kd, DIRECT);

//////////////////////////////////////////////////////////////////////
//Setup: run once
void setup() {

  //Set pimodes to input/output
  pinMode(inductorLPin, INPUT);
  pinMode(inductorRPin, INPUT);
  pinMode(motorLPin, OUTPUT);
  pinMode(motorRPin, OUTPUT);

  //Initialise the inductor value variables
  inductorLValue = analogRead(inductorLPin);
  inductorRValue = analogRead(inductorRPin);


  totalError = inductorLValue - inductorRValue;

  distanceError = (-5.065e-8*pow(totalError,3)) + (-2.839e-07*pow(totalError,2)) + (0.09438*totalError) + 0.2026;

  straight = false;
  speedUp = false;

  //Initialise setpoint variable as current position error
  //Position the inductors equidistant from the wire, and turn arduino power on
  setpoint = distanceError;

  sensorPID.SetOutputLimits((-1 * baseSpeed), baseSpeed);

  sensorPID.SetMode(AUTOMATIC);

  sensorPID.SetSampleTime(1);

}
//////////////////////////////////////////////////////////////////////
//overLine: function determines when travelling straight, and how long for
bool overLine(){
  unsigned int startStraight;
  unsigned int straightTime;

  if (!straight && inductorFValue >= 550){
    startStraight = millis();
    straight = true;
  }
  else if (inductorFValue <= 550){
    straight = false;
  }

  if (straight == true){
     straightTime = millis() - startStraight;
  }
  else{
    straightTime = 0;
  }
  
  if (straightTime >= 500){
    return true;
  }
  else {
    return false;
  }
}
//////////////////////////////////////////////////////////////////////
//brakeMotor: brakes the motors by turning on the P MOSFET
void brakeMotor(){
    digtialWrite(motorLPin, LOW);
    digtialWrite(motorLPin, LOW);
    delay(2);
    digtitalWrite(brakeRPin, HIGH);
    digitalWrite(brakeLPin, HIGH);
}
//////////////////////////////////////////////////////////////////////
//setMotorSpeeds: sets the speed of both left/right motors
void setMotorSpeeds(){
  int motorLSpeed;
  int motorRSpeed; 

  if (overLine()){
    speedUp = true;
  }
  
  if (speedUp){
    sensorPID.SetSampleTime(10);
  }
  else{
    sensorPID.SetSampleTime(1);
  }
  
  if(speedUp && millis() > 2000){
    motorLSpeed = boostSpeed - motorChange;
    motorRSpeed = boostSpeed + motorChange;
    sensorPID.SetTunings(1.5,0,0);
    sensorPID.SetOutputLimits((-1 * boostSpeed),boostSpeed);
  }  
  else if (inductorFValue <= 180 && distanceError < 0){
    brakeMotor();
    brake = true;
  }
  else{
    motorLSpeed = baseSpeed - motorChange;
    motorRSpeed = baseSpeed + motorChange;
    sensorPID.SetTunings(kp,ki,kd);
    sensorPID.SetOutputLimits((-1*baseSpeed), baseSpeed);
  }
  if(!brake){
  digitalWrite(brakeLPin, LOW);
  digitalWrite(brakeRPin, LOW);
  
  analogWrite(motorLPin, motorLSpeed);
  analogWrite(motorRPin, motorRSpeed);
  }
}
//////////////////////////////////////////////////////////////////////
//loop: run repeatedly
void loop() {
    inductorFValue = analogRead(inductorFPin);
    inductorLValue = analogRead(inductorLPin);  
    inductorRValue = analogRead(inductorRPin);


    totalError = inductorLValue - inductorRValue;
    
    distanceError = (-5.065e-8*pow(totalError,3)) + (-2.839e-07*pow(totalError,2)) + (0.09438*totalError) + 0.2026;
    
    sensorPID.Compute();

    setMotorSpeeds();
}
