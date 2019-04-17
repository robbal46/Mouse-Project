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

#define motorLPin 10
#define motorRPin 11

#define inductorLPin A0
#define inductorRPin A1
#define inductorFPin A2

////////////////////////////////////////////////////////////////////////////////
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
 
double kp = 10;
double ki = 5;
double kd = 1;

//Create PID object   (input, setpoint, kp, ki, kd, sampleTime)
PID sensorPID(&distanceError, &motorChange, &setpoint, kp, ki, kd, DIRECT);

////////////////////////////////////////////////////////////////////////////////
//Setup: run once
void setup() {

  //Set pimodes to input/output
  pinMode(inductorLPin, INPUT);
  pinMode(inductorRPin, INPUT);
  pinMode(motorLPin, OUTPUT);
  pinMode(motorRPin, OUTPUT);
  pinMode(7,INPUT_PULLUP);

  //Initialise the inductor value variables
  inductorLValue = analogRead(inductorLPin);
  inductorRValue = analogRead(inductorRPin);

  //totalError = inductorLeftValue - inductorRightValue;
  totalError = inductorLValue - inductorRValue;
  distanceError = (0.05244*totalError) + (3.013);
  //distanceError = (-5.065e-8*pow(totalError,3)) + (-2.839e-07*pow(totalError,2)) + (0.09438*totalError) + 0.2026;

  straight = false;
  speedUp = false;

  //Initialise setpoint variable as current position error
  //Position the inductors equidistant from the wire, and turn arduino power on
  setpoint = distanceError;

  sensorPID.SetOutputLimits((-1 * baseSpeed), baseSpeed);

  sensorPID.SetMode(AUTOMATIC);

  sensorPID.SetSampleTime(1);

  //Serial.begin(9600);
}
////////////////////////////////////////////////////////////////////////////////
//overLine: function determines when travelling straight, and how long for
bool overLine(){
  unsigned int startStraight;
  unsigned int straightTime;

  if (!straight && inductorFValue >= 400){
    startStraight = millis();
    straight = true;
  }
  else if (inductorFValue <= 400){
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
////////////////////////////////////////////////////////////////////////////////
//setMotorSpeeds: sets the speed of both left/right motors
void setMotorSpeeds(){
  int motorLSpeed;
  int motorRSpeed; 
  
  //baseSpeed = map(abs(distanceError), 0, 40, 90, 30);
  //Serial.println(baseSpeed); 

  if (overLine()){
    speedUp = true;
  }

  bool ramp = digitalRead(7);
  if (!ramp){
    speedUp = false;
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
  else if (inductorFValue <= 180 && distanceError < 0 && millis() > 4600 && millis() < 8600){
    motorLSpeed = 0;//slowSpeed - motorChange;
    motorRSpeed = baseSpeed + motorChange;
    sensorPID.SetOutputLimits((-1*baseSpeed), baseSpeed);
    sensorPID.SetTunings(kp,ki,kd);
  }
  else if (inductorFValue <= 180 && distanceError > 0 && millis() > 4600 && millis() < 8600){
    motorLSpeed = baseSpeed - motorChange;
    motorRSpeed = 0;//slowSpeed + motorChange;
    sensorPID.SetOutputLimits((-1*baseSpeed), baseSpeed);
    sensorPID.SetTunings(kp,ki,kd);
  } 
  else{
    motorLSpeed = baseSpeed - motorChange;
    motorRSpeed = baseSpeed + motorChange;
    sensorPID.SetTunings(kp,ki,kd);
    sensorPID.SetOutputLimits((-1*baseSpeed), baseSpeed);
  }

  analogWrite(motorLPin, motorLSpeed);
  analogWrite(motorRPin, motorRSpeed);

//  Serial.print(motorLSpeed);
//  Serial.print("   ");
//  Serial.println(motorRSpeed);

 
 
}
////////////////////////////////////////////////////////////////////////////////
//loop: run repeatedly
void loop() {

    
      inductorFValue = analogRead(inductorFPin);
    
   
    inductorLValue = analogRead(inductorLPin);  
    inductorRValue = analogRead(inductorRPin);
    

//    Serial.print(inductorLValue);
//    Serial.print("   ");
//    Serial.print(inductorRValue);
//    Serial.print("   ");
//   Serial.println(inductorFValue);

    totalError = inductorLValue - inductorRValue;
    distanceError = (0.05244*totalError) + (3.013);
    //distanceError = (-5.065e-8*pow(totalError,3)) + (-2.839e-07*pow(totalError,2)) + (0.09438*totalError) + 0.2026;
    //Serial.println(distanceError);
  
    //totalError = inductorLValue - inductorRValue;
    //Serial.println(totalError);
    //lastRead = millis();
  

    sensorPID.Compute();
    //Serial.println(motorChange);

    setMotorSpeeds();
  
}
