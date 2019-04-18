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

#define motorLPin 5
#define motorRPin 6
#define brakeLPin 8
#define brakeRPin 9

#define inductorLPin A0
#define inductorRPin A1
#define inductorFPin A2

////////////////////////////////////////////////////////////////////////////////
//PID setup
int baseSpeed = 70;
const int boostSpeed = 220;
const int startSpeed = 150;
const int slowSpeed = 0;

//Setpoint for inductor voltage
double setpoint;
//Result of adding both inductor inputs
double totalError;

//Output of PID calculation, added/subtracted from motor speed
double motorChange;

bool brake;

//Varibles to store values from inductor
int inductorLValue;
int inductorRValue;
int inductorFValue;

double distanceError;
 
double kp = 12;
double ki = 0;
double kd = 0.5;

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
  pinMode(brakeLPin, OUTPUT);
  pinMode(brakeRPin, OUTPUT);
  pinMode(7,INPUT_PULLUP);

  //Initialise the inductor value variables
  inductorLValue = analogRead(inductorLPin);
  inductorRValue = analogRead(inductorRPin);

  //totalError = inductorLeftValue - inductorRightValue;
  totalError = inductorLValue - inductorRValue;
  distanceError = (0.05244*totalError) + (3.013);
  //distanceError = (-5.065e-8*pow(totalError,3)) + (-2.839e-07*pow(totalError,2)) + (0.09438*totalError) + 0.2026;

  //Initialise setpoint variable as current position error
  //Position the inductors equidistant from the wire, and turn arduino power on
  setpoint = distanceError+3;
  brake = false;

  sensorPID.SetOutputLimits((-1 * baseSpeed), baseSpeed);

  sensorPID.SetMode(AUTOMATIC);

  sensorPID.SetSampleTime(1);

  //Serial.begin(9600);
}
////////////////////////////////////////////////////////////////////////////////
//brake: brakes the motors
void brakeMotor(){
  digitalWrite(motorRPin, LOW);
  digitalWrite(motorLPin, LOW);
  delay(2);
  digitalWrite(brakeRPin, HIGH);
  digitalWrite(brakeLPin, HIGH);
}
////////////////////////////////////////////////////////////////////////////////
//setMotorSpeeds: sets the speed of both left/right motors
void setMotorSpeeds(){
  int motorLSpeed;
  int motorRSpeed; 

  int now = millis();
  brake = false;


  if(now>9400 && now<=9500 || now>4700 && now<4800){
    brakeMotor();
    brake = true;
    
  }
  else if(now > 7000 && now <= 9450){
    motorLSpeed = boostSpeed - motorChange;
    motorRSpeed = boostSpeed + motorChange;
    sensorPID.SetTunings(0.9,0,0);
    sensorPID.SetOutputLimits((-1 * boostSpeed),boostSpeed);
  }  

  else if (inductorFValue <= 350 && distanceError < 0 && ((now > 4250 && now < 5500) || now>8500)){
    sensorPID.SetOutputLimits((-1*boostSpeed), boostSpeed);
    //sensorPID.SetTunings(12,0,0.5);
    motorRSpeed = boostSpeed + motorChange;
    motorLSpeed = 0;
  } 
  else{
    motorLSpeed = baseSpeed - motorChange;
    motorRSpeed = baseSpeed + motorChange;
    sensorPID.SetOutputLimits((-1*baseSpeed), baseSpeed);
  }
  
  if (now<300){
    motorLSpeed = startSpeed - motorChange;
    motorRSpeed = startSpeed + motorChange;
    sensorPID.SetOutputLimits((-1 * startSpeed),startSpeed);
  }
  if(!brake){
  digitalWrite(brakeLPin, LOW);
  digitalWrite(brakeRPin, LOW);
  delay(2);
  analogWrite(motorLPin, motorLSpeed);
  analogWrite(motorRPin, motorRSpeed);

  constrain(motorLSpeed,0,255);
  constrain(motorRSpeed,0,255);
  }
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
 //  Serial.println(inductorFValue);

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
