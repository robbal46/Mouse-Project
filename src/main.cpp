#include <Arduino.h>

#include "PID.h"

PID PIDmotor1(analogRead(A1), 0.001, 0.1,0.1,0.1);
double motor1Speed = 512;
void setup() {

}

void loop() {
  double error1 = PIDmotor1.computePID();
  analogWrite(A2, motor1Speed + error1);
}
