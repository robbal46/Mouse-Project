#ifndef PID_h
#define PID_h

//PID class
class PID{

  //Functions accesible by the user
  public:

    //PID object
    PID(double, double, double, double, double);

    //Functions used in PID calculations
    double computePID();

  //Variables accesible by class functions
  private:

    //PID constants
    double kp;
    double ki;
    double kd;

    //Input and setpoint variables for calculations
    double input;
    double setpoint;

    //Time variables
    unsigned long sampleTime;
    unsigned long lastTime;
    unsigned long currentTime;
    unsigned long timeChange;

    //Error variables used for calculation
    double error;
    double lastError;
    double errIntegral;
    double errDeriv;

};
#endif
