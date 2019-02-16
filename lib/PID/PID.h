#ifndef PID_h
#define PID_h

//PID class
class PID{

  //Functions accesible by the user
  public:

    //PID object
    PID(int, int, float, float, float);

    //Functions used in PID calculations
    double computePID();

  //Variables accesible by class functions
  private:

    //PID constants
    float kp;
    float ki;
    float kd;

    //Input and setpoint variables for calculations
    int input;
    int setpoint;

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
