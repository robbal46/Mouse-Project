#ifndef PID_h
#define PID_h

class PID{

  public:

    //PID object
    PID(double, double, double, double, double);

    //Functions used in PID calculations
    double computePID();

  private:

    double kp;
    double ki;
    double kd;

    double input;
    double setpoint;

    unsigned long sampleTime;
    unsigned long lastTime;
    unsigned long currentTime;
    unsigned long timeChange;

    double error;
    double lastError;

    double errIntegral;
    double errDeriv;


};
#endif
