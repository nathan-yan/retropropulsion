#ifndef _PID_H
#define _PID_H

class PIDController{
    public:
        float P;
        float I;
        float D;

        float setpoint;
        float previous_error;
        float previous_integral;
        float previous_d_error;

        int time;

        PIDController(float P, float I, float D, float setpoint);

        void start();
        void reset();

        void changeSetpoint(float setpoint);

        float step(float measurement);
}

#endif