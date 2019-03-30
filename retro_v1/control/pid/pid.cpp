/*
Copyright:  MIT License
Author:     Nathan Yan
Filename:   pid.cpp
Purpose:    A PID controller for deciding the yaw and pitch of the gimbal based 
            on the orientation of the rocket.
*/

// TODO: Test that updated PID controller works the same as old one.
#include <Arduino.h>

class PIDController{
    public:
        // Parameters
        float P;
        float I;
        float D;

        // State
        float setpoint;
        float previous_error = 0;
        float previous_integral = 0; 
        float previous_d_error = 0;

        int time;

        PIDController(float P, float I, float D, float setpoint){
            this->P = P;
            this->I = I;
            this->D = D; 

            this->setpoint = setpoint; 
        }

        void start();
        void reset();

        void changeSetpoint(float setpoint){
            this->setpoint = setpoint;
        }

        void setParameters(float P, float I, float D){
            this->P = P;
            this->I = I;
            this->D = D;
        }

        float step(float measurement, float i_error, float d_error);
};

void PIDController::start(){
    this->time = millis();      // Once you begin set the baseline time so the first iteration doesn't freak the integral term out
}

void PIDController::reset(){
    previous_error = 0;
    previous_integral = 0;
    previous_d_error = 0;
}

float PIDController::step(float measurement, float i_error, float d_error){
    int time = millis();
    
    float dt = (time - this->time) / 1000.;
    this->time = time;

    float error = measurement - setpoint;
    
    float proportional = P * error;
    float integral = I * i_error; 
    float derivative = D * d_error;

    return proportional + integral + derivative;
}
