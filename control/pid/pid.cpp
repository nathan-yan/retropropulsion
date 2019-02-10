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

        float step(float measurement, float derivative);
};

void PIDController::start(){
    this->time = millis();      // Once you begin set the baseline time so the first iteration doesn't freak the integral term out
}

void PIDController::reset(){
    previous_error = 0;
    previous_integral = 0;
    previous_d_error = 0;
}

float PIDController::step(float measurement, float d_error){
    int time = millis();
    
    float dt = (time - this->time) / 1000.;
    this->time = time;

    float error = measurement - setpoint;
    
    // d_error is in degrees per second.
    //float d_error = ((error - previous_error) / dt) * 0.8 + previous_d_error * 0.2;     // A weighted average between current derivative and previous derivative to prevent sudden changes jerking the mount

    //float d_error = derivative;       // Degrees per second

    // Integrate error over dt
    previous_integral += error * dt;

    float proportional = P * error;
    float integral = I * previous_integral; 
    float derivative = D * d_error;

    //if (derivative < 1.5 && derivative > -1.5){
    //    derivative = 0;
    //}

    previous_error = error;
    previous_d_error = d_error;

    return proportional + integral + derivative;
}

/*
float* pidLoop(float p, float i, float d,
               float measurement, float setpoint,
               float previous_error, float previous_integral, float previous_d_error,
               float dt){
    
    float error = measurement - setpoint;
    float d_error = (((error - previous_error) / dt) * 0.8 + previous_d_error * 0.2);

     //float d_error = (((error - previous_error) / dt));

    // Integrate the error over the time difference
    previous_integral += error * dt;

    float proportional = p * error;
    float integral = i * previous_integral;
    float derivative = d * d_error;

    if (derivative < 1.5 && derivative > -1.5){
        derivative = 0;
    }

    // if (proportional + integral + derivative > 6 || proportional)

    float * state = new float[4]; 
    state[0] = proportional + integral + derivative;
    state[1] = error;
    state[2] = previous_integral;
    state[3] = d_error;

    return state;
}*/