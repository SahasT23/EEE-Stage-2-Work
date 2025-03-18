/*************************************
* pid.c - PID implementation *
*************************************/
#include "pid.h"

void pid_init(PIDController* pid, float Kp, float Ki, float Kd, 
             float setpoint, float min, float max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = min;
    pid->output_max = max;
}

float pid_update(PIDController* pid, float measurement, float dt) {
    float error = pid->setpoint - measurement;
    
    // Proportional term
    float P = pid->Kp * error;
    
    // Integral term with anti-windup
    pid->integral += error * dt;
    pid->integral = fmaxf(pid->output_min, fminf(pid->output_max, pid->integral));
    float I = pid->Ki * pid->integral;
    
    // Derivative term
    float D = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // Calculate total output
    float output = P + I + D;
    
    // Constrain output
    return fmaxf(pid->output_min, fminf(pid->output_max, output));
}