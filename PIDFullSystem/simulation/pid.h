/************************************
* pid.h - Generic PID controller *
************************************/
#ifndef PID_H
#define PID_H

typedef struct {
    float Kp, Ki, Kd;     // PID gains
    float integral;       // Integral accumulator
    float prev_error;     // Previous error for derivative
    float setpoint;       // Desired target value
    float output_min;     // Minimum output limit
    float output_max;     // Maximum output limit
} PIDController;

void pid_init(PIDController* pid, float Kp, float Ki, float Kd, float setpoint, float min, float max);
float pid_update(PIDController* pid, float measurement, float dt);

#endif