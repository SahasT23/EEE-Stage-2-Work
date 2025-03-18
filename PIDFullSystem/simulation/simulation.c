#include "simulation.h"
#include <stdlib.h>

// Initialize buggy at track center
void init_simulation(BuggyState* buggy) {
    buggy->position = TRACK_CENTER;
    buggy->speed = 0.0f;
    buggy->heading = 0.0f;
    buggy->pwm_duty = 0.0f;
    buggy->L_inductance = calculate_inductance(-SENSOR_SPACING/2);
    buggy->R_inductance = calculate_inductance(SENSOR_SPACING/2);
}

// Update buggy physics and sensors
void update_buggy(BuggyState* buggy, float steering, float dt) {
    // Constrain inputs
    steering = fmaxf(-45.0f, fminf(45.0f, steering));
    
    // Vehicle dynamics
    float turn_radius = WHEELBASE / tanf(steering * M_PI/180.0f);
    buggy->heading += (buggy->speed / turn_radius) * dt;
    
    // Update position
    buggy->position += buggy->speed * sinf(buggy->heading * M_PI/180.0f) * dt;
    
    // Update sensor readings
    float L_pos = buggy->position - SENSOR_SPACING/2;
    float R_pos = buggy->position + SENSOR_SPACING/2;
    buggy->L_inductance = calculate_inductance(L_pos - TRACK_CENTER);
    buggy->R_inductance = calculate_inductance(R_pos - TRACK_CENTER);
}

// Calculate inductance based on distance from track center
float calculate_inductance(float distance_from_track) {
    // Model inductive coupling with inverse-square law + noise
    float distance = fabsf(distance_from_track);
    float base = MAX_INDUCTANCE / (1.0f + distance*distance);
    
    // Add 5% random noise
    return base * (1.0f + ((rand() % 100)/500.0f - 0.1f));
}

// PID implementation
void PID_Init(PIDController* pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Update(PIDController* pid, float error, float dt) {
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    return pid->Kp * error + 
           pid->Ki * pid->integral + 
           pid->Kd * derivative;
}