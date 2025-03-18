#ifndef SIMULATION_H
#define SIMULATION_H

#include <math.h>

// Copper track properties
#define TRACK_WIDTH      10.0f    // mm
#define TRACK_CENTER     50.0f    // mm (center of simulation area)
#define MAX_INDUCTANCE   100.0f   // Maximum sensor reading (μH)

// Vehicle physical properties
#define SENSOR_SPACING   20.0f    // mm between sensors
#define WHEELBASE        50.0f    // mm

typedef struct {
    // Vehicle state
    float position;      // Lateral position (mm)
    float speed;         // mm/s
    float heading;       // Degrees from track center
    
    // Sensor readings
    float L_inductance;
    float R_inductance;
    
    // Motor control
    float pwm_duty;      // 0-100%
} BuggyState;

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
} PIDController;

// Simulation functions
void init_simulation(BuggyState* buggy);
void update_buggy(BuggyState* buggy, float steering, float dt);

// PID functions
void PID_Init(PIDController* pid, float Kp, float Ki, float Kd);
float PID_Update(PIDController* pid, float error, float dt);

// Inductive sensor model
float calculate_inductance(float distance_from_track);

#endif