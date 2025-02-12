#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// PID constants
#define KP 1.0
#define KI 0.1
#define KD 0.01

// Motor control constants
#define MOTOR_ZERO 737
#define MOTOR_RANGE 500

// Global variables for PID control
float integral = 0.0;
float previous_error = 0.0;

// Function to compute PID control output
float compute_pid(float target, float current) {
    float error = target - current;

    // Proportional term
    float proportional = KP * error;

    // Integral term
    integral += error;
    float integral_term = KI * integral;

    // Derivative term
    float derivative = (error - previous_error);
    float derivative_term = KD * derivative;

    // Update previous error
    previous_error = error;

    // PID output
    return proportional + integral_term + derivative_term;
}

int main() {
    float target_frequency = 0.0; // Start with 0Hz
    float current_frequency;

    while (1) {
        // Alternate between 0Hz and 2Hz
        target_frequency = (target_frequency == 0.0) ? 2.0 : 0.0;

        // Read frequency from stdin
        if (scanf("%f", &current_frequency) != 1) {
            fprintf(stderr, "Error reading frequency from stdin\n");
            break;
        }

        // Log frequency to file
        FILE *log_file = fopen("step_response.log", "a");
        if (log_file) {
            fprintf(log_file, "%.2f\n", current_frequency);
            fclose(log_file);
        }

        // Compute PID output and control motor
        float pid_output = compute_pid(target_frequency, current_frequency);
        int motor_input = MOTOR_ZERO + (int)(pid_output * MOTOR_RANGE);

        // Clamp motor input
        if (motor_input < MOTOR_ZERO - MOTOR_RANGE) motor_input = MOTOR_ZERO - MOTOR_RANGE;
        if (motor_input > MOTOR_ZERO + MOTOR_RANGE) motor_input = MOTOR_ZERO + MOTOR_RANGE;

        printf("Motor input: %d\n", motor_input);
        fflush(stdout);

        usleep(20000); // Sleep for 20ms
    }

    return 0;
}