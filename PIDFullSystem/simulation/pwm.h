/*******************************
* pwm.h - PWM Simulation *
*******************************/
#ifndef PWM_H
#define PWM_H

typedef struct {
    float duty_cycle;    // Current duty cycle (0-100%)
    float frequency;     // PWM frequency (Hz)
    float resolution;    // Steps per period (simulation detail)
} PWM;

// Initialize PWM with default values
void pwm_init(PWM* pwm);

// Set duty cycle (clamped to 0-100)
void pwm_set_duty(PWM* pwm, float duty);

// Get current effective voltage (simplified model)
float pwm_get_effective_voltage(const PWM* pwm, float supply_voltage);

#endif