/*******************************
* pwm.c - PWM Implementation *
*******************************/
#include "pwm.h"

void pwm_init(PWM* pwm) {
    pwm->duty_cycle = 0.0f;
    pwm->frequency = 1000.0f;  // 1 kHz typical for motor control
    pwm->resolution = 100.0f;  // 100 steps per period
}

void pwm_set_duty(PWM* pwm, float duty) {
    // Constrain between 0-100
    pwm->duty_cycle = (duty < 0.0f) ? 0.0f : (duty > 100.0f) ? 100.0f : duty;
}

float pwm_get_effective_voltage(const PWM* pwm, float supply_voltage) {
    // Simple model: average voltage = duty cycle * supply voltage
    return (pwm->duty_cycle / 100.0f) * supply_voltage;
}