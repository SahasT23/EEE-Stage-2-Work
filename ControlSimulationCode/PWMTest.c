#include <xc.h>
#include <stdint.h>

// Configuration Bits (32 MHz internal clock)
#pragma config FEXTOSC = OFF, RSTOSC = HFINT32, CLKOUTEN = OFF, CSWEN = ON
#pragma config MCLRE = ON, PWRTE = OFF, LPBOREN = OFF, BOREN = ON
#pragma config BORV = LOW, ZCDDIS = ON, PPS1WAY = ON, STVREN = ON
#pragma config WDTCPS = WDTCPS_31, WDTE = OFF, WDTCWS = WDTCWS_7, WDTCCS = SC
#pragma config WRT = OFF, SCANE = ON, LVP = ON, CP = OFF, CPD = OFF

#define PWM_PIN RC5         // PWM output for motor control
#define SCOPE_TRIG RC4      // Oscilloscope trigger pin
#define PWM_MAX 255
#define PWM_MIN 0

volatile uint8_t pwm_duty = 128;   // Initial 50% duty cycle
volatile uint8_t adc_value = 0;    // For future ADC integration

void setup(void);
void setup_pwm(void);
void setup_timer0(void);
void setup_trigger(void);

void main(void) {
    setup();
    while(1) {
        // Main loop can be used for:
        // - ADC conversions
        // - Serial communication
        // - Duty cycle calculations
    }
}

void setup(void) {
    setup_pwm();
    setup_timer0();
    setup_trigger();
    INTCONbits.GIE = 1;     // Enable global interrupts
}

void setup_pwm(void) {
    TRISCbits.TRISC5 = 0;   // PWM output (RC5)
    ANSELCbits.ANSC5 = 0;   // Digital output
    
    // Configure PWM module (31.25 kHz @ 32MHz)
    PR2 = 0xFF;             // PWM period register
    CCP1CONbits.CCP1M = 0b1100;  // PWM mode
    T2CONbits.T2CKPS = 0b00;     // Timer2 prescaler (1:1)
    T2CONbits.TMR2ON = 1;        // Enable Timer2
    
    // Set initial duty cycle
    CCPR1L = pwm_duty >> 2;
    CCP1CONbits.DC1B = pwm_duty & 0x03;
}

void setup_timer0(void) {
    // Timer0 interrupt for PWM updates (1 kHz)
    T0CON0bits.T016BIT = 0;        // 8-bit mode
    T0CON1bits.T0CS = 0b010;       // Clock source: Fosc/4 (8 MHz)
    T0CON1bits.T0CKPS = 0b111;     // 1:256 prescaler
    TMR0L = 0;                     // Start from 0
    INTCONbits.TMR0IE = 1;         // Enable Timer0 interrupt
    T0CON0bits.T0EN = 1;           // Enable Timer0
}

void setup_trigger(void) {
    TRISCbits.TRISC4 = 0;   // Trigger output (RC4)
    ANSELCbits.ANSC4 = 0;   // Digital output
    LATCbits.LATC4 = 0;     // Initial low
}

void __interrupt() isr(void) {
    static uint8_t counter = 0;
    
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;    // Clear interrupt flag
        TMR0L = 0;                // Reset timer
        
        // Oscilloscope trigger pulse
        LATCbits.LATC4 = 1;       // Trigger high
        LATCbits.LATC4 = 0;       // Trigger low (1 cycle pulse)
        
        // Example duty cycle modulation (Ramp waveform)
        pwm_duty++;
        if(pwm_duty >= PWM_MAX) pwm_duty = PWM_MIN;
        
        // Update PWM registers
        CCPR1L = pwm_duty >> 2;
        CCP1CONbits.DC1B = pwm_duty & 0x03;
    }
}
