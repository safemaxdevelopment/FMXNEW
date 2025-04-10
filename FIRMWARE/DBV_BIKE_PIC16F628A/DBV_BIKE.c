#include <xc.h>

#define _XTAL_FREQ 4000000 // 4 MHz

//#pragma config FOSC = INTOSCIO  // Internal oscillator, I/O on RA6/OSC2/CLKOUT pin
#pragma config FOSC = HS  // Internal oscillator, I/O on RA6/OSC2/CLKOUT pin
#pragma config WDTE = OFF       // Watchdog Timer disabled
#pragma config PWRTE = OFF      // Power-up Timer disabled
#pragma config MCLRE = OFF      // MCLR pin function is digital input
#pragma config BOREN = OFF      // Brown-out Reset disabled
#pragma config LVP = OFF        // Low-Voltage Programming disabled
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection disabled
#pragma config CP = ON          // Flash Program Memory Code Protection disabled

#define BAUDRATE 9600
#define PULSE_LIMIT_COUNT 23

volatile unsigned char timer_flag = 0; // Flag to indicate timer interrupt
int overflow_count = 0;
int pulse_count = 0;
int numberOfOverflowsPerSecond;

//Apache bike Speed=79 PPR=10 cutoff=197
//TVS Pulsar Speed=46 PPR=2 cutoff=23
//First Drive bike cutoff=170
//cutt_off_pulse = (eeprom_speed_value*eeprom_ppr_value)/4;

int calculateTimer0OverflowsPerSecond(unsigned long systemClockFrequency, unsigned int prescaler) {
    // Timer0 operates at FOSC / 4
    unsigned long timer0ClockFrequency = systemClockFrequency / 4;
    
    // Apply the prescaler
    unsigned long effectiveTimer0Frequency = timer0ClockFrequency / prescaler;
    
    // Timer0 is an 8-bit timer, so it counts from 0 to 255 (256 counts)
    int numberOfOverflowsPerSecond = effectiveTimer0Frequency / 256;
    
    return numberOfOverflowsPerSecond;
}


void main(void) {
    int pulse_copy_count = 0;


    // unsigned int deviceID;// = READ_DEVICE_ID();
     //deviceID = *(unsigned char *)0x2006;
     //deviceID = (unsigned int)IDLOC;
    
    numberOfOverflowsPerSecond=calculateTimer0OverflowsPerSecond(_XTAL_FREQ,2);
    
    
    // Main loop to toggle LEDs
    while (1) {

        if (timer_flag) { // One second elapsed if this flag is true
            timer_flag = 0;
            
            pulse_copy_count = pulse_count;
            pulse_count = 0;

            if (pulse_copy_count == 0) {
                PORTBbits.RB5 = 0;
            } else {
                PORTBbits.RB5 = !PORTBbits.RB5;
            }
            
            // Control RA0 (speed limit not reached LED)
            if (pulse_copy_count < PULSE_LIMIT_COUNT) {
                PORTAbits.RA0 = 1; // Turn on RA0 if pulse count is below the limit
                PORTBbits.RB6 = 0; // Turn off RB6
            } else {
                PORTAbits.RA0 = 0; // Turn off RA0 if pulse count is 50 or above
                PORTBbits.RB6 = 1; // Turn on RB6
            }
        }
    }
}




