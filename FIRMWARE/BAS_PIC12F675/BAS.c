/*
 * File:   main.c
 * Author: user
 *
 * Created on December 20, 2023, 2:52 PM
 */

//12F675

#include <htc.h>
#include <xc.h>
#include <stdint.h>

#define LED GP2

#define _XTAL_FREQ 4000000
#define  DELAY 160


// PIC16F684 Configuration Bit Settings

// 'C' source line config statements

// CONFIG
#pragma config FOSC = INTRCIO  
//#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Detect (BOR enabled)



void main(void) {

     uint16_t i, j;
    
    
    // Blink the LED three times
    for (j = 0; j < 3; j++) {
        
        LED=1;
        __delay_ms (DELAY);
        LED=0;
        __delay_ms (DELAY);
        
    }

    // Keep the LED continuously lit
    LED=1;

    while (1) {
        // Your main code goes here
        // This loop will keep the LED continuously lit
    }

    return;
}
