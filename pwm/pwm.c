#include <XC.h>

// CONFIGURATION BITS
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Secondary Oscillator Disabled

// Defines
#define SYSCLK 40000000L
#define PWM_FREQ    500L      // 500 Hz PWM frequency (not 50kHz; adjust if needed)
#define DUTY_CYCLE  50

// Simple delay function (crude loop delay, adjust as needed)
void delay_ms(unsigned int ms) {
    unsigned int i, j;
    for(i = 0; i < ms; i++) {
         for(j = 0; j < 10000; j++) { }
    }
}

/*
   Pinout note for DIP28 PIC32MX130:
   To use OC1 on pin RB3 (pin 7), we use Peripheral Pin Select:
   RB3Rbits.RB3R must be set to 0x0005 to map OC1.
*/

void Init_pwm(void)
{
    // Map OC1 to RB3 (pin 7)
    RPB3Rbits.RPB3R = 0x0005;
 
    // Configure standard PWM mode for Output Compare Module 1
    OC1CON = 0x0006; 

    // Set Timer2 pre-scaler to 1
    T2CONbits.TCKPS = 0x0;
    
    // Set PWM period: PR = (SYSCLK / (PWM_FREQ * TMR Prescale)) - 1
    PR2 = (SYSCLK / (PWM_FREQ * 1)) - 1;
    
    // Set initial duty cycle (50%)
    OC1RS = (PR2 + 1) * ((float)DUTY_CYCLE / 100.0);
    
    // Clear and start Timer2
    T2CON = 0x0;
    T2CONSET = 0x8000;      // Enable Timer2
    
    // Enable OC1 module
    OC1CONSET = 0x8000;
}

void Set_pwm(unsigned char val)
{
    // Set duty cycle based on value (0-255)
    OC1RS = (PR2 + 1) * ((float)val / 256.0);
}

int main(void)
{
    volatile unsigned char myduty = 0;
    int i;
    
    // Disable JTAG if needed (depends on your setup)
    DDPCON = 0;
    CFGCON = 0;
    TRISAbits.TRISA0 = 0; // Set RA0 as output

    // Set RA0 (pin 2) to high
    LATAbits.LATA0 = 1; // Drive RA0 high
    
    Init_pwm();
    
    // Loop: sweep duty cycle from 0 to 255 and back
    while (1)
    {
       // Set_pwm(153);
       LATAbits.LATA0 = 1; // Drive RA0 high
    }
    return 0;
}
