#include <XC.h>
#include <stdio.h>
#include <stdlib.h>
 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Secondary Oscillator Enable (Disabled)

// Defines
#define SYSCLK 40000000L
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
 
void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

// Needed to by scanf() and gets()
int _mon_getc(int canblock)
{
	char c;
	
    if (canblock)
    {
	    while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	    c=U2RXREG;
	    if(c=='\r') c='\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
		return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA) // if data available in RX buffer
        {
		    c=U2RXREG;
		    if(c=='\r') c='\n';
			return (int)c;
        }
        else
        {
            return -1; // no characters to return
        }
    }
}

// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

#define PIN_PERIOD_1 6  // RB6: bit 6
#define PIN_PERIOD_2 0  // RB0: bit 0

// GetPeriod() works for frequencies between 200Hz and 700kHz

//This is the period the code where you input function and count and
// automatically get the period of the signal
long int GetPeriod (int n, int pin) {
    int i;

    // Wait for the square wave to go HIGH
    _CP0_SET_COUNT(0); // Reset core timer count
    while ((PORTB & (1 << pin)) == 0) {
        if (_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;  // Timeout check
    }

    // Wait for the square wave to go LOW
    _CP0_SET_COUNT(0); // Reset core timer count
    while ((PORTB & (1 << pin)) != 0) {
        if (_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;  // Timeout check
    }

    // Measure time for 'n' periods
    _CP0_SET_COUNT(0); // Reset core timer count
    for (i = 0; i < n; i++) {
        while ((PORTB & (1 << pin)) == 0) { // Wait for square wave HIGH
            if (_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;  // Timeout check
        }
        while ((PORTB & (1 << pin)) != 0) { // Wait for square wave LOW
            if (_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;  // Timeout check
        }
    }

    return _CP0_GET_COUNT(); // Return the count of the core timer
}

/* Pinout for DIP28 PIC32MX130:
                                          --------
                                   MCLR -|1     28|- AVDD 
  VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 -|2     27|- AVSS 
        VREF-/CVREF-/AN1/RPA1/CTED2/RA1 -|3     26|- AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15
   PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 -|4     25|- CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14
  PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 -|5     24|- AN11/RPB13/CTPLS/PMRD/RB13
   AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 -|6     23|- AN12/PMD0/RB12
     AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 -|7     22|- PGEC2/TMS/RPB11/PMD1/RB11
                                    VSS -|8     21|- PGED2/RPB10/CTED11/PMD2/RB10
                     OSC1/CLKI/RPA2/RA2 -|9     20|- VCAP
                OSC2/CLKO/RPA3/PMA0/RA3 -|10    19|- VSS
                         SOSCI/RPB4/RB4 -|11    18|- TDO/RPB9/SDA1/CTED4/PMD3/RB9
         SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 -|12    17|- TCK/RPB8/SCL1/CTED10/PMD4/RB8
                                    VDD -|13    16|- TDI/RPB7/CTED3/PMD5/INT0/RB7
                    PGED3/RPB5/PMD7/RB5 -|14    15|- PGEC3/RPB6/PMD6/RB6
                                          --------
*/

// Information here:
// http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/1-basic-digital-io-220
void main(void)
{
	long int count;
	float T, f;
    float r;
    float ct,c1,c2;
    c1=0.00000001; //10nf
    c2 = 0.0000001; //100nf
    float Inductor;
    DDPCON = 0;
	
	CFGCON = 0;
    
    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    
    ANSELB &= ~(1<<6); // Set RB6 as a digital I/O
    TRISB |= (1<<6);   // configure pin RB6 as input
    CNPUB |= (1<<6);   // Enable pull-up resistor for RB6
 
	waitms(500);
	printf("PIC32MX130 Period measurement using the core timer free running counter.\r\n"
	       "Connect signal to RB6 (pin 15).\r\n");
    
    while(1)
    {
        //Connect to pin4
        //We will need two frequencies to measure the two different inductors
		count=GetPeriod(100,PIN_PERIOD_1);
		if(count>0)
		{
			T=(count*2.0)/(SYSCLK*100.0);
            f=1/T;
            ct=(c1*c1)/(c1+c2);
            Inductor=1/(39.4784*f*f*ct); // L=1/(4*pi^2*f^2*C) 4pi^2~39.4784
            /*##########################*/
            //This is where in the code where we will measure inductor that will
            //directly tell us if metal content is present 

            printf("Inductor=%f\r\n", Inductor);
		}
		else
		{
			printf("NO SIGNAL                     \r");
		}
		fflush(stdout); // GCC peculiarities: need to flush stdout to get string out without a '\n'
		waitms(200);
        waitms(200);
    }
}
