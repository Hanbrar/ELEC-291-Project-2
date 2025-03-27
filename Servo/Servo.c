#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>

#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz) see figure 8.1 in datasheet for more info
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Secondary Oscillator Enable (Disabled)

#define SYSCLK 40000000L
#define FREQ 100000L // We need the ISR for timer 1 every 10 us
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)

volatile int ISR_pwm1 = 150, ISR_pwm2 = 150;
volatile int ISR_pw = 100, ISR_cnt = 0, ISR_frc;

// The Interrupt Service Routine for timer 1 is used to generate one or more standard
// hobby servo signals. The servo signal has a fixed period of 20ms and a pulse width
// between 0.6ms and 2.4ms.
void wait_1ms(void)
{
    _CP0_SET_COUNT(0); // reset the core timer count
    while (_CP0_GET_COUNT() < (SYSCLK / (2 * 1000))); // wait 1ms
}

void waitms(int len)
{
    while(len--) wait_1ms();
}

void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
    IFS0CLR = _IFS0_T1IF_MASK; // Clear timer 1 interrupt flag

    ISR_cnt++;
    if(ISR_cnt < ISR_pwm1)
        LATBbits.LATB2 = 1;
    else
        LATBbits.LATB2 = 0;

    if(ISR_cnt < ISR_pwm2)
        LATBbits.LATB5 = 1;
    else
        LATBbits.LATB5 = 0;

    if(ISR_cnt >= 2000)  // 2000 * 10us = 20ms period
    {
        ISR_cnt = 0;
        ISR_frc++;
    }
}

void SetupTimer1(void)
{
    __builtin_disable_interrupts();
    PR1 = (SYSCLK / FREQ) - 1; // Set period register so that period = 10us
    TMR1 = 0;
    T1CONbits.TCKPS = 0;       // Prescaler 1:1
    T1CONbits.TCS = 0;         // Use internal clock
    T1CONbits.ON = 1;
    IPC1bits.T1IP = 5;
    IPC1bits.T1IS = 0;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    
    INTCONbits.MVEC = 1;       // Multi-vector interrupts
    __builtin_enable_interrupts();
}

void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    // Set RX to RB8
    RPB9Rbits.RPB9R = 2;    // Set RB9 to TX

    U2MODE = 0;            // Disable autobaud; TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;        // Enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;    // Enable UART2
}

void delay_ms(int msecs)
{    
    int ticks;
    ISR_frc = 0;
    ticks = msecs / 20;
    while(ISR_frc < ticks);
}

/* SerialReceive() is a blocking function that waits for data on
 * the UART2 RX buffer and then stores all incoming data into *buffer.
 * When a carriage return ('\r') is received, a nul character is appended.
 */
unsigned int SerialReceive(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
    while(num_char < max_size)
    {
        while(!U2STAbits.URXDA);   // wait until data available in RX buffer
        *buffer = U2RXREG;         // read the received data

        while(U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = *buffer;         // echo the received character
 
        if(*buffer == '\r')        // end of string on carriage return
        {
            *buffer = '\0';
            break;
        }
 
        buffer++;
        num_char++;
    }
    return num_char;
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

void main(void)
{
    char buf[32];
    int pw;
    
    DDPCON = 0;
    CFGCON = 0;

    // Configure RB2 as digital output for PWM1
    ANSELBbits.ANSB2 = 0;   // Disable analog on RB2
    TRISBbits.TRISB2 = 0;   // Set RB2 as output
    LATBbits.LATB2 = 0;     // Initialize low

    // Configure RB5 as digital output for PWM2
    // Remove analog disable as RB5 is not analog-capable on this device
    TRISBbits.TRISB5 = 0;   // Set RB5 as output
    LATBbits.LATB5 = 0;     // Initialize low
    
    SetupTimer1();         // Set timer 1 to interrupt every 10 us
    CFGCON = 0;
    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    
    // Give putty a chance to start
    delay_ms(500);
    
    // Clear screen using ANSI escape sequence.
    printf("\x1b[2J\x1b[1;1H");
    printf("Servo signal generator for the PIC32MX130F064B.\r\n");
    printf("Outputs are on RB2 and RB5.\r\n");
    printf("By Jesus Calvino-Fraga (c) 2018.\r\n");
    printf("Pulse width between 60 (0.6ms) and 240 (2.4ms)\r\n");
    
    // Initialize PWM values to center (150 counts ~ 1.5ms)
    ISR_pwm1 = 150;
    ISR_pwm2 = 150;
    
    while (1)
    {
        // Increase ISR_pwm1 from 60 to 240 (sweeping pulse width upward)
        if (ISR_pwm1 < 240)
            ISR_pwm1++;
        else
            ISR_pwm1 = 60;

        // Decrease ISR_pwm2 from 240 to 60 (sweeping pulse width downward)
        if (ISR_pwm2 > 60)
            ISR_pwm2--;
        else
            ISR_pwm2 = 240;

        // Wait 2000 ms (each delay is 20 ms period * number of cycles)
        waitms(2000);
    }
}
