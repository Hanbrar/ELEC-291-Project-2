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
#define Baud2BRG(desired_baud) ((SYSCLK / (16*desired_baud))-1)

volatile int ISR_pwm1 = 150, ISR_pwm2 = 150;
volatile int ISR_cnt = 0, ISR_frc = 0;

// Generates a 1ms delay using the core timer.
void wait_1ms(void)
{
    _CP0_SET_COUNT(0); // reset core timer
    while (_CP0_GET_COUNT() < (SYSCLK / (2 * 1000))); // wait 1ms
}

// Wait for a given number of milliseconds.
void waitms(int len)
{
    while(len--) wait_1ms();
}

// Timer1 ISR: generates two servo PWM signals.
// Each 10 µs tick, the ISR compares the current count with the desired pulse width.
void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
    IFS0CLR = _IFS0_T1IF_MASK; // clear Timer1 interrupt flag

    ISR_cnt++;
    // PWM for servo 1 (output on RB2)
    if (ISR_cnt < ISR_pwm1)
        LATBbits.LATB2 = 1;
    else
        LATBbits.LATB2 = 0;

    // PWM for servo 2 (output on RB5)
    if (ISR_cnt < ISR_pwm2)
        LATBbits.LATB5 = 1;
    else
        LATBbits.LATB5 = 0;

    // 2000 counts * 10 µs = 20 ms period (standard for servos)
    if (ISR_cnt >= 2000) {
        ISR_cnt = 0;
        ISR_frc++;
    }
}

// Configure Timer1 to trigger every 10 µs.
void SetupTimer1(void)
{
    __builtin_disable_interrupts();
    PR1 = (SYSCLK / FREQ) - 1; // period so that each tick is 10 µs
    TMR1 = 0;
    T1CONbits.TCKPS = 0;       // prescaler 1:1
    T1CONbits.TCS = 0;         // internal clock
    T1CONbits.ON = 1;
    IPC1bits.T1IP = 5;
    IPC1bits.T1IS = 0;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    
    INTCONbits.MVEC = 1;       // multi-vector interrupts
    __builtin_enable_interrupts();
}

// Configure UART2 for serial communication (baud rate set as needed).
void UART2Configure(int baud_rate)
{
    U2RXRbits.U2RXR = 4;    // set RX to RB8
    RPB9Rbits.RPB9R = 2;    // set TX to RB9

    U2MODE = 0;            // disable autobaud; enable TX and RX, 8N1
    U2STA = 0x1400;        // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // set baud rate generator
    U2MODESET = 0x8000;    // enable UART2
}

// A simple delay using the timer ISR flag.
void delay_ms(int msecs)
{    
    int ticks;
    ISR_frc = 0;
    ticks = msecs / 20; // because the period is 20ms
    while(ISR_frc < ticks);
}

/* SerialReceive() is a blocking function that waits for data on the UART2 RX buffer
 * and then stores all incoming data into *buffer. When a carriage return ('\r') is
 * received, a nul character is appended.
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

/* Pinout for DIP28 PIC32MX130 (omitted for brevity) */

void main(void)
{
    int dir1 = 1, dir2 = -1; // direction flags for oscillation (1 = increasing, -1 = decreasing)

    DDPCON = 0;
    CFGCON = 0;

    // Configure RB2 as digital output for PWM1.
    ANSELBbits.ANSB2 = 0;   // Disable analog on RB2
    TRISBbits.TRISB2 = 0;   // Set RB2 as output
    LATBbits.LATB2 = 0;     // Initialize low

    // Configure RB5 as digital output for PWM2 (digital-only pin).
    TRISBbits.TRISB5 = 0;   // Set RB5 as output
    LATBbits.LATB5 = 0;     // Initialize low
    
    SetupTimer1();           // Set Timer1 to interrupt every 10 µs
    CFGCON = 0;
    UART2Configure(115200);  // Configure UART2 (baud rate 115200)

    // Allow time for the serial terminal to start.
    delay_ms(500);
    
    // Clear screen and print banner.
    printf("\x1b[2J\x1b[1;1H");
    printf("Servo signal generator for the PIC32MX130F064B.\r\n");
    printf("Outputs are on RB2 and RB5.\r\n");
    printf("By Jesus Calvino-Fraga (c) 2018.\r\n");
    printf("Pulse width oscillates between 100 (1.0ms) and 200 (2.0ms)\r\n");
    
    // Initialize PWM values to center position (150 ≈ 1.5ms pulse).
    ISR_pwm1 = 150;
    ISR_pwm2 = 150;
    
    // Main loop: update PWM values to create a triangular (oscillating) waveform.
    while (1)
    {
        // Update servo 1 PWM (oscillates between 100 and 200)
        ISR_pwm1 += dir1;
        if (ISR_pwm1 >= 200) {
            ISR_pwm1 = 200;
            dir1 = -1;  // reverse direction to decrease
        }
        else if (ISR_pwm1 <= 100) {
            ISR_pwm1 = 100;
            dir1 = 1;   // reverse direction to increase
        }

        // Update servo 2 PWM (oscillates between 100 and 200 in opposite phase)
        ISR_pwm2 += dir2;
        if (ISR_pwm2 >= 200) {
            ISR_pwm2 = 200;
            dir2 = -1;
        }
        else if (ISR_pwm2 <= 100) {
            ISR_pwm2 = 100;
            dir2 = 1;
        }

        // Delay between each PWM update.
        waitms(20); // 20 ms delay per step (adjust for faster/slower oscillation)
    }
}
