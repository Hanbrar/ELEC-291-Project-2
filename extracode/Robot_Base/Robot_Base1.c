#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


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
 
// Configuration Bits
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Disable secondary oscillator

// Defines
#define SYSCLK 40000000L
#define DEF_FREQ 16000L
#define Baud2BRG(desired_baud) ((SYSCLK/(16*(desired_baud)))-1)
#define Baud1BRG(desired_baud) ((SYSCLK/(16*(desired_baud)))-1)

// Global variables for Robot Base servo PWM
volatile int ISR_pwm1 = 150, ISR_pwm2 = 150, ISR_cnt = 0;

// ----------------------------------------------------------------
// Function Prototypes
// ----------------------------------------------------------------
// UART and delay functions
void UART2Configure(int baud_rate);
int UART1Configure(int desired_baud);
void putc1(char c);
int SerialTransmit1(const char *buffer);
int SerialReceive1(char *buffer, unsigned int max_size);
int SerialReceive1_timeout(char *buffer, unsigned int max_size);
void ClearFIFO(void);
void wait_1ms(void);
void delayms(int len);
#define waitms delayms  // Alias so that calls to waitms in Robot_Base code work

// JDY40 specific functions
void ReceptionOff(void);
void SendATCommand(char *s);

// Robot Base functions
void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void);
void SetupTimer1(void);
void ConfigurePins(void);
void ADCConf(void);
int ADCRead(char analogPIN);
long int GetPeriod(int n);
void uart_puts(char *s);
void PrintNumber(long int val, int Base, int digits);
void PrintFixedPoint(unsigned long number, int decimals);

// ----------------------------------------------------------------
// Function Definitions
// ----------------------------------------------------------------

// UART2 configuration (shared by both functionalities)
void UART2Configure(int baud_rate)
{
    U2RXRbits.U2RXR = 4;    // SET RX to RB8
    RPB9Rbits.RPB9R = 2;    // SET RB9 to TX

    U2MODE = 0;           // disable autobaud; enable TX and RX; 8N1
    U2STA = 0x1400;       // enable TX and RX
    U2BRG = Baud2BRG(baud_rate);
    
    U2MODESET = 0x8000;   // enable UART2
}

// UART1 functions (used for JDY40 communication)
int UART1Configure(int desired_baud)
{
    int actual_baud;
    // Set up UART1 RX on RB13
    ANSELB &= ~(1<<13);  // digital I/O
    TRISB |= (1<<13);    // input
    CNPUB |= (1<<13);    // enable pull-up
    U1RXRbits.U1RXR = 3; // Map U1RX to RB13

    // Set up UART1 TX on RB15
    ANSELB &= ~(1<<15);
    RPB15Rbits.RPB15R = 1; // Map RB15 to U1TX
    
    U1MODE = 0;
    U1STA = 0x1400;
    U1BRG = Baud1BRG(desired_baud);
    actual_baud = SYSCLK / (16 * (U1BRG+1));
    U1MODESET = 0x8000; // enable UART1

    return actual_baud;
}

void putc1(char c)
{
    while (U1STAbits.UTXBF); // wait if TX buffer is full
    U1TXREG = c;
}

int SerialTransmit1(const char *buffer)
{
    unsigned int size = strlen(buffer);
    while(size)
    {
        while (U1STAbits.UTXBF);
        U1TXREG = *buffer;
        buffer++;
        size--;
    }
    while(!U1STAbits.TRMT); // wait until transmission is complete
    return 0;
}

int SerialReceive1(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
    while(num_char < max_size)
    {
        while(!U1STAbits.URXDA); // wait for data
        *buffer = U1RXREG;
        if (*buffer == '\n')
        {
            *buffer = '\0';
            break;
        }
        buffer++;
        num_char++;
    }
    return num_char;
}

int SerialReceive1_timeout(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
    int timeout_cnt;
    while(num_char < max_size)
    {
        timeout_cnt = 0;
        while(1)
        {
            if (U1STAbits.URXDA)
            {
                timeout_cnt = 0;
                *buffer = U1RXREG;
                break;
            }
            if (++timeout_cnt == 200)
            {
                *buffer = '\n';
                break;
            }
            delayms(100);
        }
        if(*buffer == '\n')
        {
            *buffer = '\0';
            break;
        }
        buffer++;
        num_char++;
    }
    return num_char;
}

void ClearFIFO(void)
{
    unsigned char c;
    U1STA = 0x1400;  // clear FIFO and re-enable TX/RX
    while (U1STAbits.URXDA)
        c = U1RXREG;
}

// Delay functions using the core timer
void wait_1ms(void)
{
    _CP0_SET_COUNT(0);
    while (_CP0_GET_COUNT() < (SYSCLK/(2*1000)));
}

void delayms(int len)
{
    while(len--) wait_1ms();
}

// (Optional) _mon_getc so that scanf() and gets() work properly
int _mon_getc(int canblock)
{
    char c;
    if (canblock)
    {
        while (!U1STAbits.URXDA);
        c = U1RXREG;
        while (U1STAbits.UTXBF);
        U1TXREG = c;
        if(c=='\r') c='\n';
        return (int)c;
    }
    else
    {
        if (U1STAbits.URXDA)
        {
            c = U1RXREG;
            if(c=='\r') c='\n';
            return (int)c;
        }
        else
        {
            return -1;
        }
    }
}

void delayus(uint16_t uiuSec)
{
    uint32_t ulEnd, ulStart;
    ulStart = _CP0_GET_COUNT();
    ulEnd = ulStart + (SYSCLK / 2000000) * uiuSec;
    if(ulEnd > ulStart)
        while(_CP0_GET_COUNT() < ulEnd);
    else
        while((_CP0_GET_COUNT() > ulStart) || (_CP0_GET_COUNT() < ulEnd));
}

// ----------------------------------------------------------------
// JDY40 Functions
// ----------------------------------------------------------------

void ReceptionOff(void)
{
    LATB &= ~(1<<14);  // Set JDY40 'SET' pin to 0 (AT mode)
    delayms(10);
    SerialTransmit1("AT+DVID0000\r\n");
    delayms(10);
    ClearFIFO();
    LATB |= (1<<14);   // Set back to normal mode
}

void SendATCommand(char *s)
{
    char buff[40];
    printf("Command: %s", s);
    LATB &= ~(1<<14);  // AT mode
    delayms(10);
    SerialTransmit1(s);
    U1STA = 0x1400;    // clear FIFO
    SerialReceive1(buff, sizeof(buff)-1);
    LATB |= (1<<14);   // normal operation mode
    delayms(10);
    printf("Response: %s\n", buff);
}

// ----------------------------------------------------------------
// Robot Base Functions
// ----------------------------------------------------------------

// Timer1 ISR for servo PWM generation
void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
    IFS0CLR = _IFS0_T1IF_MASK; // Clear Timer1 interrupt flag
    ISR_cnt++;
    if (ISR_cnt == ISR_pwm1)
        LATAbits.LATA3 = 0;
    if (ISR_cnt == ISR_pwm2)
        LATBbits.LATB4 = 0;
    if (ISR_cnt >= 2000)
    {
        ISR_cnt = 0; // 2000 * 10us = 20ms period
        LATAbits.LATA3 = 1;
        LATBbits.LATB4 = 1;
    }
}

void SetupTimer1(void)
{
    __builtin_disable_interrupts();
    PR1 = (SYSCLK/DEF_FREQ)-1;  // DEF_FREQ is 16000L from our defines
    TMR1 = 0;
    T1CONbits.TCKPS = 0;  // 1:1 prescale
    T1CONbits.TCS = 0;    // use internal clock
    T1CONbits.ON = 1;
    IPC1bits.T1IP = 5;
    IPC1bits.T1IS = 0;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    
    INTCONbits.MVEC = 1;
    __builtin_enable_interrupts();
}

void ConfigurePins(void)
{
    // Configure analog inputs on RB2 (AN4) and RB3 (AN5)
    ANSELBbits.ANSB2 = 1;
    TRISBbits.TRISB2 = 1;
    ANSELBbits.ANSB3 = 1;
    TRISBbits.TRISB3 = 1;
    
    // Configure digital input for period measurement on RB5
    ANSELB &= ~(1<<5);
    TRISB |= (1<<5);
    CNPUB |= (1<<5);
    
    // Configure digital outputs for LED toggling and servo signals
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 0;
    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA3 = 0; // servo PWM output
    TRISBbits.TRISB4 = 0; // servo PWM output
    
    INTCONbits.MVEC = 1;
}

void ADCConf(void)
{
    AD1CON1CLR = 0x8000;    // disable ADC before configuration
    AD1CON1 = 0x00E0;       // auto-convert; manual sample
    AD1CON2 = 0;            // use AVdd/AVss as references
    AD1CON3 = 0x0f01;       // TAD and acquisition time setup
    AD1CON1SET = 0x8000;    // enable ADC
}

int ADCRead(char analogPIN)
{
    AD1CHS = analogPIN << 16;  // select analog input channel
    AD1CON1bits.SAMP = 1;      // start sampling
    while(AD1CON1bits.SAMP);   // wait for sampling to complete
    while(!AD1CON1bits.DONE);  // wait for conversion to complete
    return ADC1BUF0;
}

long int GetPeriod(int n)
{
    int i;
    _CP0_SET_COUNT(0);
    // Wait for RB5 (digital input) to go low
    while((PORTB & (1<<5)) != 0)
        if(_CP0_GET_COUNT() > (SYSCLK/8)) return 0;
    
    _CP0_SET_COUNT(0);
    // Wait for RB5 to go high
    while((PORTB & (1<<5)) == 0)
        if(_CP0_GET_COUNT() > (SYSCLK/8)) return 0;
    
    _CP0_SET_COUNT(0);
    for(i = 0; i < n; i++)
    {
        while((PORTB & (1<<5)) != 0)
            if(_CP0_GET_COUNT() > (SYSCLK/8)) return 0;
        while((PORTB & (1<<5)) == 0)
            if(_CP0_GET_COUNT() > (SYSCLK/8)) return 0;
    }
    return _CP0_GET_COUNT();
}

void uart_puts(char *s)
{
    while(*s)
    {
        putchar(*s);
        s++;
    }
}

char HexDigit[] = "0123456789ABCDEF";
void PrintNumber(long int val, int Base, int digits)
{
    int j;
    #define NBITS 32
    char buff[NBITS+1];
    buff[NBITS] = 0;
    j = NBITS-1;
    while ((val > 0) || (digits > 0))
    {
        buff[j--] = HexDigit[val % Base];
        val /= Base;
        if(digits != 0) digits--;
    }
    uart_puts(&buff[j+1]);
}

void PrintFixedPoint(unsigned long number, int decimals)
{
    int divider = 1, j;
    j = decimals;
    while(j--) divider *= 10;
    PrintNumber(number/divider, 10, 1);
    uart_puts(".");
    PrintNumber(number % divider, 10, decimals);
}

// ----------------------------------------------------------------
// Merged main() Function
// ----------------------------------------------------------------

void main(void)
{
    // Variables for JDY40 test
    char buff[80];
    char *token;
    int values[7] = {0};
    int i = 0, cnt = 0;
    char c;
    int up = 0, down = 0, left = 0, right = 0;
    int button_auto = 0, button_manual = 0, button_coin = 0;
    
    // Variables for Robot Base functionality
    int adcval;
    long int v;
    unsigned long count, f;
    unsigned char LED_toggle = 0;
    
    // System configuration
    DDPCON = 0;
    CFGCON = 0;
  
    // Initialize UARTs
    UART2Configure(115200);
    UART1Configure(9600);
    
    // Configure JDY40 SET pin on RB14 as digital output and set normal mode
    ANSELB &= ~(1<<14);
    TRISB &= ~(1<<14);
    LATB |= (1<<14);
    
    // Initialize Robot Base peripherals
    ConfigurePins();
    SetupTimer1();
    ADCConf();
    
    delayms(500);  // Allow time for terminal (PuTTY) to start
    
    printf("\r\nMerged Program: JDY40 Test and Robot Base functionalities.\r\n");
    uart_puts("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence
    uart_puts("\r\nPIC32 multi I/O example with JDY40 test integrated.\r\n");
    
    // JDY40 initialization: put the module into AT mode, then send a series of AT commands
    ReceptionOff();
    SendATCommand("AT+VER\r\n");
    SendATCommand("AT+BAUD\r\n");
    SendATCommand("AT+RFID\r\n");
    SendATCommand("AT+DVID\r\n");
    SendATCommand("AT+RFC\r\n");
    SendATCommand("AT+POWE\r\n");
    SendATCommand("AT+CLSS\r\n");
    SendATCommand("AT+DVIDABBA\r\n");
    
    // Main loop: interleave JDY40 communication and Robot Base operations
    while(1)
    {
        // --- JDY40 Test Section ---
        if(U1STAbits.URXDA)
        {
            c = U1RXREG;
            if(c == '!')
            {
                SerialReceive1(buff, sizeof(buff)-1);
                i = 0;
                token = strtok(buff, " ");
                while(token != NULL && i < 7)
                {
                    values[i] = atoi(token);
                    token = strtok(NULL, " ");
                    i++;
                }
                up = values[0];
                down = values[1];
                left = values[2];
                right = values[3];
                button_auto = values[4];
                button_manual = values[5];
                button_coin = values[6];
                
                // Check if message length is valid (13 characters expected)
                if(strlen(buff) == 13)
                {
                    if(up == 1)      { printf("UP "); }
                    if(down == 1)    { printf("DOWN "); }
                    if(left == 1)    { printf("LEFT "); }
                    if(right == 1)   { printf("RIGHT "); }
                    if(button_auto == 1)   { printf("AUTO "); }
                    if(button_manual == 1) { printf("MANUAL "); }
                    if(button_coin == 1)   { printf("PICK UP THE COIN "); }
                    printf("\r\n");
                }
                else
                {
                    ClearFIFO();
                    printf("*** BAD MESSAGE ***: %s\r\n", buff);
                }
            }
            else if(c == '@')
            {
                sprintf(buff, "%05u\n", cnt);
                cnt++;
                delayms(5);
                SerialTransmit1(buff);
            }
            else
            {
                ClearFIFO();
            }
        }


        
        
        // --- Robot Base Section ---
        adcval = ADCRead(4);  // Read ADC channel 4 (AN4 on RB2)
        uart_puts("ADC[4]=0x");
        PrintNumber(adcval, 16, 3);
        uart_puts(", V=");
        v = (adcval * 3290L) / 1023L;
        PrintFixedPoint(v, 3);
        uart_puts("V ");
        
        adcval = ADCRead(5);  // Read ADC channel 5 (AN5 on RB3)
        uart_puts("ADC[5]=0x");
        PrintNumber(adcval, 16, 3);
        uart_puts(", V=");
        v = (adcval * 3290L) / 1023L;
        PrintFixedPoint(v, 3);
        uart_puts("V ");
        
        count = GetPeriod(100);
        if(count > 0)
        {
            f = ((SYSCLK/2L) * 100L) / count;
            uart_puts("f=");
            PrintNumber(f, 10, 7);
            uart_puts("Hz, count=");
            PrintNumber(count, 10, 6);
            uart_puts("          \r\n");
        }
        else
        {
            uart_puts("NO SIGNAL                     \r\n");
        }
        
        // Toggle output pins to verify digital I/O
        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 0;
        LATBbits.LATB0 = 0;
        LATBbits.LATB1 = 0;
        LATAbits.LATA2 = 0;
        
        switch(LED_toggle++)
        {
            case 0: LATAbits.LATA0 = 1; break;
            case 1: LATAbits.LATA1 = 1; break;
            case 2: LATBbits.LATB0 = 1; break;
            case 3: LATBbits.LATB1 = 1; break;
            case 4: LATAbits.LATA2 = 1; break;
            default: break;
        }
        if(LED_toggle > 4) LED_toggle = 0;
        
        // Update servo PWM signals (values are adjusted between 100 and 200)
        if(ISR_pwm1 < 200)
            ISR_pwm1++;
        else
            ISR_pwm1 = 100;
            
        if(ISR_pwm2 > 100)
            ISR_pwm2--;
        else
            ISR_pwm2 = 200;
            
        delayms(200);
    }
}