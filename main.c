#include <XC.h>
#include <stdio.h>
#include <stdlib.h>

/*########################*/
/*LCD Definitions */ 
 
#define LCD_RS LATBbits.LATB3
#define LCD_E  LATAbits.LATA2
#define LCD_D4 LATAbits.LATA3
#define LCD_D5 LATBbits.LATB4
#define LCD_D6 LATBbits.LATA4
#define LCD_D7 LATBbits.LATB5

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

#define LCD_RS_ENABLE TRISBbits.TRISB3
#define LCD_E_ENABLE  TRISAbits.TRISA2
#define LCD_D4_ENABLE TRISAbits.TRISA3
#define LCD_D5_ENABLE TRISBbits.TRISB4
#define LCD_D6_ENABLE TRISAbits.TRISA4
#define LCD_D7_ENABLE TRISBbits.TRISB5

#define CHARS_PER_LINE 16

/*########################*/

// Configuration Bits (XC32 handles these)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Secondary Oscillator Disabled

// Defines
#define SYSCLK 40000000L
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
 
/* Part of LCD timing */
void Timer4us(unsigned char t) 
{
    T4CON = 0x8000; // enable Timer4, source PBCLK, 1:1 prescaler
 
    while( t >= 100){
        t -= 100;
        TMR4 = 0;
        while(TMR4 < SYSCLK/10000L);
    }
 
    while( t >= 10){
        t -= 10;
        TMR4 = 0;
        while(TMR4 < SYSCLK/100000L);
    }
 
    while( t > 0)
    {
        t--;
        TMR4 = 0;
        while(TMR4 < SYSCLK/1000000L);
    }
    T4CONCLR = 0x8000;
}

/*##################*/
void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select for UART2
    U2RXRbits.U2RXR = 4;    // Set RX to RB8
    RPB9Rbits.RPB9R = 2;    // Set RB9 to TX

    U2MODE = 0;           // disable autobaud, TX/RX enabled, 8N1, idle=HIGH
    U2STA = 0x1400;       // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // Baud rate setting
    
    U2MODESET = 0x8000;   // enable UART2
}

// Needed by scanf() and gets()
int _mon_getc(int canblock)
{
    char c;
    
    if (canblock)
    {
        while(!U2STAbits.URXDA);
        c = U2RXREG;
        if(c == '\r') c = '\n';
        return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA)
        {
            c = U2RXREG;
            if(c == '\r') c = '\n';
            return (int)c;
        }
        else
        {
            return -1;
        }
    }
}

// Use core timer for 1 ms delay.
void wait_1ms(void)
{
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT() < (SYSCLK/(2*1000)));
}

/* Two different waitms functions */
void waitms(unsigned int ms)
{
    unsigned int j;
    unsigned char k;
    for(j = 0; j < ms; j++)
        for(k = 0; k < 4; k++)
            Timer4us(250);
}

// Define pin numbers for period measurement on PORTB
#define PIN_PERIOD_1 6  // RB6: bit 6
#define PIN_PERIOD_2 0  // RB0: bit 0

// GetPeriod() works for frequencies between 200Hz and 700kHz
long int GetPeriod (int n, int pin) {
    int i;
    _CP0_SET_COUNT(0);
    while((PORTB & (1 << pin)) == 0) {
        if(_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;
    }
    _CP0_SET_COUNT(0);
    while((PORTB & (1 << pin)) != 0) {
        if(_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;
    }
    _CP0_SET_COUNT(0);
    for(i = 0; i < n; i++) {
        while((PORTB & (1 << pin)) == 0) {
            if(_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;
        }
        while((PORTB & (1 << pin)) != 0) {
            if(_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;
        }
    }
    return _CP0_GET_COUNT();
}

// Initialization for RB0 (used in period measurement)
void initRB0(void) {
    ANSELBbits.ANSB0 = 0;  // Disable analog on RB0
    TRISBbits.TRISB0 = 1;  // Set RB0 as input
}

// --- New: Initialization for push button on pin 5 (RB1) ---
void initPushButton(void) {
    ANSELBbits.ANSB1 = 0;   // Disable analog functionality on RB1 (pin 5)
    TRISBbits.TRISB1 = 1;   // Set RB1 as digital input
    CNPUBbits.CNPUB1 = 1;   // Enable internal pull-up on RB1 (if available)
}

/*######################*/
/*LCD Configuration*/

void LCD_pulse(void)
{
    LCD_E = 1;
    Timer4us(40);
    LCD_E = 0;
}

void LCD_byte(unsigned char x)
{
    LCD_D7 = (x & 0x80) ? 1 : 0;
    LCD_D6 = (x & 0x40) ? 1 : 0;
    LCD_D5 = (x & 0x20) ? 1 : 0;
    LCD_D4 = (x & 0x10) ? 1 : 0;
    LCD_pulse();
    Timer4us(40);
    LCD_D7 = (x & 0x08) ? 1 : 0;
    LCD_D6 = (x & 0x04) ? 1 : 0;
    LCD_D5 = (x & 0x02) ? 1 : 0;
    LCD_D4 = (x & 0x01) ? 1 : 0;
    LCD_pulse();
}

void WriteData(unsigned char x)
{
    LCD_RS = 1;
    LCD_byte(x);
    waitms(2);
}

void WriteCommand(unsigned char x)
{
    LCD_RS = 0;
    LCD_byte(x);
    waitms(5);
}

void LCD_4BIT(void)
{
    // Configure LCD pins as outputs
    LCD_RS_ENABLE = 0;
    LCD_E_ENABLE = 0;
    LCD_D4_ENABLE = 0;
    LCD_D5_ENABLE = 0;
    LCD_D6_ENABLE = 0;
    LCD_D7_ENABLE = 0;
    
    LCD_E = 0;
    waitms(20);
    // Initialize LCD in 8-bit mode then switch to 4-bit mode
    WriteCommand(0x33);
    WriteCommand(0x33);
    WriteCommand(0x32);
    
    WriteCommand(0x28);
    WriteCommand(0x0c);
    WriteCommand(0x01); // Clear screen command
    waitms(20);
    LATBbits.LATB0 = !LATBbits.LATB0;
}

void LCDprint(char * string, unsigned char line, unsigned char clear)
{
    int j;
    
    WriteCommand(line == 2 ? 0xc0 : 0x80);
    waitms(5);
    for(j = 0; string[j] != 0; j++)
        WriteData(string[j]);
    if(clear)
        for(; j < CHARS_PER_LINE; j++)
            WriteData(' ');
}

/*############################*/

/* ADC calibration functions */
void delayms(int len)
{
    while(len--) wait_1ms();
}
 
void ADCConf(void)
{
    AD1CON1CLR = 0x8000;    // disable ADC before configuration
    AD1CON1 = 0x00E0;       // internal counter for auto-convert
    AD1CON2 = 0;            // use AVSS/AVDD as voltage reference
    AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD 
    AD1CON1SET = 0x8000;    // Enable ADC
}

int ADCRead(char analogPIN)
{
    AD1CHS = analogPIN << 16;  // Select analog channel
 
    AD1CON1bits.SAMP = 1;      // Begin sampling
    while(AD1CON1bits.SAMP);   // wait for sampling to complete
    while(!AD1CON1bits.DONE);  // wait for conversion to complete
 
    return ADC1BUF0;
}

/*############################*/
void main(void)
{
    long int count, count2;
    char buff[17];
    char buff2[17];
    float T, f, c, T2, f2, l;
    float r1 = 2000;    // Resistance in ohms
    float r2 = 2000;    // Resistance in ohms
    float rk = 100;
    float ru;
    int adcval;
    float voltage;

    DDPCON = 0;
    CFGCON = 0;
    
    LCD_4BIT();

    // Configure RB2 (AN4) as an analog input
    ANSELBbits.ANSB2 = 1;
    TRISBbits.TRISB2 = 1;
    
    TRISBbits.TRISB6 = 0;
    LATBbits.LATB6 = 0;  
    INTCONbits.MVEC = 1;

    ADCConf(); // Configure ADC

    UART2Configure(115200);  // Setup UART2

    waitms(500);
    printf("PIC32MX130 Period measurement using the core timer free running counter.\r\n"
           "Connect signal to RB6 (pin 15).\r\n");
    
    initRB0();
    initPushButton(); // Initialize push button on pin 5 (RB1)
    
    while(1)
    {
        // Check push button (active low)
        if(PORTBbits.RB1 == 0)
        {
            waitms(50); // debounce delay
            if(PORTBbits.RB1 == 0)
            {
                LCDprint("Button Pressed", 1, 1);
                // Optionally add additional actions here
                // Wait for button release
                while(PORTBbits.RB1 == 0) { }
                waitms(50);
            }
        }
        
        printf("\x1b[H");  
        adcval = ADCRead(4); // Read ADC from AN4 (RB2)
        voltage = adcval * 3.3 / 1023.0;
        if(voltage > 3.0){
            sprintf(buff2, "No Resistor ");
            LCDprint(buff2, 2, 1);    
        }
        else{
            ru = (rk * (voltage / (3.3 - voltage)));
            sprintf(buff2, "Resistor: %.2f", ru);
            LCDprint(buff2, 2, 1);
        }

        count = GetPeriod(100, PIN_PERIOD_1);
        count2 = GetPeriod(100, PIN_PERIOD_2);
        if(count > 0)
        {
            T = (count * 2.0) / (SYSCLK * 100.0);
            f = 1 / T;
            c = 1.44 / ((r1 + 2*r2) * f);
            printf("f=%.2fHz, Count=%ld        \r\n", f, count);
            printf("C=%fuf        \r\n", c * 1000000);
            if(c * 1000000 > 0.0015 && c * 1000000 < 0.0020){
                sprintf(buff, "No Capacitor ");
                LCDprint(buff, 1, 1);
            }
            else{
                sprintf(buff, "C:%.4f uf", c * 1000000);
                LCDprint(buff, 1, 1);
            }
        }
        else
        {
            printf("NO SIGNAL                     \r");
            sprintf(buff, "Large Capacitor ");
            LCDprint(buff, 1, 1);
        }

        if(count2 > 0){
            T2 = (count2 * 2.0) / (SYSCLK * 100.0);
            f2 = 1 / T2;
            printf("frequency2=%.2f\r\n", f2);
            l = 1 / (39.4784 * f2 * f2 * 0.00000099);
            printf("Inductor=%f\r\n", l * (100000.0));
        }
        
        adcval = ADCRead(4);
        voltage = adcval * 3.3 / 1023.0;
        
        fflush(stdout);
        waitms(200);
        waitms(200);
        waitms(100);
    }
}
