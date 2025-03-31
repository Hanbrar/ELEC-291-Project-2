// #include <XC.h>
// #include <sys/attribs.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>

// /* Pinout for DIP28 PIC32MX130:
//                                         --------
//                                 MCLR -|1     28|- AVDD 
// VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 -|2     27|- AVSS 
//         VREF-/CVREF-/AN1/RPA1/CTED2/RA1 -|3     26|- AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15
// PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 -|4     25|- CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14
// PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 -|5     24|- AN11/RPB13/CTPLS/PMRD/RB13
// AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 -|6     23|- AN12/PMD0/RB12
//     AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 -|7     22|- PGEC2/TMS/RPB11/PMD1/RB11
//                                     VSS -|8     21|- PGED2/RPB10/CTED11/PMD2/RB10
//                     OSC1/CLKI/RPA2/RA2 -|9     20|- VCAP
//                 OSC2/CLKO/RPA3/PMA0/RA3 -|10    19|- VSS
//                         SOSCI/RPB4/RB4 -|11    18|- TDO/RPB9/SDA1/CTED4/PMD3/RB9
//         SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 -|12    17|- TCK/RPB8/SCL1/CTED10/PMD4/RB8
//                                     VDD -|13    16|- TDI/RPB7/CTED3/PMD5/INT0/RB7
//                     PGED3/RPB5/PMD7/RB5 -|14    15|- PGEC3/RPB6/PMD6/RB6
//                                         --------
// */

// // Configuration Bits (somehow XC32 takes care of this)
// #pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
// #pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
// #pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
// #pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)

// #pragma config FWDTEN = OFF         // Watchdog Timer Disabled
// #pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
// #pragma config FSOSCEN = OFF

// // Defines
// #define SYSCLK 40000000L
// //#define DEF_FREQ 16000L
// #define FREQ 100000L // We need the ISR for timer 1 every 10 us
// #define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
// #define Baud1BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)

// //timer macro
// #define ELAPSED_TIME_MS(start) ((_CP0_GET_COUNT() - (start)) / (SYSCLK/2000))
// //CP0 gives total time since power on or last reset
// //start is a snapshot of CP0 at a specific moment, like when a movement state starts
// //so if start is at 600, and CP0 keeps going up, once it gets to 800,
// //then the elapsed is 200

// enum MovementState {
//     START,
//     FORWARD,
//     BACKWARD,
//     RIGHT_45,
//     LEFT_90,
//     COIN_DETECTED
// };

// enum CoinSubState {
//     COIN_STOP,
//     COIN_SERVO,
//     COIN_RESUME
// };

// volatile int ISR_pwm1=150, ISR_pwm2=150, ISR_pwm3=150, ISR_pwm4=150, ISR_cnt=0, ISR_pwm5=240, ISR_pwm6=200;
// volatile enum MovementState movement_state = START; //set default state
// volatile unsigned int state_start_time = 0;

// //volatile int cnt = 0;

// volatile enum CoinSubState coin_substate = COIN_STOP;
// volatile unsigned int coin_start_time;
// volatile int servo_position = 0;
// const unsigned int SERVO_UPDATE_INTERVAL = 10;
// const unsigned int SERVO_STEP_DELAY = 500;

// //subject to change
// const unsigned int BACKWARD_TIME = 1000;    // 1 second in milliseconds
// const unsigned int RIGHT_45_TIME = 500;    // 0.5 second
// const unsigned int LEFT_90_TIME = 1000;   // 1 second
// const unsigned int COIN_STOP_TIME = 2000; // 2 seconds


// // The Interrupt Service Routine for timer 1 is used to generate one or more standard
// // hobby servo signals.  The servo signal has a fixed period of 20ms and a pulse width
// // between 0.6ms and 2.4ms.
// void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
// {
//     IFS0CLR=_IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0
    

//     ISR_cnt++;

//     // Turn off pins when ISR_cnt matches their respective PWM values
//     if (ISR_cnt == ISR_pwm2) {
//         LATAbits.LATA3 = 0; // Turn off RA3 (PWM output, pin 10)
//     }
//     if (ISR_cnt == ISR_pwm3) {
//         LATBbits.LATB4 = 0; // Turn off RB4 (PWM output, pin 11)
//     }
//     if (ISR_cnt == ISR_pwm1) {
//         LATAbits.LATA2 = 0; // Turn off RA2 (PWM output, pin 9)
//     }
//     if (ISR_cnt == ISR_pwm4) {
//         LATAbits.LATA4 = 0; // Turn off RA4 (PWM output, pin 12)
//     }

//     // Reset ISR_cnt and turn on all pins at the start of the PWM cycle
//     if (ISR_cnt >= 100) // 2000 * 10us = 20ms
//     {
//         ISR_cnt = 0;
//         LATAbits.LATA2 = 1; // Turn on RA2 (PWM output, pin 9)
//         LATAbits.LATA3 = 1; // Turn on RA3 (PWM output, pin 10)
//         LATBbits.LATB4 = 1; // Turn on RB4 (PWM output, pin 11)
//         LATAbits.LATA4 = 1; // Turn on RA4 (PWM output, pin 12)
//     }

// }

// //SetupTimer
// void SetupTimer1(void)
// {
//     __builtin_disable_interrupts();
//     PR1 = (SYSCLK / 10000L) - 1; // Set Timer1 period for 10 kHz PWM
//     TMR1 = 0;
//     T1CONbits.TCKPS = 0; // No prescaler
//     T1CONbits.TCS = 0;   // Use internal clock
//     T1CONbits.ON = 1;
//     IPC1bits.T1IP = 5;
//     IPC1bits.T1IS = 0;
//     IFS0bits.T1IF = 0;
//     IEC0bits.T1IE = 1;
    
//     INTCONbits.MVEC = 1; // Enable multi-vector interrupts
//     __builtin_enable_interrupts();
// }


// /*##################################*/
// #define PIN_PERIOD (PORTB&(1<<6)) // for pin 15
// // GetPeriod() seems to work fine for frequencies between 200Hz and 700kHz.
// long int GetPeriod (int n)
// {
//     int i;
//     unsigned int saved_TCNT1a, saved_TCNT1b;
    
//     _CP0_SET_COUNT(0); // resets the core timer count
//     while (PIN_PERIOD!=0) // Wait for square wave to be 0
//     {
//         if(_CP0_GET_COUNT() > (SYSCLK/8)) return 0;
//     }

//     _CP0_SET_COUNT(0); // resets the core timer count
//     while (PIN_PERIOD==0) // Wait for square wave to be 1
//     {
//         if(_CP0_GET_COUNT() > (SYSCLK/8)) return 0;
//     }
    
//     _CP0_SET_COUNT(0); // resets the core timer count
//     for(i=0; i<n; i++) // Measure the time of 'n' periods
//     {
//         while (PIN_PERIOD!=0) // Wait for square wave to be 0
//         {
//             if(_CP0_GET_COUNT() > (SYSCLK/8)) return 0;
//         }
//         while (PIN_PERIOD==0) // Wait for square wave to be 1
//         {
//             if(_CP0_GET_COUNT() > (SYSCLK/8)) return 0;
//         }
//     }

//     return  _CP0_GET_COUNT();
// }


// void UART2Configure(int baud_rate)
// {
//     // Peripheral Pin Select
//     U2RXRbits.U2RXR = 4;    //SET RX to RB8
//     RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

//     U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
//     U2STA = 0x1400;     // enable TX and RX
//     U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
//     U2MODESET = 0x8000;     // enable UART2
// }

// void uart_puts(char * s)
// {
//     while(*s)
//     {
//         putchar(*s);
//         s++;
//     }
// }

// char HexDigit[]="0123456789ABCDEF";
// void PrintNumber(long int val, int Base, int digits)
// { 
//     int j;
//     #define NBITS 32
//     char buff[NBITS+1];
//     buff[NBITS]=0;

//     j=NBITS-1;
//     while ( (val>0) | (digits>0) )
//     {
//         buff[j--]=HexDigit[val%Base];
//         val/=Base;
//         if(digits!=0) digits--;
//     }
//     uart_puts(&buff[j+1]);
// }
// /*ADC code*/
// /*################################*/
// // Good information about ADC in PIC32 found here:
// // http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/adc
// void ADCConf(void)
// {
//     AD1CON1CLR = 0x8000;    // disable ADC before configuration
//     AD1CON1 = 0x00E0;       // internal counter ends sampling and starts conversion (auto-convert), manual sample
//     AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
//     AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD 
//     AD1CON1SET=0x8000;      // Enable ADC
// }

// int ADCRead(char analogPIN)
// {
//     AD1CHS = analogPIN << 16;    // AD1CHS<16:19> controls which analog pin goes to the ADC

//     AD1CON1bits.SAMP = 1;        // Begin sampling
//     while(AD1CON1bits.SAMP);     // wait until acquisition is done
//     while(!AD1CON1bits.DONE);    // wait until conversion done

//     return ADC1BUF0;             // result stored in ADC1BUF0
// }

// void PrintFixedPoint (unsigned long number, int decimals)
// {
//     int divider=1, j;
    
//     j=decimals;
//     while(j--) divider*=10;
    
//     PrintNumber(number/divider, 10, 1);
//     uart_puts(".");
//     PrintNumber(number%divider, 10, decimals);
// }

// // Needed to by scanf() and gets()
// int _mon_getc(int canblock)
// {
//     char c;
    
//     if (canblock)
//     {
//         while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
//         c=U2RXREG;
//         while( U2STAbits.UTXBF);    // wait while TX buffer full
//         U2TXREG = c;          // echo
//         if(c=='\r') c='\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
//         return (int)c;
//     }
//     else
//     {
//         if (U2STAbits.URXDA) // if data available in RX buffer
//         {
//             c=U2RXREG;
//             if(c=='\r') c='\n';
//             return (int)c;
//         }
//         else
//         {
//             return -1; // no characters to return
//         }
//     }
// }

// /////////////////////////////////////////////////////////
// // UART1 functions used to communicate with the JDY40  //
// /////////////////////////////////////////////////////////

// // TXD1 is in pin 26
// // RXD1 is in pin 24

// int UART1Configure(int desired_baud)
// {
//     int actual_baud;

//     // Peripheral Pin Select for UART1.  These are the pins that can be used for U1RX from TABLE 11-1 of '60001168J.pdf':
//     // 0000 = RPA2
//     // 0001 = RPB6
//     // 0010 = RPA4
//     // 0011 = RPB13
//     // 0100 = RPB2

//     // Do what the caption of FIGURE 11-2 in '60001168J.pdf' says: "For input only, PPS functionality does not have
//     // priority over TRISx settings. Therefore, when configuring RPn pin for input, the corresponding bit in the
//     // TRISx register must also be configured for input (set to  1 )."
    
//     ANSELB &= ~(1<<13); // Set RB13 as a digital I/O
//     TRISB |= (1<<13);   // configure pin RB13 as input
//     CNPUB |= (1<<13);   // Enable pull-up resistor for RB13
//     U1RXRbits.U1RXR = 3; // SET U1RX to RB13

//     // These are the pins that can be used for U1TX. Check table TABLE 11-2 of '60001168J.pdf':
//     // RPA0
//     // RPB3
//     // RPB4
//     // RPB15
//     // RPB7

//     ANSELB &= ~(1<<15); // Set RB15 as a digital I/O
//     RPB15Rbits.RPB15R = 1; // SET RB15 to U1TX
    
//     U1MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
//     U1STA = 0x1400;     // enable TX and RX
//     U1BRG = Baud1BRG(desired_baud); // U1BRG = (FPb / (16*baud)) - 1
//     // Calculate actual baud rate
//     actual_baud = SYSCLK / (16 * (U1BRG+1));

//     U1MODESET = 0x8000;     // enable UART1

//     return actual_baud;
// }

// void ConfigurePins(void)
// {
//     // Configure pins as analog inputs
//     ANSELBbits.ANSB2 = 1;   // set RB2 (AN4, pin 6 of DIP28) as analog pin
//     TRISBbits.TRISB2 = 1;   // set RB2 as an input
//     ANSELBbits.ANSB3 = 1;   // set RB3 (AN5, pin 7 of DIP28) as analog pin
//     TRISBbits.TRISB3 = 1;   // set RB3 as an input
    
//     // Configure digital input pin to measure signal period
//     ANSELB &= ~(1<<5); // Set RB5 as a digital I/O (pin 14 of DIP28)
//     TRISB |= (1<<5);   // configure pin RB5 as input
//     CNPUB |= (1<<5);   // Enable pull-up resistor for RB5
    
//     // We can do the three lines above using this instead:
//     // ANSELBbits.ANSELB5=0;  Not needed because RB5 can not be analog input?
//     // TRISBbits.TRISB5=1;
//     // CNPUBbits.CNPUB5=1;
    
//     // Configure output pins
//     TRISAbits.TRISA0 = 0; // pin  2 of DIP28
//     TRISAbits.TRISA1 = 0; // pin  3 of DIP28
//     TRISBbits.TRISB0 = 0; // pin  4 of DIP28
//     TRISBbits.TRISB1 = 0; // pin  5 of DIP28
//     TRISAbits.TRISA2 = 0; // pin  9 of DIP28
//     TRISAbits.TRISA3 = 0; // pin 10 of DIP28
//     TRISBbits.TRISB4 = 0; // pin 11 of DIP28
//     TRISAbits.TRISA4 = 0; // pin 12 of DIP28
    
//     INTCONbits.MVEC = 1;
// }

// void putc1 (char c)
// {
//     while( U1STAbits.UTXBF);   // wait while TX buffer full
//     U1TXREG = c;               // send single character to transmit buffer
// }

// int SerialTransmit1(const char *buffer)
// {
//     unsigned int size = strlen(buffer);
//     while(size)
//     {
//         while( U1STAbits.UTXBF);    // wait while TX buffer full
//         U1TXREG = *buffer;          // send single character to transmit buffer
//         buffer++;                   // transmit next character on following loop
//         size--;                     // loop until all characters sent (when size = 0)
//     }

//     while( !U1STAbits.TRMT);        // wait for last transmission to finish

//     return 0;
// }

// unsigned int SerialReceive1(char *buffer, unsigned int max_size)
// {
//     unsigned int num_char = 0;

//     while(num_char < max_size)
//     {
//         while( !U1STAbits.URXDA);   // wait until data available in RX buffer
//         *buffer = U1RXREG;          // empty contents of RX buffer into *buffer pointer

//         // insert nul character to indicate end of string
//         if( *buffer == '\n')
//         {
//             *buffer = '\0';     
//             break;
//         }

//         buffer++;
//         num_char++;
//     }

//     return num_char;
// }

// // Copied from here: https://forum.microchip.com/s/topic/a5C3l000000MRVAEA4/t335776
// void delayus(uint16_t uiuSec)
// {
//     uint32_t ulEnd, ulStart;
//     ulStart = _CP0_GET_COUNT();
//     ulEnd = ulStart + (SYSCLK / 2000000) * uiuSec;
//     if(ulEnd > ulStart)
//         while(_CP0_GET_COUNT() < ulEnd);
//     else
//         while((_CP0_GET_COUNT() > ulStart) || (_CP0_GET_COUNT() < ulEnd));
// }

// unsigned int SerialReceive1_timeout(char *buffer, unsigned int max_size)
// {
//     unsigned int num_char = 0;
//     int timeout_cnt;

//     while(num_char < max_size)
//     {
//         timeout_cnt=0;
//         while(1)
//         {
//             if(U1STAbits.URXDA) // check if data is available in RX buffer
//             {
//                 timeout_cnt=0;
//                 *buffer = U1RXREG; // copy RX buffer into *buffer pointer
//                 break;
//             }
//             if(++timeout_cnt==200) // 200 * 100us = 20 ms
//             {
//                 *buffer = '\n';
//                 break;
//             }
//             delayus(100);
//         }

//         // insert nul character to indicate end of string
//         if( *buffer == '\n')
//         {
//             *buffer = '\0';     
//             break;
//         }

//         buffer++;
//         num_char++;
//     }

//     return num_char;
// }

// // Use the core timer to wait for 1 ms.
// void wait_1ms(void)
// {
//     unsigned int ui;
//     _CP0_SET_COUNT(0); // resets the core timer count

//     // get the core timer count
//     while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
// }
// void waitms(int len)
// {
//     while(len--) wait_1ms();
// }
// void delayms(int len)
// {
//     while(len--) wait_1ms();
// }

// void ClearFIFO (void)
// {
//     unsigned char c;
//     U1STA = 0x1400;     // enable TX and RX, clear FIFO
//     while(U1STAbits.URXDA) c=U1RXREG;
// }

// void SendATCommand (char * s)
// {
//     char buff[40];
//     printf("Command: %s", s);
//     LATB &= ~(1<<14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
//     delayms(10);
//     SerialTransmit1(s);
//     U1STA = 0x1400;     // enable TX and RX, clear FIFO
//     SerialReceive1(buff, sizeof(buff)-1);
//     LATB |= 1<<14; // 'SET' pin of JDY40 to 1 is normal operation mode.
//     delayms(10);
//     printf("Response: %s\n", buff);
// }

// void ReceptionOff (void)
// {
//     LATB &= ~(1<<14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
//     delayms(10);
//     SerialTransmit1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing.
//     delayms(10);
//     ClearFIFO();
//     LATB |= 1<<14; // 'SET' pin of JDY40 to 1 is normal operation mode.
// }



// void main(void)
// {
//     // init motor
//     volatile unsigned long t=0;
//     int adcval;
//     long int v;
//     unsigned char LED_toggle=0;
//     float up,down,left,right,button_auto,button_manual,button_coin;    
//     ConfigurePins();
//     SetupTimer1();
//     ADCConf(); // Configure ADC
//     char *token;
//     int values[5] = {0,0,0,0,0}; // To store extracted digits
//     int speedx,speedy;
//     float Perimeter1,Perimeter2;
//     int i = 0;
//     double PerimeterPeakVal = 12345; //update to correct value later
//     int searchFlag = 1;

//     /*Adding ADC definition*/
//     int adcval1,adcval2;

//     char buff[80];
//     int cnt=0;
//     char c;
//     /*Metal dector Configuration*/

//     //for servo
//     int servo_state;

//     long int count;
//     float T, f;
//     float r;
//     float ct,c1,c2;
//     c1=0.00000001; //10nf
//     c2 = 0.0000001; //100nf
//     float Inductor;
    
//     DDPCON = 0;
//     CFGCON = 0;

//     UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
//     UART1Configure(9600);  // Configure UART1 to communicate with JDY40 with a baud rate of 9600

//     // RB14 is connected to the 'SET' pin of the JDY40.  Configure as output:
//     ANSELB &= ~(1<<14); // Set RB14 as a digital I/O
//     TRISB &= ~(1<<14);  // configure pin RB14 as output
//     LATB |= (1<<14);    // 'SET' pin of JDY40 to 1 is normal operation mode

//     delayms(500); // Give putty time to start before we send stuff.
//     printf("\r\nJDY40 test program. PIC32 behaving as Slave.\r\n");

//     ReceptionOff();
    
//     // To check configuration (if needed)
//     SendATCommand("AT+VER\r\n");
//     SendATCommand("AT+BAUD\r\n");
//     SendATCommand("AT+RFID\r\n");
//     SendATCommand("AT+DVID\r\n");
//     SendATCommand("AT+RFC\r\n");
//     SendATCommand("AT+POWE\r\n");
//     SendATCommand("AT+CLSS\r\n");

//     // We should select an unique device ID.  The device ID can be a hex
//     // number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
//     SendATCommand("AT+DVIDBEEF\r\n");  
//     SendATCommand("AT+RFC030\r\n");

//     cnt=0;
//     /*Initialization*/
//     up=0; 
//     down=0;
//     right=0;
//     left=0;
//     button_auto=0;
//     button_manual=0;
//     button_coin=0;
//     speedx=1;
//     speedy=1;

//     while(1)
//     {	

//         /*NEW CODE ADDED HERE */
//         //motor1 is right motor
//         //motor2 is left motor

//         if (button_auto && !button_manual) {
//             //use case statements instead

//             adcval1 = ADCRead(4); // note that we call pin AN4 (RB2) by it's analog number pin 6 
//             Perimeter1=(adcval1*3290L)/1023L; // 3.290 is VDD (volts)
//             adcval2 = ADCRead(5); // note that we call pin AN4 (RB2) by it's analog number pin 6 
//             Perimeter2=(adcval2*3290L)/1023L; // 3.290 is VDD (volts)

//             switch(movement_state) {
//                 case START:
//                     movement_state = FORWARD;
//                     __builtin_disable_interrupts();
//                     //set the motors for forward when we enter FORWARD state
//                     ISR_pwm1 = 100; //motor1
//                     ISR_pwm2 = 1; //motor1
//                     ISR_pwm3 = 100; //motor2
//                     ISR_pwm4 = 1; //motor2
//                     __builtin_enable_interrupts();
//                     break;

//                 case FORWARD:
//                     if (Perimeter1 > PerimeterPeakVal || Perimeter2 > PerimeterPeakVal) {
//                         movement_state = BACKWARD; //need to move backward now
//                         state_start_time = _CP0_GET_COUNT(); //takes snapshot of current time
//                         //reverse motors, disbale/enable interrupts to prevent pwm abnormalities
//                         __builtin_disable_interrupts();
//                         //set the motors in reverse for when we enter BACKWARD state
//                         ISR_pwm1 = 1; //motor1
//                         ISR_pwm2 = 100; //motor1
//                         ISR_pwm3 = 1; //motor2
//                         ISR_pwm4 = 100; //motor2
//                         __builtin_enable_interrupts();
//                     } else { //if perimeter is not detected, check for metal detection
                       
//                         count=GetPeriod(100);
//                         if (count>0) {
//                             T=(count*2.0)/(SYSCLK*100.0);
//                             f=1/T;
//                             ct=(c1*c1)/(c1+c2);
//                             //Calculation of Inductance
//                             Inductor=1/(39.4784*f*f*ct); // L=1/(4*pi^2*f^2*C) 4pi^2~39.4784
                            
//                             if (Inductor < 0.001) { //threshold value
//                                 movement_state = COIN_DETECTED;
//                                 coin_substate = COIN_STOP; //sets up substate for coinstates
//                                 coin_start_time = _CP0_GET_COUNT(); //takes snapshot for when coin was found
//                             }
//                         }
//                     } break;

//                 case BACKWARD:
//                     if (ELAPSED_TIME_MS(state_start_time) >= BACKWARD_TIME) {
//                         if (searchFlag) {
//                             movement_state = RIGHT_45;
//                             state_start_time = _CP0_GET_COUNT(); //takes snapshot of current time
//                             __builtin_disable_interrupts();
//                             //set the motors for right turn when we enter RIGHT_45 state
//                             ISR_pwm1 = 100; //motor1
//                             ISR_pwm2 = 1; //motor1
//                             ISR_pwm3 = 1; //motor2
//                             ISR_pwm4 = 100; //motor2
//                             __builtin_enable_interrupts();
//                         } else {
//                             movement_state = LEFT_90;
//                             state_start_time = _CP0_GET_COUNT(); //takes snapshot of current time
//                             __builtin_disable_interrupts();
//                             //set the motors for left turn when we enter LEFT_90 state
//                             ISR_pwm1 = 1; //motor1
//                             ISR_pwm2 = 100; //motor1
//                             ISR_pwm3 = 100; //motor2
//                             ISR_pwm4 = 1; //motor2
//                             __builtin_enable_interrupts();
//                         }
//                         searchFlag = !searchFlag;
//                     } break;
                
//                 case RIGHT_45:
//                     if (ELAPSED_TIME_MS(state_start_time) >= RIGHT_45_TIME) {
//                         movement_state = FORWARD;
//                         __builtin_disable_interrupts();
//                         //set the motors for forward when we enter FORWARD state
//                         ISR_pwm1 = 100; //motor1
//                         ISR_pwm2 = 1; //motor1
//                         ISR_pwm3 = 100; //motor2
//                         ISR_pwm4 = 1; //motor2
//                         __builtin_enable_interrupts();
//                     } break;
                
//                 case LEFT_90:
//                     if (ELAPSED_TIME_MS(state_start_time) >= LEFT_90_TIME) {
//                         movement_state = FORWARD;
//                         __builtin_disable_interrupts();
//                         //set the motors for forward when we enter FORWARD state
//                         ISR_pwm1 = 100; //motor1
//                         ISR_pwm2 = 1; //motor1
//                         ISR_pwm3 = 100; //motor2
//                         ISR_pwm4 = 1; //motor2
//                         __builtin_enable_interrupts();
//                     } break;

//                 case COIN_DETECTED:
//                     switch (coin_start_time) { //nested case statements for coin stuff
//                         case COIN_STOP:
//                             //stop robot for 250 ms
//                             __builtin_disable_interrupts();
//                             ISR_pwm1 = 1;
//                             ISR_pwm2 = 1;
//                             ISR_pwm3 = 1;
//                             ISR_pwm4 = 1;
//                             __builtin_enable_interrupts();

//                             //pause 250ms before servo
//                             if (ELAPSED_TIME_MS(coin_start_time) >= 250) {
//                                 coin_substate = COIN_SERVO;
//                                 coin_start_time = _CP0_GET_COUNT();
//                                 servo_state = 0; ///simple reset
//                             } break;

//                         case COIN_SERVO:
//                             //servo update rate is 10ms
//                             if (ELAPSED_TIME_MS(coin_start_time) >= SERVO_UPDATE_INTERVAL) {
//                                 coin_start_time = _CP0_GET_COUNT(); //take another snapshot of current time

//                                 switch (servo_state) {
//                                     case 0:
//                                         ISR_pwm5 = 240;
//                                         ISR_pwm6 = 200;
//                                         if (ELAPSED_TIME_MS(coin_start_time) >= 2000) {
//                                             servo_state = 1;
//                                         }
//                                         break;
//                                     case 1:
//                                         ISR_pwm5 = 200;
//                                         if (ELAPSED_TIME_MS(state_start_time) >= SERVO_STEP_DELAY) {
//                                             servo_state = 2;
//                                             state_start_time = _CP0_GET_COUNT(); //take snapshot
//                                         }
//                                         break;
//                                     case 2:
//                                         if (ISR_pwm6 > 100) {
//                                             ISR_pwm6--;
//                                         } else {
//                                                 LATAbits.LATA0 = 1;  // magnet on
//                                             servo_state = 3;
//                                             state_start_time = _CP0_GET_COUNT();
//                                         }
//                                         break;
                    
//                                     case 3: 
//                                         if (ISR_pwm5 < 240) {  
//                                             ISR_pwm5++;
//                                         } else if (ELAPSED_TIME_MS(state_start_time) >= SERVO_STEP_DELAY) {
//                                             servo_state = 4;
//                                             state_start_time = _CP0_GET_COUNT();
//                                         }
//                                         break;
                                        
//                                     case 4: 
//                                         if (ISR_pwm6 < 180) { 
//                                             ISR_pwm6++;
//                                         } else if (ELAPSED_TIME_MS(state_start_time) >= SERVO_STEP_DELAY) {
//                                             servo_state = 5;
//                                             state_start_time = _CP0_GET_COUNT();
//                                         }
//                                         break;

//                                     case 5: // Release coin
//                                         if (ISR_pwm5 > 60) { 
//                                             ISR_pwm5--;
//                                         } else {
//                                             LATAbits.LATA0 = 0;  //magnet off
//                                             servo_state = 6;
//                                             state_start_time = _CP0_GET_COUNT();
//                                         }
//                                         break;
                                        
//                                     case 6: // back to default position
//                                         ISR_pwm5 = 240;  
//                                         ISR_pwm6 = 180;  
//                                         if (ELAPSED_TIME_MS(state_start_time) >= SERVO_STEP_DELAY) {
//                                             coin_substate = COIN_RESUME;
//                                         }
//                                         break;

//                                 }
//                             } break;

//                         case COIN_RESUME:
//                             movement_state = FORWARD;
//                             __builtin_disable_interrupts();
//                             //set the motors for forward when we enter FORWARD state
//                             ISR_pwm1 = 100; //motor1
//                             ISR_pwm2 = 1; //motor1
//                             ISR_pwm3 = 100; //motor2
//                             ISR_pwm4 = 1; //motor2
//                             __builtin_enable_interrupts();
//                             break;
                        

//                     } break;
//             }
//         }


//         //
//         /*NEW CODE ADDED HERE ENDS */

//         //If issue look at this informatoin
//         //manual mode code
//         else if (!button_auto && button_manual) {
            
//             printf("Speedx: %d, Speedy: %d,Perimeter1: %f,Perimeter2: %f,Inductor: %f\r\n", speedx, speedy,Perimeter1,Perimeter2,Inductor);
        
//             if(speedy>1){
//                 ISR_pwm1 = speedy; //pin 9 - motor1 forawrd
//                 ISR_pwm2 = 1; //pin 10  -motor1 backward
//                 ISR_pwm3 = speedy; //pin 11 -motor2 forward
//                 ISR_pwm4 = 1; //pin 12 -motor2 backward

//             }
//             else if(speedy<0){
//                 ISR_pwm1 = 1; //pin 9 - motor1 forawrd
//                 ISR_pwm2 = -1*speedy; //pin 10  -motor1 backward
//                 ISR_pwm3 = 1; //pin 11 -motor2 forward
//                 ISR_pwm4 = -1*speedy; //pin 12 -motor2 backward

//             }
//             else if(speedx<0){ 
//                 ISR_pwm1 = speedx; //pin 9 - motor1 forawrd
//                 ISR_pwm2 = 1; //pin 10  -motor1 backward
//                 ISR_pwm3 = 1; //pin 11 -motor2 forward
//                 ISR_pwm4 = speedx; //pin 12 -motor2 backward
//             }
//             else if(speedx>1){ 
//                 ISR_pwm1 = 1; //pin 9 - motor1 forawrd
//                 ISR_pwm2 = -1*speedx; //pin 10  -motor1 backward
//                 ISR_pwm3 = -1*speedx; //pin 11 -motor2 forward
//                 ISR_pwm4 = 1; //pin 12 -motor2 backward
//             }
//             else{
//                 ISR_pwm1 = 1; //pin 9 - motor1 forawrd
//                 ISR_pwm2 = 1; //pin 10  -motor1 backward
//                 ISR_pwm3 = 1; //pin 11 -motor2 forward
//                 ISR_pwm4 = 1; //pin 12 -motor2 backward
//             }
//         }
    
//         //communication and updating variables
//         if(U1STAbits.URXDA) // Something has arrived
//             {

                
                
//                 c=U1RXREG;
//                 if(c=='!') // Master is sending message
//                 {
//                     //SerialReceive1_timeout(buff, sizeof(buff)-1);
                    

//                     SerialReceive1(buff, sizeof(buff)-1);
                    
//                     i = 0;
//                     token = strtok(buff, " ");
//                     while(token != NULL && i < 5)
//                     {
//                         values[i] = atoi(token);
//                         token = strtok(NULL, " ");
//                         i++;
//                     }
//                     speedx = values[0];
//                     speedy = values[1];
//                     button_auto = values[2];
//                     button_manual = values[3];
//                     button_coin = values[4];
    
                    
                    
//                     if(strlen(buff)==13)
//                     {

//                         printf("Master says: %s\r\n", buff);
//                     }
//                     else
//                     {
//                         // Clear the receive 8-level FIFO of the PIC32MX, so we get a fresh reply from the slave
//                         ClearFIFO();
//                         printf("*** BAD MESSAGE ***: %s\r\n", buff);
//                     }				
//                 }
//                 else if(c=='@') // Master wants slave data
//                 {
//                     sprintf(buff, "%05u\n", cnt);
//                     cnt++;
//                     delayms(5); // The radio seems to need this delay...
//                     SerialTransmit1(buff);
//                 }
//                 else
//                 {
//                     // Clear the receive 8-level FIFO of the PIC32MX, so we get a fresh reply from the slave
//                     ClearFIFO();
//                 }
                
//             }    
//     }
// }
#include <stdint.h>
#include <XC.h>
#include <sys/attribs.h>

#pragma config FNOSC = FRCPLL
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLMUL = MUL_20
#pragma config FPLLODIV = DIV_2
#pragma config FWDTEN = OFF

#define CS      LATBbits.LATB0
#define DIN     LATBbits.LATB11
#define CLK     LATBbits.LATB14

const uint16_t SMILEY[] = {
  0x013C, 0x0242, 0x03A5, 0x0481,
  0x0581, 0x06A5, 0x0742, 0x083C
};

void SPI_Init() {
  // Disable analog functions using bitmasks
  ANSELBCLR = 1 << 0;  // RB0 (CS)
  ANSELBCLR = 1 << 11; // RB11 (DIN)
  ANSELBCLR = 1 << 14; // RB14 (CLK)

  TRISBbits.TRISB0 = 0;   // CS as output
  TRISBbits.TRISB11 = 0;  // DIN as output
  TRISBbits.TRISB14 = 0;  // CLK as output

  // PPS Configuration
  RPB11Rbits.RPB11R = 0b0011; // SDO1 on RB11

  SPI1CON = 0;
  SPI1BRG = 0;
  SPI1CONbits.MSTEN = 1;
  SPI1CONbits.MODE16 = 1;
  SPI1CONbits.CKP = 0;
  SPI1CONbits.ON = 1;
  CS = 1;
}

void MAX7219_Write(uint16_t data) {
  CS = 0;
  SPI1BUF = data;
  while (SPI1STATbits.SPIBUSY);
  CS = 1;
}

void MAX7219_Init() {
  MAX7219_Write(0x0C01);
  MAX7219_Write(0x0A07);
  MAX7219_Write(0x0900);
  MAX7219_Write(0x0B07);
}

int main(void) {
  SPI_Init();
  MAX7219_Init();	
  

  int i; // Declare outside loop for C89 compatibility
  for (i = 0; i < 8; i++) {
    MAX7219_Write(SMILEY[i]);
  }

  while (1);
  return 0;
}