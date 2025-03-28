#include "../Common/Include/stm32l051xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../Common/Include/serial.h"
#include "UART2.h"
#include <math.h>

#define SYSCLK 32000000L
#define DEF_F 15000L
#define F_CPU 32000000L

// LQFP32 pinout
//             ----------
//       VDD -|1       32|- VSS
//      PC14 -|2       31|- BOOT0
//      PC15 -|3       30|- PB7
//      NRST -|4       29|- PB6
//      VDDA -|5       28|- PB5
//       PA0 -|6       27|- PB4
//       PA1 -|7       26|- PB3   <-- Used for one push button (or ADC)
//       PA2 -|8       25|- PA15  (Used for RXD of UART2, connects to TXD of JDY40)
//       PA3 -|9       24|- PA14  (Used for TXD of UART2, connects to RXD of JDY40)
//       PA4 -|10      23|- PA13  (Used for SET of JDY40)
//       PA5 -|11      22|- PA12
//       PA6 -|12      21|- PA11
//       PA7 -|13      20|- PA10  (Reserved for RXD of UART1)
//       PB0 -|14      19|- PA9   (Reserved for TXD of UART1)
//       PB1 -|15      18|- PA8   (pushbutton)
//       VSS -|16      17|- VDD
//             ----------

//**************************************************************
// Delay functions
//**************************************************************
// Uses SysTick to delay <us> micro-seconds.
void Delay_us(unsigned char us)
{
    SysTick->LOAD = (F_CPU/(1000000L/us)) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while((SysTick->CTRL & BIT16)==0);
    SysTick->CTRL = 0x00;
}

void waitms (unsigned int ms)
{
    unsigned int j;
    unsigned char k;
    for(j=0; j<ms; j++)
        for (k=0; k<4; k++) Delay_us(250);
}

void wait_1ms(void)
{
    SysTick->LOAD = (F_CPU/1000L) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while((SysTick->CTRL & BIT16)==0);
    SysTick->CTRL = 0x00;
}

void delayms(int len)
{
    while(len--) wait_1ms();
}

//**************************************************************
// Hardware Initialization
//**************************************************************
void Hardware_Init(void)
{
    GPIOA->OSPEEDR = 0xffffffff; // All pins of port A set to very high speed
    RCC->IOPENR |= BIT0;         // Enable clock for port A

    // Configure PA13 as output (used for setting AT mode on JDY-40)
    GPIOA->MODER = (GPIOA->MODER & ~(BIT27|BIT26)) | BIT26;
    GPIOA->ODR |= BIT13; // Set PA13 to 1 = normal operation

    // Configure PA8 as input with pull-up (if used as a pushbutton)
    GPIOA->MODER &= ~(BIT16 | BIT17);
    GPIOA->PUPDR |= BIT16; 
    GPIOA->PUPDR &= ~(BIT17);
}

// Configure_Pins: sets up ADC input and a secondary pushbutton (if needed)
void Configure_Pins(void)
{
    // Configure PB1 (pin 15) as an analog input (if needed for other purposes)
    RCC->IOPENR |= BIT1;
    GPIOB->MODER |= (BIT2 | BIT3);

    // Configure PB13 as a digital input with pull-up (example configuration)
    GPIOB->MODER &= ~(BIT26 | BIT27);
    GPIOB->PUPDR |= BIT26;
    GPIOB->PUPDR &= ~BIT27;
}

// Configure_Buttons: set up PB3, PB5, and PB6 as inputs with internal pull-ups
// (These correspond to Auto_mode, Manual_mode, and coin_picking)
void Configure_Buttons(void)
{
    RCC->IOPENR |= BIT1; // Enable clock for Port B

    // Set PB3, PB5, and PB6 as inputs (00)
    GPIOB->MODER &= ~(0x3 << (3 * 2)); // PB3
    GPIOB->MODER &= ~(0x3 << (5 * 2)); // PB5
    GPIOB->MODER &= ~(0x3 << (6 * 2)); // PB6

    // Enable internal pull-ups (01)
    GPIOB->PUPDR &= ~(0x3 << (3 * 2));
    GPIOB->PUPDR |=  (0x1 << (3 * 2));

    GPIOB->PUPDR &= ~(0x3 << (5 * 2));
    GPIOB->PUPDR |=  (0x1 << (5 * 2));

    GPIOB->PUPDR &= ~(0x3 << (6 * 2));
    GPIOB->PUPDR |=  (0x1 << (6 * 2));
}

//**************************************************************
// ADC Functions
//**************************************************************
void initADC(void)
{
    RCC->APB2ENR |= BIT9; // Enable ADC clock

    // Select PCLK as ADC clock source
    ADC1->CFGR2 |= ADC_CFGR2_CKMODE;

    // ADC enable sequence
    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    if ((ADC1->CFGR1 & ADC_CFGR1_AUTOFF) == 0)
    {
        while ((ADC1->ISR & ADC_ISR_ADRDY) == 0){}
    }

    // Calibration procedure
    if ((ADC1->CR & ADC_CR_ADEN) != 0)
    {
        ADC1->CR |= ADC_CR_ADDIS;
    }
    ADC1->CR |= ADC_CR_ADCAL;
    while ((ADC1->ISR & ADC_ISR_EOCAL) == 0){}
    ADC1->ISR |= ADC_ISR_EOCAL;
}

int readADC(unsigned int channel)
{
    ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;
    ADC1->CHSELR = channel;
    ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;
    if(channel == ADC_CHSELR_CHSEL17)
    {
        ADC->CCR |= ADC_CCR_VREFEN;
    }
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0){}
    return ADC1->DR;
}

//**************************************************************
// JDY-40 (or jpy) Communication Functions
//**************************************************************
void SendATCommand(char * s)
{
    char buff[40];
    printf("Command: %s", s);
    GPIOA->ODR &= ~(BIT13); // Set PA13 to 0 => AT mode
    waitms(10);
    eputs2(s);
    egets2(buff, sizeof(buff)-1);
    GPIOA->ODR |= BIT13; // Back to normal mode
    waitms(10);
    printf("Response: %s", buff);
}

void ReceptionOff(void)
{
    GPIOA->ODR &= ~(BIT13); // Set AT mode
    waitms(10);
    eputs2("AT+DVID0000\r\n");
    waitms(10);
    GPIOA->ODR |= BIT13; // Return to normal operation
    while (ReceivedBytes2() > 0) 
        egetc2(); // Clear FIFO
}

//**************************************************************
// Main Function
//**************************************************************
int main(void)
{
    char buff[80];
    int timeout_cnt = 0;
    int up, down, left, right;
    int button_auto, button_manual, button_coin;
    int j8, j9;

    Hardware_Init();
    initUART2(9600);
    Configure_Pins();
    Configure_Buttons(); // <<< Added call so the push buttons are correctly set up
    initADC();

    waitms(1000); // Give time for terminal (e.g. putty) to start
    printf("\r\nJDY-40 Master test\r\n");

    ReceptionOff();

    // Check and configure JDY-40 settings via AT commands
    SendATCommand("AT+VER\r\n");
    SendATCommand("AT+BAUD\r\n");
    SendATCommand("AT+RFID\r\n");
    SendATCommand("AT+DVID\r\n");
    SendATCommand("AT+RFC\r\n");
    SendATCommand("AT+POWE\r\n");
    SendATCommand("AT+CLSS\r\n");
    SendATCommand("AT+DVIDABBB\r\n");
    SendATCommand("AT+RFC030\r\n");

    while(1)
    {
        // Reset all variables at the start of each loop
        up = 0; down = 0; left = 0; right = 0;
        button_auto = 0; button_manual = 0; button_coin = 0;

        // Read ADC channels for joystick (channels 8 and 9)
        j8 = readADC(ADC_CHSELR_CHSEL8);
        j9 = readADC(ADC_CHSELR_CHSEL9);

        if(j8 > 2900)
            right = 1;
        else if(j8 <= 1000)
            left = 1;

        if(j9 > 2900)
            up = 1;
        else if(j9 <= 1000)
            down = 1;

        // Read push buttons (PB3: Auto_mode, PB5: Manual_mode, PB6: coin_picking)
        if (!(GPIOB->IDR & (1 << 3))) 
            button_auto = 1;
        if (!(GPIOB->IDR & (1 << 5))) 
            button_manual = 1;
        if (!(GPIOB->IDR & (1 << 6))) 
            button_coin = 1;

        // Construct message to send to the slave device
        sprintf(buff, "%d %d %d %d %d %d %d\n", 
                up, down, left, right, button_auto, button_manual, button_coin);

        // Send the message using the JDY-40 protocol:
        eputc2('!');  // Attention character
        waitms(5);
        eputs2(buff);
        waitms(5);
        eputc2('@');  // Request message from slave

        timeout_cnt = 0;
        	while(1)
		{
			if(ReceivedBytes2()>5) break; // Something has arrived
			if(++timeout_cnt>240) break; // Wait up to 25ms for the repply
			Delay_us(100); // 100us*250=25ms
		}
		
		if(ReceivedBytes2()>5) // Something has arrived from the slave
		{
			egets2(buff, sizeof(buff)-1);
			if(strlen(buff)>0) // Check for valid message size (5 characters + new line '\n')
			{
				printf("Slave says: %s\r", buff);
			}
			else
			{
				printf("*** BAD MESSAGE ***: %s\r", buff);
				while(ReceivedBytes2()>0) egetc2(); // Clear FIFO
			}
		}
		else // Timed out waiting for reply
		{
			printf("NO RESPONSE \r\n", buff);
			while(ReceivedBytes2()>0) egetc2(); // Clear FIFO
		}
		
		waitms(50);  // Set the information interchange pace: communicate about every 50ms
        fflush(stdout);
        GPIOA->ODR ^= BIT8; // Toggle PA8 (if used for LED indication)
        delayms(500);
    }
}
