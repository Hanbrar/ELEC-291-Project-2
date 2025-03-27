#include "../Common/Include/stm32l051xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../Common/Include/serial.h"
#include "UART2.h"
#include <math.h>

#define SYSCLK 32000000L
#define DEF_F 15000L

// LQFP32 pinout
//             ----------
//       VDD -|1       32|- VSS
//      PC14 -|2       31|- BOOT0
//      PC15 -|3       30|- PB7
//      NRST -|4       29|- PB6
//      VDDA -|5       28|- PB5
//       PA0 -|6       27|- PB4
//       PA1 -|7       26|- PB3
//       PA2 -|8       25|- PA15 (Used for RXD of UART2, connects to TXD of JDY40)
//       PA3 -|9       24|- PA14 (Used for TXD of UART2, connects to RXD of JDY40)
//       PA4 -|10      23|- PA13 (Used for SET of JDY40)
//       PA5 -|11      22|- PA12
//       PA6 -|12      21|- PA11
//       PA7 -|13      20|- PA10 (Reserved for RXD of UART1)
//       PB0 -|14      19|- PA9  (Reserved for TXD of UART1)
//       PB1 -|15      18|- PA8  (pushbutton)
//       VSS -|16      17|- VDD
//             ----------

#define F_CPU 32000000L

// define push buttons
// PB3        PB5          PB6
// Auto_mode  Manual_mode  coin_picking


// Master JDY-40
// Uses SysTick to delay <us> micro-seconds.


void Delay_us(unsigned char us)
{
	// For SysTick info check the STM32L0xxx Cortex-M0 programming manual page 85.
	SysTick->LOAD = (F_CPU/(1000000L/us)) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Delay_us(250);
}

void Hardware_Init(void)
{
	GPIOA->OSPEEDR=0xffffffff; // All pins of port A configured for very high speed! Page 201 of RM0451

	RCC->IOPENR |= BIT0; // peripheral clock enable for port A

    GPIOA->MODER = (GPIOA->MODER & ~(BIT27|BIT26)) | BIT26; // Make pin PA13 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.

	GPIOA->MODER &= ~(BIT16 | BIT17); // Make pin PA8 input
	// Activate pull up for pin PA8:
	GPIOA->PUPDR |= BIT16; 
	GPIOA->PUPDR &= ~(BIT17);
}

void Configure_Buttons(void)
{
    RCC->IOPENR |= BIT1; // Enable clock for Port B

    // Clear mode bits for PB3, PB5, and PB6 (set as input: 00)
    GPIOB->MODER &= ~(0x3 << (3 * 2)); // PB3
    GPIOB->MODER &= ~(0x3 << (5 * 2)); // PB5
    GPIOB->MODER &= ~(0x3 << (6 * 2)); // PB6

    // Enable internal pull-ups for PB3, PB5, and PB6 (set to 01)
    GPIOB->PUPDR &= ~(0x3 << (3 * 2));
    GPIOB->PUPDR |=  (0x1 << (3 * 2));

    GPIOB->PUPDR &= ~(0x3 << (5 * 2));
    GPIOB->PUPDR |=  (0x1 << (5 * 2));

    GPIOB->PUPDR &= ~(0x3 << (6 * 2));
    GPIOB->PUPDR |=  (0x1 << (6 * 2));
}  

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	GPIOA->ODR &= ~(BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2(s);
	egets2(buff, sizeof(buff)-1);
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.
	waitms(10);
	printf("Response: %s", buff);
}

void ReceptionOff (void)
{
	GPIOA->ODR &= ~(BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	waitms(10);
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.
	while (ReceivedBytes2()>0) egetc2(); // Clear FIFO
}

/*Joystick/ADC Reading */

void Configure_Pins (void)
{

    // Configure PB1 (pin 15) as an analog input
    RCC->IOPENR |= BIT1;  // Enable clock for GPIOB
    GPIOB->MODER |= (BIT2 | BIT3);  // Set PB1 to analog mode

    // Configure PB13 (pin 13) as a digital input with pull-up resistor (Button)
    GPIOB->MODER &= ~(BIT26 | BIT27);  // Set PB13 as input mode
	GPIOB->PUPDR |= BIT26; // Enable internal pull-up for PB13
	GPIOB->PUPDR &= ~BIT27; // Ensure pull-down is disabled

    // Enable ADC clock
    RCC->AHBENR |= BIT28;  // Enable ADC clock
}

void wait_1ms(void)
{
	// For SysTick info check the STM32l0xxx Cortex-M0 programming manual.
	SysTick->LOAD = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	//SysTick->CTRL = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void delayms(int len)
{
	while(len--) wait_1ms();
}


/*Now ADC functions*/


void initADC(void)
{
	RCC->APB2ENR |= BIT9; // peripheral clock enable for ADC (page 175 or RM0451)

	// ADC clock selection procedure (page 746 of RM0451)
	/* (1) Select PCLK by writing 11 in CKMODE */
	ADC1->CFGR2 |= ADC_CFGR2_CKMODE; /* (1) */
	
	// ADC enable sequence procedure (page 745 of RM0451)
	/* (1) Clear the ADRDY bit */
	/* (2) Enable the ADC */
	/* (3) Wait until ADC ready */
	ADC1->ISR |= ADC_ISR_ADRDY; /* (1) */
	ADC1->CR |= ADC_CR_ADEN; /* (2) */
	if ((ADC1->CFGR1 & ADC_CFGR1_AUTOFF) == 0)
	{
		while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (3) */
		{
			/* For robust implementation, add here time-out management */
		}
	}	

	// Calibration code procedure (page 745 of RM0451)
	/* (1) Ensure that ADEN = 0 */
	/* (2) Clear ADEN */
	/* (3) Set ADCAL=1 */
	/* (4) Wait until EOCAL=1 */
	/* (5) Clear EOCAL */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
		ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	ADC1->CR |= ADC_CR_ADCAL; /* (3) */
	while ((ADC1->ISR & ADC_ISR_EOCAL) == 0) /* (4) */
	{
		/* For robust implementation, add here time-out management */
	}
	ADC1->ISR |= ADC_ISR_EOCAL; /* (5) */
}

int readADC(unsigned int channel)
{
	// Single conversion sequence code example - Software trigger (page 746 of RM0451)
	/* (1) Select HSI16 by writing 00 in CKMODE (reset value) */
	/* (2) Select the auto off mode */
	/* (3) Select channel */
	/* (4) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than17.1us */
	/* (5) Wake-up the VREFINT (only for VRefInt) */
	//ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */
	ADC1->CFGR1 |= ADC_CFGR1_AUTOFF; /* (2) */
	ADC1->CHSELR = channel; /* (3) */
	ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* (4) */
	if(channel==ADC_CHSELR_CHSEL17)
	{
		ADC->CCR |= ADC_CCR_VREFEN; /* (5) */
	}
	
	/* Performs the AD conversion */
	ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
	while ((ADC1->ISR & ADC_ISR_EOC) == 0) /* wait end of conversion */
	{
		/* For robust implementation, add here time-out management */
	}

	return ADC1->DR; // ADC_DR has the 12 bits out of the ADC
}






/*#################*/





int main(void)
{
	char buff[80];
    int timeout_cnt=0;

	int up = 0;
	int down = 0;
	int left = 0;
	int right = 0;
    int button_auto = 0;
    int button_manual = 0;
    int button_coin = 0;


    int j8, j9, button_state;

	Hardware_Init();
	initUART2(9600);

	Configure_Pins();
	initADC();
	
	waitms(1000); // Give putty some time to start.
	printf("\r\nJDY-40 Master test\r\n");

	ReceptionOff();
	
	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");
	
	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVIDABBA\r\n");
	 
	while(1)
	{
		up = 0;
		down = 0;
		right = 0;
		left = 0;

		j8 = readADC(ADC_CHSELR_CHSEL8);
        j9 = readADC(ADC_CHSELR_CHSEL9);

		if(j8>2900){
			right = 1;
		} else if (j8>1000) {
			
		} else {
			left = 1;
		}

		if(j9>2900){
			up = 1;
		} else if (j9>1000) {
			
		} else {
			down = 1;
		}

        if (!(GPIOB->IDR & (1 << 3))) {  // PB3 pressed (Auto_mode)
            button_auto = 1;
        }
        if (!(GPIOB->IDR & (1 << 5))) {  // PB5 pressed (Manual_mode)
            button_manual = 1;
        }
        if (!(GPIOB->IDR & (1 << 6))) {  // PB6 pressed (coin_picking)
            button_coin = 1;
        }
        
        


		sprintf(buff, "%d %d %d %d %d %d %d\n", up, down, left, right, button_auto, button_manual, button_coin);
		eputc2('!'); // Send a message to the slave. First send the 'attention' character which is '!'
		// Wait a bit so the slave has a chance to get ready
		waitms(5); // This may need adjustment depending on how busy is the slave
		eputs2(buff); // Send the test message
		
		
		waitms(5); // This may need adjustment depending on how busy is the slave

		eputc2('@'); // Request a message from the slave
		
		timeout_cnt=0;
		while(1)
		{
			if(ReceivedBytes2()>0) break; // Something has arrived
			if(++timeout_cnt>250) break; // Wait up to 25ms for the repply
			Delay_us(100); // 100us*250=25ms
		}
		
		if(ReceivedBytes2()>0) // Something has arrived from the slave
		{
			egets2(buff, sizeof(buff)-1);
			printf("Slave says: %s\r", buff);
		}
		else // Timed out waiting for reply
		{
			printf("NO RESPONSE\r\n", buff);
		}
		
		waitms(50);  // Set the information interchange pace: communicate about every 50ms
		fflush(stdout);
        GPIOA->ODR ^= BIT8; // Toggle PA8 (LED)
        delayms(500);
	}


}
