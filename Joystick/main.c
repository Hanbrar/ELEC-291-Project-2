#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../Common/Include/stm32l051xx.h"
#include "adc.h"

#define F_CPU 32000000L

// LQFP32 pinout with the pins that can be analog inputs.  This code uses ADC_IN9.
//                 ----------
//           VDD -|1       32|- VSS
//          PC14 -|2       31|- BOOT0
//          PC15 -|3       30|- PB7
//          NRST -|4       29|- PB6
//          VDDA -|5       28|- PB5
// (ADC_IN0) PA0 -|6       27|- PB4
// (ADC_IN1) PA1 -|7       26|- PB3
// (ADC_IN2) PA2 -|8       25|- PA15
// (ADC_IN3) PA3 -|9       24|- PA14
// (ADC_IN4) PA4 -|10      23|- PA13
// (ADC_IN5) PA5 -|11      22|- PA12
// (ADC_IN6) PA6 -|12      21|- PA11
// (ADC_IN7) PA7 -|13      20|- PA10 (Reserved for RXD)
// (ADC_IN8) PB0 -|14      19|- PA9  (Reserved for TXD)
// (ADC_IN9) PB1 -|15      18|- PA8  (LED+1k)
//           VSS -|16      17|- VDD
//                 ----------

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

void Configure_Pins(void)
{
    // Configure the pin used for a blinking LED: PA8 (pin 18)
    RCC->IOPENR |= BIT0;  // Peripheral clock enable for Port A
    GPIOA->MODER = (GPIOA->MODER & ~(BIT17 | BIT16)) | BIT16;  // Set PA8 as output

    // Configure analog inputs (PB1 and PB14) and button input (PB13)
    RCC->IOPENR |= BIT1;  // Peripheral clock enable for Port B

    // Set PB1 (pin 15) and PB14 (pin 14) to analog mode
    GPIOB->MODER |= (BIT2 | BIT3);    // Analog mode for PB1
    GPIOB->MODER |= (BIT28 | BIT29);  // Analog mode for PB14

    // Configure PB13 (pin 13) as input with pull-up resistor
    GPIOB->MODER &= ~(BIT26 | BIT27);      // Input mode for PB13
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(BIT26 | BIT27)) | BIT26;  // Pull-up for PB13

    // Enable ADC clock (required for ADC functionality)
    RCC->AHBENR |= BIT28;  // Enable ADC clock (adjust bit if needed for your MCU)
}

void main(void)
{
	float a7,a8,a9;
	int j7,j8,j9;
	
    delayms(500); // Give PuTTY time to start
	printf("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
	printf("STM32L051 ADC Test.  Analog input is PB1 (pin 15).\r\n");
	
	Configure_Pins();
	initADC();
	
	while(1)
	{


        j7 = readADC(ADC_CHSELR_CHSEL7);
        j8 = readADC(ADC_CHSELR_CHSEL8);
        j9 = readADC(ADC_CHSELR_CHSEL9);

        a7 = (j7 * 3.3f) / 0x1000;
        a8 = (j8 * 3.3f) / 0x1000;
        a9 = (j9 * 3.3f) / 0x1000;

        printf("Button=%s\r ADC8=0x%04x (%5.3fV) ADC9=0x%04x (%5.3fV)\r", 
               j7, adc_v7, j8, adc_v8, j9, adc_v9);

		fflush(stdout);
		GPIOA->ODR ^= BIT8; // Complement PA8 (pin 18)
		delayms(500);
	}
}
