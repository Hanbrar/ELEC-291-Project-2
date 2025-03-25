#include "../Common/Include/stm32l051xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../Common/Include/serial.h"
#include "UART2.h"

#define SYSCLK 32000000L
#define DEF_F 15000L

// LQFP32 pinout
//             ----------
//       VDD -|1       32|- VSS
//      PC14 -|2       31|- BOOT0
//      PC15 -|3       30|- PB7
//      NRST -|4       29|- PB6 this
//      VDDA -|5       28|- PB5 this 
//       PA0 -|6       27|- PB4
//       PA1 -|7       26|- PB3 this
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

// define push buttons
// PB3        PB5          PB6
// Auto_mode  Manual_mode  coin_picking



#define F_CPU 32000000L
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

int main(void)
{
	char buff[80];
    int timeout_cnt=0;
    int cont1=0, cont2=100;

	Hardware_Init();
	initUART2(9600);
	Configure_Buttons();

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
	 
	while (1)
    {
        // Poll the push buttons (active low due to pull-ups).
        char btn_state[32] = "";
        if (!(GPIOB->IDR & (1 << 3))) {  // PB3 pressed (Auto_mode)
            strcat(btn_state, "Auto_mode ");
        }
        if (!(GPIOB->IDR & (1 << 5))) {  // PB5 pressed (Manual_mode)
            strcat(btn_state, "Manual_mode ");
        }
        if (!(GPIOB->IDR & (1 << 6))) {  // PB6 pressed (coin_picking)
            strcat(btn_state, "coin_picking ");
        }
        
        // If any button is pressed, send the button state over UART.
        if (strlen(btn_state) > 0)
        {
            sprintf(buff, "Buttons: %s\n", btn_state);
            eputs2(buff);
            printf("%s", buff);
        }
        else
        {
            // Otherwise, send a test message.
            sprintf(buff, "%03d,%03d\n", cont1, cont2);
            eputc2('!');  // Send attention character.
            waitms(5);
            eputs2(buff); // Send test message.
            
            if (++cont1 > 200)
                cont1 = 0;
            if (++cont2 > 200)
                cont2 = 0;
        }

        waitms(5);  // Small delay

        eputc2('@'); // Request a message from the slave.

        timeout_cnt = 0;
        while (1)
        {
            if (ReceivedBytes2() > 0)
                break; // Exit if data is received.
            if (++timeout_cnt > 250)
                break; // Timeout after ~25ms.
            Delay_us(100);
        }

        if (ReceivedBytes2() > 0)
        {
            egets2(buff, sizeof(buff)-1);
            if (strlen(buff) == 6)  // Check for expected message length (example: 5 characters + newline)
                printf("Slave says: %s\r", buff);
            else
                printf("*** BAD MESSAGE ***: %s\r", buff);
        }
        else
        {
            printf("NO RESPONSE\r\n");
        }

        waitms(50);  // Pace communications (~50ms between cycles)
    }
}


}
// pb6 pb5 pb3 