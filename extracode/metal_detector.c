/*The code is in stm */

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

#define COIN_DETECTOR_H

#include "stm32l051xx.h"
#include <stdint.h>

// Define the ADC channel used for the coin sensor.
// Adjust this definition if your coin sensor is connected to a different channel.
#define COIN_SENSOR_ADC_CHANNEL    ADC_CHSELR_CHSEL0

// Initializes the coin detector hardware (GPIO configuration for the sensor).
void CoinDetector_Init(void);

// Reads the ADC value from the coin sensor.
uint16_t CoinDetector_ReadADC(void);

// Checks whether a coin is detected.
// Returns 1 if a coin is detected, 0 otherwise.
int CoinDetector_IsCoinDetected(void);


// Assume that the main project provides the ADC conversion routine.
extern int readADC(unsigned int channel);

void CoinDetector_Init(void)
{
    // Ensure the clock for GPIOA is enabled (if not already done in your Hardware_Init).
    RCC->IOPENR |= BIT0;
    
    // Configure PA0 as analog input.
    // For STM32, setting MODER bits to '11' puts the pin in analog mode.
    GPIOA->MODER |= (0x3 << (0 * 2));
    
    // Disable pull-up and pull-down resistors on PA0.
    GPIOA->PUPDR &= ~(0x3 << (0 * 2));
    
    // (Optionally) you could initialize the ADC here if it hasn’t been initialized elsewhere.
}

uint16_t CoinDetector_ReadADC(void)
{
    // Use the project's ADC reading function.
    return (uint16_t)readADC(COIN_SENSOR_ADC_CHANNEL);
}

int CoinDetector_IsCoinDetected(void)
{
    uint16_t adcValue = CoinDetector_ReadADC();
    
    // Define a threshold value based on calibration.
    // For example, with a 12-bit ADC (range 0-4095), you might find that a value above 2000
    // indicates the presence of a coin. Adjust this value based on your sensor's behavior.
    const uint16_t threshold = 2000;
    
    if(adcValue > threshold)
    {
        // Coin detected.
        return 1;
    }
    
    return 0;
}