#include "../Common/Include/stm32l051xx.h"

volatile int Count = 0;

//-----------------------------------------
// User-configurable section
//-----------------------------------------
#define SYSCLK          32000000L
#define PWM_RESOLUTION  255  // 8-bit resolution

static uint32_t g_buzzerFrequency = 1000; // Start with 1 kHz

//-----------------------------------------
// Function Prototypes
//-----------------------------------------
void SystemClock_Init(void);
void Hardware_Init(void);
void SetBuzzerFrequency(uint32_t frequency);
void BuzzerOn(void);
void BuzzerOff(void);
void ToggleLED(void);

//-----------------------------------------
// System Clock Initialization
//-----------------------------------------
void SystemClock_Init(void) {
    // Enable HSI (16 MHz) and switch to HSI+PLL for 32 MHz
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {};
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {};

    // Configure PLL to 32 MHz (HSI/2 * 4)
    RCC->CFGR |= (RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV2);
    RCC->CR   |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {};

    // Switch to PLL
    RCC->CFGR  = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {};
}

//-----------------------------------------
// Buzzer Frequency Setter
//-----------------------------------------
/**
 * @brief Updates the prescaler to achieve the desired PWM frequency.
 *        Frequency = SYSCLK / [PSC+1] / [ARR+1]
 *
 * For an 8-bit resolution, ARR is fixed at 255.
 * We solve for PSC:
 *   PSC = (SYSCLK / (frequency * (ARR+1))) - 1
 */
void SetBuzzerFrequency(uint32_t frequency) {
    if (frequency == 0) {
        frequency = 1;  // Avoid divide-by-zero
    }

    // Temporarily disable Timer
    TIM2->CR1 &= ~TIM_CR1_CEN;

    // Recalculate prescaler for new frequency
    TIM2->PSC = (uint16_t)((SYSCLK / (frequency * (PWM_RESOLUTION + 1))) - 1);

    // Re-enable Timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

//-----------------------------------------
// Enable/Disable the Buzzer
//-----------------------------------------
void BuzzerOn(void) {
    // Enable CH1 output
    TIM2->CCER |= TIM_CCER_CC1E;
}

void BuzzerOff(void) {
    // Disable CH1 output
    TIM2->CCER &= ~TIM_CCER_CC1E;
}

//-----------------------------------------
// Simple LED Toggle (PA0)
//-----------------------------------------
void ToggleLED(void) {
    GPIOA->ODR ^= (1 << 0); // Toggle PA0
}

//-----------------------------------------
// TIM2 Interrupt Handler
//-----------------------------------------
void TIM2_Handler(void) {
    // Clear update interrupt flag
    TIM2->SR &= ~1; // SR & ~BIT0

    Count++;
    if (Count >= 1000) {
        // Toggle LED every 1 second (optional)
        ToggleLED();
        Count = 0;
    }
}

//-----------------------------------------
// Hardware Initialization
//-----------------------------------------
void Hardware_Init(void) {
    SystemClock_Init(); // Initialize system clock to 32 MHz

    //-------------------------------------
    // Configure PA0 (LED) as output
    //-------------------------------------
    RCC->IOPENR |= (1 << 0);            // Enable GPIOA clock
    GPIOA->MODER &= ~((1 << 0) | (1 << 1)); // Clear bits for PA0
    GPIOA->MODER |=  (1 << 0);         // Set PA0 as output (01)

    //-------------------------------------
    // Configure PA15 (TIM2_CH1) as alternate function
    //-------------------------------------
    // Clear bits for PA15, then set AF mode
    GPIOA->MODER &= ~((1 << 30) | (1 << 31)); 
    GPIOA->MODER |=  (1 << 31);    // AF mode (10) for PA15
    // Set AF5 for TIM2_CH1
    GPIOA->AFR[1] |= (5 << (4 * (15 - 8))); 

    //-------------------------------------
    // TIM2 Configuration
    //-------------------------------------
    RCC->APB1ENR |= (1 << 0); // Enable TIM2 clock

    // Set prescaler for the initial buzzerFrequency
    TIM2->PSC  = (uint16_t)((SYSCLK / (g_buzzerFrequency * (PWM_RESOLUTION + 1))) - 1);
    TIM2->ARR  = PWM_RESOLUTION;    // 8-bit resolution
    TIM2->CCR1 = 128;               // 50% duty cycle for stable tone

    // PWM Mode 1 configuration on CH1: OC1M = 110, OC1PE enabled
    TIM2->CCMR1 |= (6 << 4) | (1 << 3);
    // Enable CH1 output
    TIM2->CCER  |= (1 << 0);

    // Auto-reload preload enable
    TIM2->CR1   |= (1 << 7); 
    // Enable TIM2
    TIM2->CR1   |= (1 << 0);

    //-------------------------------------
    // Interrupt Configuration
    //-------------------------------------
    TIM2->DIER  |= 1;            // Enable update interrupt
    NVIC->ISER[0] |= (1 << 15);  // Enable TIM2 interrupt in NVIC

    __enable_irq();
}

//-----------------------------------------
// Main Application
//-----------------------------------------
int main(void) {
    Hardware_Init();

    // Turn the buzzer on (stable tone @ 1 kHz, 50% duty)
    BuzzerOn();
    SetBuzzerFrequency(800); // Set initial frequency
    
    // [Optionally] you can also change the frequency later with:
    // SetBuzzerFrequency(2000); // e.g., 2 kHz
    // BuzzerOn();  // Ensure it’s still on if you toggled it off before

    while(1) {
        // Main loop: do whatever else you need
    }
    return 0;
}
