#include "../Common/Include/stm32l051xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../Common/Include/serial.h"
#include "UART2.h"
#include <math.h>

#define SYSCLK          32000000L
#define PWM_RESOLUTION  255  // 8-bit resolution

static uint32_t g_buzzerFrequency = 1000; // Start with 1 kHz

#define DEF_F 15000L
#define F_CPU 32000000L
#define COIN_FREQ_THRESHOLD 20000

// --- LCD Macros (from lcd.h) ---
#define LCD_RS_0 (GPIOA->ODR &= ~BIT0)
#define LCD_RS_1 (GPIOA->ODR |= BIT0)
#define LCD_E_0  (GPIOA->ODR &= ~BIT1)
#define LCD_E_1  (GPIOA->ODR |= BIT1)
#define LCD_D4_0 (GPIOA->ODR &= ~BIT2)
#define LCD_D4_1 (GPIOA->ODR |= BIT2)
#define LCD_D5_0 (GPIOA->ODR &= ~BIT3)
#define LCD_D5_1 (GPIOA->ODR |= BIT3)
#define LCD_D6_0 (GPIOA->ODR &= ~BIT4)
#define LCD_D6_1 (GPIOA->ODR |= BIT4)
#define LCD_D7_0 (GPIOA->ODR &= ~BIT5)
#define LCD_D7_1 (GPIOA->ODR |= BIT5)
#define CHARS_PER_LINE 16

// --- Forward declarations for LCD functions ---
void LCD_pulse(void);
void LCD_byte(unsigned char x);
void WriteData(unsigned char x);
void WriteCommand(unsigned char x);
void LCD_4BIT(void);
void LCDprint(char * string, unsigned char line, unsigned char clear);

//**************************************************************
// Delay and SysTick-based wait functions
//**************************************************************
volatile int Count = 0;

// Function Prototypes
void SystemClock_Init(void);
void Hardware_Init(void);
void SetBuzzerFrequency(uint32_t frequency);
void BuzzerOn(void);
void BuzzerOff(void);
void ToggleLED(void);

void SystemClock_Init(void)
{
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

//-----------------------------------------------------------
// Buzzer Frequency Setter (still uses ARR=255, CH2 on TIM2)
//-----------------------------------------------------------
void SetBuzzerFrequency(uint32_t frequency)
{
    if (frequency == 0) frequency = 1;  // Avoid divide-by-zero

    // Temporarily disable Timer
    TIM2->CR1 &= ~TIM_CR1_CEN;

    // Recalculate prescaler for new frequency
    // freq = SYSCLK / (PSC+1) / (ARR+1)
    // ARR=255 => PSC = (SYSCLK / (freq * 256)) - 1
    TIM2->PSC = (uint16_t)((SYSCLK / (frequency * (PWM_RESOLUTION + 1))) - 1);

    // Re-enable Timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

//-----------------------------------------------------------
// Enable/Disable Buzzer (TIM2_CH2 => CC2E is bit 4 of CCER)
//-----------------------------------------------------------
void BuzzerOn(void)
{
    TIM2->CCER |= (1 << 4);  // CC2E=1 => CH2 output enable
}

void BuzzerOff(void)
{
    TIM2->CCER &= ~(1 << 4); // CC2E=0 => CH2 output disable
}

//-----------------------------------------------------------
// Simple LED Toggle on PB4 (pin #27)  // UPDATED
//-----------------------------------------------------------
void ToggleLED(void)
{
    GPIOB->ODR ^= (1 << 4); // Toggle PB4
}

//-----------------------------------------------------------
// TIM2 Interrupt Handler
//-----------------------------------------------------------
void TIM2_Handler(void)
{
    // Clear update interrupt flag
    TIM2->SR &= ~1;

    Count++;
    if (Count >= 1000) {
        // Toggle LED every 1 second
        ToggleLED();
        Count = 0;
    }
}

//-----------------------------------------------------------
// Hardware Initialization
//-----------------------------------------------------------
void Hardware_Init(void)
{
    SystemClock_Init(); // Initialize system clock to 32 MHz

    //-------------------------------------------------------
    // 1) Configure PB4 as output (LED)  // UPDATED
    //-------------------------------------------------------
    RCC->IOPENR |= (1 << 1);        // Enable GPIOB clock (bit1 => port B)
    // PB4 => pin #27 in your LQFP32 pinout
    GPIOB->MODER &= ~(3 << (4 * 2));  // Clear mode bits for PB4
    GPIOB->MODER |=  (1 << (4 * 2));  // Set PB4 as output (01)
    // (optionally set push-pull/speed, but default push-pull is fine)

    //-------------------------------------------------------
    // 2) Configure PB7 as AF for TIM2_CH2 (PWM) // UPDATED
    //-------------------------------------------------------
    //    PB7 => pin #30 in your LQFP32 pinout
    //    TIM2_CH2 is usually AF2 on STM32L0xx
    GPIOB->MODER &= ~(3 << (7 * 2));     // Clear mode bits for PB7
    GPIOB->MODER |=  (2 << (7 * 2));     // Set to AF mode (10)
    GPIOB->AFR[0] &= ~(0xF << (7 * 4));  // Clear AF bits for PB7
    GPIOB->AFR[0] |=  (2 << (7 * 4));    // AF2 => TIM2_CH2

    //-------------------------------------------------------
    // 3) TIM2 Configuration (Channel 2 for Buzzer)
    //-------------------------------------------------------
    RCC->APB1ENR |= (1 << 0);  // Enable TIM2 clock
    TIM2->CR1   &= ~TIM_CR1_CEN;  // Disable TIM2 before config

    // Prescaler => set by SetBuzzerFrequency()
    TIM2->PSC  = (uint16_t)((SYSCLK / (g_buzzerFrequency * (PWM_RESOLUTION + 1))) - 1);
    TIM2->ARR  = PWM_RESOLUTION;   // 8-bit => 255
    TIM2->CCR2 = 128;             // 50% duty cycle => stable tone

    // Use Channel 2 in PWM mode 1:  
    //   - For Channel 2 => bits 14:12 are OC2M,
    //   - bit 11 is OC2PE
    TIM2->CCMR1 &= ~(0xFF00);            // Clear top byte (channel 2 config)
    TIM2->CCMR1 |=  (6 << 12) | (1 << 11); // OC2M=110 (PWM1), OC2PE=1

    // Enable CH2 output => CC2E (bit 4) in CCER
    TIM2->CCER  |= (1 << 4);

    // Auto-reload preload enable
    TIM2->CR1   |= (1 << 7);

    // Enable TIM2
    TIM2->CR1   |= (1 << 0);

    //-------------------------------------------------------
    // 4) TIM2 Interrupt Configuration (for blinking LED)
    //-------------------------------------------------------
    TIM2->DIER  |= 1;            // Update interrupt enable (UIE)
    NVIC->ISER[0] |= (1 << 15);  // Enable TIM2 IRQ in NVIC (vector 15 on L0)

    __enable_irq();

    //-------------------------------------------------------
    // 5) Other Pins (JDY-40 config, etc.) remain as is
    //-------------------------------------------------------
    GPIOA->OSPEEDR = 0xffffffff; // All pins of port A set to very high speed
    RCC->IOPENR |= BIT0;         // Enable clock for port A

    // Configure PA13 as output (SET pin for JDY-40)
    GPIOA->MODER = (GPIOA->MODER & ~(BIT27|BIT26)) | BIT26;
    GPIOA->ODR   |= BIT13; // Set PA13=1 => normal operation

    // If you have a pushbutton on PA8
    GPIOA->MODER &= ~(BIT16 | BIT17); // set as input
    GPIOA->PUPDR |=  BIT16; 
    GPIOA->PUPDR &= ~BIT17;
}

//-----------------------------------------------------------
// SysTick-based microsecond delay
//-----------------------------------------------------------
void Delay_us(unsigned char us)
{
    SysTick->LOAD = (F_CPU/(1000000L/us)) - 1;
    SysTick->VAL  = 0;
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
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while((SysTick->CTRL & BIT16)==0);
    SysTick->CTRL = 0x00;
}

void delayms(int len)
{
    while(len--) wait_1ms();
}

//-----------------------------------------------------------
// LCD Functions
//-----------------------------------------------------------
void LCD_pulse(void)
{
    LCD_E_1;
    Delay_us(40);
    LCD_E_0;
}

void LCD_byte(unsigned char x)
{
    // Send high nibble
    if(x & 0x80) LCD_D7_1; else LCD_D7_0;
    if(x & 0x40) LCD_D6_1; else LCD_D6_0;
    if(x & 0x20) LCD_D5_1; else LCD_D5_0;
    if(x & 0x10) LCD_D4_1; else LCD_D4_0;
    LCD_pulse();
    Delay_us(40);
    // Send low nibble
    if(x & 0x08) LCD_D7_1; else LCD_D7_0;
    if(x & 0x04) LCD_D6_1; else LCD_D6_0;
    if(x & 0x02) LCD_D5_1; else LCD_D5_0;
    if(x & 0x01) LCD_D4_1; else LCD_D4_0;
    LCD_pulse();
}

void WriteData(unsigned char x)
{
    LCD_RS_1;
    LCD_byte(x);
    waitms(2);
}

void WriteCommand(unsigned char x)
{
    LCD_RS_0;
    LCD_byte(x);
    waitms(5);
}

void LCD_4BIT(void)
{
    LCD_E_0;  // Resting state
    waitms(20);
    // Force LCD to 8-bit mode first, then switch to 4-bit
    WriteCommand(0x33);
    WriteCommand(0x33);
    WriteCommand(0x32);  // 4-bit mode
    // Configure: 2 lines, 5x8 font, display ON, clear
    WriteCommand(0x28);
    WriteCommand(0x0c);
    WriteCommand(0x01);
    waitms(20);
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

//-----------------------------------------------------------
// Misc Setup Functions
//-----------------------------------------------------------
void Configure_Pins(void)
{
    // Example: PB1 as analog input
    RCC->IOPENR |= BIT1;
    GPIOB->MODER |= (BIT2 | BIT3);

    // Example: PB13 as digital input with pull-up
    GPIOB->MODER &= ~(BIT26 | BIT27);
    GPIOB->PUPDR |=  BIT26;
    GPIOB->PUPDR &= ~BIT27;
}

// Configure_Buttons: PB3, PB5, PB6 as inputs w/ pull-ups
void Configure_Buttons(void)
{
    RCC->IOPENR |= BIT1; // Enable clock for Port B

    // PB3, PB5, PB6 => inputs (00)
    GPIOB->MODER &= ~(0x3 << (3*2)); // PB3
    GPIOB->MODER &= ~(0x3 << (5*2)); // PB5
    GPIOB->MODER &= ~(0x3 << (6*2)); // PB6

    // Enable pull-ups
    GPIOB->PUPDR &= ~(0x3 << (3*2));
    GPIOB->PUPDR |=  (0x1 << (3*2));

    GPIOB->PUPDR &= ~(0x3 << (5*2));
    GPIOB->PUPDR |=  (0x1 << (5*2));

    GPIOB->PUPDR &= ~(0x3 << (6*2));
    GPIOB->PUPDR |=  (0x1 << (6*2));
}

//-----------------------------------------------------------
// ADC
//-----------------------------------------------------------
void initADC(void)
{
    RCC->APB2ENR |= BIT9; // Enable ADC clock

    // Select PCLK as ADC clock source
    ADC1->CFGR2 |= ADC_CFGR2_CKMODE;

    // ADC enable sequence
    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CR  |= ADC_CR_ADEN;
    if ((ADC1->CFGR1 & ADC_CFGR1_AUTOFF) == 0)
    {
        while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}
    }

    // Calibration
    if ((ADC1->CR & ADC_CR_ADEN) != 0)
    {
        ADC1->CR |= ADC_CR_ADDIS;
    }
    ADC1->CR |= ADC_CR_ADCAL;
    while ((ADC1->ISR & ADC_ISR_EOCAL) == 0) {}
    ADC1->ISR |= ADC_ISR_EOCAL;
}

int readADC(unsigned int channel)
{
    ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;
    ADC1->CHSELR = channel;
    ADC1->SMPR |= (ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2);
    if(channel == ADC_CHSELR_CHSEL17)
    {
        ADC->CCR |= ADC_CCR_VREFEN;
    }
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {}
    return ADC1->DR;
}

//-----------------------------------------------------------
// JDY-40 Communication
//-----------------------------------------------------------
void SendATCommand(char * s)
{
    char buff[40];
    printf("Command: %s", s);
    GPIOA->ODR &= ~(BIT13); // 0 => AT mode
    waitms(10);
    eputs2(s);
    egets2(buff, sizeof(buff)-1);
    GPIOA->ODR |= BIT13; // normal mode
    waitms(10);
    printf("Response: %s", buff);
}

void ReceptionOff(void)
{
    GPIOA->ODR &= ~(BIT13); // AT mode
    waitms(10);
    eputs2("AT+DVID0000\r\n");
    waitms(10);
    GPIOA->ODR |= BIT13; // normal
    while (ReceivedBytes2() > 0) {
        egetc2(); // Clear FIFO
    }
}

// Example joystick function
int voltageToDuty(float adc_v)
{
    int duty = (int)(((adc_v - 1.6) / 1.6) * 100.0);

    if (duty > 100) {
        duty = 100;
    } else if (duty < -100) {
        duty = -100;
    }
    if (abs(duty) < 5) {
        duty = 1;
    }
    return duty;
}

//-----------------------------------------------------------
// LCD GPIO: PA0–PA5 for LCD signals (unchanged from your code)
//-----------------------------------------------------------
void LCD_GPIO_Init(void)
{
    RCC->IOPENR |= BIT0;
    GPIOA->MODER &= ~0xFFF;  // Clear mode bits for PA0–PA5
    GPIOA->MODER |= 0x555;   // Output for those pins
    GPIOA->OTYPER &= ~0x3F;  // Push-pull
}

//-----------------------------------------------------------
// Main Function
//-----------------------------------------------------------
int main(void)
{
    char buff[80];
    int timeout_cnt = 0;
    int up, down, left, right, duty_x, duty_y;
    int button_auto, button_manual, button_coin;
    int j8, j9;
    float adc_v8, adc_v9;

    Hardware_Init();
    initUART2(9600);
    Configure_Pins();
    Configure_Buttons();
    initADC();

    // LCD Init
    LCD_GPIO_Init();
    LCD_4BIT();
    LCDprint("Initializing...", 1, 1);

    waitms(1000);
    printf("\r\nJDY-40 Master test\r\n");

    ReceptionOff();

    // Check JDY-40 settings
    SendATCommand("AT+VER\r\n");
    SendATCommand("AT+BAUD\r\n");
    SendATCommand("AT+RFID\r\n");
    SendATCommand("AT+DVID\r\n");
    SendATCommand("AT+RFC\r\n");
    SendATCommand("AT+POWE\r\n");
    SendATCommand("AT+CLSS\r\n");
    SendATCommand("AT+DVIDCACC\r\n");
    SendATCommand("AT+RFC030\r\n");

    while(1)
    {
        // Reset each loop
        up=0; down=0; left=0; right=0; duty_x=0; duty_y=0;
        button_auto=0; button_manual=0; button_coin=0;

        // Read joystick (ADC channels 8, 9 as example)
        j8 = readADC(ADC_CHSELR_CHSEL8);
        j9 = readADC(ADC_CHSELR_CHSEL9);
        adc_v8 = (j8 * 3.3f) / 0x1000;
        adc_v9 = (j9 * 3.3f) / 0x1000;

        duty_x = voltageToDuty(adc_v8);
        duty_y = voltageToDuty(adc_v9);

        BuzzerOn(); // Just an example

        // Read pushbuttons on PB3 (Auto), PB5 (Manual), PB6 (Coin)
        if (!(GPIOB->IDR & (1 << 3))) {
            button_auto = 1;
            BuzzerOff();
        }
        if (!(GPIOB->IDR & (1 << 5))) 
            button_manual = 1;
        if (!(GPIOB->IDR & (1 << 6))) 
            button_coin = 1;

        // Construct message to send to JDY-40 slave
        sprintf(buff, "%d %d %d %d %d\n",
                duty_x, duty_y, button_auto, button_manual, button_coin);

        // Send
        eputc2('@');
        eputc2('!');
        waitms(5);
        eputs2(buff);
        waitms(5);

        // Wait for reply
        timeout_cnt = 0;
        while(1)
        {
            if(ReceivedBytes2() > 5) break;
            if(++timeout_cnt > 200) break;  // ~25ms
            Delay_us(100);
        }

        if(ReceivedBytes2() > 5)
        {
            egets2(buff, sizeof(buff)-1);
            if(strlen(buff) > 0)
            {
                // Remove trailing newlines
                for(int i=0; buff[i] != '\0'; i++){
                    if(buff[i]=='\n' || buff[i]=='\r'){
                        buff[i] = '\0';
                        break;
                    }
                }
                printf("Slave says: %s\r\n", buff);
                char displayStr[17];
                sprintf(displayStr, "Freq: %s", buff);
                LCDprint(displayStr, 2, 1);

                // Convert freq from string to int
                // Then scale by e.g. "/100" as your original code suggests
                SetBuzzerFrequency( atoi(buff) / 100 );
            }
            else
            {
                // Clear leftover
                while(ReceivedBytes2()>0) egetc2();
            }
        }
        else
        {
            printf("No response\r\n");
            while(ReceivedBytes2()>0) egetc2();
        }

        waitms(50); 
        fflush(stdout);
    }
}
