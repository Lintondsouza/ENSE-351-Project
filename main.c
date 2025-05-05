//Name:Linton Joswin Dsouza
//Student ID: 200470698
//Project : Using the MQ135 and dth22 sensor to read air quality and evironmental condition values

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

#define LOOP_400MS (3200000UL) 
#define DHT22_PIN   4         // Using PA4 for the sensor data line
#define DHT22_PORT  GPIOA


#define SYSTEM_CORE_CLOCK 72000000UL  // 72 MHz
#define BAUDRATE          9600
#define MQ135_THRESHOLD   60



// Busy-loop delay functions for a 72 MHz clock.
#define LOOP_1MS (SYSTEM_CORE_CLOCK / 1000UL)  // 72,000 iterations per 1ms

static void delay_1ms(void)
{
    for (volatile uint32_t i = 0; i < LOOP_1MS; i++) {
        __NOP();
    }
}

static void delay_500ms(void)
{
    for (uint32_t i = 0; i < 200; i++) {
        delay_1ms();
    }
}

// Timer initialization for microsecond delays using TIM2
static void TIM2_Init_1MHz(void)
{
    RCC->APB1ENR |= (1 << 0);    // Enable TIM2 clock
    TIM2->PSC = 35;             // Set prescaler for 1 MHz (assuming APB1 = 36MHz)
    TIM2->ARR = 0xFFFF;         // 16-bit free-running counter
    TIM2->CR1 = 1;              // Enable timer
}

// Returns current timer count (in microseconds, modulo 65536)
static inline uint16_t timer_us(void)
{
    return (uint16_t)(TIM2->CNT & 0xFFFF);
}

// Delay function in microseconds using TIM2
static void delay_us(uint16_t us)
{
    uint16_t start = timer_us();
    while ((uint16_t)(timer_us() - start) < us)
        ; // Busy wait
}

void USART1_Init(void)
{
    // Enable clocks for GPIOA and USART1.
    RCC->APB2ENR |= (1 << 2) | (1 << 14);
    
    // Configure PA9 as Alternate Function Push-Pull (TX)
    GPIOA->CRH &= ~(0xF << 4);
    GPIOA->CRH |= (0xB << 4); // MODE=11 (50MHz), CNF=10 (AF push-pull)
    
    // Configure PA10 as Input Floating (RX)
    GPIOA->CRH &= ~(0xF << 8);
    GPIOA->CRH |= (0x4 << 8); // MODE=00, CNF=01 (input floating)
    
    // Set baud rate (for 9600 bps at 72MHz; 7500 == 0x1D4C)
    USART1->BRR = 7500;
    
    // Configure for 8 data bits, 1 stop bit, no parity.
    USART1->CR2 &= ~(3 << 12);
    USART1->CR1 &= ~(1 << 12);  // M=0: 8 data bits.
    USART1->CR1 &= ~(1 << 10);  // PCE=0: no parity.
    
    // Enable transmitter and receiver.
    USART1->CR1 |= (1 << 2) | (1 << 3);
    
    // Enable USART.
    USART1->CR1 |= (1 << 13);
}

void USART1_SendChar(char c)
{
    while (!(USART1->SR & (1 << 7))) { } // Wait until TXE flag is set.
    USART1->DR = (uint8_t)c;
    while (!(USART1->SR & (1 << 6))) { } // Wait until transmission complete.
}

void USART1_SendString(const char *s)
{
    while (*s) {
        USART1_SendChar(*s++);
    }
}

// Returns 0 on success, 1 on timeout error, 2 on checksum error.
static int DHT22_Read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint16_t timeout;
    
    // Send Start Signal: pull PA4 low for ~18ms then high for ~30us.
    // Configure PA4 as output push-pull (50MHz)
    GPIOA->CRL &= ~(0xF << (4 * 4));
    GPIOA->CRL |= (0x3 << (4 * 4)); // Output mode, push-pull.
    GPIOA->ODR &= ~(1 << DHT22_PIN);  // Drive low.
    delay_us(18000);                  // 18ms delay.
    GPIOA->ODR |= (1 << DHT22_PIN);   // Drive high.
    delay_us(30);                     // 30탎 delay.
    
    // Change PA4 to input floating.
    GPIOA->CRL &= ~(0xF << (4 * 4));
    GPIOA->CRL |= (0x4 << (4 * 4));  // Input floating.

    // Wait for sensor response (expect ~80탎 low, then ~80탎 high).
    timeout = 0;
    while (GPIOA->IDR & (1 << DHT22_PIN)) {
        delay_us(1);
        if (++timeout > 100)
            return 1; // Timeout error.
    }
    timeout = 0;
    while (!(GPIOA->IDR & (1 << DHT22_PIN))) {
        delay_us(1);
        if (++timeout > 100)
            return 1;
    }
    timeout = 0;
    while (GPIOA->IDR & (1 << DHT22_PIN)) {
        delay_us(1);
        if (++timeout > 100)
            return 1;
    }
    
    // Read 40 bits of data.
    for (int i = 0; i < 40; i++) {
        // Wait for the start of the pulse (low-to-high transition).
        timeout = 0;
        while (!(GPIOA->IDR & (1 << DHT22_PIN))) {
            delay_us(1);
            if (++timeout > 100)
                return 1;
        }
        // Measure length of the high pulse.
        uint16_t t = 0;
        while (GPIOA->IDR & (1 << DHT22_PIN)) {
            delay_us(1);
            t++;
            if (t > 100)
                break;
        }
        // A long pulse (> 40탎) means bit '1'.
        if (t > 40)
            data[i / 8] |= (1 << (7 - (i % 8)));
    }
    
    // Checksum verification.
    if (((uint8_t)(data[0] + data[1] + data[2] + data[3])) != data[4])
        return 2; // Checksum error.
    
    // Convert data to floating point values.
    *humidity    = ((data[0] << 8) | data[1]) * 0.1f;
    *temperature = (((data[2] & 0x7F) << 8) | data[3]) * 0.1f;
    if (data[2] & 0x80)
        *temperature = -*temperature;
        
    return 0;
}

void ADC1_Init(void)
{
    // Enable clocks for GPIOA and ADC1.
    RCC->APB2ENR |= (1 << 2) | (1 << 9);

    // Configure PA0 as analog input (MODE=00, CNF=00).
    GPIOA->CRL &= ~(0xF << (0 * 4));

    // Turn on ADC1.
    ADC1->CR2 |= (1 << 0); // ADON = 1.
    
    // Allow ADC to stabilize.
    delay_1ms();
    
    // Set sample time for channel 0 to 239.5 cycles (for improved accuracy).
    ADC1->SMPR2 |= (7 << 0);
    
    // Calibrate ADC with timeout.
    uint32_t timeout = 1000;
    
    // Reset calibration.
    ADC1->CR2 |= (1 << 2);   // ADC_CR2_RSTCAL.
    while ((ADC1->CR2 & (1 << 2)) && timeout) {
        timeout--;
    }
    if (timeout == 0) {
        USART1_SendString("Calibration Reset Timeout\r\n");
    }
    
    // Start calibration.
    timeout = 1000;
    ADC1->CR2 |= (1 << 3);   // ADC_CR2_CAL.
    while ((ADC1->CR2 & (1 << 3)) && timeout) {
        timeout--;
    }
    if (timeout == 0) {
        USART1_SendString("Calibration Timeout\r\n");
    }
    
    // Perform a dummy conversion to allow ADC to settle.
    ADC1->SQR3 = 0;                  // Select channel 0.
    ADC1->CR2 |= (1 << 22);           // Start conversion (SWSTART).
    timeout = 100000;
    while (!(ADC1->SR & (1 << 1)) && timeout) { 
        timeout--;
    }
    (void)ADC1->DR;                  // Discard the dummy value.
}

uint16_t ADC1_ReadChannel0(void)
{
    uint32_t timeout = 1000000;  // Timeout counter.

    // Select channel 0.
    ADC1->SQR3 = 0;

    // Start conversion.
    ADC1->CR2 |= (1 << 22);

    // Wait for conversion completion (EOC flag) with timeout.
    while (!(ADC1->SR & (1 << 1)) && timeout--) { }
    
    if (timeout == 0) {
        USART1_SendString("ADC Conversion Timeout\r\n");
        return 0;
    }
    
    // Return ADC conversion result.
    return ADC1->DR;
}

void TIM3_PWM_Init(void)
{
    // Enable clocks for TIM3 and GPIOA.
    RCC->APB1ENR |= (1 << 1);  // TIM3 clock enable.
    RCC->APB2ENR |= (1 << 2);  // GPIOA clock enable.

    // Configure PA6 as Alternate Function Push-Pull.
    GPIOA->CRL &= ~(0xF << (6 * 4));
    GPIOA->CRL |=  (0xB << (6 * 4));
    TIM3->PSC = 0;
    TIM3->ARR = 35999;
    TIM3->CCR1 = 0; // Start with buzzer off (0% duty cycle).

    // Configure PWM mode 1 on Channel 1.
    TIM3->CCMR1 &= ~((7 << 4) | (1 << 3)); // Clear OC1M and OC1PE bits.
    TIM3->CCMR1 |= (6 << 4);   // Set OC1M to PWM mode 1.
    TIM3->CCMR1 |= (1 << 3);   // Enable preload for channel 1.

    // Enable capture/compare on Channel 1.
    TIM3->CCER |= 1;

    // Enable auto-reload preload.
    TIM3->CR1 |= (1 << 7);

    // Start TIM3.
    TIM3->CR1 |= 1;
}

void Buzzer_On(void)
{
    // Turn buzzer on at 50% duty cycle.
    TIM3->CCR1 = TIM3->ARR / 2;
}

void Buzzer_Off(void)
{
    // Turn buzzer off (0% duty cycle).
    TIM3->CCR1 = 0;
}

int main(void)
{
    float temp, hum;
    char buffer[50];
    uint32_t loop_counter = 0;

    /* Initialize all peripherals */
    TIM2_Init_1MHz();  // For microsecond delays needed by the DHT22
    USART1_Init();     // For UART communication (debug output)
    ADC1_Init();       // For reading MQ135 sensor via ADC
    TIM3_PWM_Init();   // For controlling the buzzer via PWM

    /* Startup Messages */
    USART1_SendString("Starting...\r\n");
    USART1_SendString("MQ135 + Buzzer Demo @ 72MHz\r\n");

    while (1)
    {
        if (loop_counter % 1 == 0)
        {
            int result = DHT22_Read(&temp, &hum);
            if (result == 0) {
                sprintf(buffer, "Temp: %.1f C, Hum: %.1f%%\r\n", temp, hum);
                USART1_SendString(buffer);
            } else if (result == 1) {
                USART1_SendString("Sensor timeout error\r\n");
            } else if (result == 2) {
                USART1_SendString("Checksum error\r\n");
            }
        }
        
        // Read MQ135 sensor via ADC and control buzzer.
        uint16_t mq135_value = ADC1_ReadChannel0();
        sprintf(buffer, "MQ135 ADC: %u\r\n", mq135_value);
        USART1_SendString(buffer);
        
        if (mq135_value > MQ135_THRESHOLD)
        {
            Buzzer_On();
            USART1_SendString("Buzzer ON\r\n");
        }
        else
        {
            Buzzer_Off();
            USART1_SendString("Buzzer OFF\r\n");
        }
        
        // Wait 500ms before next ADC reading.
        delay_500ms();
        loop_counter++;
    }
}
