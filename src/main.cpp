#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <avr/interrupt.h>

// Configuration
// #define F_CPU 16000000UL
#define SCL_CLOCK 100000L
#define RTC_ADDRESS 0x68
#define SEC_REGISTER 0x00
#define MIN_REGISTER 0x01
#define HOUR_REGISTER 0x02

// Global variables for new time setting
volatile uint8_t Newhours = 12;
volatile uint8_t Newminutes = 0;
volatile uint8_t Newseconds = 0;
volatile const char *Newstr = "";

// Flag to indicate ISR active state
volatile uint8_t isr_active = 0;

// Global variables for button press flags
volatile uint8_t button4_pressed = 0;
volatile uint8_t button5_pressed = 0;

// Function prototypes
void I2C_Init(void);
uint8_t I2C_Start(void);
uint8_t I2C_Stop(void);
uint8_t I2C_Write(uint8_t data);
uint8_t I2C_ReadAck(void);
uint8_t I2C_ReadNack(void);
void I2C_WriteData(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data);
uint8_t I2C_ReadData(uint8_t deviceAddress, uint8_t registerAddress);
void BlinkAllSegments(void);
void SetTimeUsingButtons(void);

// RTC Functions
void RTC_Init(void);
void RTC_SetTime(uint8_t hours, uint8_t minutes, uint8_t seconds, const char *str);
void RTC_GetTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds, uint8_t *isPM);

// Utility Functions
uint8_t decToBcd(uint8_t val);
uint8_t bcdToDec(uint8_t val);

// Seven-Segment Display Functions
void set_port_output(void);
void display_digit(uint8_t digit, uint8_t position);
void display_time_on_7segment(uint8_t hours, uint8_t minutes, uint8_t seconds);
void display_time_setting(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t state);

// UART Functions
void UART_Init(unsigned int ubrr);
void UART_Transmit(unsigned char data);
void UART_Printf(const char *fmt, ...);

// I2C Implementation
void I2C_Init(void)
{
    TWSR = 0x00;
    TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
    TWCR = (1 << TWEN);
}

uint8_t I2C_Start(void)
{
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    return (TWSR & 0xF8);
}

uint8_t I2C_Stop(void)
{
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
    while (TWCR & (1 << TWSTO));
    return (TWSR & (1 << TWSTO));
}

uint8_t I2C_Write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    return (TWSR & 0xF8);
}

uint8_t I2C_ReadAck(void)
{
    TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t I2C_ReadNack(void)
{
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

void I2C_WriteData(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data)
{
    I2C_Start();
    I2C_Write(deviceAddress << 1);
    I2C_Write(registerAddress);
    I2C_Write(data);
    I2C_Stop();
}

uint8_t I2C_ReadData(uint8_t deviceAddress, uint8_t registerAddress)
{
    uint8_t data;
    I2C_Start();
    I2C_Write(deviceAddress << 1);
    I2C_Write(registerAddress);
    I2C_Start();
    I2C_Write((deviceAddress << 1) | 0x01);
    data = I2C_ReadNack();
    I2C_Stop();
    return data;
}

// RTC Implementation
void RTC_Init(void)
{
    I2C_Init();
}

void RTC_SetTime(uint8_t hours, uint8_t minutes, uint8_t seconds, const char *str)
{
    uint8_t bcdHours = decToBcd(hours);
    bcdHours |= (1 << 6); // Set bit 6 for 12-hour mode (AM/PM)

    if (!strcmp(str, "pm") || !strcmp(str, "PM") || !strcmp(str, "P") || !strcmp(str, "p"))
    {
        bcdHours |= (1 << 5); // Set bit 5 for PM
    }
    else
    {
        bcdHours &= ~(1 << 5); // Clear bit 5 for non-PM
    }

    I2C_WriteData(RTC_ADDRESS, SEC_REGISTER, decToBcd(seconds));
    I2C_WriteData(RTC_ADDRESS, MIN_REGISTER, decToBcd(minutes));
    I2C_WriteData(RTC_ADDRESS, HOUR_REGISTER, bcdHours);
}

void RTC_GetTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds, uint8_t *isPM)
{
    uint8_t rawHours;

    *seconds = bcdToDec(I2C_ReadData(RTC_ADDRESS, SEC_REGISTER) & 0x7F);
    *minutes = bcdToDec(I2C_ReadData(RTC_ADDRESS, MIN_REGISTER) & 0x7F);
    rawHours = I2C_ReadData(RTC_ADDRESS, HOUR_REGISTER);

    *isPM = (rawHours & (1 << 5)) ? 1 : 0;
    *hours = bcdToDec(rawHours & 0x1F);
}

// Utility Functions Implementation
uint8_t decToBcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

uint8_t bcdToDec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

// Seven-Segment Display Functions
uint8_t digit_patterns[] = {
    0b00111111,
    0b00000110,
    0b01011011,
    0b01001111,
    0b01100110,
    0b01101101,
    0b01111101,
    0b00000111,
    0b01111111,
    0b01101111
};

void set_port_output(void)
{
    DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2);
    DDRC |= (1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3);
    DDRD |= (1 << DDD2) | (1 << DDD3) | (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7);
}

void display_digit(uint8_t digit, uint8_t position)
{
    if (digit > 9)
        return;

    uint8_t segments = ~digit_patterns[digit];

    PORTB = (PORTB & 0b11111000) | (segments & 0b00000111);
    PORTC = (PORTC & 0b11110000) | ((segments & 0b01111000) >> 3);

    PORTD |= (1 << position);
    _delay_ms(5);
    PORTD &= ~(1 << position);
}

void display_time_on_7segment(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    display_digit(hours / 10, PORTD2);
    display_digit(hours % 10, PORTD3);
    display_digit(minutes / 10, PORTD4);
    display_digit(minutes % 10, PORTD5);
    display_digit(seconds / 10, PORTD6);
    display_digit(seconds % 10, PORTD7);
}

void display_time_setting(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t state)
{
    // Clear all segments before updating
    PORTD &= ~(0b11111100);

    if (state == 0)
    {
        // Display hours
        display_digit(hours / 10, PORTD2);
        display_digit(hours % 10, PORTD3);
    }
    else if (state == 1)
    {
        // Display minutes
        display_digit(minutes / 10, PORTD4);
        display_digit(minutes % 10, PORTD5);
    }
    else if (state == 2)
    {
        // Display seconds
        display_digit(seconds / 10, PORTD6);
        display_digit(seconds % 10, PORTD7);
    }
}

// UART Functions
void UART_Init(unsigned int ubrr)
{
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
}

void UART_Transmit(unsigned char data)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void UART_Printf(const char *fmt, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    for (char *p = buffer; *p != '\0'; p++)
    {
        UART_Transmit(*p);
    }
}

void setup(void)
{
    // Configure PB4 and PB5 as input with pull-up resistors
    DDRB &= ~(1 << DDB4);
    PORTB |= (1 << PORTB4);
    DDRB &= ~(1 << DDB5);
    PORTB |= (1 << DDB5);
    
    // Enable pin change interrupt for PB4 and PB5
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT4) | (1 << PCINT5);
    
    sei(); // Enable global interrupts
}

// Function to set time using buttons
void SetTimeUsingButtons(void)
{
    static uint8_t state = -1;  // Initialize state to 0
    static uint8_t hours = 0, minutes = 0, seconds = 0;

    while (isr_active)
    {
        display_time_setting(hours, minutes, seconds, state); // Continuously update display

        if (button5_pressed)
        {
            button5_pressed = 0; // Clear flag after handling
           // UART_Printf("Button 5 Pressed, State: %d\r\n", state);

            switch (state)
            {
            case 0:
                hours = (hours + 1) % 13; // Increment hours from 0 to 12
                //UART_Printf("Increment Hours: %d\r\n", hours);
                break;
            case 1:
                minutes = (minutes + 1) % 60; // Increment minutes from 0 to 59
                //UART_Printf("Increment Minutes: %d\r\n", minutes);
                break;
            case 2:
                seconds = (seconds + 1) % 60; // Increment seconds from 0 to 59
                //UART_Printf("Increment Seconds: %d\r\n", seconds);
                break;
            default:
                //UART_Printf("Invalid State: %d\r\n", state);
                break;
            }
        }

        if (button4_pressed)
        {
            button4_pressed = 0; // Clear flag after handling
            //UART_Printf("Button 4 Pressed, State Before: %d\r\n", state);

            state = (state + 1) % 4; // Move to next state
            //UART_Printf("State After: %d\r\n", state);

            if (state == 3)
            {
                RTC_SetTime(hours, minutes, seconds, "AM");
                isr_active = 0;
            }
        }
    }
}

// Interrupt Service Routine (ISR) for pin change interrupt on PB4 and PB5
ISR(PCINT0_vect)
{
    _delay_ms(50); // Debounce delay

    if (bit_is_clear(PINB, PINB4))
    {
        isr_active = 1;
        button4_pressed = 1;
    }

    if (bit_is_clear(PINB, PINB5))
    {
        isr_active = 1;
        button5_pressed = 1;
    }
}

int main(void)
{
    uint8_t hours, minutes, seconds, isPM;
    char period[3];

    setup();
    UART_Init(103);
    RTC_Init();
    set_port_output();
    RTC_SetTime(12, 13, 0, "");

    while (1)
    {
        if (isr_active)
        {
            SetTimeUsingButtons();
        }

        RTC_GetTime(&hours, &minutes, &seconds, &isPM);
        display_time_on_7segment(hours, minutes, seconds);

        if (isPM)
            strcpy(period, "PM");
        else
            strcpy(period, "AM");

        UART_Printf("Time: %02d:%02d:%02d %s\r\n", hours, minutes, seconds, period);
        _delay_ms(10);
    }

    return 0;
}
