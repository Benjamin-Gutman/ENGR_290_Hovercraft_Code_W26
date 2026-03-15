// uart.c
// UART communication functions
#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>
#include "uart.h"

#define BAUD 9600UL
#define UBRR_VALUE ((F_CPU/16/BAUD) - 1)

void UartInitialize(void)
{
    // Set baud rate
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);

    // Frame format: 8 data bits, no parity, 1 stop bit (8N1)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    // Enable transmitter (TX) on PD1
    UCSR0B = (1 << TXEN0);
}

void UartPrintCharacter(char c)
{
    // Wait until transmit buffer is empty
    while (!(UCSR0A & (1 << UDRE0))) { }

    // Send byte
    UDR0 = c;
}

void UartPrintString(const char *s)
{
    while (*s)
    {
        UartPrintCharacter(*s);
        s++;
    }
}

void UartAddNewLine(void)
{
    UartPrintCharacter('\r');
    UartPrintCharacter('\n');
}

void UartPrint_u16(uint16_t value)
{
    char buffer[6];
    uint8_t i = 0;

    if (value == 0)
    {
        UartPrintCharacter('0');
        return;
    }

    while (value > 0)
    {
        buffer[i++] = '0' + (value % 10);
        value /= 10;
    }

    while (i > 0)
    {
        UartPrintCharacter(buffer[--i]);
    }
}
