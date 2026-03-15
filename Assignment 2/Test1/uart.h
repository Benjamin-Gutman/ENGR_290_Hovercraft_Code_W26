#ifndef UART_H
#define UART_H

#include <stdint.h>

void UartInitialize(void);
void UartPrintCharacter(char c);
void UartPrintString(const char *s);
void UartAddNewLine(void);
void UartPrint_u16(uint16_t value);
void UartPrint_i16(int16_t value);
void UartPrint_float(float value);

#endif
