#ifndef UART_H
#define UART_H

#include "stm32f446xx.h"

// Function definitions
void uart_init(USART_TypeDef *UARTx);
void usart2_write(int ch);
uint8_t usart2_read(void);
void printS(const char myString[]);
void printH(uint8_t value);
void printI(int32_t number);

#endif /* UART_H */