#ifndef UART_COMPAT_H
#define UART_COMPAT_H

#include <avr/pgmspace.h>
#include <uart.h>

/**
 * @brief Compatibility layer for old UART function names
 * Maps transmitByte/transmitString to Peter Fleury UART library
 */

// Single byte transmission
#define transmitByte(data) uart_putc(data)

// String transmission from RAM
#define transmitString(str) uart_puts((char*)(str))

// Newline transmission
#define TX_NEWLINE uart_puts("\r\n")

/**
 * @brief Transmit string from PROGMEM (FLASH memory)
 * @param string Pointer to string in PROGMEM
 */
static inline void transmitString_P(const char* string)
{
    char c;
    while ((c = pgm_read_byte(string++)))
        uart_putc(c);
}

/**
 * @brief Transmit hexadecimal value as string
 * @param value Value to transmit in hex format
 */
static inline void transmitHex(unsigned char type, unsigned long value)
{
    char hex_string[12];
    char *ptr = hex_string;
    unsigned char nibble;
    unsigned char i;
    unsigned char digits;
    
    // Determine number of digits based on type
    if(type == 0) // BYTE
        digits = 2;
    else if(type == 1) // INT/WORD
        digits = 4;
    else // LONG
        digits = 8;
    
    uart_puts("0x");
    
    // Convert to hex string
    for(i = 0; i < digits; i++)
    {
        nibble = (value >> ((digits - 1 - i) * 4)) & 0x0F;
        if(nibble < 10)
            uart_putc('0' + nibble);
        else
            uart_putc('A' + nibble - 10);
    }
}

// Type definitions for transmitHex
#define BYTE 0
#define INT  1
#define LONG 2

#endif // UART_COMPAT_H