/* 
 * GPIO library for AVR-GCC.
 * (c) 2019-2024 Tomas Fryza, MIT license
 *
 * Developed using PlatformIO and AVR 8-bit Toolchain 3.6.2.
 * Tested on Arduino Uno board and ATmega328P, 16 MHz.
 */

// -- Includes -------------------------------------------------------
#include <gpio.h>


/**
 * @brief  Configure one output pin.
 * @param  reg Address of Data Direction Register, such as &DDRB
 * @param  pin Pin designation in the interval 0 to 7
 * @return None
 */

void gpio_mode_output(volatile uint8_t *reg, uint8_t pin)
{
    *reg = *reg | (1<<pin);
}


/**
 * @brief  Configure one input pin and enable internal pull-up resistor.
 * @param  reg Address of Data Direction Register, such as &DDRB
 * @param  pin Pin designation in the interval 0 to 7
 * @return None
 */

void gpio_mode_input_pullup(volatile uint8_t *reg, uint8_t pin)
{
    *reg = *reg & ~(1<<pin);  // Data Direction Register
    reg++;                    // Change pointer to Data Register
    *reg = *reg | (1<<pin);   // Data Register
}


/**
 * @brief  Write one pin to a low logic level.
 * @param  reg Address of Port Register, such as &PORTB
 * @param  pin Pin designation in the interval 0 to 7
 * @return None
 */

void gpio_write_low(volatile uint8_t *reg, uint8_t pin)
{
    *reg = *reg & ~(1<<pin);   // Data Direction Register
}


/**
 * @brief  Write one pin to a high logic level.
 * @param  reg Address of Port Register, such as &PORTB
 * @param  pin Pin designation in the interval 0 to 7
 * @return None
 */

void gpio_write_high(volatile uint8_t *reg, uint8_t pin)
{
    *reg = *reg | (1<<pin);   // Data Register
}


/**
 * @brief  Read a value from input pin.
 * @param  reg Address of Pin Register, such as &PINB
 * @param  pin Pin designation in the interval 0 to 7
 * @return Pin value (0 or 1)
 */

uint8_t gpio_read(volatile uint8_t *reg, uint8_t pin)
{
    uint8_t temp;

    temp = *reg & (1<<pin);

    if (temp != 0) {
        return 1;
    }
    else {
        return 0;
    }
}

/**
 * @brief  Configure one input pin without pull-up resistor.
 * @param  reg Address of Data Direction Register, such as &DDRB.
 * @param  pin Pin designation in the interval 0 to 7.
 * @return None.
 */

void gpio_mode_input_nopull(volatile uint8_t *reg, uint8_t pin)
{
    *reg = *reg & ~(1<<pin);   // Data Direction Register
}

/**
 * @brief  Invert the state of one output pin and clear its data register.
 * @param  reg Address of Port Register, such as &PORTB
 * @param  pin Pin designation in the interval 0 to 7
 * @return None
 */

 void gpio_toggle(volatile uint8_t *reg, uint8_t pin)
 {
    *reg = *reg ^ (1 << pin);
    reg++;                    // Change pointer to Data Register
    *reg = *reg & ~(1<<pin);  // Data Direction Register
 }
