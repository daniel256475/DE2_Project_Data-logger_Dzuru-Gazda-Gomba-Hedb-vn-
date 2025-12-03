/*
 * Use USART unit and transmit data between ATmega328P and computer.
 * (c) 2018-2025 Tomas Fryza, MIT license
 *
 * Developed using PlatformIO and Atmel AVR platform.
 * Tested on Arduino Uno board and ATmega328P, 16 MHz.
 */

// -- Includes ---------------------------------------------
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <uart.h>           // Peter Fleury's UART library
#include <stdlib.h>         // C library. Needed for number conversions
#include "SPI_routines.h"
#include "sd_routines.h"
#include "FAT32.h"
#include "gpio.h"
#include "rtc.h"
#include <twi.h>
#include "bme280.h"
#include "SensirionI2CSgp41.h"
#include <math.h>
#include <string.h>
#include <util/delay.h>
#include <stdio.h>

#define UART_ON
#define SD_write
#define ACTIVITY_LED_PORT   PORTC
#define L_ACT    2
#define STATUS_LED_PORT     PORTC
#define L_STATUS      1
#define ERROR_LED_PORT      PORTC
#define L_ERROR       0
volatile uint8_t printRTC = 0;
volatile uint8_t counterTim1 = 0;

uint8_t SD_OK, FS_OK, BM_OK, SGP_OK, RTC_OK = 0;

/**
 * @brief  Initialize BME280 sensor and configure basic settings.
 * @param  dev          Pointer to BME280 device structure.
 * @param  dev_addr_ptr Pointer to 7-bit I2C address.
 * @return BME280_OK on success, error code otherwise.
 */
int bme_init_simple(struct bme280_dev *dev, uint8_t *dev_addr_ptr);

/**
 * @brief  Perform one measurement from BME280 sensor.
 * @param  dev       Pointer to BME280 device structure.
 * @param  t100      Pointer to store temperature in 0.01 °C units.
 * @param  press_pa  Pointer to store pressure in Pascal.
 * @param  hum_x1024 Pointer to store humidity in percent × 1024.
 * @return 0 on success, error code otherwise.
 */
int bme_read_once(struct bme280_dev *dev, int32_t *t100, uint32_t *press_pa, uint32_t *hum_x1024);

/**
 * @brief  Initialize SGP41 sensor and algorithms.
 * @return 0 on success, non-zero on error.
 */
int sgp41_init_simple(void);

/**
 * @brief  Perform one measurement from SGP41 sensor.
 * @param  voc_index Pointer to store VOC air quality index.
 * @param  nox_index Pointer to store NOx air quality index.
 * @return 0 on success, non-zero on error.
 */
int sgp41_measure_once(int32_t *voc_index, int32_t *nox_index);

volatile uint8_t measurement_flag = 0;

/**
 * @brief  Convert ASCII month string sum to month number (1-12).
 * @param  sum ASCII sum of month string (e.g., "Dec" = 268).
 * @return Month number (1-12), or 0 if invalid.
 */
uint8_t get_month_from_ascii_sum(uint16_t sum) {
    switch(sum) {
        case 268: return 12; // Dec
        case 269: return 2;  // Feb
        case 281: return 1;  // Jan
        case 285: return 8;  // Aug
        case 288: return 3;  // Mar
        case 291: return 4;  // Apr
        case 294: return 10; // Oct
        case 295: return 5;  // May
        case 296: return 9;  // Sep
        case 299: return 7;  // Jul
        case 301: return 6;  // Jun
        case 307: return 11; // Nov
        default: return 0;   // Error
    }
}

/**
 * @brief  Configure external interrupt INT0 on falling edge (PD2).
 * @return none
 */
static void setup_ext_int(void)
{
    // PD2 = INT0
    gpio_mode_input_nopull(&DDRD, 2);   // set PD2 as input with pull-up (gpio.h only)
    EICRA &= ~((1 << ISC00));           // clear bits for INT0/INT1 in EICRA
    EICRA |= (1 << ISC01);
    // EICRA |= (1 << ISC01);           // ISC01 = 1, ISC00 = 0 -> falling edge
    EIMSK |= (1 << INT0);               // enable external interrupt INT0

    // PCICR |= (1 << PCIE0);           // enable pin change interrupt for PCINT[7:0] (PORTB)
    // PCMSK0 |= (1 << PCINT2);         // enable pin change interrupt
}

/**
 * @brief  Select interrupt source: RTC external interrupt or Timer1 overflow.
 * 
 * If RTC is available, use external interrupt INT0 on PD2 (RTC square-wave output).
 * Otherwise, use Timer1 overflow interrupt.
 * 
 * @return none
 */
void set_interrupt_source(void)
{
    if (RTC_OK == 0)
    {
        // RTC is OK, use external interrupt on PD2
        tim1_ovf_disable();
        rtc_write_reg(0x0e, 0x00); // Enable square-wave output
        setup_ext_int();
    }
    else
    {   
        // RTC not OK, use Timer1 interrupt
        EIMSK &= ~(1 << INT0);  // Disable INT0
        tim1_ovf_enable();
    }
}

/**
 * @brief  Receive date and time from UART and set RTC.
 * 
 * Expected format: YYYY,MM,DD,HH,MM,SS,DOW
 * where DOW = 1 (Sunday) to 7 (Saturday).
 * 
 * @return none
 */
void set_time_uart(void)
{
    uart_puts("Waiting for time data (YYYY,MM,DD,HH,MM,SS,DOW)...\r\n");
    
    char time_buffer[30];
    uint8_t idx = 0;
    
    while(1) {
        uint16_t data = uart_getc();
        if(data & UART_NO_DATA) continue;
        
        char ch = (char)(data & 0xFF);
        
        if(ch == '\r' || ch == '\n') break;
        
        time_buffer[idx++] = ch;
        if(idx >= 29) break;
    }
    time_buffer[idx] = '\0';
    
    // Parse: YYYY,MM,DD,HH,MM,SS,DOW
    int year, month, day, hour, minute, second, dow;
    if(sscanf(time_buffer, "%d,%d,%d,%d,%d,%d,%d", 
              &year, &month, &day, &hour, &minute, &second, &dow) == 7) {
        rtc_set_time(hour, minute, second);
        rtc_set_date(dow, day, month, (year % 100));
        // rtc_set_day(dow);  // 1=Sunday ... 7=Saturday
        
        const char* days[] = {"", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
        char msg[60];
        sprintf(msg, "Time set: %04d-%02d-%02d (%s) %02d:%02d:%02d\r\n", 
                year, month, day, days[dow], hour, minute, second);
        uart_puts(msg);
    } else {
        uart_puts("Error parsing time!\r\n");
    }
}

/**
 * @brief  Main program entry point.
 * 
 * Initializes all peripherals (UART, SPI, SD, FAT32, I2C, sensors, RTC, interrupts).
 * Continuously monitors sensor data and logs to SD card at regular intervals.
 * Handles multiple sensor failures gracefully.
 * 
 * @return Program return code (0 on normal exit).
 */
int main(void)
{
    tim1_ovf_1sec();
    uint8_t hour, minute, second; 
    uint8_t day, date, month, year;
    
    // unsigned char error;
    // unsigned char fileName[] = "DATA.TXT";
    
    // Initialize UART
    uart_init(UART_BAUD_SELECT(115200, F_CPU));
    twi_init();
    // Configure SPI pins for ATmega328P
    gpio_mode_output(&DDRB, PB3);  // MOSI
    gpio_mode_output(&DDRB, PB5);  // SCK
    gpio_mode_output(&DDRD, PD4);  // SS (Chip Select)
    gpio_mode_input_nopull(&DDRB, PB4);  // MISO as input
    
    // Set SS high initially (SD card deselected)
    gpio_write_high(&PORTD, PIND4);

    gpio_mode_output(&DDRC, 0); //led red   ERROR
    gpio_mode_output(&DDRC, 1); //led green STATUS
    gpio_mode_output(&DDRC, 2); //led yellow ACT
    gpio_write_high(&PORTC, 0);
    gpio_write_high(&PORTC, 1);
    gpio_write_high(&PORTC, 2);
    
    sei();
    
    gpio_write_low(&ACTIVITY_LED_PORT, L_ACT); // ACT ON
    // gpio_write_low(&STATUS_LED_PORT, L_STATUS); // STATUS ON
    gpio_write_low(&ERROR_LED_PORT, L_ERROR); // ERROR ON

    #ifdef UART_ON
        uart_puts_P("\r\n=== SD Card FAT32 Test ===\r\n");
    #endif
    // Initialize SPI
    #ifdef SD_write
    spi_init();
    #ifdef UART_ON
    uart_puts_P("SPI initialized\r\n");
    #endif
    
    // Step 1: Initialize SD card
    #ifdef UART_ON
    uart_puts_P("Initializing SD card...\r\n");
    #endif
    // error = SD_init();
    
    if(SD_init())
    {
        #ifdef UART_ON
            uart_puts_P("SD Card Initialization Error!\r\n");
            uart_puts_P("System continuing without SD card...\r\n");
        #endif
        SD_OK = 1;
        // while(1);  // Stop execution
    }
    #ifdef UART_ON
        uart_puts_P("SD card initialized successfully!\r\n");
        // Step 2: Initialize FAT32
        uart_puts_P("Reading boot sector...\r\n");
    #endif
    
    if(getBootSectorData())
    {
        #ifdef UART_ON
            uart_puts_P("FAT32 initialization failed!\r\n");
            uart_puts_P("System continuing without SD card...\r\n");
        #endif
        FS_OK = 1;
        // while(1);  // Stop execution
    }
    else
    {
        #ifdef UART_ON
            uart_puts_P("FAT32 initialized successfully!\r\n");
        #endif
        // uart_puts_P("\r\n");
        FS_OK = 0;
    }
    #endif
     /* Initialize BME280 */
    struct bme280_dev bme_dev;
    uint8_t bme_addr = BME280_I2C_ADDR_PRIM;
    if (bme_init_simple(&bme_dev, &bme_addr) != BME280_OK) {
        
        #ifdef UART_ON
            uart_puts_P("BME init failed\r\n");
        #endif
        BM_OK = 1;
        // while (1);
    }
    _delay_ms(100);
    /* Initialize SGP41 */
    if (sgp41_init_simple() != 0) 
    {
        #ifdef UART_ON
            uart_puts_P("SGP41 init warning\r\n");
        #endif
        SGP_OK = 1;
        // while (1);
    }
    if (twi_test_address(RTC_ADDRESS))
    {
        #ifdef UART_ON
            uart_puts_P("RTC not found!\r\n");
            uart_puts_P("System using Compile time\r\n");
        #endif
        RTC_OK = 1;
        // Set compile time to local variables (RTC not available)
        int hour_comp, minute_comp, second_comp;
        sscanf(__TIME__, "%d:%d:%d", &hour_comp, &minute_comp, &second_comp);
        hour = (uint8_t)hour_comp;
        minute = (uint8_t)minute_comp;
        second = (uint8_t)second_comp;

        int year_comp, month_comp, day_comp;
        char month_str[4];
        sscanf(__DATE__, "%s %d %d", month_str, &day_comp, &year_comp);
        uint16_t month_sum = month_str[0] + month_str[1] + month_str[2];
        month_comp = get_month_from_ascii_sum(month_sum);
        
        date = (uint8_t)day_comp;
        month = (uint8_t)month_comp;
        year = (uint8_t)(year_comp % 100);
    }
    else
    {
        #ifdef UART_ON
            uart_puts_P("RTC found!\r\n");
        #endif
        // RTC_OK = 0;
        rtc_write_reg(0x0e, 0x00); // Enable square-wave output
    }
    if (RTC_OK & SGP_OK & SD_OK & FS_OK & BM_OK)
    {
        #ifdef UART_ON
            uart_puts_P("No sensors and SD card available, system halted!\r\n");
        #endif
        gpio_write_high(&ACTIVITY_LED_PORT, L_ACT); // ACT OFF
        while(1);
    }
    else if (RTC_OK | SGP_OK | BM_OK | SD_OK | FS_OK)
    {
        #ifdef UART_ON
            uart_puts_P("System initialized with warnings!\r\n");
        #endif
        gpio_write_low(&ERROR_LED_PORT, L_ERROR); // ERROR ON
    }
    
    else
    {
        #ifdef UART_ON
            uart_puts_P("System initialized successfully!\r\n");
        #endif
        gpio_write_high(&ERROR_LED_PORT, L_ERROR); // ERROR OFF
        gpio_write_high(&STATUS_LED_PORT, L_ACT); // ACT OFF
        gpio_write_low(&STATUS_LED_PORT, L_STATUS); // STATUS ON
    }

    char buffer[50];
    // sprintf(buffer, "Date: %02d/%02d/%04d\r\n", day, month, year);
    // uart_puts(buffer);
    // sprintf(buffer, "Time COMPILE: %02d:%02d:%02d\r\n", hour, minute, second);
    // uart_puts(buffer);
    // rtc_get_time(&hour, &minute, &second);
    // sprintf(buffer, "Time RTC: %02d:%02d:%02d\r\n", hour, minute, second);
    // uart_puts(buffer);
    // _delay_ms(100);
    // int hour_comp, minute_comp, second_comp;
    // sscanf(__TIME__, "%d:%d:%d", &hour_comp, &minute_comp, &second_comp);
    // rtc_set_time((uint8_t)hour_comp, (uint8_t)minute_comp, (uint8_t)second_comp); // Set time to compile time
    // rtc_set_date(7, 29, 11, 25);
    set_interrupt_source();
    
    while (1)
    {   
        #ifdef UART_ON
        if(printRTC == 1)
        {
            printRTC = 0;
            if (!(twi_test_address(RTC_ADDRESS)))
            {
                rtc_get_time(&hour, &minute, &second);
                sprintf(buffer, "Time RTC: %02d:%02d:%02d\r\n", hour, minute, second);
                uart_puts(buffer);
                rtc_get_date(&date, &month, &year);
                sprintf(buffer, "Date RTC: %02d/%02d/20%02d\r\n", date, month, year);
                uart_puts(buffer);
                // gpio_write_high(&ERROR_LED_PORT, L_ERROR); // ERROR OFF
                RTC_OK = 0;
            }
            else
            {
                // #ifdef UART_ON
                uart_puts_P("RTC not found, using Compile time\r\n");
                // #endif
                gpio_write_low(&ERROR_LED_PORT, L_ERROR); // ERROR ON
            }
            
            
            
        }
        #endif

        if(RTC_OK | SGP_OK | BM_OK | SD_OK | FS_OK)
        {
            gpio_write_low(&ERROR_LED_PORT, L_ERROR); // ERROR ON
        }
        else
        {
            gpio_write_high(&ERROR_LED_PORT, L_ERROR); // ERROR OFF
        }

        if (measurement_flag)
        {
            measurement_flag = 0;
            // uart_puts_P("Starting measurement...\r\n");
            

            /* Read sensors */
            char sdString[100];
            memset(sdString, 0, sizeof(sdString));  //Reset at the beginning
            
            int32_t t100 = 0;
            uint32_t press_pa = 0;
            uint32_t hum_x1024 = 0;
            
            if(twi_test_address(RTC_ADDRESS))
            {
                RTC_OK = 1;
                set_interrupt_source();
            }
            else
            {
                RTC_OK = 0;
                rtc_get_time(&hour, &minute, &second);
                rtc_get_date(&date, &month, &year);
                set_interrupt_source();
            }
            
            snprintf(sdString, sizeof(sdString), "%02d:%02d:%02d,%02d/%02d/20%02d,",
                     hour, minute, second, date, month, year);
            
            if (bme_read_once(&bme_dev, &t100, &press_pa, &hum_x1024) == 0) {
                int32_t temp_int = t100 / 100;
                int32_t temp_frac = (t100 >= 0) ? (t100 % 100) : ((-t100) % 100);
                
                uint32_t press_hpa_int = press_pa / 100;
                uint32_t press_hpa_frac = press_pa % 100;
                
                uint32_t hum_percent_x100 = (hum_x1024 * 100 + 512) / 1024;
                uint32_t hum_int = hum_percent_x100 / 100;
                uint32_t hum_frac = hum_percent_x100 % 100;
                
                double P = (double)press_pa;
                double alt = 44330.0 * (1.0 - pow(P / 101325.0, 0.19029495718363465));
                int32_t alt_int = (int32_t)alt;
                int32_t alt_frac = (int32_t)(fabs(alt - (double)alt_int) * 100.0 + 0.5);
                
                char temp_buf[64];
                snprintf(temp_buf, sizeof(temp_buf), "%02ld.%02ld,%03lu.%02lu,%02lu.%02lu,%03ld.%02ld,",
                         (long)temp_int, (long)temp_frac,
                         (unsigned long)press_hpa_int, (unsigned long)press_hpa_frac,
                         (unsigned long)hum_int, (unsigned long)hum_frac,
                         (long)alt_int, (long)alt_frac);
                strncat(sdString, temp_buf, sizeof(sdString) - strlen(sdString) - 1);
                BM_OK = 0;
            } else {
                gpio_write_low(&ERROR_LED_PORT, L_ERROR); // ERROR ON
                strncat(sdString, "ERR,ERR,ERR,ERR,", sizeof(sdString) - strlen(sdString) - 1);
                BM_OK = 1;
            }
            
            /* Read SGP41 */
            int32_t voc_idx = 0;
            int32_t nox_idx = 0;
            if (sgp41_measure_once(&voc_idx, &nox_idx) == 0) {
                char temp_buf[32];
                snprintf(temp_buf, sizeof(temp_buf), "%ld,%ld\n", (long)voc_idx, (long)nox_idx);
                strncat(sdString, temp_buf, sizeof(sdString) - strlen(sdString) - 1);
                SGP_OK = 0;
            } else {
                strncat(sdString, "ERR,ERR\n", sizeof(sdString) - strlen(sdString) - 1);
                SGP_OK = 1;
            }
            
            #ifdef UART_ON
            // uart_puts(sdString);
            #endif
            uint8_t write_error = 0;
            /* Write to SD card */
            // uart_puts("Writing data to SD card...\r\n");
            // _delay_ms(100); // Small delay before SD write
            // uart_puts("FR");
            if (1)
            {
                // Reset dataString BEFORE copying
                // uart_puts("in condition\r\n");
                // _delay_ms(50);
                // cli();
                memset(dataString, 0, MAX_STRING_SIZE);
                
                // Copy sdString to dataString
                strncpy((char*)dataString, sdString, MAX_STRING_SIZE - 1);
                // dataString[MAX_STRING_SIZE - 1] = '\0';
                
                // Create a new copy of the file name
                unsigned char fileName[12];
                strcpy((char*)fileName, "data1.csv");
                
                gpio_write_low(&ACTIVITY_LED_PORT, L_ACT);
                #ifdef SD_write
                write_error = writeFile(fileName);
                #endif
                // #ifndef SD_write
                // write_error = 0;
                // #endif
                 if (write_error) {
                    sei();
                    uart_puts_P("SD write error!\r\n");
                    gpio_write_low(&ERROR_LED_PORT, L_ERROR);
                } else {
                    sei();
                    uart_puts_P("Data saved.\r\n");
                }
                gpio_write_high(&ACTIVITY_LED_PORT, L_ACT);
                
                // sei();
            }
            
            // Re-enable INT0
            
        }
        
    }
    return 0;
}

/**
 * @brief  Timer1 overflow interrupt handler (triggered every 1 second).
 * @return none
 */
ISR(TIMER1_OVF_vect)
{    
    // counterTim1++;
    // if (counterTim1>=5)
    // {
    //     measurement_flag = 1;
    // }
    
//     if(counterTim1 >= 10)  // Every 10 seconds
//     {
//         counterTim1 = 0;
//         // printRTC = 1;
//     }
    // cli();
    // Check if PC2 (PCINT10) caused the interrupt
    // if (PINC & (1 << PC2)) {
    //     // Rising edge detected
    //     measurement_flag = 1;
    // }
    counterTim1++;
    if (counterTim1 >= 10) // Every 5 SQW pulses (5 seconds)
    {
        counterTim1 = 0;    
        measurement_flag = 1;
        // uart_puts_P("SQW Interrupt Triggered\r\n");
        // gpio_toggle(&ACTIVITY_LED_PORT, L_ERROR);
    }
    // sei();
    // uart_puts_P("Timer1 Interrupt Triggered\r\n");
}

/**
 * @brief  External interrupt INT0 handler (PD2, triggered by RTC square-wave).
 * @return none
 */
ISR(INT0_vect)
{
    // cli();
    // Check if PC2 (PCINT10) caused the interrupt
    // if (PINC & (1 << PC2)) {
    //     // Rising edge detected
    //     measurement_flag = 1;
    // }
    counterTim1++;
    if (counterTim1 >= 10) // Every 5 SQW pulses (5 seconds)
    {
        counterTim1 = 0;    
        measurement_flag = 1;
        printRTC = 1;
        // uart_puts_P("SQW Interrupt Triggered\r\n");
        // gpio_toggle(&ACTIVITY_LED_PORT, L_ERROR);
    }
    // sei();
    // uart_puts_P("SQW Interrupt Triggered\r\n");
}

/**
 * @brief  External interrupt INT1 handler (PD3).
 * @return none
 */
ISR(INT1_vect)
{
    // uart_puts_P("INT1 (PD3) Triggered\r\n");
}