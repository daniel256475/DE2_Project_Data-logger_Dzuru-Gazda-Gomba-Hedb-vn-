#ifndef RTC_H
    #define RTC_H
#endif  
// #include <avr/io.h>
#include <stdint.h>

/* I2C address of DS1307 RTC */
#define RTC_ADDRESS 0x68

/* RTC register addresses (DS1307) */
#define RTC_SEC     0x00  /* Seconds (and CH bit) */
#define RTC_MIN     0x01  /* Minutes */
#define RTC_HOUR    0x02  /* Hours (12/24 bit) */
#define RTC_DAY     0x03  /* Day of week */
#define RTC_DATE    0x04  /* Date (day of month) */
#define RTC_MONTH   0x05  /* Month */
#define RTC_YEAR    0x06  /* Year (00..99) */
#define RTC_CONTROL 0x07  /* Control register (square-wave output) */

/**
 * @brief  Initialize RTC device: start oscillator, select 24-hour mode, and clear control register.
 * @return None.
 */
void rtc_setup(void);

/**
 * @brief  Read a single RTC register over I2C.
 * @param  reg_addr Register address (use RTC_* macros).
 * @return Byte value read from the device.
 */
uint8_t rtc_read_reg(uint8_t reg_addr);


/**
 * @brief  Write a single RTC register over I2C.
 * @param  reg_addr Register address (use RTC_* macros).
 * @param  data     Byte to write.
 * @return Result code from underlying TWI write (0 = success).
 */
uint8_t rtc_write_reg(uint8_t reg_addr, uint8_t data);

/**
 * @brief  Set RTC time (hours, minutes, seconds) in decimal format.
 * @param  hours Hour value in range 0–23.
 * @param  minutes Minute value in range 0–59.
 * @param  seconds Second value in range 0–59.
 * @return 0 on success (currently always returns 0).
 */
uint8_t rtc_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds);

/**
 * @brief  Read current time from RTC and convert BCD to decimal.
 * @param  hours    Pointer to store hour value (must not be NULL).
 * @param  minutes  Pointer to store minute value (must not be NULL).
 * @param  seconds  Pointer to store second value (must not be NULL).
 * @return 0 on success.
 */
uint8_t rtc_get_time(uint8_t *hours, uint8_t *minutes, uint8_t *seconds);

/**
 * @brief  Set RTC calendar date (day of week, date, month, year) in decimal format.
 * @param  day    Day of week in range 1–7.
 * @param  date   Day of month in range 1–31.
 * @param  month  Month in range 1–12.
 * @param  year   Year in range 0–99.
 * @return 0 on success.
 */
uint8_t rtc_set_date(uint8_t day, uint8_t date, uint8_t month, uint8_t year);
