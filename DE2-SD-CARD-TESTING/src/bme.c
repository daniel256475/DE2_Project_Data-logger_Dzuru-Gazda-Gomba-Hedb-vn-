#include <avr/io.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include "twi.h"
#include "bme280.h"

/* Expose simple init + single-measure API for integration into a combined main
 * The file provides:
 *  - int bme_init_simple(struct bme280_dev *dev, uint8_t dev_addr)
 *  - int bme_read_once(struct bme280_dev *dev, int32_t *t100, uint32_t *press_pa, uint32_t *hum_x1024)
 *
 * Note: this module uses the project's TWI helper functions (twi.h) for I2C
 * callbacks. The caller must call `twi_init()` before `bme_init_simple()` and
 * is responsible for UART output.
 */

/* Manufacturer integer compensation functions (from Bosch Sensortec). Returns:
 *  - Temperature: int32 in 0.01 degC
 *  - Pressure: uint32 in Q24.8 (Pa * 256)
 *  - Humidity: uint32 in Q22.10 (percent * 1024)
 */

 /**
 * @brief  Compensate raw temperature ADC value to actual temperature using calibration data.
 * @param  adc_T Raw temperature ADC value from BME280 sensor.
 * @param  calib Pointer to calibration data structure.
 * @return Temperature in 0.01 °C units (e.g., 2500 = 25.00 °C).
 */
static int32_t BME280_compensate_T_int32(int32_t adc_T, struct bme280_calib_data *calib)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib->dig_t1 << 1))) * ((int32_t)calib->dig_t2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib->dig_t1)) * ((adc_T >> 4) - ((int32_t)calib->dig_t1))) >> 12) *
            ((int32_t)calib->dig_t3)) >> 14;
    calib->t_fine = var1 + var2;
    T = (calib->t_fine * 5 + 128) >> 8;
    return T;
}

/**
 * @brief  Compensate raw pressure ADC value to actual pressure using calibration data.
 * @param  adc_P Raw pressure ADC value from BME280 sensor.
 * @param  calib Pointer to calibration data structure.
 * @return Pressure in Q24.8 format (Pa * 256).
 */
static uint32_t BME280_compensate_P_int64(int32_t adc_P, struct bme280_calib_data *calib)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)calib->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib->dig_p6;
    var2 = var2 + ((var1 * (int64_t)calib->dig_p5) << 17);
    var2 = var2 + (((int64_t)calib->dig_p4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib->dig_p3) >> 8) + ((var1 * (int64_t)calib->dig_p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)calib->dig_p1)) >> 33;
    if (var1 == 0)
    {
        return 0; /* avoid exception caused by division by zero */
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib->dig_p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib->dig_p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib->dig_p7) << 4);
    return (uint32_t)p;
}

/**
 * @brief  Compensate raw humidity ADC value to actual humidity using calibration data.
 * @param  adc_H Raw humidity ADC value from BME280 sensor.
 * @param  calib Pointer to calibration data structure.
 * @return Humidity in Q22.10 format (percent * 1024).
 */
static uint32_t bme280_compensate_H_int32(int32_t adc_H, struct bme280_calib_data *calib)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (calib->t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib->dig_h4) << 20) - (((int32_t)calib->dig_h5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)calib->dig_h6)) >> 10) * (((v_x1_u32r * ((int32_t)calib->dig_h3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)calib->dig_h2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib->dig_h1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

/**
 * @brief  I2C read callback for Bosch BME280 driver.
 * @param  reg_addr Starting register address to read from.
 * @param  reg_data Pointer to buffer for received data.
 * @param  len      Number of bytes to read.
 * @param  intf_ptr Pointer to I2C address (7-bit).
 * @return BME280_INTF_RET_SUCCESS on success, error code otherwise.
 */
static BME280_INTF_RET_TYPE user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *((uint8_t *)intf_ptr);
    if (len == 0 || reg_data == NULL)
        return BME280_E_INVALID_LEN;

    twi_readfrom_mem_into(dev_addr, reg_addr, (volatile uint8_t *)reg_data, (uint8_t)len);
    return BME280_INTF_RET_SUCCESS;
}

/**
 * @brief  I2C write callback for Bosch BME280 driver.
 * @param  reg_addr Starting register address to write to.
 * @param  reg_data Pointer to data to write.
 * @param  len      Number of bytes to write.
 * @param  intf_ptr Pointer to I2C address (7-bit).
 * @return BME280_INTF_RET_SUCCESS on success, error code otherwise.
 */
static BME280_INTF_RET_TYPE user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *((uint8_t *)intf_ptr);
    twi_start();
    if (twi_write((dev_addr << 1) | TWI_WRITE) != 0) { twi_stop(); return BME280_E_COMM_FAIL; }
    if (twi_write(reg_addr) != 0) { twi_stop(); return BME280_E_COMM_FAIL; }
    for (uint32_t i = 0; i < len; i++)
    {
        if (twi_write(reg_data[i]) != 0) { twi_stop(); return BME280_E_COMM_FAIL; }
    }
    twi_stop();
    return BME280_INTF_RET_SUCCESS;
}

/**
 * @brief  Delay callback for Bosch BME280 driver.
 * @param  period   Delay duration in microseconds.
 * @param  intf_ptr Unused interface pointer.
 * @return none
 */
static void user_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    while (period >= 1000)
    {
        _delay_ms(1);
        period -= 1000;
    }
    while (period--)
        _delay_us(1);
}

/**
 * @brief  Initialize BME280 device and configure basic sensor settings.
 * 
 * Caller must call `twi_init()` before calling this function.
 * 
 * @param  dev           Pointer to BME280 device structure.
 * @param  dev_addr_ptr  Pointer to 7-bit I2C address (e.g., BME280_I2C_ADDR_PRIM).
 * @return BME280_OK (0) on success, negative error code otherwise.
 */
int bme_init_simple(struct bme280_dev *dev, uint8_t *dev_addr_ptr)
{
    dev->intf = BME280_I2C_INTF;
    dev->intf_ptr = dev_addr_ptr; /* use caller-owned address storage */
    dev->read = user_i2c_read;
    dev->write = user_i2c_write;
    dev->delay_us = user_delay_us;

    int8_t rslt = bme280_init(dev);
    if (rslt != BME280_OK)
    {
        return rslt;
    }

    /* Configure sensor: oversampling x1 for T/P/H */
    struct bme280_settings settings;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.filter = BME280_FILTER_COEFF_OFF;
    settings.standby_time = BME280_STANDBY_TIME_125_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, dev);
    if (rslt != BME280_OK)
    {
        return rslt;
    }
    return BME280_OK;
}

/**
 * @brief  Perform one forced measurement and return compensated sensor values.
 * @param  dev       Pointer to BME280 device structure (initialized by bme_init_simple).
 * @param  t100      Pointer to store temperature in 0.01 °C units.
 * @param  press_pa  Pointer to store pressure in Pascal (Pa).
 * @param  hum_x1024 Pointer to store humidity in percent × 1024.
 * @return 0 on success, negative error code otherwise.
 */
int bme_read_once(struct bme280_dev *dev, int32_t *t100, uint32_t *press_pa, uint32_t *hum_x1024)
{
    /* Trigger one-shot measurement (forced) */
    int8_t rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, dev);
    if (rslt != BME280_OK) return -1;

    /* Wait required measurement time */
    struct bme280_settings settings;
    /* read back current settings to compute delay
     * If reading settings fails, fall back to a conservative delay
     */
    rslt = bme280_get_sensor_settings(&settings, dev);
    uint32_t meas_delay_us = 0;
    if (rslt == BME280_OK)
        bme280_cal_meas_delay(&meas_delay_us, &settings);
    else
        meas_delay_us = 10000; /* 10 ms conservative */

    user_delay_us(meas_delay_us, dev->intf_ptr);

    /* Read raw measurement registers (8 bytes: press(3), temp(3), hum(2)) */
    uint8_t data[8];
    rslt = bme280_get_regs(BME280_REG_DATA, data, BME280_LEN_P_T_H_DATA, dev);
    if (rslt != BME280_OK)
    {
        return -2;
    }

    uint32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
    uint32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4);
    uint32_t adc_H = ((uint32_t)data[6] << 8) | (uint32_t)data[7];

    /* Compensate using manufacturer's integer routines */
    *t100 = BME280_compensate_T_int32((int32_t)adc_T, &dev->calib_data); /* 0.01 degC */
    uint32_t p_q24_8 = BME280_compensate_P_int64((int32_t)adc_P, &dev->calib_data); /* Q24.8 */
    *hum_x1024 = bme280_compensate_H_int32((int32_t)adc_H, &dev->calib_data); /* Q22.10 */

    /* Convert pressure Q24.8 -> Pa (divide by 256) */
    *press_pa = (uint32_t)((uint64_t)p_q24_8 / 256ULL);

    return 0;
}