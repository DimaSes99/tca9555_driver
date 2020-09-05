/**
 * @author Alexander Grin
 * @copyright (C) 2020 Grin Development. All rights reserved.
 */

#include "tca9555_driver.h"

/****************************** Common macros ********************************/
/** C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL                                (0)
#else
#define NULL                                ((void *) 0)
#endif
#endif

/** Define Endian-ness */
#ifndef __ORDER_LITTLE_ENDIAN__
#define __ORDER_LITTLE_ENDIAN__             (0)
#endif

#ifndef __BYTE_ORDER__
#define __BYTE_ORDER__                      __ORDER_LITTLE_ENDIAN__
#endif

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN                       (1)
#endif
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#ifndef BIG_ENDIAN
#define BIG_ENDIAN                          (1)
#endif
#endif

#if ((BIG_ENDIAN) || (defined __C51__) || (defined __RC51__) || (defined _CC51))
#define htole16(x) (((x) >> 8) | ((x) << 8))
#define le16toh(x) (((x) >> 8) | ((x) << 8))
//! Little Endian Compilers: Bit swapping necessary
#elif ((LITTLE_ENDIAN) || (defined __GNUC__) || (defined __clang__) ||(defined SDCC) || (defined HI_TECH_C) || (defined __ICC8051__))
#define htole16(x) (x)
#define le16toh(x) (x)

#else
#error "Compiler not defined. Endian-ness of the compiler cannot be determined."
#endif

/************************** Bit operation macros *****************************/
#define BIT_SET(reg,nbit)                   (reg) |=  (1<<(nbit))
#define BIT_CLR(reg,nbit)                   (reg) &= ~(1<<(nbit))
#define BIT_TGL(reg,nbit)                   (reg) ^=  (1<<(nbit))
#define BIT_CHECK(reg,nbit)                 ((reg) &  (1<<(nbit)))


#define TCA9555_REGISTER_LEN                (0x02)

/**
 * @brief TCA9555 register
 */
enum tca9555_register
{
    TCA9555_INPUT_REG = 0x00,
    TCA9555_OUTPUT_REG = 0x02,
    TCA9555_POLARITY_INV_REG = 0x4,
    TCA9555_CONFIGURATION_REG = 0x6,
    TCA9555_MAX_REG
};

/**
 * @brief Check device pointer structure
 *
 * @param device - pointer to device struct
 * @return TCA9555_OK - success
 *         TCA9555_E_NULL_PTR - i2c_read or i2c_write functions is null
 *         TCA9555_E_FAILED - invalid i2c_addr
 */
static tca9555_err_t tca9555_device_check(const tca9555_dev_t *device)
{
    tca9555_err_t rslt = TCA9555_OK;
    if(device == NULL ||
        device->i2c_read == NULL ||
        device->i2c_write == NULL) {
        rslt = TCA9555_E_NULL_PTR;
    }
    if(device->i2c_addr >= TCA9555_ADDR_MAX) {
        rslt = TCA9555_E_FAILED;
    }
    return rslt;
}

/**
 * @brief Read TCA9555 register
 *
 * @param device - pointer to device struct
 * @param reg - read register
 * @param data - pointer to the data
 * @return TCA9555_OK - success
 *         TCA9555_E_NULL_PTR - i2c_read or i2c_write functions is null
 *         TCA9555_E_FAILED - invalid device i2c_addr or read register
 */
static tca9555_err_t tca9555_read_reg(const tca9555_dev_t *device, uint8_t reg,
        uint16_t *data)
{
    uint16_t rslt = TCA9555_OK;

    rslt = tca9555_device_check(device);
    if(rslt != TCA9555_OK)
        goto exit;

    if(reg >= TCA9555_MAX_REG) {
        rslt = TCA9555_E_FAILED;
        goto exit;
    }

    if(data == NULL) {
        rslt = TCA9555_E_NULL_PTR;
        goto exit;
    }

    rslt = device->i2c_read(device->i2c_addr, reg,
            (uint8_t *)data, TCA9555_REGISTER_LEN);
    if(rslt != TCA9555_OK) {
        rslt = TCA9555_E_I2C_COM_FAILED;
        goto exit;
    }

    *data = le16toh(*data);

    exit: return rslt;
}

/**
 * @brief Write TCA9555 register
 *
 * @param device - pointer to device struct
 * @param reg - write register
 * @param data - pointer to the data
 * @return TCA9555_OK - success
 *         TCA9555_E_NULL_PTR - i2c_read or i2c_write functions is null
 *         TCA9555_E_FAILED - invalid device i2c_addr or read register
 */
static tca9555_err_t tca9555_write_reg(const tca9555_dev_t *device, uint8_t reg,
        uint16_t *data)
{
    uint16_t rslt = TCA9555_OK;

    rslt = tca9555_device_check(device);
    if(rslt != TCA9555_OK)
        goto exit;

    if(reg >= TCA9555_MAX_REG) {
        rslt = TCA9555_E_FAILED;
        goto exit;
    }

    if(data == NULL) {
        rslt = TCA9555_E_NULL_PTR;
        goto exit;
    }

    *data = htole16(*data);

    rslt = device->i2c_write(device->i2c_addr, reg,
            (uint8_t *)data, TCA9555_REGISTER_LEN);
    if(rslt != TCA9555_OK) {
        rslt = TCA9555_E_I2C_COM_FAILED;
        goto exit;
    }

    exit: return rslt;
}

/**
 * @brief Initialization of GPIO pin
 *
 * @param device - pointer to device structure
 * @param pin - GPIO pin
 * @param mode - GPIO mode
 * @return TCA9555_OK - success
 *         other - error code from tca9555_err_t enumeration
 */
tca9555_err_t tca9555_gpio_init(tca9555_dev_t *device, tca9555_gpio_pin_t pin,
        tca9555_gpio_mode_t mode)
{
    tca9555_err_t rslt;
    uint16_t config_register = {0};

    rslt = tca9555_device_check(device);
    if(rslt != TCA9555_OK)
        goto exit;

    if(pin >= TCA9555_GPIO_MAX) {
        rslt = TCA9555_E_INVALID_ARG;
        goto exit;
    }

    if(mode != TCA9555_GPIO_OUTPUT && mode != TCA9555_GPIO_INPUT) {
        rslt = TCA9555_E_INVALID_ARG;
        goto exit;
    }

    rslt = tca9555_read_reg(device, TCA9555_CONFIGURATION_REG, &config_register);
    if(rslt != TCA9555_OK)
        goto exit;

    if(mode == TCA9555_GPIO_OUTPUT)
      BIT_CLR(config_register, pin);
    else
      BIT_SET(config_register, pin);

    rslt = tca9555_write_reg(device, TCA9555_CONFIGURATION_REG, &config_register);

    exit: return rslt;
}

/**
 * @brief Set polarity of pins configurated as inputs
 *
 * @param device - pointer to device structure
 * @param pin - GPIO pin
 * @param polarity - GPIO input pin polarity
 * @return TCA9555_OK - success
 *         other - error code from tca9555_err_t enumeration
 */
tca9555_err_t tca9555_gpio_polarity(tca9555_dev_t *device,
        tca9555_gpio_pin_t pin, tca9555_gpio_polarity_t polarity)
{
    tca9555_err_t rslt;
    uint16_t polarity_register = {0};

    rslt = tca9555_device_check(device);
    if(rslt != TCA9555_OK)
        goto exit;

    if(pin >= TCA9555_GPIO_MAX) {
        rslt = TCA9555_E_INVALID_ARG;
        goto exit;
    }

    if(polarity != TCA9555_GPIO_POLARITY_ORIGINAL &&
            polarity != TCA9555_GPIO_POLARITY_INVERT) {
        rslt = TCA9555_E_INVALID_ARG;
        goto exit;
    }

    rslt = tca9555_read_reg(device, TCA9555_POLARITY_INV_REG, &polarity_register);
    if(rslt != TCA9555_OK)
        goto exit;

    if(polarity == TCA9555_GPIO_POLARITY_INVERT)
      BIT_SET(polarity_register, pin);
    else
      BIT_CLR(polarity_register, pin);

    rslt = tca9555_write_reg(device, TCA9555_POLARITY_INV_REG, &polarity_register);

    exit: return rslt;
}

/**
 * @brief Read GPIO state
 *
 * @param device - pointer to device structure
 * @param pin - GPIO pin
 * @param state - pointer to return GPIO state value
 * @return TCA9555_OK - success
 *         other - error code from tca9555_err_t enumeration
 */
tca9555_err_t tca9555_gpio_read(tca9555_dev_t *device,
        tca9555_gpio_pin_t pin, tca9555_gpio_state_t *state)
{
    tca9555_err_t rslt;
    uint16_t input_register = {0};

    rslt = tca9555_device_check(device);
    if(rslt != TCA9555_OK)
        goto exit;

    if(pin >= TCA9555_GPIO_MAX) {
        rslt = TCA9555_E_INVALID_ARG;
        goto exit;
    }

    rslt = tca9555_read_reg(device, TCA9555_INPUT_REG, &input_register);
    if(rslt != TCA9555_OK)
        goto exit;

    if(BIT_CHECK(input_register, pin)) {
        *state = TCA9555_GPIO_PIN_SET;
    }
    else {
        *state = TCA9555_GPIO_PIN_RESET;
    }

    exit: return rslt;
}

/**
 * @brief Write GPIO state
 *
 * @param device - pointer to device structure
 * @param pin - GPIO pin
 * @param state - GPIO state
 * @return TCA9555_OK - success
 *         other - error code from tca9555_err_t enumeration
 */
tca9555_err_t tca9555_gpio_write(tca9555_dev_t *device, tca9555_gpio_pin_t pin,
        tca9555_gpio_state_t state)
{
    tca9555_err_t rslt;
    uint16_t output_register = {0};

    rslt = tca9555_device_check(device);
    if(rslt != TCA9555_OK)
        goto exit;

    if(pin >= TCA9555_GPIO_MAX) {
        rslt = TCA9555_E_INVALID_ARG;
        goto exit;
    }

    if(state != TCA9555_GPIO_PIN_RESET && state != TCA9555_GPIO_PIN_SET) {
        rslt = TCA9555_E_INVALID_ARG;
        goto exit;
    }

    rslt = tca9555_read_reg(device, TCA9555_OUTPUT_REG, &output_register);
    if(rslt != TCA9555_OK)
        goto exit;

    if(state == TCA9555_GPIO_PIN_SET)
      BIT_SET(output_register, pin);
    else
      BIT_CLR(output_register, pin);

    rslt = tca9555_write_reg(device, TCA9555_OUTPUT_REG, &output_register);

    exit: return rslt;
}

/**
 * @brief Toggle GPIO pin
 *
 * @param device - pointer to device structure
 * @param pin - GPIO pin
 * @return TCA9555_OK - success
 *         other - error code from tca9555_err_t enumeration
 */
tca9555_err_t tca9555_gpio_toggle(tca9555_dev_t *device, tca9555_gpio_pin_t pin)
{
    tca9555_err_t rslt;
    uint16_t output_register = {0};

    rslt = tca9555_device_check(device);
    if(rslt != TCA9555_OK)
        goto exit;

    if(pin >= TCA9555_GPIO_MAX) {
        rslt = TCA9555_E_INVALID_ARG;
        goto exit;
    }

    rslt = tca9555_read_reg(device, TCA9555_OUTPUT_REG, &output_register);
    if(rslt != TCA9555_OK)
        goto exit;

    if(BIT_CHECK(output_register, pin))
      BIT_CLR(output_register, pin);
    else
      BIT_SET(output_register, pin);

    rslt = tca9555_write_reg(device, TCA9555_OUTPUT_REG, &output_register);

    exit: return rslt;
}
