/**
 * @author Alexander Grin
 * @copyright (C) 2020 Grin Development. All rights reserved.
 */

#ifndef TCA9555_DRIVER_H
#define TCA9555_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*************************** C types headers *****************************/
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/**
 * @brief TCA9555 error code
 */
typedef enum {
    TCA9555_OK = 0,
    TCA9555_E_FAILED = -1,
    TCA9555_E_NULL_PTR = -2,
    TCA9555_E_INVALID_ARG = -3,
    TCA9555_E_I2C_COM_FAILED = -4
} tca9555_err_t;

/**
 * @brief I2C slave address
 */
typedef enum {
    TCA9555_ADDR_A2L_A1L_A0L = 0x20,
    TCA9555_ADDR_A2L_A1L_A0H,
    TCA9555_ADDR_A2L_A1H_A0L,
    TCA9555_ADDR_A2L_A1H_A0H,
    TCA9555_ADDR_A2H_A1L_A0L,
    TCA9555_ADDR_A2H_A1L_A0H,
    TCA9555_ADDR_A2H_A1H_A0L,
    TCA9555_ADDR_A2H_A1H_A0H,
    TCA9555_ADDR_MAX
} tca9555_addr_t;

/**
 * @brief TCA9555 GPIO pins
 */
typedef enum {
    TCA9555_GPIO_P00 = 0,
    TCA9555_GPIO_P01,
    TCA9555_GPIO_P02,
    TCA9555_GPIO_P03,
    TCA9555_GPIO_P04,
    TCA9555_GPIO_P05,
    TCA9555_GPIO_P06,
    TCA9555_GPIO_P07,
    TCA9555_GPIO_P10,
    TCA9555_GPIO_P11,
    TCA9555_GPIO_P12,
    TCA9555_GPIO_P13,
    TCA9555_GPIO_P14,
    TCA9555_GPIO_P15,
    TCA9555_GPIO_P16,
    TCA9555_GPIO_P17,
    TCA9555_GPIO_MAX
} tca9555_gpio_pin_t;

/**
 * @brief TCA9555 GPIO mode
 */
typedef enum {
    TCA9555_GPIO_OUTPUT = 0,
    TCA9555_GPIO_INPUT = 1
} tca9555_gpio_mode_t;

/**
 * @brief TCA9555 GPIO input polarity
 */
typedef enum {
    TCA9555_GPIO_POLARITY_ORIGINAL = 0,
    TCA9555_GPIO_POLARITY_INVERT = 1
} tca9555_gpio_polarity_t;

/**
 * @brief TCA9555 GPIO state
 */
typedef enum {
    TCA9555_GPIO_PIN_RESET = 0,
    TCA9555_GPIO_PIN_SET = 1
} tca9555_gpio_state_t;

/**
 * @brief I2C communication function pointer
 *
 * @param dev_addr - device address
 * @param data - pointer to the data buffer
 * @param len - data buffer length
 * @return TCA9555_OK - communication success
 *         other - error code from tca9555_err_t enumeration
 */
typedef int8_t (*tca955_i2c_com_fptr_t)(uint8_t dev_addr, uint8_t reg,
        uint8_t *data, uint16_t len);

typedef struct {
    const tca955_i2c_com_fptr_t i2c_read;
    const tca955_i2c_com_fptr_t i2c_write;
    const tca9555_addr_t i2c_addr;
} tca9555_dev_t;

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
        tca9555_gpio_mode_t mode);

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
        tca9555_gpio_pin_t pin, tca9555_gpio_polarity_t polarity);

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
        tca9555_gpio_pin_t pin, tca9555_gpio_state_t *state);

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
        tca9555_gpio_state_t state);

/**
 * @brief Toggle GPIO pin
 *
 * @param device - pointer to device structure
 * @param pin - GPIO pin
 * @return TCA9555_OK - success
 *         other - error code from tca9555_err_t enumeration
 */
tca9555_err_t tca9555_gpio_toggle(tca9555_dev_t *device, tca9555_gpio_pin_t pin);

#ifdef __cplusplus
}
#endif

#endif //!TCA9555_DRIVER_H
