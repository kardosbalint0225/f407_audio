#ifdef TEST

#ifndef __I2C_EX_H
#define __I2C_EX_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(I2C_FLTR_ANOFF)&&defined(I2C_FLTR_DNF)

#include "def.h"

#define I2C_ANALOGFILTER_ENABLE        0x00000000U
#define I2C_ANALOGFILTER_DISABLE       I2C_FLTR_ANOFF

HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter);

#endif

#ifdef __cplusplus
}
#endif

#endif 

#endif

