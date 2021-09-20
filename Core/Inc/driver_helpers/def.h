#ifdef TEST

#ifndef __F4xx_HAL_DEF
#define __F4xx_HAL_DEF

#ifdef __cplusplus
 extern "C" {
#endif

#include "f4xx.h"
#include <stddef.h>
 
#define __HAL_LINKDMA(__HANDLE__, __PPP_DMA_FIELD__, __DMA_HANDLE__) \
						do{                                                      \
                (__HANDLE__)->__PPP_DMA_FIELD__ = &(__DMA_HANDLE__); \
                (__DMA_HANDLE__).Parent = (__HANDLE__);              \
            } while(0U) 
 
typedef enum 
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

typedef enum 
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U  
} HAL_LockTypeDef;

#ifdef __cplusplus
}
#endif

#endif 

#endif


