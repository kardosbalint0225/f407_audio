#ifndef __DMA_EX_H
#define __DMA_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "def.h"

typedef enum
{
  MEMORY0      = 0x00U,    /*!< Memory 0     */
  MEMORY1      = 0x01U     /*!< Memory 1     */
} HAL_DMA_MemoryTypeDef;

HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_ChangeMemory(DMA_HandleTypeDef *hdma, uint32_t Address, HAL_DMA_MemoryTypeDef memory);

#ifdef __cplusplus
}
#endif

#endif 

