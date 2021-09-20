#ifdef TEST

#ifndef __F4xx_H
#define __F4xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

#include "f407xx.h"

typedef enum 
{
  RESET = 0U, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0U, 
  ENABLE = !DISABLE
} FunctionalState;

typedef enum
{
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __F4xx_H */

#endif

