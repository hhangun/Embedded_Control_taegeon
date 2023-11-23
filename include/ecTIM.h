/**
******************************************************************************
* @author  HanTaeGeon
* @Mod		 2023-11-06	
* @brief   Embedded Controller:  EC_HAL_for_student_exercise 
* 
******************************************************************************
*/

#ifndef __EC_TIM_H 
#define __EC_TIM_H
#include "stm32f411xe.h"
#include "ecSTM32F411.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


// ICn selection according to CHn
#define FIRST 1
#define SECOND 2

// Edge Type
#define IC_RISE 0
#define IC_FALL 1
#define IC_BOTH 2

// IC Number
#define IC_1 1
#define IC_2 2
#define IC_3 3
#define IC_4 4


/* Timer Configuration */
void TIM_init(TIM_TypeDef *TIMx, uint32_t msec);  
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec);  
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_period(TIM_TypeDef* TIMx, uint32_t msec);

void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec); 
void TIM_UI_enable(TIM_TypeDef* TIMx);
void TIM_UI_disable(TIM_TypeDef* TIMx);

uint32_t is_UIF(TIM_TypeDef *TIMx);
void clear_UIF(TIM_TypeDef *TIMx);

/* Input Capture*/
void ICAP_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);
void ICAP_init(PinName_t pinName);
void ICAP_setup(PinName_t pinName, int ICn, int edge_type);
void ICAP_counter_us(PinName_t pinName, int usec);
uint32_t ICAP_capture(TIM_TypeDef* TIMx, uint32_t ICn);

uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t CCnum);  // CCnum= 1~4
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t CCnum);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif