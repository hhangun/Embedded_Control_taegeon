/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Han Taegeon
Modified         : 2023-10-14
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_EXTI
/----------------------------------------------------------------*/

#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecEXTI.h"


void EXTI_init(GPIO_TypeDef *Port, int Pin, int trig_type,int priority){

	// SYSCFG peripheral clock enable	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;		
	
	// Connect External Line to the GPIO
	int EXTICR_port;
	if			(Port == GPIOA) EXTICR_port = 0;
	else if	(Port == GPIOB) EXTICR_port = 1;
	else if	(Port == GPIOC) EXTICR_port = 2;
	else if	(Port == GPIOD) EXTICR_port = 3;
	else 										EXTICR_port = 4;
	
	SYSCFG->EXTICR[Pin >> 2] &= ~(15 << 4*(Pin & 0x03));			// clear 4 bits
	SYSCFG->EXTICR[Pin >> 2] |= EXTICR_port << 4*(Pin & 0x03);			// connect port number
	
	// Configure Trigger edge
	if (trig_type == FALL) EXTI->FTSR |= 1UL << Pin;   // Falling trigger enable 
	else if	(trig_type == RISE) EXTI->RTSR |= 1UL << Pin;   // Rising trigger enable 
	else if	(trig_type == BOTH) {			// Both falling/rising trigger enable
		EXTI->RTSR |= 1UL << Pin; 
		EXTI->FTSR |= 1UL << Pin;
	} 
	
	// Configure Interrupt Mask (Interrupt enabled)
	EXTI->IMR  |= 1 << Pin;     // not masked
	
	
	// NVIC(IRQ) Setting
	int EXTI_IRQn = 0;
	
	if (Pin == 0) EXTI_IRQn = EXTI0_IRQn;
	else if (Pin == 1) EXTI_IRQn = EXTI1_IRQn;
	else if (Pin == 2) EXTI_IRQn = EXTI2_IRQn;
	else if (Pin == 3) EXTI_IRQn = EXTI3_IRQn;
	else if (Pin == 4) EXTI_IRQn = EXTI4_IRQn;
	else if	(Pin > 4 && Pin < 10) 	EXTI_IRQn = EXTI9_5_IRQn;
	else 			EXTI_IRQn = EXTI15_10_IRQn;
								
	NVIC_SetPriority(EXTI_IRQn, priority);	// EXTI priority
	NVIC_EnableIRQ(EXTI_IRQn); 	// EXTI IRQ enable
}


void EXTI_enable(uint32_t pin) {
	EXTI->IMR |= 1UL << pin;     // not masked (i.e., Interrupt enabled)
}
void EXTI_disable(uint32_t pin) {
	EXTI->IMR |= ~(1UL << pin);     // masked (i.e., Interrupt disabled)
}

uint32_t is_pending_EXTI(uint32_t pin){
	uint32_t EXTI_PRx = pin;     	// check  EXTI pending 	
	return ((EXTI->PR & 1 << pin) == 1 << pin);
}


void clear_pending_EXTI(uint32_t pin){
	EXTI->PR |= 1 << pin;     // clear EXTI pending 
}
