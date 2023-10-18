/**
******************************************************************************
* @author  Han Taegeon
* @Mod		 2023-10-10
* @brief   Embedded Controller:  Tutorial EXTI
*
******************************************************************************
*/


#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);

int main(void) {

	// System CLOCK, GPIO Initialiization ----------------------------------------
	setup();


	// EXTI Initialiization ------------------------------------------------------	

	// SYSCFG peripheral clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Connect External Line to the GPIO
	// Button: PC_13 -> EXTICR3(EXTI13)
	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

	// Falling trigger enable (Button: pull-up)
	EXTI->FTSR |= 1UL << BUTTON_PIN;

	// Unmask (Enable) EXT interrupt
	EXTI->IMR |= 1UL << BUTTON_PIN;

	// Interrupt IRQn, Priority
	NVIC_SetPriority(EXTI15_10_IRQn, 0);  		// Set EXTI priority as 0	
	NVIC_EnableIRQ(EXTI15_10_IRQn); 			// Enable EXTI 

	while (1);
}


void EXTI15_10_IRQHandler(void) {
	if ((EXTI->PR & EXTI_PR_PR13) == EXTI_PR_PR13) {
		LED_toggle();
	EXTI->PR |= EXTI_PR_PR13; // cleared by writing '1'
	}
}


// Initialiization 
void setup(void)
{
	RCC_PLL_init();                         // System Clock = 84MHz
	// Initialize GPIOA_5 for Output
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()	
	// Initialize GPIOC_13 for Input Button
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
}
