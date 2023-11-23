/*
******************************************************************************
* @author   Han TaeGeon
* @Mod      2023. 11. 06
* @brief   Embedded Controller:  Stepper moter
******************************************************************************
*/

#include "ecSTM32F411.h"

void setup(void);
void EXTI15_10_IRQHandler(void);

int main(void){
	 // Initialization 
	setup();
	
	Stepper_step(2048, 0, FULL);		//Step 30x64 Direction 0 or 1 Mode in FULL mode
	//Stepper_step(2048, 1, FULL);
	//Stepper_step(4096, 0, HALF);		//Step 30x128 Direction 0 or 1 Mode in HALF mode
	//Stepper_step(4096, 1, HALF);
	
	// Infinite Loop
	while(1){;}
		
}

void setup(void){
	
	RCC_PLL_init();		// System Clock = 84MHz
	SysTick_init();		// SysTick init
	
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);	// External Interrupt Setting
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);		//  GPIOC BUTTON PIN initialization

	
	Stepper_init(GPIOB, 10, GPIOB, 4, GPIOB, 5, GPIOB, 3);		// Stepper GPIO pin initialization
	Stepper_setSpeed(14, FULL);			// set stepper motor speed in FULL mode
	//Stepper_setSpeed(14, HALF);			// set stepper motor speed in HALF mode
}

void EXTI15_10_IRQHandler(void){
	if (is_pending_EXTI(BUTTON_PIN)) {		 // pendig 1
		Stepper_stop();
		clear_pending_EXTI(BUTTON_PIN);			// cleared by writing '1'
	}
}