/**
******************************************************************************
* @author  HanTaegeon
* @Mod		 2023-10-14 by YKKIM  	
* @brief   Embedded Controller:  LAB Systick&EXTI with API
*					 - 7 segment
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"
#include "ecEXTI.h"

int count = 0;
// Initialiization 
void setup(void);
void EXTI15_10_IRQHandler(void);

int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();//EXTI for Pin 13

	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		sevensegment_display(count % 10);
		delay_ms(1000);
		count++;
		if (count >9) count =0;
		SysTick_reset();
	}
}

//EXTI for Pin 13

void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		count = 0;
		sevensegment_display(count % 10);
		clear_pending_EXTI(BUTTON_PIN); 
	}
}


void setup(void)
{
	RCC_PLL_init();
	SysTick_init();
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	sevensegment_init();
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PD);
	// push-pull
	GPIO_otype(GPIOA, LED_PA7, 0);
	GPIO_otype(GPIOB, LED_PB6, 0);
	GPIO_otype(GPIOC, LED_PC7, 0);
	GPIO_otype(GPIOA, LED_PA9, 0);
	// No pull-up-pull-down
	GPIO_pupd(GPIOA, LED_PA7, EC_NONE);
	GPIO_pupd(GPIOB, LED_PB6, EC_NONE);
	GPIO_pupd(GPIOC, LED_PC7, EC_NONE);
	GPIO_pupd(GPIOA, LED_PA9, EC_NONE);
	// Medium Speed
	GPIO_ospeed(GPIOA, LED_PA7, EC_MEDIUM);
	GPIO_ospeed(GPIOB, LED_PB6, EC_MEDIUM);
	GPIO_ospeed(GPIOC, LED_PC7, EC_MEDIUM);
	GPIO_ospeed(GPIOA, LED_PA9, EC_MEDIUM);
}