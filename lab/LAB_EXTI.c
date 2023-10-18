/**
******************************************************************************
* @author  HanTaegeon
* @Mod		 2023-10-14
* @brief   Embedded Controller:  LAB Systick&EXTI
*					 - 7 segment
******************************************************************************
*/

#include "stm32F411xe.h"

#include "ecEXTI.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"

#define LED_PIN	5
#define BUTTON_PIN 13

// Initialiization 
void setup(void);
void EXTI15_10_IRQHandler(void);

int count = 0;
int time = 0;

int main(void) {
	setup();

	
	while (1) {}
}

//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {
	time++;
	if (is_pending_EXTI(BUTTON_PIN) && time<1000000) {
		LED_toggle();
		sevensegment_display(count % 10);
		//clear_pending_EXTI(BUTTON_PIN); 
		count++;
		if(count > 9) count = 0;
	}
}

void setup(void)
{
	RCC_PLL_init();
	SysTick_init();
	sevensegment_init();
	GPIO_init(GPIOA, LED_PIN, OUTPUT);
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
	
	// Priority Highest(0) External Interrupt 
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
}
