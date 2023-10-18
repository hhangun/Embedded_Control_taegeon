/**
******************************************************************************
* @author	Han Taegeon
* @Mod		2023.09.24
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle LED LD2 by Button B1 pressing
* 
******************************************************************************
*/



#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	unsigned int ledpin = LOW;
	int button_press = 0; // 1: button pressed, 0: no press or press again after press once
	int button_state = 0; // 0: down, 1: up
	// Inifinite Loop ----------------------------------------------------------
	while(1){

			if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
					button_press = 1;
					button_state = 0;
			}
				else
					button_state = 1;
			
				if(button_press == 1 && button_state == 1){
					ledpin = !ledpin;
					button_press = 0;
				}
				else{
					button_state = 1;
				}
				
				GPIO_write(GPIOA, LED_PIN, ledpin);

	}
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT); 			// calls RCC_GPIOC_enable()
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    		// calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PIN, EC_PU);					// calls pull-up
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);			// calls pull-up
	GPIO_otype(GPIOA, LED_PIN, 1);						// calls Output Open-Drain
	GPIO_ospeed(GPIOA, LED_PIN, EC_MEDIUM);		// calls Medium speed

}
