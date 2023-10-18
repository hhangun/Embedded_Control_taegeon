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

#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	unsigned int ledpin = 0;
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
					// Status according to ledpin
					switch(ledpin){
					case 0: 	// First LED ON, Others OFF
						GPIO_write(GPIOA, 5, HIGH);
						GPIO_write(GPIOA, 6, LOW);
						GPIO_write(GPIOA, 7, LOW);
						GPIO_write(GPIOB, 6, LOW);
						ledpin++;
						break;
					case 1: 	// Second LED ON, Others OFF
						GPIO_write(GPIOA, 5, LOW);
						GPIO_write(GPIOA, 6, HIGH);
						GPIO_write(GPIOA, 7, LOW);
						GPIO_write(GPIOB, 6, LOW);
						ledpin++;
						break;
					case 2: 	// Third LED ON, Others OFF
						GPIO_write(GPIOA, 5, LOW);
						GPIO_write(GPIOA, 6, LOW);
						GPIO_write(GPIOA, 7, HIGH);
						GPIO_write(GPIOB, 6, LOW);
						ledpin++;
						break;
					default: 	// Fourth LED ON, Others OFF
						GPIO_write(GPIOA, 5, LOW);
						GPIO_write(GPIOA, 6, LOW);
						GPIO_write(GPIOA, 7, LOW);
						GPIO_write(GPIOB, 6, HIGH);
						ledpin = 0;
				}
					
					button_press = 0;
				}
				else{
					button_state = 1;
				}
	}
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT); 			// calls RCC_GPIOC_enable()
	GPIO_init(GPIOA, LED_PIN, OUTPUT);   			// calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PIN, EC_PU);					// calls pull-up
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);			// calls pull-up
	GPIO_otype(GPIOA, LED_PIN, 0);						// calls Output push-pull
	GPIO_ospeed(GPIOA, LED_PIN, EC_MEDIUM);		// calls Medium speed
	
	GPIO_init(GPIOA, 5, OUTPUT);							// PA5_enable
	GPIO_init(GPIOA, 6, OUTPUT);							// PA6_enable
	GPIO_init(GPIOA, 7, OUTPUT);							// PA7_enable
	GPIO_init(GPIOB, 6, OUTPUT);							// PB6_enable

}
