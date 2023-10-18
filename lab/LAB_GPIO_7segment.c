/**
******************************************************************************
* @author  Han Taegeon
* @Mod		 2023-10-03	
* @brief   Embedded Controller:  Tutorial Digital In/Out 7-segment Display
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define BUTTON_PIN 13
#define B_DOWN 0
#define B_UP 1

void setup(void);
	
int main(void) {	
	// Initialiization --------------------------------------------------------
	setup();
	unsigned int cnt = 0;
	int button_state = 0;			// button down(0), button up(1)
	int button_press = 0;			// button pressed once(1), no press or press more after one pressing(0)
	
	// Inifinite Loop ----------------------------------------------------------
		while(1){
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
			button_state = B_DOWN;
			button_press = 1;
			
		}
		else
			button_state = B_UP;
		
		if(button_state == B_UP && button_press == 1){		
				if (cnt > 9) 
					cnt = 0;
					
				sevensegment_display(cnt % 10);
				//sevensegment_decoder(cnt % 10);
				
				for(int i = 0; i < 500000;i++){}			// before it gets tired
					
				button_press = 0;
				cnt++;
		}
	}		
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);  // calls pull-up
	// push-pull
	GPIO_otype(GPIOA, LED_PA5, 0);
	GPIO_otype(GPIOA, LED_PA6, 0);
	GPIO_otype(GPIOA, LED_PA7, 0);
	GPIO_otype(GPIOB, LED_PB6, 0);
	GPIO_otype(GPIOC, LED_PC7, 0);
	GPIO_otype(GPIOA, LED_PA9, 0);
	GPIO_otype(GPIOA, LED_PA8, 0);
	GPIO_otype(GPIOB, LED_PB10, 0);
	// No pull-up-pull-down
	GPIO_pupd(GPIOA, LED_PA5, EC_NONE);
	GPIO_pupd(GPIOA, LED_PA6, EC_NONE);
	GPIO_pupd(GPIOA, LED_PA7, EC_NONE);
	GPIO_pupd(GPIOB, LED_PB6, EC_NONE);
	GPIO_pupd(GPIOC, LED_PC7, EC_NONE);
	GPIO_pupd(GPIOA, LED_PA9, EC_NONE);
	GPIO_pupd(GPIOA, LED_PA8, EC_NONE);
	GPIO_pupd(GPIOB, LED_PB10, EC_NONE);
	// Medium Speed
	GPIO_ospeed(GPIOA, LED_PA5, EC_MEDIUM);
	GPIO_ospeed(GPIOA, LED_PA6, EC_MEDIUM);
	GPIO_ospeed(GPIOA, LED_PA7, EC_MEDIUM);
	GPIO_ospeed(GPIOB, LED_PB6, EC_MEDIUM);
	GPIO_ospeed(GPIOC, LED_PC7, EC_MEDIUM);
	GPIO_ospeed(GPIOA, LED_PA9, EC_MEDIUM);
	GPIO_ospeed(GPIOA, LED_PA8, EC_MEDIUM);
	GPIO_ospeed(GPIOB, LED_PB10, EC_MEDIUM);
	// Initialiization of output
	//sevensegment_init();
	sevensegment_display_init(); 
	//sevensegment_decoder(0);
}
