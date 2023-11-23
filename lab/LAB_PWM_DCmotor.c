/**
******************************************************************************
* @author  HanTaegeon
* @Mod		 2023-10-25 by YKKIM  	
* @brief   Embedded Controller:  LAB Timer & PWM - DC motor
* 
******************************************************************************
*/

// *************	PWM output  **************


// #include "ecSTM32F411.h"
//#include "ecPinNames.h"
//#include "ecGPIO.h"
//#include "ecSysTick.h"
//#include "ecRCC.h"
//#include "ecTIM.h"
//#include "ecPWM.h"   // ecPWM2.h
#include "stm32f411xe.h"
#include "math.h"
#include "gather.h"


// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN 13
#define Direction_PIN 2
#define PWM_PIN PA_0

void setup(void);
void EXTI15_10_IRQHandler(void);

static volatile int flag = 1;						// variables to change duty
static volatile int dir = -1;						// stop or continue
static volatile float duty = 0.25;			// PWM duty
static volatile uint32_t count = 0;			// Count for 0.5s
static volatile int State = 1;					// Button_State

int main(void) { 
	// Initialiization --------------------------------------------------------
  setup();

	// Inifinite Loop ----------------------------------------------------------
	while(1){
		if(State == 1){
		PWM_duty(PWM_PIN,(float)duty);
		}
		else if(State == -1){
			PWM_duty(PWM_PIN,(float)0);
		}
	}
}

	void TIM3_IRQHandler(void){
		if(is_UIF(TIM3)){			// Check UIF(update interrupt flag)
			count++;
			if (count > 2000){
				if(State == 1){
					flag *= -1;
					if(flag == 1)	{duty = 0.25;}
					else if(flag == -1)	{duty = 0.75;}
				count = 0;
				}
			}
			clear_UIF(TIM3); 		// Clear UI flag by writing 0
		}
	}


//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		State *= -1;
		dir *= -1;
		for(int i = 0; i < 50000; i++){}
		clear_pending_EXTI(BUTTON_PIN); 
	}
}


// Initialiization 
void setup(void) {	
	RCC_PLL_init();
	SysTick_init();
		
	// PWM of 1 msec:  TIM2_CH1 (PA_0 AFmode)
	GPIO_init(GPIOC, Direction_PIN, OUTPUT);
	GPIO_pupd(GPIOC, Direction_PIN, EC_PU);
	
	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	
	PWM_init(PWM_PIN);	
	PWM_period(PWM_PIN, 1);   // 1 msec PWM period
	
	TIM_UI_init(TIM3, 1);
}