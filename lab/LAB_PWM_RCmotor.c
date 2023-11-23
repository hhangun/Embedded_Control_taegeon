/**
******************************************************************************
* @author  HanTaegeon
* @Mod		 2023-10-25 by YKKIM  	
* @brief   Embedded Controller:  LAB Timer & PWM - Servo motor
* 
******************************************************************************
*/

// *************	PWM output  **************
#include "stm32f411xe.h"
#include "math.h"

// #include "ecSTM32F411.h"
//#include "ecPinNames.h"
//#include "ecGPIO.h"
//#include "ecSysTick.h"
//#include "ecRCC.h"
//#include "ecTIM.h"
//#include "ecPWM.h"   // ecPWM2.h
#include "gather.h"


// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN 13
#define PWM_PIN PA_0

void setup(void);
void EXTI15_10_IRQHandler(void);

static volatile int dir = 1;				// direction of motor
static volatile float duty = 0.5;				// PWM duty
static volatile uint32_t count = 0;			// Count for 0.5s
static volatile uint32_t State = 0;			// Button_State
static volatile uint32_t period = 1;		// PWM Period

int main(void) { 
	// Initialiization --------------------------------------------------------
  setup();

	// Inifinite Loop ----------------------------------------------------------
	while(1){
		PWM_pulsewidth(PWM_PIN,(float)duty/period);
	}
}

void TIM3_IRQHandler(void) {
	if (is_UIF(TIM3)) {			// Check UIF(update interrupt flag)
		count++;
		if (count > 500) {
			if (State == 0) {
				if (duty >= 2.5 || duty < 0.5) { dir *= -1; }
				duty += (float)(dir * 0.1111);
			}
			else if (State == 1) { duty = 0.5; }
			count = 0;
			State = 0;
		}
		clear_UIF(TIM3); 		// Clear UI flag by writing 0
	}
}


//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		State = 1;
		clear_pending_EXTI(BUTTON_PIN); 
	}
}


// Initialiization 
void setup(void) {	
	RCC_PLL_init();
	SysTick_init();
		
	// PWM of 1 msec:  TIM2_CH2 (PA_1 AFmode)	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	
	PWM_init(PWM_PIN);	
	PWM_period(PWM_PIN, 20);   // 20 msec PWM period
	
	TIM_UI_init(TIM3, 1);
}