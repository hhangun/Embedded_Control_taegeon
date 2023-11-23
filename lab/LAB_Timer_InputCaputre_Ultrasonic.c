/**
******************************************************************************
* @author  Han TaeGeon
* @Mod		 2023-11-06	
* @brief   Embedded Controller:  LAB - Timer Input Capture 
*					 						- with Ultrasonic Distance Sensor
* 
******************************************************************************
*/

//#include "stm32f411xe.h"
//#include "math.h"
//#include "ecGPIO.h"
//#include "ecRCC.h"
//#include "ecTIM.h"
//#include "ecPWM.h"
//#include "ecUART_simple_student.h"
//#include "ecSysTIck.h"
#include "ecSTM32F411.h".h"


uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;

#define TRIG PA_6
#define ECHO PB_6

void setup(void);

int main(void){
	
	setup();
	
	while(1){
		distance = (float) timeInterval * 340.0 / 2.0 / 10.0; 	// [mm] -> [cm]
		printf("%f cm\r\n", distance);
		delay_ms(500);
	}
}


void TIM4_IRQHandler(void){
	if(is_UIF(TIM4)){                     // Update interrupt
		uint32_t ovf_cnt = 0;									// overflow count
		clear_UIF(TIM4);  							    // clear update interrupt flag
	}
	
	if(is_CCIF(TIM4, 1)){ 								// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1 = TIM4->CCR1;									// Capture TimeStart
		clear_CCIF(TIM4, 1);                // clear capture/compare interrupt flag 
	}			
	
	else if(is_CCIF(TIM4, 2)){ 									// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		time2 = TIM4->CCR2;									// Capture TimeEnd
		timeInterval = (time2-time1+(TIM4->ARR+1)*ovf_cnt)*1e-2; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		ovf_cnt = 0;                       // overflow reset
		clear_CCIF(TIM4,2);								  // clear capture/compare interrupt flag 
	}
}

void setup(){
	
	RCC_PLL_init(); 
	SysTick_init();
	UART2_init();
  
// PWM configuration ---------------------------------------------------------------------	
	PWM_init(TRIG);			// PA_6: Ultrasonic trig pulse
	PWM_period_us(TRIG, 50000);    // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG, 10);   // PWM pulse width of 10us
	
	
// Input Capture configuration -----------------------------------------------------------------------	
	ICAP_init(ECHO);    	// PB_6 as input caputre
 	ICAP_counter_us(ECHO, 10);   	// ICAP counter step time as 10us
	ICAP_setup(ECHO, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect

}
