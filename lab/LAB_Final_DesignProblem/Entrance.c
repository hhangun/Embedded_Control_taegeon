/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Lim Soon Ho, Han Taegeon
Modified         : 2023-12-15
Language/ver     : C++ in Keil uVision

Description      : Make automatic entrance system
/----------------------------------------------------------------*/

# include "ecSTM32F411.h"

#define PWM_PIN PA_1

#define TRIG PC_7
#define ECHO PB_6

uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;

void setup(void);

static volatile float count = 0;

int main(void){

   setup();
   printf("start\r\n");
   delay_ms(8000);
   
   while(1){
      PWM_duty(PWM_PIN, (0.5 + (1.f/9.f)*(float)count)/20.f);	// Control Servo motor
      distance = (float) timeInterval * 340.0 / 2.0 / 10.0;    // [mm] -> [cm]
      
      if(distance > 2 && distance < 400){   
         printf("distance = %f\r\n", distance);
         delay_ms(1000);
         
				// Control LED and Servo motor by distance
         if(distance < 15){
            count = 10;
					  GPIO_write(GPIOA, 5, LOW);
						GPIO_write(GPIOA, 6, HIGH);
					 PWM_duty(PWM_PIN, (0.5 + (1.f/9.f)*(float)count)/20.f);
					 delay_ms(1000);
				 }
         
         else if (distance >= 15){
            count = 0;
					  GPIO_write(GPIOA, 5, HIGH);
						GPIO_write(GPIOA, 6, LOW);
				 }
      } 
   }   
}

void TIM4_IRQHandler(void){
   if(is_UIF(TIM4)){                        // Update interrupt
      ovf_cnt++;                          // overflow count
      clear_UIF(TIM4);                              // clear update interrupt flag
 } 
   if(is_CCIF(TIM4, 1)){                         // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
      time1 = TIM4->CCR1;                           // Capture TimeStart
      clear_CCIF(TIM4, 1);                // clear capture/compare interrupt flag 
   }                                              
   else if(is_CCIF(TIM4, 2)){                   // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
      time2 = TIM4->CCR2;                           // Capture TimeEnd
      timeInterval = ((time2-time1)+(TIM4->ARR+1)*ovf_cnt)/100;    // (10us * counter pulse -> [msec] unit) Total time of echo pulse
      ovf_cnt = 0;                        // overflow reset
      clear_CCIF(TIM4,2);                          // clear capture/compare interrupt flag 
   }
}

void setup(void){
   
   RCC_PLL_init();
   SysTick_init();
   UART2_init();
   
   PWM_init(PWM_PIN);   
   PWM_period_ms(PWM_PIN, 20);   // 20 msec PWM period
   
   EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
   GPIO_init(GPIOC, BUTTON_PIN, INPUT);
   GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	
	 GPIO_init(GPIOA, 5, OUTPUT);							// LED RED
	 GPIO_pupd(GPIOA, 5, EC_PD);
   GPIO_otype(GPIOA, 5, EC_PUSH_PULL);
	 GPIO_init(GPIOA, 6, OUTPUT);							// LED BLUE
   GPIO_pupd(GPIOA, 6, EC_PD);
   GPIO_otype(GPIOA, 6, EC_PUSH_PULL);
	
   SysTick_init();
   UART2_init();
  
	 // PWM configuration ---------------------------------------------------------------------   
   PWM_init(TRIG);         // PA_7: Ultrasonic trig pulse
   PWM_period_us(TRIG, 50000);     // PWM of 50ms period. Use period_us()
   PWM_pulsewidth_us(TRIG, 10);   // PWM pulse width of 10us
   
   
	 // Input Capture configuration -----------------------------------------------------------------------   
   ICAP_init(ECHO);       // PB_6 as input caputre CH1
   ICAP_counter_us(ECHO, 10);      // ICAP counter step time as 10us
   ICAP_setup(ECHO, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
   ICAP_setup(ECHO, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect
}