/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Lim Soon Ho, Han Taegeon
Modified         : 2023-12-15
Language/ver     : C++ in Arduino
Description      : Smart home and parking lot
/----------------------------------------------------------------*/

#include "ecSTM32F411.h"
#define PWM_P  PA_1	// roof
#define PWM_P2 PA_0 // door

#define TRIG PC_7
#define ECHO PB_6

void setup(void);

int buz = 0;
int state = 0;
int move = 0;	// moving detection sensor
float value = 0;	// jodo sensor value
static volatile float count = 9;	// roof angle control
static volatile float count2 = 3;	// door angle control
int button_press = 0;
int button_state = 1;
int detect = 0;
int mode_change  = 3;

uint32_t ovf_cnt = 0;
float distance = 40;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;
int time = 0;


int main(void){
	setup();
	printf("MCU Initialized\r\n"); 
	printf("light = %f", value);
	
	while(1){
		
		distance = (float) timeInterval * 340.0 / 2.0 / 10.0;    // [mm] -> [cm]
		printf("distance: %f\r\n", distance);
		//printf("buz: %d\r\n",buz);
		printf("mode: %d\r\n", button_press);
		
		//////////////////// Normal Mode ////////////////////
		if(button_press == 0){
		// Buzz toggele
		if(distance < 15 && buz == 0){
			for(int i = 0; i < 5; i++){
			state = GPIO_read(GPIOA, 8);
			GPIO_write(GPIOA, 8, !state);
			delay_ms(500);
			}
			buz = 1;
		}
		else
			GPIO_write(GPIOA, 8, LOW);	// Buzz off
		
		// trash value
		if(distance > 90 && distance < 400)
			buz = 0;
		
		// Roof control by CDS sensor value
		if(value <= 1300)
				 count -= 1;
 	 else if(value > 1300)
				 count += 1;
	 
	 if(count >= 10) count = 9;
	 else if(count <= 0) count = 1;
	 
		PWM_duty(PWM_P, (0.5 + (1.f/9.f)*(float)count)/20.f);		// roof duty
		
	  // Moving detection sensor
	  move = GPIO_read(GPIOA, 6); 
		if(move){
			count2 = 9;
			GPIO_write(GPIOA, 9, HIGH);	// if moving detection sensor detected, LED ON
			PWM_duty(PWM_P2, (0.5 + (1.f/9.f)*(float)count2)/20.f);	// Open the DOOR
			delay_ms(3000);
		}
		
		GPIO_write(GPIOA, 9 ,LOW);	// LED OFF 
		count2 -= 0.5;
		PWM_duty(PWM_P2, (0.5 + (1.f/9.f)*(float)count2)/20.f);	// Closed the DOOR
		
		if(count2 < 3) count2 = 3;
		
		printf("light = %f\r\n", value);
		delay_ms(500);	
		}
		
				//////////////////// Security Mode ////////////////////
		if(button_press == 1){	// safety mode
				move = GPIO_read(GPIOA, 6); 
				if(move)
					detect = 1;
				
				if(detect == 1){	
				PWM_duty(PWM_P, (0.5 + (1.f/9.f)*(float)9)/20.f);		// Closed the roof
				PWM_duty(PWM_P2, (0.5 + (1.f/9.f)*(float)3)/20.f);	// Closed the DOOR
				state = GPIO_read(GPIOA, 8);
				for(int i = 0; i < 5; i++){
					GPIO_write(GPIOA, 8, !state);	// BUZZ toggle
				
				LED_toggle();	
				}
				delay_ms(500);
				}
		}
		
				//////////////////// Saving Mode ////////////////////
		if(button_press == 2){
			move = GPIO_read(GPIOA, 6);
			if(move){
			count2 = 9;
			GPIO_write(GPIOA, 9, LOW);	// if moving detection sensor detected, LED OFF
			PWM_duty(PWM_P2, (0.5 + (1.f/9.f)*(float)count2)/20.f);	// Open the DOOR
			delay_ms(3000);
			}
			count2 -= 0.5;
		PWM_duty(PWM_P2, (0.5 + (1.f/9.f)*(float)count2)/20.f);	// Closed the DOOR
		
		if(count2 < 3) count2 = 3;
		}
	}
}


void setup(){
	RCC_PLL_init();
	SysTick_init();
	UART2_init();
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	// input pin
	GPIO_init(GPIOA, 6, INPUT);
	GPIO_pupd(GPIOA, 6, EC_PD);
	GPIO_otype(GPIOA, 6, EC_PUSH_PULL);
	
	// led pin 
	GPIO_init(GPIOA, 9, OUTPUT);
	GPIO_pupd(GPIOA, 9, EC_PD);
	GPIO_otype(GPIOA, 9, EC_PUSH_PULL);
	
	// buzz
	GPIO_init(GPIOA, 8, OUTPUT);
	GPIO_pupd(GPIOA, 8, EC_PD);
	GPIO_otype(GPIOA, 8, EC_PUSH_PULL);
	
	// button
	//GPIO_init(GPIOA, 3, INPUT);
	//GPIO_pupd(GPIOA, 3, EC_PD);
	//GPIO_otype(GPIOA, 3, EC_PUSH_PULL);
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PD);
	GPIO_otype(GPIOC, BUTTON_PIN, EC_PUSH_PULL);
	
	
	 // ADC Init
   ADC_init(PB_0);   // priority 1
	
	 // servo motor1
	 PWM_init(PWM_P);   
   PWM_period_ms(PWM_P, 20);   // 20 msec PWM period
	
	 // servo motor2
	 PWM_init(PWM_P2);   
   PWM_period_ms(PWM_P, 20);   // 20 msec PWM period
	 
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

// Jodo Sensor
void ADC_IRQHandler(void){
   if(is_ADC_OVR())
      clear_ADC_OVR();
   
   if(is_ADC_EOC()){      // after finishing sequence
       value = ADC_read(); 
		}
	 
}

// Ultra Sensor
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

void EXTI15_10_IRQHandler(void) 
{
      time++;
   
      if (is_pending_EXTI(BUTTON_PIN) && time > 100000) {
				button_press ++;
      if (button_press > 3) button_press = 0;
      clear_pending_EXTI(BUTTON_PIN);
      }
}
	
	