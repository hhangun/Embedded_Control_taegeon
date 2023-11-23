/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Han Taegeon
Modified         : 2023-11-23
Language/ver     : C++ in Keil uVision

Description      : USART_Bluetooth
/----------------------------------------------------------------*/


#include "ecSTM32F411.h"

#define MAX_BUF    10
#define END_CHAR    13
#define MOTOR_A   2
#define MOTOR_B   3
#define TRIG PA_6
#define ECHO PB_6


// static volatile uint8_t buffer[MAX_BUF]={0, };
// static volatile uint8_t PC_string[MAX_BUF]={0, };
static volatile uint8_t PC_data = 0;
static volatile uint8_t BT_data = 0;
static char mode = ' ';
static char Direction = ' ';	// direction control
static uint32_t count = 0;   // led count
static uint32_t cnt = 3;	// angle control
static int VER = 1;		// speed control
static int auto_state = 0;


static double dutyA = 0;      // PWM of Motor A 
static double dutyB = 0;   // PWM of Motor B 
static unsigned int dir = 1;

static volatile int bReceive = 0; // flag
static volatile int state = 3;	// Mode discrimination

// ADC
static PinName_t seqCHn[2] = {PB_0, PB_1};
static uint32_t value1, value2;
static int flag = 0;

// UltraSonic
static uint32_t ovf_cnt = 0;
static double distance = 0;
static float timeInterval = 0;
static float time1 = 0;
static float time2 = 0;

void setup(void);
void manual_mode();
void angle_go();
void angle_back();

// Initiallization
void setup(void){
   RCC_PLL_init();
   SysTick_init();
   
   // PWM init
   PWM_init(PA_0);
   PWM_init(PA_1);

   PWM_period_us(PA_0, 200);   // 1 msec PWM period
   PWM_period_us(PA_1, 200);   // 1 msec PWM period
   
    // Ultrasonic
    PWM_init(TRIG);         // PA_6: Ultrasonic trig pulse
    PWM_period_us(TRIG, 50000);    // PWM of 50ms period. Use period_us()
    PWM_pulsewidth_us(TRIG, 10);   // PWM pulse width of 10us
   
    ICAP_init(ECHO);       // PB_6 as input caputre
    ICAP_counter_us(ECHO, 10);      // ICAP counter step time as 10us
    ICAP_setup(ECHO, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
    ICAP_setup(ECHO, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect
   
   // GPIO 
   GPIO_init(GPIOA, LED_PIN, OUTPUT);   // LED PIN
   GPIO_init(GPIOC, MOTOR_A, OUTPUT);   // motorA direction
   GPIO_init(GPIOC, MOTOR_B, OUTPUT);   // motorB direction
   mcu_init(GPIOA, LED_PIN);
   mcu_init(GPIOC, MOTOR_A);
   mcu_init(GPIOC, MOTOR_B);
   
   // USART2: USB serial init
   UART2_init();
   UART2_baud(BAUD_9600);

   // USART1: BT serial init 
   UART1_init();
   UART1_baud(BAUD_9600);
   USART_setting(USART1,GPIOA, 9, GPIOA, 10, 9600);
	 
	 // TIM3
	 TIM_UI_init(TIM3, 1);         // TIM3 Update-Event Interrupt every 1 msec 
   TIM_UI_enable(TIM3);

   
   // ADC Init
   ADC_init(PB_0);   // priority 1
   ADC_init(PB_1);

   // ADC channel sequence setting
   ADC_sequence(seqCHn, 2);
}

int main(void){   
   setup();
   printf("MCU Initialized\r\n");   
	 
   while(1){		 
		 distance = (float) timeInterval * 340.0 / 2.0 / 10.0;    // [mm] -> [cm]
		 if(distance < 0) distance = distance*(-1);
      if (bReceive == 1){         // flag
         bReceive = 0;
            
         if(state == 0){
            manual_mode();
         }
             else if(state == 1){
							 // Erase distance error
							if (distance >= 400) continue;
             }
						  PWM_duty(PA_0, dutyA);
						  PWM_duty(PA_1, dutyB);
      }
			if(state == 0){
			// Display values in Tera Term
						USART_write(USART1,(uint8_t*)"MOD : ", 6);
						USART_write(USART1, &mode, 1);
						USART_write(USART1, (uint8_t*)" DIR : ", 7);
						USART_write(USART1, &Direction, 1);
						USART_write(USART1, (uint8_t*)" STR : ", 7);
						char ssttrr[20];
						int len = sprintf(ssttrr, "%d", cnt);
						USART_write(USART1, (uint8_t*)ssttrr, len);
						
						USART_write(USART1, &cnt, 1);
						USART_write(USART1, (uint8_t*)" VER : ", 7);
						char vveerr[20];
						int len2 = sprintf(vveerr, "%d", VER);
						USART_write(USART1, (uint8_t*)vveerr, len2);
			
						USART_write(USART1, (uint8_t*)"\r\n", 2);
			}
			else if(state == 1){
						// Display values in Tera Term
						USART_write(USART1,(uint8_t*)"distance : ", 11);
						char dist[20];
						int d = sprintf(dist, "%f", distance);
						USART_write(USART1, (uint8_t*)dist, d);
				
						USART_write(USART1,(uint8_t*)" state : ", 9 );
						char auto_st[30];
						if(auto_state == 1 || auto_state == 4){
							sprintf(auto_st, " state : GO straight");
							USART_write(USART1, &auto_st, 20); }
						else if(auto_state == 2){
							sprintf(auto_st, " state : Right");
							USART_write(USART1, &auto_st, 14); }
						else if(auto_state == 3){
							sprintf(auto_st, " state : Left");
							USART_write(USART1, &auto_st, 13); }
						else if(auto_state == 0){
							sprintf(auto_st, " state : ");
							USART_write(USART1, &auto_st, 9); }
						else if(auto_state == 5){
							sprintf(auto_st, " state : Stop");
							USART_write(USART1, &auto_st, 13); }
						USART_write(USART1, (uint8_t*)"\r\n", 2);
					}
						delay_ms(1000);
		}
}

void TIM3_IRQHandler(void){
   
   if(is_UIF(TIM3)){         // Check UIF(update interrupt flag)
		 // If modeA, LED blink at 1 sec
      if(BT_data == 'a' || BT_data == 'A'){
         count++;
         if (count <= 1000) GPIO_write(GPIOA, LED_PIN, 0);
            else if(count <= 2000)
               GPIO_write(GPIOA, LED_PIN, 1);
               else
                  count = 0;
			}
			else if(BT_data == 'm' || BT_data == 'M')
				GPIO_write(GPIOA, LED_PIN, 1);
      clear_UIF(TIM3);       // Clear UI flag by writing 0
   }
}

void TIM4_IRQHandler(void){
   if(is_UIF(TIM4)){                     // Update interrupt
      uint32_t ovf_cnt = 0;                           // overflow count
      clear_UIF(TIM4);                           // clear update interrupt flag
   }
   
   if(is_CCIF(TIM4, 1)){                         // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
      time1 = TIM4->CCR1;                           // Capture TimeStart
      clear_CCIF(TIM4, 1);                // clear capture/compare interrupt flag 
   }         
   
   else if(is_CCIF(TIM4, 2)){                            // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
      time2 = TIM4->CCR2;                           // Capture TimeEnd
      timeInterval = (time2-time1+(TIM4->ARR+1)*ovf_cnt)*1e-2;    // (10us * counter pulse -> [msec] unit) Total time of echo pulse
      ovf_cnt = 0;                       // overflow reset
      clear_CCIF(TIM4,2);                          // clear capture/compare interrupt flag 
   }  
}

void USART2_IRQHandler(){                // USART2 RX Interrupt : Recommended
   if(is_USART2_RXNE()){
      PC_data = USART2_read();      // RX from UART2 (PC)
      
      USART2_write(&PC_data,1);      // TX to USART2    (PC)    Echo of keyboard typing
   }
}

void USART1_IRQHandler(){                // USART2 RX Interrupt : Recommended
   if(is_USART1_RXNE()){
      USART_write(USART1, (uint8_t*)"Data = ", 7); 
		  BT_data = USART1_read();               // RX from USART1 
      USART_write(USART1, &BT_data, 1);   // TX to USART1
		  
			USART_write(USART1, (uint8_t*)"\r\n", 2);
		 
      if(BT_data == END_CHAR)      // Press enter key
         USART_write(USART1, "\r\n", 2);
      
      else if(BT_data == 'm' || BT_data == 'M'){
            mode = 'M';
            dutyA = 1;   //stop
            dutyB = 1;
        state = 0;
         }
      else if(BT_data == 'a' || BT_data == 'A'){
            mode = 'A';
            Direction = 'F';
            dutyA = 0;
            dutyB = 0;
        state = 1;
         }
      bReceive = 1;    // flag = 1	 
      }
}

// A or a
void ADC_IRQHandler(void){
   if(is_ADC_OVR())
      clear_ADC_OVR();
   
   if(is_ADC_EOC()){      // after finishing sequence
      if (flag==0)
         value1 = ADC_read();  
      else if (flag==1)
         value2 = ADC_read();
         
      flag =! flag;      // flag toggle
		}
         if(state == 1){
       if(value1 < 1000 && value2 < 1000){   // Go straight
									dutyA = 1;
									dutyB = 1;
									auto_state = 1;
								}
								else if(value1 < 1000 && value2 > 1000){   // Go right
									dutyA = 0.8;
									dutyB = 1;
									auto_state = 2;
								}
								else if(value1 > 1000 && value2 < 1000){   // Go left
									dutyA = 1;
									dutyB = 0.8;
									auto_state = 3;
								}
								else if(value1 > 1000 && value2 > 1000){
									dutyA = 1;
									dutyB = 1;
									auto_state = 4;
								}
								if(distance < 20){
									dutyA = 0;
									dutyB = 0;
									auto_state = 5;
								}	
							PWM_duty(PA_0, dutyA);
						  PWM_duty(PA_1, dutyB);								
	}
}

void manual_mode(){
		double sp = 0.1;
         if(BT_data == 'B') {
                Direction = 'B';	// for display in tera term
                dir = 1;
								VER = 1;
                dutyA = 0.5;
                dutyB = 0.5;
             }
         else if(BT_data == 'F') {
                Direction = 'F';	// for display in tera term
                dir = 0;
								VER = 1;
                dutyA = 0.5;
                dutyB = 0.5;
         }
               if(dir == 1){

               // Control a car speed
               switch(BT_data){
                  case 'W' :    // Go Right
										if(cnt>0){
										cnt--;   
										angle_go();
										}
										else {}		break;
                  case 'Q' :		// Go Left
										if(cnt<6){
										cnt++;
                    angle_go();
										}
										else {}		break;	
                  case 'S' :   // Stop
                     dutyA = 1;
                     dutyB = 1;   break;
                  
                  default    : break;
               }
                  // Control Speed
                  if(BT_data == 'O'){	// speed down
                     dutyA += sp;
                     dutyB += sp;
										 VER--;
                  }
                  else if(BT_data == 'P'){	// speed up
                     dutyA -= sp;
                     dutyB -= sp;
										VER++;
                  }
									if(VER < 0){	// speed limit
										VER = 0;
										dutyA -= sp;
										dutyB -= sp;
									}
									else if(VER > 3){	// speed limit
										VER = 3;
										dutyA += sp;
										dutyB += sp;
									}
               }
               
               else if(dir == 0){
                  switch(BT_data){
                  case 'Q' :    // Go left
                    if(cnt<6){
										cnt++;   
										angle_back();
										}
										else {}		break;
                  case 'W' :		// Go right
										if(cnt>0){
										cnt--;
                    angle_back();
										}
										else {}		break;
                  case 'S' :   // Stop
                     dutyA = 0;
                     dutyB = 0;   break;

                  default    : break;
               }
                  //Control speed
                  if(BT_data == 'P'){	// speed up
                     dutyA += sp;
                     dutyB += sp;
										 VER++;
                  }
                  else if(BT_data == 'O'){	// speed down
                     dutyA -= sp;
                     dutyB -= sp;
										VER--;
                  }
									if(VER < 0){	// speed limit
										VER = 0;
										dutyA += sp;
										dutyB += sp;
									}
									else if(VER > 3){	// speed limit
										VER = 3;
										dutyA -= sp;
										dutyB -= sp;
									}
            }
         GPIO_write(GPIOC, MOTOR_A, dir);   
         GPIO_write(GPIOC, MOTOR_B, dir);			
}

// Angle change with cnt value
void angle_go(){
	if(cnt == 3){
	dutyA = 0.5;
	dutyB = 0.5;
	}
	else if(cnt == 2){
		dutyA = 0.6;
		dutyB = 0.3;
	}
	else if(cnt == 1){
		dutyA = 0.6;
		dutyB = 0.2;
	}
	else if(cnt <= 0){
		dutyA = 0.6;
		dutyB = 0.1;
	}
	else if(cnt == 4){
		dutyA = 0.3;
		dutyB = 0.6;
	}
	else if(cnt == 5){
		dutyA = 0.2;
		dutyB = 0.6;
	}
	else if(cnt <= 6){
		dutyA = 0.1;
		dutyB = 0.6;
	}
}

// Angle change with cnt value at 
void angle_back(){
	if(cnt == 3){
	dutyA = 0.5;
	dutyB = 0.5;
	}
	else if(cnt == 2){
		dutyA = 0.4;
		dutyB = 0.7;
	}
	else if(cnt == 1){
		dutyA = 0.4;
		dutyB = 0.8;
	}
	else if(cnt <= 0){
		dutyA = 0.4;
		dutyB = 0.9;
	}
	else if(cnt == 4){
		dutyA = 0.7;
		dutyB = 0.4;
	}
	else if(cnt == 5){
		dutyA = 0.8;
		dutyB = 0.4;
	}
	else if(cnt <= 6){
		dutyA = 0.9;
		dutyB = 0.4;
	}
}
