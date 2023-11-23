/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Han Taegeon
Modified         : 2023-11-16
Language/ver     : C++ in Keil uVision

Description      : USART_Bluetooth
/----------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecSysTick.h"

#define MAX_BUF 	10
#define END_CHAR 	13
#define MOTOR_A	2
#define MOTOR_B	3

static volatile uint8_t buffer[MAX_BUF]={0, };
static volatile uint8_t PC_string[MAX_BUF]={0, };
static volatile uint8_t PC_data = 0;
static volatile uint8_t BT_data = 0;


static float dutyA = 0;		// PWM of Motor A 
static float dutyB = 0;   // PWM of Motor B 

static volatile int bReceive =0; // flag

// Initiallization
void setup(void){
	RCC_PLL_init();
	SysTick_init();
	
	// PWM init
	PWM_init(PA_0);
	PWM_init(PA_1);

	PWM_period_us(PA_0, 200);   // 1 usec PWM period
	PWM_period_us(PA_1, 200);   // 1 usec PWM period
	
	// GPIO 
	GPIO_init(GPIOA, LED_PIN, OUTPUT);	// LED PIN
	GPIO_init(GPIOC, MOTOR_A, OUTPUT);	// motorA direction
	GPIO_init(GPIOC, MOTOR_B, OUTPUT);	// motorB direction
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
}


int main(void){	
	setup();
	printf("MCU Initialized\r\n");	
	
	while(1){
		if (bReceive == 1){			// flag
			bReceive = 0;
		
				// LED ON/OFF
				if(BT_data == 'A')	GPIO_write(GPIOA, LED_PIN, 0);
				else if(BT_data == 'H') GPIO_write(GPIOA, LED_PIN, 1);
					
					// Control a car direction and speed
					switch(BT_data){
						case 'L' : 	// Go left
							GPIO_write(GPIOC, MOTOR_A, 1);
							GPIO_write(GPIOC, MOTOR_B, 1);
							dutyA = 0.5;
							dutyB = 0.2;
						break;
						case 'R' :  // Go right
							GPIO_write(GPIOC, MOTOR_A, 1);
							GPIO_write(GPIOC, MOTOR_B, 1);
							dutyA = 0.2;
							dutyB = 0.5;
						break;
						case 'U' :	// Go straight
							GPIO_write(GPIOC, MOTOR_A, 1);
							GPIO_write(GPIOC, MOTOR_B, 1);						
							dutyA = 0.2;
							dutyB = 0.2;
						break;
						case 'S' :	// Stop
							GPIO_write(GPIOC, MOTOR_A, 1);
							GPIO_write(GPIOC, MOTOR_B, 1);
							dutyA = 1;
							dutyB = 1;
						break;
						
						case 'B' :	// Go back
							GPIO_write(GPIOC, MOTOR_A, 0);
							GPIO_write(GPIOC, MOTOR_B, 0);
							dutyA = 0.8;
							dutyB = 0.8;
						break;
						case 'N' :	// Go back left
							GPIO_write(GPIOC, MOTOR_A, 0);
							GPIO_write(GPIOC, MOTOR_B, 0);
							dutyA = 0.8;
							dutyB = 0.5;
						break;
						case 'V' :	// Go back right
							GPIO_write(GPIOC, MOTOR_A, 0);
							GPIO_write(GPIOC, MOTOR_B, 0);
							dutyA = 0.5;
							dutyB = 0.8;
						break;
						default	 : break;
			}
		}
		PWM_duty(PA_0, dutyA);
		PWM_duty(PA_1, dutyB);
	}
}

void USART2_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART2_RXNE()){
		PC_data = USART2_read();		// RX from UART2 (PC)
		
		USART2_write(&PC_data,1);		// TX to USART2	 (PC)	 Echo of keyboard typing
		//USART1_write(&PC_data,1);		// TX to USART1	 (BT)
	}
}

void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART1_RXNE()){
		
		BT_data = USART1_read();					// RX from USART1 
		USART_write(USART1, &BT_data, 1);	// TX to USART1
			
		if(BT_data == END_CHAR)		// Press enter key
			USART_write(USART1, "\r\n", 2);
		
		bReceive = 1; 	// flag = 1
		}		
}
