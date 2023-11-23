/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Han Taegeon
Modified         : 2023-11-11
Language/ver     : C++ in Keil uVision

Description      : USART_LED
/----------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecSysTick.h"

#define MAX_BUF 	10
#define END_CHAR 	13

static volatile uint8_t buffer[MAX_BUF]={0, };
static volatile uint8_t PC_string[MAX_BUF]={0, };
static volatile uint8_t PC_data = 0;
static volatile uint8_t BT_data = 0;


static volatile int idx = 0;
static volatile int bReceive =0;

void setup(void){
	RCC_PLL_init();
	SysTick_init();
	
	// USART2: USB serial init
	UART2_init();
	UART2_baud(BAUD_9600);

	// USART1: BT serial init 
	UART1_init();
	UART1_baud(BAUD_9600);
}

int main(void){	
	setup();
	printf("MCU Initialized\r\n");	
	
	while(1){
		if (bReceive == 1){
			printf("PC_string: %s\r\n", PC_string);				
			bReceive = 0;
		}
	}
}

void USART2_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART2_RXNE()){
		PC_data = USART2_read();		// RX from UART2 (PC)
		
		USART2_write(&PC_data,1);		// TX to USART2	 (PC)	 Echo of keyboard typing
		USART1_write(&PC_data,1);		// TX to USART1	 (BT)
		
	}
}


void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART1_RXNE()){
		GPIO_init(GPIOA, LED_PIN, OUTPUT);
		if(BT_data == 'L')	GPIO_write(GPIOA, LED_PIN, 1);
		else if(BT_data == 'H') GPIO_write(GPIOA, LED_PIN, 0);
		BT_data = USART1_read();

		printf("RX: %c \r\n",BT_data); // TX to USART2(PC)
	}
}