/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Han Taegeon
Modified         : 2023-10-14
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO_7-segment
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"



void GPIO_init(GPIO_TypeDef *Port, int pin, unsigned int mode){     
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	if (Port == GPIOD)
		RCC_GPIOD_enable();
	
	// Make it for GPIOB, GPIOD..GPIOH

	// You can also make a more general function of
	// void RCC_GPIO_enable(GPIO_TypeDef *Port); 

	GPIO_mode(Port, pin, mode);
	
}


// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, unsigned int mode){
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |= mode<<(2*pin);    
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, unsigned int speed){
	Port->OSPEEDR &= ~(3UL<<(2*pin));
	Port->OSPEEDR |= speed<<(2*pin);
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(GPIO_TypeDef *Port, int pin, unsigned int type){
	Port->OTYPER &= ~(1UL)<< pin;
	Port->OTYPER |= (type<< pin);
	
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(GPIO_TypeDef *Port, int pin, unsigned int pupd){
	Port->PUPDR &= ~(3UL<<2*pin);
	Port->PUPDR |= pupd<<(2*pin);	//write
}

int GPIO_read(GPIO_TypeDef *Port, int pin){
	unsigned int BVal = (Port->IDR)>>pin & (1);

	return BVal;  
}

void GPIO_write(GPIO_TypeDef *Port, int pin, unsigned int Output){
	Port->ODR &= ~(1<<pin);
	Port->ODR |= (Output << pin);
}


void sevensegment_init(void){
		// Calls RCC_GPIO_enable()
		GPIO_init(GPIOA, LED_PA5, OUTPUT);		
		GPIO_init(GPIOA, LED_PA6, OUTPUT);
		GPIO_init(GPIOA, LED_PA7, OUTPUT);
		GPIO_init(GPIOB, LED_PB6, OUTPUT);
		GPIO_init(GPIOC, LED_PC7, OUTPUT);
		GPIO_init(GPIOA, LED_PA9, OUTPUT);
		GPIO_init(GPIOA, LED_PA8, OUTPUT);
		GPIO_init(GPIOB, LED_PB10, OUTPUT);
}


void sevensegment_decoder(uint8_t  num){
	  //pins are sorted from upper left corner of the display to the lower right corner
    //the display has a common cathode
    //the display actally has 8 led's, the last one is a dot 
		unsigned int led[8]={LED_PA5,LED_PA6,LED_PA7,LED_PB6,LED_PC7,LED_PA9,LED_PA8,LED_PB10};
	
		//each led that has to light up gets a 1, every other led gets a 0
		//its in order of the DigitalOut Pins above
		unsigned int number[10][8]={
												{0,0,0,0,0,0,1,1},    //zero
												{1,0,0,1,1,1,1,1},    //one
												{0,0,1,0,0,1,0,1},    //two
												{0,0,0,0,1,1,0,1},    //three
												{1,0,0,1,1,0,0,1},    //four
												{0,1,0,0,1,0,0,1},    //five
												{0,1,0,0,0,0,0,1},    //six
												{0,0,0,1,1,0,1,1},    //seven
												{0,0,0,0,0,0,0,1},    //eight
												{0,0,0,0,1,0,0,1},    //nine				
		};
            //all led's off
        for(int i = 0; i<8;i++){led[i] = 0;}
 
            //display shows the number in this case 6
        for (int i=0; i<8; i++){led[i] = number[num][i];}         //the digit after "number" is displayed
	
		GPIO_write(GPIOA, LED_PA5,  led[0]);
		GPIO_write(GPIOA, LED_PA6,  led[1]);
		GPIO_write(GPIOA, LED_PA7,  led[2]);
		GPIO_write(GPIOB, LED_PB6,  led[3]);
		GPIO_write(GPIOC, LED_PC7,  led[4]);
		GPIO_write(GPIOA, LED_PA9,  led[5]);
		GPIO_write(GPIOA, LED_PA8,  led[6]);
		GPIO_write(GPIOB, LED_PB10, led[7]);
}


void sevensegment_display_init(void){
			// Calls RCC_GPIO_enable()
		GPIO_init(GPIOA, LED_PA7, OUTPUT);		
		GPIO_init(GPIOB, LED_PB6, OUTPUT);
		GPIO_init(GPIOC, LED_PC7, OUTPUT);
		GPIO_init(GPIOA, LED_PA9, OUTPUT);
}

void sevensegment_display(uint8_t  num){
	//pins are sorted from upper left corner of the display to the lower right corner
    //the display has a common cathode
    //the display actally has 8 led's, the last one is a dot 
		unsigned int led[4]={LED_PA7,LED_PB6,LED_PC7,LED_PA9}; // A,B,C,D
	
		//each led that has to light up gets a 1, every other led gets a 0
		//its in order of the DigitalOut Pins above
		unsigned int number[10][4]={		
												{0,0,0,0},		//zero
												{0,0,0,1},    //one
												{0,0,1,0},    //two
												{0,0,1,1},    //three
												{0,1,0,0},    //four
												{0,1,0,1},    //five
												{0,1,1,0},    //six
												{0,1,1,1},    //seven
												{1,0,0,0},    //eight
												{1,0,0,1},    //nine	
		};
            //all led's off
        for(int i = 0; i<4;i++){led[i] = 0;}
 
            //display shows the number in this case 6
        for (int i=0; i<4; i++){led[i] = number[num][i];}         //the digit after "number" is displayed
	
		GPIO_write(GPIOA, LED_PA7,  led[3]);
		GPIO_write(GPIOB, LED_PB6,  led[2]);
		GPIO_write(GPIOC, LED_PC7,  led[1]);
		GPIO_write(GPIOA, LED_PA9,  led[0]);

}

void LED_toggle(){
	int led_state = GPIO_read(GPIOA, LED_PIN);
	int time = 0;
	while(time < 1000)
		time++;
	
	GPIO_write(GPIOA, LED_PIN, !led_state);
}