# LAB: EXTI & SysTick



**Date:** 2023-10-14

**Name/ID** : 한태건 21900793

**Demo Video:**  Problem1: https://youtu.be/EVq-_aUwseo

​						  Problem2: https://youtu.be/uHPben6xLv0



## Introduction

In this lab, you are required to create two simple programs using interrupt:

(1) displaying the number counting from 0 to 9 with Button Press

(2) counting at a rate of 1 second

You must submit

- LAB Report (*.md & *.pdf)
- Zip source files(main*.c, ecRCC.h, ecGPIO.h, ecSysTick.c etc...).
  - Only the source files. Do not submit project files



### Requirement

#### Hardware

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - 4 LEDs and load resistance
  - 7-segment display(5101ASR)
  - Array resistor (330 ohm)
  - breadboard



#### Software

- Keil uVision, CMSIS, EC_HAL library











## Problem 1: Counting numbers on 7-Segment using EXTI Button

### 1-1. Create HAL library

1. [Download sample header files](https://github.com/ykkimhgu/EC-student/tree/main/include/lib-student): **ecEXTI_student.h, ecEXTI_student.c**

2. Rename these files as **ecEXTI.h, ecEXTI.c**

- You MUST write your name and other information at the top of the library code files.
- Save these files in your directory `EC \lib\`.

3. Declare and define the following functions in your library : **ecEXTI.h**



**ecEXTI.h**

```c
void EXTI_init(GPIO_TypeDef *port, int pin, int trig_type, int priority);
void EXTI_enable(uint32_t pin);  // mask in IMR
void EXTI_disable(uint32_t pin);  // unmask in IMR
uint32_t  is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);
```



### 1-2. Procedure

1. Create a new project under the directory `\EC\LAB\LAB_EXTI`

- The project name is “**LAB_EXTI”.**
- Create a new source file named as “**LAB_EXTI.c”**

> You MUST write your name on the source file inside the comment section.

2. Include your updated library in `\EC\lib\` to your project.

- **ecGPIO.h, ecGPIO.c**
- **ecRCC.h, ecRCC.c**
- **ecEXTI.h, ecEXTI.c**

3. Use the decoder chip (**74LS47**). Connect it to the bread board and 7-segment display.

> Then, you need only 4 Digital out pins of MCU to display from 0 to 9.

4. First, check if every number, 0 to 9, can be displayed properly on the 7-segment.

5. Then, create a code to display the number counting from 0 to 9 and repeats

- by pressing the push button. (External Interrupt)

6. You must use your library function of EXTI.

7. Refer to an [sample code](https://ykkim.gitbook.io/ec/firmware-programming/example-code#button-interrupt)





### Configuration

| Digital In for Button (B1) | Digital Out for 7-Segment decoder             |
| -------------------------- | --------------------------------------------- |
| Digital In                 | Digital Out                                   |
| PC13                       | PA7, PB6, PC7, PA9                            |
| PULL-UP                    | Push-Pull, No Pull-up-Pull-down, Medium Speed |



### Circuit Diagram

<img width="535" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/fe6abb7d-aa2a-4981-b651-8214ae9e18a8">



### Discussion

1. We can use two different methods to detect an external signal: polling and interrupt. What are the advantages and disadvantages of each approach?

   ㅇPolling : Polling is a method of checking whether a signal has entered by circulating a thread every specific period. Use the while statement to check the code inside. This has the disadvantage of consuming a lot of resources in the system because you have to keep checking every specific cycle. In addition, since it is checked every specific cycle, it is impossible to check whether the signal has come in at the correct time, and there is an error depending on the cycle. However, it has the advantage of being easy to implement codes and easy to change priorities.
   
   
   
   ㅇInterrupt : When an external interrupt pin receives a signal, it immediately runs the interrupt source and returns to the original code. Interrupt is more efficient because it allows the CPU to perform other tasks until an external signal is generated.You can know the exact timing of the signal coming in and the reaction time is fast. The system load is low because it is processed only when interruption occurs. However, implementing interrupt processing is complex, requiring the task of configuring and managing an interrupt service routine (ISR).



2. What would happen if the EXTI interrupt handler does not clear the interrupt pending flag? Check with your code

   If the pending flag is not cleared after an interrupt occurs, the microcontroller continues to run the Interrupt Service Routine (ISR). This means that toggling in the code within the interrupt handler is progressing extremely fast. As a result, the LED appears to be on continuously and the LED of the 7-segment appears to be on as long as it is not initialized. In addition, interrupt has priority. If a high-priority interrupt occurs while serving a low-priority interrupt, a low-priority ISR may not run again. High priority ISRs run first. In addition, interrupts can be nested. In other words, the interrupt may be interrupted by another interrupt. Unpredictable behavior can occur if the interrupt pending flag is not cleared.
   
   
   
   <img width="337" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/75d66d56-b748-4e3a-9fee-68eab6930591">
   
   <img src="https://github.com/hhangun/open_picture/assets/110027113/8db0e2c5-a1a9-4fad-b273-009ab7dc3a97" alt="image" style="zoom: 25%;" />



### Code

Your code goes here.

Explain your source code with the necessary comments.



The codes below set up what is needed to solve the problem.

```c
#include "stm32F411xe.h"

#include "ecEXTI.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"

#define LED_PIN	5
#define BUTTON_PIN 13

// Initialiization 
void setup(void);
void EXTI15_10_IRQHandler(void);

int count = 0;
int time = 0;

int main(void) {
	setup();
	while (1) {}
}
```

The code below is a function that changes numbers each time a button is pressed.

```c
//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {
	time++;
	if (is_pending_EXTI(BUTTON_PIN) && time<50000) {
		sevensegment_display(count % 10);
		clear_pending_EXTI(BUTTON_PIN); 
		count++;
		if(count > 9) count = 0;
	}
}
```

The codes below are set according to the conditions given in the question.

```c
void setup(void)
{
	RCC_PLL_init();
	SysTick_init();
	sevensegment_init();
	GPIO_init(GPIOA, LED_PIN, OUTPUT);
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PD);
	// push-pull
	GPIO_otype(GPIOA, LED_PA7, 0);
	GPIO_otype(GPIOB, LED_PB6, 0);
	GPIO_otype(GPIOC, LED_PC7, 0);
	GPIO_otype(GPIOA, LED_PA9, 0);
	// No pull-up-pull-down
	GPIO_pupd(GPIOA, LED_PA7, EC_NONE);
	GPIO_pupd(GPIOB, LED_PB6, EC_NONE);
	GPIO_pupd(GPIOC, LED_PC7, EC_NONE);
	GPIO_pupd(GPIOA, LED_PA9, EC_NONE);
	// Medium Speed
	GPIO_ospeed(GPIOA, LED_PA7, EC_MEDIUM);
	GPIO_ospeed(GPIOB, LED_PB6, EC_MEDIUM);
	GPIO_ospeed(GPIOC, LED_PC7, EC_MEDIUM);
	GPIO_ospeed(GPIOA, LED_PA9, EC_MEDIUM);
	
	// Priority Highest(0) External Interrupt 
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
}
```





### Results

**Experiment images**

This is the result of expressing that the number changes from 0 to 9 each time a button is pressed.

<img width="360" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/5a6ff139-d9d3-4ffe-9547-837ef6e50c21"><img width="357" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/d0228467-b809-4a5d-8bf5-b2ab608eb3b9">



**Results**

The code is executed and the number increases from 0 to 1 each time a button is pressed. If you press the initialization button (black button) in the middle, the number starts from 0 again.



**Demo Video Link :** https://youtu.be/EVq-_aUwseo





## Problem 2: Counting numbers on 7-Segment using SysTick

Display the number 0 to 9 on the 7-segment LED at the rate of 1 sec. After displaying up to 9, then it should display ‘0’ and continue counting.

When the button is pressed, the number should be reset ‘0’ and start counting again.



### 2-1. Create HAL library

1. [Download sample header files](https://github.com/ykkimhgu/EC-student/tree/main/include/lib-student): **ecSysTick_student.h, ecSysTick_student.c**

2. Rename these files as **ecSysTick.h, ecSysTick.c**

   - You MUST write your name and other information at the top of the library code files.

   - Save these files in your directory `EC \lib\`.

3. Declare and define the following functions in your library : **ecSysTick.h**

   

**ecSysTick.h**

```c
void SysTick_init(uint32_t msec);
void delay_ms(uint32_t msec);
uint32_t SysTick_val(void);
void SysTick_reset (void);
void SysTick_enable(void);
void SysTick_disable (void)
```



### 2-2. Procedure

1. Create a new project under the directory

   `\EC\LAB\LAB_EXTI_SysTick`

- The project name is “**LAB_EXTI_SysTick”.**
- Create a new source file named as “**LAB_EXTI_SysTick.c”**

> You MUST write your name on the source file inside the comment section.



2. Include your updated library in `\EC\lib\` to your project.

- **ecGPIO.h, ecGPIO.c**

- **ecRCC.h, ecRCC.c**

- **ecEXTI.h, ecEXTI.c**

- **ecSysTick.h, ecSysTick.c**

  

3. Use the decoder chip (**74LS47**). Connect it to the bread board and 7-segment display.

   > Then, you need only 4 Digital out pins of MCU to display from 0 to 9.

4. First, check if every number, 0 to 9, can be displayed properly on the 7-segment.

5. Then, create a code to display the number counting from 0 to 9 and repeats at the rate of 1 second.

6. When the button is pressed, it should start from '0' again.

   > Use EXTI for this button reset.



### Configuration

| Digital In for Button (B1) | Digital Out for 7-Segment decoder             |
| -------------------------- | --------------------------------------------- |
| Digital In                 | Digital Out                                   |
| PC13                       | PA7, PB6, PC7, PA9                            |
| PULL-UP                    | Push-Pull, No Pull-up-Pull-down, Medium Speed |



### Circuit Diagram

<img width="535" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/fe6abb7d-aa2a-4981-b651-8214ae9e18a8">



### Code

Your code goes here.

Explain your source code with necessary comments.



The codes below set up what is needed to solve the problem.

```c
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"

int count = 0;
// Initialiization 
void setup(void);
void EXTI15_10_IRQHandler(void);
```

It is a code that increases the number of 7-segment every second by giving delay.

```c
int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();//EXTI for Pin 13

	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		sevensegment_display(count % 10);
		delay_ms(1000);
		count++;
		if (count >9) count =0;
		SysTick_reset();
	}
}
```

The codes below are set according to the conditions given in the question.

```c
void setup(void)
{
	RCC_PLL_init();
	SysTick_init();
	sevensegment_init();
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PD);
	// push-pull
	GPIO_otype(GPIOA, LED_PA7, 0);
	GPIO_otype(GPIOB, LED_PB6, 0);
	GPIO_otype(GPIOC, LED_PC7, 0);
	GPIO_otype(GPIOA, LED_PA9, 0);
	// No pull-up-pull-down
	GPIO_pupd(GPIOA, LED_PA7, EC_NONE);
	GPIO_pupd(GPIOB, LED_PB6, EC_NONE);
	GPIO_pupd(GPIOC, LED_PC7, EC_NONE);
	GPIO_pupd(GPIOA, LED_PA9, EC_NONE);
	// Medium Speed
	GPIO_ospeed(GPIOA, LED_PA7, EC_MEDIUM);
	GPIO_ospeed(GPIOB, LED_PB6, EC_MEDIUM);
	GPIO_ospeed(GPIOC, LED_PC7, EC_MEDIUM);
	GPIO_ospeed(GPIOA, LED_PA9, EC_MEDIUM);
}
```



### Results

**Experiment images** 

<img width="360" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/5a6ff139-d9d3-4ffe-9547-837ef6e50c21"><img width="357" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/d0228467-b809-4a5d-8bf5-b2ab608eb3b9">



**Results**

If you just execute the code, the number goes up from 0 to 9 in 1-second increments. Also, after one second in the number 9, it starts from zero again. It keeps repeating like this. If the number goes up and you press the button in the middle, it starts again from zero.



**Demo Video Link :** https://youtu.be/uHPben6xLv0



## Reference

Young-Keun Kim (2023). https://ykkim.gitbook.io/ec/



## Troubleshooting

Among the LEDs of the 7-segment, only the middle light came in faintly, did not come in, or came in brightly. At first, I thought it was a breadboard problem, but the same phenomenon occurred even when I moved. However, the jump line was clearly stuck in place, but nevertheless, when I changed the line just in case, the problem was solved and then the 7-segment number appeared properly.



After completing the code, I ran it. There was often a problem that the number went up twice when I pressed the button once. I thought it was a bouncing problem, so I increased the time as much as I did before, and the problem was solved.



## Appendix

- ecGPIO.h

```c
#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

#define HIGH 1
#define LOW  0

#define EC_NONE 0
#define EC_PU 1
#define EC_PD 2

#define EC_PUSH_PULL 0
#define EC_OPEN_DRAIN 1

#define EC_LOW 0
#define EC_MEDIUM 1
#define EC_FAST 2
#define EC_HIGH 3

#define LED_PA5 	5
#define LED_PA6 	6
#define LED_PA7 	7
#define LED_PB6 	6
#define LED_PC7 	7
#define LED_PA9 	9
#define LED_PA8 	8
#define LED_PB10 	10
#define BUTTON_PIN 13
#define LED_PIN 5

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void GPIO_init(GPIO_TypeDef *Port, int pin, unsigned int mode);
void GPIO_write(GPIO_TypeDef *Port, int pin, unsigned int Output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_mode(GPIO_TypeDef* Port, int pin, unsigned  int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, unsigned int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, unsigned int type);
void GPIO_pupd(GPIO_TypeDef* Port, int pin, unsigned int pupd);

void sevensegment_init(void); 
void sevensegment_decoder(uint8_t  num);

void sevensegment_display_init(void); 
void sevensegment_display(uint8_t  num);

void LED_toggle();

 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```

- ecGPIO.c

```c
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
```

- ecEXTI.h

```c
#ifndef __EC_EXTI_H
#define __EC_EXTI_H

#include "stm32f411xe.h"

#define FALL 0
#define RISE 1
#define BOTH 2

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void EXTI_init(GPIO_TypeDef *Port, int pin, int trig, int priority);
void EXTI_enable(uint32_t pin);		// mask in IMR
void EXTI_disable(uint32_t pin);	 // unmask in IMR
uint32_t is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);

#ifdef __cplusplus
}
#endif /* __cplusplus */
	 
#endif
```

- ecEXTI.c

```c
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecEXTI.h"


void EXTI_init(GPIO_TypeDef *Port, int Pin, int trig_type,int priority){

	// SYSCFG peripheral clock enable	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;		
	
	// Connect External Line to the GPIO
	int EXTICR_port;
	if			(Port == GPIOA) EXTICR_port = 0;
	else if	(Port == GPIOB) EXTICR_port = 1;
	else if	(Port == GPIOC) EXTICR_port = 2;
	else if	(Port == GPIOD) EXTICR_port = 3;
	else 										EXTICR_port = 4;
	
	SYSCFG->EXTICR[Pin >> 2] &= ~(15 << 4*(Pin & 0x03));			// clear 4 bits
	SYSCFG->EXTICR[Pin >> 2] |= EXTICR_port << 4*(Pin & 0x03);			// connect port number
	
	// Configure Trigger edge
	if (trig_type == FALL) EXTI->FTSR |= 1UL << Pin;   // Falling trigger enable 
	else if	(trig_type == RISE) EXTI->RTSR |= 1UL << Pin;   // Rising trigger enable 
	else if	(trig_type == BOTH) {			// Both falling/rising trigger enable
		EXTI->RTSR |= 1UL << Pin; 
		EXTI->FTSR |= 1UL << Pin;
	} 
	
	// Configure Interrupt Mask (Interrupt enabled)
	EXTI->IMR  |= 1 << Pin;     // not masked
	
	
	// NVIC(IRQ) Setting
	int EXTI_IRQn = 0;
	
	if (Pin == 0) EXTI_IRQn = EXTI0_IRQn;
	else if (Pin == 1) EXTI_IRQn = EXTI1_IRQn;
	else if (Pin == 2) EXTI_IRQn = EXTI2_IRQn;
	else if (Pin == 3) EXTI_IRQn = EXTI3_IRQn;
	else if (Pin == 4) EXTI_IRQn = EXTI4_IRQn;
	else if	(Pin > 4 && Pin < 10) 	EXTI_IRQn = EXTI9_5_IRQn;
	else 			EXTI_IRQn = EXTI15_10_IRQn;
								
	NVIC_SetPriority(EXTI_IRQn, priority);	// EXTI priority
	NVIC_EnableIRQ(EXTI_IRQn); 	// EXTI IRQ enable
}


void EXTI_enable(uint32_t pin) {
	EXTI->IMR |= 1UL << pin;     // not masked (i.e., Interrupt enabled)
}
void EXTI_disable(uint32_t pin) {
	EXTI->IMR |= ~(1UL << pin);     // masked (i.e., Interrupt disabled)
}

uint32_t is_pending_EXTI(uint32_t pin){
	uint32_t EXTI_PRx = pin;     	// check  EXTI pending 	
	return ((EXTI->PR & 1 << pin) == 1 << pin);
}


void clear_pending_EXTI(uint32_t pin){
	EXTI->PR |= 1 << pin;     // clear EXTI pending 
}
```

- ecSysTick.h

```c
#ifndef __EC_SYSTICK_H
#define __EC_SYSTICK_H

#include "stm32f4xx.h"
#include "ecRCC.h"
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

extern volatile uint32_t msTicks;
void SysTick_init(void);
void SysTick_Handler(void);
void SysTick_counter();
void delay_ms(uint32_t msec);
void SysTick_reset(void);
uint32_t SysTick_val(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```

- ecSysTick.c

```c
#include "ecSysTick.h"

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

volatile uint32_t msTicks=0;

//EC_SYSTEM_CLK

void SysTick_init(void){	
	//  SysTick Control and Status Register
	SysTick->CTRL = 0;											// Disable SysTick IRQ and SysTick Counter

	// Select processor clock
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

	// uint32_t MCU_CLK=EC_SYSTEM_CLK
	// SysTick Reload Value Register
	SysTick->LOAD = MCU_CLK_PLL / 1000 - 1;						// 1ms, for HSI PLL = 84MHz.

	// SysTick Current Value Register
	SysTick->VAL = 0;

	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	
	// Enable SysTick IRQ and SysTick Timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
		
	NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC
}

void SysTick_Handler(void){
	SysTick_counter();	
}

void SysTick_counter(){
	msTicks++;
}	

void delay_ms (uint32_t mesc){
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < mesc);
	
	msTicks = 0;
}

void SysTick_reset(void)
{
	// SysTick Current Value Register
	SysTick->VAL = 0;
}

uint32_t SysTick_val(void) {
	return SysTick->VAL;
}
```

