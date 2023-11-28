# LAB: GPIO Digital InOut

**Date:** 2022-09-24

**Name/ID: ** 한태건 21900793

**Github:** repository link

**Demo Video:**  Problem 2 : https://youtu.be/VdfmB1l0U3g

​						  Problem 3 : https://youtu.be/7XoeXW-boC4



# I. Introduction

In this lab, you are required to create a simple program that toggle multiple LEDs with a push-button input. Create HAL drivers for GPIO digital in and out control and use your library.



### Requirement

#### Hardware

- MCU

  - NUCLEO-F411RE

- Actuator/Sensor/Others:

  - LEDs x 4

  - Resistor 330 ohm x 4, breadboard

    

#### Software

- Keil uVision, CMSIS, EC_HAL library





# II. Problem

## 1. Problem 1: Create EC_HAL library

### Procedure

Create the library directory `\repos\EC\lib\`.

Save your header library files in this directory. [See here for detail.](https://ykkim.gitbook.io/ec/uvision/adding-my-api-header-in-uvision)

> DO NOT make duplicates of library files under each project folders

Create your own library for Digital_In and Out : `ecGPIO.h, ecGPIO.c`



### Code

**ecRCC.h**

The codes below are Digital InOut functions in ecRCC.h.

```c
void RCC_HSI_init(void);  
void RCC_GPIOA_enable(void);   
void RCC_GPIOB_enable(void); 
void RCC_GPIOC_enable(void);
```

**ecGPIO.h**

The codes below are Digital InOut functions in ecGPIO.h.

```c
void GPIO_init(GPIO_TypeDef *Port, int pin,  int mode);  
void GPIO_write(GPIO_TypeDef *Port, int pin,  int output);  
int  GPIO_read(GPIO_TypeDef *Port, int pin);  
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);  
void GPIO_ospeed(GPIO_TypeDef* Port, int pin,  int speed);  
void GPIO_otype(GPIO_TypeDef* Port, int pin,  int type);  
void GPIO_pupd(GPIO_TypeDef* Port, int pin,  int pupd);
```



**Example code**

GPIO Mode : Configures GPIO pin modes (Input, Output, AlterFunc, Analog)

```c
/* ecGPIO.c  */

// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, unsigned int mode){
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |= mode<<(2*pin);    
}
```

GPIO Speed : Configures the Output speed (Low, Medium, Fast, High)

```c
// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, unsigned int speed){
	Port->OSPEEDR &= ~(3UL<<(2*pin));
	Port->OSPEEDR |= speed<<(2*pin);
}
```

GPIO Output Type : Configures the Output Type (output push-pull, output open drain)

```c
// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(GPIO_TypeDef *Port, int pin, unsigned int type){
	Port->OTYPER &= ~(1UL)<< pin;
	Port->OTYPER |= (type<< pin);
	
}
```

GPIO pupd : Configures the Output Type (No pull-up / pull-down, Pull-up, Pull-down, Reserved) 

```c
// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(GPIO_TypeDef *Port, int pin, unsigned int pupd){
	Port->PUPDR &= ~(3UL<<2*pin);
	Port->PUPDR |= pupd<<(2*pin);	//write
}
```

GPIO Read : Input data register

```c
int GPIO_read(GPIO_TypeDef *Port, int pin){
	unsigned int BVal = (Port->IDR)>>pin & (1);

	return 0;  
}
```

GPIO write : Output data register

```c
void GPIO_write(GPIO_TypeDef *Port, int pin, unsigned int Output){
	Port->ODR &= ~(1<<pin);
	Port->ODR |= (Output << pin);
}
```





## Problem 2: Toggle LED with Button

### Procedure

1. Create a new project under the directory `\repos\EC\LAB\`

- The project name is “**LAB_GPIO_DIO_LED”.**

- Name the source file as “**LAB_GPIO_DIO_LED.c”**

- Use the [example code provided here](https://github.com/ykkimhgu/EC-student/blob/main/lab/lab-student/LAB_GPIO_DIO_LED_student.c).

  

2. Include your library **ecGPIO.h, ecGPIO.c** in `\repos\EC\lib\`.

> You MUST write your name in the top of the source file, inside the comment section.



3. Toggle the LED by pushing the button.

- Push button (LED ON), Push Button (LED OFF) and repeat



### Configuration

| Button (B1)   | LED                               |
| ------------- | --------------------------------- |
| Digital In    | Digital Out                       |
| GPIOC, Pin 13 | GPIOA, Pin 5                      |
| PULL-UP       | Open-Drain, Pull-up, Medium Speed |





### Code

The codes below set up what is needed to solve the problem.

```c
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	unsigned int ledpin = LOW;
	int button_press = 0; // 1: button pressed, 0: no press or press again after press once
	int button_state = 0; // 0: down, 1: up
```

Initiate the infinite loop.

```c
// Inifinite Loop ----------------------------------------------------------
	while(1){
			if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
					button_press = 1;
					button_state = 0;
			}
				else
					button_state = 1;
			
				if(button_press == 1 && button_state == 1){
					ledpin = !ledpin;
					button_press = 0;
				}
				else{
					button_state = 1;
				}
				GPIO_write(GPIOA, LED_PIN, ledpin);
	}
}
```

The codes below are Digital InOut functions.

```c
// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  		// calls RCC_GPIOC_enable()
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    		// calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PIN, EC_PU);	    	// calls pull-up 
    GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);		// calls pull-up
	GPIO_otype(GPIOA, LED_PIN, 1);				// calls Output Open-Drain
	GPIO_ospeed(GPIOA, LED_PIN, EC_MEDIUM);		// calls Medium speed

}
```



### Discussion

1. Find out a typical solution for software debouncing and hardware debouncing.

   If the microprocessor reads the output due to mechanical vibration, the part entered when entering the key may be mistakenly recognized as having pressed the key several times. This phenomenon is caused by a bounce. Accordingly, debouncing may control a bounce phenomenon and obtain a stable input signal.

   Software debounceing uses a delay function. During this delay, any additional changes in the state of the switch are ignored. This allows for effective filtering of noise or sudden state changes. However, I used conditional statements such as 'if - else' to debounce by changing variables through state changes.

   In general, Hardware debouncing uses resistors and capacitors. These can make an RC low-pass filter.  This filter smooths out the signal transitions by preventing rapid voltage changes when the button is pressed or released. And using a flip-flop or latch can solve the bouncing problem. It stores the state of the button. When the button is pushed, it toggles the flip-flop or latch, and outputs the stable output. This method mitigates bouncing.



2. What method of debouncing did this NUCLEO board use for the push-button(B1)?

   As mentioned above, conditional statements were used. No delay variables were used. In the code, several variables were used. So, whenever the state changed, the values of the variables were adjusted in the condition statement to output the desired result value.



### Analysis

Push-Pull determines the output of the output port using internal power. A stable voltage level is provided by driving the voltage level. This is because the output can be actively connected to VCC or ground through the internal transistor.
Unlike Push-Pull, Open-Drain uses external power, not internal power. It is used due to the voltage difference between the power source of the MCU and the external device connected to the external port. If it does not work at low, use an external push-pull resistor to operate at high.





#### Demo Video

Link : https://youtu.be/7XoeXW-boC4

<img src="C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20230924002655393.png" alt="image-20230924002655393" style="zoom: 25%;" />





## Problem 3 : Toggle LED with Button

### Procedure

1. Create a new project under the directory `\repos\EC\LAB\`

- The project name is “**LAB_GPIO_DIO_multiLED”.**
- Name the source file as “**LAB_GPIO_DIO_multiLED.c”**

> You MUST write your name in the top of the source file, inside the comment section.
>
> 

​	2.Include your library **ecGPIO.h, ecGPIO.c** in `\repos\lib\`.



​	3.Connect 4 LEDs externally with necessary load resistors.

- As Button B1 is Pressed, light one LED at a time, in sequence.
- Example: LED0--> LED1--> …LED3--> …LED0….



### Configuration

| Button        | LED                              |
| ------------- | -------------------------------- |
| Digital In    | Digital Out                      |
| GPIOC, Pin 13 | PA5, PA6, PA7, PB6               |
| PULL-UP       | Push-Pull, Pull-up, Medium Speed |



### Circuit Diagram

Circuit diagram

<img src="C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20230923204557426.png" alt="image-20230923204557426" style="zoom:50%;" />



### Code

The codes below set up what is needed to solve the problem.

```c
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	unsigned int ledpin = 0;
	int button_press = 0; // 1: button pressed, 0: no press or press again after press once
	int button_state = 0; // 0: down, 1: up
```

Initiate the infinite loop.

```c
	// Inifinite Loop ----------------------------------------------------------
	while(1){

			if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
					button_press = 1;
					button_state = 0;
			}
				else
					button_state = 1;
			
				if(button_press == 1 && button_state == 1){
					// Status according to ledpin
					switch(ledpin){
					case 0: 	// First LED ON, Others OFF
						GPIO_write(GPIOA, 5, HIGH);
						GPIO_write(GPIOA, 6, LOW);
						GPIO_write(GPIOA, 7, LOW);
						GPIO_write(GPIOB, 6, LOW);
						ledpin++;
						break;
					case 1: 	// Second LED ON, Others OFF
						GPIO_write(GPIOA, 5, LOW);
						GPIO_write(GPIOA, 6, HIGH);
						GPIO_write(GPIOA, 7, LOW);
						GPIO_write(GPIOB, 6, LOW);
						ledpin++;
						break;
					case 2: 	// Third LED ON, Others OFF
						GPIO_write(GPIOA, 5, LOW);
						GPIO_write(GPIOA, 6, LOW);
						GPIO_write(GPIOA, 7, HIGH);
						GPIO_write(GPIOB, 6, LOW);
						ledpin++;
						break;
					default: 	// Fourth LED ON, Others OFF
						GPIO_write(GPIOA, 5, LOW);
						GPIO_write(GPIOA, 6, LOW);
						GPIO_write(GPIOA, 7, LOW);
						GPIO_write(GPIOB, 6, HIGH);
						ledpin = 0;
				}
					
					button_press = 0;
				}
				else{
					button_state = 1;
				}
```

The codes below are Digital InOut functions.

```c
// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT); 		// calls RCC_GPIOC_enable()
	GPIO_init(GPIOA, LED_PIN, OUTPUT);   		// calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PIN, EC_PU);			// calls pull-up
    GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);		// calls pull-up
	GPIO_otype(GPIOA, LED_PIN, 0);				// calls Output push-pull
	GPIO_ospeed(GPIOA, LED_PIN, EC_MEDIUM);		// calls Medium speed
	
	GPIO_init(GPIOA, 5, OUTPUT);							// PA5_enable
	GPIO_init(GPIOA, 6, OUTPUT);							// PA6_enable
	GPIO_init(GPIOA, 7, OUTPUT);							// PA7_enable
	GPIO_init(GPIOB, 6, OUTPUT);							// PB6_enable

}
```



### Results

Experiment images and results

| ![img](file:///C:/Users/skrua/AppData/Local/Temp/msohtmlclip1/01/clip_image002.jpg) | ![img](file:///C:/Users/skrua/AppData/Local/Temp/msohtmlclip1/01/clip_image004.jpg) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| (1)   Press the button once                                  | (2)   Press the button twice                                 |
| ![img](file:///C:/Users/skrua/AppData/Local/Temp/msohtmlclip1/01/clip_image006.jpg) | ![img](file:///C:/Users/skrua/AppData/Local/Temp/msohtmlclip1/01/clip_image008.jpg) |
| (3)   Press the button three times                           | (4)   Press the button four times                            |

Each time the button is pressed, the lead is turned on in the order of PA5 (led0), PA6 (led1), PA7 (led2), and PB6 (led3).



#### Demo Video

Link : https://youtu.be/wH3xLnVl7x4

<img src="C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20230924002758153.png" alt="image-20230924002758153" style="zoom: 25%;" />



### Discussion

Find out a typical solution for software debouncing and hardware debouncing. What method of debouncing did this NUCLEO board use for the push-button(B1)?

Typical solution for software debouncing and hardware debouncing is written down in  Discussion of Problem 2. 

Conditional statements were used and delay variables were not used. Software debouncing was implemented by adding a switch conditional statement to the conditional statement in the Problem 2 code. I created an algorithm that changes the led switch when I press and release the button. I press the button once, but sometimes the button is not recognized properly due to vibration. Therefore, controls the LED by detecting the button press and release states and creates a pattern in which the LED circulates. The result values were output differently depending on the situation with various conditional statements.



# III. Reference & Troubleshooting

### Reference

```
Young-Keun Kim (2023). https://ykkim.gitbook.io/ec/
```



### Troubleshooting

At first, only one 'if-else' conditional statement was used, causing a bouncing problem. The reason is that I thought of pushing and releasing the button as one. So, I tried to delay the time by controlling pressing and releasing separately. There is a way to use the delay function to solve this problem, but I added a variable and used another 'if-else' conditional statement. Through this problem, I debouncing was performed.
