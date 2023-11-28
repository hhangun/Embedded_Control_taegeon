# LAB: Stepper Motor



**Date:** 2023-11-06

**Author:** Han TaeGeon

**Demo Video:** https://youtu.be/fhXFbCNvXks



## Introduction

In this lab, we will learn how to drive a stepper motor with digital output of GPIOs of MCU. You will use a FSM to design the algorithm for stepper motor control.

You must submit

- LAB Report (*.pdf)

- Zip source files(main*.c, ecRCC.h, ecGPIO.h, ecSysTick.c etc...).

  - Only the source files. Do not submit project files

    

### Requirement

#### Hardware

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - 3Stepper Motor 28BYJ-48
  - Motor Driver ULN2003
  - breadboard

#### Software

- Keil uVision, CMSIS, EC_HAL library

  

## Problem 1: Stepper Motor

### Hardware Connection

Read specification sheet of the motor and the motor driver for wiring and min/max input voltage/current.

![img](https://user-images.githubusercontent.com/91526930/197428440-9f4a9c8c-2d81-4d0e-a4e2-b4a4b9def44d.png)

![img](https://user-images.githubusercontent.com/91526930/197428469-a0d7a8fa-ba4c-482f-8688-ea87cfd9f4e0.png)

### Stepper Motor Sequence

We will use unipolar stepper motor for this lab

Fill in the blanks of each output data depending on the below sequence.

**Full-stepping sequence**

![img](https://user-images.githubusercontent.com/91526930/197428513-f9a23147-3448-4bed-bda2-c90325b8c143.png)

<img width="589" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/fac43b11-5113-496b-8714-4c642bf45c31">



**Half-stepping sequence**

![img](https://user-images.githubusercontent.com/91526930/197429006-d552ab16-0bbf-4c52-bdce-a0f2bfe5f0d8.png)

<img width="589" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/aa2d7aef-8c4a-4ca4-a167-84db2148b30a">



### Finite State Machine

Draw a State Table for Full-Step Sequence. Use Moore FSM for this case. If you want, you may use Mealy FSM.

[See *‘Programming FSM’* for hints](https://ykkim.gitbook.io/ec/ec-course/lab/lab-smart-mini-fan-with-stm32-duino#example-code)

Each states(S) mean each steps. And DIR means direction of motor.

- Full-Stepping Sequence

  <img width="628" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/ac7c4bd4-a86a-40e9-818d-a98357c31607">



- Half-Stepping Sequence

  <img width="628" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/36e43729-e0d6-4db3-94e3-ddb3d9e658b1">



## Problem 2: Firmware Programming

### Create HAL library

Download files:

- [ecStepper_student.h, ecStepper_student.c](https://github.com/ykkimhgu/EC-student/blob/main/include/lib-student/)

Then, change the library files as ecStepper.h, ecStepper.c

Declare and define the following functions in your library.

You must update your header files located in the directory `EC \lib\`.

**ecStepper.h**

```c
#ifndef __EC_STEPPER_H
#define __EC_STEPPER_H

#include "ecSTM32F411.h"


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//State mode
#define HALF 0
#define FULL 1	 
	 
/* Stepper Motor */
//stepper motor function

typedef struct{
	GPIO_TypeDef *port1;
	int pin1;
	GPIO_TypeDef *port2;
	int pin2;
	GPIO_TypeDef *port3;
	int pin3;
	GPIO_TypeDef *port4;
	int pin4;
	uint32_t _step_num;
} Stepper_t;

	 
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
void Stepper_setSpeed(long whatSpeed, uint32_t mode);
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode); 
void Stepper_stop(void);
void Stepper_pinOut (uint32_t state, uint32_t mode);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```

> Note that these are blocking stepper controllers. While the stepper is running, the MCU cannot process other polling commands. If you can, modify it to be the non-blocking controller.



### Procedure

1. Create a new project under the directory `\repos\EC\LAB\LAB_Stepper_Motor`

   - The project name is “**LAB_Stepper_Motor”.**

   - Create a new source file named as “**LAB_Stepper_Motor.c”**

     > You MUST write your name on the source file inside the comment section.

2. Include your updated library in `\repos\EC\lib\` to your project.

   - **ecGPIO.h, ecGPIO.c**
   - **ecRCC.h, ecRCC.c**
   - **ecEXTI.h, ecEXTI.c**
   - **ecSysTick.h**, **ecSysTick.c**
   - **ecStepper.h** **ecStepper.h**

3. Connect the MCU to the motor driver and the stepper motor.

4. Find out the number of steps required to rotate 1 revolution using Full-steppping.

5. Then, rotate the stepper motor 10 revolutions with 2 rpm. Measure if the motor rotates one revolution per second.

6. Repeat the above process in the opposite direction.

7. Increase and decrease the speed of the motor as fast as it can rotate to find the maximum and minimum speed of the motor.

8. Apply the half-stepping and repeat the above.

   

### Configuration

| Digital Out                                                  | SysTick |
| :----------------------------------------------------------- | ------- |
| PB10, PB4, PB5, PB3 <br />NO Pull-up Pull-down <br />Push-Pull <br />Fast | delay() |



### Requirement

You have to program the stepping sequence using the state table. You can define the states using structures. Refer to *‘Programming FSM’* for hints.

<img width="89" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/ce72a24f-aeec-49fa-b4f6-9225fd0a61d1" style="zoom: 150%;" >

​											<img width="195" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/75f8085a-d97e-48cb-850a-807ddc50008c" style="zoom:150%;" > <img width="174" alt="image" src="https://github.com/hhangun/open_picture/assets/110027113/68e49778-df7c-43fe-a233-941de64e5349" style="zoom:150%;" >



### Discussion

1. Find out the trapezoid-shape velocity profile for a stepper motor. When is this profile necessary?

   Vibration may occur at high speeds or control control may be unstable. This may be caused by changes in step intervals, motor control according to modes, inertia according to loads, and the like. There are many ways to solve this problem, but one of them is the shapezoid-shape. This softens precise control and movement. In the linear section, the speed of the motor does not change rapidly, but gradually increases or decreases. Therefore, vibration and noise are reduced. Therefore, the trapezoidal-shape is used to reduce vibration and noise at high speeds and to increase stability.

   

2. How would you change the code more efficiently for micro-stepping control? You don’t have to code this but need to explain your strategy.

   The micro-stepping control divides each entire step into smaller steps to smooth the rotation of the motor. In addition, it can be precisely adjusted by increasing the number of states.  This is achieved by controlling the current to the motor windings using the PWM voltage.  The intensity of the current may be adjusted by adjusting the duty ratio through PWM. As values according to the duty ratio are added, they are eventually divided into smaller intervals. In other words, it increases the number of states and softens motor rotation. In the case of full mode, there are four states, S0, S1, S2, and S3, but states with duty values added to each state can be created. The duty ratio can be set as we want. It can be set to 10% or 25%, etc. This micro-stepping control allows us to rotate at a very small angle, so we can get smooth movement at a slow speed.

   



### Code

Your code goes here: [ADD Code LINK such as github](https://github.com/ykkimhgu/EC-student/)

Stepper step according to mode and direction.

```c
#include "ecSTM32F411.h"

void setup(void);
void EXTI15_10_IRQHandler(void);

int main(void){
	 // Initialization 
	setup();
	
	Stepper_step(2048, 0, FULL);		//Step 30x64 Direction 0 or 1 Mode in FULL mode
	//Stepper_step(2048, 1, FULL);
	//Stepper_step(4096, 0, HALF);		//Step 30x128 Direction 0 or 1 Mode in HALF mode
	//Stepper_step(4096, 1, HALF);
	
	// Infinite Loop
	while(1){;}
		
}
```

The codes below are set according to the conditions given in the question.

```c
void setup(void){
	
	RCC_PLL_init();		// System Clock = 84MHz
	SysTick_init();		// SysTick init
	
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);	// External Interrupt Setting
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);		//  GPIOC BUTTON PIN initialization

	
	Stepper_init(GPIOB, 10, GPIOB, 4, GPIOB, 5, GPIOB, 3);		// Stepper GPIO pin initialization
	Stepper_setSpeed(14, FULL);			// set stepper motor speed in FULL mode
	//Stepper_setSpeed(14, HALF);			// set stepper motor speed in HALF mode
}
```

The code below is a function in which the stepper motor stops when the button is pressed by an external interrupt.

```c
void EXTI15_10_IRQHandler(void){
	if (is_pending_EXTI(BUTTON_PIN)) {		 // pendig 1
		Stepper_stop();
		clear_pending_EXTI(BUTTON_PIN);			// cleared by writing '1'
	}
}
```



### Results

Experiment images

<img src="https://github.com/hhangun/open_picture/assets/110027113/4c120df9-d9f0-4aa3-b841-a5f54117304e" alt="image" style="zoom: 25%;" /><img src="https://github.com/hhangun/open_picture/assets/110027113/57cc0942-e49f-47c5-b4b8-4361d5739569" alt="image" style="zoom: 25%;" />



Results

In the spec sheet of the step motor, the step per rev is (5.625/64)degrees, and the gear ratio is 1/32, so the number of steps required to turn 1 in the FULL step is 32 x 64 = 2048. I turned the motor at 2rpm, but it took quite a while, and I increased the rpm to find the maximum speed of the motor, and I could see that 14rpm was the maximum speed. The result was the same when it was done in the opposite direction. When it was done in half mode, the same results as in full mode were obtained, and it was confirmed that vibration and noise were reduced compared to full mode.



#### **Demo Video**

Link: https://youtu.be/fhXFbCNvXks



## Reference

```
Young-Keun Kim (2023). https://ykkim.gitbook.io/ec/
```

```
Danielle, C. (2017, November 21). What Is Microstepping? Linear Motion Tips. https://www.linearmotiontips.com/microstepping-basics/
```



## Troubleshooting

The speed of the step motor adjusts the delay value by converting rpm to msec. The delay calculation is as follows.
$$
1rev = 64*32*step,\ \frac{y*rev}{min}=\frac{y*step*64*32}{min},\ \frac{y*rev}{msec}=\frac{y*step*64*32}{60*1000*msec}
\\ =>\frac{msec}{y*step}=\frac{60*1000}{y*64*32}
$$
The above equation is when FULL mode, and to find the delay of HALF mode, divide 2 with the above equation. The step motor could not be operated because the delay value was not accurately obtained. However, the motor was eventually operated by the above formula, and it was confirmed that it operated normally according to the mode.



## Appendix

- ecStepper.h

  ```c
  #ifndef __EC_STEPPER_H
  #define __EC_STEPPER_H
  
  #include "ecSTM32F411.h"
  
  
  #ifdef __cplusplus
   extern "C" {
  #endif /* __cplusplus */
  
  //State mode
  #define HALF 0
  #define FULL 1	 
  	 
  /* Stepper Motor */
  //stepper motor function
  
  typedef struct{
  	GPIO_TypeDef *port1;
  	int pin1;
  	GPIO_TypeDef *port2;
  	int pin2;
  	GPIO_TypeDef *port3;
  	int pin3;
  	GPIO_TypeDef *port4;
  	int pin4;
  	uint32_t _step_num;
  } Stepper_t;
  
  	 
  void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
  void Stepper_setSpeed(long whatSpeed, uint32_t mode);
  void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode); 
  void Stepper_stop(void);
  void Stepper_pinOut (uint32_t state, uint32_t mode);
  
  
  
  #ifdef __cplusplus
  }
  #endif /* __cplusplus */
  
  #endif
  ```

- ecStepper.c

  ```c
  #include "ecStepper.h"
  
  //State number 
  #define S0 0
  #define S1 1
  #define S2 2
  #define S3 3
  #define S4 4
  #define S5 5
  #define S6 6
  #define S7 7
  
  
  // Stepper Motor function
  uint32_t direction = 1; 
  uint32_t step_delay = 100; 
  uint32_t step_per_rev = 64*32;
  	 
  
  // Stepper Motor variable
  volatile Stepper_t myStepper; 
  
  
  //FULL stepping sequence  - FSM
  typedef struct {
  	uint8_t out;
    	uint32_t next[2];
  } State_full_t;
  
  State_full_t FSM_full[4] = {  
   	{0xC,{S1,S3}}, //0b1100
   	{0x6,{S2,S0}}, //0b0110
   	{0x3,{S3,S1}}, //0b0011
  	{0x9,{S0,S2}}, //0b1001
  };
  
  //HALF stepping sequence
  typedef struct {
  	uint8_t out;
    	uint32_t next[2];
  } State_half_t;
  
  State_half_t FSM_half[8] = { 
   	{0x8,{S1,S7}}, //0b1000
  	{0xC,{S2,S0}}, //0b1100
  	{0x4,{S3,S1}}, //0b0100
  	{0x6,{S4,S2}}, //0b0110
  	{0x2,{S5,S3}}, //0b0010
  	{0x3,{S6,S4}}, //0b0011
  	{0x1,{S7,S5}}, //0b0001
  	{0x9,{S0,S6}}, //0b1001
  };
  
  
  
  void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
  	 
  	//  GPIO Digital Out Initiation
  	myStepper.port1 = port1;
  	myStepper.pin1  = pin1;
  	myStepper.port2 = port2;
  	myStepper.pin2  = pin2;
  	myStepper.port3 = port3;
  	myStepper.pin3  = pin3;
  	myStepper.port4 = port4;
  	myStepper.pin4  = pin4;
  	
  	//  GPIO Digital Out Initiation
  	// No pull-up Pull-down , Push-Pull, Fast	
  	// Pin1 ~ Port4
  	GPIO_init(myStepper.port1, myStepper.pin1, OUTPUT);
  	GPIO_init(myStepper.port2, myStepper.pin2, OUTPUT);
  	GPIO_init(myStepper.port3, myStepper.pin3, OUTPUT);
  	GPIO_init(myStepper.port4, myStepper.pin4, OUTPUT);
  	
  	mcu_init(myStepper.port1, myStepper.pin1);
  	mcu_init(myStepper.port2, myStepper.pin2);
  	mcu_init(myStepper.port3, myStepper.pin3);
  	mcu_init(myStepper.port4, myStepper.pin4);
  }
  
  
  void Stepper_pinOut (uint32_t state, uint32_t mode){	
     	if (mode == FULL){         // FULL mode
  		GPIO_write(myStepper.port1, myStepper.pin1, (FSM_full[state].out & 0x8) >> 3);
    	GPIO_write(myStepper.port2, myStepper.pin2, (FSM_full[state].out & 0x4) >> 2);
  		GPIO_write(myStepper.port3, myStepper.pin3, (FSM_full[state].out & 0x2) >> 1);
  		GPIO_write(myStepper.port4, myStepper.pin4, (FSM_full[state].out & 0x1) >> 0);
  		}	 
   	else if (mode == HALF){    // HALF mode
  		GPIO_write(myStepper.port1, myStepper.pin1, (FSM_half[state].out & 0x8) >> 3);
    	GPIO_write(myStepper.port2, myStepper.pin2, (FSM_half[state].out & 0x4) >> 2);
  		GPIO_write(myStepper.port3, myStepper.pin3, (FSM_half[state].out & 0x2) >> 1);
  		GPIO_write(myStepper.port4, myStepper.pin4, (FSM_half[state].out & 0x1) >> 0);
  	}
  }
  
  
  void Stepper_setSpeed (long whatSpeed, uint32_t mode){      		// rpm [rev/min]
  	if(mode == FULL)	
  		step_delay = 	(60*1000)/(whatSpeed*step_per_rev);  // Convert rpm to  [msec] delay
  	else if(mode == HALF)	
  		step_delay = 	(60*1000)/(whatSpeed*step_per_rev*2);  // Convert rpm to  [msec] delay
  }
  
  
  void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode){
  	 uint32_t state = 0;
  	 myStepper._step_num = steps;
  
  	 for(; myStepper._step_num > 0; myStepper._step_num--){ // run for step size
  		delay_ms(step_delay);                     		  // delay (step_delay); 				 
  	    	if (mode == FULL)
  			state = FSM_full[state].next[direction];      // state = next state
  		else if (mode == HALF) 
  			state = FSM_half[state].next[direction];      // state = next state		
  		
  		Stepper_pinOut(state, mode);
     	}
  }
  
  
  void Stepper_stop (void){ 
  		myStepper._step_num = 1;
  	GPIO_write(myStepper.port1, myStepper.pin1, myStepper._step_num);
  	GPIO_write(myStepper.port2, myStepper.pin2, myStepper._step_num);
  	GPIO_write(myStepper.port3, myStepper.pin3, myStepper._step_num);
  	GPIO_write(myStepper.port4, myStepper.pin4, myStepper._step_num);
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

- ecGPIO.h

  ```c
  #ifndef __ECGPIO_H
  #define __ECGPIO_H
  
  
  #include "ecRCC.h"
  
  
  #define INPUT  0x00
  #define OUTPUT 0x01
  #define AF     0x02
  #define ANALOG 0x03
  
  #define HIGH 1
  #define LOW  0
  
  #define EC_NONE 0
  #define  EC_PU 1
  #define EC_PD 2
  
  #define EC_PUSH_PULL 0
  #define EC_OPEN_DRAIN 1
  
  #define EC_LOW 0
  #define EC_MEDIUM 1
  #define EC_FAST 2
  #define EC_HIGH 3
  
  #define LED_A0 0
  #define LED_A1 1
  #define LED_B0 0
  #define LED_C1 1
  #define LED_PB9		9
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
  
  #define Direction_PIN 2
  #define PWM_PIN PA_0
  
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
  void LED_UP(uint8_t  num);
  
  void LED_toggle();
  
  void mcu_init(GPIO_TypeDef* Port, int pin);
   
  #ifdef __cplusplus
  }
  #endif /* __cplusplus */
  
  #endif
  ```

- ecGPIO.c

  ```c
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
  		unsigned int led[8]={LED_PB9,LED_PA6,LED_PA7,LED_PB6,LED_PC7,LED_PA9,LED_PA8,LED_PB10};
  	
  		//each led that has to light up gets a 1, every other led gets a 0
  		//its in order of the DigitalOut Pins above
  		unsigned int number[11][8]={
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
  											  {0,0,1,1,0,0,0,1},		//P
  		};
              //all led's off
          for(int i = 0; i<8;i++){led[i] = 0;}
   
              //display shows the number in this case 6
          for (int i=0; i<8; i++){led[i] = number[num][i];}         //the digit after "number" is displayed
  	
  		GPIO_write(GPIOB, LED_PB9,  led[0]);
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
  
  void LED_UP(uint8_t  num) {
  	unsigned int led[4] = {LED_A0, LED_A1, LED_B0, LED_C1};
  	unsigned int number[16][4]={
  												{0,0,0,0},    //zero			
  												{0,0,0,1},    //one       
  												{0,0,1,0},    //two       
  												{0,0,1,1},    //three     
  												{0,1,0,0},    //four
  												{0,1,0,1},    //five
  												{0,1,1,0},    //six
  												{0,1,1,1},    //seven
  												{1,0,0,0},    //eight
  												{1,0,0,1},    //nine		
  												{1,0,1,0},    //ten
  												{1,0,1,1},    //eleven
  												{1,1,0,0},    //twelve
  												{1,1,0,1},    //thirteen
  												{1,1,1,0},    //fourteen
  												{1,1,1,1},		//fifteen
  											};
  
  											for(int i = 0; i<4;i++){led[i] = 0;}
  											for (int i=0; i<4; i++){led[i] = number[num][i];}
  											
  		GPIO_write(GPIOA, LED_A0, led[0]);
  		GPIO_write(GPIOA, LED_A1, led[1]);
  		GPIO_write(GPIOB, LED_B0, led[2]);
  		GPIO_write(GPIOC, LED_C1, led[3]);
  }
  
  
  void LED_toggle(){
  	int led_state = GPIO_read(GPIOA, LED_PIN);
  	int time = 0;
  	while(time < 1000)
  		time++;
  	
  	GPIO_write(GPIOA, LED_PIN, !led_state);
  }
  
  void mcu_init(GPIO_TypeDef *Port, int pin){
  	GPIO_pupd(Port, pin, EC_NONE);
  	GPIO_otype(Port, pin, EC_PUSH_PULL);
  	GPIO_ospeed(Port, pin, EC_FAST);
  }
  ```

- ecSTM32F411.h

  ```c
  #include "ecEXTI.h"
  #include "ecGPIO.h"
  #include "ecPinNames.h"
  #include "ecPWM.h"
  #include "ecRCC.h"
  #include "ecSysTick.h"
  #include "ecTIM.h"
  #include "ecUART_simple.h"
  #include "ecPWM.h"
  #include "ecPinNames.h"
  #include "ecStepper.h"
  
  #include "stm32f411xe.h"
  #include "stm32f4xx.h"
  #include "math.h"
  ```

  