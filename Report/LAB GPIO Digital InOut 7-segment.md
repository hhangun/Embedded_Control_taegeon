# LAB: GPIO Digital InOut 7-segment

**Date** : 2023-09-30

**Name/ID** : 한태건 21900793

**Demo video** : https://youtu.be/aCHubyOOQ_Q



#  I. Introduction

In this lab, you are required to create a simple program to control a 7-segment display to show a decimal number (0~9) that increases by pressing a push-button.



## Requirement

##### Hardware

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - 7-segment display(5101ASR)
  - Array resistor (330 ohm)
  - breadboard

##### Software

- Keil uVision, CMSIS, EC_HAL library



## Exercise

Fill in the table

![image-20230926192447646](C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20230926192447646.png)





# II. Problem

## Problem 1: Connecting 7-Segment Display

### Procedure

Review 7-segment Decoder and Display from Digital Logic lecture.

- Read here: [7-segment tutorial]()
- Read here: [How to connect 7-segment decoder to MCU]()

The popular BCD 7-segment decoder chips are **74LS47 and CD4511**.

Instead of using the decoder chip, we are going to make the 7-segment decoder with the MCU programming.

> Do not use the 7-segmment decoder

![image-20230926192627926](C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20230926192627926.png)

Connect the common anode 7-segment with the given array resistors.

Apply VCC and GND to the 7-segment display.

Apply 'H' to any 7-segment pin 'a'~'g' and observe if that LED is turned on or off

- example: Set 'H' on PA5 of MCU and connect to 'a' of the 7-segment.



### Connection Diagram

Circuit diagram

![image-20230926192804864](C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20230926192804864.png)



### Discussion

1. Draw the truth table for the BCD 7-segment decoder with the 4-bit input.

##### Truth table

![a](C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20230927223541233.png)

a~g : 7-segment pin 





2. What are the common cathode and common anode of 7-segment display?

   In the case of the Common Cathode 7-segment display, the cathodes of all seven segments are connected together and share a common connection. The cathode is also called a negative terminal. The anode terminals are connected to the ground, and a ground or Vcc is connected to the + pole to control the light of the LED.

   In the case of the Common Anode 7-segment display, the anodes of all seven segments are connected together and share a common connection. The anode is also called a positive terminal. The anode terminals are connected to the Vcc, and the ground or Vcc is connected to the - pole to control the light of the LED



3. Does the LED of a 7-segment display (common anode) pin turn ON when 'HIGH' is given to the LED pin from the MCU?

   No, It is a common anode, so the LED turns on when it receives a LOW signal, not a HIGH signal.





## Problem 2: Display 0~9 with button press

### Procedure

1. Create a new project under the directory `\repos\EC\LAB\LAB_GPIO_7segment`

- The project name is “**LAB_GPIO_7segment”.**
- Create a new source file named as “**LAB_GPIO_7segment.c”**
- Refer to the [sample code](https://github.com/ykkimhgu/EC-student/tree/main/tutorial/tutorial-student)



2. Include your updated library in `\repos\EC\lib\` to your project.

- **ecGPIO.h, ecGPIO.c**
- **ecRCC.h, ecRCC.c**



3. Declare and Define the following functions in your library

- You can refer to [an example code of 7-segment control](https://os.mbed.com/users/ShingyoujiPai/code/7SegmentDisplay/file/463ff11d33fa/main.cpp/)

**ecGPIO.h**

```
void sevensegment_init(void); 
void sevensegment_decoder(uint8_t  num);
```

1. First, check if every number, 0 to 9, can be displayed properly
2. Then, create a code to display the number from 0 to 9 with each button press. After the number '9', it should start from '0' again.



### Configuration

| Digital In for Button (B1) | Digital Out for 7-Segment                                    |
| -------------------------- | ------------------------------------------------------------ |
| Digital In                 | Digital Out                                                  |
| PC13                       | PA5, PA6, PA7, PB6, PC7, PA9, PA8, PB10 ('a'~'h', respectively) |
| PULL-UP                    | Push-Pull, No Pull-up-Pull-down, Medium Speed                |



### Code

The codes below set up what is needed to solve the problem.

```c
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define BUTTON_PIN 13
#define B_DOWN 0
#define B_UP 1

void setup(void);
	
int main(void) {	
	// Initialiization --------------------------------------------------------
	setup();
	unsigned int cnt = 0;
	int button_state = 0;			// button down(0), button up(1)
	int button_press = 0;			// button pressed once(1), no press or press more after one pressing(0)
```

nitiate the infinite loop.

```c
while(1){
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
			button_state = B_DOWN;
			button_press = 1;
			
		}
		else
			button_state = B_UP;
		
		if(button_state == B_UP && button_press == 1){		
				if (cnt > 9) 
					cnt = 0;
					
				sevensegment_decoder(cnt % 10);

				for(int i = 0; i < 500000;i++){}			// before it gets tired
					
				button_press = 0;
				cnt++;
		}
	}		
}
```

The codes below are Digital InOut functions.

```c
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);  // calls pull-up
	// push-pull
	GPIO_otype(GPIOA, LED_PA5, 0);
	GPIO_otype(GPIOA, LED_PA6, 0);
	GPIO_otype(GPIOA, LED_PA7, 0);
	GPIO_otype(GPIOB, LED_PB6, 0);
	GPIO_otype(GPIOC, LED_PC7, 0);
	GPIO_otype(GPIOA, LED_PA9, 0);
	GPIO_otype(GPIOA, LED_PA8, 0);
	GPIO_otype(GPIOB, LED_PB10, 0);
	// No pull-up-pull-down
	GPIO_pupd(GPIOA, LED_PA5, EC_NONE);
	GPIO_pupd(GPIOA, LED_PA6, EC_NONE);
	GPIO_pupd(GPIOA, LED_PA7, EC_NONE);
	GPIO_pupd(GPIOB, LED_PB6, EC_NONE);
	GPIO_pupd(GPIOC, LED_PC7, EC_NONE);
	GPIO_pupd(GPIOA, LED_PA9, EC_NONE);
	GPIO_pupd(GPIOA, LED_PA8, EC_NONE);
	GPIO_pupd(GPIOB, LED_PB10, EC_NONE);
	// Medium Speed
	GPIO_ospeed(GPIOA, LED_PA5, EC_MEDIUM);
	GPIO_ospeed(GPIOA, LED_PA6, EC_MEDIUM);
	GPIO_ospeed(GPIOA, LED_PA7, EC_MEDIUM);
	GPIO_ospeed(GPIOB, LED_PB6, EC_MEDIUM);
	GPIO_ospeed(GPIOC, LED_PC7, EC_MEDIUM);
	GPIO_ospeed(GPIOA, LED_PA9, EC_MEDIUM);
	GPIO_ospeed(GPIOA, LED_PA8, EC_MEDIUM);
	GPIO_ospeed(GPIOB, LED_PB10, EC_MEDIUM);
	// Initialiization of output
	sevensegment_init();
    // Start from 0
    sevensegment_decoder(0);
}
```



### Results

**Experiment images**

![image-20230928214933362](C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20230928214933362.png)



Show from 0 to 9 using the 7-segment LED. Starting at zero, each time a button is pressed, the number increases by one. The 7-segment only shows up to the number 9, so if you press it again on the number 9, it shows again from 0. Of course, the dot part LED didn't keep on turning on.



#### Demo Video

Link : https://youtu.be/aCHubyOOQ_Q





## Problem 3: Using both 7-Segment Decoder and 7-segment display

### Procedure

Now, use the decoder chip (**74LS47**). Connect it to the bread board.

Then, you need only 4 Digital out pins of MCU to display from 0 to 9.

![img](https://424033796-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-MgmrEstOHxu62gXxq1t%2Fuploads%2FOLBpZY2YOH4KNgHnb7du%2Fimage.png?alt=media&token=97cf1fb5-c747-40e4-b43a-4f743400bfa5)



### Connection Diagram

Circuit diagram

![image-20231003185104113](C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20231003185104113.png)



1. Work on the same project and code.

- i.e. : project “**LAB_GPIO_7segment”.** and source file named as “**LAB_GPIO_7segment.c”**

  ```c
  void sevensegment_display_init(void); 
  void sevensegment_display(uint8_t  num);
  ```
  
1. First, check if every number, 0 to 9, can be displayed properly
  
2. Then, create a code to display the number from 0 to 9 with each button press. After the number '9', it should start from '0' again.
     

### Configuration

| Digital In for Button (B1) | Digital Out for 7-Segment                     |
| -------------------------- | --------------------------------------------- |
| Digital In                 | Digital Out                                   |
| PC13                       | PA7, PB6, PC7, PA9                            |
| PULL-UP                    | Push-Pull, No Pull-up-Pull-down, Medium Speed |





### Results

Experiment images

<img src="C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20231003201739498.png" alt="image-20231003201739498" style="zoom:50%;" />ser-images\image-20231003201739498.png" alt="image-20231003201739498" style="zoom:50%;" />![image-20231003201818540](C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20231003201818540.png)<img src="C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20231003201739498.png" alt="image-20231003201739498" style="zoom:50%;" />ser-images\image-20231003201739498.png" alt="image-20231003201739498" style="zoom:50%;" />![image-20231003201818540](C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20231003201818540.png)

<img src="C:\Users\skrua\AppData\Roaming\Typora\typora-user-images\image-20231003201912140.png" alt="image-20231003201912140" style="zoom:50%;" />



It shows 0 to 9 using the 7 segment LED, and the number increases by 1 every time the button is pressed. This time, a decoder was used to express numbers with four inputs.



# III. Reference & Troubleshooting

### Reference

Young-Keun Kim (2023). https://ykkim.gitbook.io/ec/



### Troubleshootingg

There was a problem with the resistor array, so some of the LEDs turned on a low light and pressed the button to change the number, but the original number did not appear due to the weak light that first came in. At first, I thought it was a code problem, but I checked my code repeatedly and there was no problem. So I thought it was an external problem, I thought about it, but the reason why the light came in seemed to be the resistor array problem. Therefore, I changed the resistor array, the light came in normally and the number appeared correctly.

Again, the problem of bouncing using the button was the same as LAB2, and again, the problem was solved by dividing the button up, down, and delaying. Also, the for-statement was used to delay.





### Appendix

- **ecGPIO.h**

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
  
   
  #ifdef __cplusplus
  }
  #endif /* __cplusplus */
  
  #endif
  ```

  

- **ecGPIO.c**

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
  ```

  

- **ecRCC.h**

  ```c
  #ifndef __EC_RCC_H
  #define __EC_RCC_H
  
  #ifdef __cplusplus
   extern "C" {
  #endif /* __cplusplus */
  
  //#include "stm32f411xe.h"
  
  void RCC_HSI_init(void);
  void RCC_PLL_init(void);
  void RCC_GPIOA_enable(void);
  void RCC_GPIOB_enable(void);
  void RCC_GPIOC_enable(void);
  void RCC_GPIOD_enable(void);
  // void RCC_GPIO_enable(GPIO_TypeDef * GPIOx);
  
  extern int EC_SYSCL;
  
  #ifdef __cplusplus
  }
  #endif /* __cplusplus */
  
  #endif
  ```

  

- **ecRCC.c**

  ```c
  #include "stm32f4xx.h"
  #include "ecRCC.h"
  
  volatile int EC_SYSCLK=16000000;
  
  void RCC_HSI_init(void) {
  	// Enable High Speed Internal Clock (HSI = 16 MHz)
    //RCC->CR |= ((uint32_t)RCC_CR_HSION);
  	RCC->CR |= 0x00000001U;
  	
    // wait until HSI is ready
    //while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 ) {;}
  	while ( (RCC->CR & 0x00000002U) == 0 ) ;
  	
    // Select HSI as system clock source 
    RCC->CFGR &= (uint32_t)(~RCC_CFGR_SW); 								// not essential
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI; 								//00: HSI16 oscillator used as system clock
  
  	// Wait till HSI is used as system clock source
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != 0 );
  		   
  	//EC_SYSTEM_CLK=16000000;
  		//EC_SYSCLK=16000000;
  		EC_SYSCLK=16000000;
  }
  
  void RCC_PLL_init(void) {	
  	// To correctly read data from FLASH memory, the number of wait states (LATENCY)
    // must be correctly programmed according to the frequency of the CPU clock
    // (HCLK) and the supply voltage of the device.		
  	FLASH->ACR &= ~FLASH_ACR_LATENCY;
  	FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;
  		
  	// Enable the Internal High Speed oscillator (HSI)
  	RCC->CR |= RCC_CR_HSION;
  	while((RCC->CR & RCC_CR_HSIRDY) == 0);
  	
  	// Disable PLL for configuration
  	RCC->CR    &= ~RCC_CR_PLLON;
  	
  	// Select clock source to PLL
  	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC; 		// Set source for PLL: clear bits
  	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // Set source for PLL: 0 =HSI, 1 = HSE
  	
  	// Make PLL as 84 MHz
  	// f(VCO clock) = f(PLL clock input) * (PLLN / PLLM) = 16MHz * 84/8 = 168 MHz
  	// f(PLL_R) = f(VCO clock) / PLLP = 168MHz/2 = 84MHz
  	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN) | 84U << 6;
  	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM) | 8U ; 
  	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;  // 00: PLLP = 2, 01: PLLP = 4, 10: PLLP = 6, 11: PLLP = 8	
  	
  	
  	// Enable PLL after configuration
  	RCC->CR   |= RCC_CR_PLLON; 
  	while((RCC->CR & RCC_CR_PLLRDY)>>25 != 0);
  	
  	// Select PLL as system clock
  	RCC->CFGR &= ~RCC_CFGR_SW;
  	RCC->CFGR |= RCC_CFGR_SW_PLL;
  	
  	// Wait until System Clock has been selected
  	while ((RCC->CFGR & RCC_CFGR_SWS) != 8UL);
  	
  	// The maximum frequency of the AHB and APB2 is 100MHz,
  	// The maximum frequency of the APB1 is 50 MHz.
  	RCC->CFGR &= ~RCC_CFGR_HPRE;  		// AHB prescaler = 1; SYSCLK not divided (84MHz)
  	RCC->CFGR &= ~RCC_CFGR_PPRE1; 		// APB high-speed prescaler (APB1) = 2, HCLK divided by 2 (42MHz)
  	RCC->CFGR |=  RCC_CFGR_PPRE1_2;
  	RCC->CFGR &= ~RCC_CFGR_PPRE2; 		// APB high-speed prescaler (APB2) = 1, HCLK not divided	(84MHz)
  	
  	EC_SYSCLK=84000000;
  }
  
  
  void RCC_GPIOA_enable(void)
  {
  	// HSI is used as system clock         
  	RCC_HSI_init();
  	// RCC Peripheral Clock Enable Register 
  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  }
  
  void RCC_GPIOB_enable(void)
  {
  	// HSI is used as system clock         
  	RCC_HSI_init();
  	// RCC Peripheral Clock Enable Register 
  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  }
  
  void RCC_GPIOC_enable(void)
  {
  	// HSI is used as system clock         
  	RCC_HSI_init();
  	// RCC Peripheral Clock Enable Register 
  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  }
  
  void RCC_GPIOD_enable(void)
  {
  	// HSI is used as system clock         
  	RCC_HSI_init();
  	// RCC Peripheral Clock Enable Register 
  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  }
  ```

  