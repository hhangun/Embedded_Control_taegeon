/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Han Taegeon
Modified         : 2023-11-06
Language/ver     : C++ in Keil uVision

/----------------------------------------------------------------*/


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
