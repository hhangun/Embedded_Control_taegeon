/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Lim Soon Ho, Han Taegeon
Modified         : 2023-12-15
Language/ver     : C++ in Arduino
Description      : Control RC car
/----------------------------------------------------------------*/

# include "ecSTM32F411.h"

void setup(void);

char btData[1] = {0};
int i = 0;
uint8_t xData    = 0;
uint8_t zData    = 0;

// DC motor pin
#define Motor_A PA_0
#define Motor_B PA_1

// Servo motor pin
#define SERVO_PIN PB_6

float Duty_A = 0.0;
float Duty_B = 0.0;
int dir = 0;
int time = 0;
float count = 19.0;

int main(void){

   setup();
   
   PWM_duty(SERVO_PIN, (0.5 + (1.f/9.f)*(float)count)/20.f);
   PWM_duty(Motor_A, Duty_A);
   PWM_duty(Motor_B, Duty_B);
   GPIO_write(GPIOC, 10, dir);
   GPIO_write(GPIOC, 12, dir);
   
   while(1){

   }
}

void setup(void){
   RCC_PLL_init();
   SysTick_init();
   
   
    PWM_init(Motor_A);
    PWM_init(Motor_B);
    PWM_period_us(Motor_A, 200);
    PWM_period_us(Motor_B, 200);
   
   
   // dir pin setup (PC_10: motor A, PC_12: motor B)
    GPIO_init(GPIOC, 10, OUTPUT);
    GPIO_init(GPIOC, 12, OUTPUT);
    GPIO_pupd(GPIOC, 10, EC_NONE);
    GPIO_pupd(GPIOC, 12, EC_NONE);
    GPIO_otype(GPIOC, 10, EC_PUSH_PULL);
    GPIO_otype(GPIOC, 12, EC_PUSH_PULL);
   
   // servo motor setup
    PWM_init(SERVO_PIN);   
    PWM_period_ms(SERVO_PIN, 10);  
   
   UART1_init();
   UART1_baud(9600);
   
   UART2_init();
   UART2_baud(9600);
   
   GPIO_init(GPIOA, LED_PIN, OUTPUT);
   GPIO_pupd(GPIOA, LED_PIN, EC_PU);
   
}



void USART1_IRQHandler(){
      if(is_USART_RXNE(USART1)){
            time++;
         if(time > 1000){
                     
            PWM_duty(Motor_A, Duty_A);
            PWM_duty(Motor_B, Duty_B);
            GPIO_write(GPIOC, 10, dir);
            GPIO_write(GPIOC, 12, dir);
               
            PWM_duty(SERVO_PIN, (0.5 + (1.f/9.f)*(float)count)/20.f);
            
            for( i = 0; i < 2; i++){
            btData[i]  = USART1_read();
            }
            time = 0;
            
         printf("servo: %c\r\n\n",btData[0]);
         printf("DC motor: %c\r\n",btData[1]);
         
         switch(btData[0]){   
         case 'b':
                  count = count + 0.4;
                if (count > 26)
                   count = 26;               // maximum value: 44
             printf("count(b): %1.f\r\n", count);
            break;
                  
         case 'a':
             count = count - 0.4;
               if(count < 12)
                  count = 12;            // minimum value: 8
              printf("count(a): %1.f\r\n", count);
            break;
            
         case 's' || 'S':
                count = 19;
            break;
      }
         
         
         

      switch(btData[1]){
         
            case 'A':   //backward
           dir = 0; 
           Duty_A = 0.5;
           Duty_B = 0.5;
                printf("backward\r\n");
          break;
            case 'B':      //Backward Speed Up
           dir = 0;   
           Duty_A = 0.9;
           Duty_B = 0.9;
                printf("BACK\r\n");
          break;
        case 'C':   //forward
           dir = 1;
           Duty_A = 0.5;
           Duty_B = 0.5;
            printf("forward\r\n");
          break;
        case 'D':   // Forward Speed Up
           dir = 1;
           Duty_A = 0.1;
           Duty_B = 0.1;
            printf("FORWARD\r\n");
          break;
            case 'S':
               dir = 0;
              Duty_A = 0.0;
          Duty_B = 0.0;
            break;
            
   }
	} 
 }
}
