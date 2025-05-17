#include "stm32f0xx.h"
#include "buttons.h"
#include "matrix.h"
#include "cross.h"

void init(void);

uint8_t systick_flag_check(void);

void CaseLedInvert(uint8_t state, int8_t num);

uint8_t button_1_read(void); 
uint8_t button_2_read(void);
uint8_t button_3_read(void);
uint8_t button_4_read(void);

void SysTick_Handler(void);

void paint(uint8_t start_x, uint8_t start_y, uint8_t shift);
//void move(void);

void init(void){
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
  
  GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5);
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1 | GPIO_PUPDR_PUPDR5_1;
  GPIOC->MODER |= GPIO_MODER_MODER12_0;
  GPIOA->MODER |= GPIO_MODER_MODER15_0;
  
  GPIOC->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
  GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
  
  SystemCoreClockUpdate();
  SysTick->LOAD = SystemCoreClock / 1000 - 1; // 1 ms
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
}


uint8_t systick_flag_check(void){
  return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ? 1 : 0;
}

static ConfButton button1;
static ConfButton button2;
static ConfButton button3;
static ConfButton button4;

void SysTick_Handler(void){
	GPIOA->ODR |= GPIO_ODR_15;
	HandlerButton(&button1);
	HandlerButton(&button2);
	GPIOA->ODR &= ~GPIO_ODR_15;
	__NOP();
	__NOP();
	__NOP();
	GPIOC->ODR |= GPIO_ODR_12;
	HandlerButton(&button3);
	HandlerButton(&button4);
	GPIOC->ODR &= ~GPIO_ODR_12;
			
	CaseLedInvert(PushButtListnr(&button1), 1);
	CaseLedInvert(PushButtListnr(&button2), 3);
	CaseLedInvert(PushButtListnr(&button3), 2);
	CaseLedInvert(PushButtListnr(&button4), 4);
}

static uint8_t start_x = 1;
static uint8_t start_y = 7;
static uint8_t shift = 5;

void CaseLedInvert(uint8_t state, int8_t num){
  if (state){
    switch (num){
      case 1:
				if (start_x == 1){
					break;
				}
				start_x = start_x - 1;
        GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_6) == GPIO_ODR_6) ? GPIO_BSRR_BR_6 : GPIO_BSRR_BS_6;
        break;
      case 2:
				if (start_y < 128){
					start_y = (uint8_t)(start_y << 1);
					shift = (uint8_t)(shift << 1);
				}
        GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_9) == GPIO_ODR_9) ? GPIO_BSRR_BR_9 : GPIO_BSRR_BS_9;
        break;
      case 3:
				if (start_x == 6){
					break;
				}
				start_x = start_x + 1;
        GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_7) == GPIO_ODR_7) ? GPIO_BSRR_BR_7 : GPIO_BSRR_BS_7;
        break;
      case 4:
				if (start_y >= 8){
					start_y = (start_y >> 1);
					shift= (shift >> 1);
				}
        GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_8) == GPIO_ODR_8) ? GPIO_BSRR_BR_8 : GPIO_BSRR_BS_8;
        break;
      default:
        break;
    }
  }
}

uint8_t button_1_read(void){
  return (GPIO_IDR_4 & GPIOA->IDR) ? 1 : 0;
}

uint8_t button_2_read(void){
  return (GPIO_IDR_5 & GPIOA->IDR) ? 1 : 0;
}

uint8_t button_3_read(void){
  return (GPIO_IDR_4 & GPIOA->IDR) ? 1 : 0;
}

uint8_t button_4_read(void){
  return (GPIO_IDR_5 & GPIOA->IDR) ? 1 : 0;
}

int main(void){
	button1 = InitButton(button_1_read);
	button2 = InitButton(button_2_read);
	button3 = InitButton(button_3_read);
	button4 = InitButton(button_4_read);
	uint8_t sysTick = 0;
	init();
	matrix_init();
	while(1){
		sysTick = systick_flag_check();
		paint(start_x, start_y, shift);
	}
}
