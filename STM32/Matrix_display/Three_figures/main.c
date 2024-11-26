#include "button.h"
#include "driver.h"
#include "control.h"
#include "stm32f0xx.h"

void init(void);
void matrix_init(void);
uint8_t systick_flag_check(void);
void SysTick_Handler(void);
uint8_t button_0_read(void); 
uint8_t button_1_read(void); 
uint8_t button_2_read(void);
uint8_t button_3_read(void);
uint8_t button_4_read(void);

void init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	
	GPIOA->MODER &= ~(GPIO_MODER_MODER0|GPIO_MODER_MODER4|GPIO_MODER_MODER5);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4|GPIO_PUPDR_PUPDR5);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1|GPIO_PUPDR_PUPDR5_1;
	GPIOC->MODER |= GPIO_MODER_MODER12_0;
	GPIOA->MODER |= GPIO_MODER_MODER15_0;
	
	GPIOC->MODER &= ~(GPIO_MODER_MODER6|GPIO_MODER_MODER7|GPIO_MODER_MODER8|GPIO_MODER_MODER9);
	GPIOC->MODER |= GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0;
	
	SystemCoreClockUpdate();
	SysTick->LOAD = SystemCoreClock / 1000 - 1; // 1 ms
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
}

//new
//PB15__SPI2_MOSI
//PB13__SPI2_SCK
//PA8__GPO_LE
void matrix_init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN;
	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	
	//GPIOB->AFR[0] |= Fn << 4 * (Pn); Pn<8
	//GPIOB->AFR[1] |= Fn << 4 * (Pn-8); Pn>7
	
	GPIOB->AFR[1] |= (0 << 4 * (13-8))|(0 << 4 * (15-8));
	GPIOB->MODER &= ~(GPIO_MODER_MODER13 | GPIO_MODER_MODER15);
	GPIOB->MODER |= (GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1);

	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR | SPI_CR1_MSTR;
	SPI2->CR2 = SPI_CR2_DS;
	SPI2->CR1 |= SPI_CR1_SPE;
}
//new end

uint8_t systick_flag_check(void){
	return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ? 1 : 0;
}

//volatile uint8_t a = 0;
static ConfButton buttons[5];
static ScreenScene screenScene;

uint8_t button_0_read(void){
	return (GPIO_IDR_0 & GPIOA->IDR) ? 1 : 0;
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

void SysTick_Handler(void){
	Handler5Button(buttons);
	ScreenDraw(&screenScene);
}

int main(void){
	init();
	matrix_init();
	buttons[0] = InitButton(button_0_read);
	buttons[1] = InitButton(button_1_read);
	buttons[2] = InitButton(button_2_read);
	buttons[3] = InitButton(button_3_read);
	buttons[4] = InitButton(button_4_read);
	screenScene = InitScene();
	ScreenObject screenObject[3] =
	{
		{
			{0x0F, 0x09, 0x09, 0x0F, 0x00, 0x00, 0x00, 0x00}, 0, 0, // square
		},
		{
			{0x09, 0x06, 0x06, 0x09, 0x00, 0x00, 0x00, 0x00}, 0, 0, // cross
		},
		{
			{0x06, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00, 0x00}, 0, 0, // circle
		}
	};
	uint8_t currMovedObj = 0;
	while(1){
		ObjectMove(buttons, screenObject, 5, 3, &currMovedObj);
		ScreenSet(&screenScene, screenObject, 3);
	}
}
