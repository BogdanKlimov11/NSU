#include "button.h"
#include "driver.h"
#include "control.h"
#include "stm32f0xx.h"
#include "multim.h"

#define buffersize 128

void init(void);
void matrix_init(void);
void adc_init(void);
void dma_init(void);

uint8_t systick_flag_check(void);

uint8_t button_0_read(void); 
uint8_t button_1_read(void); 
uint8_t button_2_read(void);
uint8_t button_3_read(void);
uint8_t button_4_read(void);

void SysTick_Handler(void);
void SPI2_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);

static ConfButton buttons[5];
static ScreenScene screenScene;
static uint8_t BufferADC[buffersize];
static Multim multim;

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
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	
	//GPIOB->AFR[0] |= Fn << 4 * (Pn); Pn<8
	//GPIOB->AFR[1] |= Fn << 4 * (Pn-8); Pn>7
	
	GPIOB->AFR[1] |= (0 << 4 * (13-8))|(0 << 4 * (15-8));
	GPIOB->MODER &= ~(GPIO_MODER_MODER13 | GPIO_MODER_MODER15);
	GPIOB->MODER |= (GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1);

	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR | SPI_CR1_MSTR;
	SPI2->CR2 = SPI_CR2_DS | SPI_CR2_RXNEIE;
	//SPI2->CR2 = SPI_CR2_DS;
	SPI2->CR1 |= SPI_CR1_SPE;

	NVIC_EnableIRQ(SPI2_IRQn);
	SPI2->DR = 0;
}
//new end

void adc_init(void){
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER1; //PA1 - ADC IN 1

	if((ADC1->CR & ADC_CR_ADEN) != 0){ /* (1) */
  ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while((ADC1->CR & ADC_CR_ADEN) != 0){
		/* For robust implementation, add here time-out management */
	}
	//ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL; /* (4) */
	while((ADC1->CR & ADC_CR_ADCAL) != 0){ /* (5) */
		/* For robust implementation, add here time-out management */
	}

	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRDY));
	ADC1->CFGR1 = ADC_CFGR1_RES_1 | ADC_CFGR1_OVRMOD | ADC_CFGR1_CONT;
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
	ADC1->CFGR1 |= ADC_CFGR1_DMACFG;
	ADC1->CHSELR |= ADC_CHSELR_CHSEL1;

	ADC1->CR |= ADC_CR_ADSTART;
}

void dma_init(void){
	RCC->AHBENR |= RCC_AHBENR_DMAEN;
	DMA1_Channel1->CCR = DMA_CCR_CIRC; 
	DMA1_Channel1->CCR |= DMA_CCR_TCIE;
	DMA1_Channel1->CCR |= DMA_CCR_HTIE;
	DMA1_Channel1->CCR |= DMA_CCR_MINC;
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;
	DMA1_Channel1->CNDTR = buffersize;
	DMA1_Channel1->CPAR = (uint32_t)(&ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)BufferADC;
	
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	
	NVIC_EnableIRQ(DMA1_Channel1_IRQn); 
}

uint8_t systick_flag_check(void){
  return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ? 1 : 0;
}

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
	CheckCalc(&multim);
}

void SPI2_IRQHandler(void){
	if(SPI2->SR & SPI_SR_TXE){
		//pass
	}
	if(SPI2->SR & SPI_SR_RXNE){
		uint16_t tmp = (uint16_t)SPI2->DR;
		tmp = 0;
		ScreenDraw(&screenScene);
	}
}

void DMA1_Channel1_IRQHandler(void){
	if(DMA1->ISR & DMA_ISR_HTIF1){
		//aver = 200;
		multim.currCalc = 0;
		DMA1->IFCR = DMA_IFCR_CHTIF1;
	}
	if(DMA1->ISR & DMA_ISR_TCIF1){
		//aver = 100;
		multim.currCalc = 1;
		DMA1->IFCR = DMA_IFCR_CTCIF1;
	}
}

int main(void){
	init();
	matrix_init();
	adc_init();
	dma_init();
	buttons[0] = InitButton(button_0_read);
	buttons[1] = InitButton(button_1_read);
	buttons[2] = InitButton(button_2_read);
	buttons[3] = InitButton(button_3_read);
	buttons[4] = InitButton(button_4_read);
	screenScene = InitScene();
	multim = InitMultim(128, 10);
	ScreenObject screenObject[1] = 
	{{{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},0,0}};
  while(1){
		ChangeCountInterruptsWait(&multim, buttons);
		CalcAver(&multim, BufferADC, screenObject[0].templ);
		ScreenSet(&screenScene, screenObject, 1);
  }
}
