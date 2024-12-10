#include "stm32f0xx.h"

#define BUFFER_SIZE 256

// Define the DAC channel for output
#define DAC_CHANNEL DAC_Channel_1

// Define DMA buffer for DAC signal
static uint32_t dma_buffer[BUFFER_SIZE];

void init(void);
void dac_init(void);
void dma_init(void);
void matrix_init(void);

void generate_sawtooth_signal(void);

void SPI2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);

// Function to initialize the system
void init(void){
	// Enable GPIOA and GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;

	// Initialize GPIO pins for various functions
	GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1 | GPIO_PUPDR_PUPDR5_1;
	GPIOC->MODER |= GPIO_MODER_MODER12_0;
	GPIOA->MODER |= GPIO_MODER_MODER15_0;

	GPIOC->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;

	// Update the system core clock
	SystemCoreClockUpdate();
}

// Function to initialize DAC for waveform generation
void dac_init(void){
	// Enable DAC clock
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	// Configure DAC channel
	DAC->CR |= DAC_CR_EN1; // Enable DAC Channel 1
	DAC->CR |= DAC_CR_TEN1; // Enable trigger for DAC

	// Set the DAC output to the desired voltage range
	DAC->DHR12R1 = 0; // Initialize DAC with zero output value
}

// Function to initialize DMA for transferring data to DAC
void dma_init(void){
	// Enable DMA1 clock
	RCC->AHBENR |= RCC_AHBENR_DMAEN;

	// Configure DMA settings
	DMA1_Channel3->CCR &= ~DMA_CCR_EN; // Disable DMA channel before configuration
	DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_CIRC; // Memory increment, transfer direction, circular mode
	DMA1_Channel3->CNDTR = BUFFER_SIZE; // Number of data items
	DMA1_Channel3->CPAR = (uint32_t)&DAC->DHR12R1; // DMA peripheral address (DAC Data Holding Register)
	DMA1_Channel3->CMAR = (uint32_t)dma_buffer; // DMA memory address (buffer)

	// Enable DMA channel
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

// Function to initialize SPI for controlling the 8x8 LED matrix
void matrix_init(void){
	// Enable GPIOA and GPIOB clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

	// Configure GPIO for SPI pins (PA8 for chip select, PB13 for clock, PB15 for data)
	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_0;

	GPIOB->AFR[1] |= (0 << 4 * (13-8)) | (0 << 4 * (15-8)); // Set alternate function for SPI pins
	GPIOB->MODER &= ~(GPIO_MODER_MODER13 | GPIO_MODER_MODER15);
	GPIOB->MODER |= (GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1); // Set SPI pins to alternate function

	// Enable SPI2 clock
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	// Configure SPI settings
	SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR | SPI_CR1_MSTR; // Slave select management, SPI baud rate, master mode
	SPI2->CR2 = SPI_CR2_DS | SPI_CR2_RXNEIE; // Data size and RXNE interrupt enable
	SPI2->CR1 |= SPI_CR1_SPE; // Enable SPI

	// Enable SPI interrupts
	NVIC_EnableIRQ(SPI2_IRQn);
}

// Function to generate a sawtooth waveform for DAC
void generate_sawtooth_signal(void){
	static uint32_t value = 0;

	// Update DAC output value to generate sawtooth signal
	DAC->DHR12R1 = value;
	value = (value + 1) % 4096; // Keep value in the 12-bit range (0 to 4095)
}

// Interrupt handler for SPI transmission (optional, if using interrupts)
void SPI2_IRQHandler(void){
	// Handle SPI interrupt (e.g., for completing data transmission)
}

// DMA interrupt handler (optional, if using interrupts)
void DMA1_Channel3_IRQHandler(void){
	// Handle DMA interrupt for DAC data transfer
	if(DMA1->ISR & DMA_ISR_TCIF3){ // Transfer complete interrupt
		DMA1->IFCR |= DMA_IFCR_CTCIF3; // Clear interrupt flag
		// Additional actions after DMA transfer complete (e.g., update the signal)
	}
}

// Main function to initialize and run the project
int main(void){
	// System initialization
	init();

	// Initialize DAC, DMA, and SPI for matrix
	dac_init();
	dma_init();
	matrix_init();

	// Enable DMA transfer and start signal generation
	DMA1_Channel3->CCR |= DMA_CCR_EN;

	// Main loop
	while(1){
		// Generate sawtooth waveform for DAC
		generate_sawtooth_signal();
		// SPI transmission for matrix control can go here (e.g., to update the LED matrix display)
	}
}
