#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "bdshot.h"
#include "setup.h"

static void setup_BDshot(); // Bidirectional DShot
static void setup_DMA();

void setup()
{
	// BDshot specific setup:
	setup_BDshot();
	setup_DMA();
}

static void setup_BDshot()
{
	// TIM1 - only for generating time basement all outputs are set by GPIOs:
	// enable TIM1 clock:
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	// register is buffered and overflow DMA request:
	TIM1->CR1 = 0x0;
	TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	// DMA request:
	TIM1->DIER |= TIM_DIER_CC1DE; // channel 1 request

	// TIM1 is 100 [MHz]:
	TIM1->PSC = 0;
	TIM1->ARR = DSHOT_TX_SECTION_TICKS - 1;
	TIM1->CCR1 = DSHOT_TX_SECTION_TICKS / 2;

	// TIM1 enable:
	TIM1->EGR |= TIM_EGR_UG;
	TIM1->CR1 |= TIM_CR1_CEN;
}

static void setup_DMA()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// bidirectional DSHOT:
	// for TIM1
	DMA2_Stream3->CR = 0x0;
	while (DMA2_Stream3->CR & DMA_SxCR_EN)
	{
		; // wait
	}
	DMA2_Stream3->CR |= (6U << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL;
	// all the other parameters will be set afterward
}
