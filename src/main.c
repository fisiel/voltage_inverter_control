/* ------------------- BIBLIOTEKI ------------------- */
#include "stm32f4xx.h"
#include <math.h>
/* -------------------------------------------------- */

/* ------------------- ZMIENNE ------------------- */
#define period 10000.0
#define U_nom  400.0
#define f_nom  50.0
#define f_imp  5000.0

volatile float f_max = 0.0;
volatile float f_zad = 0.0;
volatile float freq = 0.0;
volatile float k = 0.0;
volatile float U_DC = 0.0;
volatile float I_a = 0.0;
volatile float I_b = 0.0;
volatile float fi = 0.0;
volatile float t0 = 0.0;
volatile float t1 = 0.0;
volatile float t2 = 0.0;

volatile uint16_t it_state= 0;
volatile uint16_t sector = 0;
volatile uint16_t safety = 0;
/* ----------------------------------------------- */

/* ------------------- STRUKTURY INICJALIZACYJNE ------------------- */
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
TIM_OCInitTypeDef  TIM_OCTInitStructure;
TIM_BDTRInitTypeDef TIM_BDTR_StructInit;
NVIC_InitTypeDef NVIC_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
ADC_InitTypeDef ADC_InitStructure;
/* ----------------------------------------------------------------- */

/* ------------------- PROTOTYPY FUNKCJI ------------------- */
void TIM_InitPWM(void);
void GPIO_InitPins(void);
void ADC_InitModule(void);
/* --------------------------------------------------------- */

/* ------------------- PRZERWANIA ------------------- */
void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		if(it_state == 0)
		{
			f_max = f_nom * (U_DC / U_nom) * (sqrt(3.0) / 2.0);

			f_zad = k * f_max;

			if(f_zad > freq)
			{
				freq = freq + 0.0001;
			}
			else if(f_zad < freq)
			{
				freq = freq - 0.0001;
			}


			fi = fi + (2.0 * (M_PI / f_imp) * freq);

			if(fi > 2 * M_PI)
			{
				fi = 0;
			}

			sector = fi / (M_PI / 3.0);

			t1 = period * (freq / f_max) * (sqrt(3.0) / 2) * sinf(M_PI / 3.0  - fi + sector * M_PI  / 3.0) - 1;
			t2 = period * (freq / f_max) * (sqrt(3.0) / 2) * sinf(fi - sector * M_PI / 3.0) - 1;
			t0 = (period - t1 - t2) / 2.0 - 1;

			if(freq < 0.03 * f_max)
			{
				t1 = 0;
				t2 = 0;
				t0 = 0;
			}

			it_state = 1;
		}
		else
		{
			ADC_SoftwareStartInjectedConv(ADC1);
			k = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1) / 4095.0;
			U_DC = (ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2) / 100.0) - 2;
			I_a = (ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3) - 3400) / 2;
			I_b = (ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_4) - 3400) / 2;

			if( fabs(I_a) > 4000 || fabs(I_b) > 4000)
			{
				safety++;

				if(safety == 100)
				{
					t1 = 0;
					t2 = 0;
					t0 = 0;
				}
			}

			switch(sector)
			{
				case 0	:
					TIM1->CCR1 = t0 + t1 + t2;
					TIM1->CCR2 = t0 + t2;
					TIM1->CCR3 = t0;
					break;

				case 1	:
					TIM1->CCR1 = t0 + t1;
					TIM1->CCR2 = t0 + t1 + t2;
					TIM1->CCR3 = t0;
					break;

				case 2	:
					TIM1->CCR1 = t0;
					TIM1->CCR2 = t0 + t1 + t2;
					TIM1->CCR3 = t0 + t2;
					break;

				case 3	:
					TIM1->CCR1 = t0;
					TIM1->CCR2 = t0 + t1;
					TIM1->CCR3 = t0 + t1 + t2;
					break;

				case 4	:
					TIM1->CCR1 = t0 + t2;
					TIM1->CCR2 = t0;
					TIM1->CCR3 = t0 + t1 + t2;
					break;

				case 5	:
					TIM1->CCR1 = t0 + t1 + t2;
					TIM1->CCR2 = t0;
					TIM1->CCR3 = t0 + t1;
					break;

				default:
					break;
			}


			it_state = 0;
		}

		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
/* -------------------------------------------------- */

/* ------------------- FUNKCJA GLOWNA ------------------- */
int main(void)
{
	GPIO_InitPins();
	TIM_InitPWM();
	ADC_InitModule();

	for(;;)
	{

	}
}
/* ------------------------------------------------------ */

/* ------------------- DEFINICJE FUNKCJI ------------------- */
void GPIO_InitPins(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void TIM_InitPWM(void)
{
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInitStructure.TIM_Period = period - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_OCStructInit(&TIM_OCTInitStructure);
	TIM_OCTInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCTInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCTInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCTInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCTInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
	TIM_OCTInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCTInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCTInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM1, &TIM_OCTInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OCTInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM1, &TIM_OCTInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OCTInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(TIM1, &TIM_OCTInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_BDTRStructInit(&TIM_BDTR_StructInit);
	TIM_BDTR_StructInit.TIM_OSSRState = TIM_OSSRState_Disable;
	TIM_BDTR_StructInit.TIM_OSSIState = TIM_OSSIState_Disable;
	TIM_BDTR_StructInit.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTR_StructInit.TIM_Break = TIM_Break_Enable;
	TIM_BDTR_StructInit.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTR_StructInit.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
	TIM_BDTR_StructInit.TIM_DeadTime = 18;
	TIM_BDTRConfig(TIM1, &TIM_BDTR_StructInit);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
}

void ADC_InitModule(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_CommonStructInit(&ADC_CommonInitStructure);
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 0;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_InjectedSequencerLengthConfig(ADC1, 4);

	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Rising);
	ADC_ContinuousModeCmd(ADC1, DISABLE);
	ADC_InjectedDiscModeCmd(ADC1, DISABLE);
	ADC_AutoInjectedConvCmd(ADC1, DISABLE);

	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_480Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_480Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_480Cycles);

	ADC_Cmd(ADC1, ENABLE);

}
/* ----------------------------------------------------------*/
