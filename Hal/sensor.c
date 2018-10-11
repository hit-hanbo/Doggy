#include "stm32f1xx_hal.h"
#include "sensor.h"
#include "gizwits_protocol.h"
#include "gizwits_product.h"
#include "stm32f1xx_hal_rcc.h"

#define RESOLUTION_FIRE 0.15          // resolution for fire sensor
#define RESOLUTION_GAS  0.15          // resolution for gas sensor

extern ADC_HandleTypeDef hadc1;

ADC_ValueTypeDef ADC_Value_Current;
ADC_ValueTypeDef ADC_Value_Compare;

uint16_t diff(uint16_t x, uint16_t y)
{
	if(x < y)
		return y - x;
	else
		return 0;
}

void ADC_Value_Handler(void)
{
	uint16_t Diff;
	uint16_t Temperature;
	uint16_t Fire_Flag, Gas_Flag;
	Fire_Flag = currentDataPoint.valueFire_1 + currentDataPoint.valueFire_2 +
			currentDataPoint.valueFire_3 + currentDataPoint.valueFire_4;
	Gas_Flag = currentDataPoint.valueGas;
												 // Read Current Status
	ADC_Value_Compare = ADC_Value_Current;       // Set Compare Value
	ADC_Sample();                                // ADC Sample

	Diff = diff(ADC_Value_Current.Fire_A, ADC_Value_Compare.Fire_A);
												 // Calculate Diff of 2 values
	if(Diff >  ADC_Value_Current.Fire_A >> 3)  // Alarm for Fire !
	{
		currentDataPoint.valueFire_1 = 1;        // Alarm
	}

	Diff = diff(ADC_Value_Current.Fire_B, ADC_Value_Compare.Fire_B);
												 // Calculate Diff of 2 values
	if(Diff > ADC_Value_Current.Fire_B >> 3)  // Alarm for Fire !
	{
		currentDataPoint.valueFire_2 = 1;        // Alarm
	}

	Diff = diff(ADC_Value_Current.Fire_C, ADC_Value_Compare.Fire_C);
												 // Calculate Diff of 2 values
	if(Diff >  ADC_Value_Current.Fire_C >> 3)  // Alarm for Fire !
	{
		currentDataPoint.valueFire_3 = 1;        // Alarm
	}
	Diff = diff(ADC_Value_Current.Fire_D, ADC_Value_Compare.Fire_D);
												 // Calculate Diff of 2 values
	if(Diff > ADC_Value_Current.Fire_D >> 3)  // Alarm for Fire !
	{
		currentDataPoint.valueFire_4 = 1;        // Alarm
	}

	Diff = diff(ADC_Value_Current.Gas_A, ADC_Value_Compare.Gas_A);
												 // Calculate Diff of 2 values
	if(Diff > ADC_Value_Current.Gas_A >> 3)  // Alarm for Fire !
	{
		currentDataPoint.valueGas = 1;        // Alarm
	}

	Temperature = ( 1775 - ADC_Value_Current.Temperature ) / 5 + 25;
	currentDataPoint.valueTemperature = Temperature;
}

void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  MX_GPIO_Init();
  MX_DMA_Init();
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel
    */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}


void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 |
		  	  	  	     GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
