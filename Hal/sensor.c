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

extern uint32_t ADC_Value_Buffer[6];

/*   @ Brief: Calculate Diff of two UINT32
 *   @ Param: uint32_t X , uint32_t Y
 *   @ Retval y - x : X <  Y
 *                0 : X >= Y
 */
uint32_t diff(uint32_t x, uint32_t y)
{
	if(x < y)
		return y - x;
	else
		return 0;
}

/*   @ Brief: Calculate ADC_Value & Update CurrentDataPoint
 *   @ Param: None
 *   @ Retval None
*/
void ADC_Value_Handler(void)
{
	uint32_t Diff;
	uint16_t Temperature;
	uint16_t Fire_Flag, Gas_Flag;
	Fire_Flag = currentDataPoint.valueFire_1 + currentDataPoint.valueFire_2 +
							currentDataPoint.valueFire_3 + currentDataPoint.valueFire_4;
	Gas_Flag = currentDataPoint.valueGas;
												 // Read Current Status
	ADC_Value_Compare = ADC_Value_Current;       // Set Compare Value

	ADC_Value_Current.Fire_A = ADC_Value_Buffer[0];
	ADC_Value_Current.Fire_B = ADC_Value_Buffer[1];
	ADC_Value_Current.Fire_C = ADC_Value_Buffer[2];
	ADC_Value_Current.Fire_D = ADC_Value_Buffer[3];
	ADC_Value_Current.Gas_A  = 0x0FFF - ADC_Value_Buffer[4];
	ADC_Value_Current.Temperature = ADC_Value_Buffer[5];

	Diff = diff(ADC_Value_Current.Fire_A, ADC_Value_Compare.Fire_A);
												 // Calculate Diff of 2 values
	if(Diff > (uint32_t) ADC_Value_Current.Fire_A * RESOLUTION_FIRE)  // Alarm for Fire !
	{
		currentDataPoint.valueFire_1 = 1;        // Alarm
	}

	Diff = diff(ADC_Value_Current.Fire_B, ADC_Value_Compare.Fire_B);
												 // Calculate Diff of 2 values
	if(Diff > (uint32_t) ADC_Value_Current.Fire_B * RESOLUTION_FIRE)  // Alarm for Fire !
	{
		currentDataPoint.valueFire_2 = 1;        // Alarm
	}

	Diff = diff(ADC_Value_Current.Fire_C, ADC_Value_Compare.Fire_C);
												 // Calculate Diff of 2 values
	if(Diff > (uint32_t) ADC_Value_Current.Fire_C * RESOLUTION_FIRE)  // Alarm for Fire !
	{
		currentDataPoint.valueFire_3 = 1;        // Alarm
	}
	Diff = diff(ADC_Value_Current.Fire_D, ADC_Value_Compare.Fire_D);
												 // Calculate Diff of 2 values
	if(Diff > (uint32_t) ADC_Value_Current.Fire_D * RESOLUTION_FIRE)  // Alarm for Fire !
	{
		currentDataPoint.valueFire_4 = 1;        // Alarm
	}

	Diff = diff(ADC_Value_Current.Gas_A, ADC_Value_Compare.Gas_A);
												 // Calculate Diff of 2 values
	if(Diff > (uint32_t) ADC_Value_Current.Gas_A >> 3)  // Alarm for Fire !
	{
		currentDataPoint.valueGas = 1;        // Alarm
	}

	Temperature = (( ADC_Value_Buffer[5] * 165 ) >> 9) - 535;
//	printf("\r\n\r\n%d\r\n\r\n", Temperature);
	if(Temperature > 30 || Temperature < 1)
		currentDataPoint.valueTemperature = 23;
	else
		currentDataPoint.valueTemperature = Temperature / 6;
}

DMA_HandleTypeDef hdma_adc1;

void MX_ADC1_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  ADC_ChannelConfTypeDef sConfig;
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
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void MX_DMA_Init(void) 
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(&hadc1,DMA_Handle,hdma_adc1);
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/*    @ Brief: BSP_LED_Init
 *    @ Param: None
 *    @ Retval None
 */
void LED_Init(void)
{
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
		RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
