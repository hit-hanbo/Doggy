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

void ADC_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->CFGR |= (1 << 15);
	RCC->CFGR &= ~(1 << 14);
	GPIOA->CRL  &= 0xf00ff000;   // PA0 1 4 5 6 Analog IN
	
	ADC1->CR1 &= (0xf << 16);    // ADC Independent
	ADC1->CR1 |= (1 << 8);       // ADC Scan Mode
	ADC1->CR2 |= (1 << 23);      // ENABLE Temperature Sensor
	ADC1->CR2 &= ~(1 << 11);     // ADC Right Align
	ADC1->CR2 |= (0x7 << 17);    // Software Trigger Convert
	ADC1->CR2 |= (1 << 20);      // ENABLE Trigger
	
	ADC1->SQR1 |= 0x00500000;    // 6 Channels
	ADC1->SQR3  = 0x00629020;    
	ADC1->SMPR2 = 0x001ff03f;
	
	ADC1->CR2 |= (1 << 0);
	ADC1->CR2 |= (1 << 2);         // A/D Cablication 
	while(ADC1->CR2 & 0x00000002U);// Wait
	
}

uint16_t ADC_Get_Val(void)
{
	ADC1->CR2 |= (1 << 22);
	// while((ADC1->SR & 0x00000002U));
	HAL_Delay(1);
	return ADC1->DR;
}

void ADC_Sample(void)
{
	HAL_ADC_Start(&hadc1);                       // Start AD Convert
	HAL_ADC_PollForConversion(&hadc1, 0xff);     // Poll For Conversion
	HAL_Delay(1);
	// while(!(ADC1->SR & ADC_SR_EOC));
	ADC_Value_Current.Fire_A = HAL_ADC_GetValue(&hadc1); // Get ADC_CHANNEL_0

	HAL_ADC_Start(&hadc1);                       // Start AD Convert
	HAL_ADC_PollForConversion(&hadc1, 0xff);     // Poll For Conversion
	HAL_Delay(1);
	// while(!(ADC1->SR & ADC_SR_EOC));
	ADC_Value_Current.Fire_B = HAL_ADC_GetValue(&hadc1); // Get ADC_CHANNEL_1

	HAL_ADC_Start(&hadc1);                       // Start AD Convert
	HAL_ADC_PollForConversion(&hadc1, 0xff);     // Poll For Conversion
	HAL_Delay(1);
	// while(!(ADC1->SR & ADC_SR_EOC));
	ADC_Value_Current.Gas_A = HAL_ADC_GetValue(&hadc1); // Get ADC_CHANNEL_4

	HAL_ADC_Start(&hadc1);                       // Start AD Convert
	HAL_ADC_PollForConversion(&hadc1, 0xff);     // Poll For Conversion
	HAL_Delay(1);
	// while(!(ADC1->SR & ADC_SR_EOC));
	ADC_Value_Current.Fire_C = HAL_ADC_GetValue(&hadc1); // Get ADC_CHANNEL_5

	HAL_ADC_Start(&hadc1);                       // Start AD Convert
	HAL_ADC_PollForConversion(&hadc1, 0xff);     // Poll For Conversion
	HAL_Delay(1);
	// while(!(ADC1->SR & ADC_SR_EOC));
	ADC_Value_Current.Fire_D = HAL_ADC_GetValue(&hadc1); // Get ADC_CHANNEL_6

	HAL_ADC_Start(&hadc1);                       // Start AD Convert
	HAL_ADC_PollForConversion(&hadc1, 0xff);     // Poll For Conversion
	HAL_Delay(1);
	// while(!(ADC1->SR & ADC_SR_EOC));
	ADC_Value_Current.Temperature = HAL_ADC_GetValue(&hadc1); // Get ADC_CHANNEL_T

	HAL_ADC_Stop(&hadc1);                        // Stop AD Convert
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
	GPIO_InitTypeDef GPIO_InitStruct;
    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfDiscConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}
