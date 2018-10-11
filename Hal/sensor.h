#ifndef SENSOR_H_
#define SENSOR_H_

#include "stm32f1xx.h"
#include "stm32f1xx_hal_adc.h"

typedef struct{
	uint32_t Fire_A;
	uint32_t Fire_B;
	uint32_t Fire_C;
	uint32_t Fire_D;
	uint32_t Temperature;
	uint32_t Gas_A;
}ADC_ValueTypeDef;

void ADC_Init(void);
void ADC_Sample(void);
uint16_t ADC_Get_Val(void);
void ADC_Value_Handler(void);
void MX_ADC1_Init(void);

#endif
